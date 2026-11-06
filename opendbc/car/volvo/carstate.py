import copy
from opendbc.can import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.volvo.values import CarControllerParams, DBC, CANBUS, EcmAccState
from opendbc.car import Bus, create_button_events, structs

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


def all_frames(cp: CANParser, name: str) -> list[dict[str, float]]:
  sigs = cp.vl_all[name]
  return [dict(zip(sigs.keys(), vals, strict=True)) for vals in zip(*sigs.values(), strict=True)]


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.cruiseState_enabled_prev = False
    self.eps_torque_timer = 0
    self.frame = 0
    self._cruise_speed_prev_kph = 0
    self._pending_delta = 0
    self._icbm_suppress_frames = 0
    self._custom_acc_step = 1  # set by carcontroller each frame; default 1 km/h
    self.driver_resume = False  # driver's Resume held (bit set, inverted copy cleared)
    # Set by the carcontroller: OP's stop to 0 owns the ACC standstill
    self.full_stop_active = False
    self.full_stop_standstill = False
    self.lka_state_prev: bool | None = None
    self.downhill = False  # set by the carcontroller
    self.ecm_standstill_ready = False

  def update(self, can_parsers) -> structs.CarState:
    pt_cp = can_parsers[Bus.pt]
    cam_cp = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # car speed
    ret.vEgoRaw = pt_cp.vl["VehicleSpeed1"]["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    # gas pedal
    ret.gasPressed = pt_cp.vl["AccPedal"]["AccPedal"] >= 10

    # brake pedal
    ret.brakePressed = pt_cp.vl["Brake_Info"]["BrakePedal"] == 2

    # steering
    ret.steeringAngleDeg = pt_cp.vl["PSCM1"]["SteeringAngleServo"]
    ret.steeringRateDeg = pt_cp.vl["SAS0"]["SteeringRateOfChange"]
    self.steeringDirection = pt_cp.vl["SAS0"]["SteeringDirection"]
    ret.steeringTorque = pt_cp.vl["PSCM1"]["EPSTorque"]
    if self.steeringDirection:
      ret.steeringTorque = -abs(ret.steeringTorque)
    ret.steeringTorqueEps = pt_cp.vl["PSCM1"]["LKATorque"]
    ret.steeringPressed = abs(ret.steeringTorque) > 50

    # cruise state
    set_speed = pt_cp.vl["ACC_Speed"]["ACC_Speed"] * CV.KPH_TO_MS
    # Downhill (from the carcontroller): the planner targets a bit over the set, so OP holds it without fighting it
    ret.cruiseState.speed = set_speed + (CarControllerParams.DOWNHILL_SET_OFFSET if self.downhill else 0.)
    # The driver's set, as the car shows it (ICBM compares its target to it)
    ret.cruiseState.speedCluster = set_speed
    ret.cruiseState.available = bool(cam_cp.vl["FSM0"]["ACC_Available"])
    # the ECM is the ACC master
    self.ecm_acc_engaged = int(pt_cp.vl["EngineData"]["ACC_State_ECM"]) == EcmAccState.ENGAGED
    ret.cruiseState.enabled = self.ecm_acc_engaged
    # Camera standstill hold, gated by the car's standstill; OP's own in its stop to 0
    if self.full_stop_active:
      ret.cruiseState.standstill = self.full_stop_standstill and ret.standstill
    else:
      ret.cruiseState.standstill = bool(cam_cp.vl["FSM3"]["ACC_Standstill"]) and ret.standstill
    # The car holds the stop (BrakeReqAck b0 bit7, route 807)
    self.car_hold = bool(pt_cp.vl["BrakeReqAck"]["StandstillHold"])
    # The ECM takes a resume only once 0x1FF b0 goes 70 -> D0, ~1.5 s after ACC_Standstill (routes 899, 89b)
    self.ecm_standstill_ready = int(pt_cp.vl["BrakeHold"]["Byte_0"]) == 0xD0
    ret.cruiseState.nonAdaptive = False
    ret.accFaulted = False
    self.acc_distance = cam_cp.vl["FSM1"]["ACC_Distance"]

    # Check if servo stops responding when ACC is active
    if ret.cruiseState.enabled and ret.vEgo > CarControllerParams.MIN_STEER_SPEED:
      if not self.cruiseState_enabled_prev:
        self.eps_torque_timer = 0

      if ret.steeringTorqueEps == 0:
        self.eps_torque_timer += 1
      else:
        self.eps_torque_timer = 0

      ret.steerFaultTemporary = self.eps_torque_timer >= CarControllerParams.STEER_TIMEOUT
    else:
      ret.steerFaultTemporary = False

    self.cruiseState_enabled_prev = ret.cruiseState.enabled

    # Gear lever position (route 8a6)
    ret.gearShifter = {
      0: GearShifter.park,
      1: GearShifter.reverse,
      2: GearShifter.neutral,
      3: GearShifter.drive,
    }[int(pt_cp.vl["GearSelector"]["GearLever"])]

    # safety
    ret.stockFcw = False
    ret.stockAeb = False

    # button presses
    ret.leftBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 1
    ret.rightBlinker = pt_cp.vl["MiscCarInfo"]["TurnSignal"] == 3

    # Cruise button events from set speed changes, paused after ICBM presses
    cruise_kph_now = round(set_speed * CV.MS_TO_KPH)  # the driver's set, not the downhill offset
    # Stopped, Set+/- only sets the speed: no accel/decel press, or the planner reads it as "go" (route 8b2, 572 s/591 s)
    if ret.standstill:
      self._pending_delta = 0
    elif self._icbm_suppress_frames > 0:
      self._cruise_speed_prev_kph = cruise_kph_now
      self._icbm_suppress_frames -= 1
    elif ret.cruiseState.enabled and self.cruiseState_enabled_prev:
      self._pending_delta += cruise_kph_now - self._cruise_speed_prev_kph
    self._cruise_speed_prev_kph = cruise_kph_now

    # One event per ACC step, on delta >= 1
    step = max(1, self._custom_acc_step)
    button_events = []
    if self._pending_delta >= 1:
      button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.accelCruise))
      self._pending_delta = max(0, self._pending_delta - step)
    elif self._pending_delta <= -1:
      button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.decelCruise))
      self._pending_delta = min(0, self._pending_delta + step)

    driver_resume = bool(pt_cp.vl["CCButtons"]["ACCResumeBtn"]) and not pt_cp.vl["CCButtons"]["ACCResumeBtnInv"]
    button_events.extend(create_button_events(driver_resume, self.driver_resume, {True: ButtonType.resumeCruise}))
    self.driver_resume = driver_resume
    # LKA button: no own bit, each press flips LKA_State (route 8a8); MADS takes it as a press and release
    lka_state = bool(pt_cp.vl["CCButtons"]["LKA_State"])
    if self.lka_state_prev is not None and lka_state != self.lka_state_prev:
      button_events.extend(create_button_events(True, False, {True: ButtonType.lkas}) +
                           create_button_events(False, True, {True: ButtonType.lkas}))
    self.lka_state_prev = lka_state
    ret.buttonEvents = button_events

    # lock info
    ret.doorOpen = not all([pt_cp.vl["Doors"]["DriverDoorClosed"], pt_cp.vl["Doors"]["PassengerDoorClosed"]])
    ret.seatbeltUnlatched = False

    self.pscm_stock_values = pt_cp.vl["PSCM1"]
    self.stock_FSM1 = copy.copy(cam_cp.vl["FSM1"])
    self.stock_FSM3 = copy.copy(cam_cp.vl["FSM3"])
    self.cam_fsm3_mode = int(self.stock_FSM3["ACC_Mode"])  # camera's FSM3 mode (LongMode)
    self.ACC_Check = cam_cp.vl["FSM3"]["ACC_Check"]
    # All frames this cycle of the relayed messages
    self.stock_FSM0_frames = all_frames(cam_cp, "FSM0")
    self.stock_FSM4_frames = all_frames(cam_cp, "FSM4")
    self.stock_BrakeLevel_frames = all_frames(pt_cp, "BrakeLevel")
    self.stock_BrakeStatus_frames = all_frames(pt_cp, "BrakeStatus")
    self.stock_wheelspeed1_frames = all_frames(pt_cp, "wheelspeed1")
    self.stock_BrakeReqAck_frames = all_frames(pt_cp, "BrakeReqAck")
    self.stock_BrakeHold_frames = all_frames(pt_cp, "BrakeHold")
    self.stock_EngineData_frames = all_frames(pt_cp, "EngineData")
    self.stock_AccHold_frames = all_frames(pt_cp, "AccHold")
    self.stock_CCButtons_frames = all_frames(pt_cp, "CCButtons")

    # TSR speed limit from camera (0 = no sign)
    tsr_raw = cam_cp.vl["FSM5"]["TSR_Speed"]
    ret_sp.speedLimit = tsr_raw * CV.KPH_TO_MS if tsr_raw > 0 else 0.0

    self.frame += 1
    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      ("VehicleSpeed1", 50),
      ("AccPedal", 100),
      ("BrakePedal", 50),
      ("Brake_Info", 50),
      ("PSCM1", 50),
      ("ACC_Speed", 50),
      ("MiscCarInfo", 25),
      ("Doors", 20),
      ("SAS0", 100),
      ("EngineData", 66),
      ("CCButtons", 66),
      ("GearSelector", 66),
      ("BrakeLevel", 50),
      ("BrakeStatus", 25),
      ("wheelspeed1", 50),
      ("BrakeReqAck", 50),
      ("BrakeHold", 50),
      ("AccHold", 25),
    ]

    cam_messages = [
      ("FSM0", 100),
      ("FSM1", 50),
      ("FSM3", 50),
      ("FSM4", 33),
      ("FSM5", 10),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CANBUS.pt),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CANBUS.cam),
    }
