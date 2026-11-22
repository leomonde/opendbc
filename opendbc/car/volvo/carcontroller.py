"""Volvo V60 controller; with OP's long on, OP owns the ECM and the camera's ACC is shown standby."""
import math

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.volvo.volvocan import (create_button_msg, create_lka_msg, create_lkas_state_msg, create_longitudinal,
                                        create_fsm0, create_fsm4, create_radar, create_cam_wheel_status,
                                        create_cam_brake_request_ack, create_cam_brake_hold, create_cam_engine_data,
                                        create_resume_button, create_cam_acc_hold, create_cam_button, fsm4_brake_value,
                                        next_long_mode)
from opendbc.car.volvo.values import CarControllerParams, LongMode, SteerDirection
from opendbc.sunnypilot.car.volvo.icbm import IntelligentCruiseButtonManagementInterface

try:
  from openpilot.common.params import Params
except (ImportError, OSError):  # not built: no custom ACC step
  Params = None


def cam_clean(CS) -> bool:
  """Camera engaged with no cancel nor abort: its frames can be the ECM's templates."""
  fsm3 = CS.stock_FSM3
  if fsm3["ACC_NoTargetCancel"] or int(fsm3["Byte_6"]):
    return False
  if any(f["ACC_AbortCode"] or not f["ACC_Enabled"] for f in CS.stock_FSM0_frames):
    return False
  return not any(int(f["Byte_2"]) == 0xFC for f in CS.stock_FSM4_frames)


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CCP = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.frame = 0
    # Custom ACC step, refreshed every 100 frames
    self._params = Params() if Params is not None else None
    self._custom_acc_step = self._read_acc_step()
    self.apply_steer_prev = 0
    self.apply_steer_dir_prev = SteerDirection.NONE
    self.latActive_prev = False
    self.steer_blocked = False
    self.steer_blocked_cnt = 0
    self.steer_dir_bf_block = SteerDirection.NONE
    # Resume (SNG)
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    self.op_resume_frames = 0
    self.sng_ack_frames = 0
    self.resume_hold_frames = 0  # Resume held to the ECM, frames left
    self.last_ccbuttons = None  # CEM's last CCButtons, copied for the Resume
    # Longitudinal
    self.next_long_tx_nanos = 0
    self.long_mode = LongMode.COAST
    self.accel = 0.0
    self.own = False  # OP owns the ECM with the camera's templates, the camera shown ACC standby
    self.exit_standby_frames = 0  # OP's long ended with the ECM still engaged: camera still shown standby
    self.tpl_fsm0: dict[int, dict] = {}  # camera's clean frames: FSM0 by Byte_0, FSM3 data/type, FSM4
    self.tpl_fsm3: dict[str, dict] = {}
    self.tpl_fsm4: dict | None = None
    self.brake_hold_b0: int | None = None
    # OP's stop to 0 (lead, red light, stop sign), as the camera's (route 807)
    self.driver_gas = False  # driver on the gas: OP's long off, the ECM kept engaged with no accel nor brake
    self.stopped_frames = 0
    self.car_hold_frames = 0
    self.stopping = False  # FSM4 b1 F9
    self.holding = False  # the car holds the stop: brake request released
    self.standstill = False  # FSM3 ACC_Standstill to the ECM, and the cruise standstill carstate reports
    self.driving_off = False  # resume sent, standstill released
    self.drive_off_request = False  # set with the resume burst, taken on the next frame
    self.drive_off_frames = 0
    self.drive_off_moved = False

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if CC.cruiseControl.cancel and CS.out.vEgo > CarControllerParams.MIN_STEER_SPEED and not CS.out.brakePressed:
      can_sends.append(create_button_msg(self.packer, cancel=True))

    if self.frame % 2 == 0:
      can_sends += self._lateral(CC, CS)

    self._ownership(CC, CS)
    # Carstate takes the ACC standstill from OP while it owns the ECM
    CS.op_owns_ecm = self.own
    CS.op_standstill = self.standstill
    self._resume_buttons(CC, CS, can_sends)
    for stock_fsm0 in CS.stock_FSM0_frames:
      can_sends.append(create_fsm0(self.packer, self._fsm0(stock_fsm0, CC.hudControl.leadVisible), CS.ecm_acc_engaged))

    if now_nanos >= self.next_long_tx_nanos:
      period = int(CarControllerParams.LONG_TX_PERIOD * 1e9)
      next_tx = self.next_long_tx_nanos + period
      if self.next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + period
      self.next_long_tx_nanos = next_tx
      can_sends += self._longitudinal(CC, CS)

    can_sends += self._relay_fsm4(CS)
    can_sends += self._relay_to_camera(CS)

    if self.frame % 100 == 0:
      self._custom_acc_step = self._read_acc_step()
    CS._custom_acc_step = self._custom_acc_step

    icbm_sends = IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame)
    if icbm_sends:
      CS._pending_delta = 0
      CS._icbm_suppress_frames = 25
    can_sends.extend(icbm_sends)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev
    if CC.longActive:
      new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends

  def _read_acc_step(self) -> int:
    if self._params is None:
      return 1
    return max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))

  def _lateral(self, CC, CS):
    """FSM2 steering request and PSCM1 servo state to the camera, 50 Hz."""
    CCP = CarControllerParams
    actuators = CC.actuators
    if CC.latActive and CS.out.vEgo > CCP.MIN_STEER_SPEED:
      apply_steer = apply_std_steer_angle_limits(
        actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw,
        CS.out.steeringAngleDeg, CC.latActive, CCP.ANGLE_LIMITS
      )
      apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

      error = CS.out.steeringAngleDeg - apply_steer
      error_with_deadzone = 0 if abs(error) < CCP.DEADZONE else error

      if not self.latActive_prev:
        self.apply_steer_dir_prev = apply_steer_dir

      # Block direction flips until the servo catches up
      if self.steer_blocked:
        if (apply_steer_dir == self.steer_dir_bf_block) or (self.steer_blocked_cnt <= 0) or (error_with_deadzone == 0):
          self.steer_blocked = False
      else:
        if apply_steer_dir != self.apply_steer_dir_prev and error_with_deadzone != 0:
          self.steer_blocked = True
          self.steer_blocked_cnt = CCP.BLOCK_LEN
          self.steer_dir_bf_block = self.apply_steer_dir_prev

      if self.steer_blocked:
        self.steer_blocked_cnt -= 1
        apply_steer_dir = SteerDirection.NONE
      elif error_with_deadzone == 0:
        apply_steer_dir = self.apply_steer_dir_prev
    else:
      apply_steer = 0
      apply_steer_dir = SteerDirection.NONE

    self.apply_steer_prev = apply_steer
    self.apply_steer_dir_prev = apply_steer_dir
    self.latActive_prev = CC.latActive

    return [
      create_lka_msg(self.packer, apply_steer, int(apply_steer_dir)),
      # Servo state to the camera
      create_lkas_state_msg(self.packer, CS.out.steeringAngleDeg, CS.pscm_stock_values),
    ]

  def _resume_buttons(self, CC, CS, can_sends):
    """Resume burst when the lead moves away."""
    CCP = CarControllerParams
    at_standstill = CS.out.cruiseState.enabled and CS.out.cruiseState.standstill and CS.out.vEgo < 0.01

    self.op_resume_frames = self.op_resume_frames + 1 if CC.cruiseControl.resume else 0
    op_resume = self.op_resume_frames * DT_CTRL >= CCP.SNG_OP_RESUME_TIME

    if (self.frame - self.last_resume_frame) * DT_CTRL > CCP.RESUME_BURST_INTERVAL:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0
      # In OP's stop only the planner's resume drives off (as Ford/Toyota), not the camera's distance jitter (route 884)
      lead_moved = CS.acc_distance > self.distance and not self.own
      # The driver's Resume also drives off OP's stop: the ECM takes it, but held in standstill it stays (route 8ab, 436 s)
      driver_resume = CS.driver_resume and self.own
      drive_off = lead_moved or op_resume or driver_resume or self.driver_gas
      # The ECM takes the driver's Resume at once, so OP follows it without waiting for its ready (route 38, 42 s)
      if at_standstill and self.waiting and drive_off and (driver_resume or self._drive_off_ready(CS)):
        self.resume_hold_frames = CCP.RESUME_HOLD_FRAMES
        if self.sng_count == 0:
          self.sng_ack_frames = round(CCP.SNG_ACK_TIME / CCP.LONG_TX_PERIOD)  # counted down per FSM3
        self.sng_count += 1
        self.drive_off_request = self.own
      if self.waiting and (self.sng_count >= CCP.RESUME_BURST_MAX or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    # Resume held as the driver's: the CEM's frame with Resume pressed, every frame
    if CS.stock_CCButtons_frames:
      self.last_ccbuttons = CS.stock_CCButtons_frames[-1]
    if self.resume_hold_frames > 0 and self.last_ccbuttons is not None:
      can_sends.append(create_resume_button(self.packer, self.last_ccbuttons))
    self.resume_hold_frames = max(self.resume_hold_frames - 1, 0)

  # ---- OP owning the ECM ----

  def _ownership(self, CC, CS):
    """Templates, and when OP takes and gives back the ECM."""
    CCP = CarControllerParams
    clean = cam_clean(CS)
    if CS.ecm_acc_engaged and clean and not self.own:
      for f in CS.stock_FSM0_frames:
        self.tpl_fsm0[int(f["Byte_0"])] = f
      # ACC_Check is the camera's instant ack: kept out of the template, the ECM braked in the drive-off (routes 8a2, 8a3)
      self.tpl_fsm3["data" if int(CS.stock_FSM3["FrameKind"]) == 0x81 else "type"] = {**CS.stock_FSM3, "ACC_Check": 0}
      if CS.stock_FSM4_frames:
        self.tpl_fsm4 = CS.stock_FSM4_frames[-1]
    have_templates = self.tpl_fsm0 and len(self.tpl_fsm3) == 2 and self.tpl_fsm4 is not None

    # Driver's gas keeps OP owning (the panda keeps the camera off the ECM) until OP's long is back (route 518bf777|6e)
    self.driver_gas = self.own and CC.enabled and (CS.out.gasPressed or (self.driver_gas and not CC.longActive))
    # Ended by the driver with the ECM still engaged: the camera kept in standby until the ECM drops (route 896)
    if self.exit_standby_frames > 0:
      done = not CS.ecm_acc_engaged or self.exit_standby_frames * DT_CTRL >= CCP.EXIT_STANDBY_MAX
      self.exit_standby_frames = 0 if (done or self.own) else self.exit_standby_frames + 1
    if not (CC.longActive or self.driver_gas):
      if self.own and CS.ecm_acc_engaged:
        self.exit_standby_frames = 1
      self._release()
    elif not self.own and have_templates and clean:
      self.own = True
      if CS.out.standstill:
        # Engaged at a stop: in standstill at once, as the camera, or the car keeps holding (route 38, 40 s)
        self.stopping = self.standstill = True
        self.stopped_frames = int(CCP.STANDSTILL_DELAY / DT_CTRL)

    if self.own:
      self._update_stop(CC, CS)
    self.drive_off_request = False

  def _release(self):
    self.own = self.driving_off = self.driver_gas = False
    self.standstill = self.stopping = self.holding = False
    self.stopped_frames = self.car_hold_frames = 0

  def _update_stop(self, CC, CS):
    """Stopping, the car's hold, standstill and the drive-off, as the camera does them."""
    CCP = CarControllerParams
    standstill = CS.out.standstill
    self.stopped_frames = self.stopped_frames + 1 if standstill else 0

    # The driver's gas drives off as OP's resume does
    self.drive_off_request |= self.driver_gas and not self.driving_off
    if self.drive_off_request:
      self.driving_off, self.drive_off_frames, self.drive_off_moved = True, 0, False
    if self.driving_off:
      # Stopped again (or never moved): standstill sequence again, resume retried
      self.drive_off_frames += 1
      self.drive_off_moved |= not standstill
      if standstill and self.drive_off_moved:
        self.driving_off = False
        self.stopped_frames = 1
      elif standstill and self.drive_off_frames * DT_CTRL > CCP.GO_RETRY_TIME:
        # Never moved: straight back to standstill, so the resume burst goes again
        self.driving_off = False
        self.stopping = True
        self.stopped_frames = int(CCP.STANDSTILL_DELAY / DT_CTRL)

    # Latched until the resume only once in standstill; before it OP drives off on traction, as the camera (route 807)
    at_stop = not self.driving_off and standstill and (self.standstill or CS.ecm_standstill_ready)
    stopping = not self.driving_off and ((CS.out.vEgo < CCP.STOPPING_FLAG_SPEED and CC.actuators.accel <= 0.) or
                                              (at_stop and (self.stopping or CS.car_hold)))
    self.car_hold_frames = self.car_hold_frames + 1 if (stopping and standstill and CS.car_hold) else 0
    self.stopping = stopping
    # Held until OP asks to drive after the resume, so no brake pulse at the start
    self.holding = not self.driver_gas and (
      self.car_hold_frames > CCP.HOLD_RELEASE_FRAMES or
      (self.holding and self.driving_off and standstill and CC.actuators.accel <= 0.))
    self.standstill = stopping and self.stopped_frames * DT_CTRL >= CCP.STANDSTILL_DELAY

  def _drive_off_ready(self, CS) -> bool:
    """The resume burst may go: the ECM ready to drive off from its standstill (or it never said so in time)."""
    if not self.own or CS.ecm_standstill_ready:
      return True
    CCP = CarControllerParams
    return self.stopped_frames * DT_CTRL >= CCP.STANDSTILL_DELAY + CCP.ECM_READY_TIMEOUT

  def _fsm0(self, stock_fsm0, lead: bool):
    """FSM0 to the ECM: the camera's, or its template while OP owns the ECM (its standby kept from the ECM)."""
    if not self.own:
      return stock_fsm0
    tpl = self.tpl_fsm0.get(int(stock_fsm0["Byte_0"]), next(iter(self.tpl_fsm0.values())))
    # Lead icon from OP's lead, not the template's frozen one, which flickered between keys (route 37, seg 3)
    return {**tpl, "ACC_FrontCar": int(lead)}

  # ---- Longitudinal ----

  def _longitudinal(self, CC, CS):
    """FSM1/FSM3 at 50 Hz: OP's accel and mode on the camera's template, or the camera's own frames."""
    CCP = CarControllerParams
    if self.own:
      if self.driver_gas:
        accel, mode = 0., LongMode.COAST
      else:
        accel, mode = self._long_decision(CC, CS)
      if self.holding:
        # The car holds the stop: the camera's standstill request (route 807)
        accel, mode = CCP.HOLD_ACCEL, LongMode.BRAKE
      elif self.stopping:
        # Stopping flag on: at least the camera's brake, or it drops to 0 and the car rolls on (route 2e, 59 s)
        accel, mode = min(accel, CCP.STOPPING_ACCEL), LongMode.BRAKE
      self.long_mode = mode
      fsm3 = self.tpl_fsm3["data" if int(CS.stock_FSM3["FrameKind"]) == 0x81 else "type"]
      standstill = self.standstill
    elif CS.ecm_acc_engaged:
      # The camera drives (OP's long off, or its frames not seen yet): its frames as they are, with the resume's ACC_Check
      accel, mode, standstill = float(CS.stock_FSM3["ACC_AccelerationRequest"]), None, None
      self.long_mode = LongMode.COAST
      fsm3 = CS.stock_FSM3
    else:
      # ECM off: the panda forwards the camera's FSM1/FSM3, a copy would be duplicated or blocked (route 2c, 837 s)
      self.long_mode = LongMode.COAST
      return []
    self.accel = accel

    if self.sng_ack_frames > 0:
      acc_check = 1
      self.sng_ack_frames -= 1
    else:
      acc_check = int(fsm3["ACC_Check"])
    return [
      create_radar(self.packer, CS.stock_FSM1, mode),
      create_longitudinal(self.packer, fsm3, accel, acc_check, mode, standstill=standstill),
    ]

  def _long_decision(self, CC, CS):
    """OP's accel within the limits and rate, and its mode."""
    CCP = CarControllerParams
    accel = min(max(CC.actuators.accel, CCP.ACCEL_MIN), CCP.ACCEL_MAX)
    accel = self._jerk_limit(accel, self.accel)
    brake_offset = self._slope_brake_offset(CC, CS.out.vEgo)
    return accel, next_long_mode(self.long_mode, accel, brake_offset, CS.out.vEgo)

  def _jerk_limit(self, accel, last_accel):
    """Rate limit OP's accel request: braking only against jumps, rising moderate."""
    CCP = CarControllerParams
    dt = CCP.LONG_TX_PERIOD
    return min(max(accel, last_accel - CCP.JERK_DOWN * dt), last_accel + CCP.JERK_UP * dt)

  @staticmethod
  def _slope_brake_offset(CC, v_ego=None):
    """Coast accel from pitch, faded out at low speed where coasting does not slow the car (route 22, 878 s)."""
    if len(CC.orientationNED) != 3:
      return 0.
    CCP = CarControllerParams
    offset = min(-CCP.COAST_PITCH_GAIN * math.sin(CC.orientationNED[1]), CCP.DOWNHILL_BRAKE_OFFSET_MAX)
    if v_ego is None:
      return offset
    lo, hi = CCP.BRAKE_ENTER_BP
    return offset * min(max((v_ego - lo) / (hi - lo), 0.), 1.)

  def _relay_fsm4(self, CS):
    """FSM4 to the ECM: OP's brake on the camera's template, or the camera's own."""
    sends = []
    for stock_fsm4 in CS.stock_FSM4_frames:
      if not self.own:
        sends.append(create_fsm4(self.packer, stock_fsm4, self.accel))
        continue
      # b0 alternates 55/AA as the camera's, b1 F1 -> F9 stopping, no brake but OP's
      fsm4 = {**self.tpl_fsm4, "Byte_0": stock_fsm4["Byte_0"], "ACC_BrakeRequest": 0, "ACC_Stopping": int(self.stopping),
              "Byte_5_b7": 1, "ACC_BrakeTarget": fsm4_brake_value(0., False) & 0x3FFF}
      # The car holds (or the driver's gas): brake request released, like the camera 50 ms after the hold
      mode = None if (self.holding or self.driver_gas) else self.long_mode
      sends.append(create_fsm4(self.packer, fsm4, self.accel, mode))
    return sends

  def _relay_to_camera(self, CS):
    """Car frames to the camera: with OP owning the ECM, ACC standby, no ACC brake, hold nor engaging button shown."""
    standby = self.own or self.exit_standby_frames > 0
    sends = []
    for stock_wheel_rear in CS.stock_WheelRear_frames:
      sends.append(create_cam_wheel_status(self.packer, stock_wheel_rear, standby))
    for stock_brake_req_ack in CS.stock_BrakeReqAck_frames:
      sends.append(create_cam_brake_request_ack(self.packer, stock_brake_req_ack, standby))
    for stock_engine_data in CS.stock_EngineData_frames:
      sends.append(create_cam_engine_data(self.packer, stock_engine_data, standby))
    for stock_acc_hold in CS.stock_AccHold_frames:
      sends.append(create_cam_acc_hold(self.packer, stock_acc_hold, standby))
    for stock_ccbuttons in CS.stock_CCButtons_frames:
      sends.append(create_cam_button(self.packer, stock_ccbuttons, standby))
    # 0x1FF b0 held while stopped (its 0x70 -> 0xD0 fires the camera's brake alert)
    hold_mask = self.own and CS.out.standstill
    for stock_brake_hold in CS.stock_BrakeHold_frames:
      if not hold_mask or self.brake_hold_b0 is None:
        self.brake_hold_b0 = int(stock_brake_hold["Byte_0"])
      sends.append(create_cam_brake_hold(self.packer, stock_brake_hold, self.brake_hold_b0 if hold_mask else None))
    return sends
