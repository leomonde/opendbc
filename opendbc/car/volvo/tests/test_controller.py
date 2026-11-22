import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, ButtonType, CanData, structs
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.values import CAR, DBC, CarControllerParams, EcmAccState

DBC_NAME = DBC[CAR.VOLVO_V60][Bus.pt]


def params():
  cp = CarInterface.get_non_essential_params(CAR.VOLVO_V60)
  return cp, CarInterface.get_non_essential_params_sp(cp, CAR.VOLVO_V60)


def required_frames(**overrides):
  packer = CANPacker(DBC_NAME)
  values = {
    "VehicleSpeed1": {"VehicleSpeed": 72},
    "AccPedal": {"AccPedal": 0},
    "Brake_Info": {"BrakePedal": 0},
    "PSCM1": {"SteeringAngleServo": 12.5, "EPSTorque": 10, "LKATorque": 5},
    "ACC_Speed": {"ACC_Speed": 100},
    "MiscCarInfo": {"TurnSignal": 1},
    "Doors": {"DriverDoorClosed": 1, "PassengerDoorClosed": 1},
    "SAS0": {"SteeringRateOfChange": 2, "SteeringDirection": 0},
    "EngineData": {"ACC_State_ECM": EcmAccState.ENGAGED},
    "CCButtons": {},
    "GearSelector": {"GearLever": 3},
    "WheelRear": {}, "BrakeReqAck": {}, "BrakeHold": {}, "AccHold": {},
  }
  cam_values = {"FSM0": {"ACC_Available": 1}, "FSM1": {}, "FSM3": {"ACC_Standstill": 0}, "FSM4": {}, "FSM5": {"TSR_Speed": 0}}
  for name, signals in overrides.items():
    (cam_values if name in cam_values else values)[name] = signals
  return [CanData(*packer.make_can_msg(name, 0, signals)) for name, signals in values.items()] + \
         [CanData(*packer.make_can_msg(name, 2, signals)) for name, signals in cam_values.items()]


def car_state():
  cp, cp_sp = params()
  cs = CarState(cp, cp_sp)
  parsers = cs.get_can_parsers(cp, cp_sp)
  for parser in parsers.values():
    parser.update([(1_000_000_000, required_frames())])
  return cs, parsers


def test_gas_press_and_release_have_the_panda_hysteresis():
  # A foot held at 10% flickered gas each frame (route 00000007): 10% presses, under 9% releases, as the panda
  cs, parsers = car_state()
  t = 1_000_000_000
  for pedal, pressed in ((9.9, False), (10, True), (9.9, True), (9, True), (8.9, False), (9.9, False), (10, True)):
    t += 10_000_000
    for parser in parsers.values():
      parser.update([(t, required_frames(AccPedal={"AccPedal": pedal}))])
    assert cs.update(parsers)[0].gasPressed == pressed, pedal


def test_driver_onoff_is_a_cancel_button():
  # Route 22, 418.4 s: the driver's On/Off reached the panda's permission before OP saw the ECM disengage
  cs, parsers = car_state()
  t = 1_000_000_000
  events = []
  for pressed in (False, True, True, False):
    t += 20_000_000
    buttons = {"ACCOnOffBtn": 1, "ACCOnOffBtnInv": 0} if pressed else {"ACCOnOffBtnInv": 1}
    for parser in parsers.values():
      parser.update([(t, required_frames(CCButtons=buttons))])
    events.append([(e.type, e.pressed) for e in cs.update(parsers)[0].buttonEvents if e.type == ButtonType.cancel])
  assert events == [[], [(ButtonType.cancel, True)], [], [(ButtonType.cancel, False)]]


def test_cruise_speed_is_the_drivers_set():
  # No downhill offset any more: the planner gets the driver's set
  cs, parsers = car_state()
  ret, _ = cs.update(parsers)
  assert ret.cruiseState.speed == pytest.approx(ret.cruiseState.speedCluster)


def control(lat_active=False, long_active=False, accel=0.):
  cc = structs.CarControl(enabled=long_active, latActive=lat_active, longActive=long_active)
  cc.actuators.accel = accel
  return cc.as_reader()


def controller_state(speed=20.):
  cs = type("CS", (), {})()
  cs.out = structs.CarState(vEgo=speed, vEgoRaw=speed)
  cs.pscm_stock_values = {"byte0": 0, "byte4": 0, "byte7": 0, "LKAActive": 0xFF, "EPSTorque": 0}
  cs.ecm_acc_engaged = False
  cs.ecm_standstill_ready = False
  cs.car_hold = False
  cs.acc_distance = 255
  cs.driver_resume = False
  cs.stock_FSM1 = {"ACC_Braking": 0}
  cs.stock_FSM3 = {"ACC_AccelerationRequest": 0., "ACC_Mode": 1, "ACC_NoTargetCancel": 0, "Byte_6": 0, "FrameKind": 0x81, "ACC_Check": 0}
  for name in ("FSM0", "FSM4", "WheelRear", "BrakeReqAck", "BrakeHold", "EngineData", "AccHold", "CCButtons"):
    setattr(cs, f"stock_{name}_frames", [])
  return cs


def controller():
  cp, cp_sp = params()
  return CarController({Bus.pt: DBC_NAME}, cp, cp_sp)


def run(cc, CC, cs, now_nanos):
  return cc.update(CC, structs.CarControlSP(), cs, now_nanos)[1]


def test_longitudinal_tx_is_50hz_and_keeps_the_relays():
  cc = controller()
  cs = controller_state()
  cs.stock_CCButtons_frames = [{"ACCResumeBtn": 0, "ACCResumeBtnInv": 1, "ACCSetBtn": 0, "ACCSetBtnInv": 1}]
  cs.ecm_acc_engaged = True
  sent = []
  for i in range(10):
    sent += [s[0] for s in run(cc, control(), cs, i * 10_000_000)]
  assert sent.count(0x270) == sent.count(0x260) == 5  # FSM3, FSM1
  assert sent.count(0x127) == 10  # CEM frames relayed to the camera


def test_no_fsm1_fsm3_with_the_ecm_off():
  # Route 2c, 837 s: with the ECM off the panda forwards the camera's frames and blocked OP's copies
  cc = controller()
  cs = controller_state()
  sent = []
  for i in range(10):
    sent += [s[0] for s in run(cc, control(), cs, i * 10_000_000)]
  assert 0x270 not in sent and 0x260 not in sent


def test_stopping_keeps_the_camera_brake():
  # Route 2e, 59 s: the planner's -0.15 at the stopping flag dropped the brake to 0 and the car rolled on a climb
  cc = controller()
  cs = controller_state(speed=0.5)
  cs.ecm_acc_engaged = True
  cc.own = True
  cc.tpl_fsm3 = {"data": dict(cs.stock_FSM3), "type": {**cs.stock_FSM3, "FrameKind": 0}}
  cc.tpl_fsm4 = {"Byte_0": 0x55, "ACC_BrakeRequest": 0, "ACC_BrakeTarget": 13332, "Byte_5_b7": 1}
  cc.tpl_fsm0 = {0: {"Byte_0": 0, "ACC_AbortCode": 0, "ACC_Enabled": 1}}
  sends = run(cc, control(long_active=True, accel=-0.15), cs, 0)
  fsm3 = next(s[1] for s in sends if s[0] == 0x270)
  assert cc.stopping and fsm3[2] & 0x3 == 0  # BRAKE
  assert fsm3[1] == round((CarControllerParams.STOPPING_ACCEL + 5.04) / 0.04)
