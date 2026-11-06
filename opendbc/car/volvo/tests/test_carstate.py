import pytest

from opendbc.can import CANPacker
from opendbc.car import Bus, ButtonType, CanData, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.interface import CarInterface
from opendbc.car.volvo.values import CAR, DBC, EcmAccState

GearShifter = structs.CarState.GearShifter
CCBUTTONS = 0x127
IDLE = bytes.fromhex("88108761a7ff0000")  # CCButtons, no button pressed (V60 routes)


@pytest.fixture
def car():
  cp = CarInterface.get_non_essential_params(CAR.VOLVO_V60)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.VOLVO_V60)
  return CarState(cp, cp_sp), CarState.get_can_parsers(cp, cp_sp)


def step(car, t, dat):
  cs, parsers = car
  for parser in parsers.values():
    parser.update([(t, [CanData(CCBUTTONS, dat, 0)])])
  ret, _ = cs.update(parsers)
  return [(be.type, be.pressed) for be in ret.buttonEvents]


@pytest.mark.parametrize("tsr_kph", [0, 30, 80, 120])
def test_tsr_speed_limit(car, tsr_kph):
  cs, parsers = car
  packer = CANPacker(DBC[CAR.VOLVO_V60][Bus.pt])
  addr, dat, bus = packer.make_can_msg("FSM5", 2, {"TSR_Speed": tsr_kph})
  for parser in parsers.values():
    parser.update([(1_000_000, [CanData(addr, dat, bus)])])
  _, ret_sp = cs.update(parsers)
  assert ret_sp.speedLimit == pytest.approx(tsr_kph * CV.KPH_TO_MS)


@pytest.mark.parametrize("gear, speed_kph, expected", [
  (0, 0, GearShifter.park),
  (1, 0, GearShifter.reverse),
  (1, 8, GearShifter.reverse),
  (2, 0, GearShifter.neutral),
  (3, 40, GearShifter.drive),
])
def test_gear_lever(car, gear, speed_kph, expected):
  cs, parsers = car
  packer = CANPacker(DBC[CAR.VOLVO_V60][Bus.pt])
  msgs = [packer.make_can_msg("GearSelector", 0, {"GearLever": gear}),
          packer.make_can_msg("VehicleSpeed1", 0, {"VehicleSpeed": speed_kph})]
  for parser in parsers.values():
    parser.update([(1_000_000, [CanData(*m) for m in msgs])])
  ret, _ = cs.update(parsers)
  assert ret.gearShifter == expected


def test_lka_press_events(car):
  cs, parsers = car
  packer = CANPacker(DBC[CAR.VOLVO_V60][Bus.pt])
  events = []
  for t, lka in enumerate((1, 1, 0, 0, 1)):
    msgs = [packer.make_can_msg("CCButtons", 0, {"LKA_State": lka})]
    for parser in parsers.values():
      parser.update([(1_000_000 * (t + 1), [CanData(*m) for m in msgs])])
    ret, _ = cs.update(parsers)
    events.append([(be.type, be.pressed) for be in ret.buttonEvents if be.type == ButtonType.lkas])
  # one press and release per LKA_State flip (MADS toggles on it), none at start
  flip = [(ButtonType.lkas, True), (ButtonType.lkas, False)]
  assert events == [[], [], flip, [], flip]


def test_driver_resume_events(car):
  # Resume needs the bit set and its inverted copy cleared, frames from route 8ab
  resume = bytes.fromhex("88108761a7df0020")
  no_inv = bytes.fromhex("88108761a7ff0020")
  assert step(car, 1_000_000, IDLE) == []
  assert step(car, 2_000_000, no_inv) == []
  assert step(car, 3_000_000, resume) == [(ButtonType.resumeCruise, True)]
  assert step(car, 4_000_000, resume) == []
  assert step(car, 5_000_000, IDLE) == [(ButtonType.resumeCruise, False)]


@pytest.mark.parametrize("speed_kph, expected", [(0, []), (40, [(ButtonType.accelCruise, False)])])
def test_set_plus_stopped_is_not_accel(car, speed_kph, expected):
  # Stopped, Set+ only sets the speed: an accelCruise there reads as "go" to the planner (route 8b2, 572 s)
  cs, parsers = car
  packer = CANPacker(DBC[CAR.VOLVO_V60][Bus.pt])
  events = []
  for t, set_kph in enumerate((30, 30, 35)):
    msgs = [packer.make_can_msg("EngineData", 0, {"ACC_State_ECM": EcmAccState.ENGAGED}),
            packer.make_can_msg("VehicleSpeed1", 0, {"VehicleSpeed": speed_kph}),
            packer.make_can_msg("ACC_Speed", 0, {"ACC_Speed": set_kph})]
    for parser in parsers.values():
      parser.update([(1_000_000 * (t + 1), [CanData(*m) for m in msgs])])
    ret, _ = cs.update(parsers)
    events = [(be.type, be.pressed) for be in ret.buttonEvents]
  assert events == expected
