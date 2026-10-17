import copy
import math

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams, LongMode

DBC = "volvo_v60_2015_pt"
FSM3 = 0x270

# stock FSM3 frames from the camera, route 00000808 (ACC engaged); b4=0x81 data, 0x00 type
STOCK = {
  ("data", LongMode.BRAKE): ["aa7ad40081000000", "aa7ed40081000000"],
  ("data", LongMode.COAST): ["aa6ed5b881000000", "aa78d5dd81000000"],
  ("data", LongMode.DRIVE): ["aa98d65f81000000", "aa88d61d81000000"],
  ("type", LongMode.BRAKE): ["0082000000000000", "0076000000000000"],
  ("type", LongMode.COAST): ["007a01ec00000000", "007c01f300000000"],
  ("type", LongMode.DRIVE): ["0088022100000000", "008a022600000000"],
}
ALL_FRAMES = [(kind, mode, bytes.fromhex(h)) for (kind, mode), hs in STOCK.items() for h in hs]


def parse_fsm3(dat: bytes) -> dict:
  cp = CANParser(DBC, [("FSM3", 50)], 0)
  cp.update([(0, [(FSM3, dat, 0)])])
  return copy.copy(cp.vl["FSM3"])


def build(stock: bytes, accel=None, mode=None) -> bytes:
  fsm3 = parse_fsm3(stock)
  if accel is None:
    accel = fsm3["ACC_AccelerationRequest"]
  return volvocan.create_longitudinal(CANPacker(DBC), fsm3, accel, int(fsm3["ACC_Check"]), mode)[1]


@pytest.mark.parametrize("kind,mode,stock", ALL_FRAMES)
def test_relay_is_byte_identical(kind, mode, stock):
  assert build(stock) == stock


@pytest.mark.parametrize("kind,mode,stock", ALL_FRAMES)
def test_stock_mode_and_traction_reproduce_frame(kind, mode, stock):
  # feeding the camera's own mode back must not touch Byte_2; BRAKE frames also match Byte_3 (always 0)
  out = build(stock, mode=mode)
  assert out[2] == stock[2]
  assert out[:2] == stock[:2] and out[4:] == stock[4:]
  if mode == LongMode.BRAKE:
    assert out[3] == 0


@pytest.mark.parametrize("kind,stock_mode,stock", ALL_FRAMES)
@pytest.mark.parametrize("mode", list(LongMode))
def test_op_mode_keeps_cadence(kind, stock_mode, stock, mode):
  accel = {LongMode.BRAKE: -1.5, LongMode.COAST: -0.1, LongMode.DRIVE: 0.8}[mode]
  out = build(stock, accel=accel, mode=mode)
  assert out[2] & 0x03 == mode
  assert out[2] & 0xFC == stock[2] & 0xFC  # 0xD4 data / 0x00 type kept
  assert out[4] == stock[4]                # data/type marker kept
  assert out[0] == stock[0]                # standstill and status bits kept
  assert out[1] == math.floor((accel + 5.04) / 0.04 + 0.5)  # packer rounds half up
  assert out[3] == volvocan.fsm3_traction(accel, mode)


def test_traction():
  assert volvocan.fsm3_traction(-2.0, LongMode.BRAKE) == 0
  assert volvocan.fsm3_traction(1.0, LongMode.DRIVE) == 85
  assert volvocan.fsm3_traction(2.0, LongMode.DRIVE) == 165
  assert volvocan.fsm3_traction(-0.5, LongMode.DRIVE) == 0
  assert volvocan.fsm3_traction(-0.1, LongMode.COAST) == 256 - 21
  assert volvocan.fsm3_traction(0.5, LongMode.COAST) == 0xFF  # never non-negative in COAST


def test_mode_hysteresis():
  CCP = CarControllerParams
  m = LongMode.COAST
  m = volvocan.next_long_mode(m, -0.4)
  assert m == LongMode.COAST
  m = volvocan.next_long_mode(m, CCP.BRAKE_ENTER - 0.01)
  assert m == LongMode.BRAKE
  m = volvocan.next_long_mode(m, CCP.BRAKE_EXIT - 0.05)  # stays braking until accel back to BRAKE_EXIT
  assert m == LongMode.BRAKE
  m = volvocan.next_long_mode(m, CCP.BRAKE_EXIT + 0.05)
  assert m == LongMode.COAST
  m = volvocan.next_long_mode(m, 0.06)
  assert m == LongMode.COAST
  m = volvocan.next_long_mode(m, 0.1)
  assert m == LongMode.DRIVE
  m = volvocan.next_long_mode(m, 0.05)
  assert m == LongMode.DRIVE
  m = volvocan.next_long_mode(m, -1.0)  # hard brake request goes straight from DRIVE to BRAKE
  assert m == LongMode.BRAKE
  m = volvocan.next_long_mode(m, 1.0)
  assert m == LongMode.DRIVE
