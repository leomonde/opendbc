import math
from types import SimpleNamespace

import pytest

from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.values import CarControllerParams, LongMode
from opendbc.car.volvo.volvocan import next_long_mode

CCP = CarControllerParams
HARD = CCP.CAMERA_BRAKE_HONOR_BELOW - 0.5


def controller(cam_brake=False):
  cc = CarController.__new__(CarController)
  cc.cam_brake = cam_brake
  cc.cam_brake_honor = False
  return cc


def test_hard_camera_brake_followed_and_latched():
  cc = controller(cam_brake=True)
  cc.update_cam_brake_honor(HARD, False)
  assert cc.cam_brake_honor
  cc.update_cam_brake_honor(-1.0, False)
  assert cc.cam_brake_honor  # latched while the camera brakes
  cc.cam_brake = False
  cc.update_cam_brake_honor(-1.0, False)
  assert not cc.cam_brake_honor


def test_mild_camera_brake_ignored_except_final_stop():
  cc = controller(cam_brake=True)
  cc.update_cam_brake_honor(-1.0, False)
  assert not cc.cam_brake_honor
  cc.update_cam_brake_honor(-1.0, True)
  assert cc.cam_brake_honor


def cs(mode):
  return SimpleNamespace(cam_fsm3_mode=mode)


@pytest.mark.parametrize("op_mode", [LongMode.BRAKE, LongMode.COAST, LongMode.DRIVE])
def test_fsm3_mirrors_camera_mode_0(op_mode):
  assert CarController.fsm3_mode(cs(LongMode.BRAKE), op_mode) == LongMode.BRAKE


@pytest.mark.parametrize("cam_mode", [LongMode.COAST, LongMode.DRIVE])
@pytest.mark.parametrize("op_mode", [LongMode.BRAKE, LongMode.COAST, LongMode.DRIVE])
def test_fsm3_keeps_op_mode_otherwise(cam_mode, op_mode):
  assert CarController.fsm3_mode(cs(cam_mode), op_mode) == op_mode


def test_fsm3_untouched_without_op_mode():
  assert CarController.fsm3_mode(cs(LongMode.BRAKE), None) is None


def offset(pitch_deg):
  return CarController.slope_brake_offset(SimpleNamespace(orientationNED=[0., math.radians(pitch_deg), 0.]))


def test_uphill_coast_is_not_braking():
  # route 858: -0.84 on a 5.4 deg slope is just coasting; route 85b: the coast cap overshot to -1.15
  assert next_long_mode(LongMode.COAST, -0.84) == LongMode.BRAKE
  assert next_long_mode(LongMode.COAST, -0.84, offset(5.4)) == LongMode.COAST
  assert next_long_mode(LongMode.COAST, -1.15, offset(5.0)) == LongMode.COAST
  assert next_long_mode(LongMode.COAST, -1.6, offset(5.0)) == LongMode.BRAKE
  assert next_long_mode(LongMode.BRAKE, -0.8, offset(5.0)) == LongMode.BRAKE  # holds down to the usual exit


def test_downhill_brakes_earlier():
  # route 85a: on a 1.5 deg descent a -0.4 request coasted and the car sped up
  assert next_long_mode(LongMode.COAST, -0.4) == LongMode.COAST
  assert next_long_mode(LongMode.COAST, -0.4, offset(-1.5)) == LongMode.BRAKE
  assert offset(-10.) == CCP.DOWNHILL_BRAKE_OFFSET_MAX
  assert next_long_mode(LongMode.BRAKE, 0.01, offset(-10.)) == LongMode.COAST


def test_flat_unchanged():
  assert offset(0.) == 0.
  assert CarController.slope_brake_offset(SimpleNamespace(orientationNED=[])) == 0.
