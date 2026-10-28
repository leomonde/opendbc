import math
from types import SimpleNamespace

import pytest

from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.values import CarControllerParams, LongMode
from opendbc.car.volvo import volvocan
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


def test_traction_continuous_at_drive_switch():
  # Coast and drive traction meet at the DRIVE threshold, like the camera
  a = CCP.DRIVE_ENTER
  coast = volvocan.fsm3_traction(a, LongMode.COAST) - 256
  drive = volvocan.fsm3_traction(a, LongMode.DRIVE)
  assert drive - coast <= 6


KPH = 1 / 3.6


def tolerance(v_kph, set_kph=110., accel=-0.5, pitch_deg=-2., lead_m=255, braking=False):
  CC = SimpleNamespace(hudControl=SimpleNamespace(setSpeed=set_kph * KPH),
                       orientationNED=[0., math.radians(pitch_deg), 0.])
  CS = SimpleNamespace(out=SimpleNamespace(vEgo=v_kph * KPH), acc_distance=lead_m)
  return CarController.coast_tolerance(CC, CS, accel, CarController.slope_brake_offset(CC), braking)


def test_coast_tolerance_downhill_over_set():
  assert tolerance(110.5)
  assert tolerance(114.)
  assert not tolerance(115.5)          # above set + 5: brakes
  assert not tolerance(90.)            # held below the set: no tolerance
  assert not tolerance(112., pitch_deg=0.)  # flat
  assert tolerance(112., accel=-1.2)   # overspeed request still coasts
  assert not tolerance(112., accel=-1.6)  # strong request
  assert not tolerance(112., lead_m=40)   # camera target closer than 2.5 s
  assert tolerance(112., lead_m=120)


def test_coast_tolerance_brakes_back_to_set():
  assert not tolerance(113., braking=True)
  assert not tolerance(110.5, braking=True)
  assert tolerance(109.8, braking=True)
  assert not tolerance(108., braking=True)  # well below the set: normal logic


def limited(accel, last):
  cc = CarController.__new__(CarController)
  cc.LONG_TX_PERIOD_NANOS = 20_000_000
  return cc.jerk_limit(accel, last)


def test_jerk_limit_ramps():
  dt = 0.02
  assert abs(limited(1.3, 0.0) - CCP.JERK_UP * dt) < 1e-9
  assert abs(limited(-0.4, 1.3) - (1.3 - CCP.JERK_DOWN_LIFT * dt)) < 1e-9
  assert abs(limited(-3.5, -1.0) - (-1.0 - CCP.JERK_DOWN_BRAKE * dt)) < 1e-9
  assert limited(0.31, 0.3) == 0.31


def test_jerk_limit_reaches_target():
  a = 1.3
  for _ in range(100):
    a = limited(-0.4, a)
  assert a == -0.4


def cam_target(dist, v=24.):
  return SimpleNamespace(acc_distance=dist, out=SimpleNamespace(vEgo=v))


def test_accel_capped_by_camera_with_target():
  # Motorcycle case: OP wants +2, camera holds back
  assert CarController.camera_accel_cap(cam_target(22), 2.0, -0.5) == CCP.CAMERA_ACCEL_CAP_FLOOR
  assert CarController.camera_accel_cap(cam_target(22), 2.0, 0.6) == 0.6


def test_accel_cap_never_adds_braking():
  assert CarController.camera_accel_cap(cam_target(22), -1.5, -0.5) == -1.5
  assert CarController.camera_accel_cap(cam_target(22), -0.2, -2.0) == -0.2


def test_accel_cap_off_without_close_target():
  assert CarController.camera_accel_cap(cam_target(255), 2.0, -0.5) == 2.0
  assert CarController.camera_accel_cap(cam_target(70), 2.0, 1.0) == 2.0
  assert CarController.camera_accel_cap(cam_target(9, v=0.), 1.0, 0.2) == 0.2
