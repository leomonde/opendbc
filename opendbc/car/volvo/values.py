from dataclasses import dataclass, field
from enum import IntEnum

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds, DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

# EUCD buses: pt, body (radar), cam


class SteerDirection(IntEnum):
  """LKASteerDirection."""
  NONE = 0
  RIGHT = 1
  LEFT = 2
  BOTH = 3


class LongMode(IntEnum):
  """FSM3 ACC_Mode."""
  BRAKE = 0
  COAST = 1
  DRIVE = 2


class EcmAccState(IntEnum):
  """EngineData ACC_State_ECM."""
  OFF = 1
  STANDBY = 2
  ENGAGED = 3


class CarControllerParams:
  # Lateral
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(90,
    ([0., 5., 15.], [5., .8, .15]),
    ([0., 5., 15.], [5., 3.5, 0.4]),
  )
  STEER_TIMEOUT = 30 / DT_CTRL  # frames of 0 torque before the EPS faults
  BLOCK_LEN = 8   # frames to block steering on a direction change
  DEADZONE = 0.2  # deg, keep the previous direction inside it
  BUTTON_BURST = 15  # frames per ICBM button press
  # Resume held this long on copies of the CEM's CCButtons: a one-shot burst was lost amid its frames (route 89e)
  RESUME_HOLD_FRAMES = int(0.3 / DT_CTRL)
  # s of planner resume before SNG: 0.2 let a red-light flicker through, the ECM drove off on its own (route 8a4)
  SNG_OP_RESUME_TIME = 0.5
  MIN_STEER_SPEED = 0.1  # m/s, no steering nor cancel button below this

  # OP's stop to 0 (route 807): below the enter speed, braking, OP owns the ECM and the camera is shown ACC standby
  FULL_STOP = True
  FULL_STOP_ENTER_SPEED = 25. * CV.KPH_TO_MS  # OP takes the ECM below this while braking; 30 handed back and forth with the resync at 32 (route 89b)
  STOPPING_FLAG_SPEED = 2.6 * CV.KPH_TO_MS  # FSM4 b1 F1 -> F9 below this
  HOLD_RELEASE_FRAMES = int(0.05 / DT_CTRL)  # brake request released after the car's hold
  STANDSTILL_DELAY = 3.0  # s stopped before FSM3 ACC_Standstill
  HOLD_ACCEL = -0.08  # m/s^2, FSM3 accel while the car holds (raw 0x7C)
  GO_RETRY_TIME = 2.0  # s still stopped after the resume burst -> standstill again, retry
  ECM_READY_TIMEOUT = 3.0  # s after FSM3 ACC_Standstill: resume sent even without the ECM's ready (seen 1.5 s)
  # Camera shown ACC standby for the whole stop, re-engaged like on the driver's Set: Set, then EngineData 30 ms later
  CAM_RESYNC_SPEED = 32. * CV.KPH_TO_MS  # re-engage above this (its ACC minimum is 30)
  CAM_SET_FRAMES = int(0.16 / DT_CTRL)  # Set press to the camera, as long as the driver's (route 885)
  CAM_SPOOF_RELEASE_FRAMES = int(0.03 / DT_CTRL)  # EngineData engaged again this long after the Set
  CAM_RESYNC_RETRY = 2.0  # s without the camera back -> standby again for 1 s, Set again
  CAM_RESYNC_ATTEMPTS = 3  # then the camera's frames back to the ECM (ACC drops)
  FULL_STOP_EXIT_STANDBY_MAX = 1.0  # s, stop ended with the ECM still engaged: camera kept in standby at most this long
  CAM_CLEAN_FRAMES = int(0.3 / DT_CTRL)  # camera engaged and clean this long -> its frames back to the ECM

  # Longitudinal limits
  ACCEL_MIN = -3.5  # m/s^2, as the panda's (the camera's harder brake goes out as its own frame)
  ACCEL_MAX = 2.0   # m/s^2

  # FSM3 mode hysteresis on OP accel
  BRAKE_ENTER = -0.50  # m/s^2
  BRAKE_EXIT = -0.15
  DRIVE_ENTER = 0.08
  DRIVE_EXIT = 0.04

  # FSM3 ACC_TractionRequest = gain * accel + offset, as the camera sends it
  TRACTION_COAST = (94., -9.)
  TRACTION_DRIVE = (97., -6.)

  # FSM4 14-bit brake target vs accel, with and without brake flag
  FSM4_BRAKE_ACCEL_BP = [-3.5, -2.5, -1.75, -1.25, -0.8, -0.45, -0.2, 0.0, 0.2]
  FSM4_BRAKE_VALUE_V = [12519, 12850, 12964, 13073, 13143, 13227, 13279, 13328, 13378]
  FSM4_NO_BRAKE_ACCEL_BP = [-0.45, -0.2, 0.0, 0.2, 0.45, 0.8, 1.5]
  FSM4_NO_BRAKE_VALUE_V = [13276, 13299, 13318, 13338, 13333, 13377, 13449]

  # Brake acks to the camera, masked while OP brakes alone
  CAM_BRAKE_MASK_MSGS = frozenset({"wheelspeed1", "BrakeReqAck"})
  CAM_BRAKE_MASK_HOLD_FRAMES = int(1.0 / DT_CTRL)
  CAM_ACK_RELEASE_VALUE = 13280  # spoofed BrakeReqAck released at this FSM4 target

  # FSM3 mirrors camera mode 0; camera hydraulic brake followed below this
  CAMERA_BRAKE_HONOR_BELOW = -3.5  # m/s^2
  # With a close camera target, OP accel capped at the camera's, not below the floor
  CAMERA_ACCEL_CAP_FLOOR = 0.0  # m/s^2
  CAMERA_ACCEL_CAP_GAP = 1.5  # s, camera target closer than this
  CAMERA_ACCEL_CAP_MIN_DIST = 10.  # m, at low speed

  # BRAKE thresholds shifted by the slope's coast accel
  COAST_PITCH_GAIN = 5.65  # m/s^2 per sin(pitch)
  DOWNHILL_BRAKE_OFFSET_MAX = 0.15  # m/s^2, downhill shift cap
  # Extra uphill margin before BRAKE
  UPHILL_BRAKE_ENTER_EXTRA = 0.6  # fraction of the uphill offset
  # Downhill: the planner targets set + this, so OP holds it with light braking (routes 89f, 8a1)
  DOWNHILL_SET_OFFSET = 5. * CV.KPH_TO_MS
  DOWNHILL_ENTER_PITCH = -2.  # deg, downhill below this for DOWNHILL_ENTER_TIME
  DOWNHILL_EXIT_PITCH = -1.  # deg, back to the set above this for DOWNHILL_EXIT_TIME
  DOWNHILL_ENTER_TIME = 2.0  # s
  DOWNHILL_EXIT_TIME = 1.0  # s

  # Accel request ramps (jerk limits)
  JERK_UP = 2.0  # m/s^3, rising
  JERK_DOWN_LIFT = 2.5  # m/s^3, falling while positive
  JERK_DOWN_BRAKE = 4.0  # m/s^3, falling at or below 0

  def __init__(self, CP):
    pass

class CANBUS:
  pt = 0
  body = 1  # radar bus
  cam = 2


# Delphi ESR 2.5 radar
RADAR_ESR = "ESR"

@dataclass
class VolvoEUCDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'volvo_v60_2015_pt',
    Bus.radar: RADAR_ESR,
  })


@dataclass
class VolvoCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control & Lane Keeping Aid"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass(frozen=True)
class VolvoCarSpecs(CarSpecs):
  steerRatio: float = 15.0
  centerToFrontRatio: float = 0.44
  minSteerSpeed: float = 0.  # the port's own limit is MIN_STEER_SPEED


class CAR(Platforms):
  VOLVO_V60 = VolvoEUCDPlatformConfig(
    [VolvoCarDocs("Volvo V60 2010-18")],
    VolvoCarSpecs(mass=1750, wheelbase=2.776),
  )


VOLVO_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf1a2)
VOLVO_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf1a2)

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [VOLVO_VERSION_REQUEST],
      [VOLVO_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
