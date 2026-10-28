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
  BUTTON_BURST = 15  # frames per button press
  SNG_OP_RESUME_TIME = 0.5  # s of OP resume before SNG

  # Longitudinal limits
  ACCEL_MIN = -4.0  # m/s^2
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
  # Downhill coast tolerance over the ACC set speed
  COAST_TOLERANCE = 5. * CV.KPH_TO_MS  # brakes from set + this, releases at set
  COAST_TOLERANCE_MAX_DECEL = -1.5  # m/s^2, stronger requests brake
  COAST_TOLERANCE_LEAD_GAP = 2.5  # s, no tolerance with a closer camera target
  COAST_TOLERANCE_AT_SET = 1. * CV.KPH_TO_MS  # below the set, still counted as at set

  # Accel request ramps (jerk limits)
  JERK_UP = 2.0  # m/s^3, rising
  JERK_DOWN_LIFT = 2.5  # m/s^3, falling while positive
  JERK_DOWN_BRAKE = 4.0  # m/s^3, falling at or below 0

  # Camera final stop behind a lead
  CAMERA_STOPS_BEHIND_LEAD = True
  CAM_STOP_HANDOVER_SPEED = 20. * CV.KPH_TO_MS  # camera stops below this, with a target
  CAM_STOP_RETURN_SPEED = 30. * CV.KPH_TO_MS  # OP takes back above this
  CAM_STOP_TRANSITION_TIME = 1.0  # s, OP/camera blend
  OP_BRAKE_OVERRIDE_ACCEL = -0.3  # m/s^2, OP brakes below this if the camera doesn't
  OP_BRAKE_OVERRIDE_MARGIN = 1.0  # m/s^2, OP brakes if this far below the camera

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
  minSteerSpeed: float = 0.1


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
