from dataclasses import dataclass, field
from enum import IntEnum

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds, DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

# Volvo EUCD/C1MCA CAN buses: pt (BCM/FSM/PSCM/SAS), chassis (DIM/PAM via CEM), ext (SODL/SODR via CEM)


class SteerDirection(IntEnum):
  """LKASteerDirection: EUCD needs 8-frame wait on direction change; C1MCA can use BOTH."""
  NONE = 0
  RIGHT = 1
  LEFT = 2
  BOTH = 3


class LongMode(IntEnum):
  """FSM3 Byte_2 bits 1:0. The ECM uses the hydraulic brake only in BRAKE."""
  BRAKE = 0
  COAST = 1
  DRIVE = 2


class EcmAccState(IntEnum):
  """EngineData Byte_4 bits 5:4, the ECM's ACC state."""
  OFF = 1
  STANDBY = 2
  ENGAGED = 3


class NoTargetHold(IntEnum):
  """Red-light stop with no target: HOLD shows the camera the ECM in standby while the ECM stays engaged;
  RELEASE shows it engaged again until the camera re-engages."""
  OFF = 0
  HOLD = 1
  RELEASE = 2


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
  SNG_OP_RESUME_TIME = 0.5  # s of openpilot's resume request before it triggers the SNG resume

  # Longitudinal limits
  ACCEL_MIN = -4.0  # m/s^2
  ACCEL_MAX = 2.0   # m/s^2

  # FSM3 mode hysteresis on OP accel (fit to the stock camera)
  BRAKE_ENTER = -0.50  # m/s^2
  BRAKE_EXIT = -0.15
  DRIVE_ENTER = 0.08
  DRIVE_EXIT = 0.04

  # FSM4 14-bit brake target vs accel (stock camera means), with and without the brake flag
  FSM4_BRAKE_ACCEL_BP = [-3.5, -2.5, -1.75, -1.25, -0.8, -0.45, -0.2, 0.0, 0.2]
  FSM4_BRAKE_VALUE_V = [12519, 12850, 12964, 13073, 13143, 13227, 13279, 13328, 13378]
  FSM4_NO_BRAKE_ACCEL_BP = [-0.45, -0.2, 0.0, 0.2, 0.45, 0.8, 1.5]
  FSM4_NO_BRAKE_VALUE_V = [13276, 13299, 13318, 13338, 13333, 13377, 13449]

  # Brake acks to the camera (0x220 Byte_2 bit2, 0x299 Byte_5 bit3): masked while OP brakes alone, and this long after
  CAM_BRAKE_MASK_MSGS = frozenset({0x220, 0x299})
  CAM_BRAKE_MASK_HOLD_FRAMES = int(1.0 / DT_CTRL)

  # Camera brake requests: OP ignores mild ones (spoofed ack), honors them below CAMERA_BRAKE_HONOR_BELOW
  OP_IGNORES_MILD_CAMERA_BRAKE = True
  CAMERA_BRAKE_HONOR_BELOW = -2.0  # m/s^2
  CAM_ACK_RELEASE_VALUE = 13280  # spoofed 0x299 ack drops when the camera's FSM4 target reaches this (no braking)

  # Camera final stop behind a lead
  CAMERA_STOPS_BEHIND_LEAD = True
  CAM_STOP_HANDOVER_SPEED = 10. * CV.KPH_TO_MS  # camera takes over below this, with a target
  CAM_STOP_RETURN_SPEED = 15. * CV.KPH_TO_MS    # OP takes back above this
  CAM_STOP_TRANSITION_TIME = 1.0  # s, blend between OP and camera
  OP_BRAKE_OVERRIDE_ACCEL = -0.3  # m/s^2, OP brakes if it wants less than this and the camera does not brake
  OP_BRAKE_OVERRIDE_MARGIN = 1.0  # m/s^2, OP brakes if it wants this much more than the camera

  # Red-light stop with no target (on for the virtual red light test; route 832 locked the camera up)
  OP_STOPS_WITHOUT_TARGET = True
  NO_TARGET_STOP_SPEED = 10. * CV.KPH_TO_MS  # hide the camera's no-target cancel below this
  STOPPING_FLAG_SPEED = 3.5 * CV.KPH_TO_MS   # FSM4 "stopping" flag below this
  NO_TARGET_STANDSTILL_DELAY = 2.5  # s stopped before FSM3 reports standstill
  NO_TARGET_RELEASE_TIMEOUT = 3.0   # s for the camera to re-engage after the go
  NO_TARGET_FAKE_BUTTONS = True  # show the camera an On/Off press at the hold and a Resume press at the go
  # the camera cancels ~1 s below 5 km/h with no target (routes 827/828/832/835): show it this speed instead.
  # Only in experimental mode, with no camera target and no pedal.
  NO_TARGET_SPEED_FLOOR = 6.  # km/h, None = off
  NO_TARGET_BUTTON_TIME = 0.15  # s, a real press
  NO_TARGET_STATE_DELAY = 0.04  # s from the press to the ECM state change, as the real ECM

  # Keep the ECM engaged on the camera's last good frames after a camera abort (latched a fault in testing)
  OP_OWNS_ACC_AFTER_CAMERA_ABORT = False

  def __init__(self, CP):
    pass

class CANBUS:
  pt = 0
  body = 1  # aux bus — carries Delphi ESR 2.5 radar (0x500..0x53F) and CEM dynamics
  cam = 2


# Delphi ESR 2.5 on aux bus (0x500–0x53F, 20Hz, 64 tracks); read directly so planner sees leads independently of stock FSM
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
  minSteerSpeed: float = 1.0


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
