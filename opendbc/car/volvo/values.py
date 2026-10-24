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
  CAM_ACK_RELEASE_VALUE = 13280  # spoofed 0x299 ack drops when the camera's FSM4 target reaches this (no braking)

  # Camera brake requests: OP ignores mild ones (spoofed ack), follows them at once below this, latched while it brakes
  CAMERA_BRAKE_HONOR_BELOW = -2.0  # m/s^2

  # The camera aborts (0x10) after ~3-6 s of its FSM3 mode 0 while the car gets another mode (routes 821-854),
  # so FSM3 shows mode 0 whenever the camera does (mode bits only: FSM4 decides the hydraulic brake).

  # BRAKE mode thresholds follow what coasting gives on the slope: lower uphill (route 858 braked on a 5° slope),
  # higher downhill (route 85a coasted 3-7 km/h over the limit on a 1.5° descent)
  COAST_PITCH_GAIN = 5.65  # m/s^2 per sin(pitch), openpilot's coast fit
  DOWNHILL_BRAKE_OFFSET_MAX = 0.15  # m/s^2, keeps BRAKE exit (-0.15 + this) at or below 0
  # Uphill, BRAKE is entered this much further below coast: OP's request overshoots the coast cap a little
  # (route 85b, 5° climb: -1.1 vs -0.8 coast, hydraulic brake jerks)
  UPHILL_BRAKE_ENTER_EXTRA = 0.6  # fraction of the uphill offset

  # Camera final stop behind a lead
  CAMERA_STOPS_BEHIND_LEAD = True
  CAM_STOP_HANDOVER_SPEED = 10. * CV.KPH_TO_MS  # camera takes over below this, with a target
  CAM_STOP_RETURN_SPEED = 15. * CV.KPH_TO_MS    # OP takes back above this
  CAM_STOP_TRANSITION_TIME = 1.0  # s, blend between OP and camera
  OP_BRAKE_OVERRIDE_ACCEL = -0.3  # m/s^2, OP brakes if it wants less than this and the camera does not brake
  OP_BRAKE_OVERRIDE_MARGIN = 1.0  # m/s^2, OP brakes if it wants this much more than the camera

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
