from dataclasses import dataclass, field
from enum import IntEnum

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds, DT_CTRL
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
  """FSM3 Byte_2 bits[1:0], longitudinal mode chosen by the stock camera.
  ECM applies hydraulic brake only in BRAKE (routes 807/808: 0 brake frames in COAST/DRIVE)."""
  BRAKE = 0
  COAST = 1
  DRIVE = 2


class EcmAccState(IntEnum):
  """EngineData Byte_4 bits 5:4, ACC state in the ECM (stock route 808: 1 with the camera's ACC off, 2 standby, 3 engaged).
  Pressing On/Off in standby goes to OFF, so bit 4 alone (OFF and ENGAGED) is not "engaged" (route 823)."""
  OFF = 1
  STANDBY = 2
  ENGAGED = 3


class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(90,
    ([0., 5., 15.], [5., .8, .15]),
    ([0., 5., 15.], [5., 3.5, 0.4]),
  )

  STEER_TIMEOUT = 30 / DT_CTRL  # frames before steer fault on sustained 0-torque from EPS
  BLOCK_LEN = 8   # EUCD: frames to block steering on direction change (servo ignores otherwise)
  DEADZONE = 0.2  # deg: hold previous direction inside deadzone to avoid unwind
  BUTTON_BURST = 15  # CAN messages per button press to increase ECU acceptance probability

  ACCEL_MIN = -4.0  # m/s^2
  ACCEL_MAX = 2.0   # m/s^2

  # Keep the ECM engaged with OP's FSM0/FSM3/FSM4 after the camera aborts ACC. Off: in route 822 it held the ECM
  # engaged for 38 s with the camera's ACC off; after the ECM disengaged the camera latched a fault (FSM0 Byte_3 0x04,
  # ACC unavailable) for the rest of the drive, with OP off too.
  OP_OWNS_ACC_AFTER_CAMERA_ABORT = False

  # LongMode hysteresis on OP accel, tuned on stock camera route 808 (87% mode agreement).
  # COAST alone reaches ~-0.3 m/s^2 at -0.45 request (engine braking), so BRAKE starts below that.
  BRAKE_ENTER = -0.50  # m/s^2
  BRAKE_EXIT = -0.15
  DRIVE_ENTER = 0.08
  DRIVE_EXIT = 0.04

  # FSM4 Byte_5/Byte_6 = 16-bit value: bit15 always set, bit14 = hydraulic brake request (ECM brakes only with it,
  # route 80e), bits 13..0 = decel target. Mean 14-bit value vs accel from the stock camera (808); stock changes it by
  # ~1 count per frame, so it must come from OP's accel, not mixed with the camera's Byte_6 (routes 812/814: ACC fault).
  FSM4_BRAKE_ACCEL_BP = [-3.5, -2.5, -1.75, -1.25, -0.8, -0.45, -0.2, 0.0, 0.2]
  FSM4_BRAKE_VALUE_V = [12519, 12850, 12964, 13073, 13143, 13227, 13279, 13328, 13378]
  # Brake response the camera sees (car -> camera relay): the ECM/ABS answer a FSM4 brake request in 0x323 Byte_4 bit4,
  # 0x133 Byte_1, 0x220 Byte_2 bit2 and 0x299 Byte_5 bit3 (routes 812/814/815/816, even without pressure; in stock 808
  # 0x299 0xF7 only ever follows the camera's own request, 30-180 ms later); seeing that without having asked, the camera aborts
  # ACC ~1.5 s later (FSM0 Byte_3 0x10). Masked while OP brakes and the camera does not, and this long after.
  CAM_BRAKE_MASK_HOLD_FRAMES = int(1.0 / DT_CTRL)
  # Which of those are masked. In stock 808, 0x299 and 0x220 appear only with the camera's own request (0% with driver
  # braking): they are the ack. 0x323 and 0x133 also appear with driver braking: brake applied. Route 816 (0x133+0x323
  # only) still aborted; 821-824 (all four) did not. Narrow it one drive at a time (0x299+0x220, then 0x299).
  CAM_BRAKE_MASK_MSGS = frozenset({0x133, 0x323, 0x220, 0x299})
  FSM4_NO_BRAKE_ACCEL_BP = [-0.45, -0.2, 0.0, 0.2, 0.45, 0.8, 1.5]
  FSM4_NO_BRAKE_VALUE_V = [13276, 13299, 13318, 13338, 13333, 13377, 13449]

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
