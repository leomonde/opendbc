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
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(90,
    ([0., 5., 15.], [5., .8, .15]),
    ([0., 5., 15.], [5., 3.5, 0.4]),
  )
  STEER_TIMEOUT = 30 / DT_CTRL  # frames of 0 torque before the EPS faults
  BLOCK_LEN = 8   # frames to block steering on a direction change
  DEADZONE = 0.2  # deg, keep the previous direction inside it
  # Resume held this long on copies of the CEM's CCButtons: a one-shot burst was lost amid its frames (route 89e)
  RESUME_HOLD_FRAMES = int(0.3 / DT_CTRL)
  # s of planner resume before SNG: 0.2 let a red-light flicker through, the ECM drove off on its own (route 8a4)
  SNG_OP_RESUME_TIME = 0.5
  RESUME_BURST_INTERVAL = 1.0  # s between resume bursts at a standstill
  RESUME_BURST_MAX = 5  # bursts per standstill
  SNG_ACK_TIME = 0.5  # s of FSM3 ACC_Check with the first burst, as the camera's ack (~300 ms after engaging)
  MIN_STEER_SPEED = 0.1  # m/s, no steering nor cancel button below this
  BUTTON_BURST = 15  # frames per ICBM button press

  # OP's stop to 0 (route 807), with OP owning the ECM
  STOPPING_FLAG_SPEED = 2.6 * CV.KPH_TO_MS  # FSM4 b1 F1 -> F9 below this
  HOLD_RELEASE_FRAMES = int(0.05 / DT_CTRL)  # brake request released after the car's hold
  STANDSTILL_DELAY = 3.0  # s stopped before FSM3 ACC_Standstill
  HOLD_ACCEL = -0.08  # m/s^2, FSM3 accel while the car holds (raw 0x7C)
  STOPPING_ACCEL = -0.64  # m/s^2, at most while stopping, as the camera from its F9 to the car's hold (route 807)
  GO_RETRY_TIME = 2.0  # s still stopped after the resume burst -> standstill again, retry
  ECM_READY_TIMEOUT = 3.0  # s after FSM3 ACC_Standstill: resume sent even without the ECM's ready (seen 1.5 s)
  EXIT_STANDBY_MAX = 1.0  # s, OP's long ended with the ECM still engaged: camera kept in standby at most this long

  # Longitudinal: FSM1/FSM3 sent at 50 Hz, limits
  LONG_TX_PERIOD = 0.02  # s
  ACCEL_MIN = -3.5  # m/s^2, as the panda's
  ACCEL_MAX = 2.0   # m/s^2

  # FSM3 mode hysteresis on OP accel
  BRAKE_ENTER = -0.50  # m/s^2, at and above BRAKE_ENTER_BP[-1]
  BRAKE_EXIT = -0.15
  # Slower, BRAKE enters earlier like the camera's, as coasting there does not slow the car (routes 0a, 22, 24)
  BRAKE_ENTER_BP = [15. * CV.KPH_TO_MS, 40. * CV.KPH_TO_MS]
  BRAKE_ENTER_V = [-0.10, BRAKE_ENTER]
  BRAKE_HYSTERESIS = BRAKE_EXIT - BRAKE_ENTER  # exit this much above the enter, never above 0
  DRIVE_ENTER = 0.08
  DRIVE_EXIT = 0.04

  # FSM3 Byte_3 traction = gain * accel + offset, as the camera sends it
  TRACTION_COAST = (94., -9.)
  TRACTION_DRIVE = (97., -6.)

  # FSM4 14-bit brake target = zero + gain * accel, as the camera's own (248 segments, route 29); no-brake map below
  FSM4_BRAKE_ZERO = 13332
  FSM4_BRAKE_GAIN = 255.  # counts per m/s^2
  FSM4_BRAKE_ACCEL_RANGE = (ACCEL_MIN, 0.2)
  FSM4_NO_BRAKE_ACCEL_BP = [-0.45, -0.2, 0.0, 0.2, 0.45, 0.8, 1.5]
  FSM4_NO_BRAKE_VALUE_V = [13276, 13299, 13318, 13338, 13333, 13377, 13449]

  # BRAKE thresholds shifted by the slope's coast accel
  COAST_PITCH_GAIN = 5.65  # m/s^2 per sin(pitch)
  DOWNHILL_BRAKE_OFFSET_MAX = 0.15  # m/s^2, downhill shift cap
  # Extra uphill margin before BRAKE
  UPHILL_BRAKE_ENTER_EXTRA = 0.6  # fraction of the uphill offset

  # Accel rate limits: falling only against jumps (as Toyota), rising moderate (route 1f, 678 s)
  JERK_UP = 4.0     # m/s^3
  JERK_DOWN = 12.0  # m/s^3

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
    VolvoCarSpecs(wheelbase=2.776),
  )


VOLVO_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf1a2)
VOLVO_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf1a2)

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[0-9]{8} [A-Z]{2}\x00{13}",
  requests=[
    Request(
      [VOLVO_VERSION_REQUEST],
      [VOLVO_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
