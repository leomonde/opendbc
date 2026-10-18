from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.volvo.carcontroller import CarController
from opendbc.car.volvo.carstate import CarState
from opendbc.car.volvo.radar_interface import RadarInterface
from opendbc.car.volvo.values import DBC

#ButtonType = car.CarState.ButtonEvent.Type
#EventName = car.CarEvent.EventName

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  # Without this binding, CarInterfaceBase uses the default RadarInterfaceBase stub
  # which returns empty RadarData — liveTracks.points would stay at 0.
  RadarInterface = RadarInterface

  @staticmethod
  #def _get_params(ret, candidate: CAR, fingerprint, car_fw, experimental_long, docs):
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "volvo"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.volvo)]
    # ret.dashcamOnly = True

    # When Bus.radar is in DBC, RadarInterface decodes the Delphi ESR 2.5 directly
    # (64 tracks at 20Hz) instead of relying on stock FSM's single-lead ACC_Distance.
    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 0.8

    # OP long: stock ACC still engages and owns the set speed (pcmCruise); OP replaces FSM3 accel/mode/traction
    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long
    if ret.openpilotLongitudinalControl:
      ret.longitudinalActuatorDelay = 0.5  # FSM3 accel request -> aEgo, ~0.6s in stock routes 807/808
      # the car delivers ~70% of the requested decel (route 821: -1.1 asked, -0.8 measured) and stopped 1.7 m behind
      # the lead; integral on the accel error closes that gap (no proportional term, the ECM has its own loop)
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.5]

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
 car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True
    return ret
