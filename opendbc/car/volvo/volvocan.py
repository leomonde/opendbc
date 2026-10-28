import numpy as np

from opendbc.car.volvo.values import CANBUS, CarControllerParams, LongMode


def create_button_msg(packer, resume=False, cancel=False, set_plus=False, minus=False, bus=0):
  # TODO: validate
  msg = {
    "ACCOnOffBtn": cancel,
    "ACCOnOffBtnInv": not cancel,
    "ACCResumeBtn": resume,
    "ACCResumeBtnInv": not resume,
    "ACCSetBtn": set_plus,
    "ACCSetBtnInv": not set_plus,
    "ACCMinusBtn": minus,
    "ACCMinusBtnInv": not minus,
  }
  return packer.make_can_msg("CCButtons", bus, msg)


def create_lkas_state_msg(packer, steering_angle: float, stock_values: dict):
  # Servo state to the camera, LKA torque/active cleared
  msg = {
    "LKATorque": 0,
    "SteeringAngleServo": steering_angle,
    "byte0": stock_values["byte0"],
    "byte4": stock_values["byte4"],
    "byte7": stock_values["byte7"],
    "LKAActive": int(stock_values["LKAActive"]) & 0xF5,
    "EPSTorque": stock_values["EPSTorque"],
  }
  return packer.make_can_msg("PSCM1", 2, msg)


def calculate_lka_checksum(dat: bytearray) -> int:
  steer_angle_request = ((dat[3] & 0x3F) << 8) + dat[4]
  steering_direction_request = dat[5] & 0x03
  trqlim = dat[2]

  s = (trqlim + steering_direction_request + steer_angle_request + (steer_angle_request >> 8)) & 0xFF
  return s ^ 0xFF


def create_lka_msg(packer, apply_steer: float, steer_direction: int):
  values = {
    "LKAAngleReq": apply_steer,
    "LKASteerDirection": steer_direction,
    "TrqLim": 0,

    "SET_X_22": 0x25,
    "SET_X_02": 0,
    "SET_X_10": 0x10,
    "SET_X_A4": 0xa7,
  }

  # calculate checksum
  dat = packer.make_can_msg("FSM2", 0, values)[1]
  values["Checksum"] = calculate_lka_checksum(dat)

  return packer.make_can_msg("FSM2", 0, values)


def next_long_mode(mode: LongMode, accel: float, brake_offset: float = 0.) -> LongMode:
  # brake_offset shifts the BRAKE thresholds by the slope
  CCP = CarControllerParams
  if mode == LongMode.BRAKE:
    if accel <= CCP.BRAKE_EXIT + brake_offset:
      return LongMode.BRAKE
  elif mode == LongMode.DRIVE:
    if accel >= CCP.DRIVE_EXIT:
      return LongMode.DRIVE
  enter_offset = brake_offset * (1. + CCP.UPHILL_BRAKE_ENTER_EXTRA) if brake_offset < 0. else brake_offset
  if accel < CCP.BRAKE_ENTER + enter_offset:
    return LongMode.BRAKE
  if accel > CCP.DRIVE_ENTER:
    return LongMode.DRIVE
  return LongMode.COAST


def fsm3_traction(accel: float, mode: LongMode) -> int:
  # FSM3 ACC_TractionRequest
  if mode == LongMode.BRAKE:
    return 0
  if mode == LongMode.DRIVE:
    gain, offset = CarControllerParams.TRACTION_DRIVE
    return min(max(round(gain * accel + offset), 0), 255)
  gain, offset = CarControllerParams.TRACTION_COAST
  return min(max(round(gain * accel + offset), -127), -1) & 0xFF


def _copy(stock, skip=()):
  # Stock frame signals, without the ones overlapping others
  return {k: v for k, v in stock.items() if k not in skip}


def create_longitudinal(packer, stock_fsm3, accel, acc_check, mode: LongMode | None = None):
  # Camera FSM3 with OP's accel, ACC_Check, mode and traction
  values = _copy(stock_fsm3)
  values |= {
    "ACC_AccelerationRequest": accel,
    "ACC_Check": acc_check,
  }
  if mode is not None:
    values["ACC_Mode"] = int(mode)
    values["ACC_TractionRequest"] = fsm3_traction(accel, mode)
  return packer.make_can_msg("FSM3", 0, values)


def fsm4_brake_value(accel: float, brake: bool) -> int:
  # FSM4 brake word: 0x8000 | brake flag 0x4000 | 14-bit target
  CCP = CarControllerParams
  bp, v = (CCP.FSM4_BRAKE_ACCEL_BP, CCP.FSM4_BRAKE_VALUE_V) if brake else (CCP.FSM4_NO_BRAKE_ACCEL_BP, CCP.FSM4_NO_BRAKE_VALUE_V)
  return 0x8000 | (0x4000 if brake else 0) | int(round(float(np.interp(accel, bp, v))))


def create_fsm4(packer, stock_fsm4, accel: float, mode: LongMode | None = None, cam_weight: float = 0.):
  # Camera FSM4; brake flag and target follow OP's accel when either side brakes
  values = _copy(stock_fsm4)
  op_brake = mode == LongMode.BRAKE
  cam_brake = bool(stock_fsm4["ACC_BrakeRequest"])
  if mode is not None and (op_brake or cam_brake):
    value = fsm4_brake_value(accel, op_brake)
    if cam_weight > 0. and op_brake and cam_brake:
      # Blend into the camera's target during its stop
      cam_value = int(stock_fsm4["ACC_BrakeTarget"])
      blended = int(round((value & 0x3FFF) + cam_weight * (cam_value - (value & 0x3FFF))))
      value = (value & 0xC000) | blended
    values["Byte_5_b7"] = (value >> 15) & 1
    values["ACC_BrakeRequest"] = (value >> 14) & 1
    values["ACC_BrakeTarget"] = value & 0x3FFF
  return packer.make_can_msg("FSM4", 0, values)


def create_fsm0(packer, stock_fsm0, ecm_engaged: bool):
  # Camera FSM0, never ACC enabled without the ECM engaged
  values = _copy(stock_fsm0, skip=("ACC_BrakeAlert",))
  if not ecm_engaged:
    values["ACC_Enabled"] = 0
  return packer.make_can_msg("FSM0", 0, values)


def create_cam_brake_status(packer, stock_brake_status, mask: bool):
  # BrakeStatus to the camera; masked clears brake applied
  values = _copy(stock_brake_status)
  if mask:
    values["BrakeApplied"] = 0
  return packer.make_can_msg("BrakeStatus", CANBUS.cam, values)


def create_cam_brake_level(packer, stock_brake_level, frozen_level: int | None):
  # BrakeLevel to the camera; masked holds the brake level
  values = _copy(stock_brake_level)
  if frozen_level is not None:
    values["BrakeLevel"] = frozen_level & 0x7F
  return packer.make_can_msg("BrakeLevel", CANBUS.cam, values)


def create_cam_wheel_status(packer, stock_wheelspeed1, mask: bool, spoof: bool = False):
  # wheelspeed1 to the camera; brake request bit masked or spoofed
  values = _copy(stock_wheelspeed1, skip=("WhlSpdRR", "WhlSpdLR"))
  if mask:
    values["BrakeReqActive"] = 0
  elif spoof:
    values["BrakeReqActive"] = 1
  return packer.make_can_msg("wheelspeed1", CANBUS.cam, values)


def create_cam_brake_request_ack(packer, stock_brake_req_ack, mask: bool, spoof: bool = False):
  # BrakeReqAck to the camera; brake ack (active low) masked or spoofed
  values = _copy(stock_brake_req_ack)
  if mask:
    values["BrakeReqNotAck"] = 1
  elif spoof:
    values["BrakeReqNotAck"] = 0
  return packer.make_can_msg("BrakeReqAck", CANBUS.cam, values)


def create_radar(packer, stock_fsm1, mode: LongMode | None = None):
  # Camera FSM1; ACC_Braking follows OP braking
  values = _copy(stock_fsm1)
  if mode is not None:
    values["ACC_Braking"] = 1 if mode == LongMode.BRAKE else 0
  return packer.make_can_msg("FSM1", 0, values)
