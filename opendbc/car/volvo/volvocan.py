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
  # zero LKATorque/LKAActive to prevent stock LKA interference
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


def next_long_mode(mode: LongMode, accel: float) -> LongMode:
  CCP = CarControllerParams
  if mode == LongMode.BRAKE:
    if accel <= CCP.BRAKE_EXIT:
      return LongMode.BRAKE
  elif mode == LongMode.DRIVE:
    if accel >= CCP.DRIVE_EXIT:
      return LongMode.DRIVE
  if accel < CCP.BRAKE_ENTER:
    return LongMode.BRAKE
  if accel > CCP.DRIVE_ENTER:
    return LongMode.DRIVE
  return LongMode.COAST


def fsm3_traction(accel: float, mode: LongMode) -> int:
  # FSM3 Byte_3: traction request fit from stock camera (808): 0 in BRAKE,
  # 89*a-4 in DRIVE (R2 0.88, stock range 0..165), signed 57*a-15 in COAST
  if mode == LongMode.BRAKE:
    return 0
  if mode == LongMode.DRIVE:
    return min(max(round(89 * accel - 4), 0), 165)
  return min(max(round(57 * accel - 15), -127), -1) & 0xFF


def create_longitudinal(packer, stock_fsm3, accel, acc_check, mode: LongMode | None = None, good_fsm3=None):
  # pass stock FSM3 verbatim except ACC_AccelerationRequest and ACC_Check; bit flip faults ECU (drive 27).
  # With mode (OP long), also own Byte_2 bits[1:0] (LongMode) and Byte_3 (traction). Byte_2 upper bits
  # stay stock so the data (0xD4) / type (0x00) frame cadence of the camera is kept.
  # With good_fsm3 (OP owns ACC after the camera's abort), the camera's abort marks (Byte_0 0xAB, Byte_2 0xFC,
  # Byte_6 0x01) are replaced by the last frame before the abort.
  values = {s: stock_fsm3[s] for s in (
    "ACC_Standstill",
    "Byte_01",
    "Byte_02",
    "Byte_2",
    "Byte_3",
    "Byte_4",
    "Byte_5",
    "Byte_6",
    "Byte_7",
  )}
  values |= {
    "ACC_AccelerationRequest": accel,
    "ACC_Check": acc_check,
  }
  if mode is not None:
    values["Byte_2"] = (int(stock_fsm3["Byte_2"]) & 0xFC) | int(mode)
    values["Byte_3"] = fsm3_traction(accel, mode)
  if good_fsm3 is not None:
    values |= {s: good_fsm3[s] for s in ("ACC_Standstill", "Byte_01", "Byte_02")}
    base = 0xD4 if int(stock_fsm3["Byte_4"]) == 0x81 else 0x00  # data / type frame
    values["Byte_2"] = base | (int(mode) if mode is not None else int(good_fsm3["Byte_2"]) & 0x03)
    values["Byte_6"] = 0
  return packer.make_can_msg("FSM3", 0, values)


def fsm4_brake_value(accel: float, brake: bool) -> int:
  # FSM4 Byte_5/Byte_6 16-bit value for OP's accel: 0x8000 | (0x4000 if brake) | 14-bit decel target
  CCP = CarControllerParams
  bp, v = (CCP.FSM4_BRAKE_ACCEL_BP, CCP.FSM4_BRAKE_VALUE_V) if brake else (CCP.FSM4_NO_BRAKE_ACCEL_BP, CCP.FSM4_NO_BRAKE_VALUE_V)
  return 0x8000 | (0x4000 if brake else 0) | int(round(float(np.interp(accel, bp, v))))


def create_fsm4(packer, stock_fsm4, accel: float, mode: LongMode | None = None, good_byte2: int | None = None):
  # relay one stock FSM4 per camera frame (Byte_0 alternates 0x55/0xAA in runs of 3-4, so no duplicates).
  # With mode (OP long), the ECM's hydraulic brake follows OP: Byte_5/Byte_6 come from OP's accel when OP brakes,
  # or when the camera brakes and OP does not; otherwise they stay the camera's.
  values = {s: stock_fsm4[s] for s in ("Byte_0", "Byte_1", "Byte_2", "ACC_LeadSpeed", "Byte_4", "Byte_5", "Byte_6", "Byte_7")}
  op_brake = mode == LongMode.BRAKE
  cam_brake = bool(int(stock_fsm4["Byte_5"]) & 0x40)
  if mode is not None and (op_brake or cam_brake):
    value = fsm4_brake_value(accel, op_brake)
    values["Byte_5"] = value >> 8
    values["Byte_6"] = value & 0xFF
  if good_byte2 is not None:  # OP owns ACC after the camera's abort: its Byte_2 goes to 0xFC
    values["Byte_2"] = good_byte2
  return packer.make_can_msg("FSM4", 0, values)


def create_fsm0(packer, stock_fsm0, ecm_engaged: bool, good_fsm0=None):
  # relay one camera FSM0 per frame (panda always blocks the camera's). The ECM is the ACC master (it leads FSM0 by
  # 35-60 ms in stock routes); FSM0 never reports ACC enabled when the ECM is not engaged (enforced by the safety too).
  # With good_fsm0, OP owns ACC (OP long engaged, camera's ACC function aborted): keep the state before the abort
  # instead of the camera's (Byte_2 0x86/0x87 -> 0x82, Byte_3 0x10, Byte_7 bit2).
  values = {s: stock_fsm0[s] for s in BYTES}
  if good_fsm0 is not None:
    values["Byte_2"] = (int(good_fsm0["Byte_2"]) & ~0x01) | (int(stock_fsm0["Byte_2"]) & 0x01)  # FrontCar stays live
    values["Byte_3"] = good_fsm0["Byte_3"]
    values["Byte_7"] = int(stock_fsm0["Byte_7"]) & ~0x04
  if not ecm_engaged:
    values["Byte_2"] = int(values["Byte_2"]) & ~0x04
  return packer.make_can_msg("FSM0", 0, values)


BYTES = tuple(f"Byte_{i}" for i in range(8))


def create_cam_brake_status(packer, stock_323, mask: bool):
  # relay 0x323 to the camera; masked: Byte_4 bit4 (ECM brake request acknowledge, 0xC5 -> 0xD5) cleared
  values = {s: stock_323[s] for s in BYTES}
  if mask:
    values["Byte_4"] = int(stock_323["Byte_4"]) & ~0x10
  return packer.make_can_msg("NEW_MSG_323", CANBUS.cam, values)


def create_cam_brake_level(packer, stock_133, frozen_level: int | None):
  # relay 0x133 to the camera; masked: Byte_1 bits 6..0 (ABS brake level) held at the value before OP braked
  values = {s: stock_133[s] for s in BYTES}
  if frozen_level is not None:
    values["Byte_1"] = (int(stock_133["Byte_1"]) & 0x80) | (frozen_level & 0x7F)
  return packer.make_can_msg("NEW_MSG_133", CANBUS.cam, values)


def create_cam_wheel_status(packer, stock_220, mask: bool):
  # relay 0x220 (wheelspeed1) to the camera; masked: Byte_2 bit2 (brake request active, 0xE0 -> 0xE4) cleared
  values = {s: stock_220[s] for s in BYTES}
  if mask:
    values["Byte_2"] = int(stock_220["Byte_2"]) & ~0x04
  return packer.make_can_msg("wheelspeed1", CANBUS.cam, values)


def create_cam_brake_request_ack(packer, stock_299, mask: bool):
  # relay 0x299 to the camera; masked: Byte_5 bit3 (brake request acknowledge, active low: 0xFF -> 0xF7) set again
  values = {s: stock_299[s] for s in BYTES}
  if mask:
    values["Byte_5"] = int(stock_299["Byte_5"]) | 0x08
  return packer.make_can_msg("NEW_MSG_299", CANBUS.cam, values)


def create_radar(packer, stock_fsm1, mode: LongMode | None = None):
  # pass stock FSM1 verbatim; spoofing ACC_Distance=255 caused stock ACC to disengage below 30 km/h.
  # With mode (OP long), Byte_2 bit2 follows OP braking (camera sets it in BRAKE: 0xB8->0xBC data, 0x00->0x04 type).
  values = {s: stock_fsm1[s] for s in (
    "ACC_Distance",
    "Byte_1",
    "Byte_2",
    "Byte_3",
    "Byte_4",
    "Byte_5",
    "Byte_6",
    "Byte_7",
  )}
  if mode is not None:
    values["Byte_2"] = (int(stock_fsm1["Byte_2"]) & ~0x04) | (0x04 if mode == LongMode.BRAKE else 0)
  return packer.make_can_msg("FSM1", 0, values)
