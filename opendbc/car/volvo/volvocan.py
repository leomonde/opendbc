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
  # servo state to the camera, LKA torque/active cleared
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
  # FSM3 Byte_3 traction request (fit to the stock camera)
  if mode == LongMode.BRAKE:
    return 0
  if mode == LongMode.DRIVE:
    return min(max(round(89 * accel - 4), 0), 165)
  return min(max(round(57 * accel - 15), -127), -1) & 0xFF


def create_longitudinal(packer, stock_fsm3, accel, acc_check, mode: LongMode | None = None, good_fsm3=None,
                        hide_no_target_cancel: bool = False, standstill: bool = False):
  # Camera FSM3 with OP's accel and ACC_Check; with mode, also OP's mode (Byte_2 bits 1:0) and traction.
  # good_fsm3: the camera's last good frame, held toward the ECM.
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
  if hide_no_target_cancel:
    # Byte_2 bit5: the camera's cancel when slow with no target
    values["Byte_2"] = int(values["Byte_2"]) & ~0x20
  if good_fsm3 is not None:
    values |= {s: good_fsm3[s] for s in ("ACC_Standstill", "Byte_01", "Byte_02")}
    base = 0xD4 if int(stock_fsm3["Byte_4"]) == 0x81 else 0x00  # data / type frame
    values["Byte_2"] = base | (int(mode) if mode is not None else int(good_fsm3["Byte_2"]) & 0x03)
    values["Byte_6"] = 0
  if standstill:
    values["ACC_Standstill"] = 1
  return packer.make_can_msg("FSM3", 0, values)


def fsm4_brake_value(accel: float, brake: bool) -> int:
  # FSM4 Byte_5/Byte_6: 0x8000 | brake flag 0x4000 | 14-bit target
  CCP = CarControllerParams
  bp, v = (CCP.FSM4_BRAKE_ACCEL_BP, CCP.FSM4_BRAKE_VALUE_V) if brake else (CCP.FSM4_NO_BRAKE_ACCEL_BP, CCP.FSM4_NO_BRAKE_VALUE_V)
  return 0x8000 | (0x4000 if brake else 0) | int(round(float(np.interp(accel, bp, v))))


def create_fsm4(packer, stock_fsm4, accel: float, mode: LongMode | None = None, good_byte2: int | None = None,
                stopping: bool = False, cam_weight: float = 0.):
  # Camera FSM4 relayed per frame; with mode, the brake flag and target follow OP's accel when either side brakes.
  values = {s: stock_fsm4[s] for s in ("Byte_0", "Byte_1", "Byte_2", "ACC_LeadSpeed", "Byte_4", "Byte_5", "Byte_6", "Byte_7")}
  op_brake = mode == LongMode.BRAKE
  cam_brake = bool(int(stock_fsm4["Byte_5"]) & 0x40)
  if mode is not None and (op_brake or cam_brake):
    value = fsm4_brake_value(accel, op_brake)
    if cam_weight > 0. and op_brake and cam_brake:
      # blend into the camera's target while handing it the stop
      cam_value = ((int(stock_fsm4["Byte_5"]) << 8) | int(stock_fsm4["Byte_6"])) & 0x3FFF
      blended = int(round((value & 0x3FFF) + cam_weight * (cam_value - (value & 0x3FFF))))
      value = (value & 0xC000) | blended
    values["Byte_5"] = value >> 8
    values["Byte_6"] = value & 0xFF
  if good_byte2 is not None:
    values["Byte_2"] = good_byte2
  if stopping:
    # Byte_1 bit3: "stopping" flag
    values["Byte_1"] = int(stock_fsm4["Byte_1"]) | 0x08
  return packer.make_can_msg("FSM4", 0, values)


def create_fsm0(packer, stock_fsm0, ecm_engaged: bool, good_fsm0=None):
  # Camera FSM0 relayed per frame, never ACC enabled without the ECM engaged.
  # good_fsm0: the camera's last good state, held toward the ECM (lead bit stays live).
  values = {s: stock_fsm0[s] for s in BYTES}
  if good_fsm0 is not None:
    values["Byte_2"] = (int(good_fsm0["Byte_2"]) & ~0x01) | (int(stock_fsm0["Byte_2"]) & 0x01)
    values["Byte_3"] = good_fsm0["Byte_3"]
    values["Byte_7"] = int(stock_fsm0["Byte_7"]) & ~0x04
  if not ecm_engaged:
    values["Byte_2"] = int(values["Byte_2"]) & ~0x04
  return packer.make_can_msg("FSM0", 0, values)


BYTES = tuple(f"Byte_{i}" for i in range(8))


def create_cam_brake_status(packer, stock_323, mask: bool):
  # 0x323 to the camera; masked: Byte_4 bit4 (brake applied) cleared
  values = {s: stock_323[s] for s in BYTES}
  if mask:
    values["Byte_4"] = int(stock_323["Byte_4"]) & ~0x10
  return packer.make_can_msg("NEW_MSG_323", CANBUS.cam, values)


def create_cam_brake_level(packer, stock_133, frozen_level: int | None):
  # 0x133 to the camera; masked: Byte_1 bits 6:0 (brake level) held
  values = {s: stock_133[s] for s in BYTES}
  if frozen_level is not None:
    values["Byte_1"] = (int(stock_133["Byte_1"]) & 0x80) | (frozen_level & 0x7F)
  return packer.make_can_msg("NEW_MSG_133", CANBUS.cam, values)


def create_cam_wheel_status(packer, stock_220, mask: bool, spoof: bool = False, min_speed_kph: float | None = None):
  # 0x220 to the camera; Byte_2 bit2 (brake request active): masked clears it, spoofed sets it
  values = {s: stock_220[s] for s in BYTES}
  if mask:
    values["Byte_2"] = int(stock_220["Byte_2"]) & ~0x04
  elif spoof:
    values["Byte_2"] = int(stock_220["Byte_2"]) | 0x04
  if min_speed_kph is not None:
    # rear wheel speeds (Byte_4/5, Byte_6/7): 0.01 km/h, offset -327.68
    floor = round((min_speed_kph + 327.68) / 0.01)
    for hi, lo in (("Byte_4", "Byte_5"), ("Byte_6", "Byte_7")):
      raw = max((int(values[hi]) << 8) | int(values[lo]), floor)
      values[hi], values[lo] = raw >> 8, raw & 0xFF
  return packer.make_can_msg("wheelspeed1", CANBUS.cam, values)


def create_cam_vehicle_speed(packer, stock_148, min_speed_kph: float | None = None):
  # 0x148 to the camera; min_speed_kph raises VehicleSpeed (Byte_6/7, 0.01 km/h)
  values = {s: stock_148[s] for s in BYTES}
  if min_speed_kph is not None:
    raw = max((int(values["Byte_6"]) << 8) | int(values["Byte_7"]), round(min_speed_kph / 0.01))
    values["Byte_6"], values["Byte_7"] = raw >> 8, raw & 0xFF
  return packer.make_can_msg("VehicleSpeed1", CANBUS.cam, values)


def create_cam_brake_request_ack(packer, stock_299, mask: bool, spoof: bool = False):
  # 0x299 to the camera; Byte_5 bit3 (brake ack, active low): masked sets it, spoofed clears it
  values = {s: stock_299[s] for s in BYTES}
  if mask:
    values["Byte_5"] = int(stock_299["Byte_5"]) | 0x08
  elif spoof:
    values["Byte_5"] = int(stock_299["Byte_5"]) & ~0x08
  return packer.make_can_msg("NEW_MSG_299", CANBUS.cam, values)


def create_cam_engine_data(packer, stock_12a, acc_state: int | None = None):
  # 0x12A to the camera; acc_state replaces the ECM's ACC state (Byte_4 bits 5:4)
  values = {s: stock_12a[s] for s in BYTES}
  if acc_state is not None:
    values["Byte_4"] = (int(stock_12a["Byte_4"]) & ~0x30) | ((acc_state & 0x3) << 4)
  return packer.make_can_msg("EngineData", CANBUS.cam, values)


CAM_BUTTON_BITS = {"onoff": 0x08, "resume": 0x20}  # Byte_7 bit set, Byte_5 bit cleared (inverted copy)


def create_cam_buttons(packer, stock_127, press: str | None = None):
  # 0x127 to the camera; press adds a button press (On/Off or Resume) to the driver's
  values = {s: stock_127[s] for s in BYTES}
  if press is not None:
    bit = CAM_BUTTON_BITS[press]
    values["Byte_7"] = int(stock_127["Byte_7"]) | bit
    values["Byte_5"] = int(stock_127["Byte_5"]) & ~bit
  return packer.make_can_msg("CCButtons", CANBUS.cam, values)


def create_radar(packer, stock_fsm1, mode: LongMode | None = None):
  # Camera FSM1 relayed; with mode, Byte_2 bit2 follows OP braking
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
