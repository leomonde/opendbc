from opendbc.car.volvo.values import CarControllerParams, LongMode


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


def create_longitudinal(packer, stock_fsm3, accel, acc_check, mode: LongMode | None = None):
  # pass stock FSM3 verbatim except ACC_AccelerationRequest and ACC_Check; bit flip faults ECU (drive 27).
  # With mode (OP long), also own Byte_2 bits[1:0] (LongMode) and Byte_3 (traction). Byte_2 upper bits
  # stay stock so the data (0xD4) / type (0x00) frame cadence of the camera is kept.
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
  return packer.make_can_msg("FSM3", 0, values)


def fsm4_brake_byte(accel: float, mode: LongMode) -> int:
  # FSM4 Byte_5: 0xF0 | level when braking (bit6 = hydraulic brake request), 0xB0 | level otherwise
  CCP = CarControllerParams
  brake = mode == LongMode.BRAKE
  levels, default = (CCP.FSM4_BRAKE_LEVELS, 0x4) if brake else (CCP.FSM4_NO_BRAKE_LEVELS, 0x5)
  level = next((lvl for limit, lvl in levels if accel < limit), default)
  return (0xF0 if brake else 0xB0) | level


def create_fsm4(packer, stock_fsm4, accel: float, mode: LongMode | None = None):
  # relay one stock FSM4 per camera frame (Byte_0 alternates 0x55/0xAA in runs of 3-4, so no duplicates).
  # With mode (OP long), own Byte_5 so the ECM's hydraulic brake follows OP and not the camera.
  values = {s: stock_fsm4[s] for s in ("Byte_0", "Byte_1", "Byte_2", "ACC_LeadSpeed", "Byte_4", "Byte_5", "Byte_6", "Byte_7")}
  if mode is not None:
    values["Byte_5"] = fsm4_brake_byte(accel, mode)
  return packer.make_can_msg("FSM4", 0, values)


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
