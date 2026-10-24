import math

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs

try:
  from openpilot.common.params import Params
except ImportError:
  Params = None
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams, LongMode, SteerDirection
from opendbc.sunnypilot.car.volvo.icbm import IntelligentCruiseButtonManagementInterface

LongCtrlState = structs.CarControl.Actuators.LongControlState
CCP = CarControllerParams


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    # Lateral
    self.apply_steer_prev = 0
    self.apply_steer_dir_prev = SteerDirection.NONE
    self.latActive_prev = False
    self.steer_blocked = False
    self.steer_blocked_cnt = 0
    self.steer_dir_bf_block = SteerDirection.NONE

    # Custom ACC step for carstate, refreshed every 100 frames
    self._params = Params() if Params is not None else None
    self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1)) if self._params is not None else 1

    # Buttons: lead SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    self.op_resume_frames = 0
    self.sng_ack_frames = 0  # frames of FSM3 ACC_Check=1, so the ECM accepts OP's resume

    # FSM3/FSM1 sent at 50 Hz wall-clock
    self.next_long_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000

    # Longitudinal decision
    self.long_mode = LongMode.COAST
    self.accel = 0.0
    self.cam_brake = False
    self.cam_brake_honor = False

    # Camera final stop behind a lead
    self.cam_stop = False
    self.cam_stop_w = 0.
    self.cam_relay = False
    self.cam_weight = 0.

    # Brake acks to the camera
    self.spoof_cam_ack = False
    self.cam_brake_target = 0
    self.cam_mask_frames = 0
    self.brake_level_133 = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if CC.cruiseControl.cancel and CS.out.vEgo > self.CP.minSteerSpeed and not CS.out.brakePressed:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    if self.frame % 2 == 0:
      can_sends += self.lateral(CC, CS)

    self.resume_buttons(CC, CS, can_sends)
    self.relay_fsm0(CS, can_sends, CS.ecm_acc_engaged)

    if now_nanos >= self.next_long_tx_nanos:
      next_tx = self.next_long_tx_nanos + self.LONG_TX_PERIOD_NANOS
      if self.next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.LONG_TX_PERIOD_NANOS
      self.next_long_tx_nanos = next_tx
      can_sends += self.longitudinal(CC, CS)

    can_sends += self.relay_fsm4(CC, CS)
    can_sends += self.relay_brake_acks(CC, CS)

    if self.frame % 100 == 0 and self._params is not None:
      self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))
    CS._custom_acc_step = self._custom_acc_step

    icbm_sends = IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer_pt, self.frame, self.last_button_frame)
    if icbm_sends:
      CS._pending_delta = 0
      CS._icbm_suppress_frames = 25
    can_sends.extend(icbm_sends)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_steer_prev
    if CC.longActive:
      new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends

  # ---- Lateral ----

  def lateral(self, CC, CS):
    """Steering angle request (FSM2) and servo state to the camera (PSCM1), at 50 Hz."""
    actuators = CC.actuators
    if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
      apply_steer = apply_std_steer_angle_limits(
        actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw,
        CS.out.steeringAngleDeg, CC.latActive, CCP.ANGLE_LIMITS
      )
      apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

      error = CS.out.steeringAngleDeg - apply_steer
      error_with_deadzone = 0 if abs(error) < CCP.DEADZONE else error

      if not self.latActive_prev:
        self.apply_steer_dir_prev = apply_steer_dir

      # block direction flips until the servo catches up
      if self.steer_blocked:
        if (apply_steer_dir == self.steer_dir_bf_block) or (self.steer_blocked_cnt <= 0) or (error_with_deadzone == 0):
          self.steer_blocked = False
      else:
        if apply_steer_dir != self.apply_steer_dir_prev and error_with_deadzone != 0:
          self.steer_blocked = True
          self.steer_blocked_cnt = CCP.BLOCK_LEN
          self.steer_dir_bf_block = self.apply_steer_dir_prev

      if self.steer_blocked:
        self.steer_blocked_cnt -= 1
        apply_steer_dir = SteerDirection.NONE
      elif error_with_deadzone == 0:
        apply_steer_dir = self.apply_steer_dir_prev
    else:
      apply_steer = 0
      apply_steer_dir = SteerDirection.NONE

    self.apply_steer_prev = apply_steer
    self.apply_steer_dir_prev = apply_steer_dir
    self.latActive_prev = CC.latActive

    return [
      volvocan.create_lka_msg(self.packer_pt, apply_steer, int(apply_steer_dir)),
      # servo state to the camera, so the servo keeps accepting commands
      volvocan.create_lkas_state_msg(self.packer_pt, CS.out.steeringAngleDeg, CS.pscm_stock_values),
    ]

  # ---- Buttons ----

  def resume_buttons(self, CC, CS, can_sends):
    """Resume burst when the lead moves away (camera FSM1 distance or openpilot's resume request)."""
    at_standstill = CS.out.cruiseState.enabled and CS.out.cruiseState.standstill and CS.out.vEgo < 0.01

    # openpilot's own lead (radar + camera) left: its resume request, held for SNG_OP_RESUME_TIME
    self.op_resume_frames = self.op_resume_frames + 1 if CC.cruiseControl.resume else 0
    op_resume = self.op_resume_frames * DT_CTRL >= CCP.SNG_OP_RESUME_TIME

    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0
      if at_standstill and self.waiting and (CS.acc_distance > self.distance or op_resume):
        can_sends.extend(self.resume_burst())
        if self.sng_count == 0:
          self.sng_ack_frames = 25
        self.sng_count += 1
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

  def resume_burst(self):
    return [volvocan.create_button_msg(self.packer_pt, resume=True)] * self.CCP.BUTTON_BURST

  # ---- Camera -> ECM relays ----

  def relay_fsm0(self, CS, can_sends, ecm_engaged):
    """FSM0 relayed 1:1, never ACC enabled without the ECM engaged."""
    for stock_fsm0 in CS.stock_FSM0_frames:
      can_sends.append(volvocan.create_fsm0(self.packer_pt, stock_fsm0, ecm_engaged))

  def relay_fsm4(self, CC, CS):
    """FSM4 relayed 1:1; with OP long, OP sets the brake flag and target."""
    sends = []
    fsm4_mode = self.long_mode if (CC.longActive and not self.cam_relay) else None
    for stock_fsm4 in CS.stock_FSM4_frames:
      self.cam_brake = bool(int(stock_fsm4["Byte_5"]) & 0x40)
      sends.append(volvocan.create_fsm4(self.packer_pt, stock_fsm4, self.accel, fsm4_mode, cam_weight=self.cam_weight))
    return sends

  # ---- Longitudinal ----

  def longitudinal(self, CC, CS):
    """Accel and mode decision, FSM1/FSM3 sent at 50 Hz."""
    if CS.stock_FSM4_frames:
      self.cam_brake = bool(int(CS.stock_FSM4_frames[-1]["Byte_5"]) & 0x40)
    if CC.longActive:
      accel, long_mode = self.long_decision(CC, CS)
    else:
      accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
      self.long_mode = LongMode.COAST
      self.cam_brake_honor = False
      self.update_cam_stop(CC, CS)
      self.cam_relay = False
      self.cam_weight = 0.
      long_mode = None
    self.accel = accel

    # OP not braking on the camera's request: its ack is spoofed
    self.spoof_cam_ack = (CC.longActive and not self.cam_relay and self.cam_brake and not self.cam_brake_honor and
                          self.long_mode != LongMode.BRAKE)

    if self.sng_ack_frames > 0:
      acc_check = 1
      self.sng_ack_frames -= 1
    else:
      acc_check = int(CS.stock_FSM3["ACC_Check"])

    return [
      volvocan.create_radar(self.packer_pt, CS.stock_FSM1),
      volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, acc_check, self.fsm3_mode(CS, long_mode)),
    ]

  def long_decision(self, CC, CS):
    """OP long active: accel and mode from OP, the camera's brake request, or the camera's final stop."""
    actuators = CC.actuators
    accel = min(max(actuators.accel, CCP.ACCEL_MIN), CCP.ACCEL_MAX)
    cam_accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])

    self.update_cam_stop(CC, CS)
    op_brakes_more = (actuators.longControlState == LongCtrlState.pid and
                      ((not self.cam_brake and cam_accel >= 0. and accel < CCP.OP_BRAKE_OVERRIDE_ACCEL) or
                       accel < cam_accel - CCP.OP_BRAKE_OVERRIDE_MARGIN))
    w = 0. if op_brakes_more else self.cam_stop_w
    self.cam_relay = w >= 1.
    self.cam_weight = w
    self.update_cam_brake_honor(cam_accel, w > 0.)

    if self.cam_relay:
      # camera owns the stop: FSM3/FSM4 relayed untouched
      self.long_mode = LongMode(CS.cam_fsm3_mode) if CS.cam_fsm3_mode <= LongMode.DRIVE else LongMode.COAST
      return cam_accel, None

    if self.cam_brake_honor:
      accel = min(accel, cam_accel)
    if w > 0.:
      # blending into the camera's stop
      accel = accel + w * (cam_accel - accel)
    self.long_mode = LongMode.BRAKE if self.cam_brake_honor else volvocan.next_long_mode(self.long_mode, accel, self.slope_brake_offset(CC))
    return accel, self.long_mode

  # ---- Camera rules ----

  def update_cam_brake_honor(self, cam_accel, cam_stopping):
    """The camera's hydraulic brake is followed during its final stop, and at once below CAMERA_BRAKE_HONOR_BELOW,
    latched until it releases. Milder ones are ignored (ack spoofed). OP braking more is never limited."""
    if not self.cam_brake:
      self.cam_brake_honor = False
    elif cam_stopping or cam_accel < CCP.CAMERA_BRAKE_HONOR_BELOW:
      self.cam_brake_honor = True

  @staticmethod
  def fsm3_mode(CS, long_mode):
    """Mode written to FSM3: mode 0 whenever the camera is in it, else OP's.
    FSM3 only: FSM4 and the brake-ack mask stay on OP's mode."""
    if long_mode is not None and CS.cam_fsm3_mode == LongMode.BRAKE:
      return LongMode.BRAKE
    return long_mode

  @staticmethod
  def slope_brake_offset(CC):
    """Slope part of the coast accel (m/s^2) from the car's pitch: < 0 uphill, > 0 (capped) downhill."""
    if len(CC.orientationNED) != 3:
      return 0.
    return min(-CCP.COAST_PITCH_GAIN * math.sin(CC.orientationNED[1]), CCP.DOWNHILL_BRAKE_OFFSET_MAX)

  def update_cam_stop(self, CC, CS):
    """Camera final stop behind a lead: state and blend weight, at 50 Hz."""
    lead = CS.acc_distance < 255
    v = CS.out.vEgo
    if not (CCP.CAMERA_STOPS_BEHIND_LEAD and CC.longActive):
      self.cam_stop = False
      if not CC.longActive:
        self.cam_stop_w = 0.
    elif not self.cam_stop and lead and v < CCP.CAM_STOP_HANDOVER_SPEED:
      self.cam_stop = True
    elif self.cam_stop and (v > CCP.CAM_STOP_RETURN_SPEED or (not lead and v > 1.)):
      self.cam_stop = False
    step = (self.LONG_TX_PERIOD_NANOS * 1e-9) / CCP.CAM_STOP_TRANSITION_TIME
    self.cam_stop_w = min(max(self.cam_stop_w + (step if self.cam_stop else -step), 0.), 1.)

  # ---- Car -> camera relays ----

  def relay_brake_acks(self, CC, CS):
    """Brake acks to the camera: masked while OP brakes alone, spoofed while OP ignores the camera's request."""
    if self.cam_brake:
      self.cam_mask_frames = 0
    elif CC.longActive and self.long_mode == LongMode.BRAKE and not self.cam_relay:
      self.cam_mask_frames = CCP.CAM_BRAKE_MASK_HOLD_FRAMES
    elif self.cam_mask_frames > 0:
      self.cam_mask_frames -= 1
    mask = self.cam_mask_frames > 0
    masked = CCP.CAM_BRAKE_MASK_MSGS
    mask_133, mask_323, mask_220, mask_299 = (mask and addr in masked for addr in (0x133, 0x323, 0x220, 0x299))
    spoof_220, spoof_299 = (self.spoof_cam_ack and addr in masked for addr in (0x220, 0x299))
    if CS.stock_FSM4_frames:
      last_fsm4 = CS.stock_FSM4_frames[-1]
      self.cam_brake_target = ((int(last_fsm4["Byte_5"]) << 8) | int(last_fsm4["Byte_6"])) & 0x3FFF
    spoof_299 = spoof_299 and self.cam_brake_target < CCP.CAM_ACK_RELEASE_VALUE  # ack drops like the ECM's

    sends = []
    for stock_133 in CS.stock_133_frames:
      if not mask_133:
        self.brake_level_133 = int(stock_133["Byte_1"]) & 0x7F
      sends.append(volvocan.create_cam_brake_level(self.packer_pt, stock_133, self.brake_level_133 if mask_133 else None))
    for stock_323 in CS.stock_323_frames:
      sends.append(volvocan.create_cam_brake_status(self.packer_pt, stock_323, mask_323))
    for stock_220 in CS.stock_220_frames:
      sends.append(volvocan.create_cam_wheel_status(self.packer_pt, stock_220, mask_220, spoof_220))
    for stock_299 in CS.stock_299_frames:
      sends.append(volvocan.create_cam_brake_request_ack(self.packer_pt, stock_299, mask_299, spoof_299))
    return sends
