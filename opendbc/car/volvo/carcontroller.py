from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs

try:
  from openpilot.common.params import Params
except ImportError:
  Params = None
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams, EcmAccState, LongMode, NoTargetHold, SteerDirection
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

    # Buttons: lead SNG and green-light resume
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    self.go_count = 0
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

    # No-target stop
    self.nt_hold = NoTargetHold.OFF
    self.nt_release_frames = 0
    self.nt_departing = False
    self.nt_button = None  # fake press shown to the camera: 'onoff' / 'resume'
    self.nt_button_frames = 0
    self.stopped_frames = 0
    self.op_stopping_slow = False
    self.op_stopping_flag = False

    # Brake acks to the camera
    self.spoof_cam_ack = False
    self.cam_brake_target = 0
    self.cam_mask_frames = 0
    self.brake_level_133 = 0

    # OP holding the camera's last good frames toward the ECM
    self.op_owns_acc = False
    self.cam_was_enabled = False
    self.fsm0_good = None
    self.fsm3_good = {}
    self.fsm4_good_byte2 = None

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if CC.cruiseControl.cancel and CS.out.vEgo > self.CP.minSteerSpeed and not CS.out.brakePressed:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    if self.frame % 2 == 0:
      can_sends += self.lateral(CC, CS)

    go_request = self.resume_buttons(CC, CS, can_sends)
    ecm_engaged = CS.ecm_acc_engaged
    self.update_no_target_stop(CC, CS, go_request, ecm_engaged)
    good_fsm3 = self.relay_fsm0(CC, CS, can_sends, ecm_engaged)

    if now_nanos >= self.next_long_tx_nanos:
      next_tx = self.next_long_tx_nanos + self.LONG_TX_PERIOD_NANOS
      if self.next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.LONG_TX_PERIOD_NANOS
      self.next_long_tx_nanos = next_tx
      can_sends += self.longitudinal(CC, CS, good_fsm3)

    can_sends += self.relay_fsm4(CC, CS)
    can_sends += self.relay_brake_acks(CC, CS)

    cam_ecm_state = self.cam_ecm_state(ecm_engaged)
    for stock_12a in CS.stock_12A_frames:
      can_sends.append(volvocan.create_cam_engine_data(self.packer_pt, stock_12a, cam_ecm_state))
    press = self.nt_button if self.nt_button_frames * DT_CTRL < CCP.NO_TARGET_BUTTON_TIME else None
    for stock_127 in CS.stock_127_frames:
      can_sends.append(volvocan.create_cam_buttons(self.packer_pt, stock_127, press))

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
    """Resume burst: lead SNG (lead moved away) and green light (OP wants to go from a no-target stop).
    Returns go_request."""
    actuators = CC.actuators
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

    go_request = (CCP.OP_STOPS_WITHOUT_TARGET and CC.longActive and CS.out.standstill and
                  (self.nt_hold == NoTargetHold.HOLD or not CS.out.cruiseState.standstill) and
                  actuators.longControlState != LongCtrlState.stopping and actuators.accel > 0.)
    if go_request and not self.waiting and (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      can_sends.extend(self.resume_burst())
      if self.go_count == 0:
        self.sng_ack_frames = 25
      self.go_count += 1
    if self.go_count > 0 and (self.go_count >= 5 or not go_request):
      self.go_count = 0
      self.last_resume_frame = self.frame
    return go_request

  def resume_burst(self):
    return [volvocan.create_button_msg(self.packer_pt, resume=True)] * self.CCP.BUTTON_BURST

  # ---- No-target stop (red light) ----

  def update_no_target_stop(self, CC, CS, go_request, ecm_engaged):
    """NoTargetHold state: OFF -> HOLD on the camera's no-target cancel, -> RELEASE when OP wants to go."""
    actuators = CC.actuators
    if not ecm_engaged:
      self.cam_was_enabled = False

    self.stopped_frames = self.stopped_frames + 1 if CS.out.standstill else 0
    if not (CCP.OP_STOPS_WITHOUT_TARGET and CC.enabled and ecm_engaged) or CS.out.brakePressed:
      self.nt_hold = NoTargetHold.OFF
    elif self.nt_hold == NoTargetHold.OFF:
      wants_to_go = go_request or actuators.accel > 0. or actuators.longControlState == LongCtrlState.starting
      cam_cancel = bool(int(CS.stock_FSM3["Byte_2"]) & 0x20)
      if self.op_stopping_slow and not wants_to_go and cam_cancel:
        self.nt_hold = NoTargetHold.HOLD
        self.start_cam_button('onoff')
    elif self.nt_hold == NoTargetHold.HOLD:
      if go_request or CS.out.gasPressed or self.long_mode != LongMode.BRAKE:
        self.nt_hold = NoTargetHold.RELEASE
        self.nt_release_frames = 0
        self.nt_departing = True
        self.start_cam_button('resume')
    else:
      self.nt_release_frames += 1
      if self.nt_release_frames * DT_CTRL > CCP.NO_TARGET_RELEASE_TIMEOUT:
        self.nt_hold = NoTargetHold.OFF

    self.nt_button_frames += 1  # a fake press always lasts NO_TARGET_BUTTON_TIME, like a real one

    # keep hiding the no-target cancel while pulling away slowly
    if not CC.longActive or CS.out.vEgo > CCP.NO_TARGET_STOP_SPEED:
      self.nt_departing = False

  def start_cam_button(self, press):
    self.nt_button = press if CCP.NO_TARGET_FAKE_BUTTONS else None
    self.nt_button_frames = 0

  def cam_ecm_state(self, ecm_engaged):
    """ECM state shown to the camera: standby while holding, switched a moment after the fake press."""
    if not ecm_engaged or self.nt_hold == NoTargetHold.OFF:
      return None
    after_press = not CCP.NO_TARGET_FAKE_BUTTONS or self.nt_button_frames * DT_CTRL >= CCP.NO_TARGET_STATE_DELAY
    if self.nt_hold == NoTargetHold.HOLD:
      return EcmAccState.STANDBY if after_press else None
    return None if after_press else EcmAccState.STANDBY  # RELEASE: engaged again after the Resume press

  # ---- Camera -> ECM relays ----

  def relay_fsm0(self, CC, CS, can_sends, ecm_engaged):
    """FSM0 relayed 1:1; the camera's last good frame while OP owns ACC. Returns the FSM3 frame to hold, if any."""
    for stock_fsm0 in CS.stock_FSM0_frames:
      cam_ok = bool(int(stock_fsm0["Byte_2"]) & 0x04) and int(stock_fsm0["Byte_3"]) == 0
      if cam_ok:
        self.fsm0_good = stock_fsm0
        self.cam_was_enabled = self.cam_was_enabled or ecm_engaged
        if self.nt_hold == NoTargetHold.RELEASE:
          self.nt_hold = NoTargetHold.OFF  # camera re-engaged after the go
      takeover = CCP.OP_OWNS_ACC_AFTER_CAMERA_ABORT and self.CP.openpilotLongitudinalControl and self.cam_was_enabled
      self.op_owns_acc = (CC.enabled and ecm_engaged and not cam_ok and self.fsm0_good is not None and
                          (takeover or self.nt_hold != NoTargetHold.OFF))
      can_sends.append(volvocan.create_fsm0(self.packer_pt, stock_fsm0, ecm_engaged, self.fsm0_good if self.op_owns_acc else None))
    if not (ecm_engaged and CC.enabled):
      self.op_owns_acc = False

    # last good FSM3 per frame kind (data / type)
    fsm3_kind = int(CS.stock_FSM3["Byte_4"]) == 0x81
    if not self.op_owns_acc:
      if int(CS.stock_FSM3["Byte_6"]) == 0 and (int(CS.stock_FSM3["Byte_2"]) & 0xFC) not in (0xFC, 0xDC):
        self.fsm3_good[fsm3_kind] = CS.stock_FSM3
    return self.fsm3_good.get(fsm3_kind) if self.op_owns_acc else None

  def relay_fsm4(self, CC, CS):
    """FSM4 relayed 1:1; with OP long, OP sets the brake flag and target."""
    sends = []
    fsm4_mode = self.long_mode if (CC.longActive and not self.cam_relay) else None
    for stock_fsm4 in CS.stock_FSM4_frames:
      self.cam_brake = bool(int(stock_fsm4["Byte_5"]) & 0x40)
      if not self.op_owns_acc and int(stock_fsm4["Byte_2"]) != 0xFC:
        self.fsm4_good_byte2 = int(stock_fsm4["Byte_2"])
      good_byte2 = self.fsm4_good_byte2 if self.op_owns_acc else None
      sends.append(volvocan.create_fsm4(self.packer_pt, stock_fsm4, self.accel, fsm4_mode, good_byte2,
                                        stopping=self.op_stopping_flag, cam_weight=self.cam_weight))
    return sends

  # ---- Longitudinal ----

  def longitudinal(self, CC, CS, good_fsm3):
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

    self.op_stopping_slow = (CCP.OP_STOPS_WITHOUT_TARGET and CC.longActive and self.cam_stop_w == 0. and
                             self.long_mode == LongMode.BRAKE and CS.out.vEgo < CCP.NO_TARGET_STOP_SPEED)
    self.op_stopping_flag = self.op_stopping_slow and CS.out.vEgo < CCP.STOPPING_FLAG_SPEED
    # OP not braking on the camera's request: its ack is spoofed
    self.spoof_cam_ack = (CC.longActive and not self.cam_relay and self.cam_brake and not self.cam_brake_honor and
                          self.long_mode != LongMode.BRAKE)

    if self.sng_ack_frames > 0:
      acc_check = 1
      self.sng_ack_frames -= 1
    else:
      acc_check = int(CS.stock_FSM3["ACC_Check"])

    nt_standstill = self.nt_hold == NoTargetHold.HOLD and self.stopped_frames * DT_CTRL >= CCP.NO_TARGET_STANDSTILL_DELAY
    hide_cancel = self.op_stopping_slow or self.nt_departing or self.nt_hold != NoTargetHold.OFF
    return [
      volvocan.create_radar(self.packer_pt, CS.stock_FSM1),
      volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, acc_check, long_mode, good_fsm3,
                                   hide_no_target_cancel=hide_cancel, standstill=nt_standstill),
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

    if self.cam_relay:
      # camera owns the stop: FSM3/FSM4 relayed untouched
      cam_mode = int(CS.stock_FSM3["Byte_2"]) & 0x3
      self.long_mode = LongMode(cam_mode) if cam_mode <= LongMode.DRIVE else LongMode.COAST
      self.cam_brake_honor = self.cam_brake
      return cam_accel, None

    if w > 0.:
      # blending into the camera's stop
      accel = accel + w * (cam_accel - accel)
      self.long_mode = LongMode.BRAKE if self.cam_brake else volvocan.next_long_mode(self.long_mode, accel)
      self.cam_brake_honor = self.cam_brake
      return accel, self.long_mode

    if not self.cam_brake:
      self.cam_brake_honor = False
    elif not self.cam_brake_honor:
      # latched until the camera releases the brake
      self.cam_brake_honor = not CCP.OP_IGNORES_MILD_CAMERA_BRAKE or cam_accel < CCP.CAMERA_BRAKE_HONOR_BELOW
    if self.cam_brake_honor:
      accel = min(accel, cam_accel)
      self.long_mode = LongMode.BRAKE
    else:
      self.long_mode = volvocan.next_long_mode(self.long_mode, accel)
    return accel, self.long_mode

  def update_cam_stop(self, CC, CS):
    """Camera final stop behind a lead: state and blend weight, at 50 Hz."""
    lead = CS.acc_distance < 255
    v = CS.out.vEgo
    if not (CCP.CAMERA_STOPS_BEHIND_LEAD and CC.longActive) or self.nt_hold != NoTargetHold.OFF:
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
