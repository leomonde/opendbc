from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL

try:
  from openpilot.common.params import Params
except ImportError:
  Params = None
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.values import CarControllerParams, LongMode, SteerDirection
from opendbc.sunnypilot.car.volvo.icbm import IntelligentCruiseButtonManagementInterface


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CP = CP
    self.CCP = CarControllerParams(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

    self.apply_steer_prev = 0
    self.apply_steer_dir_prev = SteerDirection.NONE

    self.latActive_prev = False
    self.steer_blocked = False
    self.steer_blocked_cnt = 0
    self.steer_dir_bf_block = SteerDirection.NONE

    # Custom ACC increment: read once at init, refreshed every 100 frames.
    # Passed to carstate so _pending_delta emits the right number of events.
    self._params = Params() if Params is not None else None
    self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1)) if self._params is not None else 1

    # SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    # FSM3 ACC_Check=1 ack window: stock FSM3 has ACC_Check=0; ECU ignores
    # OP's CCButtons resume unless FSM3 confirms it. Force 25 frames (~0.5s).
    self.sng_ack_frames = 0

    # wall-clock gate for FSM3/FSM1 TX — frame%2 unreliable due to controlsd jitter;
    # fwd_hook blocks stock FSM3/FSM1 when controls_allowed && !gas_pressed, relay at 50Hz.
    self.next_long_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000  # 50Hz, matching stock FSM3/FSM1

    self.long_mode = LongMode.COAST
    self.accel = 0.0
    self.cam_brake = False
    self.cam_mask_frames = 0
    self.brake_level_133 = 0

    # OP owning ACC after the camera's abort (see FSM0 in update)
    self.op_owns_acc = False
    self.cam_was_enabled = False
    self.fsm0_good = None
    self.fsm3_good = None
    self.fsm4_good_byte2 = None

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # TODO: verify if this minSteerSpeed guard is still needed
    if pcm_cancel_cmd and CS.out.vEgo > self.CP.minSteerSpeed and not CS.out.brakePressed:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    # run at 50hz
    if self.frame % 2 == 0:
      if CC.latActive and CS.out.vEgo > self.CP.minSteerSpeed:
        apply_steer = apply_std_steer_angle_limits(
          actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw,
          CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS
        )
        apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

        error = CS.out.steeringAngleDeg - apply_steer
        error_with_deadzone = 0 if abs(error) < CarControllerParams.DEADZONE else error

        # Update prev with desired if just enabled.
        if not self.latActive_prev:
          self.apply_steer_dir_prev = apply_steer_dir

        if self.steer_blocked:
          if (apply_steer_dir == self.steer_dir_bf_block) or (self.steer_blocked_cnt <= 0) or (error_with_deadzone == 0):
            self.steer_blocked = False
        else:
          if apply_steer_dir != self.apply_steer_dir_prev and error_with_deadzone != 0:
            self.steer_blocked = True
            self.steer_blocked_cnt = CarControllerParams.BLOCK_LEN
            self.steer_dir_bf_block = self.apply_steer_dir_prev

        if self.steer_blocked:
          self.steer_blocked_cnt -= 1
          apply_steer_dir = SteerDirection.NONE
        elif error_with_deadzone == 0:
          # Set old request when inside deadzone
          apply_steer_dir = self.apply_steer_dir_prev

      else:
        apply_steer = 0
        apply_steer_dir = SteerDirection.NONE

      can_sends.append(volvocan.create_lka_msg(self.packer_pt, apply_steer, int(apply_steer_dir)))

      self.apply_steer_prev = apply_steer
      self.apply_steer_dir_prev = apply_steer_dir
      self.latActive_prev = CC.latActive

      # Manipulate data from servo to FSM
      # Avoids faults that will stop servo from accepting steering commands.
      can_sends.append(volvocan.create_lkas_state_msg(self.packer_pt, CS.out.steeringAngleDeg, CS.pscm_stock_values))

    at_standstill = (CS.out.cruiseState.enabled and CS.out.cruiseState.standstill
                     and CS.out.vEgo < 0.01)

    # SNG
    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0

      lead_moved = CS.acc_distance > self.distance

      if at_standstill and self.waiting and lead_moved:
        can_sends.extend([volvocan.create_button_msg(self.packer_pt, resume=True)] * self.CCP.BUTTON_BURST)
        if self.sng_count == 0:
          self.sng_ack_frames = 25
        self.sng_count += 1
      # disable sending resume after 5 cycles sent or if no more in standstill
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    # FSM0: relay each camera frame 1:1 (panda always blocks the camera's). With OP long the camera aborts its ACC
    # function ~1.5 s after OP brakes for something it did not see (routes 812-816). The ECM is the ACC master
    # (EngineData leads FSM0), so once the camera was enabled in this engagement, OP keeps owning ACC while the
    # driver keeps the ECM engaged, instead of passing on the camera's abort (off by default, see
    # OP_OWNS_ACC_AFTER_CAMERA_ABORT). The safety only allows FSM0 with ACC enabled while the ECM is engaged.
    ecm_engaged = CS.ecm_acc_engaged
    if not ecm_engaged:
      self.cam_was_enabled = False
    for stock_fsm0 in CS.stock_FSM0_frames:
      cam_ok = bool(int(stock_fsm0["Byte_2"]) & 0x04) and int(stock_fsm0["Byte_3"]) == 0
      if cam_ok:
        self.fsm0_good = stock_fsm0
        self.cam_was_enabled = self.cam_was_enabled or ecm_engaged
      self.op_owns_acc = (CarControllerParams.OP_OWNS_ACC_AFTER_CAMERA_ABORT and self.CP.openpilotLongitudinalControl and
                          CC.enabled and ecm_engaged and self.cam_was_enabled and not cam_ok)
      can_sends.append(volvocan.create_fsm0(self.packer_pt, stock_fsm0, ecm_engaged, self.fsm0_good if self.op_owns_acc else None))
    if not (ecm_engaged and CC.enabled):
      self.op_owns_acc = False
    if not self.op_owns_acc:
      if int(CS.stock_FSM3["Byte_6"]) == 0 and (int(CS.stock_FSM3["Byte_2"]) & 0xFC) != 0xFC:
        self.fsm3_good = CS.stock_FSM3
    good_fsm3 = self.fsm3_good if self.op_owns_acc else None

    # FSM3/FSM1 at 50Hz wall-clock: relay stock values so ECU doesn't fault.
    # Override ACC_Check=1 during SNG resume so ECU acknowledges OP's CCButtons resume.
    long_tx_due = now_nanos >= self.next_long_tx_nanos
    if long_tx_due:
      next_tx = self.next_long_tx_nanos + self.LONG_TX_PERIOD_NANOS
      if self.next_long_tx_nanos == 0 or next_tx <= now_nanos:
        next_tx = now_nanos + self.LONG_TX_PERIOD_NANOS
      self.next_long_tx_nanos = next_tx

      # OP long: OP owns accel, mode and traction; otherwise relay the camera's request.
      # When the camera asks for the hydraulic brake (FSM4 Byte_5 bit6), OP keeps it and may only brake harder: the
      # camera aborts ACC ~1.5 s after its brake request goes unanswered by the car (route 822, FSM0 Byte_3 0x10).
      if CS.stock_FSM4_frames:
        self.cam_brake = bool(int(CS.stock_FSM4_frames[-1]["Byte_5"]) & 0x40)
      if CC.longActive:
        accel = min(max(actuators.accel, CarControllerParams.ACCEL_MIN), CarControllerParams.ACCEL_MAX)
        if self.cam_brake:
          accel = min(accel, float(CS.stock_FSM3["ACC_AccelerationRequest"]))
          self.long_mode = LongMode.BRAKE
        else:
          self.long_mode = volvocan.next_long_mode(self.long_mode, accel)
        long_mode = self.long_mode
      else:
        accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
        self.long_mode = LongMode.COAST
        long_mode = None
      self.accel = accel

      if self.sng_ack_frames > 0:
        acc_check = 1
        self.sng_ack_frames -= 1
      else:
        acc_check = int(CS.stock_FSM3["ACC_Check"])

      # FSM1 Byte_2 bit2 stays the camera's for now: with it and FSM4 bit6 both from OP, the camera aborted ACC
      # ~1.5 s after OP braked (routes 812/814, FSM0 Byte_3 0x10). Test FSM4 bit6 alone to tell which one causes it.
      can_sends.append(volvocan.create_radar(self.packer_pt, CS.stock_FSM1))
      can_sends.append(volvocan.create_longitudinal(self.packer_pt, CS.stock_FSM3, accel, acc_check, long_mode, good_fsm3))

    # FSM4: relay each camera frame 1:1 (panda always blocks the camera's); with OP long, OP owns the brake flag
    # (set whenever OP or the camera brakes, see FSM3 above)
    fsm4_mode = self.long_mode if CC.longActive else None
    for stock_fsm4 in CS.stock_FSM4_frames:
      self.cam_brake = bool(int(stock_fsm4["Byte_5"]) & 0x40)
      if not self.op_owns_acc and int(stock_fsm4["Byte_2"]) != 0xFC:
        self.fsm4_good_byte2 = int(stock_fsm4["Byte_2"])
      good_byte2 = self.fsm4_good_byte2 if self.op_owns_acc else None
      can_sends.append(volvocan.create_fsm4(self.packer_pt, stock_fsm4, self.accel, fsm4_mode, good_byte2))

    # Car -> camera brake response (0x133, 0x323, 0x220, 0x299): relay 1:1 (panda always blocks the car's), masked while OP brakes
    # and the camera does not, so the camera does not see a brake it never asked for and abort ACC
    if self.cam_brake:
      self.cam_mask_frames = 0
    elif CC.longActive and self.long_mode == LongMode.BRAKE:
      self.cam_mask_frames = CarControllerParams.CAM_BRAKE_MASK_HOLD_FRAMES
    elif self.cam_mask_frames > 0:
      self.cam_mask_frames -= 1
    mask = self.cam_mask_frames > 0
    for stock_133 in CS.stock_133_frames:
      if not mask:
        self.brake_level_133 = int(stock_133["Byte_1"]) & 0x7F
      can_sends.append(volvocan.create_cam_brake_level(self.packer_pt, stock_133, self.brake_level_133 if mask else None))
    for stock_323 in CS.stock_323_frames:
      can_sends.append(volvocan.create_cam_brake_status(self.packer_pt, stock_323, mask))
    for stock_220 in CS.stock_220_frames:
      can_sends.append(volvocan.create_cam_wheel_status(self.packer_pt, stock_220, mask))
    for stock_299 in CS.stock_299_frames:
      can_sends.append(volvocan.create_cam_brake_request_ack(self.packer_pt, stock_299, mask))

    # Refresh custom ACC step every 100 frames and forward to carstate so that
    # _pending_delta emits exactly one synthetic event per physical ACC step.
    if self.frame % 100 == 0 and self._params is not None:
      self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1))
    CS._custom_acc_step = self._custom_acc_step

    # Intelligent Cruise Button Management
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
