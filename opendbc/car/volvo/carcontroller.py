import math

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL

try:
  from openpilot.common.params import Params
except ImportError:
  Params = None
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volvo import volvocan
from opendbc.car.volvo.full_stop import FullStop
from opendbc.car.volvo.values import CarControllerParams, LongMode, SteerDirection
from opendbc.sunnypilot.car.volvo.icbm import IntelligentCruiseButtonManagementInterface

CCP = CarControllerParams


def cam_brake_leftover(CS) -> bool:
  """Camera still in brake mode with no target and no braking asked: its brake kept 6 s after a false target (route 8bd, 193.6 s)."""
  return CS.acc_distance >= 255 and float(CS.stock_FSM3["ACC_AccelerationRequest"]) >= 0.


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

    # Custom ACC step, refreshed every 100 frames
    self._params = Params() if Params is not None else None
    self._custom_acc_step = max(1, int(self._params.get("CustomAccShortPressIncrement", return_default=True) or 1)) if self._params is not None else 1

    # Buttons: lead SNG
    self.last_resume_frame = 0
    self.distance = 0
    self.waiting = False
    self.sng_count = 0
    self.op_resume_frames = 0
    self.sng_ack_frames = 0  # FSM3 ACC_Check=1 frames for OP's resume
    self.resume_hold_frames = 0  # Resume held to the ECM, frames left
    self.last_ccbuttons = None  # CEM's last CCButtons, copied for the Resume

    # Downhill: set speed + offset to the planner
    self.downhill = False
    self.downhill_frames = 0  # frames the pitch has been past the enter/exit threshold

    # FSM3/FSM1 sent at 50 Hz wall-clock
    self.next_long_tx_nanos = 0
    self.LONG_TX_PERIOD_NANOS = 20_000_000

    # Longitudinal decision
    self.long_mode = LongMode.COAST
    self.accel = 0.0
    self.cam_brake = False
    self.cam_brake_honor = False
    self.cam_hard_brake = False  # the camera brakes harder than OP's limit: its FSM3/FSM4 go out unchanged

    # Brake acks to the camera
    self.spoof_cam_ack = False
    self.cam_brake_target = 0
    self.cam_mask_frames = 0
    self.brake_level = 0
    self.brake_hold_b0: int | None = None

    self.full_stop = FullStop()  # OP's stop to 0 (lead, red light, stop sign)

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    if CC.cruiseControl.cancel and CS.out.vEgo > CCP.MIN_STEER_SPEED and not CS.out.brakePressed:
      can_sends.append(volvocan.create_button_msg(self.packer_pt, cancel=True))

    if self.frame % 2 == 0:
      can_sends += self.lateral(CC, CS)

    self.full_stop.update(CC, CS)
    self.update_downhill(CC, CS)
    # Carstate takes the ACC standstill from OP during the stop
    CS.full_stop_active = self.full_stop.active
    CS.full_stop_standstill = self.full_stop.standstill
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
    """FSM2 steering request and PSCM1 to the camera."""
    actuators = CC.actuators
    if CC.latActive and CS.out.vEgo > CCP.MIN_STEER_SPEED:
      apply_steer = apply_std_steer_angle_limits(
        actuators.steeringAngleDeg, self.apply_steer_prev, CS.out.vEgoRaw,
        CS.out.steeringAngleDeg, CC.latActive, CCP.ANGLE_LIMITS
      )
      apply_steer_dir = SteerDirection.LEFT if apply_steer > 0 else SteerDirection.RIGHT

      error = CS.out.steeringAngleDeg - apply_steer
      error_with_deadzone = 0 if abs(error) < CCP.DEADZONE else error

      if not self.latActive_prev:
        self.apply_steer_dir_prev = apply_steer_dir

      # Block direction flips until the servo catches up
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
      # Servo state to the camera
      volvocan.create_lkas_state_msg(self.packer_pt, CS.out.steeringAngleDeg, CS.pscm_stock_values),
    ]

  # ---- Buttons ----

  def resume_buttons(self, CC, CS, can_sends):
    """Resume burst when the lead moves away."""
    at_standstill = CS.out.cruiseState.enabled and CS.out.cruiseState.standstill and CS.out.vEgo < 0.01

    # OP's resume request, held for SNG_OP_RESUME_TIME
    self.op_resume_frames = self.op_resume_frames + 1 if CC.cruiseControl.resume else 0
    op_resume = self.op_resume_frames * DT_CTRL >= CCP.SNG_OP_RESUME_TIME

    if (self.frame - self.last_resume_frame) * DT_CTRL > 1.00:
      if at_standstill and not self.waiting:
        self.distance = CS.acc_distance
        self.waiting = True
        self.sng_count = 0
      # In OP's stop only the planner's resume drives off (as Ford/Toyota), not the camera's distance jitter (route 884)
      lead_moved = CS.acc_distance > self.distance and not self.full_stop.active
      # The driver's Resume also drives off OP's stop: the ECM takes it, but held in standstill it stays (route 8ab, 436 s)
      driver_resume = CS.driver_resume and self.full_stop.active
      drive_off = lead_moved or op_resume or driver_resume or self.full_stop.driver_gas
      if at_standstill and self.waiting and drive_off and self.full_stop.drive_off_ready(CS):
        self.resume_hold_frames = CCP.RESUME_HOLD_FRAMES
        if self.sng_count == 0:
          self.sng_ack_frames = 25
        self.sng_count += 1
        self.full_stop.drive_off_request = self.full_stop.active
      if self.waiting and (self.sng_count >= 5 or not CS.out.cruiseState.standstill):
        self.waiting = False
        self.last_resume_frame = self.frame

    # Resume held as the driver's: the CEM's frame with Resume pressed, every frame
    if CS.stock_CCButtons_frames:
      self.last_ccbuttons = CS.stock_CCButtons_frames[-1]
    if self.resume_hold_frames > 0 and self.last_ccbuttons is not None:
      can_sends.append(volvocan.create_resume_button(self.packer_pt, self.last_ccbuttons))
    self.resume_hold_frames = max(self.resume_hold_frames - 1, 0)

  def update_downhill(self, CC, CS):
    """Downhill from the calibrated pitch, with hysteresis: carstate reports the set speed + offset then."""
    pitch = math.degrees(CC.orientationNED[1]) if len(CC.orientationNED) == 3 else 0.
    past = pitch > CCP.DOWNHILL_EXIT_PITCH if self.downhill else pitch < CCP.DOWNHILL_ENTER_PITCH
    self.downhill_frames = self.downhill_frames + 1 if past else 0
    hold_time = CCP.DOWNHILL_EXIT_TIME if self.downhill else CCP.DOWNHILL_ENTER_TIME
    if self.downhill_frames * DT_CTRL >= hold_time:
      self.downhill = not self.downhill
      self.downhill_frames = 0
    CS.downhill = self.downhill

  # ---- Camera -> ECM relays ----

  def relay_fsm0(self, CS, can_sends, ecm_engaged):
    """FSM0 relayed, never ACC enabled without the ECM."""
    for stock_fsm0 in CS.stock_FSM0_frames:
      can_sends.append(volvocan.create_fsm0(self.packer_pt, self.full_stop.fsm0(stock_fsm0), ecm_engaged))

  def relay_fsm4(self, CC, CS):
    """FSM4 relayed; OP sets brake flag and target."""
    sends = []
    fsm4_mode = self.long_mode if (CC.longActive and not self.cam_hard_brake) else None
    for stock_fsm4 in CS.stock_FSM4_frames:
      self.cam_brake = bool(stock_fsm4["ACC_BrakeRequest"])
      mode = fsm4_mode
      if self.full_stop.active:
        stock_fsm4, op_mode = self.full_stop.fsm4(stock_fsm4)
        mode = mode if op_mode else None
      sends.append(volvocan.create_fsm4(self.packer_pt, stock_fsm4, self.accel, mode))
    return sends

  # ---- Longitudinal ----

  def longitudinal(self, CC, CS):
    """Accel and mode, FSM1/FSM3 at 50 Hz."""
    if CS.stock_FSM4_frames:
      self.cam_brake = bool(CS.stock_FSM4_frames[-1]["ACC_BrakeRequest"])
    if self.full_stop.driver_gas:
      # Driver's gas: no accel nor brake from OP, from the first frame the panda sees the pedal
      accel, long_mode = 0., LongMode.COAST
      self.long_mode = LongMode.COAST
      self.cam_brake_honor = False
    elif CC.longActive:
      accel, long_mode = self.long_decision(CC, CS)
    else:
      accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
      self.long_mode = LongMode.COAST
      self.cam_brake_honor = False
      long_mode = None
    if self.full_stop.holding:
      # The car holds the stop: the camera's standstill request (route 807)
      accel = CCP.HOLD_ACCEL
      long_mode = self.long_mode = LongMode.BRAKE
    self.accel = accel
    # The panda only lets the camera's own frames past OP's limit, so they go out unchanged
    cam_accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
    self.cam_hard_brake = CC.longActive and self.cam_brake_honor and not self.full_stop.active and cam_accel < CCP.ACCEL_MIN

    # Ack spoofed while OP ignores the camera's brake
    self.spoof_cam_ack = (CC.longActive and self.cam_brake and not self.cam_brake_honor and
                          self.long_mode != LongMode.BRAKE and not self.full_stop.active)

    stock_fsm3 = self.full_stop.fsm3(CS)
    # OP's own frames within the panda's limit, also when following the camera's brake (route 895, 384.3 s)
    accel = max(accel, CCP.ACCEL_MIN)
    if self.sng_ack_frames > 0:
      acc_check = 1
      self.sng_ack_frames -= 1
    else:
      acc_check = int(stock_fsm3["ACC_Check"])

    if self.full_stop.active:
      # OP's mode as is (the template's mode is stale), standstill from OP, camera cancel masked
      return [
        volvocan.create_radar(self.packer_pt, CS.stock_FSM1, long_mode),
        volvocan.create_longitudinal(self.packer_pt, stock_fsm3, accel, acc_check, long_mode, standstill=self.full_stop.standstill),
      ]
    if self.cam_hard_brake:
      return [
        volvocan.create_radar(self.packer_pt, CS.stock_FSM1),
        volvocan.create_longitudinal(self.packer_pt, stock_fsm3, cam_accel, int(stock_fsm3["ACC_Check"])),
      ]
    return [
      volvocan.create_radar(self.packer_pt, CS.stock_FSM1),
      volvocan.create_longitudinal(self.packer_pt, stock_fsm3, accel, acc_check, self.fsm3_mode(CS, long_mode)),
    ]

  def long_decision(self, CC, CS):
    """Accel and mode with OP long active."""
    actuators = CC.actuators
    accel = min(max(actuators.accel, CCP.ACCEL_MIN), CCP.ACCEL_MAX)
    accel = self.jerk_limit(accel, self.accel)
    cam_accel = float(CS.stock_FSM3["ACC_AccelerationRequest"])
    accel = self.camera_accel_cap(CS, accel, cam_accel)
    accel = self.camera_brake(accel, cam_accel, cam_brake_leftover(CS))
    brake_offset = self.slope_brake_offset(CC)

    if self.cam_brake_honor:
      self.long_mode = LongMode.BRAKE
    else:
      self.long_mode = volvocan.next_long_mode(self.long_mode, accel, brake_offset)
    return accel, self.long_mode

  def jerk_limit(self, accel, last_accel):
    """Ramp OP's accel request (jerk limits)."""
    dt = self.LONG_TX_PERIOD_NANOS * 1e-9
    down = CCP.JERK_DOWN_LIFT if last_accel > 0. else CCP.JERK_DOWN_BRAKE
    return min(max(accel, last_accel - down * dt), last_accel + CCP.JERK_UP * dt)

  # ---- Camera rules ----

  def camera_brake(self, accel, cam_accel, leftover=False):
    """Follow the camera's hard brake, latched until it releases or only its leftover brake state is left."""
    if not self.cam_brake or leftover:
      self.cam_brake_honor = False
    elif cam_accel < CCP.CAMERA_BRAKE_HONOR_BELOW:
      self.cam_brake_honor = True
    return min(accel, cam_accel) if self.cam_brake_honor else accel

  @staticmethod
  def camera_accel_cap(CS, accel, cam_accel):
    """No more accel than the camera's while its target is close."""
    close = max(CS.out.vEgo * CCP.CAMERA_ACCEL_CAP_GAP, CCP.CAMERA_ACCEL_CAP_MIN_DIST)
    if CS.acc_distance >= min(close, 255):
      return accel
    return min(accel, max(cam_accel, CCP.CAMERA_ACCEL_CAP_FLOOR))

  @staticmethod
  def fsm3_mode(CS, long_mode):
    """FSM3 mode: 0 when the camera is in it, else OP's."""
    if long_mode is not None and CS.cam_fsm3_mode == LongMode.BRAKE and not cam_brake_leftover(CS):
      return LongMode.BRAKE
    return long_mode

  @staticmethod
  def slope_brake_offset(CC):
    """Slope part of the coast accel from pitch."""
    if len(CC.orientationNED) != 3:
      return 0.
    return min(-CCP.COAST_PITCH_GAIN * math.sin(CC.orientationNED[1]), CCP.DOWNHILL_BRAKE_OFFSET_MAX)

  # ---- Car -> camera relays ----

  def relay_brake_acks(self, CC, CS):
    """Brake acks to the camera: masked or spoofed."""
    full_stop = self.full_stop
    if self.cam_brake and not full_stop.active:
      self.cam_mask_frames = 0
    elif (CC.longActive and self.long_mode == LongMode.BRAKE) or full_stop.active:
      # OP's stop: the camera never asked, so it never sees the car brake or hold
      self.cam_mask_frames = CCP.CAM_BRAKE_MASK_HOLD_FRAMES
    elif self.cam_mask_frames > 0:
      self.cam_mask_frames -= 1
    mask = self.cam_mask_frames > 0
    masked = CCP.CAM_BRAKE_MASK_MSGS
    mask_brake_level, mask_brake_status, mask_wheelspeed1, mask_brake_req_ack = (
      mask and msg in masked for msg in ("BrakeLevel", "BrakeStatus", "wheelspeed1", "BrakeReqAck"))
    spoof_wheelspeed1, spoof_brake_req_ack = (self.spoof_cam_ack and msg in masked for msg in ("wheelspeed1", "BrakeReqAck"))
    if CS.stock_FSM4_frames:
      last_fsm4 = CS.stock_FSM4_frames[-1]
      self.cam_brake_target = int(last_fsm4["ACC_BrakeTarget"])
    spoof_brake_req_ack = spoof_brake_req_ack and self.cam_brake_target < CCP.CAM_ACK_RELEASE_VALUE

    cam_standby = full_stop.show_cam_standby

    sends = []
    for stock_brake_level in CS.stock_BrakeLevel_frames:
      if not mask_brake_level:
        self.brake_level = int(stock_brake_level["BrakeLevel"])
      sends.append(volvocan.create_cam_brake_level(self.packer_pt, stock_brake_level, self.brake_level if mask_brake_level else None))
    for stock_brake_status in CS.stock_BrakeStatus_frames:
      sends.append(volvocan.create_cam_brake_status(self.packer_pt, stock_brake_status, mask_brake_status))
    for stock_wheelspeed1 in CS.stock_wheelspeed1_frames:
      sends.append(volvocan.create_cam_wheel_status(self.packer_pt, stock_wheelspeed1, mask_wheelspeed1, spoof_wheelspeed1, hide_hold=mask,
                                                    show_standby=cam_standby))
    for stock_brake_req_ack in CS.stock_BrakeReqAck_frames:
      sends.append(volvocan.create_cam_brake_request_ack(self.packer_pt, stock_brake_req_ack, mask_brake_req_ack, spoof_brake_req_ack,
                                                         hide_hold=mask))
    # EngineData and 0x325 always relayed (the panda blocks them); ACC standby while the camera is shown it
    for stock_engine_data in CS.stock_EngineData_frames:
      sends.append(volvocan.create_cam_engine_data(self.packer_pt, stock_engine_data, cam_standby))
    for stock_acc_hold in CS.stock_AccHold_frames:
      sends.append(volvocan.create_cam_acc_hold(self.packer_pt, stock_acc_hold, cam_standby))
    # CCButtons always relayed (the panda blocks them), with Set held while re-engaging the camera
    set_plus = full_stop.take_cam_set()
    for stock_ccbuttons in CS.stock_CCButtons_frames:
      sends.append(volvocan.create_cam_button(self.packer_pt, stock_ccbuttons, set_plus))
    # 0x1FF b0 held from the stop on, while stopped in OP's stop
    hold_mask = full_stop.active and CS.out.standstill
    for stock_brake_hold in CS.stock_BrakeHold_frames:
      if not hold_mask or self.brake_hold_b0 is None:
        self.brake_hold_b0 = int(stock_brake_hold["Byte_0"])
      sends.append(volvocan.create_cam_brake_hold(self.packer_pt, stock_brake_hold, self.brake_hold_b0 if hold_mask else None))
    return sends
