from cereal import car
from opendbc.can.packer import CANPacker

from opendbc.car import DT_CTRL
from opendbc.car.chery import cherycan
from opendbc.car.chery.values import (
  AUTORESUME_BURST_FRAMES,
  AUTORESUME_CYCLE_S,
  CarControllerParams,
  DBC,
  HUD_STEP,
  LANE_KEEP_STEP,
  LKAS_INFO_STEP,
  lowpass_steer_cmd,
)
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

ButtonType = car.CarState.ButtonEvent.Type


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.last_apply_angle: float | None = None
    self.prev_brake_pressed = False

    # ACC arming + auto-resume from standstill state.
    self.acc_armed = False
    self.autoresume_burst_left = 0
    self.autoresume_last_burst_frame = -10_000
    self.autoresume_burst_idx = 0  # alternates RES (even) / SET (odd)
    self.hud_counter = 0

  def _compute_apply_angle(self, CS, actuators, steer_req):
    if not steer_req:
      return CS.out.steeringAngleDeg
    meas = CS.out.steeringAngleDeg
    prev = self.last_apply_angle if self.last_apply_angle is not None else meas
    filtered = lowpass_steer_cmd(actuators.steeringAngleDeg, self.last_apply_angle)
    self.last_apply_angle = apply_std_steer_angle_limits(
      filtered, prev, CS.out.vEgo, meas, True, CarControllerParams.ANGLE_LIMITS,
    )
    return self.last_apply_angle

  def _update_acc_armed(self, CS):
    """Arm when ACC is actively engaged while moving; disarm only on explicit driver intent
    (brake press edge or ICC-toggle press). Stays armed across stock auto-disengage at standstill."""
    if CS.out.cruiseState.enabled and not CS.out.standstill:
      self.acc_armed = True

    brake_edge = CS.out.brakePressed and not self.prev_brake_pressed
    icc_press = any(be.pressed and be.type == ButtonType.altButton2 for be in CS.out.buttonEvents)
    if brake_edge or icc_press:
      self.acc_armed = False
    self.prev_brake_pressed = CS.out.brakePressed

  def _auto_resume(self, CS, can_sends):
    """Spam alternating RES / SET button presses while standing still and armed,
    so stock RES (+1 km/h side-effect) cancels with stock SET (-1 km/h)."""
    if self.CP.openpilotLongitudinalControl or not self.acc_armed \
       or not CS.out.standstill or CS.out.brakePressed:
      self.autoresume_burst_left = 0
      self.autoresume_burst_idx = 0
      return

    elapsed = (self.frame - self.autoresume_last_burst_frame) * DT_CTRL
    if self.autoresume_burst_left == 0 and elapsed >= AUTORESUME_CYCLE_S:
      self.autoresume_burst_left = AUTORESUME_BURST_FRAMES
      self.autoresume_last_burst_frame = self.frame

    if self.autoresume_burst_left > 0 and self.frame % 2 == 0:
      button = "CRUISE_BUTTON" if self.autoresume_burst_idx % 2 else "RES_BUTTON"
      ctr = (CS.pcm_button_counter + 1) % 16
      can_sends.append(cherycan.create_pcm_button(self.packer, ctr, 0, button))
      can_sends.append(cherycan.create_pcm_button(self.packer, ctr, 2, button))
      self.autoresume_burst_left -= 1
      if self.autoresume_burst_left == 0:
        self.autoresume_burst_idx += 1

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    lat_active = CC.latActive and not CS.out.standstill
    driver_over = CS.out.steeringPressed or CS.steer_related_intervention
    steer_req = lat_active and not driver_over

    apply_angle = CS.out.steeringAngleDeg
    if self.frame % LANE_KEEP_STEP == 0:
      apply_angle = self._compute_apply_angle(CS, CC.actuators, steer_req)
      can_sends.append(cherycan.create_lane_keep_command(
        self.packer, apply_angle, steer_req, CS.out.steeringAngleDeg,
      ))

    if self.frame % LKAS_INFO_STEP == 0:
      can_sends.append(cherycan.create_lkas_info_torque_spoof(
        self.packer, steer_req, steer_req,
        steer_related=CS.lkas_info_steer_related,
        apply_spoof_offset=not driver_over,
      ))

    if self.frame % HUD_STEP == 0:
      can_sends.append(cherycan.create_hud_override(self.packer, CS.cam_hud, self.hud_counter))
      self.hud_counter = (self.hud_counter + 1) % 16

    self._update_acc_armed(CS)
    self._auto_resume(CS, can_sends)

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends
