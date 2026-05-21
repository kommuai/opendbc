from cereal import car
from opendbc.can.packer import CANPacker

from opendbc.car import DT_CTRL
from opendbc.car.chery import cherycan
from opendbc.car.chery.values import (
  AUTORESUME_BURST_FRAMES,
  AUTORESUME_CYCLE_S,
  CarControllerParams,
  DBC,
  LANE_KEEP_STEP,
  LKAS_INFO_STEP,
  chery_steering_deg_sign,
  lowpass_steer_cmd,
)
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

ButtonType = car.CarState.ButtonEvent.Type


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.steer_sign = chery_steering_deg_sign(CP)
    self.last_apply_angle = None
    self.autoresume_burst_left = 0
    self.autoresume_last_burst_frame = -10_000
    self.acc_armed = False
    self.prev_brake_pressed = False
    self.autoresume_next_is_set = False  # toggled after each burst; False schedules RES first
    self.autoresume_burst_is_set = False

  def _compute_apply_angle(self, CS, actuators, lat_active):
    if not lat_active:
      return CS.out.steeringAngleDeg
    meas = CS.out.steeringAngleDeg
    prev = self.last_apply_angle if self.last_apply_angle is not None else meas
    filtered = lowpass_steer_cmd(actuators.steeringAngleDeg, self.last_apply_angle)
    self.last_apply_angle = apply_std_steer_angle_limits(
      filtered, prev, CS.out.vEgo, meas, True, CarControllerParams.ANGLE_LIMITS,
    )
    return self.last_apply_angle

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
        self.packer, apply_angle, steer_req, CS.out.steeringAngleDeg, self.steer_sign,
      ))

    if self.frame % LKAS_INFO_STEP == 0:
      can_sends.append(cherycan.create_lkas_info_torque_spoof(
        self.packer, lat_active, CS.out.steeringTorque,
        steer_req,
        lkas_enable=steer_req, steer_related=CS.lkas_info_steer_related,
        apply_spoof_offset=not driver_over,
      ))

    # Auto-resume from standstill (acc_armed state machine; disarm on brake or ICC only).
    if CS.out.cruiseState.enabled and not CS.out.standstill:
      self.acc_armed = True

    brake_press_edge = CS.out.brakePressed and not self.prev_brake_pressed
    button_press_edge = any(be.pressed and be.type == ButtonType.altButton2 for be in CS.out.buttonEvents)
    if brake_press_edge or button_press_edge:
      self.acc_armed = False
    self.prev_brake_pressed = CS.out.brakePressed

    auto_resume = (self.acc_armed
                   and CS.out.standstill
                   and not CS.out.brakePressed
                   and not self.CP.openpilotLongitudinalControl)
    if auto_resume:
      if self.autoresume_burst_left == 0 \
         and (self.frame - self.autoresume_last_burst_frame) * DT_CTRL >= AUTORESUME_CYCLE_S:
        self.autoresume_burst_is_set = self.autoresume_next_is_set
        self.autoresume_next_is_set = not self.autoresume_next_is_set
        self.autoresume_burst_left = AUTORESUME_BURST_FRAMES
        self.autoresume_last_burst_frame = self.frame

      if self.autoresume_burst_left > 0 and self.frame % 2 == 0:
        ctr = (CS.pcm_button_counter + 1) % 16
        press = cherycan.create_pcm_set_press if self.autoresume_burst_is_set else cherycan.create_pcm_res_press
        can_sends.append(press(self.packer, ctr, 0))
        can_sends.append(press(self.packer, ctr, 2))
        self.autoresume_burst_left -= 1
    else:
      self.autoresume_burst_left = 0
      self.autoresume_next_is_set = False
      self.autoresume_burst_is_set = False

    new_actuators = CC.actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle
    self.frame += 1
    return new_actuators, can_sends
