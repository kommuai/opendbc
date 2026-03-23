import math
import numpy as np

from opendbc.can.packer import CANPacker

from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.byd.bydcan import (
  create_accel_command,
  create_can_steer_command,
  create_lkas_hud,
  create_steering_torque_spoof_camera,
  send_buttons,
)
from opendbc.car.byd.values import DBC, CAR, ACCEL_MULT, CANBUS
from opendbc.car.byd.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import AngleSteeringLimits, apply_std_steer_angle_limits

STEER_LOWPASS_HZ = 2
STEER_DT = 0.02
MAX_STEER_ANGLE_OFFSET_DEG = 10
LKA_COOLDOWN_MIN_FRAMES = 30
BUTTON_KEEPALIVE_FRAMES = 100
SPOOF_DURATION_FRAMES = 50
SPOOF_CYCLE_FRAMES = 150


def lowpass_1pole(x, y_prev):
  if y_prev is None:
    return x
  alpha = math.exp(-2.0 * math.pi * STEER_LOWPASS_HZ * STEER_DT)
  return alpha * y_prev + (1.0 - alpha) * x


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])

    self.lka_active = False
    self.last_apply_angle = 0
    self.accel_mult = ACCEL_MULT[CP.carFingerprint]
    self.lka_cooldown = 0
    self.prev_press = False
    self.lka_latched = False
    self.button_send_bus = CANBUS.cam_bus if CP.carFingerprint in (CAR.BYD_ATTO3, CAR.BYD_M6) else CANBUS.main_bus

  def _update_lka_latch_state(self, CS):
    if self.CP.carFingerprint in (CAR.BYD_M6, CAR.BYD_SEAL, CAR.BYD_SEALION7):
      rising_edge = CS.lkas_rdy_btn and not self.prev_press
      if rising_edge:
        if not self.lka_latched:
          self.lka_latched = True
        else:
          self.lka_latched = False
          self.lka_cooldown = 0
          self.lka_active = False
      self.prev_press = CS.lkas_rdy_btn

      if CS.out.brakePressed:
        self.lka_latched = False
        self.lka_cooldown = 0
        self.lka_active = False
      elif self.lka_latched:
        self.lka_active = True
        self.lka_cooldown += 1
    else:
      if CS.lka_on:
        self.lka_cooldown += 1
        self.lka_active = True
      if not CS.lka_on and CS.lkas_rdy_btn:
        self.lka_active = False
        self.lka_cooldown = 0

  def _compute_apply_angle(self, CS, actuators, lat_active):
    apply_angle = CS.out.steeringAngleDeg
    if not lat_active:
      return apply_angle

    apply_angle = lowpass_1pole(actuators.steeringAngleDeg, self.last_apply_angle)
    apply_angle = apply_std_steer_angle_limits(
      apply_angle,
      self.last_apply_angle,
      CS.out.vEgo,
      CS.out.steeringAngleDeg,
      lat_active,
      CarControllerParams.ANGLE_LIMITS,
    )
    apply_angle = float(np.clip(apply_angle,
                                CS.out.steeringAngleDeg - MAX_STEER_ANGLE_OFFSET_DEG,
                                CS.out.steeringAngleDeg + MAX_STEER_ANGLE_OFFSET_DEG))
    self.last_apply_angle = apply_angle
    return apply_angle

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel

    self._update_lka_latch_state(CS)

    lat_active = (self.lka_cooldown > LKA_COOLDOWN_MIN_FRAMES) and enabled and self.lka_active and not CS.out.standstill

    apply_angle = CS.out.steeringAngleDeg
    if (self.frame % 2) == 0:
      apply_angle = self._compute_apply_angle(CS, actuators, lat_active)
      can_sends.append(
        create_can_steer_command(
          self.packer,
          apply_angle,
          lat_active,
          CS.out.standstill,
          CS.lkas_healthy,
          CS.lkas_rdy_btn or CS.out.brakePressed,
        )
      )
      can_sends.append(
        create_lkas_hud(
          self.packer,
          lat_active,
          CS.lss_state,
          CS.lss_alert,
          CS.tsr,
          CS.abh,
          CS.passthrough,
          CS.HMA,
          CS.pt2,
          CS.pt3,
          CS.pt4,
          CS.pt5,
          CS.lka_on,
        )
      )

      if self.CP.openpilotLongitudinalControl:
        long_active = CC.enabled and not CS.out.gasPressed
        brake_hold = CS.out.standstill and actuators.accel < 0
        can_sends.append(create_accel_command(self.packer, actuators.accel, long_active, self.accel_mult, brake_hold))
      else:
        if CS.out.genericToggle or (CS.out.standstill and CC.enabled and (self.frame % BUTTON_KEEPALIVE_FRAMES == 0)):
          can_sends.append(send_buttons(self.packer, 1, 0, self.button_send_bus))

    if self.CP.carFingerprint in (CAR.BYD_M6, CAR.BYD_SEAL):
      cycle_position = self.frame % SPOOF_CYCLE_FRAMES
      spoof_active = cycle_position < SPOOF_DURATION_FRAMES

      if (self.frame % 5) == 0:
        can_sends.append(
          create_steering_torque_spoof_camera(self.packer, lat_active, CS.out.steeringTorque, spoof_active)
        )

    if pcm_cancel_cmd:
      can_sends.append(send_buttons(self.packer, 0, 1, self.button_send_bus))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
