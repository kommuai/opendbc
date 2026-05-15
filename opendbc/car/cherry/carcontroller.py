import math

import numpy as np

from opendbc.can.packer import CANPacker

from opendbc.car import DT_CTRL
from opendbc.car.cherry.cherrycan import (
  create_lane_keep_command,
  create_lkas_info_torque_spoof,
  create_pcm_icc_toggle_press,
)
from opendbc.car.cherry.values import CANBUS, CarControllerParams, DBC, cherry_steering_deg_sign
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

# BYD carcontroller: 2 Hz one-pole on steering command before rate/angle limits (LANE_KEEP @ 50 Hz).
STEER_LOWPASS_HZ = 2.0
STEER_DT = 0.02

SPOOF_DURATION_FRAMES = 50
SPOOF_CYCLE_FRAMES = 150


def lowpass_1pole(x: float, y_prev: float | None) -> float:
  if y_prev is None:
    return x
  alpha = math.exp(-2.0 * math.pi * STEER_LOWPASS_HZ * STEER_DT)
  return alpha * y_prev + (1.0 - alpha) * x


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.last_apply_angle = None
    self.steering_deg_sign = cherry_steering_deg_sign(CP)
    self.last_resume_button_frame = 0

  def _compute_apply_angle(self, CS, actuators, lat_active):
    meas_deg = CS.out.steeringAngleDeg
    if not lat_active:
      return meas_deg

    limits = CarControllerParams.ANGLE_LIMITS
    prev = self.last_apply_angle if self.last_apply_angle is not None else meas_deg
    after_lowpass = lowpass_1pole(actuators.steeringAngleDeg, self.last_apply_angle)
    apply_angle = apply_std_steer_angle_limits(
      after_lowpass,
      prev,
      CS.out.vEgo,
      meas_deg,
      True,
      limits,
    )
    self.last_apply_angle = apply_angle
    return apply_angle

  def update(self, CC, CS, now_nanos):
    del now_nanos
    can_sends = []

    enabled = CC.latActive
    actuators = CC.actuators

    # On observed J7 logs, stock LKAS enable bits can stay low even while openpilot is engaged.
    # Gate only on openpilot lateral activation + vehicle motion so we don't force LKAS_ENABLE=0.
    lat_active = enabled and not CS.out.standstill

    # Release LKAS_ENABLE while the driver is on the wheel (route 2026-05-14--07-49-04: avoids EPS fault).
    driver_overriding = (
      CS.out.steeringPressed
      or getattr(CS, "steer_related_intervention", False)
    )
    steer_req = lat_active and not driver_overriding

    apply_angle = CS.out.steeringAngleDeg
    if (self.frame % 2) == 0:
      apply_angle = self._compute_apply_angle(CS, actuators, steer_req)
      can_sends.append(
        create_lane_keep_command(
          self.packer,
          apply_angle,
          steer_req,
          CS.out.steeringAngleDeg,
          self.steering_deg_sign,
        )
      )

    # LKAS_INFO MAIN_TORQUE spoof (BYD STEERING_TORQUE pattern) — 20 Hz, intermittent cycle.
    if (self.frame % 5) == 0:
      cycle_position = self.frame % SPOOF_CYCLE_FRAMES
      spoof_active = cycle_position < SPOOF_DURATION_FRAMES
      # MAIN_TORQUE mirrors EPS DRIVER_TORQUE; skip fake offset while driver is steering.
      can_sends.append(
        create_lkas_info_torque_spoof(
          self.packer,
          lat_active,
          CS.out.steeringTorque,
          spoof_active,
          lkas_enable=steer_req,
          steer_related=getattr(CS, "lkas_info_steer_related", 0.0),
          apply_spoof_offset=not driver_overriding,
        )
      )

    # Stock ACC resume from standstill: ICC_TOGGLE on PCM_BUTTONS (Chrysler/Honda-style rate limit).
    if (
      not self.CP.openpilotLongitudinalControl
      and CC.cruiseControl.resume
      and not CS.out.brakePressed
      and (self.frame - self.last_resume_button_frame) * DT_CTRL > 0.05
    ):
      self.last_resume_button_frame = self.frame
      pcm_ctr = (getattr(CS, "pcm_button_counter", 0) + 1) % 16
      can_sends.append(create_pcm_icc_toggle_press(self.packer, pcm_ctr))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
