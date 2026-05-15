import numpy as np

from opendbc.can.packer import CANPacker

from opendbc.car.cherry.cherrycan import create_lane_keep_command, create_lkas_info_torque_spoof
from opendbc.car.cherry.values import CarControllerParams, DBC, cherry_steering_deg_sign
from opendbc.car.interfaces import CarControllerBase

SPOOF_DURATION_FRAMES = 50
SPOOF_CYCLE_FRAMES = 150


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.last_apply_angle = None
    self.steering_deg_sign = cherry_steering_deg_sign(CP)

  def _compute_apply_angle(self, CS, actuators, lat_active):
    meas_deg = CS.out.steeringAngleDeg
    if not lat_active:
      return meas_deg

    limits = CarControllerParams.ANGLE_LIMITS
    prev = self.last_apply_angle if self.last_apply_angle is not None else meas_deg
    target = actuators.steeringAngleDeg
    steer_up = prev * target >= 0. and abs(target) > abs(prev)
    rate_limits = limits.ANGLE_RATE_LIMIT_UP if steer_up else limits.ANGLE_RATE_LIMIT_DOWN
    angle_rate_lim = np.interp(CS.out.vEgo, rate_limits[0], rate_limits[1])
    apply_angle = float(np.clip(target, prev - angle_rate_lim, prev + angle_rate_lim))
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

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
