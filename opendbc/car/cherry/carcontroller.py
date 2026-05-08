import numpy as np

from opendbc.can.packer import CANPacker

from opendbc.car.cherry.cherrycan import create_lane_keep_command
from opendbc.car.cherry.values import CarControllerParams, DBC
from opendbc.car.interfaces import CarControllerBase

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]["pt"])
    self.last_apply_angle = None

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

    stock_lka_ok = CS.lkas_enable_lane
    lat_active = enabled and stock_lka_ok and not CS.out.standstill

    apply_angle = CS.out.steeringAngleDeg
    if (self.frame % 2) == 0:
      apply_angle = self._compute_apply_angle(CS, actuators, lat_active)
      can_sends.append(
        create_lane_keep_command(
          self.packer,
          apply_angle,
          lat_active,
          CS.out.steeringAngleDeg,
        )
      )

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
