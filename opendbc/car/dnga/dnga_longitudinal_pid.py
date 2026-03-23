import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.common.pid import PIDController
from opendbc.car.lateral import apply_center_deadzone

A_TARGET_DIR_EPS = 0.03
SIGN_FLIP_UNWIND_DECAY = 0.33
POSITIVE_INTEGRATOR_CAP = 0.5

DNGA_LONG_KP = 0.55
DNGA_LONG_KI_BP = [0.0, 20.0]
DNGA_LONG_KI_V = [0.15, 0.10]


class DNGAAccelPID:
  def __init__(self, *, pos_limit: float, neg_limit: float, cruise_deadzone: float, min_v_ego: float):
    # Uses the same PIDController implementation as other OpenPilot platforms.
    self.pid = PIDController(
      DNGA_LONG_KP,
      (DNGA_LONG_KI_BP, DNGA_LONG_KI_V),
      k_f=1.0,
      pos_limit=pos_limit,
      neg_limit=neg_limit,
      rate=1 / (DT_CTRL * 5),
    )
    self.cruise_deadzone = float(cruise_deadzone)
    self.min_v_ego = float(min_v_ego)

  def update_long_accel(self, *, long_active: bool, a_target: float, a_ego: float, v_ego: float,
                         prev_a_target: float, freeze_integrator: bool) -> tuple[float, bool]:
    """
    Returns (long_accel, reset_prev_a_target_to_zero).

    reset_prev_a_target_to_zero preserves the original DNGA carcontroller behavior
    when longActive is false (even though the caller overwrites prev_a_target each frame).
    """
    if not long_active:
      self.pid.reset()
      return float(a_target), True

    prev_sign = int(np.sign(prev_a_target)) if abs(prev_a_target) > A_TARGET_DIR_EPS else 0
    curr_sign = int(np.sign(a_target)) if abs(a_target) > A_TARGET_DIR_EPS else 0

    if prev_sign != 0 and curr_sign != 0 and prev_sign != curr_sign:
      self.pid.i *= SIGN_FLIP_UNWIND_DECAY
      if abs(self.pid.i) < 1e-4:
        self.pid.i = 0.0

    if self.pid.i > POSITIVE_INTEGRATOR_CAP:
      self.pid.i = POSITIVE_INTEGRATOR_CAP

    error = float(a_target - a_ego)
    long_accel = float(self.pid.update(
      error,
      speed=v_ego,
      feedforward=float(a_target),
      freeze_integrator=freeze_integrator,
    ))

    if v_ego >= self.min_v_ego:
      long_accel = apply_center_deadzone(long_accel, self.cruise_deadzone)

    return long_accel, False

