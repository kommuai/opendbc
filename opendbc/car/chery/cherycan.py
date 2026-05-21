"""Chery / Jaecoo ADAS CAN TX (chery_general_pt.dbc)."""

import random
from dataclasses import dataclass, field

from opendbc.car import rate_limit
from opendbc.car.chrysler.chryslercan import chrysler_checksum
from opendbc.car.crc import CRC8J1850
from opendbc.car.chery.values import (
  CANBUS,
  LANE_KEEP_PADDING,
  SPOOF_NEG_PROB,
  SPOOF_TORQUE_MAX,
  SPOOF_TORQUE_MIN,
  SPOOF_TORQUE_RAMP,
  SPOOF_TORQUE_VAR_MIN,
  SPOOF_VAR_PROB,
)

# --- Re-exported / referenced by opendbc.can.dbc; do not rename. ---

def chery_checksum(address: int, sig, d: bytearray) -> int:
  del address, sig
  crc = 0
  for i in range(len(d) - 1):
    crc ^= d[i]
    crc = CRC8J1850[crc]
  return crc ^ 0x0A


def chery_pcm_buttons_checksum(address: int, sig, d: bytearray) -> int:
  del address, sig
  return chrysler_checksum(0, None, bytearray(list(d[1:6]) + [0]))


# --- TX builders ---

def create_lane_keep_command(packer, steer_angle_deg, steer_req, meas_angle_deg):
  cmd = steer_angle_deg if steer_req else meas_angle_deg
  return packer.make_can_msg("LANE_KEEP", CANBUS.main_bus, {
    "STEER_CMD_ANGLE": float(cmd),
    "LKAS_ENABLE": int(steer_req),
    **LANE_KEEP_PADDING,
  })


_PCM_BUTTON_FIELDS = ("ICC_TOGGLE", "CRUISE_BUTTON", "RES_BUTTON")


def create_pcm_button(packer, counter: int, bus: int, button: str):
  """Build a PCM_BUTTONS frame asserting exactly one of ICC_TOGGLE / CRUISE_BUTTON (SET) / RES_BUTTON.

  TX on bus 0 (PT) and bus 2 (camera): panda does not forward our own TX between buses.
  """
  assert button in _PCM_BUTTON_FIELDS, button
  signals = {b: int(b == button) for b in _PCM_BUTTON_FIELDS}
  signals["COUNTER"] = int(counter) % 16
  return packer.make_can_msg("PCM_BUTTONS", bus, signals)


# --- LKAS torque spoof: keeps stock "hands on wheel" detector quiet during LKAS. ---

@dataclass
class _TorqueSpoof:
  offset: float = 0.0
  target: float = 0.0
  ramp: float = field(default=SPOOF_TORQUE_RAMP)
  max_torque: float = field(default=SPOOF_TORQUE_MAX)

  def _pick_target(self) -> None:
    self.target = random.uniform(SPOOF_TORQUE_MIN, self.max_torque)
    if random.random() < SPOOF_NEG_PROB:
      self.target = -self.target

  def _maybe_variate(self) -> None:
    if random.random() < SPOOF_VAR_PROB:
      self.target = random.uniform(SPOOF_TORQUE_VAR_MIN, self.max_torque)
      if random.random() < 0.5:
        self.target = -self.target

  def step(self, active: bool, apply_offset: bool) -> float:
    if active and apply_offset:
      if abs(self.target) < 0.1:
        self._pick_target()
      if abs(self.target - self.offset) > 0.1:
        self.offset = rate_limit(self.target, self.offset, -self.ramp, self.ramp)
      else:
        self._maybe_variate()
    elif abs(self.offset) > 0.1:
      self.offset = rate_limit(0.0, self.offset, -self.ramp, self.ramp)
    else:
      self.offset = self.target = 0.0
    return self.offset


_SPOOF = _TorqueSpoof()


def create_lkas_info_torque_spoof(packer, main_torque, lkas_enable, spoof_active,
                                  steer_related=0.0, apply_spoof_offset=True):
  torque = float(main_torque) + _SPOOF.step(spoof_active, apply_spoof_offset)
  return packer.make_can_msg("LKAS_INFO", CANBUS.main_bus, {
    "MAIN_TORQUE": max(0.0, min(torque, 1023.0)),
    "LKAS_ENABLE": int(lkas_enable),
    "STEER_RELATED": float(steer_related),
  })
