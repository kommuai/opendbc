"""Cherry ADAS TX helpers — cherry_general_pt.dbc.

LANE_KEEP TX bus: the stock camera transmits 0x345 on the camera leg (panda bus 2);
we inject ours on the vehicle/ECU leg (panda bus 0). Cherry safety blocks bus 2 -> bus 0
forwarding of 0x345 (see opendbc/safety/modes/cherry.h::cherry_fwd_hook) so the ECU only
sees openpilot's frame when the relay is open. TX-ing on bus 2 instead drops the frame
because cherry's TX whitelist binds 0x345 to bus 0 only.

LKAS_INFO (0x394) torque spoof: same bus as CarState (PT / bus 0). MAIN_TORQUE is ramped
like BYD STEERING_TORQUE spoof; COUNTER/CHECKSUM come from CANPacker + cherry_checksum.
"""

import random

from opendbc.car import rate_limit
from opendbc.car.chrysler.chryslercan import chrysler_checksum
from opendbc.car.crc import CRC8J1850
from opendbc.car.cherry.values import CANBUS

SPOOF_TARGET_MIN = 5.0
SPOOF_TARGET_VARIATION_MIN = 4.0
SPOOF_TARGET_NEGATIVE_PROB = 0.3
SPOOF_VARIATION_PROB = 0.2

# Idle padding bytes (names per DBC); tune from stock logs if ECU is strict.
_LANE_KEEP_PADDING = {
  "SET_ME_XFF": 255,
  "SET_ME_XFC": 252,
  "SET_ME_XF4": 244,
  "SET_ME_X63": 99,
  "SET_ME_XF": 15,
}


def cherry_checksum(address: int, sig, d: bytearray) -> int:
  """CRC-8/J1850 profile over bytes 0..len-2 with Cherry-specific final XOR (0x0A)."""
  del address, sig
  crc = 0x00
  for i in range(len(d) - 1):
    crc ^= d[i]
    crc = CRC8J1850[crc]
  return crc ^ 0x0A


def cherry_pcm_buttons_checksum(address: int, sig, d: bytearray) -> int:
  """PCM_BUTTONS (0x360): CHECKSUM_BUTTONS in byte 0 over payload bytes 1..5.

  Matches stock Jaecoo J7 frames on route 2026-05-14--07-49-04 (Chrysler-style CRC).
  """
  del address, sig
  tmp = bytearray(list(d[1:6]) + [0])
  return chrysler_checksum(0, None, tmp)


def create_lane_keep_command(
  packer,
  steer_angle_deg: float,
  steer_req: bool,
  meas_angle_deg: float,
  steering_deg_sign: float = 1.0,
):
  """LANE_KEEP (0x345) on ADAS bus — STEER_CMD_ANGLE (DBC scale 0.1, offset -780.1, same as EPS).

  steer_angle_deg / meas_angle_deg are openpilot convention (+ = left). steering_deg_sign maps to ECU CAN
  (Jaecoo J7: +1.0 via cherry_steering_deg_sign). When steer_req is false, command tracks measured angle (LKAS_ENABLE=0).
  """
  cmd_angle = steering_deg_sign * (steer_angle_deg if steer_req else meas_angle_deg)
  values = {
    "STEER_CMD_ANGLE": float(cmd_angle),
    "LKAS_ENABLE": 1 if steer_req else 0,
    **_LANE_KEEP_PADDING,
  }
  return packer.make_can_msg("LANE_KEEP", CANBUS.main_bus, values)


# Module-level state for realistic torque ramp-up (BYD create_steering_torque_spoof_camera pattern).
_torque_spoof_state = {
  "current_torque_offset": 0.0,
  "target_torque": 0.0,
  "ramp_rate": 3.0,
  "max_torque": 10.0,
}


def create_lkas_info_torque_spoof(
  packer,
  lat_active: bool,
  main_torque: float,
  spoof_active: bool,
  lkas_enable: bool | None = None,
  steer_related: float = 0.0,
  apply_spoof_offset: bool = True,
):
  """LKAS_INFO (0x394) on panda bus 0 (PT) — not bus 2.

  main_torque should be EPS DRIVER_TORQUE (same units as LKAS_INFO MAIN_TORQUE in DBC).
  spoof_active: BYD-style intermittent cycle; ramps fake offset on top of main_torque.
  apply_spoof_offset: set False when the driver is on the wheel — use real EPS torque only.
  lkas_enable: defaults to lat_active; cleared by carcontroller when driver overrides.
  steer_related: passthrough from last stock LKAS_INFO frame (physical 0.0 at rest).
  """
  if spoof_active and apply_spoof_offset:
    if abs(_torque_spoof_state["target_torque"]) < 0.1:
      _torque_spoof_state["target_torque"] = random.uniform(SPOOF_TARGET_MIN, _torque_spoof_state["max_torque"])
      if random.random() < SPOOF_TARGET_NEGATIVE_PROB:
        _torque_spoof_state["target_torque"] = -_torque_spoof_state["target_torque"]

    target = _torque_spoof_state["target_torque"]
    current = _torque_spoof_state["current_torque_offset"]
    ramp = _torque_spoof_state["ramp_rate"]
    if abs(target - current) > 0.1:
      _torque_spoof_state["current_torque_offset"] = rate_limit(target, current, -ramp, ramp)
    else:
      if random.random() < SPOOF_VARIATION_PROB:
        _torque_spoof_state["target_torque"] = random.uniform(
          SPOOF_TARGET_VARIATION_MIN, _torque_spoof_state["max_torque"]
        )
        if random.random() < 0.5:
          _torque_spoof_state["target_torque"] = -_torque_spoof_state["target_torque"]
  else:
    current = _torque_spoof_state["current_torque_offset"]
    if abs(current) > 0.1:
      ramp = _torque_spoof_state["ramp_rate"]
      _torque_spoof_state["current_torque_offset"] = rate_limit(0.0, current, -ramp, ramp)
    else:
      _torque_spoof_state["current_torque_offset"] = 0.0
      _torque_spoof_state["target_torque"] = 0.0

  main_torque = float(main_torque) + _torque_spoof_state["current_torque_offset"]
  main_torque = max(0.0, min(main_torque, 1023.0))

  if lkas_enable is None:
    lkas_enable = lat_active

  values = {
    "MAIN_TORQUE": main_torque,
    "LKAS_ENABLE": 1 if lkas_enable else 0,
    "STEER_RELATED": float(steer_related),
  }
  return packer.make_can_msg("LKAS_INFO", CANBUS.main_bus, values)


def create_pcm_icc_toggle_press(packer, counter: int):
  """PCM_BUTTONS ICC_TOGGLE pulse for stock ACC resume from standstill (bus 0)."""
  return packer.make_can_msg(
    "PCM_BUTTONS",
    CANBUS.main_bus,
    {
      "ICC_TOGGLE": 1,
      "CRUISE_BUTTON": 0,
      "COUNTER": int(counter) % 16,
    },
  )
