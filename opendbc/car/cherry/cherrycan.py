"""Cherry ADAS TX helpers — cherry_general_pt.dbc.

LANE_KEEP TX bus: the stock camera transmits 0x345 on the camera leg (panda bus 2);
we inject ours on the vehicle/ECU leg (panda bus 0). Cherry safety blocks bus 2 -> bus 0
forwarding of 0x345 (see opendbc/safety/modes/cherry.h::cherry_fwd_hook) so the ECU only
sees openpilot's frame when the relay is open. TX-ing on bus 2 instead drops the frame
because cherry's TX whitelist binds 0x345 to bus 0 only.
"""

from opendbc.car.crc import CRC8J1850
from opendbc.car.cherry.values import CANBUS

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
