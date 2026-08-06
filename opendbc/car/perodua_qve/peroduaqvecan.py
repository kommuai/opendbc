from opendbc.car.perodua_qve.qve_checksum_data import LUT_ACC, LUT_LKAS, P_ACC, P_LKAS
from opendbc.car.perodua_qve.values import CANBUS

LKAS_ADDR = 0xA5
ACC_ADDR = 0x1AE
LKAS_STEER_NEUTRAL_DEG = 0.0


def qve_checksum(address, sig, d: bytearray) -> int:
  del sig
  if address == LKAS_ADDR:
    x = LUT_LKAS.get(d[1], 0)
    p = P_LKAS
  elif address == ACC_ADDR:
    c = d[1]
    x = LUT_ACC.get(c, LUT_ACC.get(c % 15, 0))
    p = P_ACC
  else:
    return 0
  for i in range(2, 8):
    x ^= p[d[i]]
  return x


def _lkas_byte1_values(counter_byte: int) -> dict[str, int]:
  return {
    "SET_ME_7": 7,
    "COUNTER": counter_byte & 0x0F,
  }


def create_lkas_command(packer, angle_deg: float, active: bool, counter_byte: int):
  cmd = angle_deg if active else LKAS_STEER_NEUTRAL_DEG
  values = {
    **_lkas_byte1_values(counter_byte),
    "STEER_REQ": active,
    "LKAS_B2": 0xFF,
    "SET_ME_1": 1,
    "STEER_CMD": cmd,
    "LKAS_B5": 0x92,
    "LKAS_B6": 0xDB,
    "LKAS_B7": 0x6A,
  }
  return packer.make_can_msg("LKAS", CANBUS.pt, values)


def create_acc_command(packer, acc_req: bool, counter: int):
  return packer.make_can_msg("ADAS_ACC", CANBUS.pt, {
    "COUNTER": counter,
    "ACC_BYTE2": 0xFF if acc_req else 0x00,
    "ACC_REQ": acc_req,
    "ACC_CMD": 0,
    "ACC_BYTE6": 0x60,
  })
