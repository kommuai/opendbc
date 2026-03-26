from opendbc.car.crc import CRC8H2F


def proton_checksum(address: int, sig, d: bytearray) -> int:
  """CRC8 8H2F/AUTOSAR. Checksum in last byte; CRC over bytes 0..size-2, final XOR 0xFF."""
  del address, sig
  crc = 0xFF
  for i in range(len(d) - 1):
    crc ^= d[i]
    crc = CRC8H2F[crc]
  return crc ^ 0xFF


def create_can_steer_command(packer, steer, steer_req, wheel_touch_warning, wheel_touch_warning_2,
                             lks_aux, lks_audio, lks_tactile, lks_assist_mode, lka_enable, stock_ldw_ste,
                             steer_enabled, new_lka):

  # Disable steering vibration for LDW if steer not enabled and LKS set to Warn Only mode and Tactile warning type
  ldw_steering = 0 if (
    not steer_enabled and not lks_aux and lks_assist_mode and lks_tactile and not lks_audio
  ) else stock_ldw_ste

  values = {
    "LKA_ENABLE": lka_enable,
    "LKAS_ENGAGED1": steer_req,
    "LKAS_LINE_ACTIVE": steer_req,
    "STEER_CMD": abs(steer) if steer_req else 0,
    "STEER_DIR": steer <= 0,
    "LDW_READY": 1,
    "LDW_STEERING": ldw_steering,
    "SET_ME_1": 1,
    "SET_ME_1_2": new_lka, # Currently only for X90, pre FL X50 needs to be False or LKS cannot be changed
    "LKS_STATUS": 1,
    "STOCK_LKS_AUX": lks_aux,
    "LKS_WARNING_AUDIO_TYPE": lks_audio,
    "LKS_WARNING_TACTILE_TYPE": lks_tactile,
    "LKS_ASSIST_MODE": lks_assist_mode,
    "HAND_ON_WHEEL_WARNING": wheel_touch_warning,
    "WHEEL_WARNING_CHIME": wheel_touch_warning_2,
  }

  return packer.make_can_msg("ADAS_LKAS", 0, values)


def _acc_cmd_values(accel_cmd, enabled, gas_override, cruise_standstill, resume, brake_pressed, car_standstill):
  standstill = cruise_standstill and not gas_override
  cmd = 35 if gas_override else 0 if not enabled else (-31 if standstill else accel_cmd)
  standstill_req = 0 if gas_override or not enabled else (1 if standstill else 0)
  acc_req = 1 if gas_override or enabled else 0
  cruise_disabled = 0 if gas_override or enabled else 1
  brake_out = 1 if gas_override or brake_pressed or accel_cmd < 0 else 0
  rising = 1 if gas_override or (enabled and resume) else 0
  full_cruise_stand = standstill and car_standstill
  stationary = 1 if (enabled and (not resume) and standstill) else 0
  unknown1 = stationary
  motion = 3 if gas_override else 4 if not enabled else 9 if resume else 5 if cruise_standstill else 4 if accel_cmd < 0 else 6 if accel_cmd > 0 else 1
  x6a = 0xFA if gas_override else (0x6A if not enabled else 0xFA)

  return {
    "ACC_REQ": acc_req,
    "CRUISE_DISABLED": cruise_disabled,
    "CMD": cmd,
    "CMD_OFFSET1": cmd,
    "CMD_OFFSET2": cmd,
    "SET_ME_1": 1,
    "NOT_GAS_OVERRIDE": 1,
    "STANDSTILL_REQ": standstill_req,
    "STATIONARY": stationary,
    "UNKNOWN1": unknown1,
    "BRAKE_ENGAGED": brake_out,
    "RISING_ENGAGE": rising,
    "MOTION_CONTROL": motion,
    "SET_ME_X6A": x6a,
  }


def create_acc_cmd(packer, accel_cmd, enabled, gas_override, cruise_standstill, resume, brake_pressed, car_standstill):
  values = _acc_cmd_values(accel_cmd, enabled, gas_override, cruise_standstill, resume, brake_pressed, car_standstill)
  return packer.make_can_msg("ACC_CMD", 0, values)


def send_buttons(packer, send_cruise=True):
  if send_cruise:
    values = {"CRUISE_BTN": 1}
  else:
    values = {
      "SET_BUTTON": 0,
      "RES_BUTTON": 1,
      "NEW_SIGNAL_1": 1,
      "NEW_SIGNAL_2": 1,
      "SET_ME_BUTTON_PRESSED": 1,
    }

  return packer.make_can_msg("ACC_BUTTONS", 2, values)

def create_acc_cmd_stock(packer, stock_values):
  return packer.make_can_msg("ACC_CMD", 0, stock_values)
