#pragma once

#include "opendbc/safety/declarations.h"

static bool proton_using_stock_acc = false;
static uint8_t proton_crc8_lut_8h2f[256];
static uint8_t proton_resume_window_frames = 0U;

#define PROTON_ACC_CMD         0x1A1U  // 417
#define PROTON_ADAS_LKAS       0x1B0U  // 432
#define PROTON_ACC_BUTTONS     0x283U  // 643
#define PROTON_PCM_BUTTONS     0x1A3U  // 419
#define PROTON_GAS_PEDAL       0x084U  // 132
#define PROTON_PARKING_BRAKE   0x125U  // 293
#define PROTON_WHEEL_SPEED     0x122U  // 290

#define PROTON_RESUME_WINDOW_MAX 60U

static void proton_start_resume_window(void) {
  proton_resume_window_frames = PROTON_RESUME_WINDOW_MAX;
}

static void proton_tick_resume_window(void) {
  if (proton_resume_window_frames > 0U) {
    proton_resume_window_frames--;
  }
}

static uint8_t proton_get_counter(const CANPacket_t *msg) {
  uint8_t counter = 0U;
  if (msg->addr == PROTON_ACC_CMD) {
    counter = msg->data[5] & 0xFU;
  } else if (msg->addr == PROTON_ADAS_LKAS) {
    counter = msg->data[6] & 0xFU;
  } else if (msg->addr == PROTON_PCM_BUTTONS) {
    counter = msg->data[6] & 0xFU;
  }
  return counter;
}

static uint32_t proton_get_checksum(const CANPacket_t *msg) {
  return msg->data[7];
}

static uint32_t proton_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint8_t crc = 0xFFU;
  for (int i = 0; i < (len - 1); i++) {
    crc ^= msg->data[i];
    crc = proton_crc8_lut_8h2f[crc];
  }
  return (uint8_t)(crc ^ 0xFFU);
}

static void proton_rx_hook(const CANPacket_t *msg) {
  const int bus = (int)msg->bus;
  const int addr = (int)msg->addr;

  // Keep gas tracked for generic safety bookkeeping.
  if ((bus == 0) && (addr == (int)PROTON_GAS_PEDAL)) {
    gas_pressed = (msg->data[2] > 0U);
  }

  // Conservative default to avoid unexpected disengage behavior changes.
  vehicle_moving = true;

  // Driver input path for enable/cancel semantics.
  if ((bus == 0) && (addr == (int)PROTON_ACC_BUTTONS)) {
    bool res_btn = GET_BIT(msg, 3U);      // RES_BUTTON
    bool set_btn = GET_BIT(msg, 4U);      // SET_BUTTON
    bool cancel_btn = GET_BIT(msg, 15U);  // CRUISE_BTN

    if (res_btn || set_btn) {
      controls_allowed = true;
      proton_start_resume_window();
    }
    if (cancel_btn) {
      controls_allowed = false;
      proton_resume_window_frames = 0U;
    }
  }

  // Engagement source mirrors carState behavior.
  if ((bus == 2) && (addr == (int)PROTON_ACC_CMD)) {
    bool acc_req = GET_BIT(msg, 36U);         // ACC_REQ
    bool standstill_req = GET_BIT(msg, 38U);  // STANDSTILL_REQ
    pcm_cruise_check(acc_req || standstill_req);

    if (acc_req) {
      proton_resume_window_frames = 0U;
    }
  }
}

static bool proton_tx_hook(const CANPacket_t *msg) {
  bool violation = false;
  const int addr = (int)msg->addr;

  if (addr == (int)PROTON_ADAS_LKAS) {
    bool steer_req = GET_BIT(msg, 12U);    // LKAS_ENGAGED1
    bool line_active = GET_BIT(msg, 25U);  // LKAS_LINE_ACTIVE
    bool set_me_1 = GET_BIT(msg, 9U);      // SET_ME_1

    if (!set_me_1) {
      violation = true;
    }
    if (steer_req != line_active) {
      violation = true;
    }

    // When steer request is inactive, command must be zero.
    int steer_cmd = (GET_BYTES(msg, 4, 2) >> 5) & 0x7FFU;  // STEER_CMD
    if (!steer_req && (steer_cmd != 0)) {
      violation = true;
    }

    // Allow KA2's extra hidden steering bit used beyond stock behaviour.
    if (steer_cmd > 599) {
      violation = true;
    }
  }

  if (addr == (int)PROTON_ACC_CMD) {
    if (proton_using_stock_acc) {
      violation = true;
    }

    bool acc_req = GET_BIT(msg, 36U);         // ACC_REQ
    bool cruise_disabled = GET_BIT(msg, 12U); // CRUISE_DISABLED
    bool set_me_1 = GET_BIT(msg, 33U);        // SET_ME_1

    if (!set_me_1) {
      violation = true;
    }
    if (acc_req && cruise_disabled) {
      violation = true;
    }

    proton_tick_resume_window();
  }

  if (addr == (int)PROTON_ACC_BUTTONS) {
    bool res_btn = GET_BIT(msg, 3U);          // RES_BUTTON
    bool set_btn = GET_BIT(msg, 4U);          // SET_BUTTON
    bool set_me_pressed = GET_BIT(msg, 43U);  // SET_ME_BUTTON_PRESSED

    // Keep button tx permissive to avoid blocking neutral button frames.
    if (res_btn || set_btn || set_me_pressed) {
      proton_start_resume_window();
    }
  }

  return !violation;
}

static bool proton_fwd_hook(int bus_num, int addr) {
  if (bus_num == 0) {
    return false;
  }
  if (bus_num == 2) {
    bool is_lkas_msg = (addr == (int)PROTON_ADAS_LKAS);
    bool is_acc_msg = (addr == (int)PROTON_ACC_CMD) && !proton_using_stock_acc;
    return is_lkas_msg || is_acc_msg;
  }
  return false;
}

static safety_config proton_init(uint16_t param) {
  gen_crc_lookup_table_8(0x2F, proton_crc8_lut_8h2f);
  proton_using_stock_acc = (param == 2U);
  proton_resume_window_frames = 0U;
  controls_allowed = false;

  static const CanMsg PROTON_TX_MSGS[] = {
    {PROTON_ADAS_LKAS, 0, 8, .check_relay = true},
    {PROTON_ACC_CMD, 0, 8, .check_relay = true},
    {PROTON_ACC_BUTTONS, 2, 8, .check_relay = false},
  };

  static RxCheck proton_rx_checks[] = {
    // Core control path.
    {.msg = {{PROTON_ACC_CMD, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PROTON_ADAS_LKAS, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},

    // Engagement sources.
    {.msg = {{PROTON_ACC_BUTTONS, 0, 8, 10U, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PROTON_PCM_BUTTONS, 2, 8, 20U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},

    // Additional liveness anchors used by control state.
    {.msg = {{PROTON_GAS_PEDAL, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PROTON_PARKING_BRAKE, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PROTON_WHEEL_SPEED, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
  };

  return BUILD_SAFETY_CFG(proton_rx_checks, PROTON_TX_MSGS);
}

const safety_hooks proton_hooks = {
  .init = proton_init,
  .rx = proton_rx_hook,
  .tx = proton_tx_hook,
  .fwd = proton_fwd_hook,
  .get_counter = proton_get_counter,
  .get_checksum = proton_get_checksum,
  .compute_checksum = proton_compute_checksum,
};
