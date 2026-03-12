#pragma once

#include "opendbc/safety/declarations.h"

// Proton (X50, X70, X90, etc.) - ported from panda safety

static bool proton_using_stock_acc = false;
static uint8_t proton_crc8_lut_8h2f[256];

#define PROTON_ACC_CMD      0x1A1U
#define PROTON_ADAS_LKAS    0x1B0U

static uint8_t proton_get_counter(const CANPacket_t *msg) {
  uint8_t counter = 0U;
  if (msg->addr == PROTON_ACC_CMD) {
    counter = msg->data[5] & 0xFU;
  } else if (msg->addr == PROTON_ADAS_LKAS) {
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
  SAFETY_UNUSED(msg);
  vehicle_moving = true;
  controls_allowed = true;
}

static bool proton_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return true;
}

static bool proton_fwd_hook(int bus_num, int addr) {
  if (bus_num == 0) {
    return false;  // forward to 2
  }
  if (bus_num == 2) {
    bool is_lkas_msg = (addr == 432);
    bool is_acc_msg = (addr == 417) && !proton_using_stock_acc;
    return is_lkas_msg || is_acc_msg;
  }
  return false;
}

static safety_config proton_init(uint16_t param) {
  gen_crc_lookup_table_8(0x2F, proton_crc8_lut_8h2f);

  if (param == 2U) {
    proton_using_stock_acc = true;
  }
  static const CanMsg PROTON_TX_MSGS[] = {
    {432, 0, 8, .check_relay = true},
    {417, 0, 8, .check_relay = true},
    {643, 2, 8, .check_relay = false},
  };

  static RxCheck proton_rx_checks[] = {
    {.msg = {{PROTON_ACC_CMD, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{PROTON_ADAS_LKAS, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
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
