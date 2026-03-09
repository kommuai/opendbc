#pragma once

#include "opendbc/safety/declarations.h"

// Proton (X50, X70, X90, etc.) - ported from panda safety

static bool proton_using_stock_acc = false;

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
  if (param == 2U) {
    proton_using_stock_acc = true;
  }
  static const CanMsg PROTON_TX_MSGS[] = {
    {432, 0, 8, .check_relay = true},
    {417, 0, 8, .check_relay = true},
    {643, 2, 8, .check_relay = false},
  };

  static RxCheck proton_rx_checks[] = {
    // empty; vehicle_moving/controls_allowed set in rx_hook
  };

  return BUILD_SAFETY_CFG(proton_rx_checks, PROTON_TX_MSGS);
}

const safety_hooks proton_hooks = {
  .init = proton_init,
  .rx = proton_rx_hook,
  .tx = proton_tx_hook,
  .fwd = proton_fwd_hook,
};
