#pragma once

#include "opendbc/safety/declarations.h"

static const CanMsg DNGA_TX_MSGS[] = {
  {464, 0, 8, .check_relay = false},  // STEERING_LKAS
  {628, 0, 8, .check_relay = false},  // LKAS_HUD
  {625, 0, 8, .check_relay = false},  // ACC_CMD_HUD
  {627, 0, 8, .check_relay = false},  // ACC_BRAKE
};

static RxCheck dnga_rx_checks[] = {
  // {.msg = {{0x35F, 0, 8, 20U}, {0}, {0}}},
};

static void dnga_rx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);

  // DNGA is treated as never standstill for now.
  vehicle_moving = true;
  controls_allowed = true;
}

static bool dnga_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return true;
}

static bool dnga_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == 2) {
    bool is_lkas_msg = (addr == 464) || (addr == 628);
    bool is_acc_msg = (addr == 625) || (addr == 627);
    block_msg = is_lkas_msg || is_acc_msg;
  }

  return block_msg;
}

static safety_config dnga_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = true;
  return BUILD_SAFETY_CFG(dnga_rx_checks, DNGA_TX_MSGS);
}

const safety_hooks dnga_hooks = {
  .init = dnga_init,
  .rx = dnga_rx_hook,
  .tx = dnga_tx_hook,
  .fwd = dnga_fwd_hook,
};
