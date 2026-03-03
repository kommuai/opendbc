#pragma once

#include "opendbc/safety/declarations.h"

static const CanMsg DNGA_TX_MSGS[] = {
  {464, 0, 8, .check_relay = true},  // STEERING_LKAS
  {628, 0, 8, .check_relay = true},  // LKAS_HUD
  {625, 0, 8, .check_relay = true},  // ACC_CMD_HUD
  {627, 0, 8, .check_relay = true},  // ACC_BRAKE
  {519, 0, 6, .check_relay = false},  // PCM_BUTTONS_HYBRID
  {520, 0, 6, .check_relay = false},  // PCM_BUTTONS
  {2015, 0, 8, .check_relay = false},  // DTC clear
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

static safety_config dnga_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = true;
  return BUILD_SAFETY_CFG(dnga_rx_checks, DNGA_TX_MSGS);
}

const safety_hooks dnga_hooks = {
  .init = dnga_init,
  .rx = dnga_rx_hook,
  .tx = dnga_tx_hook,
};
