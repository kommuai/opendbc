#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/defaults.h"

// Cherry (Jaecoo J7 PHEV, etc.) — permissive bring-up: no RX checks, allow all TX.
// Tighten with real limits and relay/forward rules before production use.
#define CHERRY_LANE_KEEP 0x345U

static safety_config cherry_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = true;
  return (safety_config){NULL, 0, NULL, 0, false}; // NOLINT(readability/braces)
}

static bool cherry_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return true;
}

static bool cherry_fwd_hook(int bus_num, int addr) {
  if (bus_num == 2) {
    return addr == (int)CHERRY_LANE_KEEP;
  }
  return false;
}

const safety_hooks cherry_hooks = {
  .init = cherry_init,
  .rx = default_rx_hook,
  .tx = cherry_tx_hook,
  .fwd = cherry_fwd_hook,
};
