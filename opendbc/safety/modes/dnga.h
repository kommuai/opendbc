#pragma once

#include "opendbc/safety/declarations.h"

static const CanMsg DNGA_TX_MSGS[] = {
  {464, 0, 8, .check_relay = true},  // STEERING_LKAS
  {464, 2, 8, .check_relay = true},  // STEERING_LKAS (alt bus)
  {628, 0, 8, .check_relay = true},  // LKAS_HUD
  {628, 2, 8, .check_relay = true},  // LKAS_HUD (alt bus)
  {625, 0, 8, .check_relay = true},  // ACC_BRAKE
  {625, 2, 8, .check_relay = true},  // ACC_BRAKE (alt bus)
  {627, 0, 8, .check_relay = true},  // ACC_CMD_HUD
  {627, 2, 8, .check_relay = true},  // ACC_CMD_HUD (alt bus)
  {519, 0, 6, .check_relay = false},  // PCM_BUTTONS_HYBRID
  {520, 0, 6, .check_relay = false},  // PCM_BUTTONS
  {2015, 0, 8, .check_relay = false},  // DTC clear
};

static RxCheck dnga_rx_checks[] = {
  // Wheel speed (used for vehicle_moving and for lag bookkeeping).
  {.msg = {{416, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {416, 2, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {0}}},
  // Gas pressed signal (carstate.py: gasPressed = not bool(GAS_PEDAL_2.GAS_PEDAL_STEP)).
  {.msg = {{399, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {399, 2, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {0}}},
  // Brake pressed.
  {.msg = {{161, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {161, 2, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {0}}},
  // Cruise engagement.
  {.msg = {{627, 2, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {627, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {0}}},
  {.msg = {{625, 2, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {625, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .max_counter = 0U, .ignore_quality_flag = true},
           {0}}},
};

static void dnga_rx_hook(const CANPacket_t *msg) {
  // Use decoded signals to drive safety bookkeeping. controls_allowed is driven by pcm_cruise_check().

  if (msg->addr == 416U) {
    // WHEEL_SPEED.WHEELSPEED_F: 7|24@0+ (little-endian)
    uint32_t wheelspeed_raw = ((msg->data[0] >> 7U) & 0x1U) |
                               ((uint32_t)msg->data[1] << 1U) |
                               ((uint32_t)msg->data[2] << 9U) |
                               ((uint32_t)(msg->data[3] & 0x7FU) << 17U);
    vehicle_moving = wheelspeed_raw != 0U;
  }

  if (msg->addr == 399U) {
    // carstate.py: gasPressed = not bool(GAS_PEDAL_2.GAS_PEDAL_STEP)
    gas_pressed = !GET_BIT(msg, 1U);
  }

  if (msg->addr == 161U) {
    // BRAKE.BRAKE_ENGAGED: bit 5|1@0+.
    brake_pressed = GET_BIT(msg, 5U);
  }

  bool cruise_engaged = false;
  if (msg->addr == 627U) {
    cruise_engaged = GET_BIT(msg, 13U) || GET_BIT(msg, 37U) || GET_BIT(msg, 38U);
  } else if (msg->addr == 625U) {
    cruise_engaged = GET_BIT(msg, 8U);
  }

  if (cruise_engaged) {
    pcm_cruise_check(true);
  } else if (msg->addr == 627U || msg->addr == 625U) {
    pcm_cruise_check(false);
  }
}

static bool dnga_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  // ACC_CMD_HUD: if engagement isn’t requested, ACC_CMD should be 0.
  if (msg->addr == 627U) {
    bool set_1_when_engage = GET_BIT(msg, 13U);
    if (!set_1_when_engage) {
      uint32_t acc_cmd_raw = ((msg->data[2] >> 7U) & 0x1U) |
                               ((uint32_t)msg->data[3] << 1U) |
                               ((uint32_t)(msg->data[4] & 0x7FU) << 9U);
      if (acc_cmd_raw != 0U) {
        tx = false;
      }
    }
  }

  // ACC_BRAKE: when engagement isn’t requested, BRAKE_REQ should be 0.
  if (msg->addr == 625U) {
    bool set_me_1_when_engage = GET_BIT(msg, 8U);
    if (!set_me_1_when_engage) {
      bool brake_req = GET_BIT(msg, 13U);
      if (brake_req) {
        tx = false;
      }
    }
  }

  SAFETY_UNUSED(msg);
  return tx;
}

static safety_config dnga_init(uint16_t param) {
  SAFETY_UNUSED(param);
  controls_allowed = false;
  return BUILD_SAFETY_CFG(dnga_rx_checks, DNGA_TX_MSGS);
}

const safety_hooks dnga_hooks = {
  .init = dnga_init,
  .rx = dnga_rx_hook,
  .tx = dnga_tx_hook,
};
