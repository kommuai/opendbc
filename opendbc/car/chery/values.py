import math
from enum import Enum

from opendbc.car import CarSpecs, PlatformConfig, Platforms, dbc_dict
from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.docs_definitions import CarDocs, CUSTOM_CAR_PARTS, CarFootnote, Column
from opendbc.car.lateral import AngleSteeringLimits


# --- CAN buses ---
class CANBUS:
  main_bus = 0   # PT / EPS  — LANE_KEEP + LKAS_INFO TX
  cam_bus = 2    # stock cam — HUD, LANE_KEEP RX


# --- CarController timing ---
LANE_KEEP_STEP = 2   # 50 Hz @ DT_CTRL=0.01
LKAS_INFO_STEP = 5   # 20 Hz

# 10 Hz lowpass on the OP angle command (barely filters at 50 Hz LANE_KEEP).
STEER_LOWPASS_ALPHA = math.exp(-2.0 * math.pi * 10.0 * 0.02)

# Auto-resume from standstill alternates one RES burst then one SET burst:
# stock RES bumps set-speed by +1 km/h, so SET cancels that drift.
AUTORESUME_CYCLE_S = 1.2
AUTORESUME_BURST_FRAMES = 4


# --- CarState ---
GEAR_MAP = {1: "P", 2: "R", 3: "N", 4: "D"}
# HUD FOLLOW_DISTANCE: raw 1 = 1-bar (closest) … raw 5 = 5-bar (farthest); 0/6/7 unknown.
FOLLOW_RAW_TO_PERSONALITY = {1: 0, 2: 0, 3: 1, 4: 2, 5: 2}  # 0 aggressive / 1 standard / 2 relaxed
STEER_RELATED_INTERVENTION_RAW_MIN = 36000

PT_PARSER_MSGS = [
  ("WHEELSPEED_1", 50), ("WHEELSPEED_2", 50), ("EPS", 100), ("GAS", 100),
  ("TRANSMISSION", 100), ("BRAKE_PEDAL", 50), ("STALK", 50), ("PCM_BUTTONS", 20),
  ("ADAS_RELATED", 100), ("SPEED_RELATED", 50), ("STEER_RELATED", 100),
  ("SEATBELT_287", 50), ("SEATBELT_430", 50), ("BCM_STAT_412", math.nan),
  ("BCM_STAT_465", math.nan), ("LKAS_INFO", 50),
]
CAM_PARSER_MSGS = [("HUD", 20), ("LANE_KEEP", 50), ("ACC_UNCERTAIN", 20)]


# --- cherycan TX constants ---
LANE_KEEP_PADDING = {
  "SET_ME_XFF": 255, "SET_ME_XFC": 252, "SET_ME_XF4": 244, "SET_ME_X63": 99, "SET_ME_XF": 15,
}

# LKAS torque spoof params: keep stock "hands on wheel" detector quiet while LKAS is active.
SPOOF_TORQUE_MIN = 5.0
SPOOF_TORQUE_VAR_MIN = 4.0
SPOOF_TORQUE_RAMP = 3.0
SPOOF_TORQUE_MAX = 10.0
SPOOF_NEG_PROB = 0.3
SPOOF_VAR_PROB = 0.2


def lowpass_steer_cmd(x: float, y_prev: float | None) -> float:
  if y_prev is None:
    return x
  return STEER_LOWPASS_ALPHA * y_prev + (1.0 - STEER_LOWPASS_ALPHA) * x


class CarControllerParams:
  STEER_ANGLE_MAX = 120.0
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[50.0, 40.0, 25.0])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[60.0, 50.0, 30.0])
  ANGLE_LIMITS = AngleSteeringLimits(STEER_ANGLE_MAX, ANGLE_RATE_LIMIT_UP, ANGLE_RATE_LIMIT_DOWN)


# --- Docs / platform ---
class Footnote(Enum):
  J7_NOTE = CarFootnote(
    "Support: under validation on chery_general_pt.dbc (Jaecoo J7 PHEV).",
    Column.LONGITUDINAL,
  )


class CAR(Platforms):
  CHERY_JAECOO_J7_PHEV = PlatformConfig(
    [CarDocs(
      "Jaecoo J7 PHEV 2024-26", "ALL",
      car_parts=CUSTOM_CAR_PARTS(),
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      kommu_supported=True,
      acc_low_speed=True,
      acc_speed_range="0 - 150",
      acc_stop_and_go=True,
      lkc_torque="TBD",
      lkc_speed_range="0 - 150",
      max_steering_angle="TBD",
    )],
    CarSpecs(mass=1980.0, wheelbase=2.67, steerRatio=16.0),
    dbc_dict("chery_general_pt", None),
  )


DBC = CAR.create_dbc_map()
