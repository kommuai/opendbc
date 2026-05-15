import math

from cereal import car
from opendbc.can import CANParser
from opendbc.car import Bus, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.cherry.values import (
  CANBUS,
  DBC,
  HUD_MULTIPLIER,
  STEER_RELATED_INTERVENTION_RAW_MIN,
  cherry_steering_deg_sign,
)

ButtonType = car.CarState.ButtonEvent.Type
# HUD FOLLOW_DISTANCE raw 1..5 = 5..1 ACC gap bars (Jaecoo J7); map to openpilot personality 0..2.
CHERRY_FOLLOW_RAW_TO_PERSONALITY = {
  1: 2,  # 5 bars — max gap / relaxed
  2: 2,  # 4 bars
  3: 1,  # 3 bars — standard
  4: 0,  # 2 bars
  5: 0,  # 1 bar — min gap / aggressive
}

_GEAR_TO_STR = {
  1: "P",
  2: "R",
  3: "N",
  4: "D",
}

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.prev_icc = False
    self.prev_cruise = False
    self.distance_val = 1
    self.pcm_button_counter = 0
    self.prev_angle = 0.0
    self.lkas_info_steer_related = 0.0
    self.steer_related_intervention = False

  def update(self, can_parsers):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = car.CarState.new_message()
    ret.personality = -1

    self.parse_wheel_speeds(
      ret,
      cp.vl["WHEELSPEED_2"]["WHEEL_FL"],
      cp.vl["WHEELSPEED_2"]["WHEEL_FR"],
      cp.vl["WHEELSPEED_1"]["WHEEL_BL"],
      cp.vl["WHEELSPEED_1"]["WHEEL_BR"],
    )
    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER

    ret.steeringAngleDeg = cherry_steering_deg_sign(self.CP) * float(cp.vl["EPS"]["STEERING_ANGLE"])
    ret.steeringTorque = cp.vl["EPS"]["DRIVER_TORQUE"]
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorqueEps = ret.steeringTorque * steer_dir
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 10, 5)

    ret.gasPressed = cp.vl["GAS"]["GAS_PEDAL_PRESSURE"] > 0.01
    ret.brake = cp.vl["BRAKE_PEDAL"]["BRAKE_PRESSURE"]
    ret.brakePressed = bool(ret.brake > 0.01)

    gear_raw = int(cp.vl["TRANSMISSION"]["GEAR"])
    gear_str = _GEAR_TO_STR.get(gear_raw)
    ret.gearShifter = self.parse_gear_shifter(gear_str)

    ret.standstill = ret.vEgoRaw < 0.01

    left, right = self.update_blinker_from_stalk(3, cp.vl["STALK"]["LEFT_BLINKER"], cp.vl["STALK"]["RIGHT_BLINKER"])
    ret.leftBlinker = left
    ret.rightBlinker = right

    # HUD / cruise UI on CANBUS.cam_bus (bus 2); LKAS_INFO is parsed from PT (bus 0).
    cruise_state = int(cp_cam.vl["HUD"]["CRUISE_STATE"])
    set_speed_kph = float(cp_cam.vl["HUD"]["SET_SPEED"])
    gas_override = bool(cp_cam.vl["HUD"]["GAS_OVERRIDE"])

    ret.cruiseState.available = True
    ret.cruiseState.enabled = cruise_state == 3
    if set_speed_kph > 0:
      # Confirmed: HUD.SET_SPEED already matches the meter-cluster set speed.
      ret.cruiseState.speedCluster = set_speed_kph * CV.KPH_TO_MS
      ret.cruiseState.speed = ret.cruiseState.speedCluster
    else:
      ret.cruiseState.speedCluster = 0.0
      ret.cruiseState.speed = 0.0

    # Stock ACC hold: vehicle stopped while cruise engaged (gates CC.cruiseControl.resume in controlsd).
    ret.cruiseState.standstill = ret.standstill and ret.cruiseState.enabled
    ret.cruiseState.nonAdaptive = False

    self.distance_val = int(cp_cam.vl["HUD"]["FOLLOW_DISTANCE"])
    if self.distance_val in CHERRY_FOLLOW_RAW_TO_PERSONALITY:
      ret.personality = CHERRY_FOLLOW_RAW_TO_PERSONALITY[self.distance_val]

    # FOLLOW_DISTANCE can be transient or unmapped while cycling the distance button.
    # Keep mapped values in [0, 2]; leave -1 untouched (car.capnp: optional override / unknown).
    if ret.personality != -1:
      ret.personality = max(0, min(int(ret.personality), 2))

    # PCM buttons — ICC_TOGGLE = ICC/LKAS toggle; CRUISE_BUTTON = main cruise (sparse in logs).
    self.pcm_button_counter = int(cp.vl["PCM_BUTTONS"]["COUNTER"])
    icc = bool(cp.vl["PCM_BUTTONS"]["ICC_TOGGLE"])
    cruise_btn = bool(cp.vl["PCM_BUTTONS"]["CRUISE_BUTTON"])
    ret.buttonEvents = (
      create_button_events(icc, self.prev_icc, {1: ButtonType.altButton2}) +
      create_button_events(cruise_btn, self.prev_cruise, {1: ButtonType.mainCruise})
    )
    self.prev_icc = icc
    self.prev_cruise = cruise_btn

    ret.stockAeb = bool(cp_cam.vl["HUD"]["AEB"])
    ret.stockFcw = bool(cp_cam.vl["HUD"]["PCW"])

    # 0x391: door ajar = PAYLOAD391_B3 LSB (0x08 latched vs 0x09 ajar). B1/B6 vary like counter.
    st_vl = cp.vl["STALK"]
    b3 = int(st_vl["PAYLOAD391_B3"])
    ret.doorOpen = bool((b3 & 1) != 0)
    ret.genericToggle = False
    ret.seatbeltUnlatched = bool(
      cp.vl["SEATBELT_287"]["DRIVER_UNBUCKLED"] != 0
      or cp.vl["SEATBELT_430"]["DRIVER_UNBUCKLED"] != 0
    )
    ret.espDisabled = False

    # LANE_KEEP / ACC on CANBUS.cam_bus; LKAS_INFO is on PT (bus 0) for this harness.
    self.lkas_enable_lane = bool(cp_cam.vl["LANE_KEEP"]["LKAS_ENABLE"])
    lkas_vl = cp.vl["LKAS_INFO"]
    self.lkas_enable_info = bool(lkas_vl["LKAS_ENABLE"])
    self.lkas_info_steer_related = float(lkas_vl["STEER_RELATED"])

    sr_raw = int(cp.vl["STEER_RELATED"]["STEERING_ANGLE_NOT_CALIBRATED"])
    # EPS/cluster conflict indicator (route 2026-05-14--07-49-04); used to drop LKAS_ENABLE, not OP steerFault*.
    self.steer_related_intervention = sr_raw >= STEER_RELATED_INTERVENTION_RAW_MIN

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CarState.get_can_parser(CP),
      Bus.cam: CarState.get_cam_can_parser(CP),
    }

  @staticmethod
  def get_can_parser(CP):
    signals = [
      ("WHEELSPEED_1", 50),
      ("WHEELSPEED_2", 50),
      ("EPS", 100),
      ("GAS", 100),
      ("TRANSMISSION", 100),
      ("BRAKE_PEDAL", 50),
      ("STALK", 50),
      ("PCM_BUTTONS", 20),
      ("ADAS_RELATED", 100),
      ("SPEED_RELATED", 50),
      ("STEER_RELATED", 100),
      ("SEATBELT_287", 50),
      ("SEATBELT_430", 50),
      ("BCM_STAT_412", math.nan),
      ("BCM_STAT_465", math.nan),
      ("LKAS_INFO", 50),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("HUD", 20),
      ("LANE_KEEP", 50),
      ("ACC_UNCERTAIN", 20),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.cam_bus)
