from cereal import car
from opendbc.can import CANParser
from opendbc.car import Bus, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.cherry.values import CANBUS, DBC, HUD_MULTIPLIER

ButtonType = car.CarState.ButtonEvent.Type
CHERRY_FOLLOW_RAW_TO_PERSONALITY = {
  1: 2,
  2: 1,
  3: 0,
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
    self.prev_angle = 0.0

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

    ret.steeringAngleDeg = cp.vl["EPS"]["STEERING_ANGLE"]
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

    # HUD is on bus 2 (same DBC, different bus).
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

    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    self.distance_val = int(cp_cam.vl["HUD"]["FOLLOW_DISTANCE"])
    if self.distance_val in CHERRY_FOLLOW_RAW_TO_PERSONALITY:
      ret.personality = CHERRY_FOLLOW_RAW_TO_PERSONALITY[self.distance_val]

    # PCM buttons (minimal edge detection — expand when CAN semantics confirmed)
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

    # TODO
    ret.doorOpen = False
    ret.seatbeltUnlatched = False
    ret.espDisabled = False

    # Bus 2: LANE_KEEP, LKAS_INFO, ACC_UNCERTAIN (see get_cam_can_parser).
    self.lkas_enable_lane = bool(cp_cam.vl["LANE_KEEP"]["LKAS_ENABLE"])
    self.lkas_enable_info = bool(cp_cam.vl["LKAS_INFO"]["LKAS_ENABLE"])

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
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.main_bus)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      ("HUD", 20),
      ("LANE_KEEP", 50),
      ("LKAS_INFO", 50),
      ("ACC_UNCERTAIN", 20),
    ]
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, CANBUS.cam_bus)
