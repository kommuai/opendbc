#!/usr/bin/env python3
from cereal import car
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.dnga.carcontroller import CarController, CarControllerParams
from opendbc.car.dnga.carstate import CarState
from opendbc.car.dnga.values import CAR

class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = "dnga"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.dnga)]
    ret.safetyConfigs[0].safetyParam = 1

    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerLimitTimer = 0.01  # DNGA EPS torque authority is low.
    ret.steerActuatorDelay = 0.48

    ret.lateralTuning.init("pid")

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.7933

    ret.openpilotLongitudinalControl = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.0], [255]]
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.0], [0.0]]
    ret.longitudinalTuning.kpBP = [0.0, 5.0, 20.0]
    # Perodua control is speed-command based; avoid longitudinal integrator windup.
    ret.longitudinalTuning.kiBP = [0.0]
    ret.longitudinalTuning.kiV = [0.0]
    ret.longitudinalTuning.kpV = [2.2, 2.0, 1.8]

    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.0, 20.0], [0.0, 20.0]]
    # Higher low-speed integral helps reach steering-limit feedback promptly.
    ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.08, 0.12], [0.10, 0.14]]

    wheel_speed_factor = 1.0
    lateral_kf = 0.00015
    long_kp_v = [2.2, 2.0, 1.8]

    if candidate == CAR.ALZA:
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.14, 0.18], [0.18, 0.25]]
      lateral_kf = 0.00015
      long_kp_v = [0.1, 1.2, 1.2]
      wheel_speed_factor = 1.425
    elif candidate == CAR.ATIVA:
      lateral_kf = 0.000188
      wheel_speed_factor = 1.505
    elif candidate == CAR.MYVI:
      lateral_kf = 0.00012
      wheel_speed_factor = 1.31
    elif candidate == CAR.VIOS:
      lateral_kf = 0.00018
      wheel_speed_factor = 1.43
    elif candidate == CAR.QC:
      lateral_kf = 0.00018
      wheel_speed_factor = 1.43
      ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.noOutput)]
    else:
      ret.dashcamOnly = True

    ret.lateralTuning.pid.kf = lateral_kf
    ret.longitudinalTuning.kpV = long_kp_v
    ret.wheelSpeedFactor = wheel_speed_factor

    ret.minEnableSpeed = -1
    ret.stopAccel = -1.0
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.10

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    events = self.create_common_events(ret)
    ret.events = events.to_msg()
    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
