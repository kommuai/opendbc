#!/usr/bin/env python3
from cereal import car

from opendbc.car.interfaces import CarControllerBase


class CarController(CarControllerBase):
  def update(self, CC, CS, now_nanos):
    del CC, now_nanos

    new_actuators = car.CarControl.Actuators.new_message()
    new_actuators.steeringAngleDeg = CS.out.steeringAngleDeg
    self.frame += 1
    return new_actuators, []
