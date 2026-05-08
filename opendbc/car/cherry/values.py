from collections import defaultdict
from dataclasses import dataclass, field
from enum import Enum

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.docs_definitions import CarDocs, CarParts, CUSTOM_CAR_PARTS, CarFootnote, Column
from opendbc.car.lateral import AngleSteeringLimits


@dataclass
class CherryPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("cherry_general_pt", None))


@dataclass
class CherryCarDocs(CarDocs):
  car_parts: CarParts = field(default_factory=CUSTOM_CAR_PARTS)


class CANBUS:
  main_bus = 0
  cam_bus = 2


CHERRY_SUPPORT_COMMON_FIELDS = {
  "acc_low_speed": True,
  "acc_speed_range": "0 - 150",
  "acc_stop_and_go": True,
  "lkc_torque": "TBD",
  "lkc_speed_range": "0 - 150",
  "max_steering_angle": "TBD",
}

CHERRY_LKC_ACC_NOTE = CarFootnote(
  "Support: under validation on cherry_general_pt.dbc (Jaecoo J7 PHEV).",
  Column.LONGITUDINAL,
)


class Footnote(Enum):
  J7_NOTE = CHERRY_LKC_ACC_NOTE


class CAR(Platforms):
  CHERRY_JAECOO_J7_PHEV = CherryPlatformConfig(
    [CherryCarDocs(
      "Jaecoo J7 PHEV",
      "ALL",
      footnotes=[Footnote.J7_NOTE],
      variant="All",
      kommu_supported=True,
      **CHERRY_SUPPORT_COMMON_FIELDS,
    )],
    # Approximate published curb/wheelbase; replace when confirmed on-vehicle.
    CarSpecs(mass=1980.0, wheelbase=2.67, steerRatio=16.0),
  )


DBC = CAR.create_dbc_map()
ACCEL_MULT = defaultdict(
  lambda: 1,
  {CAR.CHERRY_JAECOO_J7_PHEV: 1},
)
HUD_MULTIPLIER = 1.0


class CarControllerParams:
  STEER_ANGLE_MAX = 120.0
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[6.0, 4.0, 3.0])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0.0, 5.0, 15.0], angle_v=[8.0, 6.0, 4.0])
  ANGLE_LIMITS = AngleSteeringLimits(STEER_ANGLE_MAX, ANGLE_RATE_LIMIT_UP, ANGLE_RATE_LIMIT_DOWN)

  def __init__(self, CP):
    pass
