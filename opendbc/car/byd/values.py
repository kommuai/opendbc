from collections import defaultdict
from dataclasses import dataclass, field

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.byd.angle_rate_limit import AngleRateLimit
from opendbc.car.lateral import AngleSteeringLimits
from opendbc.car.docs_definitions import CarDocs

HUD_MULTIPLIER = 1.07


@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("byd_general_pt", "byd_radar_fd"))


class BYDCarDocs(CarDocs):
  pass


class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2


class CAR(Platforms):
  BYD_ATTO3 = BYDPlatformConfig(
    [BYDCarDocs("BYD Atto 3 2023-26", "ALL")],
    CarSpecs(mass=2090.0, wheelbase=2.72, steerRatio=16.0),
  )
  BYD_M6 = BYDPlatformConfig(
    [BYDCarDocs("BYD M6 2024-26", "ALL")],
    CarSpecs(mass=2374.0, wheelbase=2.80, steerRatio=16.0),
  )
  BYD_SEAL = BYDPlatformConfig(
    [BYDCarDocs("BYD Seal 2024-26", "ALL")],
    CarSpecs(mass=2180.0, wheelbase=2.92, steerRatio=16.0),
  )
  BYD_SEALION7 = BYDPlatformConfig(
    [BYDCarDocs("BYD Sealion 7 2024-26", "ALL")],
    CarSpecs(mass=2340.0, wheelbase=2.93, steerRatio=16.0),
  )

DBC = CAR.create_dbc_map()
ACCEL_MULT = defaultdict(
  lambda: 1,
  {
    CAR.BYD_ATTO3: 25,
    CAR.BYD_M6: 25,
    CAR.BYD_SEAL: 1,
    CAR.BYD_SEALION7: 1,
  },
)

class CarControllerParams:
  STEER_ANGLE_MAX = 94.0  # deg
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[6., 3., 1.])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[0., 5., 15.], angle_v=[8., 7., 4.])
  ANGLE_LIMITS = AngleSteeringLimits(STEER_ANGLE_MAX, ANGLE_RATE_LIMIT_UP, ANGLE_RATE_LIMIT_DOWN)

  def __init__(self, CP):
    pass
