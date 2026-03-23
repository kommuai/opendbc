from collections import defaultdict
from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs

HUD_MULTIPLIER = 1.04

@dataclass
class DNGAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("dnga_general_pt", None))

class DNGACarDocs(CarDocs):
  pass

class CANBUS:
  main_bus = 0
  cam_bus = 2

class DNGAFlags(IntFlag):
  HYBRID = 1
  SNG = 2

class CAR(Platforms):
  PERODUA_ALZA = DNGAPlatformConfig(
    [
      DNGACarDocs("Perodua Alza 2022-26", "All"),
      DNGACarDocs("Toyota Veloz 2022-26", "All"),
    ],
    CarSpecs(mass=1170.0, wheelbase=2.750, steerRatio=17.0),
    flags=DNGAFlags.SNG,
  )
  PERODUA_ATIVA = DNGAPlatformConfig(
    [
      DNGACarDocs("Perodua Ativa 2021-26", "All"),
      DNGACarDocs("Perodua Ativa Hybrid 2022", "All"),
      DNGACarDocs("Toyota Raize 2021-26", "All"),
    ],
    CarSpecs(mass=1035.0, wheelbase=2.525, steerRatio=17.0),
  )
  PERODUA_MYVI = DNGAPlatformConfig(
    [DNGACarDocs("Perodua Myvi 2022-26", "All")],
    CarSpecs(mass=1025.0, wheelbase=2.500, steerRatio=18.2),
  )
  TOYOTA_VIOS = DNGAPlatformConfig(
    [DNGACarDocs("Toyota Vios 2023-26", "All")],
    CarSpecs(mass=1035.0, wheelbase=2.620, steerRatio=17.0),
    flags=DNGAFlags.SNG,
  )
  QC = DNGAPlatformConfig(
    [DNGACarDocs("Quality Check 2020-24", "All")],
    CarSpecs(mass=1025.0, wheelbase=2.500, steerRatio=17.4),
  )


BRAKE_SCALE = defaultdict(
  lambda: 1.0,
  {
    CAR.PERODUA_ALZA: 0.65,
    CAR.PERODUA_ATIVA: 0.8,
    CAR.PERODUA_MYVI: 0.8,
    CAR.TOYOTA_VIOS: 0.68,
    CAR.QC: 0.8,
  },
)

SNG_CAR = CAR.with_flags(DNGAFlags.SNG)
HYBRID_CAR = CAR.with_flags(DNGAFlags.HYBRID)
DBC = CAR.create_dbc_map()

class CarControllerParams:
  STEER_STEP = 1
  ACCEL_MIN = -3.5
  ACCEL_MAX = 1.6  # not advisable to go higher because brake may fail
  LONG_CRUISE_ACCEL_DEADZONE = 0.08
  LONG_CRUISE_DEADZONE_MIN_V_EGO = 2.5

  def __init__(self, CP):
    self.STEER_MAX = CP.lateralParams.torqueV[0]
    assert len(CP.lateralParams.torqueV) == 1
    self.STEER_DELTA_UP = 10
    self.STEER_DELTA_DOWN = 30
