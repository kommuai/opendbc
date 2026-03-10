from collections import defaultdict
from dataclasses import dataclass, field
from enum import IntFlag

from cereal import car
from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs as CarInfo, SupportType

HUD_MULTIPLIER = 1.04
Ecu = car.CarParams.Ecu


@dataclass
class DNGAPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("dnga_general_pt", None))


class CANBUS:
  main_bus = 0
  cam_bus = 2


class DNGAFlags(IntFlag):
  HYBRID = 1
  SNG = 2


class CAR(Platforms):
  PERODUA_ALZA = DNGAPlatformConfig(
    [
      CarInfo("Perodua Alza 2022-2026", "All", support_type=SupportType.DASHCAM),
      CarInfo("Toyota Veloz 2022-2026", "All", support_type=SupportType.DASHCAM),
    ],
    CarSpecs(mass=1170.0, wheelbase=2.750, steerRatio=17.0),
    flags=DNGAFlags.SNG,
  )
  PERODUA_ATIVA = DNGAPlatformConfig(
    [
      CarInfo("Perodua Ativa 2021-2026", "All", support_type=SupportType.DASHCAM),
      CarInfo("Perodua Ativa Hybrid 2022", "All", support_type=SupportType.DASHCAM),
      CarInfo("Toyota Raize 2021-2026", "All", support_type=SupportType.DASHCAM),
    ],
    CarSpecs(mass=1035.0, wheelbase=2.525, steerRatio=17.0),
  )
  PERODUA_MYVI = DNGAPlatformConfig(
    [CarInfo("Perodua Myvi 2022-2026", "All", support_type=SupportType.DASHCAM)],
    CarSpecs(mass=1025.0, wheelbase=2.500, steerRatio=17.4),
  )
  TOYOTA_VIOS = DNGAPlatformConfig(
    [CarInfo("Toyota Vios 2023-2026", "All", support_type=SupportType.DASHCAM)],
    CarSpecs(mass=1035.0, wheelbase=2.620, steerRatio=17.0),
    flags=DNGAFlags.SNG,
  )
  QC = DNGAPlatformConfig(
    [CarInfo("Quality Check 2020-24", "All", support_type=SupportType.DASHCAM)],
    CarSpecs(mass=1025.0, wheelbase=2.500, steerRatio=17.4),
  )


BRAKE_SCALE = defaultdict(
  lambda: 1.0,
  {
    CAR.PERODUA_ALZA: 0.65,
    CAR.PERODUA_ATIVA: 0.714,
    CAR.PERODUA_MYVI: 0.714,
    CAR.TOYOTA_VIOS: 0.68,
    CAR.QC: 0.714,
  },
)

SNG_CAR = CAR.with_flags(DNGAFlags.SNG)
HYBRID_CAR = CAR.with_flags(DNGAFlags.HYBRID)
CAR_INFO = CAR.create_carinfo_map()
DBC = CAR.create_dbc_map()


def __getattr__(name):
  if name == "CarControllerParams":
    from opendbc.car.dnga.carcontroller import CarControllerParams

    return CarControllerParams
  raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
