from collections import defaultdict
from dataclasses import dataclass, field

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs as CarInfo, SupportType

HUD_MULTIPLIER = 1.07


@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("byd_general_pt", "byd_radar_fd"))


class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2


class CAR(Platforms):
  ATTO3 = BYDPlatformConfig(
    [CarInfo("BYD Atto 3 2020-24", "ALL", support_type=SupportType.CUSTOM)],
    CarSpecs(mass=2090.0, wheelbase=2.72, steerRatio=16.0),
  )
  M6 = BYDPlatformConfig(
    [CarInfo("BYD M6 2020-24", "ALL", support_type=SupportType.CUSTOM)],
    CarSpecs(mass=2374.0, wheelbase=2.80, steerRatio=16.0),
  )
  SEAL = BYDPlatformConfig(
    [CarInfo("BYD Seal 2020-24", "ALL", support_type=SupportType.CUSTOM)],
    CarSpecs(mass=2180.0, wheelbase=2.92, steerRatio=16.0),
  )
  SEALION7 = BYDPlatformConfig(
    [CarInfo("BYD Sealion 7 2020-24", "ALL", support_type=SupportType.CUSTOM)],
    CarSpecs(mass=2340.0, wheelbase=2.93, steerRatio=16.0),
  )


CAR_INFO = CAR.create_carinfo_map()
DBC = CAR.create_dbc_map()
ACCEL_MULT = defaultdict(
  lambda: 1,
  {
    CAR.ATTO3: 25,
    CAR.M6: 25,
    CAR.SEAL: 1,
    CAR.SEALION7: 1,
  },
)


def __getattr__(name):
  if name == "CarControllerParams":
    from opendbc.car.byd.carcontroller import CarControllerParams

    return CarControllerParams
  raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
