from dataclasses import dataclass, field

from cereal import car
from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs as CarInfo, SupportType

HUD_MULTIPLIER = 1.04
Ecu = car.CarParams.Ecu


@dataclass
class ProtonPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("proton_general_pt", None))


class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2


class CAR(Platforms):
  PROTON_S70 = ProtonPlatformConfig(
    [
      CarInfo("Proton S70 2020-24", "All", support_type=SupportType.CUSTOM),
      CarInfo("Proton X50 FL 2020-24", "All", support_type=SupportType.CUSTOM),
    ],
    CarSpecs(mass=1300.0, wheelbase=2.627, steerRatio=15.0),
  )
  PROTON_X50 = ProtonPlatformConfig(
    [CarInfo("Proton X50 2020-24", "All", support_type=SupportType.CUSTOM)],
    CarSpecs(mass=1370.0, wheelbase=2.6, steerRatio=15.0),
  )
  PROTON_X70 = ProtonPlatformConfig(
    [CarInfo("Proton X70 2020-24", "All", support_type=SupportType.CUSTOM)],
    CarSpecs(mass=1610.0, wheelbase=2.67, steerRatio=15.0),
  )
  PROTON_X90 = ProtonPlatformConfig(
    [CarInfo("Proton X90 2020-24", "All", support_type=SupportType.CUSTOM)],
    CarSpecs(mass=1705.0, wheelbase=2.805, steerRatio=15.0),
  )


CAR_INFO = CAR.create_carinfo_map()
DBC = CAR.create_dbc_map()


def __getattr__(name):
  if name == "CarControllerParams":
    from opendbc.car.proton.carcontroller import CarControllerParams

    return CarControllerParams
  raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
