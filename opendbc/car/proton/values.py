from dataclasses import dataclass, field

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs, SupportType

HUD_MULTIPLIER = 1.04


@dataclass
class ProtonPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict("proton_general_pt", None))


class ProtonCarDocs(CarDocs):
  pass


class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2


class CAR(Platforms):
  PROTON_S70 = ProtonPlatformConfig(
    [
      ProtonCarDocs("Proton S70 2023-26", "All", support_type=SupportType.DASHCAM),
      ProtonCarDocs("Proton X50 FL 2025-26", "All", support_type=SupportType.DASHCAM),
    ],
    CarSpecs(mass=1300.0, wheelbase=2.627, steerRatio=15.0),
  )
  PROTON_X50 = ProtonPlatformConfig(
    [ProtonCarDocs("Proton X50 2020-24", "All", support_type=SupportType.DASHCAM)],
    CarSpecs(mass=1370.0, wheelbase=2.6, steerRatio=15.0),
  )
  PROTON_X70 = ProtonPlatformConfig(
    [ProtonCarDocs("Proton X70 FL 2024-26", "All", support_type=SupportType.DASHCAM)],
    CarSpecs(mass=1610.0, wheelbase=2.67, steerRatio=15.0),
  )
  PROTON_X90 = ProtonPlatformConfig(
    [ProtonCarDocs("Proton X90 2023-26", "All", support_type=SupportType.DASHCAM)],
    CarSpecs(mass=1705.0, wheelbase=2.805, steerRatio=15.0),
  )


DBC = CAR.create_dbc_map()


def __getattr__(name):
  if name == "CarControllerParams":
    from opendbc.car.proton.carcontroller import CarControllerParams

    return CarControllerParams
  raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
