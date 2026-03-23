from dataclasses import dataclass, field

from opendbc.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from opendbc.car.docs_definitions import CarDocs

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
      ProtonCarDocs("Proton S70 2023-26", "All"),
      ProtonCarDocs("Proton X50 FL 2025-26", "All"),
    ],
    CarSpecs(mass=1300.0, wheelbase=2.627, steerRatio=15.0),
  )
  PROTON_X50 = ProtonPlatformConfig(
    [ProtonCarDocs("Proton X50 2020-24", "All")],
    CarSpecs(mass=1370.0, wheelbase=2.6, steerRatio=15.0),
  )
  PROTON_X70 = ProtonPlatformConfig(
    [ProtonCarDocs("Proton X70 FL 2024-26", "All")],
    CarSpecs(mass=1610.0, wheelbase=2.67, steerRatio=15.0),
  )
  PROTON_X90 = ProtonPlatformConfig(
    [ProtonCarDocs("Proton X90 2023-25", "All")],
    CarSpecs(mass=1705.0, wheelbase=2.805, steerRatio=15.0),
  )


DBC = CAR.create_dbc_map()

class CarControllerParams:
  STEER_STEP = 1

  def __init__(self, CP):
    self.STEER_MAX = CP.lateralParams.torqueV[0]
    assert len(CP.lateralParams.torqueV) == 1

    if CP.carFingerprint == CAR.PROTON_X90:
      self.STEER_DELTA_UP = 4
      self.STEER_DELTA_DOWN = 8
    else:
      self.STEER_DELTA_UP = 15
      self.STEER_DELTA_DOWN = 35
