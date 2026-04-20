
"""Launch-to-Moon mission scenarios.

This module creates launch-ready scenarios that start on Earth's surface and
evolve through ascent, staging, parking orbit, TLI and lunar approach.
"""

from __future__ import annotations

from dataclasses import dataclass

from atmosphere_models import EarthExponentialAtmosphere
from launch_mission_profiles import LaunchVehicleProgram, TransLunarMissionController
from nbody_engine import SimulationClock
from solar_system_factory_nbody import create_default_solar_system_nbody
from spacecraft_models import Engine, PropellantType, Spacecraft, Stage, Tank
from space_simulation_models import ReferenceFrame, Vector3


@dataclass(slots=True)
class LaunchMissionScenario:
    root: ReferenceFrame
    clock: SimulationClock
    body_index: dict[str, object]
    spacecraft: Spacecraft
    controller: TransLunarMissionController
    atmosphere: EarthExponentialAtmosphere


def create_apollo_like_launch_vehicle(name: str = "ApolloVehicle", parent=None) -> tuple[Spacecraft, LaunchVehicleProgram]:
    stage1 = Stage(
        name="S-IC",
        dry_mass_kg=131_000.0,
        tanks=[Tank("SIC_LOX_RP1", PropellantType.RP1, capacity_kg=2_140_000.0, current_mass_kg=2_140_000.0)],
        engines=[Engine("F1_CLUSTER", max_thrust_newtons=34_000_000.0, specific_impulse_seconds=263.0, tank_names=["SIC_LOX_RP1"])],
    )
    stage2 = Stage(
        name="S-II",
        dry_mass_kg=40_000.0,
        tanks=[Tank("SII_LOX_LH2", PropellantType.LH2, capacity_kg=456_000.0, current_mass_kg=456_000.0)],
        engines=[Engine("J2_STAGE2", max_thrust_newtons=5_100_000.0, specific_impulse_seconds=421.0, tank_names=["SII_LOX_LH2"])],
    )
    stage3 = Stage(
        name="S-IVB",
        dry_mass_kg=13_600.0,
        tanks=[Tank("SIVB_LOX_LH2", PropellantType.LH2, capacity_kg=109_000.0, current_mass_kg=109_000.0)],
        engines=[Engine("J2_STAGE3", max_thrust_newtons=1_000_000.0, specific_impulse_seconds=421.0, tank_names=["SIVB_LOX_LH2"])],
    )
    service = Stage(
        name="Apollo_CSM_Service",
        dry_mass_kg=11_900.0,
        tanks=[Tank("SM_Propellant", PropellantType.GENERIC, capacity_kg=18_400.0, current_mass_kg=18_400.0)],
        engines=[Engine("SPS_Main", max_thrust_newtons=91_000.0, specific_impulse_seconds=314.0, tank_names=["SM_Propellant"])],
    )
    spacecraft = Spacecraft(
        name=name,
        stages=[stage1, stage2, stage3, service],
        dry_radius_km=5.0,
        color_hex="#FFD27F",
        parent=parent,
        metadata={
            "drag_coefficient": 0.45,
            "reference_area_m2": 80.0,
            "vehicle_family": "apollo_like_launch_vehicle",
        },
    )
    program = LaunchVehicleProgram(
        ascent_stage_names=["S-IC", "S-II"],
        ascent_engine_names=["F1_CLUSTER", "J2_STAGE2"],
        parking_stage_name="S-IVB",
        parking_engine_name="J2_STAGE3",
        service_stage_name="Apollo_CSM_Service",
        service_engine_name="SPS_Main",
        parking_orbit_target_altitude_km=185.0,
        parking_coast_duration_seconds=15.0 * 60.0,
        translunar_transfer_time_seconds=72.0 * 3600.0,
        target_lunar_distance_km=384_400.0,
    )
    return spacecraft, program


def create_artemis_like_launch_vehicle(name: str = "Orion", parent=None) -> tuple[Spacecraft, LaunchVehicleProgram]:
    core = Stage(
        name="SLS_Core_Boosters",
        dry_mass_kg=200_000.0,
        tanks=[Tank("SLS_Core_Propellant", PropellantType.LH2, capacity_kg=3_050_000.0, current_mass_kg=3_050_000.0)],
        engines=[Engine("SLS_LIFTOFF_CLUSTER", max_thrust_newtons=39_000_000.0, specific_impulse_seconds=330.0, tank_names=["SLS_Core_Propellant"])],
    )
    icps = Stage(
        name="ICPS",
        dry_mass_kg=9_000.0,
        tanks=[Tank("ICPS_Propellant", PropellantType.LH2, capacity_kg=110_000.0, current_mass_kg=110_000.0)],
        engines=[Engine("RL10_ICPS", max_thrust_newtons=1_000_000.0, specific_impulse_seconds=450.0, tank_names=["ICPS_Propellant"])],
    )
    esm = Stage(
        name="Orion_ESM",
        dry_mass_kg=4_900.0,
        tanks=[
            Tank("ESM_MMH", PropellantType.MONOMETHYLHYDRAZINE, capacity_kg=4_300.0, current_mass_kg=4_300.0),
            Tank("ESM_NTO", PropellantType.DINITROGEN_TETROXIDE, capacity_kg=4_300.0, current_mass_kg=4_300.0),
        ],
        engines=[Engine("OMS_E_Main", max_thrust_newtons=25_700.0, specific_impulse_seconds=316.0, tank_names=["ESM_MMH", "ESM_NTO"])],
    )
    spacecraft = Spacecraft(
        name=name,
        stages=[core, icps, esm],
        dry_radius_km=5.0,
        color_hex="#FFAA55",
        parent=parent,
        metadata={
            "drag_coefficient": 0.42,
            "reference_area_m2": 90.0,
            "vehicle_family": "artemis_like_launch_vehicle",
        },
    )
    program = LaunchVehicleProgram(
        ascent_stage_names=["SLS_Core_Boosters"],
        ascent_engine_names=["SLS_LIFTOFF_CLUSTER"],
        parking_stage_name="ICPS",
        parking_engine_name="RL10_ICPS",
        service_stage_name="Orion_ESM",
        service_engine_name="OMS_E_Main",
        parking_orbit_target_altitude_km=180.0,
        parking_coast_duration_seconds=20.0 * 60.0,
        translunar_transfer_time_seconds=96.0 * 3600.0,
        target_lunar_distance_km=384_400.0,
    )
    return spacecraft, program


def create_translunar_launch_scenario(vehicle: str = "artemis") -> LaunchMissionScenario:
    scene = create_default_solar_system_nbody()
    root = scene.root
    body_index = scene.body_index

    if vehicle == "apollo":
        spacecraft, program = create_apollo_like_launch_vehicle(name="ApolloVehicle", parent=root)
    else:
        spacecraft, program = create_artemis_like_launch_vehicle(name="Orion", parent=root)

    controller = TransLunarMissionController(spacecraft_name=spacecraft.name, program=program)
    controller.place_on_launch_pad(spacecraft, body_index["Earth"], body_index["Moon"])

    root.update_global_transform(propagate=True)

    clock = SimulationClock(current_time_seconds=0.0, time_scale=1.0)
    atmosphere = EarthExponentialAtmosphere()

    return LaunchMissionScenario(
        root=root,
        clock=clock,
        body_index=body_index,
        spacecraft=spacecraft,
        controller=controller,
        atmosphere=atmosphere,
    )


__all__ = [
    "LaunchMissionScenario",
    "create_apollo_like_launch_vehicle",
    "create_artemis_like_launch_vehicle",
    "create_translunar_launch_scenario",
]
