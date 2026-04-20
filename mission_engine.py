
"""Mission simulation engine v2 with launch, drag, staging and translunar control."""

from __future__ import annotations

from dataclasses import dataclass
from math import inf

from atmosphere_models import EarthExponentialAtmosphere
from launch_mission_profiles import TransLunarMissionController, compute_earth_relative_snapshot
from nbody_engine import (
    SimulationClock,
    compute_accelerations,
    compute_diagnostics,
    iter_bodies,
    sync_global_transforms,
    update_axial_rotations,
)
from spacecraft_models import Spacecraft
from space_simulation_models import CelestialBody, Node, Transform, Vector3


DEFAULT_POWERED_SUBSTEP_SECONDS = 1.0 / 30.0
DEFAULT_MAX_SUBSTEP_SECONDS = 0.5
EVENT_TIME_EPSILON_SECONDS = 1e-6
LOW_ALTITUDE_FINE_STEP_KM = 200.0


@dataclass(slots=True)
class MissionStepDiagnosticsV2:
    simulated_delta_seconds: float
    phase_name: str
    altitude_km: float
    speed_km_s: float
    propellant_mass_kg: float
    total_mass_kg: float


def iter_spacecraft(root: Node) -> list[Spacecraft]:
    return [body for body in iter_bodies(root) if isinstance(body, Spacecraft)]


def get_body_index(root: Node) -> dict[str, CelestialBody]:
    return {body.name: body for body in iter_bodies(root) if isinstance(body, CelestialBody)}


def compute_drag_acceleration_km_s2(spacecraft: Spacecraft, earth: CelestialBody, atmosphere: EarthExponentialAtmosphere) -> Vector3:
    relative_position = spacecraft.global_position - earth.global_position
    altitude_km = relative_position.magnitude() - earth.radius_km
    if altitude_km <= 0.0:
        return Vector3.zero()
    if altitude_km > 180.0:
        return Vector3.zero()

    relative_velocity = spacecraft.local_velocity_km_s - earth.local_velocity_km_s
    speed_km_s = relative_velocity.magnitude()
    if speed_km_s <= 1e-9:
        return Vector3.zero()

    sample = atmosphere.sample(altitude_km * 1000.0)
    drag_coefficient = float(spacecraft.metadata.get("drag_coefficient", 0.4))
    reference_area_m2 = float(spacecraft.metadata.get("reference_area_m2", 20.0))

    speed_m_s = speed_km_s * 1000.0
    drag_force_n = 0.5 * sample.density_kg_m3 * speed_m_s * speed_m_s * drag_coefficient * reference_area_m2
    drag_accel_m_s2 = drag_force_n / max(spacecraft.total_mass_kg, 1.0)
    drag_direction = (relative_velocity * -1.0).normalized()
    return drag_direction * (drag_accel_m_s2 / 1000.0)


def _next_controller_event_time(controller: TransLunarMissionController, start_time_seconds: float, end_time_seconds: float) -> float | None:
    events = getattr(controller, "events", None)
    if not events:
        return None

    next_time = inf
    for event in events:
        if getattr(event, "executed", False):
            continue
        trigger_time = float(getattr(event, "trigger_time_seconds", inf))
        if trigger_time <= start_time_seconds + EVENT_TIME_EPSILON_SECONDS:
            continue
        if trigger_time <= end_time_seconds + EVENT_TIME_EPSILON_SECONDS:
            next_time = min(next_time, trigger_time)

    return None if next_time == inf else next_time


def _needs_fine_substep(spacecraft: Spacecraft, earth: CelestialBody) -> bool:
    if spacecraft.has_active_propulsion():
        return True

    altitude_km = (spacecraft.global_position - earth.global_position).magnitude() - earth.radius_km
    return altitude_km <= LOW_ALTITUDE_FINE_STEP_KM


def velocity_verlet_launch_step(
    root: Node,
    controller: TransLunarMissionController,
    atmosphere: EarthExponentialAtmosphere,
    dt_seconds: float,
    absolute_time_seconds: float,
) -> None:
    if dt_seconds <= 0.0:
        raise ValueError("dt_seconds must be > 0")

    bodies = iter_bodies(root)
    if not bodies:
        return

    body_index = get_body_index(root)
    earth = body_index["Earth"]
    moon = body_index["Moon"]
    spacecraft = next(body for body in bodies if isinstance(body, Spacecraft) and body.name == controller.spacecraft_name)

    controller.update(spacecraft, earth, moon, absolute_time_seconds)

    gravitational_accelerations_t = compute_accelerations(bodies)
    accelerations_t: list[Vector3] = []

    for body, gravity_acc in zip(bodies, gravitational_accelerations_t):
        if isinstance(body, Spacecraft) and body.name == controller.spacecraft_name:
            thrust_acc = body.compute_thrust_acceleration_km_s2()
            drag_acc = compute_drag_acceleration_km_s2(body, earth, atmosphere)
            accelerations_t.append(gravity_acc + thrust_acc + drag_acc)
        else:
            accelerations_t.append(gravity_acc)

    for body, acceleration in zip(bodies, accelerations_t):
        p = body.local_transform.position
        v = body.local_velocity_km_s
        new_position = p + (v * dt_seconds) + (acceleration * (0.5 * dt_seconds * dt_seconds))
        body.local_transform = Transform(position=new_position, rotation=body.local_transform.rotation)

    sync_global_transforms(root)

    gravitational_accelerations_t_dt = compute_accelerations(bodies)
    accelerations_t_dt: list[Vector3] = []

    for body, gravity_acc in zip(bodies, gravitational_accelerations_t_dt):
        if isinstance(body, Spacecraft) and body.name == controller.spacecraft_name:
            thrust_acc = body.compute_thrust_acceleration_km_s2()
            drag_acc = compute_drag_acceleration_km_s2(body, earth, atmosphere)
            accelerations_t_dt.append(gravity_acc + thrust_acc + drag_acc)
        else:
            accelerations_t_dt.append(gravity_acc)

    for body, a_t, a_t_dt in zip(bodies, accelerations_t, accelerations_t_dt):
        body.local_velocity_km_s = body.local_velocity_km_s + ((a_t + a_t_dt) * (0.5 * dt_seconds))

    spacecraft.consume_propellant(dt_seconds)

    sync_global_transforms(root)
    update_axial_rotations([body for body in bodies if isinstance(body, CelestialBody)], absolute_time_seconds + dt_seconds)

    controller.update(spacecraft, earth, moon, absolute_time_seconds + dt_seconds)


def step_launch_mission_simulation(
    root: Node,
    clock: SimulationClock,
    controller: TransLunarMissionController,
    atmosphere: EarthExponentialAtmosphere,
    real_delta_seconds: float,
    max_substep_seconds: float = DEFAULT_MAX_SUBSTEP_SECONDS,
) -> MissionStepDiagnosticsV2:
    simulated_delta = clock.advance(real_delta_seconds)
    if simulated_delta <= 0.0:
        body_index = get_body_index(root)
        sc = next(body for body in iter_bodies(root) if isinstance(body, Spacecraft) and body.name == controller.spacecraft_name)
        earth = body_index["Earth"]
        snap = compute_earth_relative_snapshot(sc, earth)
        return MissionStepDiagnosticsV2(0.0, controller.phase.value, snap.altitude_km, snap.speed_km_s, sc.propellant_mass_kg, sc.total_mass_kg)

    body_index = get_body_index(root)
    earth = body_index["Earth"]
    moon = body_index["Moon"]
    spacecraft = next(body for body in iter_bodies(root) if isinstance(body, Spacecraft) and body.name == controller.spacecraft_name)

    remaining = simulated_delta
    while remaining > 0.0:
        absolute_time = clock.current_time_seconds - remaining
        controller.update(spacecraft, earth, moon, absolute_time)

        substep_limit = max_substep_seconds
        if _needs_fine_substep(spacecraft, earth):
            substep_limit = min(substep_limit, DEFAULT_POWERED_SUBSTEP_SECONDS)

        next_event_time = _next_controller_event_time(
            controller,
            start_time_seconds=absolute_time,
            end_time_seconds=absolute_time + min(substep_limit, remaining),
        )

        dt = min(substep_limit, remaining)
        if next_event_time is not None:
            dt = min(dt, max(0.0, next_event_time - absolute_time))
        if dt <= EVENT_TIME_EPSILON_SECONDS:
            dt = min(substep_limit, remaining)

        velocity_verlet_launch_step(root, controller, atmosphere, dt, absolute_time)
        remaining -= dt

    body_index = get_body_index(root)
    sc = next(body for body in iter_bodies(root) if isinstance(body, Spacecraft) and body.name == controller.spacecraft_name)
    earth = body_index["Earth"]
    snap = compute_earth_relative_snapshot(sc, earth)
    return MissionStepDiagnosticsV2(
        simulated_delta_seconds=simulated_delta,
        phase_name=controller.phase.value,
        altitude_km=snap.altitude_km,
        speed_km_s=snap.speed_km_s,
        propellant_mass_kg=sc.propellant_mass_kg,
        total_mass_kg=sc.total_mass_kg,
    )


__all__ = [
    "DEFAULT_MAX_SUBSTEP_SECONDS",
    "DEFAULT_POWERED_SUBSTEP_SECONDS",
    "MissionStepDiagnosticsV2",
    "compute_drag_acceleration_km_s2",
    "get_body_index",
    "iter_spacecraft",
    "step_launch_mission_simulation",
]
