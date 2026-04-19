
"""Mission simulation engine.

This extends the existing N-body engine with:
- scheduled mission events,
- spacecraft guidance updates,
- spacecraft thrust and propellant consumption.

The implementation intentionally keeps the existing celestial mechanics simple
and adds a first robust layer for artificial spacecraft.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from nbody_engine import (
    SimulationClock,
    compute_accelerations,
    compute_diagnostics,
    iter_bodies,
    sync_global_transforms,
    update_axial_rotations,
)
from mission_timeline import MissionTimeline
from spacecraft_models import GuidanceMode, Spacecraft
from space_simulation_models import Node, Transform, Vector3


DEFAULT_MAX_SUBSTEP_SECONDS = 30.0


@dataclass(slots=True)
class MissionStepDiagnostics:
    simulated_delta_seconds: float
    executed_event_names: list[str]
    active_spacecraft_names: list[str]


def iter_spacecraft(root: Node) -> list[Spacecraft]:
    return [body for body in iter_bodies(root) if isinstance(body, Spacecraft)]


def _primary_center_vector(spacecraft: Spacecraft) -> Vector3 | None:
    if spacecraft.parent is None:
        return None
    # For now, use parent-frame center if the parent is the simulation root by
    # returning the vector from the parent origin to the spacecraft.
    return spacecraft.local_transform.position


def update_spacecraft_guidance(root: Node) -> None:
    for spacecraft in iter_spacecraft(root):
        center_vector = _primary_center_vector(spacecraft)
        spacecraft.update_guidance(center_vector=center_vector)


def compute_propulsive_accelerations(spacecrafts: Iterable[Spacecraft]) -> dict[str, Vector3]:
    accelerations: dict[str, Vector3] = {}
    for spacecraft in spacecrafts:
        accelerations[spacecraft.name] = spacecraft.compute_thrust_acceleration_km_s2()
    return accelerations


def velocity_verlet_mission_step(
    root: Node,
    dt_seconds: float,
    absolute_time_seconds: float,
) -> None:
    if dt_seconds <= 0.0:
        raise ValueError("dt_seconds must be > 0")

    bodies = iter_bodies(root)
    if not bodies:
        return

    spacecrafts = [body for body in bodies if isinstance(body, Spacecraft)]

    update_spacecraft_guidance(root)
    gravitational_accelerations_t = compute_accelerations(bodies)
    propulsive_accelerations_t = compute_propulsive_accelerations(spacecrafts)

    accelerations_t: list[Vector3] = []
    for body, gravity_acc in zip(bodies, gravitational_accelerations_t):
        if isinstance(body, Spacecraft):
            accelerations_t.append(gravity_acc + propulsive_accelerations_t.get(body.name, Vector3.zero()))
        else:
            accelerations_t.append(gravity_acc)

    for body, acceleration in zip(bodies, accelerations_t):
        p = body.local_transform.position
        v = body.local_velocity_km_s
        new_position = p + (v * dt_seconds) + (acceleration * (0.5 * dt_seconds * dt_seconds))
        body.local_transform = Transform(position=new_position, rotation=body.local_transform.rotation)

    sync_global_transforms(root)
    update_spacecraft_guidance(root)

    gravitational_accelerations_t_dt = compute_accelerations(bodies)
    propulsive_accelerations_t_dt = compute_propulsive_accelerations(spacecrafts)

    accelerations_t_dt: list[Vector3] = []
    for body, gravity_acc in zip(bodies, gravitational_accelerations_t_dt):
        if isinstance(body, Spacecraft):
            accelerations_t_dt.append(gravity_acc + propulsive_accelerations_t_dt.get(body.name, Vector3.zero()))
        else:
            accelerations_t_dt.append(gravity_acc)

    for body, a_t, a_t_dt in zip(bodies, accelerations_t, accelerations_t_dt):
        body.local_velocity_km_s = body.local_velocity_km_s + ((a_t + a_t_dt) * (0.5 * dt_seconds))

    for spacecraft in spacecrafts:
        spacecraft.consume_propellant(dt_seconds)

    sync_global_transforms(root)
    update_axial_rotations(bodies, absolute_time_seconds)


def step_mission_simulation(
    root: Node,
    clock: SimulationClock,
    timeline: MissionTimeline,
    spacecraft_index: dict[str, Spacecraft],
    real_delta_seconds: float,
    max_substep_seconds: float = DEFAULT_MAX_SUBSTEP_SECONDS,
) -> MissionStepDiagnostics:
    executed_event_names: list[str] = []

    simulated_delta = clock.advance(real_delta_seconds)
    if simulated_delta <= 0.0:
        return MissionStepDiagnostics(
            simulated_delta_seconds=0.0,
            executed_event_names=[],
            active_spacecraft_names=[sc.name for sc in iter_spacecraft(root)],
        )

    remaining = simulated_delta
    elapsed_since_step_start = 0.0
    while remaining > 0.0:
        dt = min(max_substep_seconds, remaining)
        current_substep_end_time = clock.current_time_seconds - remaining + dt

        executed_events = timeline.update(current_substep_end_time, spacecraft_index)
        executed_event_names.extend(event.name for event in executed_events)

        velocity_verlet_mission_step(
            root=root,
            dt_seconds=dt,
            absolute_time_seconds=current_substep_end_time,
        )

        remaining -= dt
        elapsed_since_step_start += dt

    return MissionStepDiagnostics(
        simulated_delta_seconds=simulated_delta,
        executed_event_names=executed_event_names,
        active_spacecraft_names=[sc.name for sc in iter_spacecraft(root)],
    )


__all__ = [
    "DEFAULT_MAX_SUBSTEP_SECONDS",
    "MissionStepDiagnostics",
    "compute_diagnostics",
    "iter_spacecraft",
    "step_mission_simulation",
    "update_spacecraft_guidance",
    "velocity_verlet_mission_step",
]
