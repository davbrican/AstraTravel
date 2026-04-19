"""N-body gravitational engine for the Solar System simulator.

This module performs genuine Newtonian N-body integration:
- every body attracts every other body,
- accelerations are computed pairwise,
- positions and velocities are integrated with Velocity Verlet.

Units
-----
- Distance: kilometers
- Velocity: kilometers per second
- Acceleration: kilometers per second squared
- Mass: kilograms
- Time: seconds
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Iterable, Sequence

from space_simulation_models import CelestialBody, Node, Quaternion, Transform, Vector3

G_KM3_KG_S2 = 6.67430e-20
SOFTENING_KM = 1e-3


@dataclass(slots=True)
class SimulationClock:
    current_time_seconds: float = 0.0
    time_scale: float = 1.0
    is_paused: bool = False

    def advance(self, real_delta_seconds: float) -> float:
        if real_delta_seconds < 0.0:
            raise ValueError("real_delta_seconds must be >= 0")
        if self.is_paused:
            return 0.0
        simulated_delta = real_delta_seconds * self.time_scale
        self.current_time_seconds += simulated_delta
        return simulated_delta

    def reset(self) -> None:
        self.current_time_seconds = 0.0


@dataclass(slots=True)
class NBodyDiagnostics:
    total_kinetic_energy: float
    total_potential_energy: float
    total_energy: float
    center_of_mass: Vector3
    center_of_mass_velocity: Vector3


def iter_bodies(root: Node) -> list[CelestialBody]:
    bodies: list[CelestialBody] = []
    if isinstance(root, CelestialBody):
        bodies.append(root)
    for node in root.iter_descendants():
        if isinstance(node, CelestialBody):
            bodies.append(node)
    return bodies


def compute_accelerations(bodies: Sequence[CelestialBody]) -> list[Vector3]:
    accelerations = [Vector3.zero() for _ in bodies]
    count = len(bodies)

    for i in range(count):
        body_i = bodies[i]
        pos_i = body_i.local_transform.position
        for j in range(i + 1, count):
            body_j = bodies[j]
            delta = body_j.local_transform.position - pos_i
            r2 = delta.dot(delta) + SOFTENING_KM * SOFTENING_KM
            r = sqrt(r2)
            inv_r3 = 1.0 / (r2 * r)
            factor = G_KM3_KG_S2 * inv_r3
            acc_i = delta * (factor * body_j.mass_kg)
            acc_j = delta * (factor * body_i.mass_kg)
            accelerations[i] = accelerations[i] + acc_i
            accelerations[j] = accelerations[j] - acc_j

    return accelerations


def update_axial_rotations(bodies: Iterable[CelestialBody], absolute_time_seconds: float) -> None:
    two_pi = 2.0 * 3.141592653589793
    for body in bodies:
        period = body.rotation_period_seconds
        if not period:
            continue
        phase = (absolute_time_seconds / period) * two_pi
        body.set_local_rotation(Quaternion.from_axis_angle(body.rotation_properties.rotation_axis_local, phase))


def sync_global_transforms(root: Node) -> None:
    root.update_global_transform(propagate=True)


def velocity_verlet_step(root: Node, dt_seconds: float, absolute_time_seconds: float | None = None) -> None:
    if dt_seconds <= 0.0:
        raise ValueError("dt_seconds must be > 0")

    bodies = iter_bodies(root)
    if not bodies:
        return

    accelerations_t = compute_accelerations(bodies)

    for body, acceleration in zip(bodies, accelerations_t):
        p = body.local_transform.position
        v = body.local_velocity_km_s
        new_position = p + (v * dt_seconds) + (acceleration * (0.5 * dt_seconds * dt_seconds))
        body.local_transform = Transform(position=new_position, rotation=body.local_transform.rotation)

    accelerations_t_dt = compute_accelerations(bodies)

    for body, a_t, a_t_dt in zip(bodies, accelerations_t, accelerations_t_dt):
        body.local_velocity_km_s = body.local_velocity_km_s + ((a_t + a_t_dt) * (0.5 * dt_seconds))

    sync_global_transforms(root)
    if absolute_time_seconds is not None:
        update_axial_rotations(bodies, absolute_time_seconds)


def step_simulation(root: Node, clock: SimulationClock, real_delta_seconds: float, max_substep_seconds: float = 3600.0) -> float:
    simulated_delta = clock.advance(real_delta_seconds)
    if simulated_delta <= 0.0:
        return 0.0

    remaining = simulated_delta
    while remaining > 0.0:
        dt = min(max_substep_seconds, remaining)
        velocity_verlet_step(root, dt, clock.current_time_seconds - remaining + dt)
        remaining -= dt

    return simulated_delta


def compute_diagnostics(root: Node) -> NBodyDiagnostics:
    bodies = iter_bodies(root)
    total_mass = sum(body.mass_kg for body in bodies)
    if total_mass <= 0.0:
        return NBodyDiagnostics(0.0, 0.0, 0.0, Vector3.zero(), Vector3.zero())

    total_kinetic_energy = 0.0
    total_potential_energy = 0.0
    com = Vector3.zero()
    com_velocity = Vector3.zero()

    for body in bodies:
        speed = body.local_velocity_km_s.magnitude()
        total_kinetic_energy += 0.5 * body.mass_kg * speed * speed
        com += body.local_transform.position * body.mass_kg
        com_velocity += body.local_velocity_km_s * body.mass_kg

    com = com / total_mass
    com_velocity = com_velocity / total_mass

    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            delta = bodies[j].local_transform.position - bodies[i].local_transform.position
            distance = max(delta.magnitude(), SOFTENING_KM)
            total_potential_energy -= G_KM3_KG_S2 * bodies[i].mass_kg * bodies[j].mass_kg / distance

    return NBodyDiagnostics(
        total_kinetic_energy=total_kinetic_energy,
        total_potential_energy=total_potential_energy,
        total_energy=total_kinetic_energy + total_potential_energy,
        center_of_mass=com,
        center_of_mass_velocity=com_velocity,
    )


__all__ = [
    "G_KM3_KG_S2",
    "NBodyDiagnostics",
    "SOFTENING_KM",
    "SimulationClock",
    "compute_accelerations",
    "compute_diagnostics",
    "iter_bodies",
    "step_simulation",
    "sync_global_transforms",
    "update_axial_rotations",
    "velocity_verlet_step",
]
