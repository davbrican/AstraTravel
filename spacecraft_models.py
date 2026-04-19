
"""Spacecraft domain models for mission simulation.

This module adds a first serious spacecraft layer on top of the existing
celestial-body / N-body infrastructure.

Design notes
------------
- `Spacecraft` inherits from `CelestialBody` so it can participate in the
  existing N-body gravity engine without rewriting the whole environment.
- Spacecraft mass is dynamic and derived from stages + tanks.
- Guidance is intentionally idealized in this first version:
  the spacecraft attitude is rotated instantly toward the requested guidance
  vector. Full rotational dynamics are left for a later phase.
- Propulsion is stage/tank/engine based and supports variable mass.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import acos, isclose, log
from typing import Any, Iterable, Optional

from space_simulation_models import (
    BodyType,
    CelestialBody,
    PhysicalProperties,
    Quaternion,
    RotationProperties,
    OrbitParameters,
    Transform,
    Vector3,
    VisualProperties,
    GRAVITATIONAL_CONSTANT_SI,
    KM_TO_M,
)


G0_M_S2 = 9.80665
NEWTON_TO_KMKG_S2 = 1.0 / 1000.0


class PropellantType(str, Enum):
    GENERIC = "generic"
    MMH = "mmh"
    NTO = "nto"
    LOX = "lox"
    LH2 = "lh2"
    RP1 = "rp1"
    MONOMETHYLHYDRAZINE = "monomethylhydrazine"
    DINITROGEN_TETROXIDE = "dinitrogen_tetroxide"


class GuidanceMode(str, Enum):
    INERTIAL_HOLD = "inertial_hold"
    PROGRADE = "prograde"
    RETROGRADE = "retrograde"
    RADIAL_OUT = "radial_out"
    RADIAL_IN = "radial_in"
    TARGET_VECTOR = "target_vector"
    MANUAL = "manual"


@dataclass(slots=True)
class Tank:
    name: str
    propellant_type: PropellantType
    capacity_kg: float
    current_mass_kg: float
    priority: int = 0
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.capacity_kg <= 0.0:
            raise ValueError("Tank capacity_kg must be > 0")
        if not 0.0 <= self.current_mass_kg <= self.capacity_kg:
            raise ValueError("Tank current_mass_kg must be between 0 and capacity_kg")

    @property
    def fill_ratio(self) -> float:
        return 0.0 if self.capacity_kg <= 0.0 else self.current_mass_kg / self.capacity_kg

    @property
    def is_empty(self) -> bool:
        return self.current_mass_kg <= 0.0

    def draw_mass(self, requested_mass_kg: float) -> float:
        if requested_mass_kg <= 0.0:
            return 0.0
        drawn = min(self.current_mass_kg, requested_mass_kg)
        self.current_mass_kg -= drawn
        return drawn


@dataclass(slots=True)
class Engine:
    name: str
    max_thrust_newtons: float
    specific_impulse_seconds: float
    tank_names: list[str] = field(default_factory=list)
    direction_body_frame: Vector3 = field(default_factory=lambda: Vector3(1.0, 0.0, 0.0))
    position_body_frame: Vector3 = field(default_factory=Vector3.zero)
    min_throttle: float = 0.0
    max_throttle: float = 1.0
    restartable: bool = True
    remaining_ignitions: Optional[int] = None
    gimbal_limit_deg: float = 0.0
    role: str = "main"
    is_active: bool = False
    throttle: float = 0.0

    def __post_init__(self) -> None:
        if self.max_thrust_newtons <= 0.0:
            raise ValueError("Engine max_thrust_newtons must be > 0")
        if self.specific_impulse_seconds <= 0.0:
            raise ValueError("Engine specific_impulse_seconds must be > 0")
        if self.direction_body_frame.magnitude() == 0.0:
            raise ValueError("Engine direction_body_frame cannot be zero")
        if not 0.0 <= self.min_throttle <= 1.0:
            raise ValueError("Engine min_throttle must be in [0,1]")
        if not 0.0 < self.max_throttle <= 1.0:
            raise ValueError("Engine max_throttle must be in (0,1]")
        if self.min_throttle > self.max_throttle:
            raise ValueError("Engine min_throttle cannot exceed max_throttle")
        self.direction_body_frame = self.direction_body_frame.normalized()

    def arm(self, throttle: float = 1.0) -> None:
        if not self.restartable and self.remaining_ignitions == 0:
            raise RuntimeError(f"Engine {self.name} has no ignitions left")
        throttle = max(self.min_throttle, min(self.max_throttle, throttle))
        if throttle <= 0.0:
            self.shutdown()
            return
        if not self.is_active:
            if self.remaining_ignitions is not None:
                if self.remaining_ignitions <= 0:
                    raise RuntimeError(f"Engine {self.name} cannot be ignited again")
                self.remaining_ignitions -= 1
        self.is_active = True
        self.throttle = throttle

    def shutdown(self) -> None:
        self.is_active = False
        self.throttle = 0.0

    def current_thrust_newtons(self) -> float:
        if not self.is_active or self.throttle <= 0.0:
            return 0.0
        return self.max_thrust_newtons * self.throttle

    def current_mass_flow_kg_s(self) -> float:
        thrust = self.current_thrust_newtons()
        if thrust <= 0.0:
            return 0.0
        return thrust / (self.specific_impulse_seconds * G0_M_S2)


@dataclass(slots=True)
class ThrusterCluster:
    name: str
    engine_names: list[str]
    role: str = "translation"


@dataclass(slots=True)
class Stage:
    name: str
    dry_mass_kg: float
    tanks: list[Tank] = field(default_factory=list)
    engines: list[Engine] = field(default_factory=list)
    thruster_clusters: list[ThrusterCluster] = field(default_factory=list)
    is_active: bool = True
    is_jettisoned: bool = False
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.dry_mass_kg <= 0.0:
            raise ValueError("Stage dry_mass_kg must be > 0")

    @property
    def propellant_mass_kg(self) -> float:
        return sum(tank.current_mass_kg for tank in self.tanks)

    @property
    def total_mass_kg(self) -> float:
        if self.is_jettisoned:
            return 0.0
        return self.dry_mass_kg + self.propellant_mass_kg

    def iter_active_engines(self) -> Iterable[Engine]:
        if self.is_jettisoned or not self.is_active:
            return []
        return [engine for engine in self.engines if engine.is_active]

    def get_engine(self, engine_name: str) -> Engine:
        for engine in self.engines:
            if engine.name == engine_name:
                return engine
        raise KeyError(f"Unknown engine {engine_name!r} in stage {self.name!r}")

    def get_tank(self, tank_name: str) -> Tank:
        for tank in self.tanks:
            if tank.name == tank_name:
                return tank
        raise KeyError(f"Unknown tank {tank_name!r} in stage {self.name!r}")

    def draw_propellant(self, requested_mass_kg: float, tank_names: Optional[list[str]] = None) -> float:
        if requested_mass_kg <= 0.0 or self.is_jettisoned:
            return 0.0

        candidate_tanks = (
            [self.get_tank(name) for name in tank_names]
            if tank_names
            else [tank for tank in self.tanks if not tank.is_empty]
        )
        candidate_tanks = [tank for tank in candidate_tanks if not tank.is_empty]
        if not candidate_tanks:
            return 0.0

        total_available = sum(tank.current_mass_kg for tank in candidate_tanks)
        if total_available <= 0.0:
            return 0.0

        requested_mass_kg = min(requested_mass_kg, total_available)
        drawn_total = 0.0
        remaining = requested_mass_kg

        # Draw proportionally to the available mass in each tank.
        availability = [tank.current_mass_kg for tank in candidate_tanks]
        availability_sum = sum(availability)

        for idx, tank in enumerate(candidate_tanks):
            if remaining <= 0.0:
                break
            if idx == len(candidate_tanks) - 1:
                portion = remaining
            else:
                portion = requested_mass_kg * (tank.current_mass_kg / availability_sum)
            drawn = tank.draw_mass(portion)
            drawn_total += drawn
            remaining -= drawn

        if remaining > 1e-9:
            for tank in candidate_tanks:
                if remaining <= 0.0:
                    break
                drawn = tank.draw_mass(remaining)
                drawn_total += drawn
                remaining -= drawn

        return drawn_total

    def jettison(self) -> None:
        self.is_jettisoned = True
        self.is_active = False
        for engine in self.engines:
            engine.shutdown()


@dataclass(slots=True)
class SpacecraftState:
    position_km: Vector3
    velocity_km_s: Vector3
    attitude_quaternion: Quaternion
    angular_velocity_rad_s: Vector3
    epoch_seconds: float
    reference_frame: str


def _safe_normalized(vector: Vector3) -> Vector3:
    magnitude = vector.magnitude()
    if isclose(magnitude, 0.0):
        raise ValueError("Cannot normalize a zero vector.")
    return vector / magnitude


def quaternion_from_two_vectors(source: Vector3, target: Vector3) -> Quaternion:
    a = _safe_normalized(source)
    b = _safe_normalized(target)

    dot = max(-1.0, min(1.0, a.dot(b)))
    if isclose(dot, 1.0, abs_tol=1e-12):
        return Quaternion.identity()

    if isclose(dot, -1.0, abs_tol=1e-12):
        fallback = Vector3(0.0, 1.0, 0.0)
        if abs(a.dot(fallback)) > 0.99:
            fallback = Vector3(0.0, 0.0, 1.0)
        axis = _safe_normalized(a.cross(fallback))
        return Quaternion.from_axis_angle(axis, 3.141592653589793)

    axis = _safe_normalized(a.cross(b))
    angle = acos(dot)
    return Quaternion.from_axis_angle(axis, angle)


class Spacecraft(CelestialBody):
    """Artificial spacecraft that participates in the existing N-body world."""

    def __init__(
        self,
        name: str,
        *,
        stages: list[Stage],
        dry_radius_km: float = 5.0,
        color_hex: str = "#FF5555",
        local_transform: Optional[Transform] = None,
        local_velocity_km_s: Optional[Vector3] = None,
        local_angular_velocity_rad_s: Optional[Vector3] = None,
        parent=None,
        metadata: Optional[dict[str, Any]] = None,
    ) -> None:
        if not stages:
            raise ValueError("Spacecraft requires at least one stage")

        initial_mass = sum(stage.total_mass_kg for stage in stages)
        super().__init__(
            name=name,
            body_type=BodyType.ARTIFICIAL,
            physical_properties=PhysicalProperties(
                mass_kg=max(initial_mass, 1.0),
                radius_km=max(dry_radius_km, 0.1),
            ),
            rotation_properties=RotationProperties(
                axial_tilt_deg=0.0,
                rotation_period_seconds=None,
            ),
            orbit=OrbitParameters(),
            visual_properties=VisualProperties(
                color_hex=color_hex,
                render_radius_km=max(dry_radius_km * 25.0, 20.0),
                trail_enabled=True,
            ),
            local_velocity_km_s=local_velocity_km_s or Vector3.zero(),
            local_angular_velocity_rad_s=local_angular_velocity_rad_s or Vector3.zero(),
            local_transform=local_transform or Transform.identity(),
            parent=parent,
            metadata=metadata or {},
        )
        self.stages: list[Stage] = stages
        self.guidance_mode: GuidanceMode = GuidanceMode.INERTIAL_HOLD
        self.guidance_target_vector: Optional[Vector3] = None
        self.last_propellant_consumption_kg: float = 0.0
        self.mission_tags: set[str] = set()

    @property  # type: ignore[override]
    def mass_kg(self) -> float:
        return self.total_mass_kg

    @property
    def total_mass_kg(self) -> float:
        return sum(stage.total_mass_kg for stage in self.stages if not stage.is_jettisoned)

    @property
    def dry_mass_kg(self) -> float:
        return sum(stage.dry_mass_kg for stage in self.stages if not stage.is_jettisoned)

    @property
    def propellant_mass_kg(self) -> float:
        return sum(stage.propellant_mass_kg for stage in self.stages if not stage.is_jettisoned)

    @property
    def available_delta_v_m_s(self) -> float:
        total_mass = self.total_mass_kg
        if total_mass <= 0.0:
            return 0.0

        # Use an effective Isp weighted by currently available engines.
        engines = [engine for stage in self.stages if not stage.is_jettisoned for engine in stage.engines]
        if not engines:
            return 0.0
        avg_isp = sum(engine.specific_impulse_seconds for engine in engines) / len(engines)
        final_mass = max(self.dry_mass_kg, 1e-6)
        return avg_isp * G0_M_S2 * log(total_mass / final_mass)

    @property
    def available_delta_v_km_s(self) -> float:
        return self.available_delta_v_m_s / 1000.0

    def snapshot_state(self, epoch_seconds: float, reference_frame: str) -> SpacecraftState:
        return SpacecraftState(
            position_km=self.global_position,
            velocity_km_s=self.local_velocity_km_s,
            attitude_quaternion=self.local_transform.rotation,
            angular_velocity_rad_s=self.local_angular_velocity_rad_s,
            epoch_seconds=epoch_seconds,
            reference_frame=reference_frame,
        )

    def iter_stages(self) -> Iterable[Stage]:
        return [stage for stage in self.stages if not stage.is_jettisoned]

    def iter_engines(self) -> Iterable[Engine]:
        return [engine for stage in self.iter_stages() for engine in stage.engines]

    def active_engines(self) -> list[Engine]:
        return [engine for engine in self.iter_engines() if engine.is_active]

    def get_stage(self, stage_name: str) -> Stage:
        for stage in self.stages:
            if stage.name == stage_name and not stage.is_jettisoned:
                return stage
        raise KeyError(f"Unknown active stage {stage_name!r}")

    def get_engine(self, engine_name: str) -> Engine:
        for stage in self.iter_stages():
            for engine in stage.engines:
                if engine.name == engine_name:
                    return engine
        raise KeyError(f"Unknown engine {engine_name!r}")

    def get_tank(self, tank_name: str) -> Tank:
        for stage in self.iter_stages():
            for tank in stage.tanks:
                if tank.name == tank_name:
                    return tank
        raise KeyError(f"Unknown tank {tank_name!r}")

    def arm_engine(self, engine_name: str, throttle: float = 1.0) -> None:
        self.get_engine(engine_name).arm(throttle)

    def shutdown_engine(self, engine_name: str) -> None:
        self.get_engine(engine_name).shutdown()

    def shutdown_all_engines(self) -> None:
        for engine in self.iter_engines():
            engine.shutdown()

    def jettison_stage(self, stage_name: str) -> None:
        self.get_stage(stage_name).jettison()

    def set_guidance_mode(self, mode: GuidanceMode, target_vector: Optional[Vector3] = None) -> None:
        self.guidance_mode = mode
        self.guidance_target_vector = target_vector

    def guidance_vector(self, center_vector: Optional[Vector3] = None) -> Optional[Vector3]:
        if self.guidance_mode == GuidanceMode.MANUAL:
            return None
        if self.guidance_mode == GuidanceMode.INERTIAL_HOLD:
            return self.local_transform.rotation.rotate_vector(Vector3(1.0, 0.0, 0.0))

        if self.guidance_mode == GuidanceMode.PROGRADE and self.local_velocity_km_s.magnitude() > 0.0:
            return self.local_velocity_km_s.normalized()
        if self.guidance_mode == GuidanceMode.RETROGRADE and self.local_velocity_km_s.magnitude() > 0.0:
            return (self.local_velocity_km_s * -1.0).normalized()

        if center_vector is not None and center_vector.magnitude() > 0.0:
            if self.guidance_mode == GuidanceMode.RADIAL_OUT:
                return center_vector.normalized()
            if self.guidance_mode == GuidanceMode.RADIAL_IN:
                return (center_vector * -1.0).normalized()

        if self.guidance_mode == GuidanceMode.TARGET_VECTOR and self.guidance_target_vector is not None:
            if self.guidance_target_vector.magnitude() > 0.0:
                return self.guidance_target_vector.normalized()

        return None

    def update_guidance(self, center_vector: Optional[Vector3] = None) -> None:
        target = self.guidance_vector(center_vector=center_vector)
        if target is None or target.magnitude() == 0.0:
            return
        new_rotation = quaternion_from_two_vectors(Vector3(1.0, 0.0, 0.0), target)
        self.set_local_rotation(new_rotation)

    def compute_current_thrust_newtons(self) -> float:
        return sum(engine.current_thrust_newtons() for engine in self.active_engines())

    def compute_mass_flow_kg_s(self) -> float:
        return sum(engine.current_mass_flow_kg_s() for engine in self.active_engines())

    def compute_thrust_vector_body_frame(self) -> Vector3:
        resultant = Vector3.zero()
        for engine in self.active_engines():
            resultant = resultant + (engine.direction_body_frame * engine.current_thrust_newtons())
        return resultant

    def compute_thrust_vector_inertial_newtons(self) -> Vector3:
        body_vector = self.compute_thrust_vector_body_frame()
        if body_vector.magnitude() == 0.0:
            return Vector3.zero()
        return self.local_transform.rotation.rotate_vector(body_vector)

    def compute_thrust_acceleration_km_s2(self) -> Vector3:
        total_mass = self.total_mass_kg
        if total_mass <= 0.0:
            return Vector3.zero()
        thrust_vector_n = self.compute_thrust_vector_inertial_newtons()
        if thrust_vector_n.magnitude() == 0.0:
            return Vector3.zero()
        return thrust_vector_n * (NEWTON_TO_KMKG_S2 / total_mass)

    def consume_propellant(self, dt_seconds: float) -> float:
        total_requested = 0.0
        total_drawn = 0.0

        for stage in self.iter_stages():
            for engine in stage.engines:
                if not engine.is_active:
                    continue
                requested = engine.current_mass_flow_kg_s() * dt_seconds
                total_requested += requested
                drawn = stage.draw_propellant(requested, engine.tank_names)
                total_drawn += drawn

                if drawn + 1e-9 < requested:
                    engine.shutdown()

        self.last_propellant_consumption_kg = total_drawn
        return total_drawn

    def has_active_propulsion(self) -> bool:
        return any(engine.is_active for engine in self.iter_engines())

    def metadata_summary(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "mass_kg": self.total_mass_kg,
            "dry_mass_kg": self.dry_mass_kg,
            "propellant_mass_kg": self.propellant_mass_kg,
            "delta_v_km_s": self.available_delta_v_km_s,
            "active_engines": [engine.name for engine in self.active_engines()],
            "guidance_mode": self.guidance_mode.value,
        }


def stage_total_propellant_mass(stage: Stage) -> float:
    return stage.propellant_mass_kg


def stage_total_mass(stage: Stage) -> float:
    return stage.total_mass_kg


def dynamic_gravitational_parameter_km3_s2(mass_kg: float) -> float:
    mu_m3_s2 = GRAVITATIONAL_CONSTANT_SI * mass_kg
    return mu_m3_s2 / (KM_TO_M ** 3)


__all__ = [
    "Engine",
    "GuidanceMode",
    "PropellantType",
    "Spacecraft",
    "SpacecraftState",
    "Stage",
    "Tank",
    "ThrusterCluster",
    "dynamic_gravitational_parameter_km3_s2",
    "quaternion_from_two_vectors",
    "stage_total_mass",
    "stage_total_propellant_mass",
]
