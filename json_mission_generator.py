"""Build launch mission scenarios from JSON-compatible dictionaries.

The generated controller is intentionally simple: it executes timed mission
steps and leaves the physics to `mission_engine.step_launch_mission_simulation`.
That means gravity, drag, thrust, propellant consumption and variable mass still
come from the existing simulator.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Iterable

from atmosphere_models import EarthExponentialAtmosphere
from launch_mission_profiles import _project_onto_plane, _safe_normalized
from mission_scenarios import (
    LaunchMissionScenario,
    create_apollo_like_launch_vehicle,
    create_artemis_like_launch_vehicle,
)
from nbody_engine import SimulationClock
from solar_system_factory_nbody import create_default_solar_system_nbody
from spacecraft_models import Engine, GuidanceMode, PropellantType, Spacecraft, Stage, Tank
from space_simulation_models import CelestialBody, ReferenceFrame, Vector3


class JsonMissionPhase(str, Enum):
    READY = "json_ready"
    RUNNING = "json_running"
    COMPLETE = "json_complete"


@dataclass(slots=True)
class JsonMissionEvent:
    trigger_time_seconds: float
    event_type: str
    name: str
    engine_name: str | None = None
    stage_name: str | None = None
    throttle: float | None = None
    force_newtons: float | None = None
    direction: Any = None
    executed: bool = False


@dataclass(slots=True)
class JsonMissionController:
    mission_name: str
    spacecraft_name: str
    events: list[JsonMissionEvent]
    phase: JsonMissionPhase = JsonMissionPhase.READY
    recent_events: list[str] = field(default_factory=list)

    def log_event(self, message: str) -> None:
        self.recent_events.append(message)
        self.recent_events = self.recent_events[-8:]

    def update(
        self,
        spacecraft: Spacecraft,
        earth: CelestialBody,
        moon: CelestialBody,
        current_time_seconds: float,
    ) -> None:
        pending = [event for event in self.events if not event.executed]
        if not pending:
            self.phase = JsonMissionPhase.COMPLETE
            return

        for event in pending:
            if current_time_seconds < event.trigger_time_seconds:
                continue
            self._execute_event(event, spacecraft, earth, moon)
            event.executed = True
            self.phase = JsonMissionPhase.RUNNING

    def _execute_event(
        self,
        event: JsonMissionEvent,
        spacecraft: Spacecraft,
        earth: CelestialBody,
        moon: CelestialBody,
    ) -> None:
        if event.event_type == "burn_start":
            if event.direction is not None:
                target = resolve_direction(event.direction, spacecraft, earth, moon)
                spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, target)
                spacecraft.update_guidance()
            if event.engine_name is None:
                raise ValueError(f"Event {event.name!r} requires engine_name")
            throttle = event.throttle
            if event.force_newtons is not None:
                throttle = throttle_for_force(spacecraft, event.engine_name, event.force_newtons)
            spacecraft.arm_engine(event.engine_name, throttle if throttle is not None else 1.0)
            self.log_event(f"{event.name}: burn start {event.engine_name}")
            return

        if event.event_type == "burn_stop":
            if event.engine_name is None:
                spacecraft.shutdown_all_engines()
                self.log_event(f"{event.name}: all engines shutdown")
            else:
                spacecraft.shutdown_engine(event.engine_name)
                self.log_event(f"{event.name}: burn stop {event.engine_name}")
            return

        if event.event_type == "set_guidance":
            if event.direction is None:
                raise ValueError(f"Event {event.name!r} requires direction")
            target = resolve_direction(event.direction, spacecraft, earth, moon)
            spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, target)
            spacecraft.update_guidance()
            self.log_event(f"{event.name}: guidance updated")
            return

        if event.event_type == "jettison_stage":
            if event.stage_name is None:
                raise ValueError(f"Event {event.name!r} requires stage_name")
            spacecraft.jettison_stage(event.stage_name)
            self.log_event(f"{event.name}: stage jettisoned {event.stage_name}")
            return

        if event.event_type == "shutdown":
            if event.engine_name is None:
                spacecraft.shutdown_all_engines()
                self.log_event(f"{event.name}: all engines shutdown")
            else:
                spacecraft.shutdown_engine(event.engine_name)
                self.log_event(f"{event.name}: engine shutdown {event.engine_name}")
            return

        raise ValueError(f"Unsupported event_type {event.event_type!r}")


def generate_mission(mission_json: str | Path | dict[str, Any]) -> LaunchMissionScenario:
    """Generate a launch-ready mission scenario from JSON.

    `mission_json` can be:
    - a dict
    - a JSON string
    - a path to a JSON file

    The returned object has the same shape as `LaunchMissionScenario`, so it can
    be used with `step_launch_mission_simulation`.
    """
    config = load_mission_config(mission_json)
    mission_name = str(config.get("mission_name", "JsonMission"))

    scene = create_default_solar_system_nbody()
    root = scene.root
    body_index = scene.body_index
    earth = body_index["Earth"]
    moon = body_index["Moon"]

    spacecraft = build_spacecraft(config.get("vehicle", "artemis"), root)
    place_spacecraft_from_config(spacecraft, earth, moon, config.get("start", "DEFAULT"))
    root.update_global_transform(propagate=True)

    events = build_events(config.get("steps", []))
    controller = JsonMissionController(
        mission_name=mission_name,
        spacecraft_name=spacecraft.name,
        events=events,
    )
    atmosphere = EarthExponentialAtmosphere(**config.get("atmosphere", {}))
    clock = SimulationClock(
        current_time_seconds=float(config.get("start_time_seconds", 0.0)),
        time_scale=float(config.get("time_scale", 1.0)),
    )

    return LaunchMissionScenario(
        root=root,
        clock=clock,
        body_index=body_index,
        spacecraft=spacecraft,
        controller=controller,  # type: ignore[arg-type]
        atmosphere=atmosphere,
    )


def load_mission_config(source: str | Path | dict[str, Any]) -> dict[str, Any]:
    if isinstance(source, dict):
        return source
    if isinstance(source, Path):
        return json.loads(source.read_text(encoding="utf-8"))
    text = str(source)
    if text.lstrip().startswith("{"):
        return json.loads(text)
    maybe_path = Path(text)
    if maybe_path.exists():
        return json.loads(maybe_path.read_text(encoding="utf-8"))
    return json.loads(text)


def build_spacecraft(vehicle_config: Any, root: ReferenceFrame) -> Spacecraft:
    if isinstance(vehicle_config, str):
        vehicle_name = vehicle_config.lower()
        if vehicle_name == "apollo":
            spacecraft, _program = create_apollo_like_launch_vehicle(parent=root)
            return spacecraft
        if vehicle_name == "artemis":
            spacecraft, _program = create_artemis_like_launch_vehicle(parent=root)
            return spacecraft
        raise ValueError("vehicle must be 'apollo', 'artemis' or an object")

    if not isinstance(vehicle_config, dict):
        raise TypeError("vehicle must be a string or object")

    stages = [build_stage(stage_config) for stage_config in vehicle_config.get("stages", [])]
    if not stages:
        raise ValueError("Custom vehicle requires at least one stage")

    return Spacecraft(
        name=str(vehicle_config.get("name", "JsonVehicle")),
        stages=stages,
        dry_radius_km=float(vehicle_config.get("dry_radius_km", 5.0)),
        color_hex=str(vehicle_config.get("color_hex", "#FFAA55")),
        parent=root,
        metadata=dict(vehicle_config.get("metadata", {})),
    )


def build_stage(stage_config: dict[str, Any]) -> Stage:
    return Stage(
        name=str(stage_config["name"]),
        dry_mass_kg=float(stage_config["dry_mass_kg"]),
        tanks=[build_tank(tank_config) for tank_config in stage_config.get("tanks", [])],
        engines=[build_engine(engine_config) for engine_config in stage_config.get("engines", [])],
        metadata=dict(stage_config.get("metadata", {})),
    )


def build_tank(tank_config: dict[str, Any]) -> Tank:
    propellant = PropellantType(str(tank_config.get("propellant_type", PropellantType.GENERIC.value)))
    capacity = float(tank_config["capacity_kg"])
    current = float(tank_config.get("current_mass_kg", capacity))
    return Tank(
        name=str(tank_config["name"]),
        propellant_type=propellant,
        capacity_kg=capacity,
        current_mass_kg=current,
        priority=int(tank_config.get("priority", 0)),
        metadata=dict(tank_config.get("metadata", {})),
    )


def build_engine(engine_config: dict[str, Any]) -> Engine:
    return Engine(
        name=str(engine_config["name"]),
        max_thrust_newtons=float(engine_config["max_thrust_newtons"]),
        specific_impulse_seconds=float(engine_config["specific_impulse_seconds"]),
        tank_names=[str(name) for name in engine_config.get("tank_names", [])],
        direction_body_frame=vector_from_sequence(engine_config.get("direction_body_frame", [1.0, 0.0, 0.0])),
        min_throttle=float(engine_config.get("min_throttle", 0.0)),
        max_throttle=float(engine_config.get("max_throttle", 1.0)),
        restartable=bool(engine_config.get("restartable", True)),
        remaining_ignitions=engine_config.get("remaining_ignitions"),
        gimbal_limit_deg=float(engine_config.get("gimbal_limit_deg", 0.0)),
        role=str(engine_config.get("role", "main")),
    )


def place_spacecraft_from_config(
    spacecraft: Spacecraft,
    earth: CelestialBody,
    moon: CelestialBody,
    start_config: Any,
) -> None:
    if start_config == "DEFAULT" or start_config is None:
        place_on_default_launch_pad(spacecraft, earth, moon)
        return

    if not isinstance(start_config, dict):
        raise TypeError("start must be 'DEFAULT' or an object")

    start_type = str(start_config.get("type", "DEFAULT")).lower()
    if start_type == "default":
        place_on_default_launch_pad(spacecraft, earth, moon)
        return

    if start_type == "coordinates":
        frame = str(start_config.get("frame", "absolute")).lower()
        position = vector_from_sequence(start_config["position_km"])
        velocity = vector_from_sequence(start_config.get("velocity_km_s", [0.0, 0.0, 0.0]))
        if frame == "earth_relative":
            position = earth.global_position + position
            velocity = earth.local_velocity_km_s + velocity
        elif frame != "absolute":
            raise ValueError("start.frame must be 'absolute' or 'earth_relative'")
        spacecraft.set_local_position(position)
        spacecraft.local_velocity_km_s = velocity
        direction = start_config.get("direction", "radial_out")
        spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, resolve_direction(direction, spacecraft, earth, moon))
        spacecraft.update_guidance()
        return

    if start_type == "earth_surface":
        radial = vector_from_sequence(start_config.get("radial", [1.0, 0.0, 0.0])).normalized()
        altitude_km = float(start_config.get("altitude_km", 0.1))
        tangential = vector_from_sequence(start_config.get("tangential", [0.0, 1.0, 0.0])).normalized()
        rotation_speed = float(start_config.get("earth_rotation_speed_km_s", 0.465))
        spacecraft.set_local_position(earth.global_position + radial * (earth.radius_km + altitude_km))
        spacecraft.local_velocity_km_s = earth.local_velocity_km_s + tangential * rotation_speed
        spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, radial)
        spacecraft.update_guidance()
        return

    raise ValueError("start.type must be 'DEFAULT', 'coordinates' or 'earth_surface'")


def place_on_default_launch_pad(spacecraft: Spacecraft, earth: CelestialBody, moon: CelestialBody) -> None:
    earth_orbit_normal = earth.global_position.cross(earth.local_velocity_km_s)
    if earth_orbit_normal.magnitude() == 0.0:
        earth_orbit_normal = Vector3(0.0, 0.0, 1.0)
    normal = _safe_normalized(earth_orbit_normal)

    earth_to_moon = moon.global_position - earth.global_position
    projected = _project_onto_plane(earth_to_moon, normal)
    if projected.magnitude() == 0.0:
        projected = _project_onto_plane(earth.global_position * -1.0, normal)
    radial = _safe_normalized(projected)
    tangential = _safe_normalized(normal.cross(radial))

    spacecraft.set_local_position(earth.global_position + radial * (earth.radius_km + 0.1))
    spacecraft.local_velocity_km_s = earth.local_velocity_km_s + tangential * 0.465
    spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, radial)
    spacecraft.update_guidance()


def build_events(steps: Iterable[dict[str, Any]]) -> list[JsonMissionEvent]:
    events: list[JsonMissionEvent] = []
    for index, step in enumerate(steps):
        step_type = str(step.get("type", step.get("action", "burn"))).lower()
        start = float(step.get("at", step.get("time_seconds", 0.0)))
        name = str(step.get("name", f"step_{index}_{step_type}"))

        if step_type == "burn":
            duration = float(step["duration"])
            engine_name = str(step["engine"])
            events.append(
                JsonMissionEvent(
                    trigger_time_seconds=start,
                    event_type="burn_start",
                    name=f"{name}_start",
                    engine_name=engine_name,
                    throttle=step.get("throttle"),
                    force_newtons=step.get("force_newtons"),
                    direction=step.get("direction"),
                )
            )
            events.append(
                JsonMissionEvent(
                    trigger_time_seconds=start + duration,
                    event_type="burn_stop",
                    name=f"{name}_stop",
                    engine_name=engine_name,
                )
            )
            continue

        if step_type == "set_guidance":
            events.append(
                JsonMissionEvent(
                    trigger_time_seconds=start,
                    event_type="set_guidance",
                    name=name,
                    direction=step["direction"],
                )
            )
            continue

        if step_type == "jettison_stage":
            events.append(
                JsonMissionEvent(
                    trigger_time_seconds=start,
                    event_type="jettison_stage",
                    name=name,
                    stage_name=str(step["stage"]),
                )
            )
            continue

        if step_type == "shutdown":
            events.append(
                JsonMissionEvent(
                    trigger_time_seconds=start,
                    event_type="shutdown",
                    name=name,
                    engine_name=step.get("engine"),
                )
            )
            continue

        raise ValueError(f"Unsupported step type {step_type!r}")

    return sorted(events, key=lambda event: event.trigger_time_seconds)


def resolve_direction(direction: Any, spacecraft: Spacecraft, earth: CelestialBody, moon: CelestialBody) -> Vector3:
    if isinstance(direction, str):
        key = direction.lower()
        radial = spacecraft.global_position - earth.global_position
        relative_velocity = spacecraft.local_velocity_km_s - earth.local_velocity_km_s

        if key == "radial_out":
            return radial.normalized()
        if key == "radial_in":
            return (radial * -1.0).normalized()
        if key == "prograde":
            return relative_velocity.normalized()
        if key == "retrograde":
            return (relative_velocity * -1.0).normalized()
        if key == "tangential":
            normal = earth.global_position.cross(earth.local_velocity_km_s)
            if normal.magnitude() == 0.0:
                normal = Vector3(0.0, 0.0, 1.0)
            return normal.normalized().cross(radial.normalized()).normalized()
        if key == "moon":
            return (moon.global_position - spacecraft.global_position).normalized()
        raise ValueError(f"Unsupported direction {direction!r}")

    if isinstance(direction, dict):
        blend = Vector3.zero()
        has_blend_component = False
        for key in ("radial_out", "radial_in", "prograde", "retrograde", "tangential", "moon"):
            if key not in direction:
                continue
            weight = float(direction[key])
            if weight == 0.0:
                continue
            blend = blend + resolve_direction(key, spacecraft, earth, moon) * weight
            has_blend_component = True
        if has_blend_component:
            if blend.magnitude() == 0.0:
                raise ValueError("Blended direction cannot resolve to a zero vector")
            return blend.normalized()

        if "vector" not in direction:
            raise ValueError("Direction object requires 'vector' or weighted direction keys")
        vector = vector_from_sequence(direction["vector"])
        frame = str(direction.get("frame", "absolute")).lower()
        if frame == "absolute":
            return vector.normalized()
        if frame == "earth_relative":
            return vector.normalized()
        raise ValueError("direction.frame must be 'absolute' or 'earth_relative'")

    return vector_from_sequence(direction).normalized()


def throttle_for_force(spacecraft: Spacecraft, engine_name: str, force_newtons: float) -> float:
    engine = spacecraft.get_engine(engine_name)
    requested = float(force_newtons)
    max_thrust = engine.max_thrust_newtons * engine.max_throttle
    min_thrust = engine.max_thrust_newtons * engine.min_throttle
    if requested > max_thrust:
        raise ValueError(
            f"Requested force {requested} N exceeds max available force {max_thrust} N for engine {engine_name!r}"
        )
    if requested < min_thrust:
        raise ValueError(
            f"Requested force {requested} N is below min available force {min_thrust} N for engine {engine_name!r}"
        )
    return requested / engine.max_thrust_newtons


def vector_from_sequence(value: Any) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("Vector values must be [x, y, z]")
    return Vector3(float(value[0]), float(value[1]), float(value[2]))


__all__ = [
    "JsonMissionController",
    "JsonMissionEvent",
    "JsonMissionPhase",
    "build_events",
    "generate_mission",
    "load_mission_config",
    "place_on_default_launch_pad",
    "resolve_direction",
]
