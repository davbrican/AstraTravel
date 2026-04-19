
"""Programmed translunar mission controller.

This module drives a staged launch vehicle from the launch pad to:
- vertical ascent
- gravity turn
- parking orbit insertion
- translunar injection
- translunar coast
- lunar flyby / return coast

The controller is intentionally simplified but physically grounded:
- thrust comes from the spacecraft engine model,
- propellant mass really decreases,
- stages are jettisoned,
- guidance is updated from current flight geometry.

It is designed to be reusable by different launch vehicle presets.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import atan2, degrees, sqrt
from typing import Optional

from spacecraft_models import GuidanceMode, Spacecraft
from space_simulation_models import CelestialBody, Vector3


class MissionPhase(str, Enum):
    PRELAUNCH = "prelaunch"
    ASCENT_STAGE_1 = "ascent_stage_1"
    ASCENT_STAGE_2 = "ascent_stage_2"
    ASCENT_STAGE_3 = "ascent_stage_3"
    PARKING_COAST = "parking_coast"
    TLI_BURN = "tli_burn"
    TRANSLUNAR_COAST = "translunar_coast"
    LUNAR_FLYBY = "lunar_flyby"
    RETURN_COAST = "return_coast"
    COMPLETE = "complete"


@dataclass(slots=True)
class OrbitalSnapshot:
    altitude_km: float
    speed_km_s: float
    radial_speed_km_s: float
    circular_speed_km_s: float
    specific_energy_km2_s2: float
    eccentricity: float
    semimajor_axis_km: Optional[float]
    periapsis_altitude_km: Optional[float]
    apoapsis_altitude_km: Optional[float]


def _safe_normalized(vector: Vector3) -> Vector3:
    magnitude = vector.magnitude()
    if magnitude == 0.0:
        raise ValueError("Cannot normalize a zero vector.")
    return vector / magnitude


def _project_onto_plane(vector: Vector3, plane_normal: Vector3) -> Vector3:
    normal = _safe_normalized(plane_normal)
    return vector - normal * vector.dot(normal)


def _current_stage_has_propellant(spacecraft: Spacecraft, stage_name: str) -> bool:
    try:
        return spacecraft.get_stage(stage_name).propellant_mass_kg > 1.0
    except KeyError:
        return False


def _stage_exists(spacecraft: Spacecraft, stage_name: Optional[str]) -> bool:
    if stage_name is None:
        return False
    try:
        spacecraft.get_stage(stage_name)
        return True
    except KeyError:
        return False


def _engine_exists(spacecraft: Spacecraft, engine_name: Optional[str]) -> bool:
    if engine_name is None:
        return False
    try:
        spacecraft.get_engine(engine_name)
        return True
    except KeyError:
        return False


def compute_earth_relative_snapshot(spacecraft: Spacecraft, earth: CelestialBody) -> OrbitalSnapshot:
    r_vec = spacecraft.global_position - earth.global_position
    v_vec = spacecraft.local_velocity_km_s - earth.local_velocity_km_s

    r = r_vec.magnitude()
    v = v_vec.magnitude()
    radial = _safe_normalized(r_vec)
    radial_speed = v_vec.dot(radial)
    mu = earth.gravitational_parameter_km3_s2

    circular_speed = sqrt(mu / max(r, 1e-9))
    specific_energy = 0.5 * v * v - mu / max(r, 1e-9)

    h_vec = r_vec.cross(v_vec)
    e_vec = (v_vec.cross(h_vec) / mu) - radial
    e = e_vec.magnitude()

    if specific_energy < 0.0:
        a = -mu / (2.0 * specific_energy)
        periapsis_radius = a * (1.0 - e)
        apoapsis_radius = a * (1.0 + e)
        periapsis_alt = periapsis_radius - earth.radius_km
        apoapsis_alt = apoapsis_radius - earth.radius_km
    else:
        a = None
        periapsis_alt = None
        apoapsis_alt = None

    return OrbitalSnapshot(
        altitude_km=r - earth.radius_km,
        speed_km_s=v,
        radial_speed_km_s=radial_speed,
        circular_speed_km_s=circular_speed,
        specific_energy_km2_s2=specific_energy,
        eccentricity=e,
        semimajor_axis_km=a,
        periapsis_altitude_km=periapsis_alt,
        apoapsis_altitude_km=apoapsis_alt,
    )


@dataclass(slots=True)
class LaunchVehicleProgram:
    ascent_stage_names: list[str]
    ascent_engine_names: list[str]
    parking_stage_name: Optional[str]
    parking_engine_name: Optional[str]
    service_stage_name: Optional[str]
    service_engine_name: Optional[str]
    parking_orbit_target_altitude_km: float = 185.0
    parking_coast_duration_seconds: float = 20.0 * 60.0
    translunar_transfer_time_seconds: float = 72.0 * 3600.0
    target_lunar_distance_km: float = 384_400.0
    stage_jettison_delay_seconds: float = 2.0


@dataclass(slots=True)
class TransLunarMissionController:
    spacecraft_name: str
    program: LaunchVehicleProgram
    phase: MissionPhase = MissionPhase.PRELAUNCH
    phase_start_time_seconds: float = 0.0
    recent_events: list[str] = field(default_factory=list)
    launch_plane_normal: Optional[Vector3] = None
    launch_radial_axis: Optional[Vector3] = None
    launch_tangential_axis: Optional[Vector3] = None
    pending_jettison_stage_name: Optional[str] = None
    pending_jettison_time_seconds: Optional[float] = None
    minimum_moon_distance_km: float = 1e30
    last_moon_distance_km: float = 1e30
    tli_start_time_seconds: Optional[float] = None

    def log_event(self, message: str) -> None:
        self.recent_events.append(message)
        self.recent_events = self.recent_events[-8:]

    def clear_recent_events(self) -> None:
        self.recent_events.clear()

    def _set_phase(self, new_phase: MissionPhase, current_time_seconds: float, note: Optional[str] = None) -> None:
        if new_phase == self.phase:
            return
        self.phase = new_phase
        self.phase_start_time_seconds = current_time_seconds
        if note:
            self.log_event(note)

    def _arm_engine(self, spacecraft: Spacecraft, engine_name: Optional[str], throttle: float = 1.0) -> None:
        if engine_name and _engine_exists(spacecraft, engine_name):
            spacecraft.arm_engine(engine_name, throttle)

    def _shutdown_engine(self, spacecraft: Spacecraft, engine_name: Optional[str]) -> None:
        if engine_name and _engine_exists(spacecraft, engine_name):
            spacecraft.shutdown_engine(engine_name)

    def _schedule_stage_jettison(self, stage_name: Optional[str], current_time_seconds: float) -> None:
        if stage_name is None or not _stage_exists(self_spacecraft, stage_name):  # type: ignore[name-defined]
            return

    def _try_pending_jettison(self, spacecraft: Spacecraft, current_time_seconds: float) -> None:
        if self.pending_jettison_stage_name is None or self.pending_jettison_time_seconds is None:
            return
        if current_time_seconds < self.pending_jettison_time_seconds:
            return
        stage_name = self.pending_jettison_stage_name
        if _stage_exists(spacecraft, stage_name):
            spacecraft.jettison_stage(stage_name)
            self.log_event(f"Stage jettisoned: {stage_name}")
        self.pending_jettison_stage_name = None
        self.pending_jettison_time_seconds = None

    def _queue_jettison(self, stage_name: str, current_time_seconds: float) -> None:
        self.pending_jettison_stage_name = stage_name
        self.pending_jettison_time_seconds = current_time_seconds + self.program.stage_jettison_delay_seconds

    def _initialize_launch_frame(self, earth: CelestialBody, moon: CelestialBody) -> None:
        if self.launch_plane_normal is not None:
            return
        earth_heliocentric_normal = earth.global_position.cross(earth.local_velocity_km_s)
        if earth_heliocentric_normal.magnitude() == 0.0:
            earth_heliocentric_normal = Vector3(0.0, 0.0, 1.0)
        self.launch_plane_normal = _safe_normalized(earth_heliocentric_normal)

        earth_to_moon = moon.global_position - earth.global_position
        projected = _project_onto_plane(earth_to_moon, self.launch_plane_normal)
        if projected.magnitude() == 0.0:
            projected = _project_onto_plane(earth.global_position * -1.0, self.launch_plane_normal)
        self.launch_radial_axis = _safe_normalized(projected)
        self.launch_tangential_axis = _safe_normalized(self.launch_plane_normal.cross(self.launch_radial_axis))

    def place_on_launch_pad(self, spacecraft: Spacecraft, earth: CelestialBody, moon: CelestialBody) -> None:
        self._initialize_launch_frame(earth, moon)
        radial = self.launch_radial_axis
        tangential = self.launch_tangential_axis
        assert radial is not None and tangential is not None

        launch_altitude_km = 0.1
        earth_rotation_speed_km_s = 0.465

        position = earth.global_position + radial * (earth.radius_km + launch_altitude_km)
        velocity = earth.local_velocity_km_s + tangential * earth_rotation_speed_km_s

        spacecraft.set_local_position(position)
        spacecraft.local_velocity_km_s = velocity
        spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, radial)
        spacecraft.update_guidance()
        self.log_event("Vehicle placed on launch pad")

    def _gravity_turn_target(self, altitude_km: float, radial: Vector3, tangential: Vector3, aggressiveness: float = 1.0) -> Vector3:
        # 0 km -> mostly vertical, 180 km -> mostly horizontal
        pitch_fraction = max(0.0, min(1.0, (altitude_km - 1.0) / 180.0))
        pitch_fraction = min(1.0, pitch_fraction * aggressiveness)
        target = radial * (1.0 - pitch_fraction) + tangential * pitch_fraction
        return _safe_normalized(target)

    def _predicted_moon_target(self, spacecraft: Spacecraft, moon: CelestialBody, current_time_seconds: float) -> Vector3:
        if self.tli_start_time_seconds is None:
            lead_time = self.program.translunar_transfer_time_seconds
        else:
            elapsed = current_time_seconds - self.tli_start_time_seconds
            lead_time = max(6.0 * 3600.0, self.program.translunar_transfer_time_seconds - elapsed)
        predicted = moon.global_position + moon.local_velocity_km_s * lead_time
        target = predicted - spacecraft.global_position
        if target.magnitude() == 0.0:
            target = moon.global_position - spacecraft.global_position
        return _safe_normalized(target)

    def _activate_ascent_stage(self, spacecraft: Spacecraft, stage_name: str, engine_name: str, current_time_seconds: float, phase: MissionPhase) -> None:
        self._arm_engine(spacecraft, engine_name, throttle=1.0)
        self._set_phase(phase, current_time_seconds, note=f"Ignition: {engine_name}")

    def update(self, spacecraft: Spacecraft, earth: CelestialBody, moon: CelestialBody, current_time_seconds: float) -> None:
        self._initialize_launch_frame(earth, moon)
        self._try_pending_jettison(spacecraft, current_time_seconds)

        radial_vec = spacecraft.global_position - earth.global_position
        radial = _safe_normalized(radial_vec)
        normal = self.launch_plane_normal if self.launch_plane_normal is not None else Vector3(0.0, 0.0, 1.0)
        tangential = _safe_normalized(normal.cross(radial))
        snapshot = compute_earth_relative_snapshot(spacecraft, earth)
        moon_distance_km = (moon.global_position - spacecraft.global_position).magnitude()
        self.minimum_moon_distance_km = min(self.minimum_moon_distance_km, moon_distance_km)

        if self.phase == MissionPhase.PRELAUNCH:
            spacecraft.shutdown_all_engines()
            spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, radial)
            self._activate_ascent_stage(
                spacecraft,
                self.program.ascent_stage_names[0],
                self.program.ascent_engine_names[0],
                current_time_seconds,
                MissionPhase.ASCENT_STAGE_1,
            )
            return

        if self.phase == MissionPhase.ASCENT_STAGE_1:
            spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, self._gravity_turn_target(snapshot.altitude_km, radial, tangential, aggressiveness=0.75))
            stage_name = self.program.ascent_stage_names[0]
            if not _current_stage_has_propellant(spacecraft, stage_name):
                self._shutdown_engine(spacecraft, self.program.ascent_engine_names[0])
                self._queue_jettison(stage_name, current_time_seconds)
                next_stage_name = self.program.ascent_stage_names[1] if len(self.program.ascent_stage_names) > 1 else self.program.parking_stage_name
                next_engine_name = self.program.ascent_engine_names[1] if len(self.program.ascent_engine_names) > 1 else self.program.parking_engine_name
                if next_stage_name and next_engine_name:
                    self._arm_engine(spacecraft, next_engine_name, 1.0)
                self._set_phase(MissionPhase.ASCENT_STAGE_2, current_time_seconds, note=f"Stage burnout: {stage_name}")
                return

        if self.phase == MissionPhase.ASCENT_STAGE_2:
            spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, self._gravity_turn_target(snapshot.altitude_km, radial, tangential, aggressiveness=1.0))
            current_stage_name = self.program.ascent_stage_names[1] if len(self.program.ascent_stage_names) > 1 else self.program.parking_stage_name
            current_engine_name = self.program.ascent_engine_names[1] if len(self.program.ascent_engine_names) > 1 else self.program.parking_engine_name

            reached_parking = (
                snapshot.altitude_km >= self.program.parking_orbit_target_altitude_km - 10.0
                and snapshot.speed_km_s >= snapshot.circular_speed_km_s * 0.97
                and abs(snapshot.radial_speed_km_s) <= 0.35
            )

            if reached_parking:
                self._shutdown_engine(spacecraft, current_engine_name)
                self._set_phase(MissionPhase.PARKING_COAST, current_time_seconds, note="Parking orbit reached")
                return

            if current_stage_name and not _current_stage_has_propellant(spacecraft, current_stage_name):
                self._shutdown_engine(spacecraft, current_engine_name)
                if current_stage_name != self.program.parking_stage_name:
                    self._queue_jettison(current_stage_name, current_time_seconds)
                if self.program.parking_stage_name and self.program.parking_stage_name != current_stage_name and self.program.parking_engine_name:
                    self._arm_engine(spacecraft, self.program.parking_engine_name, 1.0)
                    self._set_phase(MissionPhase.ASCENT_STAGE_3, current_time_seconds, note=f"Stage burnout: {current_stage_name}")
                else:
                    self._set_phase(MissionPhase.PARKING_COAST, current_time_seconds, note="Upper stage coast")
                return

        if self.phase == MissionPhase.ASCENT_STAGE_3:
            spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, self._gravity_turn_target(snapshot.altitude_km, radial, tangential, aggressiveness=1.15))
            current_stage_name = self.program.parking_stage_name
            current_engine_name = self.program.parking_engine_name

            reached_parking = (
                snapshot.altitude_km >= self.program.parking_orbit_target_altitude_km - 10.0
                and snapshot.speed_km_s >= snapshot.circular_speed_km_s * 0.985
                and abs(snapshot.radial_speed_km_s) <= 0.25
            )

            if reached_parking:
                self._shutdown_engine(spacecraft, current_engine_name)
                self._set_phase(MissionPhase.PARKING_COAST, current_time_seconds, note="Parking orbit reached")
                return

            if current_stage_name and not _current_stage_has_propellant(spacecraft, current_stage_name):
                self._shutdown_engine(spacecraft, current_engine_name)
                self._set_phase(MissionPhase.PARKING_COAST, current_time_seconds, note=f"Upper stage depleted: {current_stage_name}")
                return

        if self.phase == MissionPhase.PARKING_COAST:
            spacecraft.shutdown_all_engines()
            spacecraft.set_guidance_mode(GuidanceMode.PROGRADE)
            if current_time_seconds - self.phase_start_time_seconds >= self.program.parking_coast_duration_seconds:
                engine_to_use = None
                if self.program.parking_stage_name and _current_stage_has_propellant(spacecraft, self.program.parking_stage_name):
                    engine_to_use = self.program.parking_engine_name
                elif self.program.service_stage_name and _current_stage_has_propellant(spacecraft, self.program.service_stage_name):
                    engine_to_use = self.program.service_engine_name
                if engine_to_use is not None:
                    self.tli_start_time_seconds = current_time_seconds
                    spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, self._predicted_moon_target(spacecraft, moon, current_time_seconds))
                    self._arm_engine(spacecraft, engine_to_use, 1.0)
                    self._set_phase(MissionPhase.TLI_BURN, current_time_seconds, note=f"TLI ignition: {engine_to_use}")
                else:
                    self._set_phase(MissionPhase.TRANSLUNAR_COAST, current_time_seconds, note="No engine available for TLI")
                return

        if self.phase == MissionPhase.TLI_BURN:
            spacecraft.set_guidance_mode(GuidanceMode.TARGET_VECTOR, self._predicted_moon_target(spacecraft, moon, current_time_seconds))
            apoapsis_alt = snapshot.apoapsis_altitude_km or -1.0
            target_apoapsis_alt = self.program.target_lunar_distance_km - earth.radius_km
            current_stage_name = self.program.parking_stage_name if _current_stage_has_propellant(spacecraft, self.program.parking_stage_name or "") else self.program.service_stage_name
            current_engine_name = self.program.parking_engine_name if _current_stage_has_propellant(spacecraft, self.program.parking_stage_name or "") else self.program.service_engine_name

            reached_tli = apoapsis_alt >= target_apoapsis_alt * 0.92 or snapshot.specific_energy_km2_s2 > -0.2
            if reached_tli:
                self._shutdown_engine(spacecraft, current_engine_name)
                if current_stage_name == self.program.parking_stage_name and current_stage_name is not None:
                    self._queue_jettison(current_stage_name, current_time_seconds)
                self._set_phase(MissionPhase.TRANSLUNAR_COAST, current_time_seconds, note="TLI burn complete")
                return

            if current_stage_name and not _current_stage_has_propellant(spacecraft, current_stage_name):
                self._shutdown_engine(spacecraft, current_engine_name)
                if current_stage_name == self.program.parking_stage_name:
                    self._queue_jettison(current_stage_name, current_time_seconds)
                if current_stage_name != self.program.service_stage_name and self.program.service_stage_name and _current_stage_has_propellant(spacecraft, self.program.service_stage_name):
                    self._arm_engine(spacecraft, self.program.service_engine_name, 1.0)
                    self.log_event(f"Switched to service engine: {self.program.service_engine_name}")
                else:
                    self._set_phase(MissionPhase.TRANSLUNAR_COAST, current_time_seconds, note="TLI stage depleted")
                return

        if self.phase == MissionPhase.TRANSLUNAR_COAST:
            spacecraft.shutdown_all_engines()
            spacecraft.set_guidance_mode(GuidanceMode.PROGRADE)
            if moon_distance_km < 120_000.0:
                self._set_phase(MissionPhase.LUNAR_FLYBY, current_time_seconds, note="Entering lunar encounter")
                self.last_moon_distance_km = moon_distance_km
                return

        if self.phase == MissionPhase.LUNAR_FLYBY:
            spacecraft.shutdown_all_engines()
            spacecraft.set_guidance_mode(GuidanceMode.PROGRADE)
            if moon_distance_km > self.last_moon_distance_km and self.minimum_moon_distance_km < 80_000.0:
                self._set_phase(MissionPhase.RETURN_COAST, current_time_seconds, note="Lunar flyby complete")
            self.last_moon_distance_km = moon_distance_km
            return

        if self.phase == MissionPhase.RETURN_COAST:
            spacecraft.shutdown_all_engines()
            spacecraft.set_guidance_mode(GuidanceMode.INERTIAL_HOLD)
