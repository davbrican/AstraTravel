"""Factory helpers for a complete Solar System N-body scene.

This module creates a dynamic N-body scene using heliocentric initial
conditions derived from classical orbital elements at a reference epoch.

Design decisions
----------------
- For N-body integration, all bodies are direct children of the simulation root.
  Their local positions are interpreted as global heliocentric coordinates.
- The hierarchy is therefore organizational, not orbital.
- Initial states are derived from orbital elements for a simple but grounded
  baseline. They are not high-precision ephemerides.
- Distances are in kilometers, velocities in km/s, masses in kilograms.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, radians, sin, sqrt
from typing import Dict, Iterable, Optional

from space_simulation_models import (
    BodyType,
    CelestialBody,
    FrameType,
    OrbitParameters,
    PhysicalProperties,
    Quaternion,
    ReferenceFrame,
    RotationProperties,
    Transform,
    Vector3,
    VisualProperties,
)

AU_KM = 149_597_870.7
SECONDS_PER_DAY = 86_400.0
J2000_JULIAN_DAY = 2451545.0
LUNAR_RELATIVE_INCLINATION_DEG = 5.145


@dataclass(frozen=True, slots=True)
class OrbitalElements:
    semi_major_axis_au: float
    eccentricity: float
    inclination_deg: float
    longitude_of_ascending_node_deg: float
    longitude_of_perihelion_deg: float
    mean_longitude_deg: float

    @property
    def argument_of_periapsis_deg(self) -> float:
        return self.longitude_of_perihelion_deg - self.longitude_of_ascending_node_deg

    @property
    def mean_anomaly_deg(self) -> float:
        return self.mean_longitude_deg - self.longitude_of_perihelion_deg


@dataclass(frozen=True, slots=True)
class BodySeed:
    name: str
    body_type: BodyType
    mass_kg: float
    radius_km: float
    mu_km3_s2: float
    axial_tilt_deg: float
    rotation_period_seconds: Optional[float]
    color_hex: str
    render_radius_km: float
    orbital_elements: Optional[OrbitalElements] = None
    central_body_name: Optional[str] = None


@dataclass(slots=True)
class NBodySolarSystemScene:
    root: ReferenceFrame
    bodies: list[CelestialBody]
    body_index: Dict[str, CelestialBody]

    @property
    def sun(self) -> CelestialBody:
        return self.body_index["Sun"]

    @property
    def earth(self) -> CelestialBody:
        return self.body_index["Earth"]

    @property
    def moon(self) -> CelestialBody:
        return self.body_index["Moon"]


BODY_SEEDS: tuple[BodySeed, ...] = (
    BodySeed(
        name="Sun",
        body_type=BodyType.STAR,
        mass_kg=1.9885e30,
        radius_km=696_340.0,
        mu_km3_s2=132_712_440_018.0,
        axial_tilt_deg=7.25,
        rotation_period_seconds=25.05 * SECONDS_PER_DAY,
        color_hex="#FDB813",
        render_radius_km=696_340.0 * 8.0,
    ),
    BodySeed(
        name="Mercury",
        body_type=BodyType.PLANET,
        mass_kg=3.3011e23,
        radius_km=2439.7,
        mu_km3_s2=22_032.080,
        axial_tilt_deg=0.034,
        rotation_period_seconds=58.646 * SECONDS_PER_DAY,
        color_hex="#B7B8B9",
        render_radius_km=2439.7 * 90.0,
        orbital_elements=OrbitalElements(0.38709927, 0.20563593, 7.00497902, 48.33076593, 77.45779628, 252.25032350),
        central_body_name="Sun",
    ),
    BodySeed(
        name="Venus",
        body_type=BodyType.PLANET,
        mass_kg=4.8675e24,
        radius_km=6051.8,
        mu_km3_s2=324_858.592,
        axial_tilt_deg=177.36,
        rotation_period_seconds=243.025 * SECONDS_PER_DAY,
        color_hex="#E3C28B",
        render_radius_km=6051.8 * 45.0,
        orbital_elements=OrbitalElements(0.72333566, 0.00677672, 3.39467605, 76.67984255, 131.60246718, 181.97909950),
        central_body_name="Sun",
    ),
    BodySeed(
        name="Earth",
        body_type=BodyType.PLANET,
        mass_kg=5.97219e24,
        radius_km=6371.0,
        mu_km3_s2=398_600.4418,
        axial_tilt_deg=23.439281,
        rotation_period_seconds=86_164.0905,
        color_hex="#4C82FF",
        render_radius_km=6371.0 * 45.0,
        orbital_elements=OrbitalElements(1.00000261, 0.01671123, 0.00001531, 0.0, 102.93768193, 100.46457166),
        central_body_name="Sun",
    ),
    BodySeed(
        name="Moon",
        body_type=BodyType.MOON,
        mass_kg=7.342e22,
        radius_km=1737.4,
        mu_km3_s2=4902.800066,
        axial_tilt_deg=6.68,
        rotation_period_seconds=27.321661 * SECONDS_PER_DAY,
        color_hex="#D9D9D9",
        render_radius_km=1737.4 * 140.0,
        # Rough geocentric mean elements. Good enough as a first N-body seed.
        orbital_elements=OrbitalElements(384400.0 / AU_KM, 0.0549, 5.145, 125.08, 318.15, 218.316),
        central_body_name="Earth",
    ),
    BodySeed(
        name="Mars",
        body_type=BodyType.PLANET,
        mass_kg=6.4171e23,
        radius_km=3389.5,
        mu_km3_s2=42_828.375214,
        axial_tilt_deg=25.19,
        rotation_period_seconds=88_642.6848,
        color_hex="#C96B4B",
        render_radius_km=3389.5 * 70.0,
        orbital_elements=OrbitalElements(1.52371034, 0.09339410, 1.84969142, 49.55953891, 336.04084, 355.45332),
        central_body_name="Sun",
    ),
    BodySeed(
        name="Jupiter",
        body_type=BodyType.PLANET,
        mass_kg=1.8982e27,
        radius_km=69_911.0,
        mu_km3_s2=126_686_534.0,
        axial_tilt_deg=3.13,
        rotation_period_seconds=35_729.856,
        color_hex="#D9B38C",
        render_radius_km=69_911.0 * 8.0,
        orbital_elements=OrbitalElements(5.20288700, 0.04838624, 1.30439695, 100.47390909, 14.72847983, 34.39644051),
        central_body_name="Sun",
    ),
    BodySeed(
        name="Saturn",
        body_type=BodyType.PLANET,
        mass_kg=5.6834e26,
        radius_km=58_232.0,
        mu_km3_s2=37_931_187.0,
        axial_tilt_deg=26.73,
        rotation_period_seconds=38_362.4,
        color_hex="#D8C087",
        render_radius_km=58_232.0 * 8.0,
        orbital_elements=OrbitalElements(9.53667594, 0.05386179, 2.48599187, 113.66242448, 92.59887831, 49.95424423),
        central_body_name="Sun",
    ),
    BodySeed(
        name="Uranus",
        body_type=BodyType.PLANET,
        mass_kg=8.6810e25,
        radius_km=25_362.0,
        mu_km3_s2=5_793_939.0,
        axial_tilt_deg=97.77,
        rotation_period_seconds=62_064.0,
        color_hex="#8ED6E8",
        render_radius_km=25_362.0 * 10.0,
        orbital_elements=OrbitalElements(19.18916464, 0.04725744, 0.77263783, 74.01692503, 170.95427630, 313.23810451),
        central_body_name="Sun",
    ),
    BodySeed(
        name="Neptune",
        body_type=BodyType.PLANET,
        mass_kg=1.02413e26,
        radius_km=24_622.0,
        mu_km3_s2=6_836_529.0,
        axial_tilt_deg=28.32,
        rotation_period_seconds=57_996.0,
        color_hex="#426BFF",
        render_radius_km=24_622.0 * 10.0,
        orbital_elements=OrbitalElements(30.06992276, 0.00859048, 1.77004347, 131.78422574, 44.96476227, 304.87997031),
        central_body_name="Sun",
    ),
)


def create_simulation_root() -> ReferenceFrame:
    return ReferenceFrame(
        name="SimulationRoot",
        frame_type=FrameType.ROOT,
        description="Root frame for the Solar System N-body simulation.",
        local_transform=Transform.identity(),
    )


def _normalize_angle_rad(angle_rad: float) -> float:
    two_pi = 2.0 * 3.141592653589793
    return angle_rad % two_pi


def _solve_kepler(mean_anomaly_rad: float, eccentricity: float, iterations: int = 15) -> float:
    eccentric_anomaly = mean_anomaly_rad if eccentricity < 0.8 else 3.141592653589793
    for _ in range(iterations):
        f = eccentric_anomaly - eccentricity * sin(eccentric_anomaly) - mean_anomaly_rad
        fp = 1.0 - eccentricity * cos(eccentric_anomaly)
        eccentric_anomaly -= f / fp
    return eccentric_anomaly


def _orbital_state_from_elements(
    elements: OrbitalElements,
    central_mu_km3_s2: float,
) -> tuple[Vector3, Vector3]:
    """Return heliocentric/geocentric state vector from osculating elements.

    Frame convention:
    - XY is the ecliptic plane.
    - Z is the out-of-plane axis.
    """
    a = elements.semi_major_axis_au * AU_KM
    e = elements.eccentricity
    i = radians(elements.inclination_deg)
    omega = radians(elements.longitude_of_ascending_node_deg)
    w = radians(elements.argument_of_periapsis_deg)
    m = _normalize_angle_rad(radians(elements.mean_anomaly_deg))

    E = _solve_kepler(m, e)
    cos_E = cos(E)
    sin_E = sin(E)
    r = a * (1.0 - e * cos_E)
    nu = 2.0 * atan2(sqrt(1.0 + e) * sin_E / 2.0, sqrt(1.0 - e) * (1.0 + cos_E) / 2.0)
    # Numerically safer explicit formula for true anomaly.
    nu = 2.0 * atan2(sqrt(1 + e) * sin(E / 2.0), sqrt(1 - e) * cos(E / 2.0))
    p = a * (1.0 - e * e)

    x_p = r * cos(nu)
    y_p = r * sin(nu)

    v_factor = sqrt(central_mu_km3_s2 / p)
    vx_p = -v_factor * sin(nu)
    vy_p = v_factor * (e + cos(nu))

    cos_O = cos(omega)
    sin_O = sin(omega)
    cos_i = cos(i)
    sin_i = sin(i)
    cos_w = cos(w)
    sin_w = sin(w)

    r11 = cos_O * cos_w - sin_O * sin_w * cos_i
    r12 = -cos_O * sin_w + sin_O * cos_w * cos_i
    r21 = sin_O * cos_w + cos_O * sin_w * cos_i
    r22 = -sin_O * sin_w + cos_O * cos_w * cos_i
    r31 = sin_w * sin_i
    r32 = cos_w * sin_i

    position = Vector3(
        r11 * x_p + r12 * y_p,
        r21 * x_p + r22 * y_p,
        r31 * x_p + r32 * y_p,
    )
    velocity = Vector3(
        r11 * vx_p + r12 * vy_p,
        r21 * vx_p + r22 * vy_p,
        r31 * vx_p + r32 * vy_p,
    )
    return position, velocity


def _build_body(seed: BodySeed, parent: ReferenceFrame) -> CelestialBody:
    return CelestialBody(
        name=seed.name,
        body_type=seed.body_type,
        physical_properties=PhysicalProperties(
            mass_kg=seed.mass_kg,
            radius_km=seed.radius_km,
            mu_km3_s2=seed.mu_km3_s2,
        ),
        rotation_properties=RotationProperties(
            axial_tilt_deg=seed.axial_tilt_deg,
            rotation_period_seconds=seed.rotation_period_seconds,
        ),
        orbit=OrbitParameters(),
        visual_properties=VisualProperties(
            color_hex=seed.color_hex,
            render_radius_km=seed.render_radius_km,
            trail_enabled=True,
        ),
        parent=parent,
        local_transform=Transform.identity(),
        metadata={"central_body_name": seed.central_body_name, "seed_name": seed.name},
    )


def _safe_normalized(vector: Vector3) -> Vector3:
    magnitude = vector.magnitude()
    if magnitude == 0.0:
        raise ValueError("Cannot normalize a zero vector in orbital basis construction.")
    return vector / magnitude


def _build_earth_orbital_basis(earth_position: Vector3, earth_velocity: Vector3) -> tuple[Vector3, Vector3, Vector3]:
    """Build a local orbital basis from Earth's heliocentric state.

    Returns:
    - radial_axis: outward from the Sun through Earth
    - tangential_axis: prograde direction along Earth's orbit
    - normal_axis: orbital angular momentum direction
    """
    radial_axis = _safe_normalized(earth_position)
    normal_axis = _safe_normalized(earth_position.cross(earth_velocity))
    tangential_axis = _safe_normalized(normal_axis.cross(radial_axis))
    return radial_axis, tangential_axis, normal_axis


def _build_lunar_relative_state(
    earth_position: Vector3,
    earth_velocity: Vector3,
    earth_mu_km3_s2: float,
    moon_mu_km3_s2: float,
    orbit_radius_km: float,
    relative_inclination_deg: float,
) -> tuple[Vector3, Vector3]:
    """Construct Moon relative state in Earth's local orbital frame.

    By default this seeds the Moon in the same plane as Earth's heliocentric
    orbit so the Earth-Moon system is visually coherent in a Sun-centered view.
    """
    radial_axis, tangential_axis, _normal_axis = _build_earth_orbital_basis(earth_position, earth_velocity)

    moon_speed_km_s = sqrt((earth_mu_km3_s2 + moon_mu_km3_s2) / orbit_radius_km)
    rel_position = radial_axis * orbit_radius_km
    rel_velocity = tangential_axis * moon_speed_km_s

    if relative_inclination_deg != 0.0:
        inclination_quaternion = Quaternion.from_axis_angle(radial_axis, radians(relative_inclination_deg))
        rel_velocity = inclination_quaternion.rotate_vector(rel_velocity)

    return rel_position, rel_velocity


def _apply_initial_state(body_index: Dict[str, CelestialBody]) -> None:
    sun = body_index["Sun"]
    sun.set_local_transform(Transform(position=Vector3.zero(), rotation=Quaternion.identity()))
    sun.local_velocity_km_s = Vector3.zero()

    # Earth orbital elements are best interpreted as the Earth-Moon barycenter.
    # We therefore initialize the EMB first and then split Earth and Moon around it.
    earth_seed = next(seed for seed in BODY_SEEDS if seed.name == "Earth")
    moon_seed = next(seed for seed in BODY_SEEDS if seed.name == "Moon")

    emb_position, emb_velocity = _orbital_state_from_elements(
        earth_seed.orbital_elements,
        central_mu_km3_s2=sun.gravitational_parameter_km3_s2,
    )

    # Seed the Moon in Earth's local orbital frame instead of in fixed world axes.
    # This makes the Moon orbit coherently around Earth within the same overall
    # heliocentric orbital context as Earth.
    moon_rel_position, moon_rel_velocity = _build_lunar_relative_state(
        earth_position=emb_position,
        earth_velocity=emb_velocity,
        earth_mu_km3_s2=body_index["Earth"].gravitational_parameter_km3_s2,
        moon_mu_km3_s2=body_index["Moon"].gravitational_parameter_km3_s2,
        orbit_radius_km=384_400.0,
        relative_inclination_deg=LUNAR_RELATIVE_INCLINATION_DEG,
    )

    earth = body_index["Earth"]
    moon = body_index["Moon"]
    total_em_mass = earth.mass_kg + moon.mass_kg
    earth_ratio = moon.mass_kg / total_em_mass
    moon_ratio = earth.mass_kg / total_em_mass

    earth.set_local_position(emb_position - moon_rel_position * earth_ratio)
    earth.local_velocity_km_s = emb_velocity - moon_rel_velocity * earth_ratio
    moon.set_local_position(emb_position + moon_rel_position * moon_ratio)
    moon.local_velocity_km_s = emb_velocity + moon_rel_velocity * moon_ratio

    for seed in BODY_SEEDS:
        if seed.name in {"Sun", "Earth", "Moon"} or seed.orbital_elements is None or seed.central_body_name is None:
            continue

        central_body = body_index[seed.central_body_name]
        position_rel, velocity_rel = _orbital_state_from_elements(
            seed.orbital_elements,
            central_mu_km3_s2=central_body.gravitational_parameter_km3_s2,
        )
        absolute_position = central_body.local_transform.position + position_rel
        absolute_velocity = central_body.local_velocity_km_s + velocity_rel

        body = body_index[seed.name]
        body.set_local_position(absolute_position)
        body.local_velocity_km_s = absolute_velocity

    _recenter_momentum(body_index.values())


def _recenter_momentum(bodies: Iterable[CelestialBody]) -> None:
    total_mass = 0.0
    total_momentum = Vector3.zero()
    for body in bodies:
        total_mass += body.mass_kg
        total_momentum += body.local_velocity_km_s * body.mass_kg

    if total_mass <= 0.0:
        return

    barycentric_velocity = total_momentum / total_mass
    for body in bodies:
        body.local_velocity_km_s = body.local_velocity_km_s - barycentric_velocity


def create_default_solar_system_nbody() -> NBodySolarSystemScene:
    root = create_simulation_root()
    bodies: list[CelestialBody] = []
    body_index: Dict[str, CelestialBody] = {}

    for seed in BODY_SEEDS:
        body = _build_body(seed, root)
        bodies.append(body)
        body_index[seed.name] = body

    _apply_initial_state(body_index)
    root.update_global_transform(propagate=True)
    return NBodySolarSystemScene(root=root, bodies=bodies, body_index=body_index)


__all__ = [
    "AU_KM",
    "BodySeed",
    "BODY_SEEDS",
    "NBodySolarSystemScene",
    "OrbitalElements",
    "SECONDS_PER_DAY",
    "create_default_solar_system_nbody",
    "create_simulation_root",
]
