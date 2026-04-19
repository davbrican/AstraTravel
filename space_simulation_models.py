"""Core domain models for a hierarchical, heliocentric space travel simulator.

Design goals
------------
- Start simple with a heliocentric hierarchy.
- Keep the Sun as a normal node, not as a hardcoded special case.
- Allow future insertion of higher-level frames (e.g. barycenters) without
  breaking the model.
- Separate hierarchy, transforms, physical properties, orbital parameters,
  and visual properties.

Default units
-------------
- Distance: kilometers (km)
- Velocity: kilometers per second (km/s)
- Mass: kilograms (kg)
- Time: seconds (s)
- Orbit angle parameters: degrees
- Quaternion / axis-angle helpers: radians
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import acos, cos, isclose, radians, sin, sqrt
from typing import Any, Iterable, Optional
from uuid import uuid4


GRAVITATIONAL_CONSTANT_SI = 6.67430e-11  # m^3 kg^-1 s^-2
KM_TO_M = 1000.0


class BodyType(str, Enum):
    """Supported categories for celestial bodies."""

    STAR = "star"
    PLANET = "planet"
    MOON = "moon"
    DWARF_PLANET = "dwarf_planet"
    ASTEROID = "asteroid"
    COMET = "comet"
    ARTIFICIAL = "artificial"
    OTHER = "other"


class FrameType(str, Enum):
    """Supported categories for reference frames."""

    ROOT = "root"
    GENERIC = "generic"
    HELIOCENTRIC = "heliocentric"
    BARYCENTRIC = "barycentric"
    PLANET_CENTERED = "planet_centered"
    SHIP_CENTERED = "ship_centered"


@dataclass(slots=True)
class Vector3:
    """Simple immutable-like 3D vector helper.

    Even though this dataclass is mutable, all arithmetic methods return new
    instances and do not mutate the current one.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @staticmethod
    def zero() -> "Vector3":
        return Vector3(0.0, 0.0, 0.0)

    @staticmethod
    def one() -> "Vector3":
        return Vector3(1.0, 1.0, 1.0)

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vector3":
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> "Vector3":
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> "Vector3":
        if isclose(scalar, 0.0):
            raise ZeroDivisionError("Cannot divide a vector by zero.")
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def dot(self, other: "Vector3") -> float:
        return (self.x * other.x) + (self.y * other.y) + (self.z * other.z)

    def cross(self, other: "Vector3") -> "Vector3":
        return Vector3(
            (self.y * other.z) - (self.z * other.y),
            (self.z * other.x) - (self.x * other.z),
            (self.x * other.y) - (self.y * other.x),
        )

    def magnitude(self) -> float:
        return sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> "Vector3":
        magnitude = self.magnitude()
        if isclose(magnitude, 0.0):
            raise ValueError("Cannot normalize a zero vector.")
        return self / magnitude

    def distance_to(self, other: "Vector3") -> float:
        return (self - other).magnitude()

    def angle_to(self, other: "Vector3") -> float:
        denominator = self.magnitude() * other.magnitude()
        if isclose(denominator, 0.0):
            raise ValueError("Cannot compute an angle involving a zero vector.")
        value = max(-1.0, min(1.0, self.dot(other) / denominator))
        return acos(value)

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


@dataclass(slots=True)
class Quaternion:
    """Quaternion used for stable 3D rotations."""

    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @staticmethod
    def identity() -> "Quaternion":
        return Quaternion(1.0, 0.0, 0.0, 0.0)

    @staticmethod
    def from_axis_angle(axis: Vector3, angle_radians: float) -> "Quaternion":
        unit_axis = axis.normalized()
        half_angle = angle_radians / 2.0
        s = sin(half_angle)
        return Quaternion(
            w=cos(half_angle),
            x=unit_axis.x * s,
            y=unit_axis.y * s,
            z=unit_axis.z * s,
        ).normalized()

    def magnitude(self) -> float:
        return sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> "Quaternion":
        magnitude = self.magnitude()
        if isclose(magnitude, 0.0):
            raise ValueError("Cannot normalize a zero quaternion.")
        return Quaternion(
            self.w / magnitude,
            self.x / magnitude,
            self.y / magnitude,
            self.z / magnitude,
        )

    def conjugate(self) -> "Quaternion":
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def __mul__(self, other: "Quaternion") -> "Quaternion":
        return Quaternion(
            w=(self.w * other.w) - (self.x * other.x) - (self.y * other.y) - (self.z * other.z),
            x=(self.w * other.x) + (self.x * other.w) + (self.y * other.z) - (self.z * other.y),
            y=(self.w * other.y) - (self.x * other.z) + (self.y * other.w) + (self.z * other.x),
            z=(self.w * other.z) + (self.x * other.y) - (self.y * other.x) + (self.z * other.w),
        )

    def rotate_vector(self, vector: Vector3) -> Vector3:
        q = self.normalized()
        q_vector = Quaternion(0.0, vector.x, vector.y, vector.z)
        rotated = q * q_vector * q.conjugate()
        return Vector3(rotated.x, rotated.y, rotated.z)


@dataclass(slots=True)
class Transform:
    """Position + rotation relative to a parent frame."""

    position: Vector3 = field(default_factory=Vector3.zero)
    rotation: Quaternion = field(default_factory=Quaternion.identity)

    @staticmethod
    def identity() -> "Transform":
        return Transform(position=Vector3.zero(), rotation=Quaternion.identity())

    @staticmethod
    def combine(parent: "Transform", local: "Transform") -> "Transform":
        """Combine a parent's global transform with a local child transform."""
        rotated_local_position = parent.rotation.rotate_vector(local.position)
        return Transform(
            position=parent.position + rotated_local_position,
            rotation=(parent.rotation * local.rotation).normalized(),
        )


@dataclass(slots=True)
class PhysicalProperties:
    """Physical properties of a celestial body.

    Notes:
    - `radius_km` is the physical radius.
    - `mu_km3_s2` is optional. If omitted, it is derived from the mass using G.
    """

    mass_kg: float
    radius_km: float
    mu_km3_s2: Optional[float] = None

    def __post_init__(self) -> None:
        if self.mass_kg <= 0:
            raise ValueError("mass_kg must be greater than zero.")
        if self.radius_km <= 0:
            raise ValueError("radius_km must be greater than zero.")
        if self.mu_km3_s2 is not None and self.mu_km3_s2 <= 0:
            raise ValueError("mu_km3_s2 must be greater than zero when provided.")

    @property
    def gravitational_parameter_km3_s2(self) -> float:
        if self.mu_km3_s2 is not None:
            return self.mu_km3_s2

        # G gives m^3/s^2. Convert to km^3/s^2 by dividing by 1000^3.
        mu_m3_s2 = GRAVITATIONAL_CONSTANT_SI * self.mass_kg
        return mu_m3_s2 / (KM_TO_M**3)


@dataclass(slots=True)
class RotationProperties:
    """Intrinsic rotation data for a body."""

    axial_tilt_deg: float = 0.0
    rotation_period_seconds: Optional[float] = None
    rotation_axis_local: Vector3 = field(default_factory=lambda: Vector3(0.0, 1.0, 0.0))

    def __post_init__(self) -> None:
        if self.rotation_period_seconds is not None and self.rotation_period_seconds <= 0:
            raise ValueError("rotation_period_seconds must be greater than zero when provided.")
        if isclose(self.rotation_axis_local.magnitude(), 0.0):
            raise ValueError("rotation_axis_local cannot be a zero vector.")
        self.rotation_axis_local = self.rotation_axis_local.normalized()


@dataclass(slots=True)
class OrbitParameters:
    """Keplerian-like orbital parameters.

    These parameters are metadata for the orbit model.
    They do not automatically propagate the body over time yet.
    """

    central_body_id: Optional[str] = None
    semi_major_axis_km: Optional[float] = None
    eccentricity: float = 0.0
    inclination_deg: float = 0.0
    longitude_of_ascending_node_deg: float = 0.0
    argument_of_periapsis_deg: float = 0.0
    true_anomaly_deg: Optional[float] = None
    mean_anomaly_at_epoch_deg: Optional[float] = None
    epoch_seconds: Optional[float] = None

    def __post_init__(self) -> None:
        if self.semi_major_axis_km is not None and self.semi_major_axis_km <= 0:
            raise ValueError("semi_major_axis_km must be greater than zero when provided.")
        if not 0.0 <= self.eccentricity < 1.0:
            raise ValueError("eccentricity must be in the [0, 1) range for elliptical orbits.")


@dataclass(slots=True)
class VisualProperties:
    """Render-oriented data separated from physics."""

    color_hex: Optional[str] = None
    texture_path: Optional[str] = None
    mesh_path: Optional[str] = None
    render_radius_km: Optional[float] = None
    trail_enabled: bool = False

    def __post_init__(self) -> None:
        if self.render_radius_km is not None and self.render_radius_km <= 0:
            raise ValueError("render_radius_km must be greater than zero when provided.")


class Node:
    """Base hierarchical node.

    Both `ReferenceFrame` and `CelestialBody` inherit from this class.
    The main source of truth is the local transform. Global transform is derived
    from the hierarchy.
    """

    def __init__(
        self,
        name: str,
        *,
        node_id: Optional[str] = None,
        parent: Optional["Node"] = None,
        local_transform: Optional[Transform] = None,
        metadata: Optional[dict[str, Any]] = None,
    ) -> None:
        if not name.strip():
            raise ValueError("Node name cannot be empty.")

        self.id: str = node_id or str(uuid4())
        self.name: str = name
        self._parent: Optional[Node] = None
        self.children: list[Node] = []
        self.local_transform: Transform = local_transform or Transform.identity()
        self._global_transform: Transform = Transform.identity()
        self.metadata: dict[str, Any] = metadata.copy() if metadata else {}

        if parent is not None:
            parent.add_child(self)
        else:
            self.update_global_transform(propagate=False)

    @property
    def parent(self) -> Optional["Node"]:
        return self._parent

    @property
    def global_transform(self) -> Transform:
        return self._global_transform

    @property
    def global_position(self) -> Vector3:
        return self._global_transform.position

    @property
    def global_rotation(self) -> Quaternion:
        return self._global_transform.rotation

    def set_local_transform(self, transform: Transform) -> None:
        self.local_transform = transform
        self.update_global_transform()

    def set_local_position(self, position: Vector3) -> None:
        self.local_transform = Transform(position=position, rotation=self.local_transform.rotation)
        self.update_global_transform()

    def set_local_rotation(self, rotation: Quaternion) -> None:
        self.local_transform = Transform(position=self.local_transform.position, rotation=rotation.normalized())
        self.update_global_transform()

    def add_child(self, child: "Node") -> None:
        if child is self:
            raise ValueError("A node cannot be the child of itself.")
        if self.is_descendant_of(child):
            raise ValueError("Cannot create a cycle in the node hierarchy.")
        if child._parent is self:
            return

        if child._parent is not None:
            child._parent.remove_child(child)

        self.children.append(child)
        child._parent = self
        child.update_global_transform()

    def remove_child(self, child: "Node") -> None:
        if child not in self.children:
            raise ValueError(f"Node '{child.name}' is not a child of '{self.name}'.")

        self.children.remove(child)
        child._parent = None
        child.update_global_transform()

    def is_descendant_of(self, candidate_ancestor: "Node") -> bool:
        current = self._parent
        while current is not None:
            if current is candidate_ancestor:
                return True
            current = current._parent
        return False

    def iter_ancestors(self) -> Iterable["Node"]:
        current = self._parent
        while current is not None:
            yield current
            current = current._parent

    def iter_descendants(self) -> Iterable["Node"]:
        for child in self.children:
            yield child
            yield from child.iter_descendants()

    def get_path(self) -> str:
        names = [self.name]
        current = self._parent
        while current is not None:
            names.append(current.name)
            current = current._parent
        return "/".join(reversed(names))

    def find_by_id(self, node_id: str) -> Optional["Node"]:
        if self.id == node_id:
            return self
        for child in self.children:
            found = child.find_by_id(node_id)
            if found is not None:
                return found
        return None

    def find_by_name(self, name: str) -> Optional["Node"]:
        if self.name == name:
            return self
        for child in self.children:
            found = child.find_by_name(name)
            if found is not None:
                return found
        return None

    def update_global_transform(self, *, propagate: bool = True) -> None:
        if self._parent is None:
            self._global_transform = self.local_transform
        else:
            self._global_transform = Transform.combine(self._parent.global_transform, self.local_transform)

        if propagate:
            for child in self.children:
                child.update_global_transform(propagate=True)

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}(name={self.name!r}, id={self.id!r})"


class ReferenceFrame(Node):
    """Hierarchy node that represents a frame or organizational layer."""

    def __init__(
        self,
        name: str,
        *,
        frame_type: FrameType = FrameType.GENERIC,
        description: Optional[str] = None,
        node_id: Optional[str] = None,
        parent: Optional[Node] = None,
        local_transform: Optional[Transform] = None,
        metadata: Optional[dict[str, Any]] = None,
    ) -> None:
        super().__init__(
            name=name,
            node_id=node_id,
            parent=parent,
            local_transform=local_transform,
            metadata=metadata,
        )
        self.frame_type = frame_type
        self.description = description


class CelestialBody(Node):
    """Hierarchical node that represents a physical body."""

    def __init__(
        self,
        name: str,
        *,
        body_type: BodyType,
        physical_properties: PhysicalProperties,
        rotation_properties: Optional[RotationProperties] = None,
        orbit: Optional[OrbitParameters] = None,
        visual_properties: Optional[VisualProperties] = None,
        local_velocity_km_s: Optional[Vector3] = None,
        local_angular_velocity_rad_s: Optional[Vector3] = None,
        node_id: Optional[str] = None,
        parent: Optional[Node] = None,
        local_transform: Optional[Transform] = None,
        metadata: Optional[dict[str, Any]] = None,
    ) -> None:
        super().__init__(
            name=name,
            node_id=node_id,
            parent=parent,
            local_transform=local_transform,
            metadata=metadata,
        )
        self.body_type = body_type
        self.physical_properties = physical_properties
        self.rotation_properties = rotation_properties or RotationProperties()
        self.orbit = orbit or OrbitParameters()
        self.visual_properties = visual_properties or VisualProperties()
        self.local_velocity_km_s = local_velocity_km_s or Vector3.zero()
        self.local_angular_velocity_rad_s = local_angular_velocity_rad_s or Vector3.zero()

    @property
    def mass_kg(self) -> float:
        return self.physical_properties.mass_kg

    @property
    def radius_km(self) -> float:
        return self.physical_properties.radius_km

    @property
    def gravitational_parameter_km3_s2(self) -> float:
        return self.physical_properties.gravitational_parameter_km3_s2

    @property
    def axial_tilt_deg(self) -> float:
        return self.rotation_properties.axial_tilt_deg

    @property
    def rotation_period_seconds(self) -> Optional[float]:
        return self.rotation_properties.rotation_period_seconds

    def set_orbit_central_body(self, central_body: Optional["CelestialBody"]) -> None:
        self.orbit.central_body_id = None if central_body is None else central_body.id

    def physical_radius_for_render_km(self) -> float:
        return self.visual_properties.render_radius_km or self.radius_km


__all__ = [
    "BodyType",
    "FrameType",
    "Vector3",
    "Quaternion",
    "Transform",
    "PhysicalProperties",
    "RotationProperties",
    "OrbitParameters",
    "VisualProperties",
    "Node",
    "ReferenceFrame",
    "CelestialBody",
]
