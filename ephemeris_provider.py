
"""Ephemeris provider interfaces prepared for future SPICE integration."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from space_simulation_models import Quaternion, Vector3


@dataclass(slots=True)
class EphemerisState:
    position_km: Vector3
    velocity_km_s: Vector3
    attitude: Quaternion = Quaternion.identity()
    angular_velocity_rad_s: Vector3 = Vector3.zero()
    reference_frame: str = "J2000"


class EphemerisProvider(ABC):
    @abstractmethod
    def get_state(self, body_name: str, epoch_seconds: float) -> EphemerisState:
        raise NotImplementedError


class StaticEnvironmentEphemerisProvider(EphemerisProvider):
    """Very small adapter around an existing body index.

    This is only a stopgap interface so the project can adopt a clean provider
    shape before integrating SPICE kernels.
    """

    def __init__(self, body_index: dict[str, object]) -> None:
        self.body_index = body_index

    def get_state(self, body_name: str, epoch_seconds: float) -> EphemerisState:
        body = self.body_index[body_name]
        return EphemerisState(
            position_km=body.global_position,  # type: ignore[attr-defined]
            velocity_km_s=body.local_velocity_km_s,  # type: ignore[attr-defined]
            attitude=body.global_rotation,  # type: ignore[attr-defined]
            angular_velocity_rad_s=body.local_angular_velocity_rad_s,  # type: ignore[attr-defined]
            reference_frame="SimulationRoot",
        )


__all__ = [
    "EphemerisProvider",
    "EphemerisState",
    "StaticEnvironmentEphemerisProvider",
]
