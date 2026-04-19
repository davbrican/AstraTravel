
"""Atmosphere model placeholders for future higher-fidelity entry/drag work."""

from __future__ import annotations

from dataclasses import dataclass
from math import exp


@dataclass(slots=True)
class AtmosphereSample:
    density_kg_m3: float
    pressure_pa: float
    temperature_k: float


class EarthExponentialAtmosphere:
    """Simple exponential atmosphere.

    This is intentionally lightweight and only meant as a placeholder until the
    project plugs in a higher-fidelity model such as NRLMSISE-00.
    """

    def __init__(
        self,
        *,
        rho0_kg_m3: float = 1.225,
        scale_height_m: float = 8500.0,
        temperature_k: float = 288.15,
        pressure_pa: float = 101325.0,
    ) -> None:
        self.rho0_kg_m3 = rho0_kg_m3
        self.scale_height_m = scale_height_m
        self.temperature_k = temperature_k
        self.pressure_pa = pressure_pa

    def sample(self, altitude_m: float) -> AtmosphereSample:
        altitude_m = max(0.0, altitude_m)
        density = self.rho0_kg_m3 * exp(-altitude_m / self.scale_height_m)
        pressure = self.pressure_pa * exp(-altitude_m / self.scale_height_m)
        return AtmosphereSample(
            density_kg_m3=density,
            pressure_pa=pressure,
            temperature_k=self.temperature_k,
        )


__all__ = [
    "AtmosphereSample",
    "EarthExponentialAtmosphere",
]
