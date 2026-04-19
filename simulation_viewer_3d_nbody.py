"""3D viewer with two explicit Earth-system modes.

Mode 1:
- Global Sun + Earth + Moon
- Sun-centered overview so you can see Earth orbiting the Sun and the Moon
  accompanying/orbiting Earth within that context.

Mode 2:
- Local Earth + Moon
- Earth-centered detailed view so the Moon's orbit is easy to inspect.

Keyboard:
- Space: pause
- 1: global Sun-Earth-Moon mode
- 2: local Earth-Moon mode
- Up/Down: speed
- +/-: zoom
- Left/Right: azimuth
- W/S: elevation
- A/D, Q/E, Z/C: pan
- L: labels
- T: trails
- H: help
- R: reset

Mouse:
- Wheel: zoom
- Left drag: rotate
- Right/middle drag: pan
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional, Set

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from nbody_engine import SimulationClock, compute_diagnostics, step_simulation
try:
    from solar_system_factory_nbody_fixed import AU_KM, BODY_SEEDS, SECONDS_PER_DAY, create_default_solar_system_nbody
except ImportError:
    from solar_system_factory_nbody import AU_KM, BODY_SEEDS, SECONDS_PER_DAY, create_default_solar_system_nbody
from space_simulation_models import CelestialBody, Vector3


GLOBAL_MODE_BODIES = {seed.name for seed in BODY_SEEDS}
LOCAL_MODE_BODIES = {"Earth", "Moon"}

GLOBAL_KM_PER_UNIT = AU_KM / 8.0
LOCAL_KM_PER_UNIT = 100_000.0

DEFAULT_REAL_DELTA_SECONDS = 1 / 30
DEFAULT_TIME_SCALE = SECONDS_PER_DAY * 4.0
TRAIL_LENGTH = 900

GLOBAL_DISTANCE_UNITS = 18.0
LOCAL_DISTANCE_UNITS = 6.0


@dataclass(slots=True)
class CameraState:
    distance_units: float = GLOBAL_DISTANCE_UNITS
    azim: float = -60.0
    elev: float = 20.0
    pan_x: float = 0.0
    pan_y: float = 0.0
    pan_z: float = 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="3D viewer for the Sun-Earth-Moon N-body simulation")
    parser.add_argument(
        "--earth-system",
        choices=["earth", "full"],
        default="earth",
        metavar="MODE",
        help="Accepted for compatibility. Both values enable the 2-mode Earth viewer.",
    )
    parser.add_argument(
        "--start-mode",
        choices=["1", "2"],
        default="1",
        help="Start in mode 1 (global) or mode 2 (local).",
    )
    return parser.parse_args()


class NBodyViewer3D:
    def __init__(self, start_mode: str = "1") -> None:
        self.scene = create_default_solar_system_nbody()
        self.clock = SimulationClock(current_time_seconds=0.0, time_scale=DEFAULT_TIME_SCALE)

        self.camera = CameraState()
        self.help_visible = True
        self.show_labels = True
        self.show_trails = True

        self.mode = "global" if start_mode == "1" else "local"

        self.fig = plt.figure(figsize=(13, 9))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self._style_figure()

        self.body_artists: Dict[str, any] = {}
        self.surface_artists: Dict[str, any] = {}
        self.label_artists: Dict[str, any] = {}
        self.trail_artists: Dict[str, any] = {}

        self.trails: Dict[str, deque[tuple[float, float, float]]] = {
            body.name: deque(maxlen=TRAIL_LENGTH) for body in self.scene.bodies
        }

        self.info_text = self.fig.text(
            0.015, 0.015, "", color="white", fontsize=9, family="monospace", va="bottom"
        )
        self.help_text = self.fig.text(
            0.015, 0.985, "", color="white", fontsize=9, family="monospace", va="top"
        )

        self._dragging = False
        self._drag_button: Optional[int] = None
        self._last_mouse_xy: Optional[tuple[float, float]] = None

        self._apply_mode_settings(reset_camera=True)
        self._init_axes()
        self._init_artists()
        self._connect_events()

        self.anim: Optional[FuncAnimation] = None

    @property
    def visible_body_names(self) -> Set[str]:
        return GLOBAL_MODE_BODIES if self.mode == "global" else LOCAL_MODE_BODIES

    @property
    def visible_bodies(self) -> list[CelestialBody]:
        return [body for body in self.scene.bodies if body.name in self.visible_body_names]

    @property
    def km_per_unit(self) -> float:
        return GLOBAL_KM_PER_UNIT if self.mode == "global" else LOCAL_KM_PER_UNIT

    def _follow_center(self) -> Vector3:
        if self.mode == "local":
            return self.scene.earth.global_position
        return Vector3.zero()

    def _apply_mode_settings(self, reset_camera: bool = False) -> None:
        if reset_camera:
            self.camera.azim = -60.0
            self.camera.elev = 20.0
            self.camera.pan_x = 0.0
            self.camera.pan_y = 0.0
            self.camera.pan_z = 0.0
        self.camera.distance_units = GLOBAL_DISTANCE_UNITS if self.mode == "global" else LOCAL_DISTANCE_UNITS

    def _set_mode(self, mode: str) -> None:
        if mode not in {"global", "local"}:
            return
        if self.mode == mode:
            return
        self.mode = mode
        self._apply_mode_settings(reset_camera=True)
        self._clear_trails()
        self._rebuild_visible_artists()
        self.fig.canvas.draw_idle()

    def _clear_trails(self) -> None:
        for points in self.trails.values():
            points.clear()

    def _rebuild_visible_artists(self) -> None:
        self.ax.cla()
        self.body_artists.clear()
        self.surface_artists.clear()
        self.label_artists.clear()
        self.trail_artists.clear()
        self._style_figure()
        self._init_axes()
        self._init_artists()

    def _style_figure(self) -> None:
        self.ax.set_facecolor("black")
        self.fig.patch.set_facecolor("black")
        self.ax.xaxis.label.set_color("white")
        self.ax.yaxis.label.set_color("white")
        self.ax.zaxis.label.set_color("white")
        self.ax.tick_params(colors="white")
        for axis in [self.ax.xaxis, self.ax.yaxis, self.ax.zaxis]:
            axis.pane.set_facecolor((0.0, 0.0, 0.0, 1.0))
            axis.pane.set_edgecolor((0.25, 0.25, 0.25, 1.0))
        self.ax.grid(True, color=(0.2, 0.2, 0.2, 0.25))

    def _init_axes(self) -> None:
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.view_init(elev=self.camera.elev, azim=self.camera.azim)
        self._apply_bounds()

    def _vector_to_units(self, vector: Vector3) -> tuple[float, float, float]:
        return (
            vector.x / self.km_per_unit,
            vector.y / self.km_per_unit,
            vector.z / self.km_per_unit,
        )

    def _apply_bounds(self) -> None:
        d = self.camera.distance_units
        self.ax.set_xlim(-d + self.camera.pan_x, d + self.camera.pan_x)
        self.ax.set_ylim(-d + self.camera.pan_y, d + self.camera.pan_y)
        self.ax.set_zlim(-d + self.camera.pan_z, d + self.camera.pan_z)

    def _marker_size(self, body: CelestialBody) -> float:
        if body.name == "Sun":
            return 12.0
        if body.name == "Earth":
            return 7.0
        if body.name == "Moon":
            return 5.0
        return max(4.0, min(28.0, body.physical_radius_for_render_km() / 30000.0))

    def _sphere_radius_units(self, body: CelestialBody) -> float:
        radius_units = body.radius_km / self.km_per_unit
        if self.mode == "global":
            return max(radius_units, 0.10 if body.name != "Moon" else 0.07)
        return max(radius_units, 0.08 if body.name != "Moon" else 0.06)

    def _draw_body_surface(self, body: CelestialBody, x: float, y: float, z: float):
        radius = self._sphere_radius_units(body)
        color = body.visual_properties.color_hex or "white"
        u = np.linspace(0.0, 2.0 * np.pi, 28)
        v = np.linspace(0.0, np.pi, 14)
        xs = x + radius * np.outer(np.cos(u), np.sin(v))
        ys = y + radius * np.outer(np.sin(u), np.sin(v))
        zs = z + radius * np.outer(np.ones_like(u), np.cos(v))
        return self.ax.plot_surface(
            xs,
            ys,
            zs,
            color=color,
            linewidth=0.0,
            antialiased=True,
            alpha=0.92,
            shade=True,
        )

    def _init_artists(self) -> None:
        center = self._follow_center()
        for body in self.visible_bodies:
            x, y, z = self._vector_to_units(body.global_position - center)
            self.surface_artists[body.name] = self._draw_body_surface(body, x, y, z)
            label = self.ax.text(x, y, z, body.name, color="white", fontsize=8)
            trail, = self.ax.plot([], [], [], linewidth=1.0, alpha=0.70, color=body.visual_properties.color_hex or "white")
            self.label_artists[body.name] = label
            self.trail_artists[body.name] = trail
        self._apply_bounds()

    def _connect_events(self) -> None:
        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("button_press_event", self._on_mouse_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_mouse_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_mouse_move)

    def _on_scroll(self, event) -> None:
        factor = 0.88 if event.button == "up" else 1.12
        self.camera.distance_units = max(0.02, min(800.0, self.camera.distance_units * factor))

    def _on_mouse_press(self, event) -> None:
        if event.inaxes != self.ax:
            return
        self._dragging = True
        self._drag_button = event.button
        self._last_mouse_xy = (event.x, event.y)

    def _on_mouse_release(self, _event) -> None:
        self._dragging = False
        self._drag_button = None
        self._last_mouse_xy = None

    def _on_mouse_move(self, event) -> None:
        if not self._dragging or self._last_mouse_xy is None or event.inaxes != self.ax:
            return
        dx = event.x - self._last_mouse_xy[0]
        dy = event.y - self._last_mouse_xy[1]
        self._last_mouse_xy = (event.x, event.y)

        if self._drag_button == 1:
            self.camera.azim += dx * 0.35
            self.camera.elev = max(-89.0, min(89.0, self.camera.elev - dy * 0.25))
        else:
            pan_scale = self.camera.distance_units * 0.0025
            self.camera.pan_x -= dx * pan_scale
            self.camera.pan_y -= dy * pan_scale

    def _reset_scene(self) -> None:
        self.scene = create_default_solar_system_nbody()
        self.clock = SimulationClock(current_time_seconds=0.0, time_scale=DEFAULT_TIME_SCALE)
        self._apply_mode_settings(reset_camera=True)
        self._clear_trails()
        self._rebuild_visible_artists()
        self.fig.canvas.draw_idle()

    def _on_key_press(self, event) -> None:
        key = (event.key or "").lower()
        pan_step = self.camera.distance_units * 0.05

        if key == " ":
            self.clock.is_paused = not self.clock.is_paused
        elif key == "1":
            self._set_mode("global")
        elif key == "2":
            self._set_mode("local")
        elif key == "up":
            self.clock.time_scale *= 2.0
        elif key == "down":
            self.clock.time_scale = max(60.0, self.clock.time_scale / 2.0)
        elif key in {"+", "="}:
            self.camera.distance_units = max(0.02, self.camera.distance_units * 0.9)
        elif key in {"-", "_"}:
            self.camera.distance_units = min(800.0, self.camera.distance_units * 1.1)
        elif key == "left":
            self.camera.azim -= 5.0
        elif key == "right":
            self.camera.azim += 5.0
        elif key == "w":
            self.camera.elev = min(89.0, self.camera.elev + 3.0)
        elif key == "s":
            self.camera.elev = max(-89.0, self.camera.elev - 3.0)
        elif key == "a":
            self.camera.pan_x -= pan_step
        elif key == "d":
            self.camera.pan_x += pan_step
        elif key == "q":
            self.camera.pan_y += pan_step
        elif key == "e":
            self.camera.pan_y -= pan_step
        elif key == "z":
            self.camera.pan_z -= pan_step
        elif key == "c":
            self.camera.pan_z += pan_step
        elif key == "h":
            self.help_visible = not self.help_visible
        elif key == "l":
            self.show_labels = not self.show_labels
        elif key == "t":
            self.show_trails = not self.show_trails
        elif key == "r":
            self._reset_scene()

        self.fig.canvas.draw_idle()

    def _update_overlay(self) -> None:
        diagnostics = compute_diagnostics(self.scene.root)
        earth_distance_au = self.scene.earth.global_position.magnitude() / AU_KM
        moon_distance_km = (self.scene.moon.global_position - self.scene.earth.global_position).magnitude()
        visible_names = ", ".join(body.name for body in self.visible_bodies)

        mode_description = "1 = Sun+Earth+Moon (global)" if self.mode == "global" else "2 = Earth+Moon (local)"

        self.info_text.set_text(
            "\n".join(
                [
                    f"Mode: {mode_description}",
                    f"Visible: {visible_names}",
                    f"Paused: {'yes' if self.clock.is_paused else 'no'}",
                    f"Time scale: {self.clock.time_scale:,.0f} sim s / real s",
                    f"Sim days: {self.clock.current_time_seconds / SECONDS_PER_DAY:,.3f}",
                    f"Earth-Sun: {earth_distance_au:.5f} AU",
                    f"Earth-Moon: {moon_distance_km:,.0f} km",
                    f"|COM vel|: {diagnostics.center_of_mass_velocity.magnitude():.6f} km/s",
                    f"Trails: {'on' if self.show_trails else 'off'} | Labels: {'on' if self.show_labels else 'off'}",
                ]
            )
        )
        self.help_text.set_text(
            "\n".join(
                [
                    "Space pausa | 1 global Sol-Tierra-Luna | 2 local Tierra-Luna",
                    "↑/↓ velocidad | +/- zoom | ←/→ azimut | W/S elevación",
                    "A/D pan X | Q/E pan Y | Z/C pan Z",
                    "L etiquetas | T trails | H ayuda | R reset",
                    "Mouse: rueda zoom | drag izq rotar | drag dcho/mid mover",
                ]
            ) if self.help_visible else ""
        )

    def _animate(self, _frame: int):
        step_simulation(self.scene.root, self.clock, DEFAULT_REAL_DELTA_SECONDS)

        center = self._follow_center()
        self.ax.view_init(elev=self.camera.elev, azim=self.camera.azim)
        self._apply_bounds()

        artists = []
        for body in self.visible_bodies:
            point = body.global_position - center
            x, y, z = self._vector_to_units(point)

            previous_surface = self.surface_artists.get(body.name)
            if previous_surface is not None:
                previous_surface.remove()
            surface = self._draw_body_surface(body, x, y, z)
            self.surface_artists[body.name] = surface
            artists.append(surface)

            self.trails[body.name].append((x, y, z))
            trail = self.trail_artists[body.name]
            if self.show_trails and len(self.trails[body.name]) >= 2:
                xs, ys, zs = zip(*self.trails[body.name])
                trail.set_data_3d(xs, ys, zs)
                trail.set_visible(True)
            else:
                trail.set_data_3d([], [], [])
                trail.set_visible(False)
            artists.append(trail)

            label = self.label_artists[body.name]
            label.set_position((x, y))
            label.set_3d_properties(z)
            label.set_visible(self.show_labels)
            artists.append(label)

        self._apply_bounds()
        self._update_overlay()
        return artists

    def show(self) -> None:
        self.anim = FuncAnimation(
            self.fig,
            self._animate,
            interval=33,
            blit=False,
            cache_frame_data=False,
        )
        plt.show()


def main() -> None:
    args = parse_args()
    viewer = NBodyViewer3D(start_mode=args.start_mode)
    viewer.show()


if __name__ == "__main__":
    main()
