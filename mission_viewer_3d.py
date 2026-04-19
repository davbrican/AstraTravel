"""3D viewer for launch-to-Moon mission scenarios.

This viewer focuses on the new launch mission pipeline:
- launch from Earth surface
- ascent with propellant depletion and staging
- parking orbit
- TLI
- translunar coast / lunar flyby

Views:
- 1: Earth-local
- 2: Earth-Moon
- 3: spacecraft-local
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from json_mission_generator import generate_mission
from launch_mission_profiles import compute_earth_relative_snapshot
from mission_engine import step_launch_mission_simulation
from mission_scenarios import create_translunar_launch_scenario
from nbody_engine import compute_diagnostics
from solar_system_factory_nbody import AU_KM, SECONDS_PER_DAY
from spacecraft_models import Spacecraft
from space_simulation_models import CelestialBody, Vector3


EARTH_LOCAL_KM_PER_UNIT = 200.0
EARTH_MOON_KM_PER_UNIT = 50_000.0
SPACECRAFT_LOCAL_KM_PER_UNIT = 20.0

EARTH_LOCAL_DISTANCE_UNITS = 250.0
EARTH_MOON_DISTANCE_UNITS = 12.0
SPACECRAFT_LOCAL_DISTANCE_UNITS = 60.0

DEFAULT_REAL_DELTA_SECONDS = 1.0 / 30.0
TRAIL_LENGTH = 1200


@dataclass(slots=True)
class CameraState:
    distance_units: float = EARTH_LOCAL_DISTANCE_UNITS
    azim: float = -65.0
    elev: float = 22.0
    pan_x: float = 0.0
    pan_y: float = 0.0
    pan_z: float = 0.0


class MissionViewer3DV2:
    def __init__(self, vehicle: str = "artemis", start_mode: str = "1", mission_json: str | None = None) -> None:
        self.vehicle = vehicle
        self.mission_json = mission_json
        self.mission_label = mission_json or vehicle
        self._build_scenario()

        self.mode = {"1": "earth_local", "2": "earth_moon", "3": "spacecraft_local"}.get(start_mode, "earth_local")
        self.camera = CameraState()
        self.help_visible = True
        self.show_labels = True
        self.show_trails = True

        self.fig = plt.figure(figsize=(14, 9))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self._style_figure()

        self.body_artists: Dict[str, any] = {}
        self.surface_artists: Dict[str, any] = {}
        self.label_artists: Dict[str, any] = {}
        self.trail_artists: Dict[str, any] = {}
        self.trails: Dict[str, deque[tuple[float, float, float]]] = {
            body.name: deque(maxlen=TRAIL_LENGTH) for body in self.visible_bodies
        }

        self.info_text = self.fig.text(0.015, 0.015, "", color="white", fontsize=9, family="monospace", va="bottom")
        self.help_text = self.fig.text(0.015, 0.985, "", color="white", fontsize=9, family="monospace", va="top")

        self._dragging = False
        self._drag_button: Optional[int] = None
        self._last_mouse_xy: Optional[tuple[float, float]] = None

        self._apply_mode_settings(reset_camera=True)
        self._init_axes()
        self._init_artists()
        self._connect_events()

        self.anim: Optional[FuncAnimation] = None
        self.last_step_diag = None

    def _build_scenario(self) -> None:
        if self.mission_json is not None:
            self.scenario = generate_mission(self.mission_json)
            self.mission_label = self.scenario.controller.mission_name
        else:
            self.scenario = create_translunar_launch_scenario(self.vehicle)
            self.mission_label = self.vehicle
        self.root = self.scenario.root
        self.clock = self.scenario.clock
        self.spacecraft: Spacecraft = self.scenario.spacecraft
        self.controller = self.scenario.controller
        self.atmosphere = self.scenario.atmosphere
        self.body_index = self.scenario.body_index
        self.earth: CelestialBody = self.body_index["Earth"]
        self.moon: CelestialBody = self.body_index["Moon"]
        self.sun: CelestialBody = self.body_index["Sun"]

    @property
    def visible_bodies(self) -> list[CelestialBody]:
        if self.mode == "earth_local":
            return [self.earth, self.moon, self.spacecraft]
        if self.mode == "earth_moon":
            return [self.earth, self.moon, self.spacecraft, self.sun]
        return [self.spacecraft, self.earth, self.moon]

    @property
    def km_per_unit(self) -> float:
        if self.mode == "earth_local":
            return EARTH_LOCAL_KM_PER_UNIT
        if self.mode == "earth_moon":
            return EARTH_MOON_KM_PER_UNIT
        return SPACECRAFT_LOCAL_KM_PER_UNIT

    def _follow_center(self) -> Vector3:
        if self.mode == "earth_local":
            return self.earth.global_position
        if self.mode == "spacecraft_local":
            return self.spacecraft.global_position
        return self.earth.global_position

    def _apply_mode_settings(self, reset_camera: bool = False) -> None:
        if reset_camera:
            self.camera.azim = -65.0
            self.camera.elev = 22.0
            self.camera.pan_x = 0.0
            self.camera.pan_y = 0.0
            self.camera.pan_z = 0.0
        if self.mode == "earth_local":
            self.camera.distance_units = EARTH_LOCAL_DISTANCE_UNITS
        elif self.mode == "earth_moon":
            self.camera.distance_units = EARTH_MOON_DISTANCE_UNITS
        else:
            self.camera.distance_units = SPACECRAFT_LOCAL_DISTANCE_UNITS

    def _set_mode(self, mode: str) -> None:
        if mode not in {"earth_local", "earth_moon", "spacecraft_local"} or mode == self.mode:
            return
        self.mode = mode
        self._apply_mode_settings(reset_camera=True)
        self._clear_trails()
        self._rebuild_visible_artists()
        self.fig.canvas.draw_idle()

    def _clear_trails(self) -> None:
        self.trails = {body.name: deque(maxlen=TRAIL_LENGTH) for body in self.visible_bodies}

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
        return (vector.x / self.km_per_unit, vector.y / self.km_per_unit, vector.z / self.km_per_unit)

    def _apply_bounds(self) -> None:
        d = self.camera.distance_units
        self.ax.set_xlim(-d + self.camera.pan_x, d + self.camera.pan_x)
        self.ax.set_ylim(-d + self.camera.pan_y, d + self.camera.pan_y)
        self.ax.set_zlim(-d + self.camera.pan_z, d + self.camera.pan_z)

    def _marker_size(self, body: CelestialBody) -> float:
        if body.name == "Sun":
            return 10.0
        if body.name == "Earth":
            return 8.0
        if body.name == "Moon":
            return 5.0
        return 6.0

    def _sphere_radius_units(self, body: CelestialBody) -> float:
        radius_units = body.radius_km / self.km_per_unit
        if self.mode == "earth_local":
            return max(radius_units, 0.08)
        if self.mode == "earth_moon":
            return max(radius_units, 0.08 if body.name != "Moon" else 0.06)
        return max(radius_units, 0.06)

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
            if isinstance(body, Spacecraft):
                marker, = self.ax.plot([x], [y], [z], marker="o", linestyle="", markersize=self._marker_size(body), color=body.visual_properties.color_hex or "white")
                self.body_artists[body.name] = marker
            else:
                self.surface_artists[body.name] = self._draw_body_surface(body, x, y, z)
            label = self.ax.text(x, y, z, body.name, color="white", fontsize=8)
            trail, = self.ax.plot([], [], [], linewidth=1.0, alpha=0.7, color=body.visual_properties.color_hex or "white")
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
        self.camera.distance_units = max(0.02, min(4000.0, self.camera.distance_units * factor))

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

    def _reset(self) -> None:
        mode = self.mode
        self._build_scenario()
        self._clear_trails()
        self.mode = mode
        self._apply_mode_settings(reset_camera=True)
        self._rebuild_visible_artists()
        self.fig.canvas.draw_idle()

    def _on_key_press(self, event) -> None:
        key = (event.key or "").lower()
        pan_step = self.camera.distance_units * 0.05
        if key == " ":
            self.clock.is_paused = not self.clock.is_paused
        elif key == "1":
            self._set_mode("earth_local")
        elif key == "2":
            self._set_mode("earth_moon")
        elif key == "3":
            self._set_mode("spacecraft_local")
        elif key == "up":
            self.clock.time_scale *= 2.0
        elif key == "down":
            self.clock.time_scale = max(0.125, self.clock.time_scale / 2.0)
        elif key in {"+", "="}:
            self.camera.distance_units = max(0.02, self.camera.distance_units * 0.9)
        elif key in {"-", "_"}:
            self.camera.distance_units = min(4000.0, self.camera.distance_units * 1.1)
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
        elif key == "l":
            self.show_labels = not self.show_labels
        elif key == "t":
            self.show_trails = not self.show_trails
        elif key == "h":
            self.help_visible = not self.help_visible
        elif key == "r":
            self._reset()
        self.fig.canvas.draw_idle()

    def _update_overlay(self) -> None:
        diag = compute_diagnostics(self.root)
        snap = compute_earth_relative_snapshot(self.spacecraft, self.earth)
        moon_distance_km = (self.moon.global_position - self.spacecraft.global_position).magnitude()
        active_engines = ", ".join(engine.name for engine in self.spacecraft.active_engines()) or "none"
        stages = ", ".join(stage.name for stage in self.spacecraft.iter_stages())
        mode_desc = {
            "earth_local": "1 = Earth-local",
            "earth_moon": "2 = Earth-Moon",
            "spacecraft_local": "3 = spacecraft-local",
        }[self.mode]
        events = self.controller.recent_events[-4:]
        self.info_text.set_text(
            "\n".join([
                f"Mission: {self.mission_label}",
                f"Vehicle: {self.spacecraft.name}",
                f"Mode: {mode_desc}",
                f"Phase: {self.controller.phase.value}",
                f"Paused: {'yes' if self.clock.is_paused else 'no'}",
                f"Time scale: {self.clock.time_scale:.3f}x",
                f"Mission time: {self.clock.current_time_seconds:.1f} s ({self.clock.current_time_seconds / SECONDS_PER_DAY:.3f} d)",
                f"Altitude: {snap.altitude_km:,.1f} km",
                f"Speed: {snap.speed_km_s:,.3f} km/s",
                f"Propellant: {self.spacecraft.propellant_mass_kg:,.1f} kg",
                f"Total mass: {self.spacecraft.total_mass_kg:,.1f} kg",
                f"Available Δv: {self.spacecraft.available_delta_v_km_s:,.3f} km/s",
                f"Moon distance: {moon_distance_km:,.0f} km",
                f"Active engines: {active_engines}",
                f"Active stages: {stages}",
                f"|COM vel|: {diag.center_of_mass_velocity.magnitude():.6f} km/s",
                *( ["Events:"] + [f"- {evt}" for evt in events] if events else [] ),
            ])
        )
        self.help_text.set_text(
            "\n".join([
                "Space pause | 1 Earth-local | 2 Earth-Moon | 3 spacecraft-local",
                "↑/↓ time scale | +/- zoom | ←/→ azimuth | W/S elevation",
                "A/D pan X | Q/E pan Y | Z/C pan Z",
                "L labels | T trails | H help | R reset",
                "Mouse: wheel zoom | left drag rotate | right/middle drag pan",
            ]) if self.help_visible else ""
        )

    def _animate(self, _frame: int):
        self.last_step_diag = step_launch_mission_simulation(
            self.root,
            self.clock,
            self.controller,
            self.atmosphere,
            DEFAULT_REAL_DELTA_SECONDS,
        )
        center = self._follow_center()
        self.ax.view_init(elev=self.camera.elev, azim=self.camera.azim)
        self._apply_bounds()

        artists = []
        for body in self.visible_bodies:
            point = body.global_position - center
            x, y, z = self._vector_to_units(point)
            if isinstance(body, Spacecraft):
                marker = self.body_artists[body.name]
                marker.set_data_3d([x], [y], [z])
                artists.append(marker)
            else:
                previous_surface = self.surface_artists.get(body.name)
                if previous_surface is not None:
                    previous_surface.remove()
                surface = self._draw_body_surface(body, x, y, z)
                self.surface_artists[body.name] = surface
                artists.append(surface)

            if self.show_trails:
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
        self.anim = FuncAnimation(self.fig, self._animate, interval=33, blit=False, cache_frame_data=False)
        plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="3D viewer for translunar launch mission scenarios")
    parser.add_argument("--vehicle", choices=["artemis", "apollo"], default="artemis")
    parser.add_argument("--mission-json", help="Load a mission generated from a JSON file instead of --vehicle.")
    parser.add_argument("--start-mode", choices=["1", "2", "3"], default="1")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    MissionViewer3DV2(vehicle=args.vehicle, start_mode=args.start_mode, mission_json=args.mission_json).show()


if __name__ == "__main__":
    main()
