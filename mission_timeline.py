
"""Mission timeline and event system for spacecraft scenarios."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional

from spacecraft_models import GuidanceMode, Spacecraft


@dataclass(slots=True)
class MissionActionContext:
    spacecraft_index: dict[str, Spacecraft]
    timeline: "MissionTimeline"


class MissionAction:
    def execute(self, context: MissionActionContext) -> None:  # pragma: no cover - interface
        raise NotImplementedError


@dataclass(slots=True)
class StartBurnAction(MissionAction):
    spacecraft_name: str
    engine_name: str
    throttle: float = 1.0
    guidance_mode: GuidanceMode = GuidanceMode.PROGRADE
    target_vector: Optional[object] = None

    def execute(self, context: MissionActionContext) -> None:
        spacecraft = context.spacecraft_index[self.spacecraft_name]
        spacecraft.set_guidance_mode(self.guidance_mode, self.target_vector)  # type: ignore[arg-type]
        spacecraft.arm_engine(self.engine_name, self.throttle)


@dataclass(slots=True)
class StopBurnAction(MissionAction):
    spacecraft_name: str
    engine_name: Optional[str] = None

    def execute(self, context: MissionActionContext) -> None:
        spacecraft = context.spacecraft_index[self.spacecraft_name]
        if self.engine_name is None:
            spacecraft.shutdown_all_engines()
        else:
            spacecraft.shutdown_engine(self.engine_name)


@dataclass(slots=True)
class SetGuidanceAction(MissionAction):
    spacecraft_name: str
    guidance_mode: GuidanceMode
    target_vector: Optional[object] = None

    def execute(self, context: MissionActionContext) -> None:
        spacecraft = context.spacecraft_index[self.spacecraft_name]
        spacecraft.set_guidance_mode(self.guidance_mode, self.target_vector)  # type: ignore[arg-type]


@dataclass(slots=True)
class JettisonStageAction(MissionAction):
    spacecraft_name: str
    stage_name: str

    def execute(self, context: MissionActionContext) -> None:
        spacecraft = context.spacecraft_index[self.spacecraft_name]
        spacecraft.jettison_stage(self.stage_name)


@dataclass(slots=True)
class CallbackAction(MissionAction):
    callback: Callable[[MissionActionContext], None]

    def execute(self, context: MissionActionContext) -> None:
        self.callback(context)


@dataclass(slots=True)
class ScheduledMissionEvent:
    trigger_time_seconds: float
    name: str
    action: MissionAction
    is_executed: bool = False
    notes: str = ""

    def try_execute(self, current_time_seconds: float, context: MissionActionContext) -> bool:
        if self.is_executed:
            return False
        if current_time_seconds < self.trigger_time_seconds:
            return False
        self.action.execute(context)
        self.is_executed = True
        return True


@dataclass(slots=True)
class MissionTimeline:
    events: list[ScheduledMissionEvent] = field(default_factory=list)

    def add_event(self, event: ScheduledMissionEvent) -> None:
        self.events.append(event)
        self.events.sort(key=lambda item: item.trigger_time_seconds)

    def add_timed_burn(
        self,
        *,
        spacecraft_name: str,
        engine_name: str,
        start_time_seconds: float,
        duration_seconds: float,
        throttle: float = 1.0,
        guidance_mode: GuidanceMode = GuidanceMode.PROGRADE,
        name_prefix: str = "burn",
    ) -> None:
        self.add_event(
            ScheduledMissionEvent(
                trigger_time_seconds=start_time_seconds,
                name=f"{name_prefix}_start_{engine_name}",
                action=StartBurnAction(
                    spacecraft_name=spacecraft_name,
                    engine_name=engine_name,
                    throttle=throttle,
                    guidance_mode=guidance_mode,
                ),
            )
        )
        self.add_event(
            ScheduledMissionEvent(
                trigger_time_seconds=start_time_seconds + duration_seconds,
                name=f"{name_prefix}_stop_{engine_name}",
                action=StopBurnAction(spacecraft_name=spacecraft_name, engine_name=engine_name),
            )
        )

    def reset(self) -> None:
        for event in self.events:
            event.is_executed = False

    def update(self, current_time_seconds: float, spacecraft_index: dict[str, Spacecraft]) -> list[ScheduledMissionEvent]:
        executed: list[ScheduledMissionEvent] = []
        context = MissionActionContext(spacecraft_index=spacecraft_index, timeline=self)
        for event in self.events:
            if event.try_execute(current_time_seconds, context):
                executed.append(event)
        return executed

    def pending_events(self) -> list[ScheduledMissionEvent]:
        return [event for event in self.events if not event.is_executed]

    def executed_events(self) -> list[ScheduledMissionEvent]:
        return [event for event in self.events if event.is_executed]

    def __iter__(self) -> Iterable[ScheduledMissionEvent]:
        return iter(self.events)


__all__ = [
    "CallbackAction",
    "JettisonStageAction",
    "MissionAction",
    "MissionActionContext",
    "MissionTimeline",
    "ScheduledMissionEvent",
    "SetGuidanceAction",
    "StartBurnAction",
    "StopBurnAction",
]
