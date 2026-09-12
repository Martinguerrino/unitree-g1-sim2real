"""Small contracts for future components; no simulator or policy dependencies."""
from dataclasses import dataclass
from typing import Mapping, Protocol, Sequence


@dataclass(frozen=True)
class RobotState:
    joint_names: tuple[str, ...]
    q: tuple[float, ...]  # radians
    dq: tuple[float, ...]  # radians/second
    tick: int  # source counter, NOT a synchronized latency timestamp
    received_monotonic_s: float


@dataclass(frozen=True)
class MotorCommand:
    """Future impedance command in SDK motor order; not executable in this MVP."""
    joint_names: tuple[str, ...]
    q: tuple[float, ...]
    dq: tuple[float, ...]
    tau: tuple[float, ...]
    kp: tuple[float, ...]
    kd: tuple[float, ...]


class RobotInterface(Protocol):
    def read_state(self, timeout_s: float) -> RobotState: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class SkillStep:
    skill: str
    target: str


class Perception(Protocol):
    def observe(self) -> Mapping[str, object]: ...


class TaskPlanner(Protocol):
    def plan(self, instruction: str, world: Mapping[str, object]) -> Sequence[SkillStep]: ...


class SkillPlanner(Protocol):
    def expand(self, step: SkillStep, world: Mapping[str, object]) -> Sequence[SkillStep]: ...


class NavigationPlanner(Protocol):
    def plan(self, start: Sequence[float], goal: Sequence[float], world: Mapping[str, object]) -> Sequence[Sequence[float]]: ...


class ManipulationPlanner(Protocol):
    def plan(self, state: RobotState, target: Mapping[str, object]) -> Sequence[MotorCommand]: ...


class LocomotionPolicy(Protocol):
    def act(self, state: RobotState, velocity: tuple[float, float, float]) -> MotorCommand: ...


class ManipulationPolicy(Protocol):
    def act(self, state: RobotState, observation: Mapping[str, object]) -> MotorCommand: ...
