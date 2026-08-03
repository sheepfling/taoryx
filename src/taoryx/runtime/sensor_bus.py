"""Accepted-truth sensor scheduling, delivery, and estimator ports."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any, Protocol

from taoryx.sensors import MeasurementPacket, TruthPoint, TruthSegment

from .sensor_clock import SensorClockSpec

if TYPE_CHECKING:
    from .common import RuntimeProblem, RuntimeState, RuntimeVehicle


MeasurementConsumer = Callable[[MeasurementPacket[Any]], None]
TruthProvider = Callable[["RuntimeState"], TruthPoint]


class SensorModelProtocol(Protocol):
    def sample(self, truth: TruthPoint) -> MeasurementPacket[Any]:
        ...


def _due(clock: SensorClockSpec, time_s: float, *, tolerance_s: float = 1.0e-9) -> bool:
    if time_s < clock.phase_s - tolerance_s:
        return False
    index = (time_s - clock.phase_s) / clock.cadence_s
    return abs(index - round(index)) <= tolerance_s / max(clock.cadence_s, 1.0e-12)


@dataclass(slots=True)
class SensorBinding:
    """One model and its clock bound to a runtime vehicle."""

    name: str
    vehicle_name: str
    clock: SensorClockSpec
    model: SensorModelProtocol
    provenance: Mapping[str, object] = field(default_factory=dict)
    truth_provider: TruthProvider | None = None
    drop_predicate: Callable[[MeasurementPacket[Any]], bool] | None = None
    checkpoint_drop_policy_id: str | None = None
    interval_start: TruthPoint | None = None
    samples_emitted: int = 0
    invalid_samples: int = 0
    dropped_samples: int = 0
    drop_attempts: int = 0
    dropped_packets: list[MeasurementPacket[Any]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("sensor binding name must not be empty")
        if not self.vehicle_name.strip():
            raise ValueError("sensor binding vehicle name must not be empty")
        if self.clock.name != self.name:
            raise ValueError("sensor binding name must match its clock")
        if not callable(getattr(self.model, "sample", None)):
            raise TypeError("sensor model must provide sample(truth)")

    def sample_point(self, truth: TruthPoint) -> MeasurementPacket[Any]:
        packet = self.model.sample(truth)
        self._record(packet)
        return packet

    def sample_interval(self, segment: TruthSegment) -> MeasurementPacket[Any]:
        sampler = getattr(self.model, "sample_segment", None)
        packet = sampler(segment) if callable(sampler) else self.model.sample(segment.end)
        self._record(packet)
        return packet

    def _record(self, packet: MeasurementPacket[Any]) -> None:
        self.samples_emitted += 1
        if not packet.valid:
            self.invalid_samples += 1

    def reset_after_transition(self, truth: TruthPoint) -> MeasurementPacket[Any] | None:
        reset = getattr(self.model, "reset", None)
        if not callable(reset):
            raise TypeError(f"sensor model {self.name!r} cannot reset after a truth discontinuity")
        reset()
        self.interval_start = truth
        return self.sample_point(truth)

    def to_metadata(self) -> dict[str, object]:
        snapshot = getattr(self.model, "snapshot", None)
        restore = getattr(self.model, "restore", None)
        return {
            "name": self.name,
            "vehicle_name": self.vehicle_name,
            "clock": self.clock.to_metadata(),
            "provenance": dict(self.provenance),
            "truth_provider": None if self.truth_provider is None else getattr(self.truth_provider, "__class__", type(self.truth_provider)).__name__,
            "drop_policy": (
                "none"
                if self.drop_predicate is None
                else self.checkpoint_drop_policy_id or "unregistered_callback"
            ),
            "checkpointing": {
                "supported": callable(snapshot) and callable(restore),
                "protocol": "snapshot-restore" if callable(snapshot) and callable(restore) else None,
            },
            "samples_emitted": self.samples_emitted,
            "invalid_samples": self.invalid_samples,
            "dropped_samples": self.dropped_samples,
            "drop_attempts": self.drop_attempts,
            "dropped_packet_times_s": [packet.sampled_at_s for packet in self.dropped_packets],
            "interval_start_s": None if self.interval_start is None else self.interval_start.time_s,
        }


@dataclass(slots=True)
class SensorBus:
    """Sensor orchestration over immutable accepted truth only."""

    bindings: list[SensorBinding] = field(default_factory=list)
    queued: dict[str, list[MeasurementPacket[Any]]] = field(default_factory=dict)
    delivered: dict[str, list[MeasurementPacket[Any]]] = field(default_factory=dict)
    consumers: dict[str, list[MeasurementConsumer]] = field(default_factory=dict)
    initialized: bool = False

    def register(self, binding: SensorBinding) -> None:
        if any(item.name == binding.name for item in self.bindings):
            raise ValueError(f"sensor binding {binding.name!r} is already registered")
        self.bindings.append(binding)
        self.queued.setdefault(binding.name, [])
        self.delivered.setdefault(binding.name, [])
        self.consumers.setdefault(binding.name, [])

    def attach(self, problem: RuntimeProblem) -> None:
        """Attach the bus and make every binding an accepted-time boundary."""

        existing = {clock.name: clock for clock in problem.sensor_clocks}
        for binding in self.bindings:
            prior = existing.get(binding.clock.name)
            if prior is not None and prior != binding.clock:
                raise ValueError(f"problem already has a different clock for sensor {binding.name!r}")
            existing[binding.clock.name] = binding.clock
        problem.sensor_clocks = tuple(existing.values())
        problem.sensor_bus = self
        problem.metadata["sensor_execution"] = "accepted-truth-measurement-bus"
        problem.metadata["sensor_bindings"] = [binding.to_metadata() for binding in self.bindings]

    def subscribe(self, sensor_name: str, consumer: MeasurementConsumer) -> None:
        if not any(item.name == sensor_name for item in self.bindings):
            raise KeyError(f"unknown sensor binding {sensor_name!r}")
        self.consumers[sensor_name].append(consumer)

    def initialize(self, problem: RuntimeProblem) -> None:
        """Publish initial boundary baselines and initialize interval windows."""

        if self.initialized:
            return
        for binding in self.bindings:
            vehicle = self._vehicle(problem, binding)
            truth = self._truth(binding, vehicle)
            if _due(binding.clock, truth.time_s):
                if binding.clock.sample_mode == "instantaneous":
                    self._queue(binding, binding.sample_point(truth))
                else:
                    binding.interval_start = truth
            elif binding.clock.sample_mode == "interval":
                binding.interval_start = None
        self.initialized = True
        self.release_available(max((vehicle.state.time for vehicle in problem.vehicles.values()), default=0.0))

    def accepted_step(
        self,
        problem: RuntimeProblem,
        previous_states: Mapping[str, RuntimeState],
    ) -> None:
        """Consume one accepted segment for each registered active vehicle."""

        if not self.initialized:
            self.initialize(problem)
        for binding in self.bindings:
            vehicle = self._vehicle(problem, binding)
            previous = previous_states.get(vehicle.name)
            if previous is None or not vehicle.active and vehicle.state.time <= previous.time:
                continue
            start_truth = self._truth(binding, vehicle, previous)
            end_truth = self._truth(binding, vehicle)
            if end_truth.time_s <= start_truth.time_s:
                continue
            if binding.clock.sample_mode == "instantaneous":
                if _due(binding.clock, end_truth.time_s):
                    self._queue(binding, binding.sample_point(end_truth))
            else:
                self._accept_interval(binding, start_truth, end_truth)
        current_time = max((vehicle.state.time for vehicle in problem.vehicles.values()), default=0.0)
        self.release_available(current_time)

    def handle_transition(
        self,
        problem: RuntimeProblem,
        vehicle: RuntimeVehicle,
        *,
        state: RuntimeState,
        state_discontinuity: bool,
    ) -> None:
        """Rebase sensor state at a committed event without exposing a jump."""

        for binding in self.bindings:
            if binding.vehicle_name != vehicle.name:
                continue
            truth = self._truth(binding, vehicle, state)
            if state_discontinuity:
                packet = binding.reset_after_transition(truth)
                if packet is not None:
                    self._queue(binding, packet)
            else:
                binding.interval_start = truth

    def release_available(self, time_s: float) -> tuple[MeasurementPacket[Any], ...]:
        """Release packets whose declared delivery time has arrived."""

        released: list[MeasurementPacket[Any]] = []
        for binding in self.bindings:
            pending = self.queued[binding.name]
            while pending and pending[0].available_at_s <= time_s + 1.0e-12:
                packet = pending.pop(0)
                self.delivered[binding.name].append(packet)
                released.append(packet)
                for consumer in self.consumers[binding.name]:
                    consumer(packet)
        return tuple(released)

    def packets(self, sensor_name: str, *, delivered: bool = True) -> tuple[MeasurementPacket[Any], ...]:
        collection = self.delivered if delivered else self.queued
        if sensor_name not in collection:
            raise KeyError(f"unknown sensor binding {sensor_name!r}")
        return tuple(collection[sensor_name])

    def to_metadata(self) -> dict[str, object]:
        return {
            "initialized": self.initialized,
            "bindings": [binding.to_metadata() for binding in self.bindings],
            "queued_counts": {name: len(items) for name, items in self.queued.items()},
            "delivered_counts": {name: len(items) for name, items in self.delivered.items()},
        }

    def _accept_interval(self, binding: SensorBinding, start: TruthPoint, end: TruthPoint) -> None:
        if binding.interval_start is None:
            if _due(binding.clock, start.time_s):
                binding.interval_start = start
            else:
                return
        if not _due(binding.clock, end.time_s):
            return
        segment = TruthSegment(binding.interval_start, end)
        self._queue(binding, binding.sample_interval(segment))
        binding.interval_start = end

    def _queue(self, binding: SensorBinding, packet: MeasurementPacket[Any]) -> None:
        binding.drop_attempts += 1
        if binding.drop_predicate is not None and binding.drop_predicate(packet):
            binding.dropped_samples += 1
            binding.dropped_packets.append(packet)
            return
        if binding.clock.delivery_s:
            packet = replace(packet, available_at_s=packet.available_at_s + binding.clock.delivery_s)
        self.queued[binding.name].append(packet)

    @staticmethod
    def _vehicle(problem: RuntimeProblem, binding: SensorBinding) -> RuntimeVehicle:
        try:
            return problem.vehicles[binding.vehicle_name]
        except KeyError as exc:
            raise KeyError(f"sensor {binding.name!r} references unknown vehicle {binding.vehicle_name!r}") from exc

    @staticmethod
    def _truth(binding: SensorBinding, vehicle: RuntimeVehicle, state: RuntimeState | None = None) -> TruthPoint:
        selected = vehicle.state if state is None else state
        provider = binding.truth_provider or vehicle.truth_provider
        if provider is None:
            raise RuntimeError(f"vehicle {vehicle.name!r} has no committed TruthPoint provider")
        truth = provider(selected)
        if not isinstance(truth, TruthPoint):
            raise TypeError(f"vehicle {vehicle.name!r} truth provider must return TruthPoint")
        return truth
