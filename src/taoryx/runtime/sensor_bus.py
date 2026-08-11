"""Accepted-truth sensor scheduling, delivery, and estimator ports."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from copy import deepcopy
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Any, Protocol, cast

import numpy as np

from taoryx.sensor_api import (
    MeasurementPacket,
    SensorContext,
    SensorContextSegment,
    SensorSampleRequest,
    TruthPoint,
    TruthSegment,
)

from .sensor_clock import SensorClockSpec

if TYPE_CHECKING:
    from .common import RuntimeProblem, RuntimeState, RuntimeVehicle


MeasurementConsumer = Callable[[MeasurementPacket[Any]], None]
TruthProvider = Callable[["RuntimeState"], TruthPoint]
SensorContextProvider = Callable[["RuntimeState", TruthPoint], SensorContext]


class SensorModelProtocol(Protocol):
    def sample(self, truth: TruthPoint) -> MeasurementPacket[Any]:
        ...


class ContextSensorModelProtocol(Protocol):
    def sample_request(self, request: SensorSampleRequest) -> MeasurementPacket[Any]:
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
    model: object
    provenance: Mapping[str, object] = field(default_factory=dict)
    truth_provider: TruthProvider | None = None
    context_provider: SensorContextProvider | None = None
    rng_seed: int = 0
    drop_predicate: Callable[[MeasurementPacket[Any]], bool] | None = None
    checkpoint_drop_policy_id: str | None = None
    interval_start: TruthPoint | None = None
    interval_start_context: SensorContext | None = None
    samples_emitted: int = 0
    invalid_samples: int = 0
    dropped_samples: int = 0
    drop_attempts: int = 0
    dropped_packets: list[MeasurementPacket[Any]] = field(default_factory=list)
    next_sequence: int = 0
    _rng: np.random.Generator = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("sensor binding name must not be empty")
        if not self.vehicle_name.strip():
            raise ValueError("sensor binding vehicle name must not be empty")
        if self.clock.name != self.name:
            raise ValueError("sensor binding name must match its clock")
        if not callable(getattr(self.model, "sample", None)) and not callable(
            getattr(self.model, "sample_request", None)
        ):
            raise TypeError("sensor model must provide sample(truth) or sample_request(request)")
        if isinstance(self.rng_seed, bool) or not isinstance(self.rng_seed, int):
            raise TypeError("sensor binding rng_seed must be an integer")
        self._rng = np.random.default_rng(self.rng_seed)

    def sample_point(
        self,
        truth: TruthPoint,
        context: SensorContext | None = None,
    ) -> MeasurementPacket[Any]:
        context_sampler = getattr(self.model, "sample_request", None)
        if callable(context_sampler):
            selected_context = context or SensorContext.from_host(truth)
            packet = context_sampler(SensorSampleRequest(point=selected_context, rng=self._rng))
        else:
            sampler = getattr(self.model, "sample", None)
            if not callable(sampler):
                raise TypeError(f"sensor model {self.name!r} cannot sample a truth point")
            packet = sampler(truth)
        return self._record(packet)
        ####

    def sample_interval(
        self,
        segment: TruthSegment,
        context_segment: SensorContextSegment | None = None,
    ) -> MeasurementPacket[Any]:
        context_sampler = getattr(self.model, "sample_request", None)
        if callable(context_sampler):
            selected_segment = context_segment or SensorContextSegment(
                SensorContext.from_host(segment.start),
                SensorContext.from_host(segment.end),
            )
            packet = context_sampler(SensorSampleRequest(segment=selected_segment, rng=self._rng))
        else:
            sampler = getattr(self.model, "sample_segment", None)
            point_sampler = getattr(self.model, "sample", None)
            if callable(sampler):
                packet = sampler(segment)
            elif callable(point_sampler):
                packet = point_sampler(segment.end)
            else:
                raise TypeError(f"sensor model {self.name!r} cannot sample a truth interval")
        return self._record(packet)
        ####

    def _record(self, packet: MeasurementPacket[Any]) -> MeasurementPacket[Any]:
        if not isinstance(packet, MeasurementPacket):
            raise TypeError(f"sensor model {self.name!r} must return MeasurementPacket")
        if packet.sensor_id is not None and packet.sensor_id != self.name:
            raise ValueError(
                f"sensor model emitted sensor_id {packet.sensor_id!r} for binding {self.name!r}"
            )
        packet = replace(packet, sensor_id=self.name, sequence=self.next_sequence)
        self.next_sequence += 1
        self.samples_emitted += 1
        if not packet.valid:
            self.invalid_samples += 1
        return packet
        ####

    def reset_after_transition(
        self,
        truth: TruthPoint,
        context: SensorContext | None = None,
    ) -> MeasurementPacket[Any] | None:
        reset = getattr(self.model, "reset", None)
        if not callable(reset):
            raise TypeError(f"sensor model {self.name!r} cannot reset after a truth discontinuity")
        reset()
        self.interval_start = truth
        self.interval_start_context = context
        return self.sample_point(truth, context)
        ####

    def rng_state(self) -> dict[str, object]:
        """Return the runtime-owned random stream state for checkpointing."""

        return cast(dict[str, object], deepcopy(self._rng.bit_generator.state))
        ####

    def restore_rng_state(self, value: Mapping[str, object]) -> None:
        """Restore a state produced by :meth:`rng_state`."""

        self._rng.bit_generator.state = deepcopy(dict(value))
        ####

    def to_metadata(self) -> dict[str, object]:
        snapshot = getattr(self.model, "snapshot", None)
        restore = getattr(self.model, "restore", None)
        return {
            "name": self.name,
            "vehicle_name": self.vehicle_name,
            "clock": self.clock.to_metadata(),
            "provenance": dict(self.provenance),
            "truth_provider": None if self.truth_provider is None else getattr(self.truth_provider, "__class__", type(self.truth_provider)).__name__,
            "context_provider": None
            if self.context_provider is None
            else getattr(self.context_provider, "__class__", type(self.context_provider)).__name__,
            "rng_seed": self.rng_seed,
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
            "next_sequence": self.next_sequence,
        }
        ####
    ####


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
                context = self._context(binding, vehicle, truth)
                if binding.clock.sample_mode == "instantaneous":
                    self._queue(binding, binding.sample_point(truth, context))
                else:
                    binding.interval_start = truth
                    binding.interval_start_context = context
            elif binding.clock.sample_mode == "interval":
                binding.interval_start = None
                binding.interval_start_context = None
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
                    end_context = self._context(binding, vehicle, end_truth)
                    self._queue(binding, binding.sample_point(end_truth, end_context))
            else:
                start_context = self._context(binding, vehicle, start_truth, previous)
                end_context = self._context(binding, vehicle, end_truth)
                self._accept_interval(binding, start_truth, end_truth, start_context, end_context)
        current_time = max((vehicle.state.time for vehicle in problem.vehicles.values()), default=0.0)
        self.release_available(current_time)

    def accepted_context(
        self,
        sensor_name: str,
        truth: TruthPoint,
        context: SensorContext | None = None,
        *,
        previous_truth: TruthPoint | None = None,
        previous_context: SensorContext | None = None,
    ) -> tuple[MeasurementPacket[Any], ...]:
        """Publish one externally owned committed-truth boundary.

        Source-compatible runtimes sometimes own their integration loop but
        still need the standard Taoryx scheduling, delivery-latency, packet
        sequencing, and consumer semantics.  This method admits exactly that
        boundary without fabricating a :class:`RuntimeProblem` or treating
        CADAC local coordinates as an Earth/environment runtime.

        ``previous_truth`` is required for an interval sensor after its first
        boundary.  Instantaneous sensors sample only when their declared
        clock is due.  Returned packets are those that became deliverable at
        this accepted time; all packets remain available through
        :meth:`packets` as usual.
        """

        binding = self._binding(sensor_name)
        if context is not None and not np.isclose(context.host.time_s, truth.time_s, atol=1.0e-12):
            raise ValueError(f"sensor {sensor_name!r} context timestamp does not match committed truth")
        if previous_context is not None and previous_truth is None:
            raise ValueError("previous_context requires previous_truth")
        if previous_truth is not None and previous_truth.time_s >= truth.time_s:
            raise ValueError("previous_truth must precede committed truth")
        if previous_context is not None and previous_truth is not None and not np.isclose(
            previous_context.host.time_s,
            previous_truth.time_s,
            atol=1.0e-12,
        ):
            raise ValueError(f"sensor {sensor_name!r} previous context timestamp does not match previous truth")

        selected_context = context or SensorContext.from_host(truth)
        if binding.clock.sample_mode == "instantaneous":
            if _due(binding.clock, truth.time_s):
                self._queue(binding, binding.sample_point(truth, selected_context))
        elif previous_truth is None:
            if _due(binding.clock, truth.time_s):
                binding.interval_start = truth
                binding.interval_start_context = selected_context
        else:
            selected_previous_context = previous_context or SensorContext.from_host(previous_truth)
            self._accept_interval(
                binding,
                previous_truth,
                truth,
                selected_previous_context,
                selected_context,
            )
        self.initialized = True
        return self.release_available(truth.time_s)

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
            context = self._context(binding, vehicle, truth, state)
            if state_discontinuity:
                packet = binding.reset_after_transition(truth, context)
                if packet is not None:
                    self._queue(binding, packet)
            else:
                binding.interval_start = truth
                binding.interval_start_context = context

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

    def _accept_interval(
        self,
        binding: SensorBinding,
        start: TruthPoint,
        end: TruthPoint,
        start_context: SensorContext,
        end_context: SensorContext,
    ) -> None:
        if binding.interval_start is None:
            if _due(binding.clock, start.time_s):
                binding.interval_start = start
                binding.interval_start_context = start_context
            else:
                return
        if not _due(binding.clock, end.time_s):
            return
        segment = TruthSegment(binding.interval_start, end)
        context_segment = SensorContextSegment(
            binding.interval_start_context or SensorContext.from_host(binding.interval_start),
            end_context,
        )
        self._queue(binding, binding.sample_interval(segment, context_segment))
        binding.interval_start = end
        binding.interval_start_context = end_context

    def _queue(self, binding: SensorBinding, packet: MeasurementPacket[Any]) -> None:
        binding.drop_attempts += 1
        if binding.drop_predicate is not None and binding.drop_predicate(packet):
            binding.dropped_samples += 1
            binding.dropped_packets.append(packet)
            return
        if binding.clock.delivery_s:
            packet = replace(packet, available_at_s=packet.available_at_s + binding.clock.delivery_s)
        self.queued[binding.name].append(packet)

    def _binding(self, sensor_name: str) -> SensorBinding:
        for binding in self.bindings:
            if binding.name == sensor_name:
                return binding
        raise KeyError(f"unknown sensor binding {sensor_name!r}")

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

    @staticmethod
    def _context(
        binding: SensorBinding,
        vehicle: RuntimeVehicle,
        truth: TruthPoint,
        state: RuntimeState | None = None,
    ) -> SensorContext:
        selected = vehicle.state if state is None else state
        if binding.context_provider is None:
            return SensorContext.from_host(
                truth,
                snapshot_id=f"{vehicle.name}@{truth.time_s:.17g}",
            )
        context = binding.context_provider(selected, truth)
        if not isinstance(context, SensorContext):
            raise TypeError(f"sensor {binding.name!r} context provider must return SensorContext")
        if not np.isclose(context.host.time_s, truth.time_s, atol=1.0e-12):
            raise ValueError(f"sensor {binding.name!r} context timestamp does not match committed host truth")
        return context
        ####
    ####
