"""Semantic lowering for the declared Hummingbird pseudo-6DOF mission.

This is a family translator, not a generic multirotor route planner.  It
converts only the registered Hummingbird hover/yaw/translation/contact segment
vocabulary into bounded aggregate-thrust response objectives.  The resulting
plan is shared by preflight and the batch executor, so the runtime cannot fly
geometry different from what composition accepted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from .vehicle_composition import CompiledMissionGraph, CompiledMissionGraphNode, CompiledMissionTransition, CompiledSegment, CompiledVehicleComposition

HummingbirdSegmentKind = Literal["takeoff", "hover", "translation", "yaw", "touchdown"]


@dataclass(frozen=True, slots=True)
class HummingbirdMissionSegment:
    """One exact pseudo-6DOF controller/evaluator request."""

    instance_id: str
    segment_id: str
    kind: HummingbirdSegmentKind
    target_ned_m: tuple[float, float, float]
    target_heading_rad: float
    required_dwell_s: float
    maximum_duration_s: float
    maximum_speed_m_s: float | None = None
    maximum_sink_rate_m_s: float | None = None

    def as_dict(self) -> dict[str, object]:
        """Return the immutable runtime-lowering record."""

        return {
            "instance_id": self.instance_id,
            "segment_id": self.segment_id,
            "kind": self.kind,
            "target_ned_m": list(self.target_ned_m),
            "target_heading_rad": self.target_heading_rad,
            "required_dwell_s": self.required_dwell_s,
            "maximum_duration_s": self.maximum_duration_s,
            "maximum_speed_m_s": self.maximum_speed_m_s,
            "maximum_sink_rate_m_s": self.maximum_sink_rate_m_s,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class HummingbirdMissionPlan:
    """A resolved Hummingbird pseudo-6DOF controller/evaluator plan."""

    initialization_ned_m: tuple[float, float, float]
    initial_heading_rad: float
    initial_mass_kg: float
    segments: tuple[HummingbirdMissionSegment, ...]
    mission_graph: CompiledMissionGraph

    def manifest(self) -> dict[str, object]:
        """Return a stable semantic-to-native lowering record."""

        graph = self.mission_graph.model_dump(mode="json", by_alias=True)
        if self.mission_graph.status == "authored_graph_not_lowered":
            graph["translator_execution_status"] = "hummingbird_timeout_recovery_lowered"
            graph["claim_boundary"] = (
                "The generic compiler retains authored_graph_not_lowered because this graph adds a real branch. "
                "The Hummingbird translator accepts only timeout recovery to the declared final touchdown segment "
                "with previous_terminal_truth_state. Skipped required objectives remain failed."
            )
        else:
            graph["translator_execution_status"] = "template_success_sequence_lowered"
        return {
            "translator_id": "taoryx.hummingbird.hover_yaw_contact.pseudo6dof.v1",
            "initialization_ned_m": list(self.initialization_ned_m),
            "initial_heading_rad": self.initial_heading_rad,
            "initial_mass_kg": self.initial_mass_kg,
            "segments": [segment.as_dict() for segment in self.segments],
            "mission_graph": graph,
        }
        ####
    ####


def compile_hummingbird_pseudo_mission(composition: CompiledVehicleComposition) -> HummingbirdMissionPlan:
    """Translate the one registered Hummingbird pseudo mission or fail closed."""

    if composition.family_id != "hummingbird":
        raise ValueError(f"Hummingbird mission translator cannot lower family {composition.family_id!r}")
    if composition.fidelity != "pseudo_6dof":
        raise ValueError("Hummingbird mission translator currently supports pseudo_6dof only")
    if composition.mission != "multirotor_pad_box_yaw_recovery_land_v1":
        raise ValueError(f"Hummingbird mission translator has no lowering for {composition.mission!r}")
    initial_ned, initial_heading, initial_mass_kg = _initialization(composition)
    last_target = initial_ned
    last_heading = initial_heading
    segments: list[HummingbirdMissionSegment] = []
    for segment in composition.segments:
        resolved, last_target, last_heading = _segment(segment, last_target, last_heading)
        segments.append(resolved)
    mission_graph = _supported_mission_graph(composition)
    return HummingbirdMissionPlan(initial_ned, initial_heading, initial_mass_kg, tuple(segments), mission_graph)
    ####


def _initialization(composition: CompiledVehicleComposition) -> tuple[tuple[float, float, float], float, float]:
    values = composition.initialization.inputs
    if composition.initialization.id == "grounded_idle":
        north = _number(values, "pad_north_m")
        east = _number(values, "pad_east_m")
        altitude = 0.0
        mass_kg = _positive(values, "mass_kg", default=0.5)
    elif composition.initialization.id == "airborne_hover":
        north = _number(values, "north_m", default=0.0)
        east = _number(values, "east_m", default=0.0)
        altitude = _number(values, "altitude_m")
        mass_kg = 0.5
    else:
        raise ValueError(f"Hummingbird pseudo mission does not support {composition.initialization.id!r} initialization")
    if altitude < 0.0:
        raise ValueError("Hummingbird initialization altitude_m must be nonnegative")
    heading = math.radians(_number(values, "heading_deg"))
    return (north, east, -altitude), heading, mass_kg
    ####


def _supported_mission_graph(composition: CompiledVehicleComposition) -> CompiledMissionGraph:
    """Return the narrow graph subset this native translator can execute.

    A Hummingbird author may add a timeout recovery edge from any airborne
    segment directly to the declared final touchdown segment.  The controller
    keeps the committed terminal truth state and the independent evaluator
    still marks every skipped required objective as failed.  This proves a
    safe recovery path, never a timeout-as-success shortcut.
    """

    graph = composition.mission_graph
    if graph is None:
        raise ValueError("compiled Hummingbird composition must carry its template mission graph")
    if graph.status in {"linear_sequence_only", "authored_linear_sequence_lowered"}:
        return graph
    if graph.status != "authored_graph_not_lowered":
        raise ValueError(f"Hummingbird mission graph has unsupported status {graph.status!r}")

    segments = composition.segments
    touchdown_instance_id = segments[-1].instance_id
    if graph.entry_instance_id != segments[0].instance_id:
        raise ValueError("Hummingbird branch graph must preserve the declared first segment as its entry")
    nodes = {node.instance_id: node for node in graph.nodes}
    for index, segment in enumerate(segments):
        node = nodes.get(segment.instance_id)
        if node is None or node.segment_id != segment.id:
            raise ValueError(f"Hummingbird graph node for {segment.instance_id!r} does not match the selected segment")
        _require_success_sequence(node, segments, index)
        _require_no_transition(node.abort_transition, segment.instance_id, "abort")
        _require_no_transition(node.resource_limit_transition, segment.instance_id, "resource_limit")
        _require_no_transition(node.envelope_limit_transition, segment.instance_id, "envelope_limit")
        timeout = node.timeout_transition
        if timeout is None:
            continue
        if index + 1 == len(segments):
            raise ValueError("Hummingbird final touchdown node cannot declare a timeout recovery transition")
        if (
            timeout.target_instance_id != touchdown_instance_id
            or timeout.state_transfer != "previous_terminal_truth_state"
        ):
            raise ValueError(
                "Hummingbird timeout recovery must transfer the committed truth state directly to the declared final touchdown segment"
            )
    return graph
    ####


def _require_success_sequence(
    node: CompiledMissionGraphNode,
    segments: tuple[CompiledSegment, ...],
    index: int,
) -> None:
    """Ensure a branch author did not redefine nominal mission completion."""

    expected_target = None if index + 1 == len(segments) else segments[index + 1].instance_id
    transition = node.success_transition
    if expected_target is None:
        if transition is not None:
            raise ValueError("Hummingbird final touchdown success transition must remain terminal")
        return
    if (
        transition is None
        or transition.target_instance_id != expected_target
        or transition.state_transfer != "previous_terminal_truth_state"
    ):
        raise ValueError("Hummingbird branch graph must preserve every declared nominal success transition")
    ####


def _require_no_transition(
    transition: CompiledMissionTransition | None,
    instance_id: str,
    kind: str,
) -> None:
    """Reject branch outcomes that this first native recovery slice lacks."""

    if transition is not None:
        raise ValueError(f"Hummingbird graph node {instance_id!r} cannot yet execute a {kind!r} transition")
    ####


def _segment(
    segment: CompiledSegment,
    last_target: tuple[float, float, float],
    last_heading: float,
) -> tuple[HummingbirdMissionSegment, tuple[float, float, float], float]:
    values = segment.inputs
    if segment.id == "rotor_spool_takeoff":
        altitude = _number(values, "target_altitude_m")
        rate = _positive(values, "climb_rate_m_s")
        target = (last_target[0], last_target[1], -altitude)
        duration = abs(target[2] - last_target[2]) / rate + 12.0
        return (
            HummingbirdMissionSegment(segment.instance_id, segment.id, "takeoff", target, last_heading, 1.0, duration),
            target,
            last_heading,
        )
    if segment.id == "hover_dwell":
        target = _ned(values, "target_ned_m")
        heading = math.radians(_number(values, "target_heading_deg", default=math.degrees(last_heading)))
        dwell = _positive(values, "dwell_s")
        return (
            HummingbirdMissionSegment(segment.instance_id, segment.id, "hover", target, heading, dwell, dwell + 12.0),
            target,
            heading,
        )
    if segment.id == "waypoint_translation":
        target = _ned(values, "target_ned_m")
        speed = _positive(values, "maximum_speed_m_s")
        dwell = _positive(values, "dwell_s")
        distance = _distance_ned(last_target, target)
        return (
            HummingbirdMissionSegment(
                segment.instance_id,
                segment.id,
                "translation",
                target,
                last_heading,
                dwell,
                distance / speed + dwell + 15.0,
                maximum_speed_m_s=speed,
            ),
            target,
            last_heading,
        )
    if segment.id == "yaw_scan":
        heading = math.radians(_number(values, "target_heading_deg"))
        rate = _positive(values, "yaw_rate_limit_deg_s") * math.pi / 180.0
        duration = abs(_wrapped_angle(heading - last_heading)) / rate + 8.0
        return (
            HummingbirdMissionSegment(segment.instance_id, segment.id, "yaw", last_target, heading, 0.5, duration),
            last_target,
            heading,
        )
    if segment.id == "touchdown_settle_disarm":
        target = _ned(values, "pad_ned_m")
        sink_rate = _positive(values, "maximum_sink_rate_m_s")
        settle = _positive(values, "settle_s")
        duration = abs(target[2] - last_target[2]) / sink_rate + settle + 12.0
        return (
            HummingbirdMissionSegment(
                segment.instance_id,
                segment.id,
                "touchdown",
                target,
                last_heading,
                settle,
                duration,
                maximum_sink_rate_m_s=sink_rate,
            ),
            target,
            last_heading,
        )
    raise ValueError(f"Hummingbird pseudo mission does not lower segment {segment.id!r}")
    ####


def _number(values: object, name: str, *, default: float | None = None) -> float:
    if not isinstance(values, dict):
        raise ValueError(f"Hummingbird segment inputs must be a mapping to read {name!r}")
    selected = values.get(name)
    if selected is None:
        if default is None:
            raise ValueError(f"Hummingbird segment is missing {name!r}")
        return default
    value = float(selected.value)
    if not math.isfinite(value):
        raise ValueError(f"Hummingbird input {name!r} must be finite")
    return value
    ####


def _positive(values: object, name: str, *, default: float | None = None) -> float:
    value = _number(values, name, default=default)
    if value <= 0.0:
        raise ValueError(f"Hummingbird input {name!r} must be positive")
    return value
    ####


def _ned(values: object, name: str) -> tuple[float, float, float]:
    if not isinstance(values, dict):
        raise ValueError(f"Hummingbird segment inputs must be a mapping to read {name!r}")
    selected = values.get(name)
    if selected is None or not isinstance(selected.value, list | tuple) or len(selected.value) != 3:
        raise ValueError(f"Hummingbird input {name!r} must be an exact finite NED three-vector")
    values_float = tuple(float(value) for value in selected.value)
    if not all(math.isfinite(value) for value in values_float):
        raise ValueError(f"Hummingbird input {name!r} must be an exact finite NED three-vector")
    return values_float  # type: ignore[return-value]
    ####


def _distance_ned(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right, strict=True)))
    ####


def _wrapped_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi
    ####


__all__ = ["HummingbirdMissionPlan", "HummingbirdMissionSegment", "compile_hummingbird_pseudo_mission"]
