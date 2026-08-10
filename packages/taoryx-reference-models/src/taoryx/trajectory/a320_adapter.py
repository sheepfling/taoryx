"""Common-family adapters for the calibrated A320 reduced-order products.

The OpenAP product supplies the executable 3DOF performance plant and the
OpenAP/JSBSim product supplies the named pseudo-6DOF response plant.  This
module only translates those existing products into the common
``ControlPlantAdapter`` seam; it does not promote either product to a
direct-wrench or physical-surface claim.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import replace
from functools import lru_cache

from ..control_allocation import (
    ControlPlantAdapter,
    EffectorEffectiveness,
    PhysicalAllocationStep,
    ProvenancedLinearization,
    finite_difference_linearization_with_provenance,
)
from ..control_automation import ControlAutomationDeclaration
from ..family_adapter import StandardFamilyAdapter, descriptor_from_control_plant
from ..generic_tuning import NativeCoordinateLqiSample
from ..local_native_coordinate_lqi import LocalNativeCoordinateLqiScreenConfig
from ..trim import TrimResult, TrimSpec
from ..tuning_campaign import TuningCampaign, run_tuning_campaign
from .a320_openap import A320OpenAPModel, A320OpenAPOperatingPoint
from .a320_pseudo6dof import A320Pseudo6DOFModel, A320Pseudo6DOFOperatingPoint

POINT_STATE_NAMES = ("altitude_m", "mach", "mass_kg", "range_m")
POINT_CONTROL_NAMES = ("throttle_ratio", "flight_path_angle_rad")
PSEUDO_STATE_NAMES = (
    "altitude_m",
    "mach",
    "mass_kg",
    "range_m",
    "alpha_rad",
    "beta_rad",
    "roll_rate_rad_s",
    "pitch_rate_rad_s",
    "yaw_rate_rad_s",
    "bank_angle_rad",
)
PSEUDO_CONTROL_NAMES = (
    "throttle_ratio",
    "flight_path_angle_rad",
    "aileron_rad",
    "elevator_rad",
    "rudder_rad",
)
DEFAULT_OPERATING_POINT = A320OpenAPOperatingPoint(11_000.0, 0.78, 60_000.0)


def _option(options: Mapping[str, float | str], name: str, default: float) -> float:
    value = options.get(name, default)
    return float(value)
    ####


def _mode(environment: Mapping[str, float | str], default: str) -> str:
    value = environment.get("thrust_mode", default)
    return str(value)
    ####


def _point_from_state(
    state: Mapping[str, float],
    controls: Mapping[str, float],
    reference: A320OpenAPOperatingPoint,
) -> A320OpenAPOperatingPoint:
    """Build an operating point while preserving the family envelope fields."""

    return A320OpenAPOperatingPoint(
        altitude_m=float(state.get("altitude_m", reference.altitude_m)),
        mach=float(state.get("mach", reference.mach)),
        mass_kg=float(state.get("mass_kg", reference.mass_kg)),
        flap_angle_deg=reference.flap_angle_deg,
        landing_gear_extended=reference.landing_gear_extended,
        vertical_speed_mps=float(controls.get("vertical_speed_mps", reference.vertical_speed_mps)),
        thrust_mode=_mode({}, reference.thrust_mode),  # type: ignore[arg-type]
        throttle_ratio=(float(controls["throttle_ratio"]) if "throttle_ratio" in controls else reference.throttle_ratio),
        temperature_delta_k=reference.temperature_delta_k,
    )
    ####


def _point_trim_spec(point: A320OpenAPOperatingPoint, controls: Mapping[str, float]) -> TrimSpec:
    return TrimSpec(
        state_names=POINT_STATE_NAMES,
        control_names=POINT_CONTROL_NAMES,
        residual_names=("altitude_rate_mps", "axial_acceleration_mps2"),
        state_initial={
            "altitude_m": point.altitude_m,
            "mach": point.mach,
            "mass_kg": point.mass_kg,
            "range_m": 0.0,
        },
        control_initial={name: float(controls[name]) for name in POINT_CONTROL_NAMES},
        operating_point={"model": "a320-openap", "altitude_m": point.altitude_m, "mach": point.mach},
    )
    ####


def _pseudo_trim_spec(point: A320OpenAPOperatingPoint, controls: Mapping[str, float], state: Mapping[str, float]) -> TrimSpec:
    return TrimSpec(
        state_names=PSEUDO_STATE_NAMES,
        control_names=PSEUDO_CONTROL_NAMES,
        residual_names=("altitude_m", "mach", "alpha_rad", "beta_rad", "roll_rate_rad_s", "pitch_rate_rad_s", "yaw_rate_rad_s"),
        state_initial={name: float(state[name]) for name in PSEUDO_STATE_NAMES},
        control_initial={name: float(controls[name]) for name in PSEUDO_CONTROL_NAMES},
        operating_point={"model": "a320-openap-jsbsim-pseudo6dof", "altitude_m": point.altitude_m, "mach": point.mach},
    )
    ####


class A320OpenAPControlPlant(ControlPlantAdapter):
    """Adapter for the pinned OpenAP point-mass product."""

    state_names = POINT_STATE_NAMES
    control_names = POINT_CONTROL_NAMES

    def __init__(self, model: A320OpenAPModel, operating_point: A320OpenAPOperatingPoint = DEFAULT_OPERATING_POINT) -> None:
        self.model = model
        self.operating_point = operating_point
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        return self.model.point_mass_derivatives(state, effectors, thrust_mode=_mode(environment, self.operating_point.thrust_mode))
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        point = _point_from_state(target, initial_guess, self.operating_point)
        solved = self.model.trim_level_flight(point)
        spec = _point_trim_spec(point, solved.controls)
        state = dict(spec.state_initial)
        return replace(solved, spec=spec, state=state)
        ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        def evaluate(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
            return self.state_derivative(state, controls, {"thrust_mode": self.operating_point.thrust_mode})

        return finite_difference_linearization_with_provenance(
            trim.spec,
            evaluate,
            trim,
            nonlinear_plant_id="taoryx.a320.openap.point_mass",
            nonlinear_plant_revision=self.model.package_sha256,
            state_step=_option(options, "state_step", 1.0e-3),
            control_step=_option(options, "control_step", 1.0e-4),
            comparison_factor=_option(options, "comparison_factor", 0.5),
            maximum_relative_difference=_option(options, "maximum_relative_difference", 0.25),
            comparison_absolute_floor=_option(options, "comparison_absolute_floor", 1.0e-8),
            state_units={"altitude_m": "m", "mach": "1", "mass_kg": "kg", "range_m": "m"},
            control_units={"throttle_ratio": "1", "flight_path_angle_rad": "rad"},
            metadata={"package_sha256": self.model.package_sha256, "claim_boundary": "point_mass_3dof"},
        )
        ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        del state, effectors
        raise NotImplementedError("OpenAP point-mass tier has no physical effector effectiveness")
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        del state, desired_wrench, previous_effectors, dt_s
        raise NotImplementedError("OpenAP point-mass tier has no physical effector allocator")
        ####

    ####


class A320Pseudo6DOFControlPlant(ControlPlantAdapter):
    """Adapter for the named OpenAP/JSBSim pseudo-6DOF response product."""

    state_names = PSEUDO_STATE_NAMES
    control_names = PSEUDO_CONTROL_NAMES

    def __init__(self, model: A320Pseudo6DOFModel, operating_point: A320OpenAPOperatingPoint = DEFAULT_OPERATING_POINT) -> None:
        self.model = model
        self.operating_point = operating_point
        ####

    def state_derivative(
        self,
        state: Mapping[str, float],
        effectors: Mapping[str, float],
        environment: Mapping[str, float | str],
    ) -> Mapping[str, float]:
        result = self.model.six_dof_derivatives(state, effectors, thrust_mode=_mode(environment, self.operating_point.thrust_mode))
        return {**result, "bank_angle_rad": float(state["roll_rate_rad_s"])}
        ####

    def trim(self, target: Mapping[str, float], initial_guess: Mapping[str, float]) -> TrimResult:
        point = _point_from_state(target, initial_guess, self.operating_point)
        solved = self.model.trim_pseudo6dof(point)
        state = {
            "altitude_m": float(target.get("altitude_m", point.altitude_m)),
            "mach": float(target.get("mach", point.mach)),
            "mass_kg": float(target.get("mass_kg", point.mass_kg)),
            "range_m": float(target.get("range_m", 0.0)),
            "alpha_rad": float(solved.state.get("alpha_rad", 0.0)),
            "beta_rad": float(target.get("beta_rad", 0.0)),
            "roll_rate_rad_s": float(target.get("roll_rate_rad_s", 0.0)),
            "pitch_rate_rad_s": float(target.get("pitch_rate_rad_s", 0.0)),
            "yaw_rate_rad_s": float(target.get("yaw_rate_rad_s", 0.0)),
            "bank_angle_rad": float(target.get("bank_angle_rad", 0.0)),
        }
        spec = _pseudo_trim_spec(point, solved.controls, state)
        return replace(solved, spec=spec, state=state)
        ####

    def linearize(self, trim: TrimResult, options: Mapping[str, float | str]) -> ProvenancedLinearization:
        def evaluate(state: Mapping[str, float], controls: Mapping[str, float]) -> Mapping[str, float]:
            return self.state_derivative(state, controls, {"thrust_mode": self.operating_point.thrust_mode})

        return finite_difference_linearization_with_provenance(
            trim.spec,
            evaluate,
            trim,
            nonlinear_plant_id="taoryx.a320.openap_jsbsim.pseudo6dof",
            nonlinear_plant_revision=self.model.package_sha256,
            state_step=_option(options, "state_step", 1.0e-4),
            control_step=_option(options, "control_step", 1.0e-4),
            comparison_factor=_option(options, "comparison_factor", 0.5),
            maximum_relative_difference=_option(options, "maximum_relative_difference", 0.25),
            comparison_absolute_floor=_option(options, "comparison_absolute_floor", 1.0e-8),
            state_units={name: "1" for name in PSEUDO_STATE_NAMES},
            control_units={name: "1" for name in PSEUDO_CONTROL_NAMES},
            metadata={"package_sha256": self.model.package_sha256, "claim_boundary": "named pseudo_6dof response law"},
        )
        ####

    def effectiveness(self, state: Mapping[str, float], effectors: Mapping[str, float]) -> EffectorEffectiveness:
        del state, effectors
        raise NotImplementedError("A320 pseudo-6DOF has no physical effector effectiveness claim")
        ####

    def allocate(
        self,
        state: Mapping[str, float],
        desired_wrench: Mapping[str, float],
        previous_effectors: Mapping[str, float],
        dt_s: float,
    ) -> PhysicalAllocationStep:
        del state, desired_wrench, previous_effectors, dt_s
        raise NotImplementedError("A320 pseudo-6DOF has no physical effector allocator")
        ####

    ####


def build_a320_pseudo_tuning_campaign() -> TuningCampaign:
    """Return the declared A320 cruise attitude-response inner-loop screen.

    This lower-tier campaign controls the named pseudo-6DOF response states
    at the default OpenAP cruise node.  It intentionally excludes altitude,
    Mach, range, and mass, which are owned by outer energy and route guidance
    loops.  It neither claims a physical A320 surface allocator nor replaces
    source-backed conventional-aircraft flight-control validation.
    """

    states = (
        "alpha_rad",
        "beta_rad",
        "roll_rate_rad_s",
        "pitch_rate_rad_s",
        "yaw_rate_rad_s",
        "bank_angle_rad",
    )
    controls = ("aileron_rad", "elevator_rad", "rudder_rad")
    return ControlAutomationDeclaration(
        id="a320-pseudo-cruise-attitude",
        campaign_id="a320-pseudo-cruise-attitude-v1",
        family_id="a320_openap_3dof",
        tier="pseudo_6dof",
        strategy_id="powered_fixed_wing.v1",
        node_id="cruise-attitude-response",
        state_scales=dict(zip(states, (0.1, 0.1, 0.5, 0.5, 0.5, 0.5), strict=True)),
        control_scales=dict(zip(controls, (0.2, 0.2, 0.2), strict=True)),
        authority_state_names=states,
        offset_free_outputs=("bank_angle_rad",),
        profile_grid_id_prefix="a320-pseudo-cruise",
    ).build_campaign()
    ####


def build_a320_pseudo_control_adapter() -> StandardFamilyAdapter:
    """Build the campaign-owned A320 pseudo-6DOF adapter without plug-in lookup.

    A local native-control screen needs the same adapter and control ordering
    as the registered common campaign.  Keeping this constructor beside the
    model data prevents a plug-in author from reimplementing trim and
    linearization plumbing merely to turn an already retained candidate into
    a Composition endpoint.
    """

    plant = A320Pseudo6DOFControlPlant(A320Pseudo6DOFModel.from_repository())
    descriptor = descriptor_from_control_plant(
        plant,
        family_id="a320_openap_3dof",
        adapter_id="taoryx.fixed_wing.openap.v1",
        physical_family="powered_fixed_wing",
        tier="pseudo_6dof",
        state_units={
            "altitude_m": "m",
            "mach": "1",
            "mass_kg": "kg",
            "range_m": "m",
            "alpha_rad": "rad",
            "beta_rad": "rad",
            "roll_rate_rad_s": "rad/s",
            "pitch_rate_rad_s": "rad/s",
            "yaw_rate_rad_s": "rad/s",
            "bank_angle_rad": "rad",
        },
        control_units={
            "throttle_ratio": "1",
            "flight_path_angle_rad": "rad",
            "aileron_rad": "rad",
            "elevator_rad": "rad",
            "rudder_rad": "rad",
        },
        evidence_status="development",
        omitted_physics=("physical surface allocation", "manufacturer-authoritative flight dynamics"),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def build_a320_local_native_coordinate_lqi_screen_config() -> LocalNativeCoordinateLqiScreenConfig:
    """Return the pinned cruise-attitude LQI screen using the common host.

    The configuration is cached because its candidate is the exact output of
    the common trim/linearize/tune campaign.  It exposes named response-law
    controls only; these values are neither inferred A320 actuators nor route
    guidance commands.
    """

    return _cached_a320_local_native_coordinate_lqi_screen_config()
    ####


@lru_cache(maxsize=1)
def _cached_a320_local_native_coordinate_lqi_screen_config() -> LocalNativeCoordinateLqiScreenConfig:
    """Build the immutable native-control screen once per installed model data set."""

    adapter = build_a320_pseudo_control_adapter()
    report = run_tuning_campaign(adapter, build_a320_pseudo_tuning_campaign())
    if report.status != "candidate_ready" or len(report.nodes) != 1:
        raise RuntimeError("A320 pseudo cruise LQI campaign did not produce its declared candidate")
    node = report.nodes[0]
    if node.trim is None or node.lqr is None or node.lqr.best is None or node.lqr.best.lqi is None:
        raise RuntimeError("A320 pseudo cruise LQI campaign has no retained safe LQI candidate")
    plant = adapter.plant
    if not isinstance(plant, A320Pseudo6DOFControlPlant):
        raise TypeError("A320 pseudo campaign adapter did not retain its pseudo-6DOF control plant")
    initial_state = dict(node.trim.state)
    initial_state.update(
        {
            "bank_angle_rad": 0.08,
            "roll_rate_rad_s": 0.02,
            "beta_rad": 0.01,
            "yaw_rate_rad_s": 0.01,
        }
    )
    candidate = node.lqr.best
    return LocalNativeCoordinateLqiScreenConfig(
        id="a320-pseudo-cruise-native-coordinate-lqi-screen-v1",
        plant_id="taoryx.a320.openap_jsbsim.pseudo6dof",
        fidelity="pseudo_6dof",
        plant=plant,
        trim=node.trim,
        candidate=candidate,
        campaign_id="a320-pseudo-cruise-attitude-v1",
        initial_state=initial_state,
        duration_s=20.0,
        dt_s=0.02,
        assessment_state_names=(
            "alpha_rad",
            "beta_rad",
            "roll_rate_rad_s",
            "pitch_rate_rad_s",
            "yaw_rate_rad_s",
            "bank_angle_rad",
        ),
        control_lower={name: -0.2 for name in candidate.control_names},
        control_upper={name: 0.2 for name in candidate.control_names},
        integral_lower={"bank_angle_rad": -0.5},
        integral_upper={"bank_angle_rad": 0.5},
        environment={"thrust_mode": "cruise"},
        status_sample_mapper=lambda sample: _a320_native_lqi_status_sample(plant, node.trim, sample),
        final_error_fraction_limit=0.01,
        maximum_control_saturation_fraction=0.0,
    )
    ####


def _a320_native_lqi_status_sample(
    plant: A320Pseudo6DOFControlPlant,
    trim: TrimResult,
    sample: NativeCoordinateLqiSample,
) -> Mapping[str, object]:
    """Map exact local model truth into the established A320 batch statuses.

    The screen has no route integrator.  ``range_m`` is retained as its local
    along-track coordinate, while east and heading use the declared zero
    local-frame reference.  Pseudo pitch remains the model's alpha response,
    matching the existing lower-tier attitude sidecar rather than claiming a
    measured rigid-body Euler solution.
    """

    state = sample.state
    controls = dict(trim.controls)
    controls.update({name: float(value) for name, value in sample.applied_controls.items()})
    point = A320OpenAPOperatingPoint(
        altitude_m=float(state["altitude_m"]),
        mach=float(state["mach"]),
        mass_kg=float(state["mass_kg"]),
        vertical_speed_mps=0.0,
        thrust_mode="cruise",
        throttle_ratio=float(controls["throttle_ratio"]),
    )
    result = plant.model.evaluate(
        A320Pseudo6DOFOperatingPoint(
            point,
            alpha_rad=float(state["alpha_rad"]),
            beta_rad=float(state["beta_rad"]),
            roll_rate_rad_s=float(state["roll_rate_rad_s"]),
            pitch_rate_rad_s=float(state["pitch_rate_rad_s"]),
            yaw_rate_rad_s=float(state["yaw_rate_rad_s"]),
            aileron_rad=float(controls["aileron_rad"]),
            elevator_rad=float(controls["elevator_rad"]),
            rudder_rad=float(controls["rudder_rad"]),
            bank_angle_rad=float(state["bank_angle_rad"]),
        )
    )
    return {
        "north_m": float(state["range_m"]),
        "east_m": 0.0,
        "altitude_m": float(state["altitude_m"]),
        "speed_m_s": result.performance.true_airspeed_mps,
        "flight_path_angle_rad": float(controls["flight_path_angle_rad"]),
        "heading_rad": 0.0,
        "dynamic_pressure_pa": result.performance.dynamic_pressure_pa,
        "attitude_euler_deg": [
            math.degrees(float(state["bank_angle_rad"])),
            math.degrees(float(state["alpha_rad"])),
            0.0,
        ],
        "body_rate_rad_s": [
            float(state["roll_rate_rad_s"]),
            float(state["pitch_rate_rad_s"]),
            float(state["yaw_rate_rad_s"]),
        ],
        "thrust_n": result.performance.thrust_n,
        "throttle_ratio": float(controls["throttle_ratio"]),
        "mass_kg": float(state["mass_kg"]),
        "fuel_flow_kg_s": result.performance.fuel_flow_at_throttle_kg_s,
        "control_realization": "response_law",
    }
    ####


def build_a320_point_tuning_campaign() -> TuningCampaign:
    """Return the OpenAP point-mass cruise-performance local LQR screen.

    The retained mass and range states stay in the design model so the common
    campaign does not hide their local coupling to throttle and Mach.  This is
    a nominal cruise performance candidate, not a route controller, wind
    rejection result, or transport flight-qualification claim.
    """

    return ControlAutomationDeclaration(
        id="a320-point-cruise-performance",
        campaign_id="a320-point-cruise-performance-lqr-v1",
        family_id="a320_openap_3dof",
        tier="point_mass_3dof",
        strategy_id="powered_fixed_wing.v1",
        node_id="cruise-performance",
        state_scales={
            "altitude_m": 1000.0,
            "mach": 0.1,
            "mass_kg": 10_000.0,
            "range_m": 100_000.0,
        },
        control_scales={"throttle_ratio": 0.2, "flight_path_angle_rad": 0.1},
        authority_state_names=POINT_STATE_NAMES,
        profile_grid_id_prefix="a320-point-cruise",
    ).build_campaign()
    ####


__all__ = [
    "A320OpenAPControlPlant",
    "A320Pseudo6DOFControlPlant",
    "DEFAULT_OPERATING_POINT",
    "POINT_CONTROL_NAMES",
    "POINT_STATE_NAMES",
    "PSEUDO_CONTROL_NAMES",
    "PSEUDO_STATE_NAMES",
    "build_a320_point_tuning_campaign",
    "build_a320_local_native_coordinate_lqi_screen_config",
    "build_a320_pseudo_control_adapter",
    "build_a320_pseudo_tuning_campaign",
]
####
