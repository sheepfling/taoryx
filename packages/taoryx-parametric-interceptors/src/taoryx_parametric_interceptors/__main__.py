"""Command-line intake and common-Composition execution for interceptor files."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

import yaml
from pydantic import BaseModel

from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
)

from .authoring import (
    build_interceptor_authoring_report,
    interceptor_authoring_schema_bundle,
    interceptor_execution_status,
    load_interceptor_authoring_profile,
    report_json,
    write_interceptor_authoring_template,
)
from .calibration import compare_assumption_cases, load_interceptor_calibration_scenario
from .fitting import (
    apply_interceptor_fit_receipt,
    fit_interceptor_profile,
    load_interceptor_fit_campaign,
)
from .profile import AssumptionCase, InterceptorEvidenceProfile, ResolvedInterceptorProfile
from .provider import (
    DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
    MISSION_TEMPLATE_ID,
    POINT_MASS_FIDELITY_ID,
    PSEUDO6_FIDELITY_ID,
    TARGET_TRACK_MISSION_TEMPLATE_ID,
    ParametricInterceptorMissionCompositionProvider,
)
from .resolver import resolve_interceptor
from .response_analysis import (
    analyze_pseudo6_response,
    analyze_pseudo6_response_at_operating_point,
    compare_pseudo6_responses,
    compare_pseudo6_responses_at_operating_point,
    load_pseudo6_operating_point_analysis_case,
    load_pseudo6_response_analysis_request,
)


def build_parser() -> argparse.ArgumentParser:
    """Build the deliberately small file-first developer interface."""

    parser = argparse.ArgumentParser(
        prog="taoryx-interceptor",
        description=("Create, inspect, and run evidence-aware interceptor profiles. Execution always uses the standard Taoryx Mission Composition runner."),
    )
    commands = parser.add_subparsers(dest="command", required=True)

    initialize = commands.add_parser("init", help="write a valid copy-ready YAML profile")
    initialize.add_argument("output", type=Path)
    initialize.add_argument("--interceptor-id", default="my-interceptor")
    initialize.add_argument("--format", choices=("flat", "catalogue"), default="flat")
    initialize.add_argument("--force", action="store_true", help="replace an existing output file")

    schema = commands.add_parser(
        "schema",
        help="emit the machine-readable flat and catalogue authoring contracts",
    )
    schema.add_argument("--format", choices=("all", "flat", "catalogue"), default="all")
    schema.add_argument("--output", type=Path)

    inspect = commands.add_parser(
        "inspect",
        help="validate and resolve a profile, then report evidence and Composition metadata",
    )
    _add_profile_arguments(inspect)
    inspect.add_argument("--output", type=Path)

    run = commands.add_parser(
        "run",
        help="resolve one profile and run it through its standard Composition provider",
    )
    _add_profile_arguments(run)
    run.add_argument(
        "--fidelity",
        choices=(POINT_MASS_FIDELITY_ID, PSEUDO6_FIDELITY_ID),
        default=POINT_MASS_FIDELITY_ID,
    )
    run.add_argument(
        "--mission-template",
        choices=(
            MISSION_TEMPLATE_ID,
            TARGET_TRACK_MISSION_TEMPLATE_ID,
            DIRECT_ACCELERATION_MISSION_TEMPLATE_ID,
        ),
        default=MISSION_TEMPLATE_ID,
        help="select fixed-waypoint, target-track, or direct local-NEU acceleration controls",
    )
    run.add_argument(
        "--set",
        action="append",
        default=[],
        metavar="PARAMETER=VALUE",
        help="override one advertised numeric mission parameter; repeat as needed",
    )
    run.add_argument("--configuration-id")
    run.add_argument("--request-id")
    run.add_argument("--output-mode", choices=("core", "selected", "all"), default="core")
    run.add_argument("--channel", action="append", default=[])
    run.add_argument("--telemetry-group", action="append", default=[])
    run.add_argument("--cadence-s", type=float)
    run.add_argument("--maximum-samples", type=int, default=200)
    run.add_argument(
        "--allow-unqualified",
        action="store_true",
        help="explicitly execute a resolver-only profile despite its unresolved diagnostics",
    )
    run.add_argument("--output", type=Path)

    calibrate = commands.add_parser(
        "calibrate",
        help="score one immutable scenario against a resolved profile",
    )
    _add_profile_arguments(calibrate)
    calibrate.add_argument("scenario", type=Path)
    calibrate.add_argument(
        "--allow-unqualified",
        action="store_true",
        help="explicitly calibrate a resolver-only profile despite unresolved diagnostics",
    )
    calibrate.add_argument("--output", type=Path)

    compare_cases = commands.add_parser(
        "compare-cases",
        help="run one immutable scenario across conservative, nominal, and optimistic resolver cases",
    )
    compare_cases.add_argument("profile", type=Path)
    compare_cases.add_argument("scenario", type=Path)
    compare_cases.add_argument(
        "--input-format",
        choices=("auto", "flat", "catalogue"),
        default="auto",
    )
    compare_cases.add_argument(
        "--case",
        action="append",
        choices=tuple(item.value for item in AssumptionCase),
        help="compare only this assumption case; repeat to select multiple cases (default: all three)",
    )
    compare_cases.add_argument(
        "--allow-unqualified",
        action="store_true",
        help="explicitly screen a resolver-only profile despite unresolved diagnostics",
    )
    compare_cases.add_argument("--output", type=Path)

    fit = commands.add_parser(
        "fit",
        help="fit bounded simulation-only scales across a typed scenario campaign",
    )
    fit.add_argument("profile", type=Path)
    fit.add_argument("campaign", type=Path)
    fit.add_argument(
        "--input-format",
        choices=("auto", "flat", "catalogue"),
        default="auto",
    )
    fit.add_argument(
        "--allow-unqualified",
        action="store_true",
        help="explicitly fit a resolver-only profile despite unresolved diagnostics",
    )
    fit.add_argument(
        "--allow-unaccepted-candidate",
        action="store_true",
        help="permit --fitted-profile-output when the fit misses its acceptance threshold",
    )
    fit.add_argument("--fitted-profile-output", type=Path)
    fit.add_argument("--output", type=Path, help="fit-receipt JSON destination; omit for stdout")

    response = commands.add_parser(
        "analyze-response",
        help="analyze one pseudo-6DOF response law from an optional YAML request",
    )
    _add_profile_arguments(response)
    response.add_argument("request", type=Path, nargs="?")
    response.add_argument("--output", type=Path)

    operating_response = commands.add_parser(
        "analyze-operating-response",
        help="derive pseudo-6DOF response support from one typed force operating point",
    )
    _add_profile_arguments(operating_response)
    operating_response.add_argument("case", type=Path)
    operating_response.add_argument("--output", type=Path)

    compare_operating_response = commands.add_parser(
        "compare-operating-response",
        help="compare pseudo-6DOF response tuning at one typed force operating point",
    )
    compare_operating_response.add_argument("baseline_profile", type=Path)
    compare_operating_response.add_argument("candidate_profile", type=Path)
    compare_operating_response.add_argument("case", type=Path)
    compare_operating_response.add_argument(
        "--input-format",
        choices=("auto", "flat", "catalogue"),
        default="auto",
    )
    compare_operating_response.add_argument(
        "--assumption-case",
        choices=tuple(item.value for item in AssumptionCase),
    )
    compare_operating_response.add_argument("--output", type=Path)

    compare_response = commands.add_parser(
        "compare-response",
        help="compare two pseudo-6DOF response laws under one identical request",
    )
    compare_response.add_argument("baseline_profile", type=Path)
    compare_response.add_argument("candidate_profile", type=Path)
    compare_response.add_argument("request", type=Path, nargs="?")
    compare_response.add_argument(
        "--input-format",
        choices=("auto", "flat", "catalogue"),
        default="auto",
    )
    compare_response.add_argument(
        "--assumption-case",
        choices=tuple(item.value for item in AssumptionCase),
    )
    compare_response.add_argument("--output", type=Path)
    return parser
    ####


def main(argv: Sequence[str] | None = None) -> int:
    """Execute one developer command with concise, non-traceback diagnostics."""

    parser = build_parser()
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "init":
            destination = write_interceptor_authoring_template(
                arguments.output,
                interceptor_id=arguments.interceptor_id,
                input_format=arguments.format,
                overwrite=arguments.force,
            )
            _emit_json(
                {
                    "schema": "taoryx.parametric-interceptor-template-result/v1",
                    "status": "created",
                    "input_format": arguments.format,
                    "output": str(destination),
                    "next": f"taoryx-interceptor inspect {destination}",
                }
            )
            return 0
        if arguments.command == "schema":
            payload = interceptor_authoring_schema_bundle(arguments.format)
            _emit_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", arguments.output)
            return 0
        if arguments.command == "inspect":
            report = build_interceptor_authoring_report(
                arguments.profile,
                input_format=arguments.input_format,
                assumption_case=arguments.assumption_case,
            )
            _emit_text(report_json(report), arguments.output)
            return 0
        if arguments.command == "run":
            return _run(arguments)
        if arguments.command == "calibrate":
            return _calibrate(arguments)
        if arguments.command == "compare-cases":
            return _compare_cases(arguments)
        if arguments.command == "fit":
            return _fit(arguments)
        if arguments.command == "analyze-response":
            return _analyze_response(arguments)
        if arguments.command == "analyze-operating-response":
            return _analyze_operating_response(arguments)
        if arguments.command == "compare-operating-response":
            return _compare_operating_response(arguments)
        if arguments.command == "compare-response":
            return _compare_response(arguments)
        raise RuntimeError(f"unhandled interceptor command {arguments.command!r}")
    except (KeyError, OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    ####


def _run(arguments: argparse.Namespace) -> int:
    profile, _ = load_interceptor_authoring_profile(
        arguments.profile,
        input_format=arguments.input_format,
    )
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        profile,
        assumption_case=arguments.assumption_case,
    )
    resolved = provider.resolved_profile(profile.model_id)
    _require_execution_acknowledgement(resolved, arguments.allow_unqualified, operation="run")
    overrides = _mission_overrides(arguments.set)
    configuration_id = arguments.configuration_id or f"{profile.model_id}-file-run"
    configuration = provider.configuration_from_mapping(
        profile.model_id,
        overrides,
        configuration_id=configuration_id,
        fidelity=arguments.fidelity,
        mission_template_id=arguments.mission_template,
    )
    prepared = provider.validate_configuration(configuration)
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=arguments.request_id or configuration_id,
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(
                mode=arguments.output_mode,
                channels=tuple(arguments.channel),
                telemetry_groups=tuple(arguments.telemetry_group),
                cadence_s=arguments.cadence_s,
                maximum_samples_per_object=arguments.maximum_samples,
            ),
        )
    )
    _emit_text(
        json.dumps(response.model_dump(mode="json", by_alias=True), indent=2, sort_keys=True) + "\n",
        arguments.output,
    )
    return 0 if response.kind == "trajectory" else 2
    ####


def _calibrate(arguments: argparse.Namespace) -> int:
    profile, _ = load_interceptor_authoring_profile(
        arguments.profile,
        input_format=arguments.input_format,
    )
    provider = ParametricInterceptorMissionCompositionProvider.from_profiles(
        profile,
        assumption_case=arguments.assumption_case,
    )
    resolved = provider.resolved_profile(profile.model_id)
    _require_execution_acknowledgement(resolved, arguments.allow_unqualified, operation="calibrate")
    scenario = load_interceptor_calibration_scenario(arguments.scenario)
    result = provider.evaluate_calibration(profile.model_id, scenario)
    _emit_model_json(result, arguments.output)
    return 0
    ####


def _compare_cases(arguments: argparse.Namespace) -> int:
    profile, _ = load_interceptor_authoring_profile(
        arguments.profile,
        input_format=arguments.input_format,
    )
    resolved = resolve_interceptor(profile)
    _require_execution_acknowledgement(
        resolved,
        arguments.allow_unqualified,
        operation="compare assumption cases",
    )
    scenario = load_interceptor_calibration_scenario(arguments.scenario)
    selected_cases = tuple(AssumptionCase(item) for item in arguments.case) if arguments.case else tuple(AssumptionCase)
    result = compare_assumption_cases(profile, scenario, cases=selected_cases)
    _emit_model_json(result, arguments.output)
    return 0
    ####


def _fit(arguments: argparse.Namespace) -> int:
    profile, _ = load_interceptor_authoring_profile(
        arguments.profile,
        input_format=arguments.input_format,
    )
    resolved = resolve_interceptor(profile)
    _require_execution_acknowledgement(resolved, arguments.allow_unqualified, operation="fit")
    campaign = load_interceptor_fit_campaign(arguments.campaign)
    receipt = fit_interceptor_profile(profile, campaign)
    if arguments.fitted_profile_output is not None:
        fitted = apply_interceptor_fit_receipt(
            profile,
            receipt,
            allow_unaccepted=arguments.allow_unaccepted_candidate,
        )
        _emit_profile_yaml(fitted, arguments.fitted_profile_output)
    _emit_model_json(receipt, arguments.output)
    return 0
    ####


def _analyze_response(arguments: argparse.Namespace) -> int:
    profile, _ = load_interceptor_authoring_profile(
        arguments.profile,
        input_format=arguments.input_format,
    )
    resolved = resolve_interceptor(profile, assumption_case=arguments.assumption_case)
    request = None if arguments.request is None else load_pseudo6_response_analysis_request(arguments.request)
    _emit_model_json(analyze_pseudo6_response(resolved, request), arguments.output)
    return 0
    ####


def _analyze_operating_response(arguments: argparse.Namespace) -> int:
    profile, _ = load_interceptor_authoring_profile(
        arguments.profile,
        input_format=arguments.input_format,
    )
    resolved = resolve_interceptor(profile, assumption_case=arguments.assumption_case)
    case = load_pseudo6_operating_point_analysis_case(arguments.case)
    _emit_model_json(
        analyze_pseudo6_response_at_operating_point(
            resolved,
            case.operating_point,
            case.analysis,
        ),
        arguments.output,
    )
    return 0
    ####


def _compare_operating_response(arguments: argparse.Namespace) -> int:
    baseline, _ = load_interceptor_authoring_profile(
        arguments.baseline_profile,
        input_format=arguments.input_format,
    )
    candidate, _ = load_interceptor_authoring_profile(
        arguments.candidate_profile,
        input_format=arguments.input_format,
    )
    case = load_pseudo6_operating_point_analysis_case(arguments.case)
    baseline_resolved = resolve_interceptor(
        baseline,
        assumption_case=arguments.assumption_case,
    )
    candidate_resolved = resolve_interceptor(
        candidate,
        assumption_case=arguments.assumption_case,
    )
    _emit_model_json(
        compare_pseudo6_responses_at_operating_point(
            baseline_resolved,
            candidate_resolved,
            case.operating_point,
            case.analysis,
        ),
        arguments.output,
    )
    return 0
    ####


def _compare_response(arguments: argparse.Namespace) -> int:
    baseline, _ = load_interceptor_authoring_profile(
        arguments.baseline_profile,
        input_format=arguments.input_format,
    )
    candidate, _ = load_interceptor_authoring_profile(
        arguments.candidate_profile,
        input_format=arguments.input_format,
    )
    request = None if arguments.request is None else load_pseudo6_response_analysis_request(arguments.request)
    baseline_resolved = resolve_interceptor(baseline, assumption_case=arguments.assumption_case)
    candidate_resolved = resolve_interceptor(candidate, assumption_case=arguments.assumption_case)
    _emit_model_json(
        compare_pseudo6_responses(baseline_resolved, candidate_resolved, request),
        arguments.output,
    )
    return 0
    ####


def _add_profile_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("profile", type=Path)
    parser.add_argument(
        "--input-format",
        choices=("auto", "flat", "catalogue"),
        default="auto",
    )
    parser.add_argument(
        "--assumption-case",
        choices=tuple(item.value for item in AssumptionCase),
    )
    ####


def _mission_overrides(raw_values: Sequence[str]) -> dict[str, float | str]:
    values: dict[str, float | str] = {}
    for raw in raw_values:
        identifier, separator, text = raw.partition("=")
        if not separator or not identifier or not text:
            raise ValueError(f"mission override {raw!r} must use PARAMETER=VALUE")
        if identifier in values:
            raise ValueError(f"mission override {identifier!r} was supplied more than once")
        try:
            values[identifier] = float(text)
        except ValueError:
            values[identifier] = text
    return values
    ####


def _require_execution_acknowledgement(
    resolved: ResolvedInterceptorProfile,
    allow_unqualified: bool,
    *,
    operation: str,
) -> None:
    if interceptor_execution_status(resolved) == "resolver_only" and not allow_unqualified:
        diagnostics = ", ".join(resolved.required_diagnostics)
        raise ValueError(
            f"profile {resolved.model_id!r} is resolver-only because required diagnostics remain: {diagnostics}; "
            f"review the authoring report or pass --allow-unqualified to acknowledge an archetype-dominant {operation}"
        )
    ####


def _emit_json(payload: object) -> None:
    print(json.dumps(payload, indent=2, sort_keys=True))
    ####


def _emit_text(text: str, output: Path | None = None) -> None:
    if output is None:
        print(text, end="")
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")
    print(f"wrote {output}")
    ####


def _emit_model_json(model: BaseModel, output: Path | None) -> None:
    payload = model.model_dump(mode="json", by_alias=True)
    _emit_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", output)
    ####


def _emit_profile_yaml(profile: InterceptorEvidenceProfile, output: Path) -> None:
    payload = profile.model_dump(mode="json", exclude_none=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    print(f"wrote {output}")
    ####


if __name__ == "__main__":
    raise SystemExit(main())
