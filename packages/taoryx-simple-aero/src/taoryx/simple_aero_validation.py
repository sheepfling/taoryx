"""Runtime validation for the synthetic SimpleAero segment fixtures.

This validator intentionally reports fixture-level evidence only.  A segment
can parse, execute, emit finite telemetry, and expose its declared phase
without proving vehicle aero fidelity, thermal limits, or historical SimpleAero
behavior.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.problem_parser import parse_problem_text
from taoryx.runtime.runner import RunReport, run_files
from taoryx.specialized_segments import SpecializedSegmentType, specialized_segment_contract

SimpleAeroCheckStatus = Literal["pass", "fail"]
SimpleAeroFixtureStatus = Literal["pass", "fail"]

_FAMILY_CONTRACTS: dict[str, SpecializedSegmentType] = {
    "ballistic": "powered_ascent",
    "cbcr": "bank_maneuver",
    "crossrange": "bank_maneuver",
    "marv": "bank_maneuver",
    "phugoid": "alpha_profile",
    "range_extension": "alpha_profile",
    "skip": "skip_maneuver",
    "slalom": "bank_maneuver",
    "weave": "bank_maneuver",
}


@dataclass(frozen=True, slots=True)
class SimpleAeroValidationCheck:
    """One fixture-level validation gate."""

    name: str
    status: SimpleAeroCheckStatus
    message: str
    ####

    @property
    def passed(self) -> bool:
        """Return whether this gate passed."""

        return self.status == "pass"
        ####


@dataclass(frozen=True, slots=True)
class SimpleAeroSegmentValidation:
    """Fixture-level validation result for one SimpleAero segment file."""

    family: str
    problem_path: Path
    status: SimpleAeroFixtureStatus
    checks: tuple[SimpleAeroValidationCheck, ...]
    claim_boundary: str
    deferred_quality_gates: tuple[str, ...]
    runtime_report: RunReport | None = None
    ####

    @property
    def passed(self) -> bool:
        """Return whether all declared fixture gates passed."""

        return self.status == "pass"
        ####

    @property
    def physical_quality_pending(self) -> bool:
        """Return whether vehicle-quality gates remain outside this validator."""

        return bool(self.deferred_quality_gates)
        ####


def validate_simple_aero_segment_fixture(
    problem_path: str | Path,
    *,
    family: str,
    output_dir: str | Path,
    max_steps: int = 5_000,
) -> SimpleAeroSegmentValidation:
    """Validate grammar, execution, telemetry, and phase observability.

    The result is deliberately bounded to the synthetic fixture scope.  The
    returned deferred gates are the vehicle-quality gates that still require a
    real vehicle adapter and bounded aero/plant evidence.
    """

    path = Path(problem_path)
    checks: list[SimpleAeroValidationCheck] = []
    try:
        contract_name = _FAMILY_CONTRACTS[family.casefold()]
        contract = specialized_segment_contract(contract_name)
    except KeyError as error:
        return SimpleAeroSegmentValidation(
            family=family,
            problem_path=path,
            status="fail",
            checks=(SimpleAeroValidationCheck("family-contract", "fail", str(error)),),
            claim_boundary="unknown family; no validation claim",
            deferred_quality_gates=(),
        )

    source_text = path.read_text(encoding="utf-8")
    document = parse_problem_text(source_text, path=str(path), profile=GrammarProfile.TAORYX)
    grammar_errors = tuple(
        diagnostic.message for diagnostic in document.diagnostics if diagnostic.severity.value == "error"
    )
    checks.append(
        SimpleAeroValidationCheck(
            "grammar",
            "fail" if grammar_errors else "pass",
            "; ".join(grammar_errors) if grammar_errors else "TAORYX grammar accepted the fixture",
        )
    )

    report: RunReport | None = None
    if not grammar_errors:
        report = run_files(
            path,
            output_dir=output_dir,
            max_steps=max_steps,
            profile=GrammarProfile.TAORYX,
        )
    result = report.results[0] if report is not None and report.results else None
    checks.append(
        SimpleAeroValidationCheck(
            "runtime-completion",
            "pass" if result is not None and result.completed else "fail",
            "completed at a declared stop condition"
            if result is not None and result.completed
            else "did not produce a completed runtime result",
        )
    )

    history: Sequence[object] = () if result is None else result.states.get("1", ())
    named_history = tuple(getattr(state, "named", {}) for state in history)
    required_channels = ("time", "alt", "vel", "mass", "thrust")
    missing_channels = tuple(channel for channel in required_channels if not all(channel in named for named in named_history))
    checks.append(
        SimpleAeroValidationCheck(
            "required-telemetry",
            "fail" if missing_channels else "pass",
            f"missing channels: {', '.join(missing_channels)}" if missing_channels else "baseline telemetry channels are present",
        )
    )

    finite = bool(named_history) and all(
        math.isfinite(float(value))
        for named in named_history
        for value in named.values()
        if isinstance(value, (int, float))
    )
    checks.append(
        SimpleAeroValidationCheck(
            "finite-telemetry",
            "pass" if finite else "fail",
            "numeric telemetry is finite" if finite else "telemetry is empty or contains a non-finite value",
        )
    )

    times = tuple(float(getattr(state, "time")) for state in history)
    monotonic = bool(times) and all(later >= earlier for earlier, later in zip(times, times[1:]))
    checks.append(
        SimpleAeroValidationCheck(
            "monotonic-time",
            "pass" if monotonic else "fail",
            "time samples are monotonic" if monotonic else "time samples are missing or non-monotonic",
        )
    )

    segment_numbers = {int(named["_segment"]) for named in named_history if "_segment" in named}
    single_segment = segment_numbers == {1}
    checks.append(
        SimpleAeroValidationCheck(
            "single-segment-span",
            "pass" if single_segment else "fail",
            "fixture contains one isolated segment span" if single_segment else f"observed segment IDs: {sorted(segment_numbers)}",
        )
    )

    fixture_status: SimpleAeroFixtureStatus = "pass" if all(check.passed for check in checks) else "fail"
    return SimpleAeroSegmentValidation(
        family=family,
        problem_path=path,
        status=fixture_status,
        checks=tuple(checks),
        claim_boundary=contract.claim_boundary,
        deferred_quality_gates=contract.quality_gates,
        runtime_report=report,
    )
    ####


__all__ = [
    "SimpleAeroCheckStatus",
    "SimpleAeroFixtureStatus",
    "SimpleAeroSegmentValidation",
    "SimpleAeroValidationCheck",
    "validate_simple_aero_segment_fixture",
]
