from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.trajectory import ProviderRegistry, ReferencePointMassProvider, load_case_intent, load_family_catalog, resolve_case
from taoryx.trajectory.providers import ControlFrame
from taoryx.trajectory.taoryx_adapter import TaoryxPointMassAdapter

ROOT = Path(__file__).resolve().parents[2]
CATALOG = load_family_catalog(ROOT / "verification" / "alpha2_family_catalog.yaml")
CASE = resolve_case(
    load_case_intent(ROOT / "tests" / "fixtures" / "alpha2_case_contracts" / "case-heavy.yaml"),
    CATALOG,
)


def test_provider_registry_discovers_reference_and_taoryx_capabilities() -> None:
    registry = ProviderRegistry((ReferencePointMassProvider(), TaoryxPointMassAdapter()))

    assert [item.provider_id for item in registry.capabilities()] == ["reference.point_mass", "taoryx.native"]
    assert registry.provider("taoryx.native").capabilities.supports_interactive
    assert registry.provider("reference.point_mass").capabilities.provider_kind == "analytical"


def test_translation_reports_are_complete_and_explicit() -> None:
    reference = ReferencePointMassProvider().compile(CASE)
    native = TaoryxPointMassAdapter().compile(CASE)

    assert reference.translation.provider_id == "reference.point_mass"
    assert native.translation.provider_id == "taoryx.native"
    assert {entry.status for entry in native.translation.entries} >= {"native", "approximated", "unsupported"}
    assert any(entry.requested == "rigid_body_moments" for entry in native.translation.entries)


def test_reference_and_taoryx_step_transitions_have_common_physical_state() -> None:
    reference_provider = ReferencePointMassProvider()
    native_provider = TaoryxPointMassAdapter()
    reference = reference_provider.new_session(reference_provider.compile(CASE))
    native = native_provider.new_session(native_provider.compile(CASE))

    reference_step = reference.step(0.1, ControlFrame({"command.throttle": 0.5, "command.bank": 0.0}))
    native_step = native.step(0.1, ControlFrame({"command.throttle": 0.5, "command.bank": 0.0}))

    assert native_step.time_end_s == pytest.approx(reference_step.time_end_s)
    assert native_step.state.values == pytest.approx(reference_step.state.values)
    assert native_step.applied_controls == reference_step.applied_controls


def test_batch_execution_is_repeated_public_step_transition() -> None:
    controls = tuple(ControlFrame({"command.throttle": 0.25, "command.bank": 0.0}) for _ in range(20))
    for provider in (ReferencePointMassProvider(), TaoryxPointMassAdapter()):
        compiled = provider.compile(CASE)
        batch = provider.new_session(compiled).run_to_completion(controls)
        stepped_session = provider.new_session(compiled)
        stepped = [stepped_session.reset()]
        for frame in controls:
            stepped.append(stepped_session.step(0.05, frame).state)
        assert len(batch.samples) == len(stepped)
        for expected, actual in zip(batch.samples, stepped, strict=True):
            assert actual.time_s == pytest.approx(expected.time_s)
            assert actual.values.keys() == expected.values.keys()
            for key in expected.values:
                assert actual.values[key] == pytest.approx(expected.values[key])
