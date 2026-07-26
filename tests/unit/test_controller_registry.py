from __future__ import annotations

import pytest

from taoryx.controller_registry import default_controller_backend_registry


def test_default_registry_exposes_explicit_qualification_boundary() -> None:
    registry = default_controller_backend_registry()

    assert registry.ids == ("gain_scheduled_lqr", "legacy_pid_baseline", "lqi", "lqr", "rslqr")
    assert registry.get("lqr").qualification_eligible
    assert not registry.get("legacy_pid_baseline").qualification_eligible


def test_declared_but_unimplemented_backend_fails_closed() -> None:
    registry = default_controller_backend_registry()

    with pytest.raises(ValueError, match="has no executable factory"):
        registry.require_factory("rslqr")


def test_unknown_backend_diagnostic_lists_available_ids() -> None:
    registry = default_controller_backend_registry()

    with pytest.raises(KeyError, match="available"):
        registry.get("scenario_specific_pid")
