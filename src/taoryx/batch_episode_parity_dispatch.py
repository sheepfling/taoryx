"""Fail-closed dispatch for declared batch/episode parity adapters.

Parity is an exact execution-pair property.  This module centralizes the
selection of the family-owned verifier so the CLI, catalog witness gate, and
future UI do not independently guess which reduced or native runtime to use.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Protocol, cast

from .plugins import PluginCatalog, discover_plugins
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_bindings import batch_episode_parity_record


class DeclaredParityReport(Protocol):
    """Common report projection returned by family-owned parity verifiers."""

    status: str

    def as_dict(self) -> dict[str, object]:
        """Return the adapter identity and parity evidence."""

        ...


ParityVerifier = object


def verify_serialized_declared_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
    *,
    plugins: PluginCatalog | None = None,
) -> DeclaredParityReport:
    """Verify one trace through its exact registered parity adapter.

    The execution catalog must first say this exact family/mission/fidelity
    pair is ``registered``.  An unknown adapter, a merely runnable pair, or a
    mismatched report is a hard error; no neighboring family implementation is
    used as a substitute.
    """

    record = batch_episode_parity_record(composition.family_id, composition.mission, composition.fidelity)
    if record.get("availability") != "registered":
        raise ValueError(
            "no declared batch/episode parity adapter is registered for "
            f"{composition.family_id!r}/{composition.mission!r}/{composition.fidelity!r}"
        )
    adapter_id = record.get("adapter_id")
    if not isinstance(adapter_id, str):
        raise ValueError("registered batch/episode parity record lacks adapter_id")
    selected = plugins or discover_plugins()
    try:
        contribution = selected.contribution("batch_episode_parity_verifier", adapter_id)
    except KeyError:
        raise ValueError(f"no parity verifier is registered for declared adapter {adapter_id!r}")
    verifier = contribution.value
    if not callable(verifier):
        raise TypeError(
            f"plug-in {contribution.plugin.id!r} supplied a non-callable parity verifier for {adapter_id!r}"
        )
    report = cast(DeclaredParityReport, verifier(composition, payload))
    if report.as_dict().get("adapter_id") != adapter_id:
        raise ValueError("declared batch/episode parity adapter disagrees with the verifier result")
    return report
    ####


__all__ = ["DeclaredParityReport", "verify_serialized_declared_batch_episode_parity"]
