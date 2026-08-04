"""Fail-closed dispatch for declared batch/episode parity adapters.

Parity is an exact execution-pair property.  This module centralizes the
selection of the family-owned verifier so the CLI, catalog witness gate, and
future UI do not independently guess which reduced or native runtime to use.
"""

from __future__ import annotations

from collections.abc import Mapping

from .composition_batch_episode_parity import (
    BatchEpisodeParityReport,
    verify_serialized_composition_batch_episode_parity,
)
from .language_backed_batch_episode_parity import (
    LanguageBackedBatchEpisodeParityReport,
    verify_serialized_language_backed_batch_episode_parity,
)
from .local_direct_wrench_batch_episode_parity import (
    LocalDirectWrenchBatchEpisodeParityReport,
    verify_serialized_local_direct_wrench_batch_episode_parity,
)
from .reduced_fixed_wing_batch_episode_parity import (
    ReducedFixedWingBatchEpisodeParityReport,
    verify_serialized_reduced_fixed_wing_batch_episode_parity,
)
from .vehicle_composition import CompiledVehicleComposition
from .vehicle_execution_bindings import batch_episode_parity_record

DeclaredParityReport = BatchEpisodeParityReport | LanguageBackedBatchEpisodeParityReport | LocalDirectWrenchBatchEpisodeParityReport | ReducedFixedWingBatchEpisodeParityReport

_HUMMINGBIRD_ADAPTER = "taoryx.hummingbird.aggregate_thrust_batch_episode_parity.v1"
_LANGUAGE_BACKED_ADAPTER = "taoryx.language_backed.action_trace_batch_episode_parity.v1"
_LOCAL_DIRECT_WRENCH_ADAPTERS = {
    "taoryx.x15.local_direct_wrench_batch_episode_parity.v1",
    "taoryx.local_direct_wrench_batch_episode_parity.v1",
}
_REDUCED_FIXED_WING_ADAPTERS = {
    "taoryx.reduced_fixed_wing.a320_action_trace_batch_episode_parity.v1",
    "taoryx.reduced_fixed_wing.f16_action_trace_batch_episode_parity.v1",
}


def verify_serialized_declared_batch_episode_parity(
    composition: CompiledVehicleComposition,
    payload: Mapping[str, object],
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
    report: DeclaredParityReport
    if adapter_id == _HUMMINGBIRD_ADAPTER:
        report = verify_serialized_composition_batch_episode_parity(composition, payload)
    elif adapter_id == _LANGUAGE_BACKED_ADAPTER:
        report = verify_serialized_language_backed_batch_episode_parity(composition, payload)
    elif adapter_id in _LOCAL_DIRECT_WRENCH_ADAPTERS:
        report = verify_serialized_local_direct_wrench_batch_episode_parity(composition, payload)
    elif adapter_id in _REDUCED_FIXED_WING_ADAPTERS:
        report = verify_serialized_reduced_fixed_wing_batch_episode_parity(composition, payload)
    else:
        raise ValueError(f"no parity verifier is registered for declared adapter {adapter_id!r}")
    if report.as_dict().get("adapter_id") != adapter_id:
        raise ValueError("declared batch/episode parity adapter disagrees with the verifier result")
    return report
    ####


__all__ = ["DeclaredParityReport", "verify_serialized_declared_batch_episode_parity"]
