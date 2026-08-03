"""Build showcase realizations from the common family-adapter façade.

The adapter registry answers whether a family operation is callable.  This
module makes that same descriptor and operation report the source of truth
for the showcase realization, so a renderer cannot accidentally advertise a
different fidelity, control path, or state schema.
"""

from __future__ import annotations

from collections.abc import Mapping

from taoryx.family_adapter import AdapterOperation, StandardFamilyAdapter

from .contracts import EvidenceGrade, FidelityShowcaseRealization


def build_showcase_realization_from_adapter(
    adapter: StandardFamilyAdapter,
    *,
    claim: str,
    evidence_grade: EvidenceGrade,
    realization_id: str | None = None,
    semantic_command_mapping: Mapping[str, str] | None = None,
    available_physics: tuple[str, ...] = (),
    nonclaims: tuple[str, ...] = (),
    required_operations: tuple[AdapterOperation, ...] = (),
) -> FidelityShowcaseRealization:
    """Construct one showcase realization from an executable family adapter.

    ``required_operations`` is a preflight gate for the specific showcase,
    not a declaration that every adapter operation is needed by every mission.
    Physical effector names are copied only for the surface-allocation tier;
    direct-wrench channels therefore cannot be mistaken for real effectors in
    the resulting artifact.
    """

    descriptor = adapter.describe()
    capabilities = adapter.capability_report()
    unavailable = tuple(
        f"{operation} ({capabilities.capability(operation).status})"
        for operation in required_operations
        if not capabilities.capability(operation).usable
    )
    if unavailable:
        raise ValueError(
            f"{descriptor.family_id}: showcase realization requires unavailable adapter operations: "
            + ", ".join(unavailable)
        )

    control_realization = descriptor.control_realization
    physical_effectors = (
        tuple(channel.name for channel in descriptor.control_channels)
        if control_realization == "surface_allocated"
        else ()
    )
    return FidelityShowcaseRealization(
        fidelity=descriptor.tier,
        control_realization=control_realization,
        realization_id=realization_id or f"{descriptor.adapter_id}.{descriptor.tier}",
        state_schema=tuple(channel.name for channel in descriptor.state_channels),
        semantic_command_mapping=dict(semantic_command_mapping or {}),
        physical_effectors=physical_effectors,
        available_physics=available_physics,
        claim=claim,
        nonclaims=nonclaims,
        evidence_grade=evidence_grade,
    )
    ####


__all__ = ["build_showcase_realization_from_adapter"]
