"""HL-20-owned source binding for declarative trim-worklist evidence."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

from taoryx_hl20.resources import model_resource_root

from .trajectory import load_daveml_trim_binding
from .trim import TrimResult, solve_trim
from .vehicle_trim_adapters import VehicleTrimEvidenceBinding
from .vehicle_trim_orchestration import TrimWorkItem

_ROOT = model_resource_root()


def trim_evidence_binding() -> VehicleTrimEvidenceBinding:
    """Publish the HL-20 source trim binding to the generic host."""

    return VehicleTrimEvidenceBinding(
        family_id="reference_hl20_mod_k",
        adapter="taoryx.adapters.reference_hl20_mod_k.daveml_pitch_channel",
        resource_root=_ROOT,
        trim_recipe_path=_ROOT / "families/reference_hl20_mod_k/qualification/trim-recipe.yaml",
        solve_work_items=solve_hl20_trim_work_items,
    )
    ####


def solve_hl20_trim_work_items(items: Sequence[TrimWorkItem]) -> tuple[tuple[str, TrimResult], ...]:
    """Solve HL-20's source-bounded pitch-trim work items from package data."""

    binding = _binding()
    return tuple((item.point_id, _solve_work_item(item, binding)) for item in items)
    ####


def _binding() -> Any:
    """Build the exact HL-20 DAVE-ML pitch-channel evaluator."""

    sidecar = _ROOT / "families/reference_hl20_mod_k/plant/daveml-import.json"
    return load_daveml_trim_binding(
        sidecar,
        role="aerodynamics",
        state_inputs={"alpha_deg": "ALP_UNLIM"},
        control_inputs={},
        residual_outputs={"pitch_cm": "CM"},
        fixed_inputs={
            "BETA": 0.0,
            "XMACH": 1.0,
            "PB": 0.0,
            "QB": 0.0,
            "RB": 0.0,
            "VRW": 100.0,
            "H_rwy": 0.0,
            "DBFUL": 0.0,
            "DBFUR": 0.0,
            "DBFLL": 0.0,
            "DBFLR": 0.0,
            "DWFL": 0.0,
            "DWFR": 0.0,
            "DRUD": 0.0,
            "DLG": 0.0,
        },
    )
    ####


def _solve_work_item(item: TrimWorkItem, binding: Any) -> TrimResult:
    """Solve one declared pitch-trim point against the source evaluator."""

    return solve_trim(
        item.trim_spec,
        binding.as_evaluator(),
        max_nfev=100,
        residual_tolerance=1.0e-10,
        acceptance_tolerance=1.0e-10,
    )
    ####


__all__ = ["solve_hl20_trim_work_items", "trim_evidence_binding"]
