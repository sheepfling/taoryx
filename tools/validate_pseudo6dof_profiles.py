"""Validate the Alpha 3 pseudo-6DOF profile catalog."""

from __future__ import annotations

import argparse
from pathlib import Path

from taoryx.trajectory.pseudo6dof_profiles import (
    Pseudo6DOFCatalog,
    build_automatic_lowering_report,
    load_pseudo6dof_catalog,
)


def _run_executable_smoke(catalog: Pseudo6DOFCatalog) -> int:
    """Exercise the reduced reachability bindings that consume the catalog."""

    from taoryx.hl20_reachability import hl20_source_release_vehicle
    from taoryx.reachability_envelope import LaunchCommand, ReachabilityFidelity, simulate_rocket_glide
    from taoryx.trajectory import HummingbirdPseudo6DOFCommand, HummingbirdPseudo6DOFModel, build_nesc_composite_pseudo6dof
    from taoryx.x15_reachability import x15_surrogate_vehicle

    cases = (
        ("x15", x15_surrogate_vehicle(), LaunchCommand(0.0, 0.6)),
        ("hl20_mod_k", hl20_source_release_vehicle(), LaunchCommand(0.0, 1.0)),
    )
    failures = 0
    for family_id, vehicle, command in cases:
        binding, profile = catalog.for_family(family_id)
        result = simulate_rocket_glide(
            vehicle,
            command,
            fidelity=ReachabilityFidelity.PSEUDO_6DOF,
            step_size_s=0.5,
            horizon_s=2.0,
        )
        observed = {str(row.get("pseudo6dof_profile_id")) for row in result.telemetry}
        passed = binding.automatic_lowering and vehicle.pseudo6dof_profile_id == profile.id and observed == {profile.id}
        print(f"  executable {family_id}: {'pass' if passed else 'fail'} ({profile.id})")
        failures += 0 if passed else 1
    _, hummingbird_profile = catalog.for_family("hummingbird")
    hummingbird = HummingbirdPseudo6DOFModel()
    _, hummingbird_rows = hummingbird.run(
        tuple(HummingbirdPseudo6DOFCommand(yaw_rad=0.2 if index >= 2 else 0.0) for index in range(4)),
        dt_s=0.02,
    )
    hummingbird_passed = bool(hummingbird_rows) and all(row.get("response_profile_id") == hummingbird_profile.id for row in hummingbird_rows)
    print(f"  executable hummingbird: {'pass' if hummingbird_passed else 'fail'} ({hummingbird_profile.id}; aggregate thrust-vector)")
    failures += 0 if hummingbird_passed else 1
    _, nesc_profile = catalog.for_family("reference_nesc_two_stage_rocket")
    nesc = build_nesc_composite_pseudo6dof()
    nesc_passed = nesc.passed and nesc.profile_id == nesc_profile.id
    print(f"  executable reference_nesc_two_stage_rocket: {'pass' if nesc_passed else 'fail'} ({nesc_profile.id}; source-translation plus response surrogate)")
    failures += 0 if nesc_passed else 1
    return failures


def main(argv: list[str] | None = None) -> int:
    """Validate the catalog and print its family coverage."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=Path("verification/pseudo6dof_profiles.yaml"))
    parser.add_argument("--smoke", action="store_true", help="run the reduced X-15 and HL-20 executable bindings")
    args = parser.parse_args(argv)
    catalog = load_pseudo6dof_catalog(args.catalog)
    for binding in catalog.bindings:
        _, profile = catalog.for_family(binding.family_id)
        print(f"{binding.family_id}: {profile.id} ({profile.model_kind}, {profile.status})")
        report = build_automatic_lowering_report(
            binding.family_id,
            "pseudo_6dof",
            catalog=catalog,
        )
        blocker = report.first_blocker or "none"
        selected = report.selected or "none"
        print(f"  lowering: selected={selected}; first_blocker={blocker}")
        if binding.direct_wrench_profile_id is not None:
            direct_report = build_automatic_lowering_report(
                binding.family_id,
                "rigid_body_6dof_direct_wrench",
                catalog=catalog,
            )
            direct_selected = direct_report.selected or "none"
            direct_blocker = direct_report.first_blocker or "none"
            print(f"  direct bridge: selected={direct_selected}; first_blocker={direct_blocker}")
        else:
            print("  direct bridge: not_applicable; passive uncontrolled rigid-body family")
    failures = _run_executable_smoke(catalog) if args.smoke else 0
    print(f"validated {len(catalog.bindings)} Alpha 3 pseudo-6DOF family bindings")
    return 0 if failures == 0 else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
