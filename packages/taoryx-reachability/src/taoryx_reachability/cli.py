"""Command implementation for the optional reachability workbench."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import cast

from taoryx.reachability_catalog import ReachabilityCatalog, load_reachability_catalog
from taoryx.reachability_envelope import TerminalCriteria

from .resources import reachability_resource_root


def _criteria_from_arguments(arguments: argparse.Namespace) -> TerminalCriteria | None:
    names = (
        "min_speed_m_s",
        "max_speed_m_s",
        "target_x_m",
        "target_y_m",
        "target_z_m",
        "max_impact_radius_m",
        "min_impact_speed_m_s",
        "max_impact_speed_m_s",
    )
    if not arguments.require_ground_contact and not any(getattr(arguments, name) is not None for name in names):
        return None
    return TerminalCriteria(
        min_speed_m_s=0.0 if arguments.min_speed_m_s is None else arguments.min_speed_m_s,
        max_speed_m_s=math.inf if arguments.max_speed_m_s is None else arguments.max_speed_m_s,
        require_ground_contact=arguments.require_ground_contact,
        target_position_m=(
            0.0 if arguments.target_x_m is None else arguments.target_x_m,
            0.0 if arguments.target_y_m is None else arguments.target_y_m,
            0.0 if arguments.target_z_m is None else arguments.target_z_m,
        ),
        max_impact_radius_m=arguments.max_impact_radius_m,
        min_impact_speed_m_s=arguments.min_impact_speed_m_s,
        max_impact_speed_m_s=arguments.max_impact_speed_m_s,
    )
    ####


def _catalog(path: Path | None) -> ReachabilityCatalog:
    source = path or reachability_resource_root() / "verification" / "reachability_profile_catalog.yaml"
    return load_reachability_catalog(source)
    ####


def _print_json(payload: object, output: Path | None = None) -> None:
    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if output is None:
        print(text, end="")
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")
    print(f"wrote {output}")
    ####


def handle_reachability_command(parsed_arguments: object) -> int:
    """Execute one core-parsed reachability subcommand."""

    from taoryx.reachability_envelope import (
        ReachabilityFidelity,
        RocketGlideVehicle,
        generate_launch_grid,
        rerun_timed_out_artifact,
        run_reachability_envelope,
    )
    from taoryx.reachability_visualization import load_reachability_artifact, render_reachability_plot_bundle
    from taoryx.x15_native_replay import write_x15_native_boundary_replay
    from taoryx.x15_reachability import write_x15_reachability_bundle

    arguments = cast(argparse.Namespace, parsed_arguments)
    try:
        if arguments.reachability_command == "x15":
            reachability_bundle = write_x15_reachability_bundle(
                arguments.output_dir,
                workers=arguments.workers,
                step_size_s=arguments.step_size_s,
                horizon_s=arguments.horizon_s,
                criteria=_criteria_from_arguments(arguments),
                dpi=arguments.dpi,
            )
            print(f"wrote X-15 reachability bundle: {reachability_bundle.manifest_path.parent}")
            return 0
        if arguments.reachability_command == "x15-native-replay":
            native_bundle = write_x15_native_boundary_replay(
                arguments.envelope,
                arguments.output_dir,
                max_points=arguments.max_points,
                duration_s=arguments.duration_s,
                max_steps=arguments.max_steps,
            )
            print(f"wrote X-15 native replay bundle: {native_bundle.manifest_path.parent}")
            return 0
        if arguments.reachability_command == "run":
            azimuths = arguments.azimuth_deg or (-30.0, 0.0, 30.0)
            elevations = arguments.elevation_deg or (35.0, 50.0, 65.0)
            banks = arguments.bank_deg or (-30.0, 0.0, 30.0)
            commands = generate_launch_grid(
                tuple(math.radians(value) for value in azimuths),
                tuple(math.radians(value) for value in elevations),
                tuple(math.radians(value) for value in banks),
            )
            result = run_reachability_envelope(
                RocketGlideVehicle(),
                commands,
                fidelity=ReachabilityFidelity(arguments.fidelity),
                step_size_s=arguments.step_size_s,
                horizon_s=arguments.horizon_s,
                workers=arguments.workers,
                criteria=_criteria_from_arguments(arguments),
            )
            _print_json(
                result.as_dict(
                    include_trajectories=arguments.include_trajectories or not arguments.omit_trajectories
                ),
                arguments.output,
            )
            return 0
        if arguments.reachability_command == "rerun-timeouts":
            payload = load_reachability_artifact(arguments.envelope)
            result = rerun_timed_out_artifact(
                payload,
                horizon_s=arguments.horizon_s,
                step_size_s=arguments.step_size_s,
                workers=arguments.workers,
            )
            result.write_json(arguments.output)
            print(f"wrote timeout rerun: {arguments.output}")
            return 0
        if arguments.reachability_command == "plot":
            sources = tuple(load_reachability_artifact(path) for path in arguments.compare)
            report = render_reachability_plot_bundle(
                load_reachability_artifact(arguments.path),
                arguments.output_dir,
                comparison_sources=sources,
                dpi=arguments.dpi,
            )
            print(f"rendered {len(report.plot_paths)} reachability plot(s): {arguments.output_dir}")
            return 0
        catalog = _catalog(arguments.catalog)
        if arguments.reachability_command == "list":
            if arguments.kind == "profiles":
                _print_json(
                    [
                        {
                            "profile_id": profile.id,
                            "archetype": profile.archetype,
                            "status": profile.status,
                            "configurations": list(profile.configurations),
                            "coordinates": list(profile.coordinates),
                            "products": list(profile.products),
                        }
                        for profile in catalog.profiles
                    ]
                )
            elif arguments.kind == "families":
                _print_json(
                    [
                        {
                            "family_id": family.id,
                            "status": family.status,
                            "configurations": list(family.configurations),
                            "profiles": list(family.profiles),
                        }
                        for family in catalog.vehicle_families
                    ]
                )
            else:
                _print_json(
                    {
                        name: {"meaning": semantic.meaning}
                        for name, semantic in sorted(catalog.study_semantics.items())
                    }
                )
        elif arguments.kind == "profile":
            _print_json(catalog.profile(arguments.identifier).model_dump(mode="json"))
        elif arguments.kind == "family":
            _print_json(catalog.family(arguments.identifier).model_dump(mode="json"))
        else:
            _print_json(catalog.semantic(arguments.identifier).model_dump(mode="json"))
        return 0
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f"error: reachability-failed: {error}")
        return 2
    ####


__all__ = ["handle_reachability_command"]
####
