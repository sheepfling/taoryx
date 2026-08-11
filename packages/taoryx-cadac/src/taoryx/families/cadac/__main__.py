"""Command-line inspection and source-compatibility utilities for CADAC."""

from __future__ import annotations

import argparse
import json
from collections.abc import Sequence

from .ads6_aircraft import (
    Ads6AircraftThreatTrack,
    load_ads6_aircraft_source_definition,
    run_ads6_aircraft_source_compatibility,
)
from .ads6_engagement import (
    Ads6EngagementRunConfig,
    load_ads6_engagement_source_definition,
    run_ads6_engagement,
)
from .ads6_sam import (
    Ads6SamControlCommand,
    Ads6SamDirectCommand,
    load_ads6_sam_source_definition,
    run_ads6_sam_physical_plant,
)
from .ads6_srbm import load_ads6_srbm_source_definition, run_ads6_srbm_source_compatibility
from .agm6 import load_agm6_source_definition, run_agm6_source_compatibility
from .aim5 import load_aim5_source_definition, run_aim5_source_compatibility
from .aim5_parity import compare_aim5_source_plot
from .aim5_scenario import load_aim5_scenario_source_definition, run_aim5_scenario_source_compatibility
from .bundle import load_cadac_source_bundle
from .cruise5 import load_cruise5_source_definition, run_cruise5_source_compatibility
from .deck import parse_cadac_deck_file
from .falcon6 import load_falcon6_source_definition
from .ghame3 import load_ghame3_source_definition, run_ghame3_source_compatibility
from .ghame6 import Ghame6DirectCommand, load_ghame6_source_definition, run_ghame6_phase_aware_mission
from .input_parser import parse_cadac_input_file
from .magsix import load_magsix_source_definition, run_magsix_trajectory_source_compatibility
from .manifest import CADAC_MANIFEST_CATALOG, manifest_for
from .mission_composition_plugin import CadacMissionCompositionProvider
from .parity import parse_cadac_plot_file
from .plugin import CADAC_PLUGIN_CATALOG
from .rocket6g import (
    Rocket6gDirectCommand,
    load_rocket6g_source_definition,
    run_rocket6g_phase_aware_plant,
)
from .rocket6g_rcs import load_rocket6g_rcs_source_definition
from .sraam6 import load_sraam6_source_definition, run_sraam6_source_compatibility
from .table_conversion import (
    convert_cadac_deck_file_to_table_bundle,
    convert_cadac_source_bundle_to_table_bundle,
    convert_cadac_tree_to_table_bundle,
    validate_converted_table_bundle,
)


def build_parser() -> argparse.ArgumentParser:
    """Build the source-inspection and executable-prototype command surface."""

    parser = argparse.ArgumentParser(prog="python -m taoryx.families.cadac")
    subparsers = parser.add_subparsers(dest="command", required=True)

    input_parser = subparsers.add_parser("input", help="parse one CADAC input.asc file")
    input_parser.add_argument("path")

    deck_parser = subparsers.add_parser("deck", help="parse one CADAC table deck")
    deck_parser.add_argument("path")

    plot_parser = subparsers.add_parser("plot", help="parse one CADAC plot*.asc output")
    plot_parser.add_argument("path")

    bundle_parser = subparsers.add_parser("bundle", help="parse a case and resolve all referenced decks")
    bundle_parser.add_argument("path")

    convert_deck_parser = subparsers.add_parser(
        "convert-deck",
        help="convert one CADAC physical deck into a canonical Taoryx table bundle",
    )
    convert_deck_parser.add_argument("path")
    convert_deck_parser.add_argument("--output", required=True)

    convert_bundle_parser = subparsers.add_parser(
        "convert-bundle",
        help="convert all unique decks referenced by one CADAC case into canonical Taoryx tables",
    )
    convert_bundle_parser.add_argument("path")
    convert_bundle_parser.add_argument("--output", required=True)

    convert_tree_parser = subparsers.add_parser(
        "convert-tree",
        help="convert every table-bearing CADAC .asc deck under one source tree",
    )
    convert_tree_parser.add_argument("path")
    convert_tree_parser.add_argument("--output", required=True)

    validate_table_parser = subparsers.add_parser(
        "validate-table",
        help="validate a canonical Taoryx table bundle and all resource checksums",
    )
    validate_table_parser.add_argument("path")

    catalog_parser = subparsers.add_parser("catalog", help="emit phase-aware fidelity manifests")
    catalog_parser.add_argument("--package", dest="package_id")

    subparsers.add_parser("plugins", help="emit actor-level CADAC vehicle plug-in descriptors")
    subparsers.add_parser("provider-catalog", help="emit Taoryx Mission Composition metadata for every dynamic CADAC actor")

    ads6_aircraft_lower_parser = subparsers.add_parser(
        "ads6-aircraft-lower",
        help="validate and lower one standalone ADS6 AIRCRAFT3 point-mass source case",
    )
    ads6_aircraft_lower_parser.add_argument("path")

    ads6_aircraft_run_parser = subparsers.add_parser(
        "ads6-aircraft-run",
        help="execute one standalone ADS6 AIRCRAFT3 steady, g-turn, or escape source case",
    )
    ads6_aircraft_run_parser.add_argument("path")
    ads6_aircraft_run_parser.add_argument("--end-time", type=float, default=None)
    ads6_aircraft_run_parser.add_argument("--sample-step", type=float, default=None)
    ads6_aircraft_run_parser.add_argument("--threat-position", type=float, nargs=3, metavar=("N", "E", "D"), default=None)
    ads6_aircraft_run_parser.add_argument("--threat-velocity", type=float, nargs=3, metavar=("VN", "VE", "VD"), default=None)
    ads6_aircraft_run_parser.add_argument("--threat-reference-time", type=float, default=0.0)

    ads6_sam_lower_parser = subparsers.add_parser(
        "ads6-sam-lower",
        help="validate and lower one standalone ADS6 SAM physical-plant source case",
    )
    ads6_sam_lower_parser.add_argument("path")

    ads6_sam_run_parser = subparsers.add_parser(
        "ads6-sam-run",
        help="execute one standalone ADS6 SAM fin, TVC, or aggregate-RCS realization",
    )
    ads6_sam_run_parser.add_argument("path")
    ads6_sam_run_parser.add_argument(
        "--phase",
        choices=("fin_control", "tvc_control", "aggregate_rcs"),
        default="fin_control",
    )
    ads6_sam_run_parser.add_argument("--end-time", type=float, default=None)
    ads6_sam_run_parser.add_argument("--sample-step", type=float, default=None)
    ads6_sam_run_parser.add_argument("--roll-deg", type=float, default=0.0)
    ads6_sam_run_parser.add_argument("--pitch-deg", type=float, default=0.0)
    ads6_sam_run_parser.add_argument("--yaw-deg", type=float, default=0.0)
    ads6_sam_run_parser.add_argument("--tvc-mode", type=int, default=None)
    ads6_sam_run_parser.add_argument("--rcs-moment-mode", type=int, default=None)
    ads6_sam_run_parser.add_argument("--rcs-force-mode", type=int, default=None)
    ads6_sam_run_parser.add_argument("--rcs-roll-command-deg", type=float, default=None)
    ads6_sam_run_parser.add_argument("--rcs-pitch-command-deg", type=float, default=None)
    ads6_sam_run_parser.add_argument("--rcs-yaw-command-deg", type=float, default=None)

    ads6_srbm_lower_parser = subparsers.add_parser(
        "ads6-srbm-lower",
        help="validate and lower one ADS6 ROCKET5 pseudo-6DoF source case",
    )
    ads6_srbm_lower_parser.add_argument("path")

    ads6_srbm_run_parser = subparsers.add_parser(
        "ads6-srbm-run",
        help="execute one ADS6 ROCKET5 ascent, exo-ballistic, and reentry source case",
    )
    ads6_srbm_run_parser.add_argument("path")
    ads6_srbm_run_parser.add_argument("--end-time", type=float, default=None)
    ads6_srbm_run_parser.add_argument("--sample-step", type=float, default=None)

    ads6_engagement_lower_parser = subparsers.add_parser(
        "ads6-engagement-lower",
        help="validate and lower one source-ordered ADS6 SAM/target/RADAR0 package case",
    )
    ads6_engagement_lower_parser.add_argument("path")

    ads6_engagement_run_parser = subparsers.add_parser(
        "ads6-engagement-run",
        help="execute one source-ordered ADS6 aircraft- or SRBM-defense package case",
    )
    ads6_engagement_run_parser.add_argument("path")
    ads6_engagement_run_parser.add_argument("--end-time", type=float, default=None)
    ads6_engagement_run_parser.add_argument("--sample-step", type=float, default=None)
    ads6_engagement_run_parser.add_argument(
        "--command-law",
        choices=("source_controller", "hold", "line_of_sight"),
        default="source_controller",
    )
    ads6_engagement_run_parser.add_argument("--pointing-gain", type=float, default=0.35)
    ads6_engagement_run_parser.add_argument("--command-limit-deg", type=float, default=20.0)
    ads6_engagement_run_parser.add_argument("--radar-seed", type=int, default=0)

    agm6_lower_parser = subparsers.add_parser("agm6-lower", help="validate and lower one AGM6 three-actor physical-fin engagement")
    agm6_lower_parser.add_argument("path")

    agm6_run_parser = subparsers.add_parser("agm6-run", help="execute one source-ordered AGM6/TARGET3/AIRCRAFT3 engagement")
    agm6_run_parser.add_argument("path")
    agm6_run_parser.add_argument("--end-time", type=float, default=None)
    agm6_run_parser.add_argument("--sample-step", type=float, default=None)
    agm6_run_parser.add_argument("--random-seed", type=int, default=None)

    cruise5_lower_parser = subparsers.add_parser("cruise5-lower", help="validate and lower one CRUISE5 source case")
    cruise5_lower_parser.add_argument("path")

    cruise5_run_parser = subparsers.add_parser("cruise5-run", help="execute one CRUISE5 source-compatible pseudo-6DoF case")
    cruise5_run_parser.add_argument("path")
    cruise5_run_parser.add_argument("--end-time", type=float, default=None)
    cruise5_run_parser.add_argument("--sample-step", type=float, default=None)

    ghame3_lower_parser = subparsers.add_parser("ghame3-lower", help="validate and lower one GHAME3 point-mass source case")
    ghame3_lower_parser.add_argument("path")

    ghame3_run_parser = subparsers.add_parser("ghame3-run", help="execute one GHAME3 source-compatible point-mass case")
    ghame3_run_parser.add_argument("path")
    ghame3_run_parser.add_argument("--end-time", type=float, default=None)
    ghame3_run_parser.add_argument("--sample-step", type=float, default=None)

    ghame6_lower_parser = subparsers.add_parser(
        "ghame6-lower",
        help="validate and lower one phase-aware GHAME6 HYPER6/SAT3/RADAR0 source mission",
    )
    ghame6_lower_parser.add_argument("path")

    ghame6_run_parser = subparsers.add_parser(
        "ghame6-run",
        help="execute one phase-aware GHAME6 atmospheric-surface to aggregate-RCS mission",
    )
    ghame6_run_parser.add_argument("path")
    ghame6_run_parser.add_argument("--end-time", type=float, default=None)
    ghame6_run_parser.add_argument("--sample-step", type=float, default=None)
    ghame6_run_parser.add_argument("--aileron-deg", type=float, default=0.0)
    ghame6_run_parser.add_argument("--elevator-deg", type=float, default=0.0)
    ghame6_run_parser.add_argument("--rudder-deg", type=float, default=0.0)
    ghame6_run_parser.add_argument("--boost-cutoff-time", type=float, default=None)
    ghame6_run_parser.add_argument("--terminal-lock-time", type=float, default=None)
    ghame6_run_parser.add_argument("--random-seed", type=int, default=12345)

    magsix_lower_parser = subparsers.add_parser("magsix-lower", help="validate and lower one MAGSIX trajectory-only source case")
    magsix_lower_parser.add_argument("path")

    magsix_run_parser = subparsers.add_parser("magsix-run", help="execute one MAGSIX source-compatible trajectory-only case")
    magsix_run_parser.add_argument("path")
    magsix_run_parser.add_argument("--end-time-dnt", type=float, default=None)
    magsix_run_parser.add_argument("--sample-step-dnt", type=float, default=None)

    rocket6g_lower_parser = subparsers.add_parser(
        "rocket6g-lower",
        help="validate and lower one phase-aware ROCKET6G source case",
    )
    rocket6g_lower_parser.add_argument("path")

    rocket6g_run_parser = subparsers.add_parser(
        "rocket6g-run",
        help="execute one phase-aware ROCKET6G rigid-body plant case",
    )
    rocket6g_run_parser.add_argument("path")
    rocket6g_run_parser.add_argument("--end-time", type=float, default=None)
    rocket6g_run_parser.add_argument("--sample-step", type=float, default=None)
    rocket6g_run_parser.add_argument("--tvc-pitch-deg", type=float, default=0.0)
    rocket6g_run_parser.add_argument("--tvc-yaw-deg", type=float, default=0.0)
    rocket6g_run_parser.add_argument("--boost-cutoff-time", type=float, default=None)

    rocket6g_rcs_parser = subparsers.add_parser("rocket6g-rcs-lower", help="lower the aggregate ROCKET6G RCS direct-wrench subsystem")
    rocket6g_rcs_parser.add_argument("path")

    sraam6_lower_parser = subparsers.add_parser("sraam6-lower", help="validate and lower one SRAAM6 physical-fin engagement case")
    sraam6_lower_parser.add_argument("path")

    sraam6_run_parser = subparsers.add_parser("sraam6-run", help="execute one source-ordered SRAAM6/TARGET3 physical-fin engagement")
    sraam6_run_parser.add_argument("path")
    sraam6_run_parser.add_argument("--end-time", type=float, default=None)
    sraam6_run_parser.add_argument("--sample-step", type=float, default=None)

    falcon6_lower_parser = subparsers.add_parser("falcon6-lower", help="validate and lower one FALCON6 source case")
    falcon6_lower_parser.add_argument("path")

    aim5_lower_parser = subparsers.add_parser("aim5-lower", help="validate and lower one single-engagement AIM5 source case")
    aim5_lower_parser.add_argument("path")

    aim5_run_parser = subparsers.add_parser("aim5-run", help="execute one single-engagement AIM5 case with CADAC source semantics")
    aim5_run_parser.add_argument("path")
    aim5_run_parser.add_argument("--sample-step", type=float, default=None)
    aim5_run_parser.add_argument("--trace-steps", type=int, default=0)

    aim5_scenario_parser = subparsers.add_parser(
        "aim5-scenario-run",
        help="execute one single- or multiple-engagement AIM5 case with source vehicle/bus ordering",
    )
    aim5_scenario_parser.add_argument("path")
    aim5_scenario_parser.add_argument("--sample-step", type=float, default=None)
    aim5_scenario_parser.add_argument("--trace-steps", type=int, default=0)

    aim5_compare_parser = subparsers.add_parser("aim5-compare", help="compare Python AIM5 execution with one CADAC plot*.asc")
    aim5_compare_parser.add_argument("input_path")
    aim5_compare_parser.add_argument("plot_path")
    aim5_compare_parser.add_argument("--abs-tol", type=float, default=1.0e-6)
    aim5_compare_parser.add_argument("--rel-tol", type=float, default=1.0e-6)
    return parser


####


def main(argv: Sequence[str] | None = None) -> int:
    """Parse, lower, execute, or classify one CADAC source artifact."""

    args = build_parser().parse_args(list(argv) if argv is not None else None)
    if args.command == "input":
        payload = parse_cadac_input_file(args.path)
    elif args.command == "deck":
        payload = parse_cadac_deck_file(args.path)
    elif args.command == "plot":
        payload = parse_cadac_plot_file(args.path)
    elif args.command == "bundle":
        payload = load_cadac_source_bundle(args.path)
    elif args.command == "convert-deck":
        payload = convert_cadac_deck_file_to_table_bundle(args.path, args.output)
    elif args.command == "convert-bundle":
        payload = convert_cadac_source_bundle_to_table_bundle(args.path, args.output)
    elif args.command == "convert-tree":
        payload = convert_cadac_tree_to_table_bundle(args.path, args.output)
    elif args.command == "validate-table":
        loaded = validate_converted_table_bundle(args.path)
        payload = {
            "manifest": loaded.manifest.model_dump(mode="json"),
            "table_count": len(loaded.tables),
            "table_ids": [table.table_id for table in loaded.tables],
        }
    elif args.command == "catalog":
        payload = manifest_for(args.package_id) if args.package_id else CADAC_MANIFEST_CATALOG
    elif args.command == "plugins":
        payload = CADAC_PLUGIN_CATALOG
    elif args.command == "provider-catalog":
        provider = CadacMissionCompositionProvider()
        payload = {
            "provider": provider.metadata.model_dump(mode="json"),
            "models": [item.model_dump(mode="json") for item in provider.list_models()],
        }
    elif args.command == "ads6-engagement-lower":
        payload = load_ads6_engagement_source_definition(args.path)
    elif args.command == "ads6-engagement-run":
        definition = load_ads6_engagement_source_definition(args.path)
        payload = run_ads6_engagement(
            definition,
            Ads6EngagementRunConfig(
                end_time_s=args.end_time,
                sample_step_s=args.sample_step,
                command_law=args.command_law,
                pointing_gain=args.pointing_gain,
                command_limit_deg=args.command_limit_deg,
                radar_seed=args.radar_seed,
            ),
        )
    elif args.command == "ads6-aircraft-lower":
        payload = load_ads6_aircraft_source_definition(args.path)
    elif args.command == "ads6-aircraft-run":
        if (args.threat_position is None) != (args.threat_velocity is None):
            raise ValueError("ADS6 AIRCRAFT3 CLI requires both --threat-position and --threat-velocity")
        ####
        threat_track = (
            Ads6AircraftThreatTrack(
                position_ned_m=tuple(args.threat_position),
                velocity_ned_mps=tuple(args.threat_velocity),
                reference_time_s=args.threat_reference_time,
            )
            if args.threat_position is not None
            else None
        )
        definition = load_ads6_aircraft_source_definition(args.path)
        payload = run_ads6_aircraft_source_compatibility(
            definition,
            threat_track=threat_track,
            end_time_s=args.end_time,
            sample_step_s=args.sample_step,
        )
    elif args.command == "ads6-sam-lower":
        payload = load_ads6_sam_source_definition(args.path)
    elif args.command == "ads6-sam-run":
        definition = load_ads6_sam_source_definition(args.path)
        payload = run_ads6_sam_physical_plant(
            definition,
            Ads6SamDirectCommand(
                phase=args.phase,
                control=Ads6SamControlCommand(
                    roll_deg=args.roll_deg,
                    pitch_deg=args.pitch_deg,
                    yaw_deg=args.yaw_deg,
                ),
                tvc_mode=args.tvc_mode,
                rcs_moment_mode=args.rcs_moment_mode,
                rcs_force_mode=args.rcs_force_mode,
                roll_attitude_command_deg=args.rcs_roll_command_deg,
                pitch_attitude_command_deg=args.rcs_pitch_command_deg,
                yaw_attitude_command_deg=args.rcs_yaw_command_deg,
            ),
            end_time_s=args.end_time,
            sample_step_s=args.sample_step,
        )
    elif args.command == "ads6-srbm-lower":
        payload = load_ads6_srbm_source_definition(args.path)
    elif args.command == "ads6-srbm-run":
        definition = load_ads6_srbm_source_definition(args.path)
        payload = run_ads6_srbm_source_compatibility(
            definition,
            end_time_s=args.end_time,
            sample_step_s=args.sample_step,
        )
    elif args.command == "agm6-lower":
        payload = load_agm6_source_definition(args.path)
    elif args.command == "agm6-run":
        definition = load_agm6_source_definition(args.path)
        payload = run_agm6_source_compatibility(
            definition,
            end_time_s=args.end_time,
            sample_step_s=args.sample_step,
            random_seed=args.random_seed,
        )
    elif args.command == "cruise5-lower":
        payload = load_cruise5_source_definition(args.path)
    elif args.command == "cruise5-run":
        definition = load_cruise5_source_definition(args.path)
        payload = run_cruise5_source_compatibility(
            definition,
            end_time_s=args.end_time,
            sample_step_s=args.sample_step,
        )
    elif args.command == "ghame3-lower":
        payload = load_ghame3_source_definition(args.path)
    elif args.command == "ghame3-run":
        definition = load_ghame3_source_definition(args.path)
        payload = run_ghame3_source_compatibility(
            definition,
            end_time_s=args.end_time,
            sample_step_s=args.sample_step,
        )
    elif args.command == "ghame6-lower":
        payload = load_ghame6_source_definition(args.path)
    elif args.command == "ghame6-run":
        definition = load_ghame6_source_definition(args.path)
        payload = run_ghame6_phase_aware_mission(
            definition,
            Ghame6DirectCommand(
                aileron_command_deg=args.aileron_deg,
                elevator_command_deg=args.elevator_deg,
                rudder_command_deg=args.rudder_deg,
                boost_cutoff_time_s=args.boost_cutoff_time,
                terminal_lock_time_s=args.terminal_lock_time,
            ),
            end_time_s=args.end_time,
            sample_step_s=args.sample_step,
            random_seed=args.random_seed,
        )
    elif args.command == "magsix-lower":
        payload = load_magsix_source_definition(args.path)
    elif args.command == "magsix-run":
        definition = load_magsix_source_definition(args.path)
        payload = run_magsix_trajectory_source_compatibility(
            definition,
            end_time_dnt=args.end_time_dnt,
            sample_step_dnt=args.sample_step_dnt,
        )
    elif args.command == "rocket6g-lower":
        payload = load_rocket6g_source_definition(args.path)
    elif args.command == "rocket6g-run":
        definition = load_rocket6g_source_definition(args.path)
        payload = run_rocket6g_phase_aware_plant(
            definition,
            Rocket6gDirectCommand(
                tvc_pitch_command_deg=args.tvc_pitch_deg,
                tvc_yaw_command_deg=args.tvc_yaw_deg,
                boost_cutoff_time_s=args.boost_cutoff_time,
            ),
            end_time_s=args.end_time,
            sample_step_s=args.sample_step,
        )
    elif args.command == "rocket6g-rcs-lower":
        payload = load_rocket6g_rcs_source_definition(args.path)
    elif args.command == "sraam6-lower":
        payload = load_sraam6_source_definition(args.path)
    elif args.command == "sraam6-run":
        definition = load_sraam6_source_definition(args.path)
        payload = run_sraam6_source_compatibility(
            definition,
            end_time_s=args.end_time,
            sample_step_s=args.sample_step,
        )
    elif args.command == "falcon6-lower":
        payload = load_falcon6_source_definition(args.path)
    elif args.command == "aim5-lower":
        payload = load_aim5_source_definition(args.path)
    elif args.command == "aim5-run":
        definition = load_aim5_source_definition(args.path)
        payload = run_aim5_source_compatibility(definition, sample_step_s=args.sample_step, trace_steps=args.trace_steps)
    elif args.command == "aim5-scenario-run":
        definition = load_aim5_scenario_source_definition(args.path)
        payload = run_aim5_scenario_source_compatibility(
            definition,
            sample_step_s=args.sample_step,
            trace_steps=args.trace_steps,
        )
    elif args.command == "aim5-compare":
        payload = compare_aim5_source_plot(
            args.input_path,
            args.plot_path,
            absolute_tolerance=args.abs_tol,
            relative_tolerance=args.rel_tol,
        )
    else:
        raise RuntimeError(f"unhandled command {args.command!r}")
    ####
    if hasattr(payload, "model_dump_json"):
        print(payload.model_dump_json(indent=2, by_alias=True))
    else:
        print(json.dumps(payload, indent=2, sort_keys=True))
    ####
    return 0


####


if __name__ == "__main__":
    raise SystemExit(main())
####
