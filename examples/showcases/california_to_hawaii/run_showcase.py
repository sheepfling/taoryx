"""Generate the native TAORYX California-to-Hawaii showcase artifact."""

from __future__ import annotations

import argparse
import hashlib
import json
import mimetypes
import os
from pathlib import Path

from taoryx.language import GrammarProfile
from taoryx.outputs import RunArtifact
from taoryx.runtime.runner import run_files
from taoryx.showcase import (
    ArtifactFile,
    EvidenceBoardSpec,
    FidelityShowcaseRealization,
    build_showcase_run_artifact,
)

MISSION_FILE = Path(__file__).with_name("mission.prb")
AERO_TABLE_FILE = Path(__file__).with_name("aero.tbl")
START = (34.70, -120.60)
DESTINATION = (21.31, -157.86)


def build_native_artifact(output_dir: Path) -> RunArtifact:
    """Execute the declared mission and table contracts through the runner."""

    report = run_files(
        MISSION_FILE,
        (AERO_TABLE_FILE,),
        output_dir=output_dir,
        max_steps=3_000,
        profile=GrammarProfile.TAORYX,
    )
    if report.exit_code != 0 or not report.artifacts:
        diagnostics = "; ".join(f"{item.code}: {item.message}" for item in report.diagnostics)
        raise ValueError(f"native CA-HI run failed: {diagnostics}")
    return report.artifacts[0]
    ####


def _channel(artifact: RunArtifact, source_name: str) -> list[float]:
    vehicle = next(iter(artifact.vehicles.values()))
    channel = next(channel for channel in vehicle.channels.values() if channel.source_name == source_name)
    return [float(value) for value in channel.values if value is not None]
    ####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
    ####


def _payload_sha256(value: object) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _artifact_files(root: Path, scenario_hash: str) -> tuple[ArtifactFile, ...]:
    """Describe the native CA-HI packet through the common file contract."""

    names = [
        path.relative_to(root).as_posix()
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.name != "manifest.json"
    ]
    names.append("manifest.json")
    return tuple(
        ArtifactFile(
            path=name,
            sha256=scenario_hash if name == "manifest.json" else _sha256(root / name),
            media_type=mimetypes.guess_type(name)[0] or "application/octet-stream",
        )
        for name in names
    )
    ####


def _realization() -> FidelityShowcaseRealization:
    """Declare the native CA-HI control boundary without implying surfaces."""

    return FidelityShowcaseRealization(
        fidelity="rigid_body_6dof",
        control_realization="direct_wrench",
        realization_id="california-to-hawaii-native-direct-wrench-v1",
        state_schema=("ecic_position_velocity_quaternion_body_rates_mass_resource",),
        semantic_command_mapping={
            "throttle": "declared propulsion command",
            "alpha_deg": "declared aerodynamic guidance command",
            "bank_deg": "declared aerodynamic guidance command",
        },
        available_physics=(
            "native rigid-body quaternion integration",
            "declared propulsion and mass-flow segments",
            "source-bounded aerodynamic force and moment tables",
            "thermal and terminal guidance telemetry",
        ),
        claim=(
            "The native CA-HI example executes a source-bounded rigid-body "
            "trajectory with staged propulsion, mass flow, aerodynamic loads, "
            "and terminal guidance through the direct-wrench bridge."
        ),
        nonclaims=(
            "physical control-surface allocation",
            "manufacturer or flight-test controller fidelity",
            "validated thermal or aeroballistic design",
            "historical TAOS 96 compatibility",
        ),
        evidence_grade="synthetic",
    )
    ####


def write_plots(artifact: RunArtifact, output_dir: Path) -> None:
    """Render every showcase panel from native runner telemetry."""

    output_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(output_dir / ".mplconfig"))
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    vehicle = next(iter(artifact.vehicles.values()))
    times = vehicle.times
    latitude = _channel(artifact, "latitude_deg")
    longitude = _channel(artifact, "longitude_deg")
    figure, axis = plt.subplots(figsize=(8, 5), layout="constrained")
    axis.plot(longitude, latitude, color="#2563eb", linewidth=2.2)
    axis.scatter([START[1], DESTINATION[1]], [START[0], DESTINATION[0]], color=("#16a34a", "#dc2626"))
    axis.set(title="Native TAORYX California-to-Hawaii route", xlabel="Longitude (deg)", ylabel="Latitude (deg)")
    figure.savefig(output_dir / "route_map.png", dpi=140)
    plt.close(figure)

    panels = (
        ("altitude_range.png", ("altitude_m", "range_to_target_m"), "Altitude and remaining range"),
        ("altitude_mach.png", ("altitude_m", "speed_m_s", "aero_airspeed_m_s"), "Altitude and translational speed corridor"),
        ("mass_propulsion.png", ("speed_m_s", "propellant_mass_rate_kg_s", "force_body_x_n"), "Speed, propellant flow, and body-X propulsion"),
        ("thermal_exposure.png", ("heat_rate_w_m2",), "Heating rate"),
        ("forces_torques.png", ("force_body_x_n", "force_body_y_n", "force_body_z_n", "total_force_ecic_n", "moment_body_y_nm"), "Force and control-torque observables"),
        ("guidance_response.png", ("pro_nav_acceleration_m_s2", "pro_nav_achieved_aero_acceleration_m_s2", "pro_nav_acceleration_response_residual_m_s2", "pro_nav_closing_velocity_m_s"), "ProNav command, aero response, residual, and closing rate"),
        ("orientation.png", ("roll_deg", "pitch_deg", "yaw_deg"), "Integrated orientation (degrees)"),
        ("attitude_rates.png", ("qw", "qx", "qy", "qz", "wx", "wy", "wz"), "Attitude quaternion and body rates"),
    )
    for filename, channels, title in panels:
        figure, axis = plt.subplots(figsize=(8, 4.5), layout="constrained")
        for name in channels:
            axis.plot(times, _channel(artifact, name), label=name)
        axis.set(title=title, xlabel="Time (s)")
        axis.legend(loc="best", fontsize="small")
        figure.savefig(output_dir / filename, dpi=140)
        plt.close(figure)
    ####


def generate(output_dir: Path) -> Path:
    """Run the native contract and write all artifact views."""

    artifact = build_native_artifact(output_dir / "native-run")
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = artifact.write_json(output_dir / "run.json")
    artifact.write_sqlite(output_dir / "run.sqlite", run_id="california-to-hawaii")
    (output_dir / "summary.txt").write_text(artifact.format_text(max_rows=12), encoding="utf-8")
    write_plots(artifact, output_dir / "plots")
    realization = _realization()
    (output_dir / "realized_fidelity.json").write_text(
        json.dumps(realization.model_dump(mode="json"), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "claim.json").write_text(
        json.dumps(
            {
                "claim": realization.claim,
                "nonclaims": list(realization.nonclaims),
                "fidelity": realization.fidelity,
                "control_realization": realization.control_realization,
                "outcome": "completed",
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    scenario_hash = _payload_sha256(
        {
            "mission": MISSION_FILE.read_text(encoding="utf-8"),
            "aero_table": AERO_TABLE_FILE.read_text(encoding="utf-8"),
            "start": START,
            "destination": DESTINATION,
        }
    )
    showcase_run = build_showcase_run_artifact(
        realization=realization,
        run_id="california-to-hawaii-native-direct-wrench-v1",
        showcase_id="org.taoryx.showcase.california_to_hawaii.native",
        vehicle_binding_id="synthetic.ca-hi-x8-plus-boosters-v1",
        scenario_contract_sha256=scenario_hash,
        outcome="completed",
        files=_artifact_files(output_dir, scenario_hash),
        board=EvidenceBoardSpec(
            profile="family-evidence-board-v1",
            modules=(
                "trajectory_3d",
                "trajectory_vertical",
                "mission_timeline",
                "energy_and_resources",
                "attitude_and_rates",
                "semantic_controls",
                "envelope_margins",
            ),
        ),
        archetypes=(
            "mission_geometry",
            "mission_timeline",
            "dynamics_and_resources",
            "envelope_and_qualification",
        ),
    )
    manifest = {
        "schema_version": 1,
        "showcase_id": "org.taoryx.showcase.california_to_hawaii.native",
        "family": "rocket_plane",
        "vehicle": "synthetic.ca-hi-x8-plus-boosters-v1",
        "realized_fidelity": "realized_fidelity.json",
        "run_artifacts": [showcase_run.model_dump(mode="json")],
        "files": {},
    }
    manifest_path = output_dir / "manifest.json"
    manifest["files"] = {
        path.relative_to(output_dir).as_posix(): _sha256(path)
        for path in sorted(output_dir.rglob("*"))
        if path.is_file() and path != manifest_path
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return json_path
    ####


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/showcases/california_to_hawaii"))
    arguments = parser.parse_args()
    print(generate(arguments.output_dir))
    ####


if __name__ == "__main__":
    main()
