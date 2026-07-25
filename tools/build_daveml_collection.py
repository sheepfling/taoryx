"""Build a deterministic M0-M2 Taoryx collection from a verified .txair package."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import zipfile
from pathlib import Path
from typing import Any

from taoryx.trajectory import (
    CollectionManifest,
    ComponentBinding,
    ContributionAuthority,
    SourceDocument,
    StatefulComponentContract,
    TransformRecord,
    inspect_reference_package,
    load_reference_family_manifest,
    write_deterministic_collection,
)

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FAMILY = ROOT / "families/reference_f16_s119/family.yaml"


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()
####


def _json(data: Any) -> bytes:
    return (json.dumps(data, indent=2, sort_keys=True) + "\n").encode("utf-8")
####


def _read_json(package: zipfile.ZipFile, member: str) -> dict[str, Any]:
    value: Any = json.loads(package.read(member))
    if not isinstance(value, dict):
        raise ValueError(f"package member {member!r} must be a JSON object")
    return value
####


def build_f16_collection(family_manifest_path: Path, source_root: Path, output: Path) -> dict[str, Any]:
    """Build one F-16 collection and return its archive report."""

    family = load_reference_family_manifest(family_manifest_path)
    if family.family_id != "reference_f16_s119":
        raise ValueError("the first collection builder is intentionally limited to reference_f16_s119")
    package_path = source_root / family.source.package_relative_path
    package_report = inspect_reference_package(family, package_path)

    with zipfile.ZipFile(package_path) as package:
        package_manifest = _read_json(package, "manifest.json")
        control_binding = _read_json(package, "bindings/control-binding.json")
        runtime_binding = _read_json(package, "runtime/binding.json")
        acceptance = _read_json(package, "validation/acceptance.json")
        source_documents: list[SourceDocument] = []
        files: dict[str, bytes] = {}
        for document in package_manifest["source"]["documents"]:
            member = str(document["path"])
            payload = package.read(member)
            source_documents.append(
                SourceDocument(
                    document_id=f"f16-s119.{document['role']}",
                    role=str(document["role"]),
                    source_path=str(document.get("upstream_path", member)),
                    package_member=member,
                    sha256=_sha256(payload),
                    byte_identical_to_upstream=bool(document["byte_identical_to_upstream"]),
                    upstream_repository=document.get("upstream_repository"),
                    upstream_path=document.get("upstream_path"),
                )
            )
            files[f"source/{member.removeprefix('models/')}"] = payload
        trim_hold = package.read("validation/trim-hold.csv")
        runtime_package = package_path.read_bytes()
        files["runtime/aircraft.txair"] = runtime_package
        files["validation/trim-hold.csv"] = trim_hold

    documents_json = [document.model_dump(mode="json") for document in source_documents]
    runtime_components = runtime_binding.get("component_paths", {})
    manifest = CollectionManifest(
        collection_type="taoryx.txcollection/v1alpha1",
        collection_id="f16-s119",
        collection_version="1.0.0",
        family_id=family.family_id,
        canonical_authority="taoryx_canonical",
        source_documents=tuple(source_documents),
        component_bindings=(
            ComponentBinding(
                component_id="aerodynamics",
                role="aerodynamics",
                source_document_ids=("f16-s119.aerodynamics",),
                inputs=("state.alpha", "state.beta", "state.body_rates", "state.mach", "state.altitude"),
                outputs=("aero.body_force_coefficients", "aero.body_moment_coefficients"),
                runtime_member=runtime_components.get("aerodynamics"),
            ),
            ComponentBinding(
                component_id="propulsion",
                role="propulsion",
                source_document_ids=("f16-s119.propulsion",),
                inputs=("state.mach", "state.altitude", "control.throttle"),
                outputs=("propulsion.thrust",),
                runtime_member=runtime_components.get("propulsion"),
            ),
            ComponentBinding(
                component_id="mass_properties",
                role="mass-properties",
                source_document_ids=("f16-s119.mass_properties",),
                inputs=("state.cg_offset",),
                outputs=("mass.total", "mass.cg", "mass.inertia"),
                runtime_member=runtime_components.get("mass_properties"),
            ),
            ComponentBinding(
                component_id="control_mapping",
                role="control-mapping",
                source_document_ids=(),
                inputs=("control.elevator", "control.aileron", "control.rudder", "control.throttle"),
                outputs=("source.elevator_ratio", "source.aileron_ratio", "source.rudder_ratio", "source.throttle_ratio"),
                runtime_member="bindings/control-binding.json",
            ),
        ),
        contribution_authority=(
            ContributionAuthority(contribution="aerodynamic.force", producer="aerodynamics", status="authoritative", rationale="source DAVE-ML graph"),
            ContributionAuthority(contribution="aerodynamic.moment", producer="aerodynamics", status="authoritative", rationale="source DAVE-ML graph"),
            ContributionAuthority(contribution="propulsion.thrust", producer="propulsion", status="authoritative", rationale="steady-state source model"),
            ContributionAuthority(contribution="mass.inertia", producer="mass_properties", status="authoritative", rationale="fixed source model"),
        ),
        transforms=(
            TransformRecord(transform_id="frame.body", kind="frame", source="FRD", canonical="FRD", reason="package canonical body frame"),
            TransformRecord(transform_id="frame.navigation", kind="frame", source="NED", canonical="NED", reason="package canonical navigation frame"),
            TransformRecord(transform_id="attitude.quaternion", kind="axis", source="wxyz", canonical="wxyz", reason="package quaternion order"),
        ),
        stateful_components=(
            StatefulComponentContract(component_id="equations_of_motion", classification="external_required", states=("position", "velocity", "attitude", "body_rates"), source="Taoryx runtime", notes="Static DAVE-ML supplies loads; runtime integrates state."),
            StatefulComponentContract(component_id="fuel_depletion", classification="not_exported", source="runtime/binding.json", notes="Package declares constant mass and no fuel-flow output."),
            StatefulComponentContract(component_id="actuator_dynamics", classification="not_exported", source="bindings/control-binding.json", notes="Source plant consumes direct algebraic control inputs."),
        ),
        runtime_artifact="runtime/aircraft.txair",
        validation_artifact="validation/source-check-cases.json",
        source_payload_external=False,
        roundtrip_status="source_preserving_pending",
    )

    files.update(
        {
            "manifest.json": manifest.canonical_json(),
            "source/source-hashes.json": _json({document.document_id: document.sha256 for document in source_documents}),
            "canonical/documents.json": _json(documents_json),
            "canonical/bindings.json": _json({"control": control_binding, "runtime": runtime_binding}),
            "canonical/authorities.json": _json([item.model_dump(mode="json") for item in manifest.contribution_authority]),
            "canonical/transforms.json": _json([item.model_dump(mode="json") for item in manifest.transforms]),
            "canonical/assumptions.json": _json(
                {
                    "source_package": package_manifest["source"],
                    "claim_boundary": family.nonclaims,
                    "package_binding_status": package_report.status,
                }
            ),
            "dynamics/taoryx-dynamics.json": _json([item.model_dump(mode="json") for item in manifest.stateful_components]),
            "validation/source-check-cases.json": _json(acceptance),
            "validation/roundtrip-report.json": _json(
                {
                    "status": "not_started",
                    "levels": {"L0": "pending", "L1": "pending", "L2": "pending", "L3": "pending", "L4": "pending"},
                    "source_package_binding": package_report.to_dict(),
                    "case_counts": {
                        role: section.get("case_count", 0)
                        for role, section in acceptance.items()
                        if isinstance(section, dict) and "case_count" in section
                    },
                }
            ),
            "schemas/collection-manifest.json": _json(CollectionManifest.model_json_schema()),
        }
    )
    archive = write_deterministic_collection(files, output)
    return {"collection": manifest.collection_id, "archive": str(archive.path), "sha256": archive.sha256, "members": archive.members}
####


def build_hl20_collection(family_manifest_path: Path, source_root: Path, output: Path) -> dict[str, Any]:
    """Build the HL-20 source-grounded collection from its verified package."""

    family = load_reference_family_manifest(family_manifest_path)
    if family.family_id != "reference_hl20_mod_k":
        raise ValueError("the HL-20 collection builder requires reference_hl20_mod_k")
    package_path = source_root / family.source.package_relative_path
    package_report = inspect_reference_package(family, package_path)

    with zipfile.ZipFile(package_path) as package:
        package_manifest = _read_json(package, "manifest.json")
        aero_binding = _read_json(package, "runtime/aerodynamic-binding.json")
        vehicle_binding = _read_json(package, "runtime/vehicle-binding.json")
        acceptance = _read_json(package, "validation/acceptance.json")
        source_documents: list[SourceDocument] = []
        files: dict[str, bytes] = {}
        for document in package_manifest["source"]["documents"]:
            member = str(document["path"])
            payload = package.read(member)
            source_documents.append(
                SourceDocument(
                    document_id=f"hl20-mod-k.{document['role']}",
                    role=str(document["role"]),
                    source_path=str(document.get("upstream_path", member)),
                    package_member=member,
                    sha256=_sha256(payload),
                    byte_identical_to_upstream=bool(document["byte_identical_to_upstream"]),
                    upstream_repository=document.get("upstream_repository"),
                    upstream_path=document.get("upstream_path"),
                )
            )
            files[f"source/{member.removeprefix('models/')}"] = payload

        files["source/package-manifest.json"] = package.read("manifest.json")
        files["source/provenance.json"] = package.read("source/provenance.json")
        files["runtime/aircraft.txair"] = package_path.read_bytes()
        files["validation/glide-hold.csv"] = package.read("validation/glide-hold.csv")
        for member in (
            "validation/acceptance-spec.json",
            "validation/check-report.json",
            "validation/glide-hold-summary.json",
            "validation/reference-evaluation.json",
            "validation/reference-glide-trim.json",
            "tables/alpha-limit.csv",
            "tables/clean-aerodynamics.csv",
            "tables/control-increments.csv",
            "tables/mass-properties.csv",
            "tables/summary.json",
        ):
            files[f"evidence/{member}"] = package.read(member)

    source_id = "hl20-mod-k.aerodynamics"
    runtime_components = {
        "aerodynamics": "runtime/aerodynamic-binding.json",
        "mass_properties": "runtime/vehicle-binding.json",
    }
    manifest = CollectionManifest(
        collection_type="taoryx.txcollection/v1alpha1",
        collection_id="hl20-mod-k",
        collection_version="1.0.0",
        family_id=family.family_id,
        canonical_authority="taoryx_canonical",
        source_documents=tuple(source_documents),
        component_bindings=(
            ComponentBinding(
                component_id="aerodynamics",
                role="aerodynamics",
                source_document_ids=(source_id,),
                inputs=("state.alpha", "state.beta", "state.body_rates", "state.mach", "state.altitude"),
                outputs=("aero.body_force_coefficients", "aero.body_moment_coefficients"),
                runtime_member=runtime_components["aerodynamics"],
            ),
            ComponentBinding(
                component_id="mass_properties",
                role="mass-properties",
                inputs=("state.configuration",),
                outputs=("mass.total", "mass.cg", "mass.inertia"),
                runtime_member=runtime_components["mass_properties"],
            ),
            ComponentBinding(
                component_id="control_mapping",
                role="control-mapping",
                source_document_ids=(source_id,),
                inputs=("control.surface_deflections",),
                outputs=("aero.control_increments",),
                runtime_member=runtime_components["aerodynamics"],
            ),
        ),
        contribution_authority=(
            ContributionAuthority(contribution="aerodynamic.force", producer="aerodynamics", status="authoritative", rationale="byte-pinned source DAVE-ML graph"),
            ContributionAuthority(contribution="aerodynamic.moment", producer="aerodynamics", status="authoritative", rationale="byte-pinned source DAVE-ML graph"),
            ContributionAuthority(contribution="mass.inertia", producer="mass_properties", status="derived", rationale="separate fixed binding recorded by the package"),
        ),
        transforms=(
            TransformRecord(transform_id="frame.body", kind="frame", source="FRD", canonical="FRD", reason="package canonical body frame"),
            TransformRecord(transform_id="frame.navigation", kind="frame", source="NED", canonical="NED", reason="package canonical navigation frame"),
            TransformRecord(transform_id="attitude.quaternion", kind="axis", source="wxyz", canonical="wxyz", reason="package quaternion order"),
        ),
        stateful_components=(
            StatefulComponentContract(component_id="equations_of_motion", classification="external_required", states=("position", "velocity", "attitude", "body_rates"), source="Taoryx runtime", notes="Static DAVE-ML supplies aerodynamic loads; runtime integrates state."),
            StatefulComponentContract(component_id="mass_properties", classification="external_exact", states=("mass", "center_of_gravity", "inertia"), source="runtime/vehicle-binding.json", notes="Fixed unpowered mass-property binding is outside the aerodynamic DAVE-ML source."),
            StatefulComponentContract(component_id="actuator_dynamics", classification="not_exported", source="package manifest", notes="Package accepts direct aerodynamic surface deflections; actuator dynamics are not claimed."),
        ),
        runtime_artifact="runtime/aircraft.txair",
        validation_artifact="validation/source-check-cases.json",
        source_payload_external=False,
        roundtrip_status="source_preserving_pending",
    )

    files.update(
        {
            "manifest.json": manifest.canonical_json(),
            "source/source-hashes.json": _json({document.document_id: document.sha256 for document in source_documents}),
            "canonical/documents.json": _json([document.model_dump(mode="json") for document in source_documents]),
            "canonical/bindings.json": _json({"aerodynamics": aero_binding, "vehicle": vehicle_binding}),
            "canonical/authorities.json": _json([item.model_dump(mode="json") for item in manifest.contribution_authority]),
            "canonical/transforms.json": _json([item.model_dump(mode="json") for item in manifest.transforms]),
            "canonical/assumptions.json": _json(
                {
                    "source_package": package_manifest["source"],
                    "claim_boundary": family.nonclaims,
                    "package_binding_status": package_report.status,
                    "mass_properties": "separate fixed binding; not contained in the aerodynamic DAVE-ML document",
                }
            ),
            "dynamics/taoryx-dynamics.json": _json([item.model_dump(mode="json") for item in manifest.stateful_components]),
            "validation/source-check-cases.json": _json(acceptance),
            "validation/roundtrip-report.json": _json(
                {
                    "status": "not_started",
                    "levels": {"L0": "pending", "L1": "pending", "L2": "pending", "L3": "pending", "L4": "pending"},
                    "source_package_binding": package_report.to_dict(),
                    "case_counts": {"aerodynamics": acceptance.get("case_count", 0)},
                }
            ),
            "schemas/collection-manifest.json": _json(CollectionManifest.model_json_schema()),
        }
    )
    archive = write_deterministic_collection(files, output)
    return {"collection": manifest.collection_id, "archive": str(archive.path), "sha256": archive.sha256, "members": archive.members}
    ####


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family-manifest", type=Path, default=DEFAULT_FAMILY)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="Output .txcollection archive")
    return parser
####


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    family_manifest = load_reference_family_manifest(args.family_manifest.resolve())
    builders = {
        "reference_f16_s119": build_f16_collection,
        "reference_hl20_mod_k": build_hl20_collection,
    }
    try:
        builder = builders[family_manifest.family_id]
    except KeyError as error:
        raise ValueError(f"no DAVE-ML collection builder registered for {family_manifest.family_id!r}") from error
    report = builder(args.family_manifest.resolve(), args.source_root.resolve(), args.output.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
####


if __name__ == "__main__":
    sys.exit(main())
