"""DAVE-ML package-to-family import records.

The import record is deliberately separate from :class:`FamilyPackage`.
DAVE-ML source identity, canonical cycling, and runtime evidence are
provenance for a family; they are not provider-neutral case parameters.
"""

from __future__ import annotations

import hashlib
import json
import math
import zipfile
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field

from .daveml_atmosphere import DAVEMLAtmosphereBinding
from .daveml_evaluator import DAVEMLGraph, evaluate_daveml_checkdata, load_daveml_graph
from .daveml_replay import replay_reference_package
from .daveml_semantic import build_daveml_ir, compare_daveml_ir, compare_daveml_numeric, export_daveml_ir


class DAVEMLImportDocument(BaseModel):
    """One DAVE-ML source member and its canonical-cycle evidence."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    document_id: str = Field(min_length=1)
    role: str = Field(min_length=1)
    package_member: str = Field(min_length=1)
    package_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    catalog_document_id: str | None = None
    catalog_normalized_path: str | None = None
    catalog_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    canonical_ir_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    canonical_export_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    structural_diff_count: int = Field(default=0, ge=0)
    numeric_diff_count: int = Field(default=0, ge=0)
    checkdata_count: int = Field(default=0, ge=0)
    checkdata_passed: int = Field(default=0, ge=0)
    checkdata_failed: int = Field(default=0, ge=0)
    checkdata_unsupported: int = Field(default=0, ge=0)


class DAVEMLImportPackage(BaseModel):
    """Identity and source-member inventory for one imported package."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    path: str = Field(min_length=1)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    model_id: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    schema_version: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    frames: dict[str, str] = Field(default_factory=dict)
    reference_geometry: dict[str, float] = Field(default_factory=dict)
    validity_envelope: dict[str, float] = Field(default_factory=dict)
    runtime_members: tuple[str, ...] = ()
    source_documents: tuple[DAVEMLImportDocument, ...]


class DAVEMLReplayEvidence(BaseModel):
    """Evidence from the Taoryx runtime load/replay seam."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    status: str = Field(min_length=1)
    source_evaluation: str = Field(min_length=1)
    runtime_load_contract: str = Field(min_length=1)
    hold_evidence: str = Field(min_length=1)
    force_moment_residual: float = Field(ge=0.0)


class DAVEMLRoundtripEvidence(BaseModel):
    """Structural and numeric evidence for canonical source cycling."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    status: Literal["verified", "failed"]
    document_count: int = Field(ge=1)
    structural_diff_count: int = Field(default=0, ge=0)
    numeric_diff_count: int = Field(default=0, ge=0)
    checkdata_status: Literal["not_present", "verified", "verified_with_quarantine", "failed"] = "not_present"
    checkdata_count: int = Field(default=0, ge=0)
    checkdata_passed: int = Field(default=0, ge=0)
    checkdata_failed: int = Field(default=0, ge=0)
    checkdata_unsupported: int = Field(default=0, ge=0)


class DAVEMLFamilyImport(BaseModel):
    """Auditable incorporation record attached to a reference family."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(default=1, ge=1)
    kind: Literal["taoryx.daveml-family-import/v1alpha1"]
    family_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    catalog_id: str = Field(min_length=1)
    catalog_root: str = Field(min_length=1)
    source_authority: str = Field(min_length=1)
    package: DAVEMLImportPackage
    roundtrip: DAVEMLRoundtripEvidence
    replay: DAVEMLReplayEvidence
    claims: tuple[str, ...] = ()
    nonclaims: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class DAVEMLFamilyGraphBinding:
    """Hash-verified graph execution seam for one family component."""

    family_id: str
    model_id: str
    role: str
    package_path: Path
    package_sha256: str
    package_member: str
    document_sha256: str
    graph: DAVEMLGraph

    def evaluate(self, inputs: Mapping[str, float], outputs: Sequence[str]) -> dict[str, float]:
        """Evaluate outputs without bypassing the imported source identity."""

        return self.graph.evaluate(inputs, outputs)
        ####

    def unit_for(self, identifier: str) -> str | None:
        """Return the declared source unit for a bound graph variable."""

        return self.graph.unit_for(identifier)
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLFunctionChannel:
    """Typed channel adapter for one hash-verified DAVE-ML function."""

    graph: DAVEMLFamilyGraphBinding
    function_id: str
    input_channels: Mapping[str, str]
    output_channel: str

    def __post_init__(self) -> None:
        if self.function_id not in self.graph.graph.functions:
            raise ValueError(
                f"DAVE-ML function {self.function_id!r} is not present in "
                f"{self.graph.family_id!r}/{self.graph.role!r}"
            )
        if not self.input_channels:
            raise ValueError("DAVE-ML function channel requires at least one input")
        channel_names = tuple(self.input_channels)
        if len(channel_names) != len(set(channel_names)) or not all(channel_names):
            raise ValueError("DAVE-ML function input channel names must be unique and non-empty")
        source_ids = tuple(self.input_channels.values())
        if len(source_ids) != len(set(source_ids)) or not all(source_ids):
            raise ValueError("DAVE-ML function source inputs must be unique and non-empty")
        if self.output_channel not in self.graph.graph.variables:
            raise ValueError(f"DAVE-ML function output variable {self.output_channel!r} is unresolved")
        missing = [source_id for source_id in source_ids if source_id not in self.graph.graph.variables]
        if missing:
            raise ValueError("DAVE-ML function input variables are unresolved: " + ", ".join(sorted(missing)))
        ####

    def evaluate(self, channels: Mapping[str, float]) -> float:
        """Evaluate the source function from named family-library channels."""

        missing = set(self.input_channels) - set(channels)
        if missing:
            raise KeyError("DAVE-ML function channels are missing: " + ", ".join(sorted(missing)))
        values = {
            source_id: float(channels[channel])
            for channel, source_id in self.input_channels.items()
        }
        return self.graph.evaluate(values, (self.output_channel,))[self.output_channel]
        ####

    def input_unit(self, channel: str) -> str | None:
        """Return the source unit for one public input channel."""

        try:
            source_id = self.input_channels[channel]
        except KeyError as error:
            raise KeyError(f"unknown DAVE-ML function channel {channel!r}") from error
        return self.graph.unit_for(source_id)
        ####

    def output_unit(self) -> str | None:
        """Return the declared source unit for the output channel."""

        return self.graph.unit_for(self.output_channel)
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLTrimBinding:
    """Explicit DAVE-ML input/output mapping usable by the trim solver."""

    graph: DAVEMLFamilyGraphBinding
    state_inputs: Mapping[str, str]
    control_inputs: Mapping[str, str]
    residual_outputs: Mapping[str, str]
    fixed_inputs: Mapping[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.residual_outputs:
            raise ValueError("DAVE-ML trim binding requires at least one residual output")
        names = (*self.state_inputs, *self.control_inputs)
        if len(names) != len(set(names)):
            raise ValueError("DAVE-ML trim state and control inputs must be unique")
        if not all(str(name).strip() for name in self.residual_outputs):
            raise ValueError("DAVE-ML trim residual names must not be empty")
        ####

    def evaluate(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float] | None = None,
    ) -> dict[str, float]:
        """Evaluate mapped residual outputs for one operating-point query."""

        inputs = {str(name): float(value) for name, value in self.fixed_inputs.items()}
        if environment is not None:
            inputs.update({str(name): float(value) for name, value in environment.items()})
        _map_trim_inputs(inputs, state, self.state_inputs, "state")
        _map_trim_inputs(inputs, controls, self.control_inputs, "control")
        output_ids = tuple(self.residual_outputs.values())
        values = self.graph.evaluate(inputs, output_ids)
        return {residual: values[output_id] for residual, output_id in self.residual_outputs.items()}
        ####

    def as_evaluator(
        self,
        environment: Mapping[str, float] | None = None,
    ) -> Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]:
        """Return the callable shape accepted by :func:`taoryx.trim.solve_trim`."""

        return lambda state, controls: self.evaluate(state, controls, environment)
        ####

    ####


@dataclass(frozen=True, slots=True)
class DAVEMLCompositeTrimBinding:
    """Compose independent DAVE-ML component bindings for one trim query."""

    bindings: tuple[DAVEMLTrimBinding, ...]

    def __post_init__(self) -> None:
        if not self.bindings:
            raise ValueError("DAVE-ML composite trim binding requires at least one component")
        residuals = [name for binding in self.bindings for name in binding.residual_outputs]
        if len(residuals) != len(set(residuals)):
            raise ValueError("DAVE-ML composite trim residual names must be unique")
        ####

    def evaluate(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float] | None = None,
    ) -> dict[str, float]:
        """Evaluate every component and merge its named residual channels."""

        values: dict[str, float] = {}
        for binding in self.bindings:
            values.update(binding.evaluate(state, controls, environment))
        return values
        ####

    def as_evaluator(
        self,
        environment: Mapping[str, float] | None = None,
    ) -> Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]:
        """Return the callable shape accepted by :func:`taoryx.trim.solve_trim`."""

        return lambda state, controls: self.evaluate(state, controls, environment)
        ####

    ####


@dataclass(frozen=True, slots=True)
class DAVEMLFixedWingLoadBinding:
    """Convert source coefficient channels into explicit body loads in SI."""

    aerodynamics: DAVEMLTrimBinding
    reference_area_m2: float
    mean_aerodynamic_chord_m: float
    span_m: float
    dynamic_pressure_pa: float
    propulsion: DAVEMLTrimBinding | None = None
    propulsion_output: str = "thrust_lbf"
    thrust_scale_to_newtons: float = 4.4482216152605

    def __post_init__(self) -> None:
        values = (
            self.reference_area_m2,
            self.mean_aerodynamic_chord_m,
            self.span_m,
            self.dynamic_pressure_pa,
            self.thrust_scale_to_newtons,
        )
        if not all(float(value) > 0.0 for value in values):
            raise ValueError("DAVE-ML fixed-wing load geometry and scales must be positive")
        required = {"cx", "cy", "cz", "cl", "cm", "cn"}
        missing = required - set(self.aerodynamics.residual_outputs)
        if missing:
            raise ValueError("DAVE-ML aerodynamic load channels are missing: " + ", ".join(sorted(missing)))
        if self.propulsion is not None and self.propulsion_output not in self.propulsion.residual_outputs:
            raise ValueError(f"DAVE-ML propulsion output {self.propulsion_output!r} is unresolved")
        ####

    def evaluate(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float] | None = None,
    ) -> dict[str, float]:
        """Return force/moment channels without applying gravity or mass balance."""

        coefficients = self.aerodynamics.evaluate(state, controls, environment)
        scale = self.dynamic_pressure_pa * self.reference_area_m2
        loads = {
            "aerodynamic_force_x_n": scale * coefficients["cx"],
            "aerodynamic_force_y_n": scale * coefficients["cy"],
            "aerodynamic_force_z_n": scale * coefficients["cz"],
            "aerodynamic_moment_x_nm": scale * self.span_m * coefficients["cl"],
            "aerodynamic_moment_y_nm": scale * self.mean_aerodynamic_chord_m * coefficients["cm"],
            "aerodynamic_moment_z_nm": scale * self.span_m * coefficients["cn"],
        }
        if self.propulsion is not None:
            thrust = self.propulsion.evaluate(state, controls, environment)[self.propulsion_output]
            loads["propulsion_force_x_n"] = self.thrust_scale_to_newtons * thrust
        else:
            loads["propulsion_force_x_n"] = 0.0
        loads["total_force_x_n"] = loads["aerodynamic_force_x_n"] + loads["propulsion_force_x_n"]
        loads["total_force_y_n"] = loads["aerodynamic_force_y_n"]
        loads["total_force_z_n"] = loads["aerodynamic_force_z_n"]
        loads["total_moment_x_nm"] = loads["aerodynamic_moment_x_nm"]
        loads["total_moment_y_nm"] = loads["aerodynamic_moment_y_nm"]
        loads["total_moment_z_nm"] = loads["aerodynamic_moment_z_nm"]
        return loads
        ####

    def as_evaluator(
        self,
        environment: Mapping[str, float] | None = None,
    ) -> Callable[[Mapping[str, float], Mapping[str, float]], Mapping[str, float]]:
        """Return a trim-solver-compatible total-load evaluator."""

        return lambda state, controls: self.evaluate(state, controls, environment)
        ####

    def evaluate_with_atmosphere(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        atmosphere: DAVEMLAtmosphereBinding,
        *,
        geometric_altitude_m: float,
        true_airspeed_m_s: float,
        environment: Mapping[str, float] | None = None,
    ) -> dict[str, float]:
        """Evaluate loads using atmosphere-derived dynamic pressure."""

        if not math.isfinite(true_airspeed_m_s) or true_airspeed_m_s <= 0.0:
            raise ValueError("true airspeed must be finite and positive")
        properties = atmosphere.evaluate(geometric_altitude_m)
        dynamic_pressure = 0.5 * properties["density_kg_m3"] * true_airspeed_m_s**2
        return DAVEMLFixedWingLoadBinding(
            aerodynamics=self.aerodynamics,
            reference_area_m2=self.reference_area_m2,
            mean_aerodynamic_chord_m=self.mean_aerodynamic_chord_m,
            span_m=self.span_m,
            dynamic_pressure_pa=dynamic_pressure,
            propulsion=self.propulsion,
            propulsion_output=self.propulsion_output,
            thrust_scale_to_newtons=self.thrust_scale_to_newtons,
        ).evaluate(state, controls, environment)
        ####
    ####


@dataclass(frozen=True, slots=True)
class DAVEMLLiftingBodyLoadBinding:
    """Convert wind-axis lifting-body coefficients into explicit body loads."""

    aerodynamics: DAVEMLTrimBinding
    reference_area_m2: float
    mean_aerodynamic_chord_m: float
    dynamic_pressure_pa: float

    def __post_init__(self) -> None:
        values = (self.reference_area_m2, self.mean_aerodynamic_chord_m, self.dynamic_pressure_pa)
        if not all(float(value) > 0.0 for value in values):
            raise ValueError("DAVE-ML lifting-body load geometry and pressure must be positive")
        missing = {"cl", "cd", "cm"} - set(self.aerodynamics.residual_outputs)
        if missing:
            raise ValueError("DAVE-ML lifting-body channels are missing: " + ", ".join(sorted(missing)))
        ####

    def evaluate(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        environment: Mapping[str, float] | None = None,
    ) -> dict[str, float]:
        """Return wind-axis loads mapped to body X/Z and pitch moment."""

        coefficients = self.aerodynamics.evaluate(state, controls, environment)
        scale = self.dynamic_pressure_pa * self.reference_area_m2
        pitch = scale * self.mean_aerodynamic_chord_m * coefficients["cm"]
        return {
            "lift_n": scale * coefficients["cl"],
            "drag_n": scale * coefficients["cd"],
            "pitch_moment_nm": pitch,
            "body_force_x_n": -scale * coefficients["cd"],
            "body_force_y_n": 0.0,
            "body_force_z_n": -scale * coefficients["cl"],
            "body_moment_x_nm": 0.0,
            "body_moment_y_nm": pitch,
            "body_moment_z_nm": 0.0,
        }
        ####

    def evaluate_with_atmosphere(
        self,
        state: Mapping[str, float],
        controls: Mapping[str, float],
        atmosphere: DAVEMLAtmosphereBinding,
        *,
        geometric_altitude_m: float,
        true_airspeed_m_s: float,
        environment: Mapping[str, float] | None = None,
    ) -> dict[str, float]:
        """Evaluate lifting-body loads with atmosphere-derived dynamic pressure."""

        if not math.isfinite(true_airspeed_m_s) or true_airspeed_m_s <= 0.0:
            raise ValueError("true airspeed must be finite and positive")
        density = atmosphere.evaluate(geometric_altitude_m)["density_kg_m3"]
        return DAVEMLLiftingBodyLoadBinding(
            self.aerodynamics,
            self.reference_area_m2,
            self.mean_aerodynamic_chord_m,
            0.5 * density * true_airspeed_m_s**2,
        ).evaluate(state, controls, environment)
        ####
    ####


def load_daveml_family_import(path: str | Path) -> DAVEMLFamilyImport:
    """Load and validate one family import sidecar."""

    source = Path(path)
    try:
        payload: Any = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"DAVE-ML family import cannot be read: {source}: {error}") from error
    try:
        return DAVEMLFamilyImport.model_validate(payload)
    except ValueError as error:
        raise ValueError(f"invalid DAVE-ML family import {source}: {error}") from error


def load_daveml_family_graph(path: str | Path, *, role: str) -> DAVEMLFamilyGraphBinding:
    """Load one source graph after verifying its package and member hashes."""

    sidecar = Path(path)
    record = load_daveml_family_import(sidecar)
    document = next((item for item in record.package.source_documents if item.role == role), None)
    if document is None:
        raise ValueError(f"family {record.family_id!r} has no DAVE-ML source role {role!r}")
    package_path = _resolve_package_path(sidecar, record.package.path)
    package_bytes = package_path.read_bytes()
    package_sha256 = _sha256(package_bytes)
    if package_sha256 != record.package.sha256:
        raise ValueError(
            f"DAVE-ML package hash drift for {record.family_id!r}: "
            f"expected {record.package.sha256}, got {package_sha256}"
        )
    with zipfile.ZipFile(package_path) as archive:
        try:
            payload = archive.read(document.package_member)
        except KeyError as error:
            raise ValueError(f"DAVE-ML source member is missing: {document.package_member}") from error
    document_sha256 = _sha256(payload)
    if document_sha256 != document.package_sha256:
        raise ValueError(
            f"DAVE-ML source hash drift for {record.family_id!r}/{role!r}: "
            f"expected {document.package_sha256}, got {document_sha256}"
        )
    return DAVEMLFamilyGraphBinding(
        family_id=record.family_id,
        model_id=record.model_id,
        role=role,
        package_path=package_path,
        package_sha256=package_sha256,
        package_member=document.package_member,
        document_sha256=document_sha256,
        graph=load_daveml_graph(payload, document_id=document.document_id),
    )


def load_daveml_function_channel(
    path: str | Path,
    *,
    role: str,
    function_id: str,
    input_channels: Mapping[str, str],
    output_channel: str | None = None,
) -> DAVEMLFunctionChannel:
    """Load a typed function channel while preserving package provenance."""

    graph = load_daveml_family_graph(path, role=role)
    return DAVEMLFunctionChannel(
        graph=graph,
        function_id=function_id,
        input_channels=dict(input_channels),
        output_channel=output_channel or function_id,
    )


def load_daveml_trim_binding(
    path: str | Path,
    *,
    role: str,
    state_inputs: Mapping[str, str],
    control_inputs: Mapping[str, str],
    residual_outputs: Mapping[str, str],
    fixed_inputs: Mapping[str, float] | None = None,
) -> DAVEMLTrimBinding:
    """Load a solver-facing trim binding with package provenance checks."""

    return DAVEMLTrimBinding(
        graph=load_daveml_family_graph(path, role=role),
        state_inputs=dict(state_inputs),
        control_inputs=dict(control_inputs),
        residual_outputs=dict(residual_outputs),
        fixed_inputs={} if fixed_inputs is None else dict(fixed_inputs),
    )


def build_daveml_family_import(
    package_path: str | Path,
    *,
    family_id: str,
    catalog_root: str | Path,
    claims: tuple[str, ...] = (),
    nonclaims: tuple[str, ...] = (),
    package_path_label: str | None = None,
) -> DAVEMLFamilyImport:
    """Build an import record from one qualified package and catalog."""

    package_file = Path(package_path)
    catalog = Path(catalog_root)
    catalog_id, catalog_documents = _catalog_documents(catalog)
    package_bytes = package_file.read_bytes()
    package_sha256 = _sha256(package_bytes)
    with zipfile.ZipFile(package_file) as archive:
        files = {name: archive.read(name) for name in archive.namelist() if not name.endswith("/")}
    manifest = _json_object(files, "manifest.json")
    package_documents: list[DAVEMLImportDocument] = []
    source_entries = manifest.get("source", {}).get("documents", [])
    if not isinstance(source_entries, list):
        raise ValueError(f"package source documents must be a list: {package_file}")
    for entry in source_entries:
        if not isinstance(entry, dict):
            raise ValueError(f"package source document must be an object: {package_file}")
        member = str(entry.get("path", ""))
        if not member or member not in files:
            raise ValueError(f"package source member is missing: {member}")
        payload = files[member]
        if not member.casefold().endswith((".dml", ".xml")):
            continue
        document_id = f"{manifest.get('model_id', package_file.stem)}.{entry.get('role', 'source')}"
        ir = build_daveml_ir(payload, document_id=document_id)
        exported = export_daveml_ir(ir)
        structural = compare_daveml_ir(ir, build_daveml_ir(exported, document_id=document_id))
        numeric = compare_daveml_numeric(ir, build_daveml_ir(exported, document_id=document_id))
        checks = evaluate_daveml_checkdata(payload)
        catalog_match = _match_catalog_document(entry, catalog_documents)
        package_documents.append(
            DAVEMLImportDocument(
                document_id=document_id,
                role=str(entry.get("role", "source")),
                package_member=member,
                package_sha256=_sha256(payload),
                catalog_document_id=catalog_match.get("document_id"),
                catalog_normalized_path=catalog_match.get("normalized_path"),
                catalog_sha256=catalog_match.get("document_sha256"),
                canonical_ir_sha256=_sha256(ir.canonical_json()),
                canonical_export_sha256=_sha256(exported),
                structural_diff_count=len(structural),
                numeric_diff_count=len(numeric),
                checkdata_count=len(checks),
                checkdata_passed=sum(item.status == "passed" for item in checks),
                checkdata_failed=sum(item.status == "failed" for item in checks),
                checkdata_unsupported=sum(item.status == "unsupported" for item in checks),
            )
        )
    if not package_documents:
        raise ValueError(f"package has no DAVE-ML source documents: {package_file}")

    replay = replay_reference_package(package_file)
    structural_count = sum(item.structural_diff_count for item in package_documents)
    numeric_count = sum(item.numeric_diff_count for item in package_documents)
    checkdata_count = sum(item.checkdata_count for item in package_documents)
    checkdata_passed = sum(item.checkdata_passed for item in package_documents)
    checkdata_failed = sum(item.checkdata_failed for item in package_documents)
    checkdata_unsupported = sum(item.checkdata_unsupported for item in package_documents)
    package_manifest_id = str(manifest.get("model_id", ""))
    return DAVEMLFamilyImport(
        kind="taoryx.daveml-family-import/v1alpha1",
        family_id=family_id,
        model_id=package_manifest_id,
        catalog_id=catalog_id,
        catalog_root=catalog.as_posix(),
        source_authority=str(manifest.get("source", {}).get("source_kind", "daveml")),
        package=DAVEMLImportPackage(
            path=package_path_label or package_file.as_posix(),
            sha256=package_sha256,
            model_id=package_manifest_id,
            display_name=str(manifest.get("display_name", package_manifest_id)),
            schema_version=str(manifest.get("schema_version", "")),
            fidelity=str(manifest.get("fidelity", "")),
            frames=_string_mapping(manifest.get("frames", {})),
            reference_geometry=_number_mapping(manifest.get("reference_geometry", {})),
            validity_envelope=_number_mapping(manifest.get("validity_envelope", {})),
            runtime_members=tuple(
                sorted(
                    str(artifact.get("path"))
                    for artifact in manifest.get("artifacts", [])
                    if isinstance(artifact, dict) and str(artifact.get("path", "")).startswith("runtime/")
                )
            ),
            source_documents=tuple(package_documents),
        ),
        roundtrip=DAVEMLRoundtripEvidence(
            status="verified" if structural_count == 0 and numeric_count == 0 else "failed",
            document_count=len(package_documents),
            structural_diff_count=structural_count,
            numeric_diff_count=numeric_count,
            checkdata_status=_checkdata_status(checkdata_count, checkdata_failed, checkdata_unsupported),
            checkdata_count=checkdata_count,
            checkdata_passed=checkdata_passed,
            checkdata_failed=checkdata_failed,
            checkdata_unsupported=checkdata_unsupported,
        ),
        replay=DAVEMLReplayEvidence(
            status=replay.status,
            source_evaluation=replay.source_evaluation,
            runtime_load_contract=replay.runtime_load_contract,
            hold_evidence=replay.hold_evidence,
            force_moment_residual=replay.force_moment_residual,
        ),
        claims=claims,
        nonclaims=nonclaims,
    )


def _catalog_documents(catalog_root: Path) -> tuple[str, tuple[dict[str, str], ...]]:
    """Read catalog identity and normalized-source index rows."""

    release = _read_json_file(catalog_root / "release-manifest.json")
    rows = _read_json_file(catalog_root / "catalog/source-documents.json")
    if not isinstance(rows, list):
        raise ValueError("catalog/source-documents.json must contain a list")
    normalized: list[dict[str, str]] = []
    for row in rows:
        if isinstance(row, dict):
            normalized.append({key: str(value) for key, value in row.items() if value is not None})
    return str(release.get("catalog_id", release.get("artifact", ""))), tuple(normalized)


def _match_catalog_document(entry: dict[str, Any], rows: tuple[dict[str, str], ...]) -> dict[str, str]:
    """Match a package source by its upstream path without asserting byte identity."""

    upstream_path = str(entry.get("upstream_path", ""))
    source_name = Path(upstream_path).name
    candidates = [row for row in rows if Path(row.get("source_path", "")).name == source_name]
    if len(candidates) == 1:
        return candidates[0]
    return {}


def _read_json_file(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid DAVE-ML catalog JSON: {path}: {error}") from error


def _json_object(files: dict[str, bytes], name: str) -> dict[str, Any]:
    for member, payload in files.items():
        if Path(member).name.casefold() == name.casefold():
            value = json.loads(payload)
            if isinstance(value, dict):
                return value
            break
    raise ValueError(f"package is missing JSON object {name!r}")


def _string_mapping(value: Any) -> dict[str, str]:
    return {str(key): str(item) for key, item in value.items()} if isinstance(value, dict) else {}


def _number_mapping(value: Any) -> dict[str, float]:
    if not isinstance(value, dict):
        return {}
    return {str(key): float(item) for key, item in value.items()}


def _map_trim_inputs(
    destination: dict[str, float],
    values: Mapping[str, float],
    mapping: Mapping[str, str],
    kind: str,
) -> None:
    """Map named trim channels to source graph inputs without inventing defaults."""

    for trim_name, graph_input in mapping.items():
        if trim_name not in values:
            raise KeyError(f"DAVE-ML trim {kind} input is missing: {trim_name}")
        destination[graph_input] = float(values[trim_name])


def _checkdata_status(count: int, failed: int, unsupported: int) -> Literal["not_present", "verified", "verified_with_quarantine", "failed"]:
    """Classify aggregate package check-data evidence."""

    if count == 0:
        return "not_present"
    if failed:
        return "failed"
    if unsupported:
        return "verified_with_quarantine"
    return "verified"


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _resolve_package_path(sidecar: Path, package_label: str) -> Path:
    """Resolve a repository-relative package label without trusting cwd alone."""

    candidate = Path(package_label)
    if candidate.is_absolute() and candidate.is_file():
        return candidate
    roots = [Path.cwd(), sidecar.parent, *sidecar.resolve().parents]
    seen: set[Path] = set()
    for root in roots:
        resolved_root = root.resolve()
        if resolved_root in seen:
            continue
        seen.add(resolved_root)
        resolved = (resolved_root / candidate).resolve()
        if resolved.is_file():
            return resolved
    raise ValueError(f"DAVE-ML package cannot be resolved from {sidecar}: {package_label}")


__all__ = [
    "DAVEMLFamilyImport",
    "DAVEMLFamilyGraphBinding",
    "DAVEMLCompositeTrimBinding",
    "DAVEMLFixedWingLoadBinding",
    "DAVEMLLiftingBodyLoadBinding",
    "DAVEMLFunctionChannel",
    "DAVEMLImportDocument",
    "DAVEMLImportPackage",
    "DAVEMLReplayEvidence",
    "DAVEMLRoundtripEvidence",
    "build_daveml_family_import",
    "load_daveml_family_import",
    "load_daveml_family_graph",
    "load_daveml_function_channel",
    "load_daveml_trim_binding",
]
