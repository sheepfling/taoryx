"""Focused vertical proofs for registered non-physical Mission Composition endpoints."""

from __future__ import annotations

from typing import cast

import pytest

from taoryx.mission_workflow_endpoint import (
    load_mission_workflow_endpoint_catalog,
    mission_workflow_endpoint_list,
    verify_mission_workflow_endpoint,
)
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.runtime.cli import main


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover the installed distributions once for the endpoint proofs."""

    return discover_plugins(include_external=False)
    ####


def test_workflow_endpoint_catalog_exposes_exact_nonphysical_boundaries() -> None:
    """The workflow catalog remains distinct from the physical vehicle registry."""

    catalog = load_mission_workflow_endpoint_catalog()
    assert [item.id for item in catalog.endpoints] == [
        "simple-aero-fixed-ld-batch",
        "dual-launch-attached-booster-batch",
        "reference-ballistic-3dof-batch",
        "reference-waypoint-3dof-batch",
        "debug-contract-probe-batch",
    ]
    assert {item.model_kind for item in catalog.endpoints} == {
        "trajectory_workflow",
        "composition_proof_family",
        "ballistic_3dof",
        "constant_velocity_waypoint_3dof",
        "contract_probe",
    }
    assert all(item.robustness_disposition == "not_applicable" for item in catalog.endpoints)
    assert catalog.endpoint("simple-aero-fixed-ld-batch").blocked_operations == ()
    assert catalog.endpoint("dual-launch-attached-booster-batch").blocked_operations == ("step",)
    listing = mission_workflow_endpoint_list()
    listed_endpoints = cast(list[dict[str, object]], listing["endpoints"])
    assert [item["id"] for item in listed_endpoints] == [item.id for item in catalog.endpoints]
    ####


def test_workflow_endpoint_model_kind_remains_provider_extensible() -> None:
    """A new plug-in kind is verified against its provider, not a core allowlist."""

    endpoint = load_mission_workflow_endpoint_catalog().endpoint("debug-contract-probe-batch")
    extended = type(endpoint).model_validate({**endpoint.model_dump(mode="json"), "model_kind": "example.provider_owned_taxonomy"})

    assert extended.model_kind == "example.provider_owned_taxonomy"
    ####


def test_workflow_endpoint_cli_exposes_and_verifies_the_public_contract(
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Agents can discover and verify the same workflow seam without a custom script."""

    assert main(["model", "endpoint-specs"]) == 0
    listing = capsys.readouterr().out
    assert "simple-aero-fixed-ld-batch" in listing
    assert "dual-launch-attached-booster-batch" in listing
    assert "reference-ballistic-3dof-batch" in listing
    assert "reference-waypoint-3dof-batch" in listing
    assert "debug-contract-probe-batch" in listing

    assert main(["model", "verify", "simple-aero-fixed-ld-batch"]) == 0
    report = capsys.readouterr().out
    assert '"status": "pass"' in report
    assert '"execution_requested": false' in report
    ####


@pytest.mark.parametrize(
    "endpoint_id",
    (
        "simple-aero-fixed-ld-batch",
        "dual-launch-attached-booster-batch",
        "reference-ballistic-3dof-batch",
        "reference-waypoint-3dof-batch",
        "debug-contract-probe-batch",
    ),
)
def test_workflow_endpoints_compile_and_preflight_through_the_installed_provider(
    endpoint_id: str,
    plugins: PluginCatalog,
) -> None:
    """Each witness compiles through the provider that published its advertisement."""

    report = verify_mission_workflow_endpoint(endpoint_id, plugins=plugins)

    assert report["status"] == "pass", report
    records = report["records"]
    assert records["draft"]["status"] == "pass"  # type: ignore[index]
    assert records["advertisement"]["status"] == "pass"  # type: ignore[index]
    assert records["configuration_preflight"]["status"] == "pass"  # type: ignore[index]
    assert records["runner"]["status"] == "pass"  # type: ignore[index]
    assert records["execution"]["status"] == "not_requested"  # type: ignore[index]
    ####


@pytest.mark.parametrize(
    ("endpoint_id", "expected_object_count"),
    (
        ("simple-aero-fixed-ld-batch", 1),
        ("dual-launch-attached-booster-batch", 1),
        ("reference-ballistic-3dof-batch", 1),
        ("reference-waypoint-3dof-batch", 1),
        ("debug-contract-probe-batch", 3),
    ),
)
def test_workflow_endpoints_emit_their_declared_normalized_result_surface(
    endpoint_id: str,
    expected_object_count: int,
    plugins: PluginCatalog,
) -> None:
    """The real provider runner emits every required output and lifecycle event."""

    report = verify_mission_workflow_endpoint(endpoint_id, execute=True, plugins=plugins)

    assert report["status"] == "pass", report
    execution = report["records"]["execution"]  # type: ignore[index]
    assert execution["status"] == "pass"
    assert execution["result_status"] == "completed"
    assert execution["object_count"] == expected_object_count
    assert execution["missing_required_output_ids"] == []
    assert execution["missing_required_events"] == []
    ####
