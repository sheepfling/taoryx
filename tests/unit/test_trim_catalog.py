from pathlib import Path

from taoryx.trim_catalog import load_trim_catalog


def test_repository_trim_catalog_generates_solver_specs() -> None:
    catalog = load_trim_catalog(Path("verification/trim_specs.yaml"))

    assert catalog.get("b747-condition3-trim-v1").to_spec().control_names == ("thrust_n", "elevator_deg")
    assert catalog.get("x8-powered-trim-v1").to_spec().residual_names[-1] == "yaw_moment"
    assert catalog.get("hummingbird-hover-v1").status == "ready"
    assert catalog.get("x15-release-glide-v1").status == "ready"


def test_repository_trim_catalog_exposes_common_procedure_contract() -> None:
    catalog = load_trim_catalog(Path("verification/trim_specs.yaml"))

    for identifier in (
        "b747-condition3-trim-v1",
        "x8-powered-trim-v1",
        "hummingbird-hover-v1",
        "x15-release-glide-v1",
    ):
        procedure = catalog.get(identifier).to_procedure()
        assert procedure.vehicle
        assert procedure.fidelity == "rigid_body_6dof"
        assert procedure.provenance
        assert procedure.multi_start >= 2
        assert procedure.as_dict()["variables"]["residual_names"]
