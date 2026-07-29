from taoryx.vehicle_effectivity_preflight import (
    validate_vehicle_effectivity_preflight,
)


def test_f16_effectivity_screen_reads_sample_matrix() -> None:
    report = validate_vehicle_effectivity_preflight("reference_f16_s119")

    assert report.status == "development"
    assert report.metrics["matrix_rows"] == 4
    assert report.metrics["matrix_columns"] == 4
    assert report.metrics["computed_rank"] == 4
    assert report.metrics["matrix_source"] == "sample"
    assert report.metrics["condition_number"] > 1.0
    assert report.metrics["bounded_replay"]["status"] == "feasible"
    assert report.metrics["bounded_replay"]["residual_norm"] < 1.0e-9
    assert report.metrics["sign_probe"]["elevator_deg"]["positive_axes"] == ["total_force_x_n"]
    assert any(item.code == "effectivity-development-screen" for item in report.findings)


def test_hl20_effectivity_screen_preserves_synthetic_overlay_boundary() -> None:
    report = validate_vehicle_effectivity_preflight("reference_hl20_mod_k")

    assert report.status == "development"
    assert "families/reference_hl20_mod_k/overlays/logical-surface-allocation-v1.yaml" in report.evidence
    assert any(item.code == "synthetic-effectivity-overlay" for item in report.findings)
    assert any(item.code == "allocation-evidence-not-numeric" for item in report.findings)
