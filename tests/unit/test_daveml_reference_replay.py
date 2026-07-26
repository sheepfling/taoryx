from __future__ import annotations

import zipfile
from pathlib import Path

import pytest

from taoryx.trajectory import replay_reference_package

CORPUS = Path(__file__).parents[2] / "resources/aerospace/daveml/taoryx-corpus-v1.1/corpus.zip"


def _extract_package(tmp_path: Path, member: str) -> Path:
    with zipfile.ZipFile(CORPUS) as corpus:
        payload = corpus.read(member)
    path = tmp_path / Path(member).name
    path.write_bytes(payload)
    return path
    ####


@pytest.mark.parametrize(
    ("member", "model_id"),
    (
        (
            "taoryx-aerospace-data-corpus-v1.1/qualified-models/f16-s119/taoryx-f16-s119-reference-v0.7.txair",
            "f16-s119-reference",
        ),
        (
            "taoryx-aerospace-data-corpus-v1.1/qualified-models/hl20-mod-k/taoryx-hl20-mod-k-unpowered-v0.10.txair",
            "hl20-mod-k-unpowered-6dof",
        ),
    ),
)
def test_verified_reference_package_replays_through_rigid_body_contract(tmp_path: Path, member: str, model_id: str) -> None:
    """Both real Alpha 3 packages pass the local runtime boundary."""

    report = replay_reference_package(_extract_package(tmp_path, member))

    assert report.model_id == model_id
    assert report.fidelity == "6dof"
    assert report.source_evaluation.startswith("verified_pinned_package")
    assert report.runtime_load_contract == "taoryx_rigid_body_6dof"
    assert report.hold_evidence.startswith("validation/")
    assert report.force_moment_residual == 0.0
    assert report.status == "runtime_replay_qualification_passed"
    ####


def test_reference_package_rejects_tampered_ledger(tmp_path: Path) -> None:
    """Runtime replay cannot consume a package whose evidence was altered."""

    source = _extract_package(
        tmp_path,
        "taoryx-aerospace-data-corpus-v1.1/qualified-models/f16-s119/taoryx-f16-s119-reference-v0.7.txair",
    )
    tampered = tmp_path / "tampered.txair"
    with zipfile.ZipFile(source) as package, zipfile.ZipFile(tampered, "w") as output:
        for info in package.infolist():
            payload = package.read(info.filename)
            if info.filename == "validation/acceptance.json":
                payload += b" "
            output.writestr(info, payload)

    with pytest.raises(ValueError, match="checksum mismatch"):
        replay_reference_package(tampered)
    ####


def test_nesc_two_stage_rocket_replays_qualified_benchmark_evidence() -> None:
    """The NESC rocket uses benchmark/schedule evidence instead of trim hold."""

    package = Path(__file__).parents[2] / (
        "INBOX/taoryx-daveml-nesc-model-catalog-v1.0/qualified/"
        "nesc-two-stage-rocket/nesc-two-stage-rocket-v0.9.txair"
    )

    report = replay_reference_package(package)

    assert report.model_id == "nasa-nesc-two-stage-rocket-scenario17"
    assert report.fidelity == "6dof"
    assert report.hold_evidence == "validation/acceptance.json#passed"
    assert report.force_moment_residual == 0.0
    assert report.status == "runtime_replay_qualification_passed"
