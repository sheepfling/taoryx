"""Regression coverage for the loose public-boundary survey."""

from __future__ import annotations

from pathlib import Path

from tools.audit_loose_contracts import AUDIT_SCHEMA, audit, render_markdown


def _write(path: Path, text: str) -> None:
    """Write one minimal source module for the static-audit fixture."""

    path.write_text(text, encoding="utf-8")
    ####


def test_audit_reports_consumed_configuration_bags_and_model_fields(tmp_path: Path) -> None:
    """Cross-module configuration bags rank above isolated payload records."""

    _write(
        tmp_path / "producer.py",
        """from typing import Any, Mapping

def build_configuration(config: Mapping[str, Any]) -> dict[str, object]:
    return dict(config)

class Packet:
    payload: dict[str, Any]
""",
    )
    _write(
        tmp_path / "consumer.py",
        """from producer import build_configuration

def consume() -> None:
    build_configuration({\"altitude_m\": 100.0})
""",
    )

    report = audit((tmp_path,))

    assert report["schema"] == AUDIT_SCHEMA
    assert report["status"] == "survey_complete"
    findings = report["findings"]
    assert isinstance(findings, list)
    parameter = next(item for item in findings if item["symbol"] == "build_configuration(config)")
    assert parameter["configuration_projection"] is True
    assert parameter["consumer_modules"] == ["consumer"]
    assert parameter["priority"] == "critical"
    field = next(item for item in findings if item["symbol"] == "Packet.payload")
    assert field["kind"] == "loose_model_field"

    markdown = render_markdown(report)
    assert "# Loose contract boundary audit" in markdown
    assert "build_configuration(config)" in markdown
    ####


def test_audit_ignores_generated_build_trees(tmp_path: Path) -> None:
    """Generated package copies must not double-count product boundaries."""

    _write(tmp_path / "live.py", "def live(payload: dict[str, object]) -> None:\n    return None\n")
    generated = tmp_path / "build" / "copied.py"
    generated.parent.mkdir()
    _write(generated, "def copied(payload: dict[str, object]) -> None:\n    return None\n")

    report = audit((tmp_path,))

    findings = report["findings"]
    assert isinstance(findings, list)
    assert [item["symbol"] for item in findings] == ["live(payload)"]
    ####
