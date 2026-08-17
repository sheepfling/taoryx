"""Regression coverage for evidence-bounded plug-in vehicle model cards."""

from __future__ import annotations

import json
from pathlib import Path

from taoryx.model_overview import build_model_overview_catalog, render_model_overview_markdown
from taoryx.plugins import discover_plugins
from taoryx.runtime.cli import main

PROVIDER_ID = "taoryx.hummingbird.mission-composition"
MODEL_ID = "hummingbird"


def _hummingbird_card() -> dict[str, object]:
    """Build one focused card through the Hummingbird package boundary."""

    plugins = discover_plugins(include_external=False, selected=("taoryx.hummingbird",))
    providers = plugins.build_mission_composition_provider_registry()
    campaigns = plugins.build_controller_tuning_campaign_registry()
    overview = build_model_overview_catalog(
        plugins,
        providers,
        campaigns,
        provider_id=PROVIDER_ID,
        model_id=MODEL_ID,
    )
    provider_cards = overview["providers"]
    assert isinstance(provider_cards, list)
    assert len(provider_cards) == 1
    models = provider_cards[0]["models"]
    assert isinstance(models, list)
    assert len(models) == 1
    return models[0]
    ####


def test_model_overview_joins_fidelity_provenance_tuning_and_segment_parameters() -> None:
    """A card must expose the facts needed to assess one vehicle honestly."""

    card = _hummingbird_card()

    identity = card["identity"]
    assert identity["provider_id"] == PROVIDER_ID
    assert identity["id"] == MODEL_ID

    fidelity_ladder = card["fidelity_ladder"]
    assert isinstance(fidelity_ladder, list)
    pseudo = next(item for item in fidelity_ladder if item["id"] == "pseudo_6dof")
    assert pseudo["control_realization"] == "response_law"
    assert pseudo["promotion_status"] == "development"
    assert pseudo["operations"] == ["validate", "batch", "step"]

    provenance = card["provenance"]
    assert isinstance(provenance, dict)
    records = provenance["records"]
    assert isinstance(records, list)
    model_record = next(item for item in records if item["scope"] == "model")
    assert "verification/vehicle_composition_registry.yaml" in model_record["source_refs"]

    tuning = card["tuning"]
    assert isinstance(tuning, dict)
    assert tuning["status"] == "registered"
    campaigns = tuning["campaigns"]
    assert isinstance(campaigns, list)
    pseudo_campaign = next(item for item in campaigns if item["registration"]["id"] == "hummingbird-pseudo-hover-attitude-v1")
    assert pseudo_campaign["campaign_definition"]["strategy_id"] == "multirotor_hover_translation.v1"
    assert pseudo_campaign["campaign_definition"]["nodes"][0]["controller_method"] == "lqi"

    configuration = card["configuration"]
    assert isinstance(configuration, dict)
    assert configuration["parameter_count"] > 0
    segment_sequences = configuration["segment_sequences"]
    assert isinstance(segment_sequences, list)
    segments = next(item for item in segment_sequences if item["id"] == "segments")
    takeoff = next(item for item in segments["variants"] if item["id"] == "rotor_spool_takeoff")
    assert {item["id"] for item in takeoff["parameters"]} == {"target_altitude_m", "climb_rate_m_s"}
    ####


def test_model_overview_markdown_keeps_claim_boundaries_visible() -> None:
    """The readable card carries the fidelity, tuning, and parameter sections."""

    plugins = discover_plugins(include_external=False, selected=("taoryx.hummingbird",))
    providers = plugins.build_mission_composition_provider_registry()
    overview = build_model_overview_catalog(
        plugins,
        providers,
        plugins.build_controller_tuning_campaign_registry(),
        provider_id=PROVIDER_ID,
        model_id=MODEL_ID,
    )

    markdown = render_model_overview_markdown(overview)

    assert "# Taoryx vehicle model cards" in markdown
    assert "#### Provider scope" in markdown
    assert "#### Composition surface" in markdown
    assert "taoryx.trajectory-composition-advertisement/v1" in markdown
    assert "#### Fidelity ladder" in markdown
    assert "#### Data provenance" in markdown
    assert "hummingbird-pseudo-hover-attitude-v1" in markdown
    assert "#### Configuration parameters" in markdown
    assert "target_altitude_m" in markdown
    assert "> Claim boundary:" in markdown
    ####


def test_model_overview_cli_writes_a_machine_readable_card(tmp_path: Path, capsys) -> None:
    """The public command supports exact provider/model selection and JSON export."""

    destination = tmp_path / "hummingbird-card.json"

    assert (
        main(
            [
                "model",
                "overview",
                "--provider",
                PROVIDER_ID,
                "--model",
                MODEL_ID,
                "--format",
                "json",
                "--output",
                str(destination),
            ]
        )
        == 0
    )

    payload = json.loads(destination.read_text(encoding="utf-8"))
    assert payload["schema"] == "taoryx.model-overview/v1"
    assert payload["providers"][0]["models"][0]["identity"]["id"] == MODEL_ID
    assert f"wrote {destination}" in capsys.readouterr().out
    ####
