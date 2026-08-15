"""Regression coverage for plain configuration projection normalization."""

from __future__ import annotations

import pytest
from pydantic import BaseModel

from taoryx.model_authoring import (
    ModelAuthoringDraft,
    ModelAuthoringError,
    custom_sequence,
    normalize_authoring_values,
    segment_occurrence,
    select_variant,
    sequence_template,
)


class _ValueEnvelope(BaseModel):
    """Exercise Pydantic values at the public authoring ingress."""

    altitude_m: float

    ####


def test_helpers_emit_canonical_json_shaped_configuration_projections() -> None:
    """Choice and sequence helpers retain their convenient plain-value API."""

    initialization = select_variant("hover", _ValueEnvelope(altitude_m=25.0).model_dump())
    sequence = sequence_template(
        "short-hop",
        {"target_altitude_m": 25.0},
        {"target_altitude_m": 10.0},
    )
    custom = custom_sequence(segment_occurrence("coast", duration_s=2.0, instance_id="coast-01"))

    assert initialization == {"selected": "hover", "values": {"altitude_m": 25.0}}
    assert sequence == {
        "template": "short-hop",
        "items": [{"target_altitude_m": 25.0}, {"target_altitude_m": 10.0}],
    }
    assert custom == {
        "template": None,
        "items": [{"selected": "coast", "values": {"duration_s": 2.0}, "instance_id": "coast-01"}],
    }
    ####


def test_normalizer_rejects_nonportable_values_before_provider_compilation() -> None:
    """Opaque Python objects and non-string keys cannot leak through a draft."""

    with pytest.raises(ModelAuthoringError, match="nonportable-authoring-value"):
        normalize_authoring_values({"altitude_m": object()})
    with pytest.raises(ModelAuthoringError, match="nonportable-mapping-key"):
        normalize_authoring_values({1: "not a schema field"})
    ####


def test_draft_normalizes_framework_values_and_sequences_at_ingress() -> None:
    """Every serialized draft uses the same JSON-shaped projection form."""

    draft = ModelAuthoringDraft(
        draft_id="normalization-draft",
        configuration_id="normalization-config",
        provider_id="test.provider",
        model_id="test-model",
        model_version="1",
        schema_fingerprint="0" * 64,
        fidelity="point_mass_3dof",
        values={
            "initialization": _ValueEnvelope(altitude_m=25.0),
            "vector": (1.0, 2.0, 3.0),
        },
    )

    assert draft.values == {
        "initialization": {"altitude_m": 25.0},
        "vector": [1.0, 2.0, 3.0],
    }
    assert draft.public_dict(include_diagnostics=False)["values"] == draft.values
    ####
