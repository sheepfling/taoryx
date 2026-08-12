"""Shared source-owned authority projection for CADAC persistent sessions."""

from __future__ import annotations

from taoryx.trajectory.mission_composition import (
    MissionCompositionControlAuthorityState,
    MissionCompositionSessionAuthorityProfile,
)

CADAC_SOURCE_PROGRAM_PROFILE_ID = "source_program_control"
CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID = "cadac_source_program"

_SOURCE_PROGRAM_PROFILE = MissionCompositionSessionAuthorityProfile(
    id=CADAC_SOURCE_PROGRAM_PROFILE_ID,
    authority="source_program",
    availability="available",
    action_ids=(),
    description=(
        "The retained CADAC source program owns guidance, controller, and actuator-command evolution; "
        "the session caller supplies duration only."
    ),
    command_owner="source_program",
    selection_scope="provider",
    switching_policy="provider_managed",
    scheme_id="provider.program",
    lowering_chain=(
        "cadac_source_program",
        "cadac_source_guidance",
        "cadac_source_controller",
        "cadac_source_actuator_or_response_law",
    ),
    claim_boundary=(
        "This profile makes source ownership inspectable. It does not create an external command seam, replace "
        "source sensor/controller state, or claim that a caller may switch the retained controller during a session."
    ),
)

_SOURCE_PROGRAM_STATE = MissionCompositionControlAuthorityState(
    active_profile_id=CADAC_SOURCE_PROGRAM_PROFILE_ID,
    command_source_id=CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID,
    command_owner="source_program",
    lowering_chain=_SOURCE_PROGRAM_PROFILE.lowering_chain,
    selection_scope="provider",
    switching_policy="provider_managed",
)


def source_managed_session_authority_fields() -> dict[str, object]:
    """Return descriptor fields for a CADAC session with no caller actions."""

    return {
        "action_schema_projection": "selected_semantic_profile",
        "authority_profiles": (_SOURCE_PROGRAM_PROFILE,),
        "default_authority_profile_id": CADAC_SOURCE_PROGRAM_PROFILE_ID,
        "active_authority_profile_id": CADAC_SOURCE_PROGRAM_PROFILE_ID,
        "command_source_id": CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID,
    }
    ####


def source_managed_session_authority_state() -> MissionCompositionControlAuthorityState:
    """Return immutable source-program ownership for one committed boundary."""

    return _SOURCE_PROGRAM_STATE
    ####


def validate_source_managed_session_request(
    authority_profile_id: str | None,
    command_source_id: str | None,
) -> None:
    """Reject attempts to manufacture caller authority over a source-owned case."""

    if authority_profile_id not in {None, CADAC_SOURCE_PROGRAM_PROFILE_ID}:
        raise ValueError(
            f"CADAC source-managed sessions support only authority profile {CADAC_SOURCE_PROGRAM_PROFILE_ID!r}"
        )
    if command_source_id not in {None, CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID}:
        raise ValueError(
            f"CADAC source-managed sessions retain command source {CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID!r}"
        )
    ####


__all__ = [
    "CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID",
    "CADAC_SOURCE_PROGRAM_PROFILE_ID",
    "source_managed_session_authority_fields",
    "source_managed_session_authority_state",
    "validate_source_managed_session_request",
]
