"""Exact model-plan coverage for every advertised operation tuple."""

from __future__ import annotations

from collections import defaultdict

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import discover_plugins
from taoryx.trajectory.configuration_contract import TrajectoryMissionOperationMetadata


def test_every_realization_specific_operation_row_is_selectable_and_projected_exactly() -> None:
    """Keep the public operation matrix aligned with selection and plan output.

    A row advertised for a concrete realization must survive the same
    provider/model/mission/fidelity selection a user makes through the common
    composer interface.  This intentionally covers blocked operations too:
    their explicit blockers belong in the exact plan rather than on a nearby,
    incompatible realization.
    """

    plugins = discover_plugins(include_external=False)
    providers = plugins.build_mission_composition_provider_registry()
    campaigns = plugins.build_controller_tuning_campaign_registry()
    adapters = plugins.build_family_adapter_registry()
    local_screens = plugins.build_local_controller_screen_advertisement_registry()

    exercised: set[tuple[str, str, str, str, str]] = set()
    for provider in providers.providers:
        for model in provider.list_models():
            for mission in model.mission_templates:
                rows: defaultdict[tuple[str, str], list[TrajectoryMissionOperationMetadata]] = defaultdict(list)
                for operation in mission.operations:
                    assert operation.realization_id is not None
                    rows[(operation.fidelity, operation.realization_id)].append(operation)

                for (fidelity, realization_id), expected_records in rows.items():
                    plan = build_model_authoring_plan(
                        providers,
                        campaigns,
                        provider.metadata.id,
                        model.id,
                        family_adapters=adapters,
                        local_controller_screens=local_screens,
                        fidelity=fidelity,
                        realization_id=realization_id,
                        mission_template_id=mission.id,
                    )
                    assert plan["status"] == "ready_to_author"
                    execution = plan["execution_advertisement"]
                    assert execution["mission_template_id"] == mission.id
                    actual_records = execution["operation_records"]
                    expected_keys = {
                        (
                            record.operation,
                            record.status,
                            record.common_runner_status,
                            record.execution_mode,
                            record.executor_id,
                        )
                        for record in expected_records
                    }
                    actual_keys = {
                        (
                            record["operation"],
                            record["status"],
                            record["common_runner_status"],
                            record["execution_mode"],
                            record["executor_id"],
                        )
                        for record in actual_records
                    }
                    assert actual_keys == expected_keys
                    for record in expected_records:
                        if record.operation in {"batch", "step"} and record.status == "available":
                            assert record.common_runner_status == "registered"
                        if record.status == "blocked":
                            assert record.blockers
                    exercised.add((provider.metadata.id, model.id, mission.id, fidelity, realization_id))

    assert len(exercised) == 69
    ####
