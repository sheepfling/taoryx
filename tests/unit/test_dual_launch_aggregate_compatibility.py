"""One narrow compatibility seam for the extracted Dual Launch workflow."""

from __future__ import annotations

from taoryx.trajectory.dual_launch_mission_composition import build_dual_launch_example_configuration

from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider


def test_legacy_aggregate_delegates_dual_launch_execution_to_the_owned_package() -> None:
    """The aggregate preserves its public identity while the plug-in owns execution."""

    provider = RegistryMissionCompositionProvider()
    prepared = provider.validate_configuration(
        build_dual_launch_example_configuration(
            "attached_booster",
            provider.get_model_schema("dual_launch_glider"),
        )
    )
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id="dual-launch-aggregate-compatibility",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=2),
        )
    )

    assert response.kind == "trajectory"
    assert response.result.provider_id == "taoryx.registry.mission-composition"
    assert response.result.status == "completed"
    assert len(response.result.objects) == 1
    assert [item.kind for item in response.result.events if item.category == "separation"] == ["booster-glider-separation"]
    ####
