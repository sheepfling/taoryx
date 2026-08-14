"""Lightweight provider surface for the optional reachability workbench."""

from __future__ import annotations

from taoryx.reachability_registry import ReachabilityProviderMetadata


class TaoryxReachabilityProvider:
    """Delegate parsed workbench operations to the installed implementation."""

    metadata = ReachabilityProviderMetadata(
        id="taoryx.reachability.workbench",
        version="0.1.0a0",
        description="Reduced-order reachability envelopes, overlays, continuation, and visualization.",
        families=("hl20_mod_k", "x15"),
        profiles=(
            "reachability.boost_glide.prelaunch_capture.v1",
            "reachability.glider.post_release_retarget.v1",
            "reachability.high_energy_glider.first_pass.v1",
            "reachability.passive_object.terminal_footprint.v1",
        ),
        operations=("list", "inspect", "plot", "rerun-timeouts", "run", "x15", "x15-native-replay"),
        claim_boundary=(
            "A successful witness proves declared-model feasibility only. Failure to find a witness is not proof of "
            "physical impossibility unless the selected method is independently certified."
        ),
    )

    def run_cli(self, arguments: object) -> int:
        """Load and execute one parsed reachability command lazily."""

        from .cli import handle_reachability_command

        return handle_reachability_command(arguments)
        ####

    ####


__all__ = ["TaoryxReachabilityProvider"]
####
