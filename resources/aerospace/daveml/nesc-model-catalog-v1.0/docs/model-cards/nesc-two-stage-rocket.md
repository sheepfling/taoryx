        # NASA/NESC Two-Stage Rocket

        **Model ID:** `nesc-two-stage-rocket`  
        **Domain:** atmospheric-launch-vehicle  
        **Primary Taoryx fidelity:** `6dof-variable-mass`  
        **Qualification:** `qualified-baseline`  
        **Priority:** 2

        ## Intended model tiers

        3dof-variable-mass;3t+3k1-gravity-turn;6dof-variable-mass

        ## Source components

        - `nesc-all-models-two-stage-package-twostage-aero` — aerodynamics; parse `passed`; compile `passed`; checks `no_cases`
- `nesc-all-models-two-stage-package-twostage-inertia` — mass_properties; parse `passed`; compile `passed`; checks `no_cases`
- `nesc-all-models-two-stage-package-twostage-prop` — propulsion; parse `passed`; compile `passed`; checks `no_cases`

        ## NESC scenario coverage

        - `Atmos_17` — Two-Stage Rocket Sea-Level Launch

        ## Host responsibilities

        US1976; WGS84; J2; fuel integration; burnout/coast/staging events

        ## Known gaps

        Independent full trajectory-family qualification remains; existing perigee discrepancy stays visible

        ## Qualification rule

        A source being collected and executable does not by itself qualify the complete Taoryx vehicle. Qualification requires source checks, canonical frame/unit checks, dimensional force/moment checks, and the assigned trajectory/scenario comparisons.
