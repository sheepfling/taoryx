        # NASA/NESC Tumbling Brick

        **Model ID:** `nesc-tumbling-brick`  
        **Domain:** atmospheric  
        **Primary Taoryx fidelity:** `6dof-rigid-body`  
        **Qualification:** `source-ready`  
        **Priority:** 1

        ## Intended model tiers

        3t+3k2;6dof-rigid-body

        ## Source components

        - `nesc-all-models-brick-aero` — aerodynamics; parse `passed`; compile `passed`; checks `no_cases`
- `nesc-all-models-brick-inertia` — mass_properties; parse `passed`; compile `passed`; checks `no_cases`

        ## NESC scenario coverage

        - `Atmos_02` — Tumbling Brick, No Damping
- `Atmos_03` — Tumbling Brick with Aerodynamic Damping

        ## Host responsibilities

        US1976; WGS84; J2; quaternion rigid-body EOM

        ## Known gaps

        No standalone Taoryx package yet; damping acceptance must be compared against trajectory family

        ## Qualification rule

        A source being collected and executable does not by itself qualify the complete Taoryx vehicle. Qualification requires source checks, canonical frame/unit checks, dimensional force/moment checks, and the assigned trajectory/scenario comparisons.
