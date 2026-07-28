        # NASA/NESC Orbital Spheroid

        **Model ID:** `nesc-orbital-spheroid`  
        **Domain:** orbital  
        **Primary Taoryx fidelity:** `3dof-orbit`  
        **Qualification:** `source-ready`  
        **Priority:** 2

        ## Intended model tiers

        3dof-orbit;3t+3k1;6dof-attitude-check

        ## Source components

        - `nesc-all-models-orbital-sphere-inertia` — mass_properties; parse `passed`; compile `passed`; checks `no_cases`

        ## NESC scenario coverage

        - `Orbit_06A` — Orbital Sphere with Fixed Drag
- `Orbit_06B` — Orbital Sphere with Dynamic Drag
- `Orbit_07A` — Sphere with 4x4 Gravity and Third Body
- `Orbit_07B` — Sphere with 8x8 Gravity and Third Body
- `Orbit_07C` — Sphere with 4x4 Gravity, Third Body, and Drag
- `Orbit_07D` — Sphere with 8x8 Gravity, Third Body, and Drag

        ## Host responsibilities

        WGS84; inverse-square/4x4/8x8 gravity; third body; MET atmosphere

        ## Known gaps

        Drag coefficients and force authority are partly scenario-owned rather than entirely in DAVE-ML

        ## Qualification rule

        A source being collected and executable does not by itself qualify the complete Taoryx vehicle. Qualification requires source checks, canonical frame/unit checks, dimensional force/moment checks, and the assigned trajectory/scenario comparisons.
