        # NASA/NESC Atmospheric Spheroid

        **Model ID:** `nesc-atmospheric-spheroid`  
        **Domain:** atmospheric  
        **Primary Taoryx fidelity:** `3dof-point-mass`  
        **Qualification:** `source-ready`  
        **Priority:** 1

        ## Intended model tiers

        3dof-point-mass;3t+3k1;6dof-rigid-body-check

        ## Source components

        - `nesc-all-models-cannonball-aero` — aerodynamics; parse `passed`; compile `passed`; checks `no_cases`
- `nesc-all-models-cannonball-inertia` — mass_properties; parse `passed`; compile `passed`; checks `no_cases`

        ## NESC scenario coverage

        - `Atmos_01` — Dropped Sphere, Dragless
- `Atmos_04` — Dropped Sphere, Stationary Round Earth
- `Atmos_05` — Dropped Sphere, Rotating Round Earth
- `Atmos_06` — Dropped Sphere, WGS-84 Earth
- `Atmos_07` — Dropped Sphere, Steady Wind
- `Atmos_08` — Dropped Sphere, Wind Shear
- `Atmos_09` — Eastward Fired Cannonball
- `Atmos_10` — Northward Fired Cannonball

        ## Host responsibilities

        US1976; wind models; round/WGS84 Earth; inverse-square/J2 gravity

        ## Known gaps

        No standalone Taoryx package yet; scenario-owned environment and initial conditions

        ## Qualification rule

        A source being collected and executable does not by itself qualify the complete Taoryx vehicle. Qualification requires source checks, canonical frame/unit checks, dimensional force/moment checks, and the assigned trajectory/scenario comparisons.
