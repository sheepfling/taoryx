        # NASA/NESC Orbital Cylinder

        **Model ID:** `nesc-orbital-cylinder`  
        **Domain:** orbital  
        **Primary Taoryx fidelity:** `6dof-orbital-rigid-body`  
        **Qualification:** `source-ready`  
        **Priority:** 2

        ## Intended model tiers

        3dof-orbit;3t+3k2;6dof-orbital-rigid-body

        ## Source components

        - `nesc-all-models-orbital-cylinder-inertia` — mass_properties; parse `passed`; compile `passed`; checks `no_cases`

        ## NESC scenario coverage

        - `Orbit_06C` — Cylinder Plane-Change Firing
- `Orbit_06D` — Cylinder Earth-Departure Firing
- `Orbit_10A` — Cylinder Circular Orbit, Gravity Gradient, Zero Rate
- `Orbit_10B` — Cylinder Circular Orbit, Gravity Gradient, Initial Rate
- `Orbit_10C` — Cylinder Elliptical Orbit, Gravity Gradient, Zero Rate
- `Orbit_10D` — Cylinder Elliptical Orbit, Gravity Gradient, Initial Rate

        ## Host responsibilities

        WGS84; orbital EOM; applied force schedules; gravity-gradient torque

        ## Known gaps

        Applied loads and gravity-gradient law must be represented by Taoryx scenario/dynamic sidecars

        ## Qualification rule

        A source being collected and executable does not by itself qualify the complete Taoryx vehicle. Qualification requires source checks, canonical frame/unit checks, dimensional force/moment checks, and the assigned trajectory/scenario comparisons.
