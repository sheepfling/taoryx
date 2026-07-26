        # NASA/NESC International Space Station

        **Model ID:** `nesc-international-space-station`  
        **Domain:** orbital-spacecraft  
        **Primary Taoryx fidelity:** `6dof-orbital-rigid-body`  
        **Qualification:** `source-ready`  
        **Priority:** 2

        ## Intended model tiers

        3dof-orbit;3t+3k2;6dof-orbital-rigid-body

        ## Source components

        - `nesc-all-models-orbital-station-inertia` — mass_properties; parse `passed`; compile `passed`; checks `no_cases`

        ## NESC scenario coverage

        - `Orbit_02` — ISS Orbit Propagation
- `Orbit_03A` — ISS with 4x4 Gravity
- `Orbit_03B` — ISS with 8x8 Gravity
- `Orbit_04` — ISS with Third-Body Disturbances
- `Orbit_05A` — ISS Atmosphere, Minimal Solar Activity
- `Orbit_05B` — ISS Atmosphere, Mean Solar Activity
- `Orbit_05C` — ISS Atmosphere, Maximum Solar Activity
- `Orbit_08A` — ISS Free Rotation, Zero Initial Rate
- `Orbit_08B` — ISS Free Rotation, Nonzero Initial Rate
- `Orbit_09A` — ISS Torque, Zero Initial Rate
- `Orbit_09B` — ISS Torque, Nonzero Initial Rate
- `Orbit_09C` — ISS Torque and Force, Zero Initial Rate
- `Orbit_09D` — ISS Torque and Force, Nonzero Initial Rate
- `Orbit_FULL` — ISS Responding to All Effects

        ## Host responsibilities

        High-order gravity; third body; MET atmosphere; applied force/torque; cross products of inertia

        ## Known gaps

        Environment and applied loads are external to the inertia document

        ## Qualification rule

        A source being collected and executable does not by itself qualify the complete Taoryx vehicle. Qualification requires source checks, canonical frame/unit checks, dimensional force/moment checks, and the assigned trajectory/scenario comparisons.
