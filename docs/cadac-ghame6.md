# CADAC GHAME6 plug-in

`cadac.ghame6.hypersonic_vehicle` is the exact runnable GHAME6 primary model. It executes one source-ordered `HYPER6`, `SAT3`, and `RADAR0` composition.

## Fidelity

- Atmospheric HYPER6: `rigid_body_6dof_surface_allocated` through physical left/right elevons and rudder.
- Transfer/interceptor HYPER6: `rigid_body_6dof_direct_wrench` through axis-aggregate RCS.
- SAT3: `point_mass_3dof` orbital truth.
- RADAR0: static rotating-Earth site.

The public model uses a T4 run envelope because physical surfaces participate. Samples preserve the later T3 aggregate-RCS boundary.

## No TVC source path

The shipped GHAME6 source has no `tvc` module. A case that adds one fails lowering. The plug-in must not inherit ROCKET6G's physical-TVC claim merely because both primary actors are named `HYPER6`.

## Source ownership

The provider owns actor order, module order, sequential event progression, stored-derivative integration, physical surface mixing, phase resource resets, SAT3 truth, radar cadence, and the resulting one-pass communication semantics. Callers may supply bounded physical/direct command seams, end time, output cadence, and deterministic random seed.

## Result composition

Mission Composition returns three root objects:

```text
ghame6-hyper-1      cadac.ghame6.hypersonic_vehicle
ghame6-satellite-1  cadac.ghame6.satellite
ghame6-radar-1      cadac.ghame6.ground_site
```

Source phase transitions are HYPER6-owned events. Track publications are RADAR0-owned measurement events. No discarded-stage or carrier lineage is invented.

## Evidence boundary

The WGS84 rigid body, atmosphere, physical surfaces, propulsion transitions, aggregate RCS, SAT3 truth, RADAR0 measurements, and source events are executable. Full source guidance/navigation/sensor estimation, unsupported weather/wind/turbulence modes, exact stochastic parity, discarded-carrier propagation, and compiled-CADAC numerical parity are not yet promoted.
