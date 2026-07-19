# Fidelity hierarchy

## Skywalker X8 — strongest direct identification

The X8 coefficients are flight-test identified and validated against separate maneuvers. The useful region remains local to the tested flight envelope; the source paper specifically warns that accuracy degrades far from trim and for negative alpha.

## Boeing 747 — authoritative source, local derivative formulation

NASA CR-2144 is authoritative handling-qualities data. The model is a Taylor expansion about ten trim points. Landing and power-approach derivative values are explicit table entries; the cruise implementation uses digitized graph values. The dense tables therefore interpolate local derivative models rather than reproduce a global nonlinear aerodynamic database.

## Hummingbird — coherent physics, mixed parameter provenance

RotorPy implements explicit rigid-body and rotor-aerodynamic equations. The Hummingbird parameter file combines literature values with implementation tuning; it explicitly notes tuning of rotor-drag and induced-inflow terms. Use it as a research simulator, not a certified propulsion model.

## NASA GTM alternate

GTM is an excellent small twin-turbine research model. The exact polynomial aerodynamic database was located by repository path and SHA, but raw binary retrieval was unavailable through the connected interface. Its metadata and non-aero parameters are included without fabricating the missing coefficient arrays.
