# Boeing 747-100 validity and interpretation

This is a local handling-qualities model, not a global nonlinear wind-tunnel database.

- Landing and power-approach values are explicit NASA CR-2144 table entries.
- Cruise values are curated digitizations of NASA CR-2144 plots at FC3-FC10.
- The generated cruise grid is restricted to Mach overlap between adjacent altitude bands, so no row clamping is used.
- Alpha is generated as the implementation's nearest-reference trim alpha plus ±4 degrees.
- Beta is generated over ±5 degrees as a small-disturbance study grid; NASA CR-2144 does not publish a universal beta validity limit.
- Control and rate effects are linearly superposed.
- Stall, large separated flow, Reynolds effects, ground effect, and nonlinear control interactions are absent.
- Actuator limits and the JT9D installed-thrust law are implementation defaults, not direct CR-2144 data.
- Mass and inertia are configuration snapshots. Fuel flow and mass-versus-time are not modeled.
