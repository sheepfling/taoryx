# Drone POI-track loiter and return

This source-first synthetic mission expresses a vertical cold launch, three
moving POI-track guidance segments, and a return-home segment using ordinary
trajectories, segments, `*fly propnav`, and stop/transition events. The oracle
requires at least eight seconds within 25 m of each POI track and a return-home
capture within 25 m before the time guard. It exercises the common TAOS
point-mass guidance kernel; stationary-position hold, rotorcraft lift, and
motor dynamics remain explicitly outside this fixture's claim boundary.
