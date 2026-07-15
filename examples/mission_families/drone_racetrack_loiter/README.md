# Drone racetrack loiter

Status: scaffolded TAORYX extension.

This is not a historical TAOS syntax claim. The current fixture uses the
time-steppable vehicle session, changing-atmosphere provider, externally
supplied control commands, and racetrack controller. It uses the existing
kinematic-6DOF state boundary; a rotorcraft-specific force and moment model is
still a future extension.

The current acceptance oracle checks changing wind interpolation, completed
racetrack laps, and cross-track error. Future completion requires persisted
command/environment traces, attitude/rate outputs, and control saturation
events.
