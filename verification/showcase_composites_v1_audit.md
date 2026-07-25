# TAORYX showcase-composites-v1 independent audit

Archive SHA-256: `bfb8c767e8749dec3154f6a76f671d4ec47f076ec49d5a06ae83aa78aefbd6fa`

## Executive assessment

This packet is the strongest TAORYX presentation packet so far, but it is a showcase layer rather than a self-contained proof packet. It supports narrow claims of bounded, continuous, source-bounded research-surrogate controller missions. It does not independently prove source-model parity, global flight realism, controller robustness, or certification-level validity because the raw telemetry, scenario definitions, source packet, convergence histories, scoring formula, and referenced summaries are not included.

## Strong evidence

- Explicit claim boundary.
- Four nontrivial mission durations: B747 20 s, X8 120 s, Hummingbird 40 s, X-15 35 s.
- Continuity marked pass for each mission.
- Declared envelope checks.
- Plant scorecard includes sustained cases and fixed-step convergence status.
- Family panels expose altitude, speed, aerodynamic state, lateral state, and a closure channel.
- X8 and Hummingbird missions appear bounded over meaningful durations.

## Critical findings

1. The archive contains nine files and no raw telemetry, problem files, controller configuration, source model, convergence outputs, or reproducibility script.
2. `source_packet` and `summary_sha256` references cannot be verified from the archive.
3. Controller scores are effectively 100 even though final route error consumes 52-78% of the allowed limit. The score is not a meaningful measure of mission quality.
4. Route-error semantics are unclear. The X8 and Hummingbird plots appear to reset or jump as waypoint targets change; no commanded path or XY/XZ track is shown.
5. The closure time-series panel and manifest p99 values appear to refer to different residuals or normalizations. This must be reconciled in a metric dictionary.
6. Only one envelope variable is reported. Full alpha, beta, Mach, dynamic-pressure, actuator, speed, altitude, and model-table margins are needed.
7. No commanded-versus-achieved controls, actuator rates, saturation histories, body rates, or control effort are shown.
8. No source-evaluator parity results are packaged.
9. No uncertainty or Monte Carlo evidence is included.
10. Mission completion is threshold-based rather than objective-based; being merely under a generous error limit receives a perfect score.

## Family-specific findings

### Boeing 747-100

- The mission climbs about 120 m in 20 s while losing about 6 m/s and maintaining constant heading.
- This may be a valid climb/energy-transition response, but it is not obviously a route-capture mission.
- The 351.6 m final error against a 500 m limit is a weak terminal criterion.
- Required additions: commanded altitude/speed/heading, throttle/elevator histories, local attitude/rates, cross-track and along-track errors, endpoint corridor, and local-mode/source comparison.

### Skywalker X8

- The 120 s duration and in-envelope alpha are encouraging.
- Sideslip reaches about ±4 deg and changes sign rapidly at apparent waypoint transitions.
- Final route error 235.5 m against a 300 m limit does not demonstrate rectangle closure.
- Required additions: top-down path against commanded rectangle, waypoint-capture count, final return-to-start error, throttle/elevon commands and achieved positions, actuator saturation/rate, cross-track RMS, and source-model maneuver replay.

### AscTec Hummingbird

- The motion is bounded and smooth, with final route error around 0.105 m.
- The mission is very mild: speed below 0.12 m/s and roll below about 0.25 deg.
- It does not demonstrate takeoff, landing, yaw maneuvering, gust rejection, or rotor authority.
- Required additions: XYZ path, commanded path, four commanded/actual rotor speeds, motor lag, thrust/moment allocation, attitude/rates, takeoff-hover-square-yaw-return-land mission, and disturbance cases.

### X-15

- Altitude and speed histories are finite, but heading, alpha, and beta show sustained oscillation/chatter.
- Final route error is about 3.0 km against a 5.0 km limit, which is not compelling route capture.
- Required additions: control-surface/thrust commands and rates, body rates, dynamic pressure, model-envelope margins, propulsion/mass flow, heating only if validated, frequency/chatter analysis, source maneuver replay, and declared terminal event.

## Bottom-line claim supported by this packet

> TAORYX has bounded, continuous controller-mission demonstrations for four source-bounded research-surrogate plants, with declared envelope and closure gates. The packet is not a self-contained independent validation record and does not establish global or certification-level vehicle realism.
