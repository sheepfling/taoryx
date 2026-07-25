# Anduril-inspired flight-dynamics surrogate engineering review

This review supersedes loose nominal values in the initial public-surrogate
work. The models remain flight-dynamics engineering proxies, not exact vehicle
reconstructions or manufacturer-grade performance claims.

## Evidence grades

```text
P  direct public or government source
D  derived from public geometry/performance using standard physics
E  engineering estimate constrained by vehicle class and comparable systems
S  scenario placeholder requiring sensitivity analysis
```

Published brochure maxima are not assumed to occur simultaneously. Each
configuration has light, nominal, heavy, or dash mission states so payload,
fuel/energy, range, speed, and endurance remain physically coupled.

## Revised nominal roster

| Configuration | Revised nominal | Principal correction |
| --- | --- | --- |
| Bolt | 5.44 kg; approximately 0.63 kW hover; 24 m/s maximum; 45+ min endurance | P/D anchor with battery-energy consistency |
| Bolt-M | 5.9–6.8 kg; 0.80 kW hover; 40+ min endurance | Payload/variant penalty is explicit |
| Anvil | 5.5 kg with 4.5–7 kg band; 0.83 kW hover; 8–12 kW peak; 5–8 min | Replaces unsupported 15 kg nominal; E/S |
| Ghost-X | 20 kg nominal; 1.4 kW hover; 17 m/s cruise; 60–70 min nominal | Single-main-rotor model with light/nominal/heavy states |
| ALTIUS-600 | 11.3 kg; 0.55 m²; 16.6 m/s stall; 28.3 m/s cruise | High-aspect-ratio drag factor 0.030–0.045 |
| ALTIUS-700 ISR | 30 kg; 1.2 m²; 17.9 m/s stall; 4–5 h light mission | Separate from 700M payload configuration |
| ALTIUS-700M | 40 kg; 1.2 m²; 20.7 m/s stall; 75 min / 160 km mission | Heavy payload state, not ISR endurance |
| Roadrunner | 43 kg; 760 N static thrust; 40–50 m/s transition; 10–18 min | Fuel-flow model and severe VTOL penalty |
| Barracuda-100/250/500 | 50/90/205 kg; 58–60 m/s derived stall; common 5 g limit | Operational 90–100 m/s minimum is not stall speed |
| Fury/FQ-44 | 2.4 t; 13.5 m²; Mach 0.95 public ceiling; 8 g transient proxy | Low-aspect-ratio drag; no unsupported production 6DOF claim |
| Omen | 450 kg; 8.8 m²; 114 kW practical hover; 55 m/s cruise | Practical hover power reduced to a physics-consistent band |
| Thunder | 6 t nominal, 4.5–7.5 t band; 2.2 MW practical hover; 120 m/s cruise | Concept-level S-grade proxy; no precise public specs |

## Physics corrections

Rotorcraft and hover proxies use momentum-theory power with figure-of-merit,
profile, drivetrain, avionics, and control losses. Installed peak power is
separate from practical nominal hover power. A vehicle passes only if nominal
hover remains below 55–65% of installed peak power.

Fixed-wing proxies use a drag polar with induced drag derived from aspect ratio:

\[
C_D=C_{D0}+kC_L^2+C_{D,\mathrm{Mach}},
\qquad
k=\frac{1}{\pi e AR}.
\]

ALTIUS-600 uses a high-aspect-ratio `k` band of 0.030–0.045. Fury uses
approximately 0.16–0.23 unless body lift is modeled separately. Roadrunner,
Barracuda, and Fury receive configurable compressibility drag near their
critical Mach region; maximum speed is an altitude-dependent thrust/drag
intersection, not a hard velocity clamp.

Sea-level stall speed is derived from mass, area, density, and `CL_max`. Load
factor is constrained by both structural limits and available lift:

\[
L\leq\min(n_{\max}W, qSC_{L_{\max}}).
\]

Roadrunner and Omen use vertical, transition, wingborne, and recovery modes.
Thunder adds explicit nacelle/rotor tilt. Transition must preserve force and
attitude continuity; the aircraft does not switch discontinuously between a
quadrotor and a fixed-wing point mass.

## Mission-state coupling

Fuel vehicles derive fuel from gross-mass and payload constraints. Standard
states are:

```text
light/ferry  low payload, maximum fuel or energy
nominal      50–65% payload, 75–90% resource load
heavy        maximum payload, remaining allowable resource
dash         light or nominal payload, mission-specific resource
```

Battery discharge does not reduce vehicle mass. Battery-pack size, payload,
state of charge, and installed peak power are separate fields.

## Pseudo-6DOF boundary

Because moments, control derivatives, inertias, actuator bandwidth, and control
laws are mostly undisclosed, the initial Anduril-inspired library uses named
attitude-response profiles rather than fabricated torque-level 6DOF models.
Profiles add quaternion, body rates, rate limits, acceleration limits, lag,
saturation, mode scheduling, and vehicle-relative thrust-axis behavior.

Fixed-wing yaw is coordinated from bank and speed rather than directly commanded
in ordinary turns. Rotorcraft derive desired attitude from the acceleration or
thrust vector. This keeps the pseudo-6DOF model consistent with its underlying
3DOF force model.

## Qualification gates

Every surrogate configuration must pass:

1. Hover-power consistency.
2. Stall-speed derivation within 3% of the manifest value.
3. Cruise lift/drag and thrust equilibrium within 2%.
4. Maximum-speed thrust/drag intersection at a declared altitude.
5. Turn limits from both load factor and `CL_max`.
6. Energy/range/endurance reproduction only in the matching loading state.
7. Continuous hover-to-cruise transition where applicable.
8. Payload monotonicity for acceleration, climb, range, and endurance.
9. Altitude-appropriate high-speed checks for Barracuda, Roadrunner, and Fury.
10. ±25% sensitivity sweeps for E-grade parameters and full listed bands for
    S-grade parameters.

The next milestone artifact is a versioned YAML parameter pack containing
nominals, uncertainty bands, evidence grades, loading states, and automated
checks. Thunder remains `thunder_concept_proxy_v0` and is not promoted beyond
cataloged concept status by this review.

## Energy-coupled, mode-aware v0.2 architecture

The first adapter proved that the roster can be loaded and stepped. It is not
yet the model architecture we want to use for mission or controller claims.
The next version must make energy, mode, and mass-property changes part of the
same resolved vehicle instance.

### Common state contract

The shared pseudo-6-DOF state is:

\[
\mathbf{x}=\left[\mathbf r_N,\mathbf v_B,\mathbf q_{BN},
\boldsymbol\omega_B,m_f,E_b,\boldsymbol\delta,\lambda,\eta\right]
\]

where `r_N` is inertial position, `v_B` is body-axis velocity, `q_BN` is the
attitude quaternion, `omega_B` is body rate, `m_f` is remaining fuel, `E_b` is
remaining battery energy, `delta` contains actuator states, `lambda` is a
hover-to-wingborne blend, and `eta` is the Thunder nacelle angle. A specific
vehicle may omit a state only through a named fidelity profile; omitted states
must not silently reappear as hard-coded vehicle-specific behavior.

Both supported paths must resolve from this same parent configuration:

```text
resolved vehicle + loading state + environment
                  ├── 3DOF mission reduction
                  └── pseudo-6DOF attitude/rate realization
```

The 3-DOF path owns energy, acceleration, climb, turn, launch, transition,
and terminal feasibility. The pseudo-6-DOF path adds quaternion, body rates,
control allocation, sideslip/angle-of-attack proxies, actuator lag, limits,
and saturation. It is still a surrogate unless applied moments and inertias
are authoritative for the declared configuration.

### Energy backends

The parameter pack defines three selectable resource models.

| Backend | Resource | Mass policy | Required model behavior |
| --- | --- | --- | --- |
| `battery_electric` | `E_b` | Battery mass remains constant | Power draw, SOC, low-SOC derate, reserve guard |
| `fuel_burning` | `m_f` | Mass decreases continuously | Fuel flow, CG schedule, inertia schedule, reserve guard |
| `series_hybrid` | `E_b`, `m_f` | Fuel mass decreases; battery mass retained | Generator/load policy, bus power, battery buffering, landing reserve |
| `selectable` | Backend choice in configuration | Backend-specific | Run both declared alternatives and preserve the choice in provenance |

Battery power is:

\[
P_b=\frac{\sum_iP_{\mathrm{shaft},i}}
 {\eta_{\mathrm{motor}}\eta_{\mathrm{ESC}}}
 +P_{\mathrm{avionics}}+P_{\mathrm{payload}},
\qquad \dot E_b=-P_b.
\]

Below the configured SOC derate threshold, maximum available power is reduced.
The first implementation may use the linear derate in the parameter pack, but
the derate threshold and slope remain `E` or `S` unless a source establishes
them.

For fuel-burning propulsion:

\[
\dot m_f=-\mathrm{TSFC}(M,h,N)T
\]

or, for shaft-power engines,

\[
\dot m_f=-\mathrm{BSFC}\,P_{\mathrm{shaft}}.
\]

For Thunder:

\[
P_{\mathrm{bus}}=P_{\mathrm{gen}}+P_{\mathrm{batt}},
\qquad
\dot E_b=-P_{\mathrm{batt}},
\qquad
\dot m_f=-\mathrm{BSFC}_{\mathrm{gen}}P_{\mathrm{gen}}.
\]

The generator should normally operate in its efficient load band. The battery
handles vertical takeoff, conversion peaks, transients, generator-out
operation, and the landing/go-around reserve. It must not be modeled as a
passive fuel-flow multiplier.

### Mass, CG, and inertia scheduling

Fuel-burning aircraft use:

\[
m(t)=m_{\mathrm{empty}}+m_{\mathrm{payload}}+m_f(t).
\]

The first scheduler is a configurable linear CG interpolation between full and
empty fuel states. Inertia is recomputed from component masses and locations:

\[
\mathbf I=\mathbf I_{\mathrm{empty}}+
\sum_jm_j\left[(\mathbf r_j^T\mathbf r_j)\mathbf1-
\mathbf r_j\mathbf r_j^T\right].
\]

Until a geometry-backed mass distribution is available, the scheduler must
expose principal-inertia sensitivity, defaulting to a ±30% band. A controller
linearization or LQR gain set must identify the mass/resource snapshot used to
derive it; it is invalid to reuse nominal fuel-full gains after a material CG
or inertia change without an explicit scaling or retuning decision.

### Mode state machines

Modes are data, not branches hidden in a vehicle runner. Each mode declares:

```yaml
mode:
  id: transition
  entry_conditions: [speed_above_threshold, attitude_capture_valid]
  blend_state: lambda
  exit_conditions: [wingborne_lift_fraction, speed_above_transition_end]
  abort_conditions: [reserve_violation, control_authority_loss]
  resource_policy: preserve_landing_reserve
```

Initial role-specific sequences are:

| Vehicle | Mode sequence |
| --- | --- |
| Bolt ISR | `prep → vtol_launch → transit → hover_or_orbit → reposition → rtb → land` |
| Ghost-X | `vtol → climb → efficient_transit → stare_orbit → hover_stare → relocate → rtb` |
| Anvil | `alert → box_launch → max_climb → sprint_intercept → identify → terminal_or_abort` |
| ALTIUS | `eject → deploy → stabilize → prop_start → climb → orbit_or_racetrack → task → recovery_or_termination` |
| Roadrunner | `alert → vertical_launch → transition → intercept_climb → track → task_or_waveoff → return → recovery_transition → vertical_land` |
| Barracuda | `carrier_or_booster_launch → climb → efficient_cruise → loiter_or_formation → task → terminal` |
| Fury | `runway_takeoff → climb → rendezvous → package_join → cap_or_sensing → sprint_task → egress → rtb` |
| Omen | `tailsitter_launch → body_transition → climb → long_transit → relay_or_logistics → return → reverse_transition → land` |
| Thunder | `vtol → nacelle_conversion → wingborne_transit → mission_orbit → low_speed_task → egress → conversion → land` |

Launch, deployment, transition, recovery, and terminal modes must publish event
markers. A segment is not complete merely because the integrator reaches its
time limit; its entry, exit, abort, and resource conditions must be evaluated.

## Vehicle-specific v0.2 baselines

The compact table below describes the intended first-generation data, not
manufacturer specifications. Full values, bands, and grades live in the YAML
parameter pack.

| Group | Configuration policy | Primary uncertainty |
| --- | --- | --- |
| Bolt / Bolt-M | Battery multirotor; constant mass; motor lag and induced-power drain | Geometry, thrust map, and payload state |
| Anvil | Battery sprint/intercept multirotor; one-way terminal reserve | Nearly all quantitative performance fields |
| Ghost-X | Battery single-main-rotor; light/nominal/heavy loading states | Rotor/control response and configuration mass |
| ALTIUS-600/700/700M | Selectable battery or fuel backend; deployment transient | Production propulsion arrangement and payload configuration |
| Barracuda-100/250/500 | Fuel turbojet fixed-wing; throttle map, spool, altitude/Mach thrust lapse | Variant engine assignment and mass properties |
| Roadrunner | Fuel twin-jet VTOL; low-q thrust-vector channel plus aero surfaces | Exact control hardware, fuel load, and high-speed aero |
| Fury/FQ-44 | Fuel turbofan fixed-wing; transonic drag and reserve-aware mission | Mass, engine installation, and derivatives |
| Omen | Battery default plus range-extender scenario | Onboard energy architecture and numeric performance |
| Thunder/Halo | Series-hybrid dual-tiltrotor; generator/battery manager and nacelle state | All production dimensions and performance data |

Omen and ALTIUS must retain their backend choice in every result. Public
descriptions support architecture-level claims, but do not establish a single
production energy arrangement. Thunder remains concept-level even when its
series-hybrid architecture is modeled coherently.

## Validation ladder for promotion

Promotion is staged and family-specific:

1. **Data firewall** — units, grade, source reference, uncertainty, and
   missing-value policy are valid.
2. **Resource balance** — energy/fuel remains nonnegative, reserve guards
   trigger, and payload/resource monotonicity holds.
3. **Plant equilibrium** — hover or cruise trim, stall consistency, thrust/
   drag intersection, and load-factor limits pass.
4. **Transition continuity** — force, moment, attitude, and resource behavior
   remain continuous or are explicitly event-marked.
5. **Actuator/control** — allocation rank, lag, rate, saturation, and
   requested-versus-achieved telemetry pass.
6. **Paired fidelity** — 3-DOF and pseudo-6-DOF use the same parent instance
   and report expected omissions and comparison tolerances.
7. **Uncertainty/reachability** — E-grade ±25% sweeps, full S-grade bands,
   multi-start search, and resolution studies are recorded.

The first implementation milestone is not “all aircraft fly.” It is a
repeatable packet in which each configuration is classified as `cataloged`,
`data_ready`, `3dof_ready`, `pseudo_6dof_ready`, `mission_ready`, or
`blocked`, with the failing rung and evidence artifact named.
