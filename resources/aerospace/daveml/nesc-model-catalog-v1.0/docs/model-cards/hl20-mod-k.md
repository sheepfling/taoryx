# NASA HL-20 Mod K Lifting Body

**Model ID:** `hl20-mod-k`  
**Domain:** atmospheric-lifting-body  
**Primary Taoryx fidelity:** `6dof-unpowered`  
**Qualification:** `qualified-unpowered-baseline`  
**Priority:** 2

## Intended model tiers

3dof-glide;3t+3k1;3t+3k2;6dof-unpowered

## Source components

- `daveml-example-hl20-aero` — aerodynamics; parse `passed`; compile `passed`; checks `passed`

## NESC scenario coverage

- No NESC trajectory scenario is directly assigned.

## Host responsibilities

US1976; trim; direct-surface controls; quaternion EOM

## Known gaps

No propulsion, SAS/actuators, ground contact, or independent NASA trajectory qualification

## Qualification rule

A source being collected and executable does not by itself qualify the complete Taoryx vehicle. Qualification requires source checks, canonical frame/unit checks, dimensional force/moment checks, and the assigned trajectory/scenario comparisons.
