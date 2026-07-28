# DAVE-ML 1976 U.S. Standard Atmosphere

**Model ID:** `daveml-atmosphere-1976`  
**Domain:** environment-service  
**Primary Taoryx fidelity:** `static-environment`  
**Qualification:** `conformance-source`  
**Priority:** 0

## Intended model tiers

environment-table-service

## Source components

- `daveml-example-atmos-76` — environment; parse `passed`; compile `passed`; checks `failed`

## NESC scenario coverage

- No NESC trajectory scenario is directly assigned.

## Host responsibilities

DAVE-ML table evaluator; geometric/geopotential conversion host

## Known gaps

Current runtime needs stronger signal-name resolution to pass all embedded checks reliably

## Qualification rule

A source being collected and executable does not by itself qualify the complete Taoryx vehicle. Qualification requires source checks, canonical frame/unit checks, dimensional force/moment checks, and the assigned trajectory/scenario comparisons.
