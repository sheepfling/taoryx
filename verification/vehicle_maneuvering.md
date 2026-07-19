# Slower-vehicle maneuvering evidence

The maneuver tranche gives each catalog vehicle a small, repeatable flight
task after the convention firewall and source-table checks:

| Vehicle | Task | Evidence boundary |
| --- | --- | --- |
| B747 | 10 s eastward waypoint, 100 m to 140 m altitude profile | Local NASA-surrogate response; no global transport claim |
| Skywalker X8 | 20 s powered waypoint with a small altitude increase | In-envelope flight-identified surrogate; no off-trim claim |
| Hummingbird | 8 s eastward waypoint and 2.0 m to 2.2 m altitude capture | Source-frame direct-wrench surrogate; no hardware claim |

All three problem files use the ordinary TAORYX runtime status, control,
route, guidance, propulsion, and aero constructs. The rotorcraft controller
is a generic allocation/control mode selected by existing runtime attributes;
it is not an X8- or Hummingbird-specific grammar extension.

Run the numerical evidence with:

```text
python -m pytest tests/e2e/test_slower_vehicle_maneuvers.py -m slow
```

Generate the human-readable Matplotlib packet with:

```text
python -m pytest tests/e2e/test_slower_vehicle_maneuvers.py -m artifact --override-ini='addopts='
```

The packet is research evidence for bounded native TAORYX maneuvers. It does
not establish flight qualification, historical TAOS compatibility, or the
validity of the underlying public surrogate data outside its declared range.
