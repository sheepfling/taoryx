# Environment providers

TAORYX keeps environment data behind a small typed boundary. A simulation
step may consume an `EnvironmentSample` containing the atmospheric quantities
needed by the current 3-DOF force model, an ECFC wind vector, and optional
weather fields such as humidity, cloud fraction, and rain rate.

The deterministic implementations are `StaticEnvironmentProvider` and
`ScheduledEnvironmentProvider`. The latter interpolates time-keyframed samples
for replayable changing-atmosphere tests. Future CSV/NetCDF/GRIB or analytical
providers should implement `EnvironmentProvider` and return the same sample
contract; they should not alter the vehicle state representation.

Legacy `.prb` wind blocks remain source-compatible. The manual defines speed/
heading as a direction *from* which wind blows, measured clockwise from north,
and defines component signs using the same meteorological convention. The
current legacy runtime fixtures use a competing direct-vector interpretation;
that discrepancy is recorded as `TAOS-AMB-0006` and is not silently resolved.
Component input uses east/north/down values. In the adopted runtime path the
resolved physical vector is subtracted from earth-relative velocity:

```text
V_air = V_earth - V_wind
```

The generic cruise dataset fixture compiles deterministic `windv`, `windh`,
and `windd` tables alongside its aerodynamic, propulsion, and mass tables.
Those generated tables are build products; the CSV and manifest are the
canonical fixture inputs.

This boundary does not claim a live weather service, atmospheric interpolation,
or historical TAOS runtime compatibility. Those remain later provider and
verification work.
