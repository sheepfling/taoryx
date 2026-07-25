# Alpha 2 case-contract fixtures

These files are the first Alpha 2 catalog slice. They exercise provider-neutral
case resolution only; they are not a recovered TAOS vehicle model and do not
claim runtime or flight validity.

The canonical catalog is `verification/alpha2_family_catalog.yaml`. It
deliberately contains two variants, two loadouts, two missions, and two segment
plans so that `case diff` can demonstrate configuration changes without copied
problem-file runners.

The `case-t6-*` fixtures resolve the same `dual_launch_glider` family, mission,
segment plan, controls, and post-release guidance contract. Their only launch
form difference is the explicit `extensions.launch_mode`: `air_release` starts
at the declared release state, while `attached_booster` adds the catalogued
powered segment and records a continuous separation handoff.
