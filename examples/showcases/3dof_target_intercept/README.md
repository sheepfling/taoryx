# 3-DOF target intercept

This is the small, deterministic point-mass counterpart to the native CA–HI
6-DOF showcase. It is intentionally free of aerodynamic and gravity claims:
the interceptor begins on a straight closing trajectory and the target is
stationary. Its purpose is to prove that a target-hit event can be expressed
in a standard `.prb` file and executed through the ordinary parser, lowering,
and runtime pipeline.

The acceptance condition is `relrng[2] < 2` in the source problem file. The
test independently checks the final relative position, so the fixture cannot
pass merely because the stop event was reported.
