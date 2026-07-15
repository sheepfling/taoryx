# Langley winged-cone high-Mach reconstruction

Source: NASA CR-194987. This archive implements the report's M >= 6 branch.

Angles are radians inside the equations. Reference values are Sref = 3600 ft², cref = 80 ft, xref = 124 ft.

```text
CN_B = 0.48394443*a^3 + 0.11791944*a^2 + 0.15559427*a + 0.00030702135
CN_W = [4*sin(a)*cos(a)/M + 0.8*M*sin(a)^3] * S_eff/S_ref
CN   = CN_B + CN_W
CL   = CN*cos(a)

Cm_B = -0.039039436*a^3 + 0.044489436*a^2 + 0.057752634*a - 3.0901224e-06
Cm_W = CN_W*(x_ref - x_cp,W)/c_ref
Cm   = Cm_B + Cm_W

CDi,B = CN_B*a
CDi,W = (CN_W*cos(a))*a*S_eff/S_ref
```

Body-axis conversion assumes x forward and z down:

```text
CX = -CD*cos(a) + CL*sin(a)
CZ = -CD*sin(a) - CL*cos(a)
```

The component normalization is preserved as reconstructed from the source; verify it against the original report before altering the buildup.
