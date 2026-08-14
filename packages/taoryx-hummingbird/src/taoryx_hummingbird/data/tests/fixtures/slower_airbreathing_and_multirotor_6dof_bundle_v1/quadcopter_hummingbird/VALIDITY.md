# AscTec Hummingbird validity and interpretation

RotorPy models a rigid multirotor with individual rotor thrust and reaction torque, motor lag, frame drag, rotor drag, induced inflow and translational lift.

- World gravity is along negative world Z and rotor thrust is positive source-native body Z.
- Rotor positions and moments are referenced to the center of mass.
- Motor dynamics are first order with a 0.005 s time constant and 0-1500 rad/s saturation.
- The model ignores frame lift and moments caused by imbalanced frame drag.
- Wind variation across the vehicle is neglected.
- The Hummingbird parameter file notes that k_d and k_z were reduced by an order of magnitude because larger values were excessive. Treat these as tuned implementation values.
- Battery voltage, current, discharge, temperature and mass variation are absent.
- The generated wrench grids are deterministic evaluations of the RotorPy equations, not wind-tunnel measurements.
