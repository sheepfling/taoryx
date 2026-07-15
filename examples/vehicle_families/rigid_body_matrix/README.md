# Rigid-body vehicle-family matrix

These source files exercise the same TAORYX rigid-body runner with three
vehicle families:

- `rocket.prb`: powered body with staged-style propulsion data;
- `glider.prb`: aerodynamic body with standard `*fly` guidance;
- `drone.prb`: aerodynamic body with a bank command and no propulsion.

The coefficient deck is shared deliberately. The point of this matrix is to
prove that propulsion, aerodynamic tables, guidance, actuator policy, thermal
policy, and artifact production use one source-driven kernel boundary. The
deck is synthetic and is not an engineering model.
