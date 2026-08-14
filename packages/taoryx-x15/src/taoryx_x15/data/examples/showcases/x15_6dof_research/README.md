# X-15 coherent 6-DOF research flight

This is the first standard-problem demonstration using the coherent X-15
research package. The `.prb` file is executed through the ordinary TAORYX
parser, lowering, six-degree-of-freedom rigid-body model, table aerodynamic
model, and table-driven propulsion path.

It is deliberately a short bounded flight inside the supplied Mach,
altitude, angle, and sideslip envelope. The test proves that the six-axis
force/moment tables, mass-flow history, quaternion state, and native
integrator are connected. It is not a flight reconstruction and does not
make a certified aerodynamic or thermal claim.

Inputs:

- `mission.prb`: standard TAORYX 6-DOF problem file;
- `../../tests/fixtures/x15_coherent_6dof_public_research_v1/tables/`: generated
  X-15 six-axis and XLR-99 tables supplied to the runner by the test harness.
