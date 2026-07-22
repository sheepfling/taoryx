# B747 local trim pair

`SV01_3dof.prb` is the point-mass baseline. `SV01_6dof.prb` uses the native
rigid-body integrator and the B747 nominal static coefficient deck at the
selected reference flight condition. Both are deliberately short local tests;
they do not claim a global transport-aircraft model.

`SV01_long_trim_hold_6dof.prb` is the bounded long open-loop plant case.
`SV01_bounded_route_controller_6dof.prb` exercises the native `*fly propnav`
attitude path for one second while remaining inside the local table envelope.

`SV01_flight_path_hold_descent_6dof.prb` provides a 60-second source-scoped
descending corridor. `SV01_integrated_route_descent_6dof.prb` composes the
route and flight-path guidance paths for a bounded 20-second capture maneuver.
`SV01_reduced_fuel_mass_coupling_6dof.prb` uses the separately cataloged
notional first-cut fuel-flow table to exercise runtime mass coupling; it is not
historical JT9D fuel data.
