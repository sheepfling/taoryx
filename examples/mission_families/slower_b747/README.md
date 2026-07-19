# B747 local trim pair

`SV01_3dof.prb` is the point-mass baseline. `SV01_6dof.prb` uses the native
rigid-body integrator and the B747 nominal static coefficient deck at the
selected reference flight condition. Both are deliberately short local tests;
they do not claim a global transport-aircraft model.

`SV01_long_trim_hold_6dof.prb` is the bounded long open-loop plant case.
`SV01_bounded_route_controller_6dof.prb` exercises the native `*fly propnav`
attitude path for one second while remaining inside the local table envelope.
