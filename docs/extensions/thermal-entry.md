# Thermal-entry extension

Thermal entry is a TAORYX extension layer. It does not alter the historical
TAOS point-mass equations or imply a flight-certified thermal model.

The runtime carries two explicit observables:

- `heat_load`: integrated heat-rate exposure;
- `peak_heat_rate`: highest observed instantaneous rate.

The entry controller consumes heat-rate and heat-load margins and may reduce a
requested angle-of-attack command. It must remain subordinate to the vehicle
force/moment model: the controller produces a command, not a direct state
mutation.

Every thermal showcase must report:

1. heat-rate model and units;
2. atmospheric-density source;
3. nose-radius or geometry assumptions;
4. heat-rate and heat-load limits;
5. integration step and tolerance;
6. whether the result is a demonstration, a numerical verification, or a
   historical comparison.

Plots should show the trajectory and limit bands together. A plot that omits
the configured limits is not sufficient evidence of thermal control.
