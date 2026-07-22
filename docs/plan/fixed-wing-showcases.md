# Fixed-wing showcase set

The fixed-wing evidence set now has two deliberately different course shapes.

## Square course

The square is the waypoint/objective sequencing case. It shows discrete leg
changes, corner capture, altitude transitions, and return-to-start behavior.
Its red event markers are `W1`, `W2`, and `W3` in the family route panel.

## Figure-eight course

The figure-eight is a smooth steering and convention case. It uses a continuous
parametric route and a sinusoidal bank command so the vehicle must alternate
turn sense through the crossing. Its evidence should show:

- bank-command sign changes;
- achieved local-roll sign changes;
- heading continuity through the crossing;
- bounded alpha and beta;
- no velocity or attitude discontinuity;
- route error against the continuous reference.

The figure-eight route is generic runtime geometry, not a new vehicle-specific
grammar feature. It is selected through the existing route status extension
with `mode=figure-eight`; the vehicle remains described by the ordinary native
problem-file tables and controls.

These cases must be scored separately. A square waypoint miss and a figure-eight
bank reversal are different objectives and should not be collapsed into one
cross-family score.
