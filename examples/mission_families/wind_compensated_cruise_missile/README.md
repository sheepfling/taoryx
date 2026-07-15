# Wind-compensated cruise missile

Status: scaffolded from the existing wind and proportional-navigation cases.

The completed mission must demonstrate a powered low-altitude vehicle holding
or correcting its commanded heading in a deterministic wind field while
guiding toward a slowly moving target. The zero-wind case and wind case must
share the same source mission and differ only in the environment input.

Required outputs include ground-track error, air-relative speed, target range,
heading command, thrust command, and terminal miss distance. The wind heading
convention must be resolved against the manual before this becomes a normative
fixture. The TAORYX extension controller is independently exercised through
the interactive runtime while that historical syntax ambiguity remains open.
