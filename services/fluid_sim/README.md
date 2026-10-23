Pending review

# Fluid simulation

This service owns the particle state, gravity sampling, simulation worker and
the binary image shared with the Fluid screen.

## Particle model

The model contains 260 particles in a 64 by 32 coordinate space. Opening the
service scatters them in a radius-eight disc at the centre. A step applies the
two screen-plane gravity components, predicts particle positions, performs one
double-density-relaxation pass, derives velocity from the corrected positions,
and clamps particles one unit inside the boundary.

Double-density relaxation estimates ordinary and near density from neighbours
inside a four-unit radius. Ordinary pressure drives density toward 5; the
stronger near-pressure term prevents close particles collapsing into one
point. A four-unit uniform grid limits searches to the surrounding nine cells,
and each density and displacement pass stops after 48 neighbours. The hot loop
uses one Newton refinement of inverse square root.

The visual mask counts each particle into its surrounding 3 by 3 cells and
keeps cells hit at least twice. This produces a connected liquid shape in work
proportional to the particle count.

## Gravity and scheduling

The board accelerometer axes are converted to screen right/down axes, with both
configured signs negative. The service first reads the shared orientation
stream. Its retained recovery path reuses an existing MPU SPI device or starts
a local sampler if that service cannot start. This recovery bypasses the shared
orientation calibration and should be removed once orientation startup is
guaranteed for every app.

The worker targets one step every 33 ms on core 1. A normal step uses two
substeps. When computation reaches the frame budget it drops to one substep and
always yields; it restores the second substep after recovering headroom. This
protects the idle task and favours a continuous display over simulation detail.
If worker creation fails, the app's shell tick performs the normal two-substep
update.

## Snapshot boundary

Simulation output is copied into a dedicated published mask inside a short
critical section. Readers copy that stable mask under the same lock. Particle
arrays and neighbour structures remain private to the worker.

Stopping waits up to 300 ms for the worker to exit, then force-deletes it. A
forced deletion can interrupt a simulation update; the subsequent reset makes
the next start deterministic at the feature level.
