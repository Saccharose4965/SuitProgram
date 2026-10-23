Pending review

# Flappy rules

The service owns bird motion, three pipes, collision, scoring and the post-death
sequence. Its fixed playfield is 128 by 64 game units, matching the current
screen. Each step advances 1/30 second; the app decides when to call it.

## Motion and collisions

A flap sets upward velocity to -55. Gravity adds 75 units/second², with downward
speed capped at 140. Pipes move at 26 units/second through an accumulated
fractional displacement. Offscreen pipes recycle 70 units beyond the furthest
pipe, with a newly randomized gap centre.

The bird shape is shared with rendering because collision tests each occupied
pixel, including flipped orientation after death. flappy_shape.h is therefore
both the collision footprint and the visual mask, with one definition.
Pipe bodies and caps are tested separately. The living bird also dies at the
top/bottom boundaries.

## Score and death

Passing a pipe's trailing edge increments score and updates the best. The
existing equality-based check runs each step; if a pipe stays at the scoring
coordinate for multiple steps, it can count more than once. This rule remains
unchanged pending a gameplay fix.

After collision the bird flips, falls to the ground, then slides right until
clear of the pipes with an extra collision margin. It settles for roughly half
a second before marking the run finished. Pipes continue throughout these
phases. The app uses the death-frame counter for blinking.

Initialization loads the saved best through the
[scores service](../scores/README.md). Finish saves only a new record and logs
failure. The service starts no tasks and accepts no button/display types.
The app reads a const state view on the same game worker.
