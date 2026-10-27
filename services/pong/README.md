Pending review

# Pong service

## Match model

The service owns the ball, paddles, score, simulation clock, matchmaking state
and network session. AI mode simulates both ends locally. Human mode gives each
suit authority over collisions at its own paddle, allowing both costumes to
advance the ball independently between impacts.

The first suit waiting becomes side zero. Simultaneous discovery is resolved
from advertised waiting duration with MAC ordering as the tie breaker. Nonces
and a random session identifier bind later packets to that match. Offers and
accepts repeat on the heartbeat until both players enter the game.

## Distributed collision authority

Only the suit owning the approached paddle decides hit or miss. It increments
the collision sequence and broadcasts the resulting ball, its paddle and score.
The remote suit stops at an unowned collision until that event arrives.
Repeated snapshots are harmless because only the next sequence is accepted.

Each suit validates peer MAC, nonces, session, side, finite coordinates,
velocity limits, paddle range, score range and expected collision plane.
Packets from the local MAC and stale or malformed events are discarded.

## Delay and visual correction

Packets echo timing fields to estimate round-trip delay. A received collision
is advanced by its reported age, queue delay and half the best observed RTT,
bounded before the local paddle. The previously displayed position becomes a
visual correction offset that eases to zero by midfield. Simulation immediately
uses the authoritative trajectory while the viewer sees a continuous path.

Serving pauses for 400 ms. The heartbeat runs every 100 ms and a peer timeout
returns the remaining suit to waiting after three seconds. Shell stalls advance
at most 100 ms per tick, with at most four crossings per tick.

## AI opponent

The AI controls the right paddle. When the ball begins travelling towards it,
the AI chooses one aim error and keeps that target for the incoming leg. Its
error drifts towards the new target at 12 pixels per second, it waits 180 ms
before reacting and its paddle is limited to 26 pixels per second. The target
may miss the ball by up to 10 pixels. This produces smooth, fallible movement
without changing aim every frame.

## Player paddle

The app supplies a held direction each shell frame. The service approaches a
38-pixel-per-second target velocity at 160 pixels per second squared and eases
back to rest when released. Given the 31-pixel paddle travel, a sustained press
crosses the court in approximately one second. Boundary contact clears outward
velocity. Human matches distribute the resulting paddle position through their
existing state heartbeat.

## App boundary

The service exposes initialization, reset, mode start, paddle, pause, tick and
snapshot operations. Its radio callback validates and queues packets; the shell
tick mutates match state. The app owns selection, buttons, navigation and OLED.

## Verification

`python3 services/pong/tests/test_pong.py` compiles the real service and physics
with sanitizers. It runs two independent clocks through matchmaking, packet
loss, authoritative hits and misses, scoring, smoothing, malformed traffic,
disconnect recovery and AI pause. Hardware review still needs two suits with
real ESP-NOW latency and button input, plus an AI difficulty check.
