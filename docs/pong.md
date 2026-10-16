# Pong

Choose AI or Human with up/down, then select. Human waits for another suit;
first waiter takes left, second right (near-simultaneous ties use MAC ordering).
The YOU label identifies the local paddle. Back returns to the menu; in AI mode
select pauses. Multiplayer does not have unilateral pause.

Both suits simulate the ball locally. Each owns collisions/misses at its own
paddle, broadcasting the impact trajectory, paddle position and event sequence.
The receiver advances that trajectory by event age and estimated one-way link
delay, then smoothly removes the visual discrepancy by midfield. Late corrections
past midfield use a short 80 ms blend. Scoring/serves are authoritative events,
not visual interpolation across rallies.

Packets repeat the latest collision, reject stale/wrong-peer/session data, and
use alternating collision authority to tolerate loss and duplicates. A missing
prior impact pauses prediction at the local collision plane until it arrives.
Disconnect returns to waiting after three seconds. Both suits need this protocol
version; it is incompatible with the previous host-streamed Pong.

Run `python3 components/pong/tests/test_pong.py` for two-instance host tests with
different clocks, packet jitter/loss, collisions, correction, scoring and exit.
Physical two-suit play remains to be tested; no firmware was flashed by the tests.
