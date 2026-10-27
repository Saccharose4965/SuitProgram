Pending review

# Performance monitor

The performance monitor samples shell frame timing and FreeRTOS task runtime
counters. Once per reporting interval it logs active app, achieved and target
frame rates, tick/draw/total work, per-core load and the busiest tasks.

Counter deltas are compared with the preceding snapshot. Idle-task deltas
provide core utilization; task runtime deltas produce the ranked task list.
Unavailable or reset counters start a fresh baseline. The monitor allocates no
worker task and runs only when the shell submits a completed-frame sample.

This service is diagnostic. It does not alter scheduling, frame budgets or task
affinity.
