Pending review

# Input audio policy

Physical button handling can create microphone transients. This service marks
the corresponding FFT novelty interval for suppression so button clicks have
less influence on beat tracking.

On an [input activity notification](../../drivers/input/README.md), it requests backfill for
the reported debounce/polling delay plus a 20 ms guard, and two trailing frames.
These are the existing settings, preserved during extraction. The guard covers
sound preceding recognition; trailing frames cover the remaining transient.

The shell connects the callback before its first poll. The call stays synchronous,
preserving the original order relative to button delivery. The driver and policy
test verifies the exact suppression arguments, including release transitions.

This service is the button-to-audio rule. The wider audio resource policy remains
separate work, tracked in the project progress file.
