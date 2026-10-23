#include "input_audio_policy.h"
#include "fft.h"

void input_audio_policy_activity(int elapsed_ms)
{
    // Debounce latency plus the established acoustic guard and trailing frames.
    fft_suppress_novelty_timed_ms(elapsed_ms + 20, 2);
}
