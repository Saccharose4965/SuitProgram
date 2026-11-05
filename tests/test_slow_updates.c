#define main localfft_live_main
#include "../LatestFFT.c"
#undef main
#include <assert.h>

static void test_history(void) {
    double a[BPM_COUNT], b[BPM_COUNT], reference[BPM_COUNT], refVar = 0, refConsistency = 0;
    for (int i = 0; i < BPM_COUNT; ++i) a[i] = b[i] = .5 / (BPM_COUNT - 1);
    a[40] = .5; b[60] = .5;
    for (int factor = 1; factor <= 4; factor *= 2) {
        ScoreHistory h = {0};
        double mean[BPM_COUNT], variance, consistency;
        double dt = factor * BPM_REFERENCE_SECONDS, coverage = 0;
        for (int tick = factor; tick <= 12; tick += factor) {
            score_history_push(&h, tick <= 8 ? a : b, tick * BPM_REFERENCE_SECONDS, dt);
            coverage = score_history_stats(&h, mean, &variance, &consistency);
        }
        assert(fabs(coverage - STABILITY_WINDOW_SECONDS) < 1e-9);
        assert(h.count == 8 / factor);
        if (factor == 1) {
            memcpy(reference, mean, sizeof(mean)); refVar = variance; refConsistency = consistency;
        } else {
            for (int i = 0; i < BPM_COUNT; ++i) assert(fabs(mean[i] - reference[i]) < 1e-12);
            assert(fabs(variance - refVar) < 1e-12);
            assert(fabs(consistency - refConsistency) < 1e-12);
        }
        // Delayed worker: expire stale scores, don't inflate one observation to four seconds.
        score_history_push(&h, b, 30, dt);
        coverage = score_history_stats(&h, mean, &variance, &consistency);
        assert(h.count == 1 && coverage < STABILITY_WINDOW_SECONDS);
        assert(!bpm_confident_for_phase(1, coverage, 1, 1.1, 140, 140));
        // Normal timing jitter must not permanently block a full-window phase gate.
        memset(&h, 0, sizeof(h));
        for (int tick = 1; tick <= 20; ++tick)
            score_history_push(&h, a, tick * (dt + .002), dt);
        coverage = score_history_stats(&h, mean, &variance, &consistency);
        assert(bpm_confident_for_phase(1, coverage, 1, 1.1, 120, 120));
    }
    puts("History: identical mean/variance/consistency at 1x/2x/4x, stale expiry and jitter coverage passed.");
}

static void update_hold(BpmHoldState *h, double time, double variance, int trusted) {
    double bpm, confidence, evidence; int pair, candidateTrusted;
    update_final_bpm_from_accum(120, 40, trusted ? .2 : .01, 1, variance, time,
        1, 0, h, &bpm, &pair, &confidence, &evidence, &candidateTrusted);
}

static void test_timers(void) {
    double referenceSupport = 0;
    for (int factor = 1; factor <= 4; factor *= 2) {
        BpmHoldState h = {.have=1, .bpm=120, .pairIdx=40, .havePreviousTime=1,
                         .previousTrusted=1, .previousCandidate=40};
        for (int tick = factor; tick <= 40; tick += factor)
            update_hold(&h, tick * BPM_REFERENCE_SECONDS, 0, 1);
        if (factor == 1) referenceSupport = h.support[40];
        else assert(fabs(h.support[40] - referenceSupport) < 1e-10);
        // Exactly six seconds without support halves it, regardless of observation rate.
        double time = h.previousEvalTime;
        for (int i = 1; i <= 4; ++i) update_hold(&h, time + i * 1.5, 0, 0);
        assert(fabs(h.support[40] - referenceSupport / 2) < 1e-10);
        h.shockCooldown = 3;
        update_hold(&h, h.previousEvalTime + 3, 0, 0);
        assert(h.shockCooldown == 0);
    }
    BpmHoldState h = {0};
    update_hold(&h, 2, 0, 1);
    assert(!h.have); // first slow observation cannot immediately lock from two seconds of invented support
    update_hold(&h, 2 + BPM_UPDATE_SECONDS, 0, 1);
    assert(h.have && h.bpm == 120);
    double initial = h.support[40], time = h.previousEvalTime;
    update_hold(&h, time + 30, 0, 1);
    assert(h.support[40] <= initial / 32 + BPM_REFERENCE_SECONDS + 1e-9);
    // Loss and recovery measure seconds, not a count of observations.
    for (int factor = 1; factor <= 4; factor *= 2) {
        double dt = factor * .5, lost = 0;
        for (int i = 0; i < AUTO_STOP_LOST_SECONDS / dt; ++i)
            lost = update_lost_pulse(lost, dt, 0, 1, .59, 0);
        assert(lost == AUTO_STOP_LOST_SECONDS);
        for (int i = 0; i < 6 / dt; ++i) lost = update_lost_pulse(lost, dt, 0, 1, .6, 0);
        assert(lost == 0);
        assert(update_lost_pulse(lost, dt, 0, 1, 0, 1) == 0);
    }
    assert(update_lost_pulse(39, 20, 0, 1, .6, 0) == 0);
    // Compare the variance-shock rate across short and long evaluation steps.
    for (int factor = 1; factor <= 4; factor *= 2) {
        BpmHoldState shock = {.havePreviousTime=1, .havePreviousVariance=1};
        update_hold(&shock, factor * BPM_REFERENCE_SECONDS, .003 * factor, 0);
        assert(shock.shockCooldown == 0);
        shock = (BpmHoldState){.havePreviousTime=1, .havePreviousVariance=1};
        update_hold(&shock, factor * BPM_REFERENCE_SECONDS, .005 * factor, 0);
        assert(shock.shockCooldown == SHOCK_COOLDOWN_SECONDS);
    }
    // A shock blocks transitions, but must not zero an already-held matching BPM.
    BpmHoldState heldShock = {.have=1, .bpm=120, .pairIdx=40,
                              .havePreviousTime=1, .havePreviousVariance=1};
    double bpm, confidence, evidence; int pair, candidateTrusted;
    update_final_bpm_from_accum(120, 40, .2, 1, .005, BPM_REFERENCE_SECONDS,
        1, 0, &heldShock, &bpm, &pair, &confidence, &evidence, &candidateTrusted);
    assert(heldShock.shockCooldown > 0);
    assert(confidence > 0 && candidateTrusted);
    puts("Timers: cadence-invariant support/decay, cautious acquisition, stall cap, cooldown and 40s loss passed.");
}

static void test_paced_worker(void) {
    enum { N = 173 };
    double frames[N * COMB_BANDS], job[BPM_COUNT], result[BPM_COUNT], expected[BPM_COUNT];
    for (int i = 0; i < N * COMB_BANDS; ++i) frames[i] = 2 + sin(i * .17);
    compute_raw_comb_scores(frames, N, COMB_BANDS, (double)SAMPLE_RATE / FFT_SIZE, expected);
    BPMWorker w = {0};
    w.mutex = SDL_CreateMutex(); w.cond = SDL_CreateCond();
    assert(w.mutex && w.cond);
    w.jobFrames = frames; w.jobScores = job; w.resultScores = result; w.bpmCount = BPM_COUNT;
    w.jobNumFrames = N; w.jobFrameRate = (double)SAMPLE_RATE / FFT_SIZE;
    w.jobLastFrameTime = 10; w.busy = w.hasJob = 1;
    w.thread = SDL_CreateThread(bpm_worker_main, "worker-test", &w); assert(w.thread);
    Uint64 deadline = SDL_GetTicks64() + 10000;
    int ready = 0;
    while (SDL_GetTicks64() < deadline) {
        SDL_LockMutex(w.mutex); ready = w.resultReady; SDL_UnlockMutex(w.mutex);
        if (ready) break;
        SDL_Delay(5);
    }
    assert(ready);
    SDL_LockMutex(w.mutex);
    assert(w.resultLastFrameTime == 10 && w.resultNumFrames == N);
    assert(memcmp(result, expected, sizeof(expected)) == 0); // batching changes no math
    assert(w.resultWallMs >= BPM_UPDATE_SECONDS * BPM_WORK_SPREAD_FRACTION * 1000 * .95);
    printf("Worker: exact score parity; work %.1fms spread over %.1fms.\n", w.resultWorkMs, w.resultWallMs);
    w.resultReady = 0; w.busy = w.hasJob = 1; SDL_CondSignal(w.cond);
    SDL_UnlockMutex(w.mutex);
    // Discontinuity during in-flight work must discard its eventual result.
    SDL_LockMutex(w.mutex); ++w.epoch; SDL_UnlockMutex(w.mutex);
    deadline = SDL_GetTicks64() + 10000;
    int busy = 1;
    while (SDL_GetTicks64() < deadline) {
        SDL_LockMutex(w.mutex); busy = w.busy; SDL_UnlockMutex(w.mutex);
        if (!busy) break;
        SDL_Delay(5);
    }
    assert(!busy);
    SDL_LockMutex(w.mutex);
    assert(!w.resultReady);
    w.jobEpoch = w.epoch;
    w.busy = w.hasJob = 1; SDL_CondSignal(w.cond);
    SDL_UnlockMutex(w.mutex);
    SDL_Delay(20);
    Uint64 stopStart = SDL_GetTicks64();
    SDL_LockMutex(w.mutex); w.stop = 1; SDL_CondSignal(w.cond); SDL_UnlockMutex(w.mutex);
    SDL_WaitThread(w.thread, NULL);
    assert(SDL_GetTicks64() - stopStart < 500); // shutdown interrupts pacing sleep
    SDL_DestroyCond(w.cond); SDL_DestroyMutex(w.mutex);
}

int main(void) {
    test_history(); test_timers(); test_paced_worker();
    puts("Slow-update regressions passed.");
}
