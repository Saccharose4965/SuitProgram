#define main localfft_live_main
#include "../LatestFFT.c"
#undef main
#include <assert.h>
int main(void) {
    SDL_Surface *surface = SDL_CreateRGBSurfaceWithFormat(0, 512, 256, 32, SDL_PIXELFORMAT_ARGB8888);
    assert(surface);
    SDL_Renderer *r = SDL_CreateSoftwareRenderer(surface); assert(r);
    double frames[173 * COMB_BANDS];
    const double rate = SAMPLE_RATE / (double)FFT_SIZE;
    for (int run = 0; run < 16; ++run) {
        double shift = run / 16.0;
        double start = 100.137;
        for (int i = 0; i < 173; ++i) {
            double time = start + i / rate;
            double error = phase_wrap_error(time * 2 - shift, 1);
            double energy = 1 + exp(-error * error / .0008);
            for (int b = 0; b < COMB_BANDS; ++b) frames[i * COMB_BANDS + b] = energy;
        }
        PhaseEvidence e;
        assert(compute_phase_evidence_mean(frames, 173, COMB_BANDS, rate, 120, start, &e));
        double now = start + 172 / rate;
        PhaseTracker tracker = {0};
        PhaseDecision d = phase_tracker_update(&tracker, &e, PHASE_SCORE_MEAN, 120, now, 1, 0);
        assert(d.valid);
        BeatClock clock = {0};
        beat_clock_init_from_detected_beat(&clock, d.preciseBpm, d.flashAnchorTime, now);
        double last = 0;
        assert(beat_clock_emit(&clock, clock.nextBeatTime, &last));
        BeatDebug dbg = {.phaseValid=1, .targetBpm=120, .targetPhaseCount=128};
        memcpy(dbg.targetPhaseScore, e.score[0], sizeof(dbg.targetPhaseScore));
        draw_target_choice_panel(r, 0, 0, 512, 256, &dbg, &clock, 1, last);
        SDL_RenderPresent(r);
        // Full phase panel occupies x=256..511, y=128..255. Its left edge
        // must contain the selected pulse peak when the clock emits.
        int edge = 0, centre = 0;
        for (int y = 130; y < 255; ++y) {
            Uint32 *row = (Uint32 *)((char *)surface->pixels + y * surface->pitch);
            edge += row[256] == 0xffd2d2d2;
            centre += row[384] == 0xffd2d2d2;
        }
        assert(edge > 100 && centre < 10);
        printf("pulse phase %.4f: flash at graph left, height %d vs centre %d\n", shift, edge, centre);
    }
    SDL_DestroyRenderer(r); SDL_FreeSurface(surface);

    const int sizes[][2] = {{320,240}, {800,600}, {1280,800}, {1920,1080}};
    for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
        int width = sizes[i][0], height = sizes[i][1];
        surface = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_ARGB8888);
        assert(surface);
        r = SDL_CreateSoftwareRenderer(surface); assert(r);
        SDL_SetRenderDrawColor(r, 7, 7, 7, 255);
        SDL_RenderClear(r);
        BeatClock clock = {.initialized=1, .beatStartTime=10.125};
        BeatDebug dbg = {.phaseValid=1, .targetBpm=120, .targetPhaseCount=128};
        dbg.targetPhaseScore[32] = 1;
        for (int j = 0; j < 320; ++j) push_confidence_timeline(.8, 1.05);
        int top = height / 2, bottom = height - top;
        draw_target_choice_panel(r, 0, top, width, bottom, &dbg, &clock, 1, 10.125);
        SDL_RenderPresent(r);
        // The upper plots are untouched, and the full peak reaches the visible
        // last row at the phase panel's left edge at every supported size.
        Uint32 *upper = (Uint32 *)((char *)surface->pixels + (top - 1) * surface->pitch);
        for (int x = 0; x < width; ++x) assert(upper[x] == 0xff070707);
        Uint32 *lower = (Uint32 *)((char *)surface->pixels + (height - 1) * surface->pitch);
        assert(lower[width / 2] == 0xffd2d2d2);
        assert(lower[width * 3 / 4] != 0xffd2d2d2);

        SDL_DestroyRenderer(r); SDL_FreeSurface(surface);
    }
    puts("Render bounds passed at 320x240, 800x600, 1280x800 and 1920x1080.");
}
