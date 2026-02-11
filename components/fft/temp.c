// FFT_pc_corrected.c
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <alsa/asoundlib.h>
#include <SDL2/SDL.h>

#define SAMPLE_RATE 44100
#define CHANNELS 1

// FFT for everything (spectrogram + 6s analysis)
#define FFT_SIZE 1024 // must be power of 2
#define HOP_SIZE FFT_SIZE // 1 FFT per block, no overlap

// Rolling window (in seconds) for novelty/BPM and spectrogram history
#define RECORD_SECONDS 6.0 // Window size

#define WIN_WIDTH 2560
#define WIN_HEIGHT 1500

// Visualization scaling parameters
#define SPEC_SCROLL_SCALE 50   // magnitude scale for scrolling spectrogram
#define SPEC_PANEL_SCALE 50    // magnitude scale for top-right spectrum
#define NOVELTY_SCALE 800      // vertical scale for novelty curve

// ESP-style novelty cleanup
#define NOVELTY_MEAN_OFFSET 0.02
#define NOVELTY_SPIKE_ZERO_THRESHOLD 400.0

// ESP-style BPM confidence tuning (used for optional "main BPM" marker)
#define COMB_GLOBAL_MIN 0.01
#define COMB_PEAK_FRAC_OF_GLOBAL 0.25
#define COMB_PEAK_AVG_MULTIPLIER 10.0
#define SPEC_MIN_LEVEL 0.05

// BPM analysis
#define BPM_MIN 30
#define BPM_MAX 300
#define TARGET_BPM_MIN 64
#define TARGET_BPM_MAX 127
#define PHASE_SLOTS 32
#define MAX_PHASE_EVAL 256

// Novelty / band settings
#define MIN_INT_FREQ_HZ 100.0

// Spectrogram panel resolution
#define SPEC_WIDTH 260
#define SPEC_HEIGHT (WIN_HEIGHT / 2)

static double g_phaseTime = 0.0; // seconds since start, used to animate BPM phases

// ESP-style novelty state (incremental)
static int g_prevLogmagValid = 0;
static double *g_prevLogmag = NULL;

static double *g_noveltyRing = NULL;
static int g_novWritePos = 0;
static int g_novCount = 0;
static int g_novCap = 0;

// Local mean window for novelty (computed from frameRate; defaults ~0.5s)
static double *g_localMeanRing = NULL;
static int g_localWin = 0;
static int g_localIdx = 0;
static int g_localCount = 0;
static double g_localSum = 0.0;

// Optional "main BPM" marker (ESP-style peak picking)
static double g_mainBpm = 0.0;
static double g_mainBpmConf = 0.0;
static double g_mainBpmPhase = 0.0;
static double g_phaseCurveSelected[PHASE_SLOTS];
static int g_phaseCurveSelectedLen = 0;
static uint64_t g_novTotalFrames = 0;

// --- ESP-like BPM display scaling (auto, smoothed) ---
static double g_bpm_display_scale = 1.0;  // smoothed "max" for the BPM panel
static double g_bpm_display_floor = 1e-6; // prevents blow-ups when quiet
#define BPM_SCALE_SMOOTH_UP   0.25  // how fast scale increases (0..1)
#define BPM_SCALE_SMOOTH_DOWN 0.05  // how fast scale decreases (0..1)
#define BPM_HEADROOM          1.10  // extra headroom above max

// --- Smoothed BPM spectra over time (per BPM bin) ---
static double *g_famSpecSmooth = NULL;
static int g_famSpecSmoothCap = 0;

// Smoothing time constants in seconds (tune)
#define FAMSPEC_TAU_UP_SEC    0.35   // rise time constant
#define FAMSPEC_TAU_DOWN_SEC  1.20   // fall time constant (slower decay)

// ---------------- Utility functions ----------------

static inline Uint32 make_gray(Uint8 c) {
    return (0xFFu << 24) | ((Uint32)c << 16) | ((Uint32)c << 8) | (Uint32)c;
}

static int read_exact_frames(snd_pcm_t *handle, int16_t *buf, snd_pcm_uframes_t frames_wanted) {
    snd_pcm_uframes_t done = 0;
    while (done < frames_wanted) {
        snd_pcm_uframes_t to_read = frames_wanted - done;
        int rc = snd_pcm_readi(handle, buf + done, to_read);
        if (rc == -EPIPE) {
            fprintf(stderr, "overrun occurred\n");
            snd_pcm_prepare(handle);
            done = 0;
            continue;
        } else if (rc < 0) {
            fprintf(stderr, "error from read: %s\n", snd_strerror(rc));
            return rc;
        } else if (rc == 0) {
            usleep(1000);
            continue;
        }
        done += (snd_pcm_uframes_t)rc;
    }
    return 0;
}

static inline double alpha_from_tau(double dt, double tau){
    if (tau <= 1e-9) return 1.0;
    return 1.0 - exp(-dt / tau);
}

static void smooth_spectrum_asym_tau(const double *in, double *state, int n,
                                     double dt, double tauUp, double tauDown)
{
    double aUp   = alpha_from_tau(dt, tauUp);
    double aDown = alpha_from_tau(dt, tauDown);

    for (int i = 0; i < n; ++i) {
        double x = in[i];
        double y = state[i];
        double a = (x > y) ? aUp : aDown;
        state[i] = (1.0 - a) * y + a * x;
    }
}

static inline double wrap_phase01(double phase){
    phase -= floor(phase);
    if (phase < 0.0) phase += 1.0;
    return phase;
}

static void shift_phase_curve_circular(double *vals, int count, double shift_norm){
    if (!vals || count < 2) return;

    shift_norm = wrap_phase01(shift_norm);
    double shift = shift_norm * (double)count;
    double tmp[PHASE_SLOTS];

    for (int i = 0; i < count; ++i){
        double src = (double)i - shift;
        while (src < 0.0) src += (double)count;
        while (src >= (double)count) src -= (double)count;

        int i0 = (int)floor(src);
        int i1 = i0 + 1;
        if (i1 >= count) i1 = 0;
        double frac = src - (double)i0;

        double v0 = vals[i0];
        double v1 = vals[i1];
        tmp[i] = v0 * (1.0 - frac) + v1 * frac;
    }

    memcpy(vals, tmp, (size_t)count * sizeof(double));
}

// Recompute comb phase using the exact selected BPM spacing.
static void recompute_phase_for_bpm(const double *novelty, int numFrames, double frameRate,
                                    double bpm, double *phaseNormOut,
                                    double *phaseCurveOut, int *phaseCurveLenOut)
{
    if (phaseNormOut) *phaseNormOut = 0.0;
    if (phaseCurveLenOut) *phaseCurveLenOut = 0;
    if (!novelty || numFrames <= 1 || frameRate <= 0.0 || bpm <= 0.0){
        return;
    }

    double step = frameRate * 60.0 / bpm;
    int phaseCount = (int)floor(step);
    if (phaseCount < 1) phaseCount = 1;
    if (phaseCount > MAX_PHASE_EVAL) phaseCount = MAX_PHASE_EVAL;

    const double minSamplesPerPhase = 2.0;
    double phaseScores[MAX_PHASE_EVAL];
    for (int i = 0; i < phaseCount; ++i){
        phaseScores[i] = 0.0;
    }

    double bestScore = 0.0;
    int bestPhaseIndex = 0;

    for (int phase = 0; phase < phaseCount; ++phase){
        double pos = (double)phase;
        double sum = 0.0;
        int count = 0;

        while (pos < (double)(numFrames - 1)) {
            int j = (int)floor(pos);
            double t = pos - (double)j;
            double v;
            if (j >= 0 && j + 1 < numFrames) {
                v = (1.0 - t) * novelty[j] + t * novelty[j + 1];
            } else if (j >= 0 && j < numFrames) {
                v = novelty[j];
            } else {
                v = 0.0;
            }
            sum += v;
            count += 1;
            pos += step;
        }

        if ((double)count < minSamplesPerPhase){
            phaseScores[phase] = 0.0;
            continue;
        }

        phaseScores[phase] = sum;
        if (sum > bestScore){
            bestScore = sum;
            bestPhaseIndex = phase;
        }
    }

    if (phaseNormOut && bestScore > 0.0 && phaseCount > 1){
        *phaseNormOut = (double)bestPhaseIndex / (double)(phaseCount - 1);
    }

    if (!phaseCurveOut || !phaseCurveLenOut || bestScore <= 0.0){
        return;
    }

    if (phaseCount == 1){
        for (int s = 0; s < PHASE_SLOTS; ++s){
            phaseCurveOut[s] = phaseScores[0];
        }
        *phaseCurveLenOut = PHASE_SLOTS;
        return;
    }

    for (int s = 0; s < PHASE_SLOTS; ++s){
        double p = ((double)s / (double)PHASE_SLOTS) * (double)phaseCount;
        while (p >= (double)phaseCount) p -= (double)phaseCount;

        int i0 = (int)floor(p);
        int i1 = i0 + 1;
        if (i1 >= phaseCount) i1 = 0;
        double frac = p - (double)i0;

        double v0 = phaseScores[i0];
        double v1 = phaseScores[i1];
        phaseCurveOut[s] = v0 * (1.0 - frac) + v1 * frac;
    }
    *phaseCurveLenOut = PHASE_SLOTS;
}

static void select_main_bpm_phase_from_scores(const double *scoreMap,
                                              const double *novelty, int numFrames,
                                              double frameRate)
{
    g_mainBpm = 0.0;
    g_mainBpmConf = 0.0;
    g_mainBpmPhase = 0.0;
    g_phaseCurveSelectedLen = 0;

    if (!scoreMap || !novelty || numFrames <= 1 || frameRate <= 0.0){
        return;
    }

    int targetMinIdx = TARGET_BPM_MIN - BPM_MIN;
    int targetMaxIdx = TARGET_BPM_MAX - BPM_MIN;
    int bpmCount = BPM_MAX - BPM_MIN + 1;
    if (targetMinIdx < 0) targetMinIdx = 0;
    if (targetMaxIdx >= bpmCount) targetMaxIdx = bpmCount - 1;
    if (targetMaxIdx < targetMinIdx) return;

    double targetEnergy = 0.0;
    for (int i = targetMinIdx; i <= targetMaxIdx; ++i){
        double v = scoreMap[i];
        if (v > 0.0) targetEnergy += v;
    }
    if (targetEnergy <= 1e-9) return;

    int pairBestIdx = targetMinIdx;
    double pairBestScore = 0.0;
    if (targetMaxIdx > targetMinIdx){
        for (int i = targetMinIdx; i < targetMaxIdx; ++i){
            double s0 = scoreMap[i];
            double s1 = scoreMap[i + 1];
            if (s0 < 0.0) s0 = 0.0;
            if (s1 < 0.0) s1 = 0.0;
            double pairScore = s0 + s1;
            if (pairScore > pairBestScore){
                pairBestScore = pairScore;
                pairBestIdx = i;
            }
        }
    } else {
        double s0 = scoreMap[targetMinIdx];
        if (s0 < 0.0) s0 = 0.0;
        pairBestScore = s0;
    }
    if (pairBestScore < COMB_GLOBAL_MIN) return;

    double sPair0 = scoreMap[pairBestIdx];
    double sPair1 = (pairBestIdx + 1 <= targetMaxIdx) ? scoreMap[pairBestIdx + 1] : 0.0;
    if (sPair0 < 0.0) sPair0 = 0.0;
    if (sPair1 < 0.0) sPair1 = 0.0;
    double pairMass = sPair0 + sPair1;

    double bpm0 = (double)(BPM_MIN + pairBestIdx);
    double bpm1 = (double)(BPM_MIN + pairBestIdx + 1);
    double bpmSel = bpm0;
    if (pairBestIdx + 1 <= targetMaxIdx && pairMass > 1e-9){
        bpmSel = (bpm0 * sPair0 + bpm1 * sPair1) / pairMass;
    }

    int clusterL = pairBestIdx;
    int clusterR = (pairBestIdx + 1 <= targetMaxIdx) ? (pairBestIdx + 1) : pairBestIdx;
    while (clusterL > targetMinIdx){
        double inner = scoreMap[clusterL];
        double outer = scoreMap[clusterL - 1];
        if (inner < 0.0) inner = 0.0;
        if (outer < 0.0) outer = 0.0;
        if (outer < inner) clusterL--;
        else break;
    }
    while (clusterR < targetMaxIdx){
        double inner = scoreMap[clusterR];
        double outer = scoreMap[clusterR + 1];
        if (inner < 0.0) inner = 0.0;
        if (outer < 0.0) outer = 0.0;
        if (outer < inner) clusterR++;
        else break;
    }

    double clusterEnergy = 0.0;
    for (int i = clusterL; i <= clusterR; ++i){
        double v = scoreMap[i];
        if (v > 0.0) clusterEnergy += v;
    }
    double conf = clusterEnergy / (targetEnergy + 1e-9);
    if (conf < 0.0) conf = 0.0;
    if (conf > 1.0) conf = 1.0;

    // Recompute phase from comb offsets at the exact BPM selected for blink driving.
    double basePhase = 0.0;
    recompute_phase_for_bpm(novelty, numFrames, frameRate, bpmSel,
                            &basePhase, g_phaseCurveSelected, &g_phaseCurveSelectedLen);

    double bpmHz = bpmSel / 60.0;
    double elapsedSec = (double)g_novTotalFrames / frameRate;
    double phaseShift = fmod(elapsedSec * bpmHz, 1.0);
    if (phaseShift < 0.0) phaseShift += 1.0;

    g_mainBpm = bpmSel;
    g_mainBpmConf = conf;
    g_mainBpmPhase = wrap_phase01(basePhase + phaseShift);

    if (g_phaseCurveSelectedLen > 0){
        shift_phase_curve_circular(g_phaseCurveSelected, g_phaseCurveSelectedLen, phaseShift);
    }
}

// ---------------- FFT (radix-2 Cooley–Tukey) ----------------

static void fft(double *real, double *imag, int n, int inverse) {
    int i, j;

    // Bit-reversal
    j = 0;
    for (i = 0; i < n; ++i) {
        if (i < j) {
            double tr = real[i], ti = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = tr;
            imag[j] = ti;
        }
        int m = n >> 1;
        while (m > 0 && j >= m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    double angSign = inverse ? 2.0 * M_PI : -2.0 * M_PI;
    for (int len = 2; len <= n; len <<= 1) {
        double ang = angSign / (double)len;
        double wlen_r = cos(ang);
        double wlen_i = sin(ang);
        for (i = 0; i < n; i += len) {
            double wr = 1.0, wi = 0.0;
            for (j = 0; j < len / 2; ++j) {
                int u = i + j;
                int v = i + j + len / 2;
                double tr = wr * real[v] - wi * imag[v];
                double ti = wr * imag[v] + wi * real[v];
                real[v] = real[u] - tr;
                imag[v] = imag[u] - ti;
                real[u] += tr;
                imag[u] += ti;

                double nwr = wr * wlen_r - wi * wlen_i;
                double nwi = wr * wlen_i + wi * wlen_r;
                wr = nwr;
                wi = nwi;
            }
        }
    }

    if (inverse) {
        for (i = 0; i < n; ++i) {
            real[i] /= (double)n;
            imag[i] /= (double)n;
        }
    }
}

// ---------------- ALSA capture ----------------

static int setup_alsa_capture(snd_pcm_t **handle) {
    snd_pcm_t *h = NULL;
    snd_pcm_hw_params_t *params;
    int rc;
    unsigned int rate = SAMPLE_RATE;
    int dir = 0;
    snd_pcm_uframes_t frames = FFT_SIZE;

    rc = snd_pcm_open(&h, "default", SND_PCM_STREAM_CAPTURE, 0);
    if (rc < 0) {
        fprintf(stderr, "unable to open capture device: %s\n", snd_strerror(rc));
        return rc;
    }

    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(h, params);
    snd_pcm_hw_params_set_access(h, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(h, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(h, params, CHANNELS);
    snd_pcm_hw_params_set_rate_near(h, params, &rate, &dir);
    snd_pcm_hw_params_set_period_size_near(h, params, &frames, &dir);

    rc = snd_pcm_hw_params(h, params);
    if (rc < 0) {
        fprintf(stderr, "unable to set capture hw parameters: %s\n", snd_strerror(rc));
        snd_pcm_close(h);
        return rc;
    }

    *handle = h;
    return 0;
}

// ---------------- Drawing helpers ----------------

// Top-right: phase correlation curve for the selected (blink-driving) BPM
static void draw_phase_panel(SDL_Renderer *ren, int x0, int y0, int w, int h,
                             const double *phaseCurve, int phaseCount,
                             double phaseNow, double bpmNow, double confNow) {
    SDL_Rect rect = { x0, y0, w, h };
    SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
    SDL_RenderFillRect(ren, &rect);

    if (!phaseCurve || phaseCount < 2) return;

    double maxV = 0.0;
    for (int i = 0; i < phaseCount; ++i){
        if (phaseCurve[i] > maxV) maxV = phaseCurve[i];
    }
    if (maxV < 1e-9) maxV = 1e-9;

    int prevX = x0;
    int prevY = y0 + h - 1;
    for (int px = 0; px < w; ++px){
        double t = (w > 1) ? ((double)px / (double)(w - 1)) : 0.0;
        double pos = t * (double)(phaseCount - 1);
        int i0 = (int)floor(pos);
        int i1 = i0 + 1;
        if (i1 >= phaseCount) i1 = phaseCount - 1;
        double frac = pos - (double)i0;
        double v = phaseCurve[i0] * (1.0 - frac) + phaseCurve[i1] * frac;
        double norm = v / maxV;
        if (norm < 0.0) norm = 0.0;
        if (norm > 1.0) norm = 1.0;

        int x = x0 + px;
        int y = y0 + h - 1 - (int)lround(norm * (double)(h - 1));
        SDL_SetRenderDrawColor(ren, 0, 220, 255, 255);
        SDL_RenderDrawLine(ren, prevX, prevY, x, y);
        prevX = x;
        prevY = y;
    }

    double ph = wrap_phase01(phaseNow);
    int xMark = x0 + (int)lround(ph * (double)(w - 1));
    if (xMark < x0) xMark = x0;
    if (xMark >= x0 + w) xMark = x0 + w - 1;
    SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
    SDL_RenderDrawLine(ren, xMark, y0, xMark, y0 + h - 1);

    (void)bpmNow;
    (void)confNow;
}

// Bottom-left: 1D signal vs time (novelty)
static void draw_intensity_panel(SDL_Renderer *ren, int x0, int y0, int w, int h,
                          const double *values, int numFrames) {
    SDL_Rect rect = { x0, y0, w, h };
    SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
    SDL_RenderFillRect(ren, &rect);

    if (numFrames < 2) return;

    int prevX = x0;
    int prevY = y0 + h - 1;

    // For each x-pixel column, take the MAX novelty value from frames that map into it
    for (int px = 0; px < w; ++px) {
        int i0 = (int)floor((double)px     * (numFrames - 1) / (double)(w - 1));
        int i1 = (int)floor((double)(px+1) * (numFrames - 1) / (double)(w - 1));
        if (i1 < i0) i1 = i0;

        double vmax = 0.0;
        for (int i = i0; i <= i1; ++i) if (values[i] > vmax) vmax = values[i];

        int x = x0 + px;
        int y = y0 + h - 1 - (int)((vmax / (double)NOVELTY_SCALE) * (double)(h - 10));

        SDL_SetRenderDrawColor(ren, 0, 200, 255, 255);
        SDL_RenderDrawLine(ren, prevX, prevY, x, y);
        prevX = x;
        prevY = y;
    }
}

// Bottom-right: BPM spectrum + phase strip
static void draw_bpm_panel(SDL_Renderer *ren, int x0, int y0, int w, int h,
                    const double *specToDraw, const double *bpmPhase,
                    double phaseTime) {
    SDL_Rect rect = { x0, y0, w, h };
    SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
    SDL_RenderFillRect(ren, &rect);

    int dispBpmMin = TARGET_BPM_MIN;
    int dispBpmMax = TARGET_BPM_MAX;
    if (dispBpmMin < BPM_MIN) dispBpmMin = BPM_MIN;
    if (dispBpmMax > BPM_MAX) dispBpmMax = BPM_MAX;
    int dispBpmCount = dispBpmMax - dispBpmMin + 1;
    if (dispBpmCount <= 0) return;

    int barsHeight = (int)(h * 0.75);
    if (barsHeight < 10) barsHeight = h - 5;
    int phaseHeight = h - barsHeight;
    if (phaseHeight < 4) phaseHeight = 4;
    barsHeight = h - phaseHeight;

    int barsBottom = y0 + barsHeight - 1;
    int phaseTop = barsBottom + 1;
    int phaseBottom = y0 + h - 1;

    // --- Auto scale based on what we're drawing (ESP-like, smoothed) ---
    double maxV = 0.0;
    for (int i = 0; i < dispBpmCount; ++i){
        int bpm = dispBpmMin + i;
        int bin = bpm - BPM_MIN;
        double v = specToDraw[bin];
        if (v > maxV) maxV = v;
    }
    if (maxV < g_bpm_display_floor) maxV = g_bpm_display_floor;

    double target = maxV * BPM_HEADROOM;
    if (target > g_bpm_display_scale) {
        g_bpm_display_scale = (1.0 - BPM_SCALE_SMOOTH_UP)   * g_bpm_display_scale
                            + (BPM_SCALE_SMOOTH_UP)         * target;
    } else {
        g_bpm_display_scale = (1.0 - BPM_SCALE_SMOOTH_DOWN) * g_bpm_display_scale
                            + (BPM_SCALE_SMOOTH_DOWN)       * target;
    }
    double scaleMax = (g_bpm_display_scale > g_bpm_display_floor) ? g_bpm_display_scale : g_bpm_display_floor;

    double step = (double)w / (double)dispBpmCount;

    for (int i = 0; i < dispBpmCount; ++i) {
        int bpm = dispBpmMin + i;
        int bin = bpm - BPM_MIN;
        double v = specToDraw[bin];
        if (v < 0.0) v = 0.0;

        double m = v / scaleMax;
        if (m < 0.0) m = 0.0;
        if (m > 1.0) m = 1.0;

        int bar = (int)(m * (barsHeight - 4));

        int xStart = x0 + (int)lround(i * step);
        int xEnd = x0 + (int)lround((i + 1) * step) - 1;
        if (xEnd < xStart) xEnd = xStart;

        // 1 color per 10 BPM band
        int r=255, g=255, b=255;
        switch ((bpm / 10) % 10) {
            case 0: r = 255; g = 255; b = 0;   break;
            case 1: r = 255; g = 128; b = 0;   break;
            case 2: r = 255; g = 0;   b = 0;   break;
            case 3: r = 200; g = 0;   b = 200; break;
            case 4: r = 255; g = 0;   b = 255; break;
            case 5: r = 128; g = 0;   b = 255; break;
            case 6: r = 0;   g = 0;   b = 255; break;
            case 7: r = 0;   g = 128; b = 255; break;
            case 8: r = 0;   g = 255; b = 255; break;
            case 9: r = 128; g = 255; b = 0;   break;
        }

        SDL_SetRenderDrawColor(ren, (Uint8)r, (Uint8)g, (Uint8)b, 255);
        for (int x = xStart; x <= xEnd; ++x) {
            SDL_RenderDrawLine(ren, x, barsBottom, x, barsBottom - bar);
        }

        // phase strip (still uses bpmPhase from comb)
        double basePhase = bpmPhase ? bpmPhase[bin] : 0.0;
        if (basePhase < 0.0) basePhase = 0.0;
        if (basePhase > 1.0) basePhase = 1.0;

        double cycles = phaseTime * ((double)bpm / 60.0);
        double animPhase = basePhase + cycles;
        animPhase -= floor(animPhase);

        int phaseY = phaseTop + (int)(animPhase * (double)(phaseHeight - 1));
        if (phaseY < phaseTop) phaseY = phaseTop;
        if (phaseY > phaseBottom) phaseY = phaseBottom;

        SDL_SetRenderDrawColor(ren, 255, 255, 255, 255);
        for (int x = xStart; x <= xEnd; ++x) {
            SDL_RenderDrawPoint(ren, x, phaseY);
        }
    }

    // Optional main BPM marker (vertical line) if confidence is decent
    if (g_mainBpmConf > 0.25 &&
        g_mainBpm >= (double)dispBpmMin &&
        g_mainBpm <= (double)dispBpmMax) {
        double t = 0.0;
        if (dispBpmCount > 1){
            t = (double)(g_mainBpm - (double)dispBpmMin) /
                (double)(dispBpmCount - 1);
        }
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        int x = x0 + (int)lround(t * (double)(w - 1));
        SDL_SetRenderDrawColor(ren, 255, 0, 0, 255);
        SDL_RenderDrawLine(ren, x, y0, x, barsBottom);
    }
}

// ---------------- Scrolling spectrogram ----------------

static Uint32 *specPixels = NULL;
static SDL_Texture *specTex = NULL;

static void init_scrolling_spectrogram(SDL_Renderer *ren) {
    specPixels = (Uint32 *)malloc(sizeof(Uint32) * SPEC_WIDTH * SPEC_HEIGHT);
    if (!specPixels) {
        fprintf(stderr, "Failed to allocate specPixels\n");
        return;
    }

    for (int y = 0; y < SPEC_HEIGHT; ++y) {
        for (int x = 0; x < SPEC_WIDTH; ++x) {
            specPixels[y * SPEC_WIDTH + x] = make_gray(0);
        }
    }

    specTex = SDL_CreateTexture(
        ren,
        SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING,
        SPEC_WIDTH,
        SPEC_HEIGHT
    );
    if (!specTex) {
        fprintf(stderr, "SDL_CreateTexture for specTex error: %s\n", SDL_GetError());
    }
}

// mags: FFT_SIZE/2 magnitudes for the *current block*
static void update_scrolling_spectrogram(const double *mags) {
    if (!specPixels) return;

    int halfBins = FFT_SIZE / 2;

    // scroll left by 1 pixel
    for (int y = 0; y < SPEC_HEIGHT; ++y) {
        Uint32 *row = &specPixels[y * SPEC_WIDTH];
        memmove(row, row + 1, (SPEC_WIDTH - 1) * sizeof(Uint32));
    }

    int xNew = SPEC_WIDTH - 1;
    for (int y = 0; y < SPEC_HEIGHT; ++y) {
        int i = (y * halfBins) / SPEC_HEIGHT;
        if (i < 0) i = 0;
        if (i >= halfBins) i = halfBins - 1;

        double norm = mags[i] / SPEC_SCROLL_SCALE;
        if (norm > 1.0) norm = 1.0;
        if (norm < 0.0) norm = 0.0;

        double m;
        if (norm <= SPEC_MIN_LEVEL) {
            m = norm * 0.3 / (SPEC_MIN_LEVEL > 0.0 ? SPEC_MIN_LEVEL : 1.0);
        } else {
            m = 0.3 + (norm - SPEC_MIN_LEVEL) * 0.7 / (1.0 - SPEC_MIN_LEVEL);
        }

        m = log10(1.0 + 9.0 * m);
        if (m < 0.0) m = 0.0;
        if (m > 1.0) m = 1.0;

        Uint8 c = (Uint8)(m * 255.0);
        int yy = SPEC_HEIGHT - 1 - y; // low freqs at bottom
        specPixels[yy * SPEC_WIDTH + xNew] = make_gray(c);
    }
}

// ---------------- Comb BPM from novelty ----------------

static void comb_bpm_from_novelty(const double *novelty, int numFrames, double frameRate,
                           double *bpmSpecOut, double *bpmPhaseOut) {
    int bpmCount = BPM_MAX - BPM_MIN + 1;
    if (numFrames <= 1) {
        for (int i = 0; i < bpmCount; ++i) {
            bpmSpecOut[i] = 0.0;
            bpmPhaseOut[i] = 0.0;
        }
        return;
    }

    const double minSamplesPerBpm = 2.0;

    for (int i = 0; i < bpmCount; ++i) {
        int bpm = BPM_MIN + i;
        double step = frameRate * 60.0 / (double)bpm;
        int maxPhase = (int)floor(step);
        if (maxPhase < 1) maxPhase = 1;

        double bestScore = 0.0;
        int bestPhaseIndex = 0;

        for (int phase = 0; phase < maxPhase; ++phase) {
            double pos = (double)phase;
            double sum = 0.0;
            int count = 0;
            while (pos < (double)(numFrames - 1)) {
                int j = (int)floor(pos);
                double t = pos - (double)j;
                double v;
                if (j >= 0 && j + 1 < numFrames) {
                    v = (1.0 - t) * novelty[j] + t * novelty[j + 1];
                } else if (j >= 0 && j < numFrames) {
                    v = novelty[j];
                } else {
                    v = 0.0;
                }
                sum += v;
                count += 1;
                pos += step;
            }
            if (count < (int)minSamplesPerBpm) continue;
            double avg = sum / (double)count;
            double score = avg * (double)count;
            if (score > bestScore) {
                bestScore = score;
                bestPhaseIndex = phase;
            }
        }

        bpmSpecOut[i] = bestScore;
        double phaseNorm = 0.0;
        if (bestScore > 0.0 && maxPhase > 1) {
            phaseNorm = (double)bestPhaseIndex / (double)(maxPhase - 1);
        }
        bpmPhaseOut[i] = phaseNorm;
    }
}

// ---------------- ESP-style incremental novelty ----------------
static inline int clampi(int v, int lo, int hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static double novelty_local_mean_update(double v){
    if (g_localWin <= 0) return v + NOVELTY_MEAN_OFFSET;

    if (g_localCount < g_localWin){
        g_localMeanRing[g_localCount++] = v;
        g_localSum += v;
        return (g_localSum / (double)g_localCount) + NOVELTY_MEAN_OFFSET;
    }
    g_localSum -= g_localMeanRing[g_localIdx];
    g_localMeanRing[g_localIdx] = v;
    g_localSum += v;
    g_localIdx = (g_localIdx + 1) % g_localWin;
    return (g_localSum / (double)g_localWin) + NOVELTY_MEAN_OFFSET;
}

static double novelty_push_cleaned(double raw){
    if (raw > NOVELTY_SPIKE_ZERO_THRESHOLD){
        raw = 0.0;
    }
    double local = novelty_local_mean_update(raw);
    double cleaned = raw - local;
    if (cleaned < 0.0) cleaned = 0.0;
    return cleaned;
}

// ---------------- BPM baseline correction + family scoring ----------------

static void bpm_baseline_correct(const double *bpmSpec, int bpmCount, double *bcOut){
    int midIdx = bpmCount / 2;

    double lowSum = 0.0;
    for (int i = 0; i < midIdx; ++i) lowSum += bpmSpec[i];

    double highSum = 0.0;
    for (int i = midIdx; i < bpmCount; ++i) highSum += bpmSpec[i];

    double lowAvg  = (midIdx > 0) ? (lowSum / (double)midIdx) : 0.0;
    double highAvg = (bpmCount - midIdx > 0) ? (highSum / (double)(bpmCount - midIdx)) : 0.0;

    int x1 = midIdx / 2;
    int x2 = midIdx + (bpmCount - midIdx) / 2;
    if (x2 == x1){ x1 = 0; x2 = bpmCount - 1; }

    double slope = (x2 != x1) ? ((highAvg - lowAvg) / (double)(x2 - x1)) : 0.0;
    double intercept = lowAvg - slope * (double)x1;

    for (int i = 0; i < bpmCount; ++i){
        double base = intercept + slope * (double)i;
        double v = bpmSpec[i] - base;
        if (v < 0.0) v = 0.0;
        bcOut[i] = v;
    }
}

static inline double bpm_bc_at(const double *bc, double bpm){
    if (bpm < (double)BPM_MIN || bpm > (double)BPM_MAX) return 0.0;

    double idx = bpm - (double)BPM_MIN;
    int i0 = (int)floor(idx);
    int i1 = i0 + 1;

    int maxIdx = (BPM_MAX - BPM_MIN);
    if (i0 < 0) i0 = 0;
    if (i0 > maxIdx) i0 = maxIdx;
    if (i1 < 0) i1 = 0;
    if (i1 > maxIdx) i1 = maxIdx;

    double t = idx - (double)i0;
    double a = bc[i0];
    double b = bc[i1];
    return a * (1.0 - t) + b * t;
}

static void bpm_family_spectrum_from_bc(const double *bc, int bpmCount, double *famOut){
    const double wHalf = 0.50;
    const double w1    = 1.00;
    const double w2    = 0.65;
    const double w4    = 0.45;

    for (int i = 0; i < bpmCount; ++i){
        double F = (double)(BPM_MIN + i);

        double s = 0.0;
        s += wHalf * bpm_bc_at(bc, 0.5 * F);
        s += w1    * bpm_bc_at(bc, F);
        s += w2    * bpm_bc_at(bc, 2.0 * F);
        s += w4    * bpm_bc_at(bc, 4.0 * F);

        famOut[i] = s;
    }
}

// ---------------- main ----------------

int main(void) {
    int rc = 0;
    const int halfBins = FFT_SIZE / 2;
    const double frameRate = (double)SAMPLE_RATE / (double)HOP_SIZE;
    int maxFrames = (int)floor(RECORD_SECONDS * frameRate + 0.5);
    if (maxFrames < 4) maxFrames = 4;

    printf("Using ~%d frames (~%.2f seconds) at %.2f fps\n",
           maxFrames, (double)maxFrames / frameRate, frameRate);

    snd_pcm_t *cap_handle = NULL;
    int16_t *captureBuf = NULL;
    SDL_Window *win = NULL;
    SDL_Renderer *ren = NULL;

    double *hann = NULL;
    double *specReal = NULL, *specImag = NULL, *specMags = NULL;
    double *logSpecRing = NULL;
    double *novelty = NULL;
    double *bpmSpec = NULL, *bpmPhase = NULL, *bcSpec = NULL, *famSpec = NULL;

    // ---------- ESP-style incremental novelty init ----------
    g_prevLogmag = (double*)calloc((size_t)halfBins, sizeof(double));
    if (!g_prevLogmag) { fprintf(stderr, "Failed to allocate g_prevLogmag\n"); rc = 1; goto cleanup; }

    g_novCap = maxFrames;
    g_noveltyRing = (double*)calloc((size_t)g_novCap, sizeof(double));
    if (!g_noveltyRing) { fprintf(stderr, "Failed to allocate novelty ring\n"); rc = 1; goto cleanup; }

    g_localWin = (int)lrint(frameRate * 0.5);
    if (g_localWin < 1) g_localWin = 1;
    if (g_localWin > maxFrames) g_localWin = maxFrames;
    g_localMeanRing = (double*)calloc((size_t)g_localWin, sizeof(double));
    if (!g_localMeanRing) { fprintf(stderr, "Failed to allocate novelty local mean ring\n"); rc = 1; goto cleanup; }

    // Hann window
    hann = (double *)malloc(sizeof(double) * FFT_SIZE);
    if (!hann) { fprintf(stderr, "Failed to allocate Hann\n"); rc = 1; goto cleanup; }
    for (int i = 0; i < FFT_SIZE; ++i) {
        hann[i] = 0.5 * (1.0 - cos(2.0 * M_PI * i / (FFT_SIZE - 1)));
    }

    // FFT buffers
    specReal = (double *)malloc(sizeof(double) * FFT_SIZE);
    specImag = (double *)malloc(sizeof(double) * FFT_SIZE);
    specMags = (double *)malloc(sizeof(double) * halfBins);
    if (!specReal || !specImag || !specMags) { fprintf(stderr, "Failed to allocate FFT buffers\n"); rc = 1; goto cleanup; }

    // log-mag ring
    logSpecRing = (double *)malloc(sizeof(double) * (size_t)maxFrames * (size_t)halfBins);
    novelty = (double *)malloc(sizeof(double) * (size_t)maxFrames);
    if (!logSpecRing || !novelty) { fprintf(stderr, "Failed to allocate logSpec/novelty buffers\n"); rc = 1; goto cleanup; }

    const int bpmCount = BPM_MAX - BPM_MIN + 1;
    bpmSpec = (double *)malloc(sizeof(double) * (size_t)bpmCount);
    bpmPhase = (double *)malloc(sizeof(double) * (size_t)bpmCount);
    bcSpec  = (double *)malloc(sizeof(double) * (size_t)bpmCount);
    famSpec = (double *)malloc(sizeof(double) * (size_t)bpmCount);
    if (!bpmSpec || !bpmPhase || !bcSpec || !famSpec) { fprintf(stderr, "Failed to allocate BPM buffers\n"); rc = 1; goto cleanup; }

    for (int i = 0; i < bpmCount; ++i) {
        bpmSpec[i] = 0.0;
        bpmPhase[i] = 0.0;
        bcSpec[i] = 0.0;
        famSpec[i] = 0.0;
    }

    if (g_famSpecSmoothCap != bpmCount) {
        free(g_famSpecSmooth);
        g_famSpecSmooth = (double*)calloc((size_t)bpmCount, sizeof(double));
        g_famSpecSmoothCap = bpmCount;
    }
    if (!g_famSpecSmooth) { fprintf(stderr, "Failed to allocate g_famSpecSmooth\n"); rc = 1; goto cleanup; }

    // ALSA capture
    rc = setup_alsa_capture(&cap_handle);
    if (rc < 0) { rc = 1; goto cleanup; }

    captureBuf = (int16_t *)malloc(sizeof(int16_t) * FFT_SIZE);
    if (!captureBuf) { fprintf(stderr, "Failed to allocate capture buffer\n"); rc = 1; goto cleanup; }

    // Optional warm-up
    {
        printf("Warming up microphone (2 seconds)...\n");
        int warmSamples = SAMPLE_RATE * 2;
        int16_t dummyBuf[FFT_SIZE];
        int warmRemaining = warmSamples;
        while (warmRemaining > 0) {
            int toRead = warmRemaining < FFT_SIZE ? warmRemaining : FFT_SIZE;
            int rrc = snd_pcm_readi(cap_handle, dummyBuf, toRead);
            if (rrc == -EPIPE) { snd_pcm_prepare(cap_handle); continue; }
            if (rrc < 0) { fprintf(stderr, "Warm-up read error: %s\n", snd_strerror(rrc)); break; }
            warmRemaining -= (rrc > 0 ? rrc : 0);
        }
        printf("Microphone ready.\n");
    }

    // SDL init
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init error: %s\n", SDL_GetError());
        rc = 1; goto cleanup;
    }

    win = SDL_CreateWindow(
        "Rolling 6s FFT (spectro + novelty + BPM family)",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIN_WIDTH, WIN_HEIGHT,
        SDL_WINDOW_SHOWN
    );
    if (!win) { fprintf(stderr, "SDL_CreateWindow error: %s\n", SDL_GetError()); rc = 1; goto cleanup; }

    ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    if (!ren) { fprintf(stderr, "SDL_CreateRenderer error: %s\n", SDL_GetError()); rc = 1; goto cleanup; }

    init_scrolling_spectrogram(ren);

    printf("Controls: Esc to quit.\n");

    int running = 1;
    int numFramesLast = 0;
    int logWritePos = 0;
    int logCount = 0;

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = 0;
            else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) running = 0;
        }

        rc = read_exact_frames(cap_handle, captureBuf, FFT_SIZE);
        if (rc < 0) { fprintf(stderr, "read_exact_frames failed: %d\n", rc); break; }

        for (int i = 0; i < FFT_SIZE; ++i) {
            double s = captureBuf[i] / 32768.0;
            specReal[i] = s * hann[i];
            specImag[i] = 0.0;
        }
        fft(specReal, specImag, FFT_SIZE, 0);

        for (int k = 0; k < halfBins; ++k) {
            double rre = specReal[k];
            double iim = specImag[k];
            specMags[k] = sqrt(rre * rre + iim * iim);
        }

        update_scrolling_spectrogram(specMags);

        // Store log-mag in ring
        {
            double *dst = &logSpecRing[logWritePos * halfBins];
            for (int k = 0; k < halfBins; ++k) {
                dst[k] = log(1.0 + 10.0 * specMags[k]);
            }
            logWritePos = (logWritePos + 1) % maxFrames;
            if (logCount < maxFrames) logCount++;
        }

        // Novelty + BPM
        {
            const double fs = (double)SAMPLE_RATE;
            int kLow = (int)ceil(100.0 * (double)FFT_SIZE / fs);
            int kHigh = halfBins - 1;
            kLow = clampi(kLow, 0, halfBins - 1);
            kHigh = clampi(kHigh, 0, halfBins - 1);
            if (kHigh < kLow) kHigh = kLow;

            double rawNov = 0.0;
            double *currLog = &logSpecRing[((logWritePos - 1 + maxFrames) % maxFrames) * halfBins];

            if (!g_prevLogmagValid){
                memcpy(g_prevLogmag, currLog, sizeof(double) * (size_t)halfBins);
                g_prevLogmagValid = 1;
                rawNov = 0.0;
            } else {
                for (int k = kLow; k <= kHigh; ++k){
                    double d = currLog[k] - g_prevLogmag[k];
                    if (d > 0.0) rawNov += d;
                    g_prevLogmag[k] = currLog[k];
                }
            }

            double cleaned = novelty_push_cleaned(rawNov);

            g_noveltyRing[g_novWritePos] = cleaned;
            g_novWritePos = (g_novWritePos + 1) % g_novCap;
            if (g_novCount < g_novCap) g_novCount++;
            g_novTotalFrames++;

            int numFrames = g_novCount;
            int startIdx = g_novWritePos - numFrames;
            while (startIdx < 0) startIdx += g_novCap;

            for (int i = 0; i < numFrames; ++i){
                novelty[i] = g_noveltyRing[(startIdx + i) % g_novCap];
            }

            if (numFrames >= 3){
                comb_bpm_from_novelty(novelty, numFrames, frameRate, bpmSpec, bpmPhase);

                bpm_baseline_correct(bpmSpec, bpmCount, bcSpec);
                bpm_family_spectrum_from_bc(bcSpec, bpmCount, famSpec);

                double dt = 1.0 / frameRate;
                smooth_spectrum_asym_tau(famSpec, g_famSpecSmooth, bpmCount,
                                         dt, FAMSPEC_TAU_UP_SEC, FAMSPEC_TAU_DOWN_SEC);

                // Select BPM/phase exactly from the score map that drives the marker.
                select_main_bpm_phase_from_scores(g_famSpecSmooth, novelty, numFrames,
                                                  frameRate);

                numFramesLast = numFrames;
            } else {
                g_mainBpm = 0.0;
                g_mainBpmConf = 0.0;
                g_mainBpmPhase = 0.0;
                g_phaseCurveSelectedLen = 0;
                numFramesLast = numFrames;
            }
        }

        g_phaseTime += (double)FFT_SIZE / (double)SAMPLE_RATE;

        int w2 = WIN_WIDTH / 2;
        int h2 = WIN_HEIGHT / 2;

        SDL_SetRenderDrawColor(ren, 0, 0, 0, 255);
        SDL_RenderClear(ren);

        if (specTex && specPixels) {
            SDL_Rect src = { 0, 0, SPEC_WIDTH, SPEC_HEIGHT };
            SDL_Rect dst = { 0, 0, w2, h2 };
            SDL_UpdateTexture(specTex, NULL, specPixels, SPEC_WIDTH * (int)sizeof(Uint32));
            SDL_RenderCopy(ren, specTex, &src, &dst);
        }

        draw_phase_panel(ren, w2, 0, w2, h2,
                         g_phaseCurveSelected, g_phaseCurveSelectedLen,
                         g_mainBpmPhase, g_mainBpm, g_mainBpmConf);
        draw_intensity_panel(ren, 0, h2, w2, h2, novelty, numFramesLast);
        draw_bpm_panel(ren, w2, h2, w2, h2, g_famSpecSmooth, bpmPhase, g_phaseTime);

        SDL_RenderPresent(ren);
    }

cleanup:
    if (captureBuf) free(captureBuf);
    if (cap_handle) snd_pcm_close(cap_handle);

    if (specTex) SDL_DestroyTexture(specTex);
    if (specPixels) free(specPixels);
    if (ren) SDL_DestroyRenderer(ren);
    if (win) SDL_DestroyWindow(win);
    if (SDL_WasInit(SDL_INIT_VIDEO)) SDL_Quit();

    free(hann);
    free(specReal);
    free(specImag);
    free(specMags);
    free(logSpecRing);
    free(novelty);
    free(bpmSpec);
    free(bpmPhase);
    free(bcSpec);
    free(famSpec);

    free(g_famSpecSmooth);

    free(g_prevLogmag);
    free(g_noveltyRing);
    free(g_localMeanRing);

    return rc ? 1 : 0;
}
