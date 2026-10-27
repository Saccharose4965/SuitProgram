#include "twenty48_game.h"
#include "scores.h"
#include "esp_random.h"
#include "esp_log.h"
#include <string.h>
static t48_state_t s;
static bool g_high_dirty;
static const char *TAG = "twenty48";
static const char *kScoreKey = "twenty48";
// mapcoord from the Processing sketch
static inline int mapcoord(int dir, int i, int j){
    int m = dir & 3;
    if (m == 1 || m == 3) return i;
    return (m == 0) ? (3 - j) : j;
}

static void copy_grid(int dst[4][4], int src[4][4]){
    memcpy(dst, src, sizeof(int) * 16);
}

static bool equals_grid(int aG[4][4], int bG[4][4]){
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (aG[i][j] != bG[i][j]) return false;
    return true;
}

static void slide(void){
    memset(s.motion, 0, sizeof(s.motion));
    memset(s.grid_new, 0, sizeof(s.grid_new));
    for (int line = 0; line < 4; ++line){
        int shift = 0, lastValue = 0;
        for (int pos = 0; pos < 4; ++pos){
            int col = mapcoord(s.direction, line, pos);
            int row = mapcoord(s.direction + 3, line, pos);
            int value = s.grid_old[row][col]; // exponent form
            if (value == 0){
                shift++;
                continue;
            }
            int targetPos = pos - shift;                    // target index after sliding
            bool merge = (lastValue == value);              // can we merge?
            s.motion[row][col] += shift + (merge ? 1 : 0);    // distance moved (+1 if merge)
            int mergeOffset = merge ? 1 : 0;                // merge lands one earlier
            int tcol = mapcoord(s.direction, line, targetPos - mergeOffset);
            int trow = mapcoord(s.direction + 3, line, targetPos - mergeOffset);
            s.grid_new[trow][tcol] += merge ? 1 : value;       // merged cell (increment exponent) or placed exponent
            if (merge && value < 31){
                uint32_t inc = (1u << (value + 1));
                s.score += inc;
                if (s.score > s.best) {
                    s.best = s.score;
                    g_high_dirty = true;
                }
            }
            lastValue = merge ? 0 : value;                   // reset after merge
            shift += merge ? 1 : 0;                          // merged cell opens a gap
        }
    }
}

static void addtile(void){
    int tile = (esp_random() % 10 == 0) ? 2 : 1; // exponent: 1=2, 2=4
    int c = 0;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (s.grid_new[i][j] == 0) c++;
    if (c == 0) return;
    int r = (int)(esp_random() % (uint32_t)c);
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j){
            if (s.grid_new[i][j] != 0) continue;
            if (r == 0){
                s.grid_new[i][j] = tile;
                return;
            }
            r--;
        }
    }
}

void t48_game_init(void){
    memset(s.grid_old, 0, sizeof(s.grid_old));
    memset(s.grid_new, 0, sizeof(s.grid_new));
    memset(s.motion, 0, sizeof(s.motion));
    s.score = 0;
    g_high_dirty = false;
    uint32_t saved_high = 0;
    if (scores_load_high(kScoreKey, &saved_high) == ESP_OK) {
        s.best = saved_high;
    } else {
        s.best = 0;
        ESP_LOGW(TAG, "Failed to load high score");
    }
    addtile();
    addtile();
    copy_grid(s.grid_old, s.grid_new);
}

void t48_game_deinit(void){
    if (!g_high_dirty) return;

    uint32_t high_out = 0;
    if (scores_update_high(kScoreKey, s.best, &high_out) == ESP_OK) {
        s.best = high_out;
        g_high_dirty = false;
    } else {
        ESP_LOGW(TAG, "Failed to save high score");
    }
}


bool t48_game_move(int direction)
{
    if (direction < 0 || direction > 3) return false;
    s.direction = direction;
    slide();
    if (equals_grid(s.grid_old, s.grid_new)) return false;
    addtile();
    return true;
}
void t48_game_commit(void) { copy_grid(s.grid_old, s.grid_new); }

bool t48_game_is_over(void)
{
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            int value = s.grid_old[row][col];
            if (value == 0) return false;
            if (col + 1 < 4 && value == s.grid_old[row][col + 1]) return false;
            if (row + 1 < 4 && value == s.grid_old[row + 1][col]) return false;
        }
    }
    return true;
}

const t48_state_t *t48_game_state(void) { return &s; }
