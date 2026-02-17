#include "shell_profiler.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_attr.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_SHELL_PERF_LOG_ENABLE

static const char *TAG = "shell_perf";

#if (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1)
#define SHELL_PERF_HAS_RT_STATS 1
#define SHELL_PERF_TOP_LOG_EVERY 5U
#else
#define SHELL_PERF_HAS_RT_STATS 0
#endif

static int64_t s_last_log_us = 0;
#if !SHELL_PERF_HAS_RT_STATS
static bool s_warned_missing_rt_stats = false;
#endif

static char s_app_id[20] = {0};
static bool s_app_external = false;

static uint64_t s_sum_tick_us = 0;
static uint64_t s_sum_draw_us = 0;
static uint64_t s_sum_work_us = 0;
static uint32_t s_frame_count = 0;

#if SHELL_PERF_HAS_RT_STATS
typedef struct {
    TaskHandle_t handle;
    configRUN_TIME_COUNTER_TYPE runtime;
} task_prev_t;

typedef struct {
    TaskStatus_t task;
    configRUN_TIME_COUNTER_TYPE delta;
} task_delta_t;

static EXT_RAM_BSS_ATTR TaskStatus_t s_tasks[CONFIG_SHELL_PERF_LOG_MAX_TASKS];
static EXT_RAM_BSS_ATTR task_prev_t s_prev[CONFIG_SHELL_PERF_LOG_MAX_TASKS];
static EXT_RAM_BSS_ATTR task_delta_t s_deltas[CONFIG_SHELL_PERF_LOG_MAX_TASKS];
static EXT_RAM_BSS_ATTR bool s_used[CONFIG_SHELL_PERF_LOG_MAX_TASKS];
static size_t s_prev_count = 0;
static uint32_t s_stats_log_count = 0;

static configRUN_TIME_COUNTER_TYPE diff_counter(configRUN_TIME_COUNTER_TYPE now,
                                                configRUN_TIME_COUNTER_TYPE prev)
{
    // Unsigned subtraction handles wraparound for uint32_t/uint64_t counters.
    return now - prev;
}

static bool prev_lookup(TaskHandle_t handle, configRUN_TIME_COUNTER_TYPE *out_runtime)
{
    if (!out_runtime) return false;
    for (size_t i = 0; i < s_prev_count; ++i) {
        if (s_prev[i].handle == handle) {
            *out_runtime = s_prev[i].runtime;
            return true;
        }
    }
    return false;
}

static UBaseType_t capture_tasks(TaskStatus_t *out,
                                 UBaseType_t cap,
                                 configRUN_TIME_COUNTER_TYPE *out_total_runtime)
{
    if (!out || cap == 0) return 0;
    UBaseType_t wanted = uxTaskGetNumberOfTasks();
    if (wanted > cap) wanted = cap;
    return uxTaskGetSystemState(out, wanted, out_total_runtime);
}

static void update_prev_from_snapshot(const TaskStatus_t *tasks, UBaseType_t n)
{
    if (!tasks) return;
    if (n > CONFIG_SHELL_PERF_LOG_MAX_TASKS) {
        n = CONFIG_SHELL_PERF_LOG_MAX_TASKS;
    }
    s_prev_count = (size_t)n;
    for (UBaseType_t i = 0; i < n; ++i) {
        s_prev[i].handle = tasks[i].xHandle;
        s_prev[i].runtime = tasks[i].ulRunTimeCounter;
    }
}

static void log_task_core_stats(bool log_top_tasks)
{
    configRUN_TIME_COUNTER_TYPE total_runtime = 0;
    UBaseType_t n = capture_tasks(s_tasks, CONFIG_SHELL_PERF_LOG_MAX_TASKS, &total_runtime);
    (void)total_runtime;
    if (n == 0) {
        ESP_LOGW(TAG, "task stats snapshot failed");
        return;
    }
    if (n >= CONFIG_SHELL_PERF_LOG_MAX_TASKS) {
        ESP_LOGW(TAG, "task stats truncated at %d tasks", CONFIG_SHELL_PERF_LOG_MAX_TASKS);
    }

    uint64_t total_delta = 0;
    uint64_t core_total[2] = {0, 0};
    uint64_t core_idle[2] = {0, 0};

    for (UBaseType_t i = 0; i < n; ++i) {
        s_deltas[i].task = s_tasks[i];
        s_deltas[i].delta = 0;

        configRUN_TIME_COUNTER_TYPE prev_runtime = 0;
        if (prev_lookup(s_tasks[i].xHandle, &prev_runtime)) {
            s_deltas[i].delta = diff_counter(s_tasks[i].ulRunTimeCounter, prev_runtime);
        }
        total_delta += s_deltas[i].delta;

#if CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
        int core = s_tasks[i].xCoreID;
        if (core >= 0 && core < 2) {
            core_total[core] += s_deltas[i].delta;
            if (strncmp(s_tasks[i].pcTaskName, "IDLE", 4) == 0) {
                core_idle[core] += s_deltas[i].delta;
            }
        }
#endif
    }

    update_prev_from_snapshot(s_tasks, n);

    if (total_delta == 0) {
        ESP_LOGW(TAG, "task stats delta is zero; waiting for next sample");
        return;
    }

#if CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
    uint32_t util0 = (core_total[0] > 0)
        ? (uint32_t)(((core_total[0] - core_idle[0]) * 100u) / core_total[0])
        : 0;
    uint32_t util1 = (core_total[1] > 0)
        ? (uint32_t)(((core_total[1] - core_idle[1]) * 100u) / core_total[1])
        : 0;
    ESP_LOGI(TAG, "core load: c0=%" PRIu32 "%% c1=%" PRIu32 "%%", util0, util1);
#else
    ESP_LOGI(TAG, "core load unavailable (enable CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID)");
#endif

    if (!log_top_tasks) {
        return;
    }

    memset(s_used, 0, sizeof(s_used));
    int top_count = CONFIG_SHELL_PERF_LOG_TOP_TASKS;
    if (top_count > (int)n) top_count = (int)n;

    for (int rank = 0; rank < top_count; ++rank) {
        int best = -1;
        configRUN_TIME_COUNTER_TYPE best_delta = 0;
        for (UBaseType_t i = 0; i < n; ++i) {
            if (s_used[i]) continue;
            if (s_deltas[i].delta > best_delta) {
                best = (int)i;
                best_delta = s_deltas[i].delta;
            }
        }
        if (best < 0 || best_delta == 0) break;
        s_used[best] = true;
        uint32_t pct = (uint32_t)((best_delta * 100u) / total_delta);
#if CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
        ESP_LOGI(TAG, "top[%d] %s core=%d load=%" PRIu32 "%% (delta=%llu)",
                 rank + 1,
                 s_deltas[best].task.pcTaskName,
                 (int)s_deltas[best].task.xCoreID,
                 pct,
                 (unsigned long long)best_delta);
#else
        ESP_LOGI(TAG, "top[%d] %s load=%" PRIu32 "%% (delta=%llu)",
                 rank + 1,
                 s_deltas[best].task.pcTaskName,
                 pct,
                 (unsigned long long)best_delta);
#endif
    }
}
#endif // SHELL_PERF_HAS_RT_STATS

static void reset_frame_accum(const char *app_id, bool app_external)
{
    s_sum_tick_us = 0;
    s_sum_draw_us = 0;
    s_sum_work_us = 0;
    s_frame_count = 0;
    s_app_external = app_external;

    const char *src = app_id ? app_id : "none";
    strncpy(s_app_id, src, sizeof(s_app_id) - 1);
    s_app_id[sizeof(s_app_id) - 1] = '\0';
}

#endif // CONFIG_SHELL_PERF_LOG_ENABLE

void shell_profiler_init(void)
{
#if CONFIG_SHELL_PERF_LOG_ENABLE
    s_last_log_us = esp_timer_get_time();
    reset_frame_accum("none", false);
#if SHELL_PERF_HAS_RT_STATS
    configRUN_TIME_COUNTER_TYPE total_runtime = 0;
    UBaseType_t n = capture_tasks(s_tasks, CONFIG_SHELL_PERF_LOG_MAX_TASKS, &total_runtime);
    (void)total_runtime;
    update_prev_from_snapshot(s_tasks, n);
#endif
    ESP_LOGI(TAG, "enabled (period=%dms)", CONFIG_SHELL_PERF_LOG_PERIOD_MS);
#endif
}

void shell_profiler_on_frame(const char *app_id,
                             bool app_external,
                             uint32_t frame_budget_us,
                             uint32_t tick_us,
                             uint32_t draw_us,
                             uint32_t work_us)
{
#if CONFIG_SHELL_PERF_LOG_ENABLE
    if (!app_id) app_id = "none";

    if (strncmp(s_app_id, app_id, sizeof(s_app_id)) != 0 || s_app_external != app_external) {
        reset_frame_accum(app_id, app_external);
    }

    s_sum_tick_us += tick_us;
    s_sum_draw_us += draw_us;
    s_sum_work_us += work_us;
    s_frame_count++;

    int64_t now_us = esp_timer_get_time();
    int64_t period_us = (int64_t)CONFIG_SHELL_PERF_LOG_PERIOD_MS * 1000LL;
    if (s_last_log_us != 0 && (now_us - s_last_log_us) < period_us) {
        return;
    }

    if (s_frame_count == 0) {
        s_last_log_us = now_us;
        return;
    }

    double avg_tick_ms = (double)s_sum_tick_us / (double)s_frame_count / 1000.0;
    double avg_draw_ms = (double)s_sum_draw_us / (double)s_frame_count / 1000.0;
    double avg_work_ms = (double)s_sum_work_us / (double)s_frame_count / 1000.0;
    double fps = (now_us > s_last_log_us)
        ? ((double)s_frame_count * 1000000.0 / (double)(now_us - s_last_log_us))
        : 0.0;
    double budget_ms = (double)frame_budget_us / 1000.0;
    double budget_pct = (frame_budget_us > 0)
        ? ((double)s_sum_work_us * 100.0 / ((double)s_frame_count * (double)frame_budget_us))
        : 0.0;

    ESP_LOGI(TAG,
             "app=%s ext=%d fps=%.1f work=%.2fms (%.1f%% of %.2fms) tick=%.2fms draw=%.2fms",
             s_app_id,
             s_app_external ? 1 : 0,
             fps,
             avg_work_ms,
             budget_pct,
             budget_ms,
             avg_tick_ms,
             avg_draw_ms);

#if SHELL_PERF_HAS_RT_STATS
    s_stats_log_count++;
    bool log_top_tasks = (s_stats_log_count % SHELL_PERF_TOP_LOG_EVERY) == 0;
    log_task_core_stats(log_top_tasks);
#else
    if (!s_warned_missing_rt_stats) {
        ESP_LOGW(TAG,
                 "per-task/core load disabled; enable CONFIG_FREERTOS_USE_TRACE_FACILITY "
                 "and CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS");
        s_warned_missing_rt_stats = true;
    }
#endif

    s_last_log_us = now_us;
    reset_frame_accum(app_id, app_external);
#else
    (void)app_id;
    (void)app_external;
    (void)frame_budget_us;
    (void)tick_us;
    (void)draw_us;
    (void)work_us;
#endif
}
