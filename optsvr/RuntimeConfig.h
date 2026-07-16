#pragma once

#include <cstdlib>
#include <string>

namespace optsvr {

inline constexpr float GAME_FLOW_SOFT_OVER_CAP_GROWTH = 1.20f;
inline constexpr float GAME_DIR_FLOW_DYNAMIC_CAP_LOW = 0.01f;
inline constexpr float GAME_DIR_FLOW_DYNAMIC_CAP_HIGH = 0.03f;
inline constexpr float GAME_DIR_FLOW_DYNAMIC_CAP_INC = 5.0f;
inline constexpr float GAME_DIR_FLOW_DYNAMIC_CAP_DEC = 0.95f;
inline constexpr float GAME_DIR_FLOW_DYNAMIC_CAP_MIN_RATIO = 0.50f;
inline constexpr float GAME_DIR_FLOW_DYNAMIC_CAP_MAX_RATIO = 1.40f;
inline constexpr float GAME_DIR_FLOW_DECAY = 0.95f;
inline constexpr float GAME_DIR_FLOW_HORIZON_EXEMPT_RATIO = 0.2f;

// Minimal runtime configuration for the Game-only submission. The retained
// values and environment overrides match the active GAME_DOMAIN path in
// MAIN2026; unrelated domain parameters are deliberately omitted.
struct RuntimeConfig {
    bool skip_test = true;
    int debug_level = 0;
    int num_threads = 0;
    bool timing_trace = false;

    int schedule_mode = 8;
    int planner_mode = 16;
    int executor_mode = 11;
    bool precompute_heuristic = true;
    bool turn_aware_heuristic = false;

    float heap_idle_parking_ratio = 0.0f;
    int heap_idle_parking_seed = 20260627;
    int heap_idle_parking_ring = 0;

    int game_zone_hard_cap = 15;
    int game_exempt_main = 23;
    int game_density_cap = 100;
    int game_dir_threshold = 10;
    int game_route_penalty = 20;
    int game_distance_score_mode = 3;
    float game_pickup_weight = 2.0f;
    bool game_use_bfs_distance = true;
    bool game_planner_multistep_auto = false;
    int game_planner_multistep_auto_count = 25;
    bool game_require_bfs_matrix = true;   // true=矩阵未就绪时终止
    float game_flow_soft_penalty_base = 10.0f;
    int game_global_cand_topk = 32;
    int game_global_max_task_scan = 50;
    float game_flow_soft_over_cap_growth = 1.20f;
    bool game_short_task_throttle = false;
    int game_short_task_test_ticks = 40000;
    int game_short_task_final_open_ticks = 12500;
    float game_short_task_release_ratio = 0.25f;
    bool game_idle_global_match = false;
    int game_idle_global_ticks = 500;
    int game_idle_global_max_agents = 64;
    bool game_score_exempt_enabled = false;
    int game_ex_main_cap = 35;
    int game_ex_nonmain_cap = 80;
    float game_bal_assign_ratio = 1.00f;
    bool game_skip_channel_errands = false;
    bool game_dir_flow_enabled = true;
    bool game_dir_flow_hard_filter = true;
    int game_dir_flow_cap = 80;
    int game_dir_flow_gap = 0;
    float game_dir_flow_decay = 0.95f;
    int game_dir_flow_zone_horizon = 0;
    float game_dir_flow_horizon_exempt_ratio = 0.2f;
    bool game_dir_flow_dynamic_cap = true;
    int game_dir_flow_dynamic_cap_window = 500;
    int game_dir_flow_dynamic_cap_interval = 200;
    float game_dir_flow_dynamic_cap_low = 0.01f;
    float game_dir_flow_dynamic_cap_high = 0.03f;
    float game_dir_flow_dynamic_cap_inc = 5.0f;
    float game_dir_flow_dynamic_cap_dec = 0.95f;
    float game_dir_flow_dynamic_cap_min_ratio = 0.50f;
    float game_dir_flow_dynamic_cap_max_ratio = 1.40f;
    bool game_dir_flow_random_shortest = true;
    bool game_highway_enabled = true;
    bool game_heuristic_hard_direction = true;
    bool game_guide_path_enabled = false;
    bool game_idle_highway_relocate = true;
    bool game_sched_diag = false;
    int game_sched_diag_interval = 100;
    bool game_region_match = true;
    int game_region_flow_batch = 64;
    bool game_prefilter_capacity = true;
    int game_platoon_scope = 3;
    bool game_reassign = false;
    int game_lns_pct = 5;
    int game_min_dist = 10;
    float game_keep_bias = 10.0f;
    bool game_pibt_counter_flow = true;
    int game_counter_flow_cost = 20;
    bool game_urgency_priority = true;
    float game_urgency_weight = 10.0f;
    int game_urgency_threshold = 50;
    int game_search_radius = 3;

    bool dist2path_prewarm_enabled = true;
    int dist2path_prewarm_radius = 3;
    int dist2path_prewarm_threads = 0;
    int dist2path_prewarm_min_left_ms = 50;

    // Compiled only because MAIN's shared Game executor contains an unreachable
    // Iron fallback branch. These values never affect executor_mode=11.
    bool iron_open_grid_enabled = false;
    bool iron_open_grid_rows_enabled = true;
    bool iron_open_grid_cols_enabled = true;
    bool iron_open_grid_exclusive_axis = true;
    int iron_open_grid_axis_phase = 0;
    int iron_open_grid_boundary_col = 1600;
    int iron_open_grid_clearance = 2;

    static RuntimeConfig& instance() {
        static RuntimeConfig value;
        return value;
    }

    void override_from_env() {
#define OVERRIDE_BOOL(name, field) \
        do { if (std::getenv(name)) field = read_bool(name, field); } while (false)
#define OVERRIDE_INT(name, field) \
        do { if (std::getenv(name)) field = read_int(name, field); } while (false)
#define OVERRIDE_FLOAT(name, field) \
        do { if (std::getenv(name)) field = read_float(name, field); } while (false)

        OVERRIDE_BOOL("OPT_SKIP_TEST", skip_test);
        OVERRIDE_INT("OPT_DEBUG_LEVEL", debug_level);
        OVERRIDE_INT("OPT_NUM_THREADS", num_threads);
        OVERRIDE_BOOL("OPT_TIMING_TRACE", timing_trace);

        OVERRIDE_INT("GAME_ZONE_HARD_CAP", game_zone_hard_cap);
        OVERRIDE_INT("GAME_EXEMPT_MAIN", game_exempt_main);
        OVERRIDE_INT("GAME_DENSITY_CAP", game_density_cap);
        OVERRIDE_INT("GAME_DIR_THRESHOLD", game_dir_threshold);
        OVERRIDE_INT("GAME_ROUTE_PENALTY", game_route_penalty);
        OVERRIDE_INT("GAME_DISTANCE_SCORE_MODE", game_distance_score_mode);
        OVERRIDE_FLOAT("GAME_PICKUP_WEIGHT", game_pickup_weight);
        OVERRIDE_BOOL("GAME_USE_BFS_DISTANCE", game_use_bfs_distance);
        OVERRIDE_BOOL("GAME_PLANNER_MULTISTEP_AUTO", game_planner_multistep_auto);
        OVERRIDE_INT("GAME_PLANNER_MULTISTEP_COUNT", game_planner_multistep_auto_count);
        OVERRIDE_BOOL("GAME_REQUIRE_BFS_MATRIX", game_require_bfs_matrix);
        OVERRIDE_FLOAT("GAME_FLOW_SOFT_PENALTY_BASE", game_flow_soft_penalty_base);
        OVERRIDE_INT("GAME_GLOBAL_CAND_TOPK", game_global_cand_topk);
        OVERRIDE_INT("GAME_GLOBAL_MAX_TASK_SCAN", game_global_max_task_scan);
        OVERRIDE_FLOAT("GAME_FLOW_SOFT_OVER_CAP_GROWTH", game_flow_soft_over_cap_growth);
        OVERRIDE_BOOL("GAME_SHORT_TASK_THROTTLE", game_short_task_throttle);
        OVERRIDE_INT("GAME_SHORT_TASK_TEST_TICKS", game_short_task_test_ticks);
        OVERRIDE_INT("GAME_SHORT_TASK_FINAL_OPEN_TICKS", game_short_task_final_open_ticks);
        OVERRIDE_FLOAT("GAME_SHORT_TASK_RELEASE_RATIO", game_short_task_release_ratio);
        OVERRIDE_BOOL("GAME_IDLE_GLOBAL_MATCH", game_idle_global_match);
        OVERRIDE_INT("GAME_IDLE_GLOBAL_TICKS", game_idle_global_ticks);
        OVERRIDE_INT("GAME_IDLE_GLOBAL_MAX_AGENTS", game_idle_global_max_agents);
        OVERRIDE_BOOL("GAME_SCORE_EXEMPT_ENABLED", game_score_exempt_enabled);
        OVERRIDE_INT("GAME_EX_MAIN_CAP", game_ex_main_cap);
        OVERRIDE_INT("GAME_EX_NONMAIN_CAP", game_ex_nonmain_cap);
        OVERRIDE_FLOAT("GAME_BAL_ASSIGN_RATIO", game_bal_assign_ratio);
        OVERRIDE_BOOL("GAME_SKIP_CHANNEL_ERRANDS", game_skip_channel_errands);
        OVERRIDE_BOOL("GAME_DIR_FLOW_ENABLED", game_dir_flow_enabled);
        OVERRIDE_BOOL("GAME_DIR_FLOW_HARD_FILTER", game_dir_flow_hard_filter);
        OVERRIDE_INT("GAME_DIR_FLOW_CAP", game_dir_flow_cap);
        OVERRIDE_INT("GAME_DIR_FLOW_GAP", game_dir_flow_gap);
        OVERRIDE_FLOAT("GAME_DIR_FLOW_DECAY", game_dir_flow_decay);
        OVERRIDE_INT("GAME_DIR_FLOW_ZONE_HORIZON", game_dir_flow_zone_horizon);
        OVERRIDE_FLOAT("GAME_DIR_FLOW_HORIZON_EXEMPT_RATIO", game_dir_flow_horizon_exempt_ratio);
        OVERRIDE_BOOL("GAME_DIR_FLOW_DYNAMIC_CAP", game_dir_flow_dynamic_cap);
        OVERRIDE_INT("GAME_DIR_FLOW_DYNAMIC_CAP_WINDOW", game_dir_flow_dynamic_cap_window);
        OVERRIDE_INT("GAME_DIR_FLOW_DYNAMIC_CAP_INTERVAL", game_dir_flow_dynamic_cap_interval);
        OVERRIDE_FLOAT("GAME_DIR_FLOW_DYNAMIC_CAP_LOW", game_dir_flow_dynamic_cap_low);
        OVERRIDE_FLOAT("GAME_DIR_FLOW_DYNAMIC_CAP_HIGH", game_dir_flow_dynamic_cap_high);
        OVERRIDE_FLOAT("GAME_DIR_FLOW_DYNAMIC_CAP_INC", game_dir_flow_dynamic_cap_inc);
        OVERRIDE_FLOAT("GAME_DIR_FLOW_DYNAMIC_CAP_DEC", game_dir_flow_dynamic_cap_dec);
        OVERRIDE_FLOAT("GAME_DIR_FLOW_DYNAMIC_CAP_MIN_RATIO", game_dir_flow_dynamic_cap_min_ratio);
        OVERRIDE_FLOAT("GAME_DIR_FLOW_DYNAMIC_CAP_MAX_RATIO", game_dir_flow_dynamic_cap_max_ratio);
        OVERRIDE_BOOL("GAME_DIR_FLOW_RANDOM_SHORTEST", game_dir_flow_random_shortest);
        OVERRIDE_BOOL("GAME_HIGHWAY_ENABLED", game_highway_enabled);
        OVERRIDE_BOOL("GAME_HEURISTIC_HARD_DIRECTION", game_heuristic_hard_direction);
        OVERRIDE_BOOL("OPT_GAME_HEURISTIC_HARD_DIRECTION", game_heuristic_hard_direction);
        OVERRIDE_BOOL("GAME_GUIDE_PATH_ENABLED", game_guide_path_enabled);
        OVERRIDE_BOOL("GAME_IDLE_HIGHWAY_RELOCATE", game_idle_highway_relocate);
        OVERRIDE_BOOL("OPT_GAME_IDLE_HIGHWAY_RELOCATE", game_idle_highway_relocate);
        OVERRIDE_BOOL("GAME_SCHED_DIAG", game_sched_diag);
        OVERRIDE_INT("GAME_SCHED_DIAG_INTERVAL", game_sched_diag_interval);
        OVERRIDE_BOOL("GAME_REGION_MATCH", game_region_match);
        OVERRIDE_INT("GAME_REGION_FLOW_BATCH", game_region_flow_batch);
        OVERRIDE_BOOL("GAME_PREFILTER_CAPACITY", game_prefilter_capacity);
        OVERRIDE_INT("GAME_PLATOON_SCOPE", game_platoon_scope);
        OVERRIDE_BOOL("GAME_PIBT_COUNTER_FLOW", game_pibt_counter_flow);
        OVERRIDE_INT("GAME_COUNTER_FLOW_COST", game_counter_flow_cost);
        OVERRIDE_BOOL("GAME_URGENCY_PRIORITY", game_urgency_priority);
        OVERRIDE_FLOAT("GAME_URGENCY_WEIGHT", game_urgency_weight);
        OVERRIDE_INT("GAME_URGENCY_THRESHOLD", game_urgency_threshold);
        OVERRIDE_INT("GAME_SEARCH_RADIUS", game_search_radius);
        OVERRIDE_BOOL("OPT_DIST2PATH_PREWARM_ENABLED", dist2path_prewarm_enabled);
        OVERRIDE_INT("OPT_DIST2PATH_PREWARM_RADIUS", dist2path_prewarm_radius);
        OVERRIDE_INT("OPT_DIST2PATH_PREWARM_THREADS", dist2path_prewarm_threads);
        OVERRIDE_INT("OPT_DIST2PATH_PREWARM_MIN_LEFT_MS", dist2path_prewarm_min_left_ms);

#undef OVERRIDE_BOOL
#undef OVERRIDE_INT
#undef OVERRIDE_FLOAT
    }

private:
    static bool read_bool(const char* name, bool fallback) {
        const char* raw = std::getenv(name);
        if (!raw) return fallback;
        const std::string value(raw);
        if (value == "1" || value == "true" || value == "TRUE" ||
            value == "on" || value == "ON") return true;
        if (value == "0" || value == "false" || value == "FALSE" ||
            value == "off" || value == "OFF") return false;
        return fallback;
    }

    static int read_int(const char* name, int fallback) {
        const char* raw = std::getenv(name);
        if (!raw) return fallback;
        char* end = nullptr;
        const long value = std::strtol(raw, &end, 10);
        return end && *end == '\0' ? static_cast<int>(value) : fallback;
    }

    static float read_float(const char* name, float fallback) {
        const char* raw = std::getenv(name);
        if (!raw) return fallback;
        char* end = nullptr;
        const float value = std::strtof(raw, &end);
        return end && *end == '\0' ? value : fallback;
    }
};

inline RuntimeConfig& cfg() { return RuntimeConfig::instance(); }

}  // namespace optsvr
