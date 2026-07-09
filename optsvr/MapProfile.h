// optsvr/MapProfile.h — 地图特征识别（ORZ-only）
#pragma once

#include "SharedEnv.h"
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace optsvr {

enum class MapType {
    GAME_DOMAIN,
    UNKNOWN
};

enum class MapSizeClass {
    TINY, SMALL, MEDIUM, LARGE, XLARGE
};

enum class CongestionLevel {
    LOW, MEDIUM, HIGH, EXTREME
};

struct MapProfile {
    std::string     map_name;
    int             rows         = 0;
    int             cols         = 0;
    int             num_cells    = 0;
    int             free_cells   = 0;
    float           obstacle_ratio = 1.0f;
    MapType         type         = MapType::UNKNOWN;
    MapSizeClass    size_class   = MapSizeClass::MEDIUM;
    CongestionLevel congestion   = CongestionLevel::LOW;
    float           agent_density = 0.0f;
    int             num_agents   = 0;
    int             num_tasks    = 0;
};

inline static MapProfile g_map_profile;
inline MapProfile& map_profile() { return g_map_profile; }

inline std::string map_type_name(MapType type) {
    switch (type) {
        case MapType::GAME_DOMAIN: return "GAME_DOMAIN";
        default: return "UNKNOWN";
    }
}

inline MapType detect_map_type(const SharedEnvironment* env) {
    const int rows = env->rows;
    const int cols = env->cols;
    const int cells = rows * cols;
    int free_cells = 0;
    for (int i = 0; i < cells; i++) {
        if (env->map[i] == 0) free_cells++;
    }
    float obs_ratio = 1.0f - (float)free_cells / (float)cells;
    return (obs_ratio > 0.50f && cells > 500000) ? MapType::GAME_DOMAIN : MapType::UNKNOWN;
}

inline void analyze_map(SharedEnvironment* env, int num_agents, int num_tasks) {
    g_map_profile.map_name = env->map_name;
    g_map_profile.rows = env->rows;
    g_map_profile.cols = env->cols;
    g_map_profile.num_cells = env->rows * env->cols;
    g_map_profile.free_cells = 0;
    for (int i = 0; i < g_map_profile.num_cells; i++) {
        if (env->map[i] == 0) g_map_profile.free_cells++;
    }
    g_map_profile.obstacle_ratio = 1.0f - (float)g_map_profile.free_cells / (float)g_map_profile.num_cells;
    g_map_profile.type = detect_map_type(env);
    g_map_profile.num_agents = num_agents;
    g_map_profile.num_tasks = num_tasks;
    g_map_profile.agent_density = num_agents > 0 ? (float)num_agents / (float)g_map_profile.free_cells : 0;
    if (g_map_profile.free_cells < 2000) g_map_profile.size_class = MapSizeClass::TINY;
    else if (g_map_profile.free_cells < 10000) g_map_profile.size_class = MapSizeClass::SMALL;
    else if (g_map_profile.free_cells < 50000) g_map_profile.size_class = MapSizeClass::MEDIUM;
    else if (g_map_profile.free_cells < 200000) g_map_profile.size_class = MapSizeClass::LARGE;
    else g_map_profile.size_class = MapSizeClass::XLARGE;
    if (g_map_profile.agent_density < 0.02f) g_map_profile.congestion = CongestionLevel::LOW;
    else if (g_map_profile.agent_density < 0.10f) g_map_profile.congestion = CongestionLevel::MEDIUM;
    else if (g_map_profile.agent_density < 0.25f) g_map_profile.congestion = CongestionLevel::HIGH;
    else g_map_profile.congestion = CongestionLevel::EXTREME;
}

inline void load_map_tuning(SharedEnvironment* env) {
    analyze_map(env, env->num_of_agents, 0);
    if (g_map_profile.type == MapType::UNKNOWN) {
        std::fprintf(stderr, "[MapProfile] unknown map type for %s\n", env->map_name.c_str());
    }
}

}  // namespace optsvr
