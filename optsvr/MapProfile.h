// optsvr/MapProfile.h — 地图特征识别与分类
// 在 initialize 阶段调用一次，分析地图结构特征
// 后续 scheduler / planner 根据 MapProfile 选择定制策略
#pragma once

#include "SharedEnv.h"
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace optsvr {

// ================================================================
// MapType — 地图类型枚举
// ================================================================
enum class MapType {
    WAREHOUSE_DOMAIN,       // warehouse.domain：结构化走廊型大图
    CITY_DOMAIN,            // city.domain：中等障碍率，高路口比例的大图
    GAME_DOMAIN,            // game.domain：极高障碍率的大规模复杂地形
    IRON_HARVEST_DOMAIN,    // iron_harvest.domain：超大、结构化但几乎全是路口的复杂地形
    MAZE_DOMAIN,            // maze.domain：高障碍率的小型迷宫图
    RANDOM_DOMAIN,          // random.domain：小型开阔随机图
    ROOM_DOMAIN,            // room.domain：小型结构化房间图
    UNKNOWN                 // 未匹配
};

// ================================================================
// MapSizeClass — 地图尺寸等级
// ================================================================
enum class MapSizeClass {
    TINY,       // < 2,000 cells
    SMALL,      // 2,000 ~ 10,000
    MEDIUM,     // 10,000 ~ 50,000
    LARGE,      // 50,000 ~ 200,000
    XLARGE      // > 200,000
};

// ================================================================
// CongestionLevel — agent 密度/拥塞等级
// ================================================================
enum class CongestionLevel {
    LOW,        // agent_density < 0.02
    MEDIUM,     // 0.02 ~ 0.10
    HIGH,       // 0.10 ~ 0.25
    EXTREME     // > 0.25
};

// ================================================================
// MapProfile — 完整的地图特征档案
// ================================================================
struct MapProfile {
    // ---- 基本分类 ----
    MapType         type         = MapType::UNKNOWN;
    MapSizeClass    size_class   = MapSizeClass::MEDIUM;
    CongestionLevel congestion   = CongestionLevel::MEDIUM;

    // ---- 原始尺寸参数 ----
    int  rows          = 0;
    int  cols          = 0;
    int  map_size      = 0;      // rows * cols
    int  n_agents      = 0;

    // ---- 障碍物统计 ----
    int  n_obstacles   = 0;      // 障碍物格子数
    int  n_free        = 0;      // 可通行格子数
    float obstacle_density = 0;  // n_obstacles / map_size

    // ---- agent 密度 ----
    float agent_density = 0;     // n_agents / n_free

    // ---- 连通性指标 ----
    float avg_degree   = 0;      // 每个可通行格的平均可通行邻居数 (0~4)
    int   bottleneck_count = 0;  // 度数 ≤ 1 的格子（死端/瓶颈）
    int   corridor_count   = 0;  // 度数 == 2 的格子（走廊/通道）
    int   junction_count   = 0;  // 度数 >= 3 的格子（交叉路口）

    // ---- 结构化特征 ----
    bool  is_structured    = false;  // 是否有规则通道（仓库型横竖通道）
    float row_regularity   = 0;     // 行方向通道规律性 (0~1)
    float col_regularity   = 0;     // 列方向通道规律性 (0~1)

    // ---- 衍生布尔标记 ----
    bool  is_large         = false;  // map_size > 50000
    bool  is_dense_agents  = false;  // agent_density > 0.10
    bool  is_open          = false;  // obstacle_density < 0.10
    bool  has_bottleneck   = false;  // bottleneck_count 占比较高

    // ---- 地图名称（调试用） ----
    std::string map_name;

    // ---- 打印函数 ----
    void print() const {
        std::cerr << "[MapProfile] ================================\n"
                  << "  map_name        = " << map_name << "\n"
                  << "  type            = " << type_to_string() << "\n"
                  << "  size_class      = " << size_class_to_string() << "\n"
                  << "  congestion      = " << congestion_to_string() << "\n"
                  << "  rows x cols     = " << rows << " x " << cols
                  << " (" << map_size << " cells)\n"
                  << "  n_agents        = " << n_agents << "\n"
                  << "  n_obstacles     = " << n_obstacles
                  << " (density=" << obstacle_density << ")\n"
                  << "  n_free          = " << n_free
                  << " (agent_density=" << agent_density << ")\n"
                  << "  avg_degree      = " << avg_degree << "\n"
                  << "  bottleneck/corridor/junction = "
                  << bottleneck_count << " / " << corridor_count
                  << " / " << junction_count << "\n"
                  << "  is_structured   = " << is_structured
                  << " (row_reg=" << row_regularity
                  << ", col_reg=" << col_regularity << ")\n"
                  << "  is_large=" << is_large
                  << " is_dense=" << is_dense_agents
                  << " is_open=" << is_open
                  << " has_bottleneck=" << has_bottleneck << "\n"
                  << "[MapProfile] ================================\n";
    }

    std::string type_to_string() const {
        switch (type) {
            case MapType::WAREHOUSE_DOMAIN:    return "WAREHOUSE_DOMAIN";
            case MapType::CITY_DOMAIN:         return "CITY_DOMAIN";
            case MapType::GAME_DOMAIN:         return "GAME_DOMAIN";
            case MapType::IRON_HARVEST_DOMAIN: return "IRON_HARVEST_DOMAIN";
            case MapType::MAZE_DOMAIN:         return "MAZE_DOMAIN";
            case MapType::RANDOM_DOMAIN:       return "RANDOM_DOMAIN";
            case MapType::ROOM_DOMAIN:         return "ROOM_DOMAIN";
            case MapType::UNKNOWN:             return "UNKNOWN";
        }
        return "UNKNOWN";
    }

    std::string size_class_to_string() const {
        switch (size_class) {
            case MapSizeClass::TINY:   return "TINY";
            case MapSizeClass::SMALL:  return "SMALL";
            case MapSizeClass::MEDIUM: return "MEDIUM";
            case MapSizeClass::LARGE:  return "LARGE";
            case MapSizeClass::XLARGE: return "XLARGE";
        }
        return "MEDIUM";
    }

    std::string congestion_to_string() const {
        switch (congestion) {
            case CongestionLevel::LOW:     return "LOW";
            case CongestionLevel::MEDIUM:  return "MEDIUM";
            case CongestionLevel::HIGH:    return "HIGH";
            case CongestionLevel::EXTREME: return "EXTREME";
        }
        return "MEDIUM";
    }
};

// 前向声明
inline MapType identify_map_type(const MapProfile& p);

// ================================================================
// analyze_map — 分析地图特征，返回 MapProfile
// 在 initialize 阶段调用一次
// ================================================================
inline MapProfile analyze_map(const SharedEnvironment* env) {
    MapProfile p;
    p.map_name  = env->map_name;
    p.rows      = env->rows;
    p.cols      = env->cols;
    p.map_size  = env->rows * env->cols;
    p.n_agents  = env->num_of_agents;

    // ================================================================
    // Step 1: 障碍物统计
    // ================================================================
    p.n_obstacles = 0;
    for (int i = 0; i < p.map_size; i++) {
        if (env->map[i] == 1) p.n_obstacles++;
    }
    p.n_free = p.map_size - p.n_obstacles;
    p.obstacle_density = (p.map_size > 0)
        ? static_cast<float>(p.n_obstacles) / p.map_size : 0;
    p.agent_density = (p.n_free > 0)
        ? static_cast<float>(p.n_agents) / p.n_free : 0;

    // ================================================================
    // Step 2: 连通性分析 — 每个可通行格的可通行邻居数
    // ================================================================
    long long total_degree = 0;
    p.bottleneck_count = 0;
    p.corridor_count   = 0;
    p.junction_count   = 0;

    // 四方向偏移: East(+1), South(+cols), West(-1), North(-cols)
    const int dx[4] = {0, 1, 0, -1};
    const int dy[4] = {1, 0, -1, 0};

    for (int r = 0; r < p.rows; r++) {
        for (int c = 0; c < p.cols; c++) {
            int id = r * p.cols + c;
            if (env->map[id] == 1) continue; // 障碍物跳过

            int deg = 0;
            for (int d = 0; d < 4; d++) {
                int nr = r + dx[d];
                int nc = c + dy[d];
                if (nr >= 0 && nr < p.rows && nc >= 0 && nc < p.cols) {
                    int nid = nr * p.cols + nc;
                    if (env->map[nid] == 0) deg++;
                }
            }

            total_degree += deg;
            if (deg <= 1) p.bottleneck_count++;
            else if (deg == 2) p.corridor_count++;
            else p.junction_count++;  // deg >= 3
        }
    }
    p.avg_degree = (p.n_free > 0)
        ? static_cast<float>(total_degree) / p.n_free : 0;

    // ================================================================
    // Step 3: 结构化通道检测（仓库型横竖通道规律性）
    // ================================================================
    // 思路：统计每行/每列的可通行比例。
    // 如果存在交替的"全通行行"和"全障碍行"的规律，说明是仓库结构。
    //
    // row_regularity: 对所有行计算可通行率，然后看方差。
    //   仓库型地图：行可通行率呈双峰分布（有的行几乎全通行，有的行几乎全障碍）
    //   → 方差大 → regularity 高
    //   随机地图：各行可通行率都差不多 → 方差小 → regularity 低

    if (p.map_size > 0 && p.rows > 1 && p.cols > 1) {
        // 行通行率
        std::vector<float> row_free_ratio(p.rows);
        for (int r = 0; r < p.rows; r++) {
            int free_count = 0;
            for (int c = 0; c < p.cols; c++) {
                if (env->map[r * p.cols + c] == 0) free_count++;
            }
            row_free_ratio[r] = static_cast<float>(free_count) / p.cols;
        }

        // 列通行率
        std::vector<float> col_free_ratio(p.cols);
        for (int c = 0; c < p.cols; c++) {
            int free_count = 0;
            for (int r = 0; r < p.rows; r++) {
                if (env->map[r * p.cols + c] == 0) free_count++;
            }
            col_free_ratio[c] = static_cast<float>(free_count) / p.rows;
        }

        // 计算方差 → regularity
        auto compute_variance = [](const std::vector<float>& v) -> float {
            if (v.empty()) return 0;
            float mean = std::accumulate(v.begin(), v.end(), 0.0f) / v.size();
            float var = 0;
            for (float x : v) var += (x - mean) * (x - mean);
            return var / v.size();
        };

        float row_var = compute_variance(row_free_ratio);
        float col_var = compute_variance(col_free_ratio);

        // 归一化到 [0,1]：方差最大为 0.25（二值分布 0/1 各半）
        p.row_regularity = std::min(1.0f, row_var / 0.25f);
        p.col_regularity = std::min(1.0f, col_var / 0.25f);

        // 仓库型：至少一个方向的 regularity > 0.1
        p.is_structured = (p.row_regularity > 0.10f || p.col_regularity > 0.10f);
    }

    // ================================================================
    // Step 4: 尺寸等级分类
    // ================================================================
    if      (p.map_size < 2000)    p.size_class = MapSizeClass::TINY;
    else if (p.map_size < 10000)   p.size_class = MapSizeClass::SMALL;
    else if (p.map_size < 50000)   p.size_class = MapSizeClass::MEDIUM;
    else if (p.map_size < 200000)  p.size_class = MapSizeClass::LARGE;
    else                           p.size_class = MapSizeClass::XLARGE;

    // ================================================================
    // Step 5: 拥塞等级
    // ================================================================
    if      (p.agent_density < 0.02f)  p.congestion = CongestionLevel::LOW;
    else if (p.agent_density < 0.10f)  p.congestion = CongestionLevel::MEDIUM;
    else if (p.agent_density < 0.25f)  p.congestion = CongestionLevel::HIGH;
    else                               p.congestion = CongestionLevel::EXTREME;

    // ================================================================
    // Step 6: 衍生布尔标记
    // ================================================================
    p.is_large = (p.map_size > 50000);
    p.is_dense_agents = (p.agent_density > 0.10f);
    p.is_open = (p.obstacle_density < 0.10f);
    // 如果 bottleneck + corridor 占比 > 60% 的可通行格，视为高瓶颈
    p.has_bottleneck = (p.n_free > 0) &&
        (static_cast<float>(p.bottleneck_count + p.corridor_count) / p.n_free > 0.60f);

    // ================================================================
    // Step 7: 地图类型识别
    // ================================================================
    // 只根据结构特征分类，不依赖 map_name / example 文件名。
    p.type = identify_map_type(p);

    return p;
}

// ================================================================
// identify_map_type — 仅根据地图结构特征判断地图类型
//
// 当前阈值基于 MAIN2026/example_problems 的实测特征：
//   - warehouse.domain      : structured + obstacle≈0.45 + corridor≈0.71
//   - city.domain           : unstructured + obstacle≈0.27 + junction≈0.92
//   - game.domain           : obstacle≈0.90 + junction≈0.96
//   - iron_harvest.domain   : structured but corridor≈0.003、junction≈0.997
//   - maze.domain           : 小图，高障碍率，avg_degree 更低
//   - random.domain         : 小图，较开阔，非结构化
//   - room.domain           : 小图，轻度结构化且高路口比例
// ================================================================
inline MapType identify_map_type(const MapProfile& p) {
    const float corridor_ratio = (p.n_free > 0)
        ? static_cast<float>(p.corridor_count) / p.n_free : 0.0f;
    const float junction_ratio = (p.n_free > 0)
        ? static_cast<float>(p.junction_count) / p.n_free : 0.0f;
    const float bottleneck_ratio = (p.n_free > 0)
        ? static_cast<float>(p.bottleneck_count) / p.n_free : 0.0f;
    const float structure_strength = std::max(p.row_regularity, p.col_regularity);

    // 1) 小图域：maze / room / random。
    if (p.map_size < 5000) {
        if (p.obstacle_density >= 0.25f && corridor_ratio >= 0.10f && p.avg_degree <= 3.10f) {
            return MapType::MAZE_DOMAIN;
        }
        if (p.is_structured && junction_ratio >= 0.90f && corridor_ratio <= 0.05f &&
            p.obstacle_density <= 0.20f) {
            return MapType::ROOM_DOMAIN;
        }
        return MapType::RANDOM_DOMAIN;
    }

    // 2) warehouse.domain：结构化且走廊占比显著偏高。
    if (p.is_structured && structure_strength >= 0.20f &&
        p.obstacle_density >= 0.20f && corridor_ratio >= 0.35f) {
        return MapType::WAREHOUSE_DOMAIN;
    }

    // 3) iron_harvest.domain：超大、结构化，但几乎全是路口而不是走廊。
    if (p.map_size >= 2000000 && p.is_structured &&
        p.obstacle_density >= 0.25f && p.obstacle_density <= 0.50f &&
        junction_ratio >= 0.95f && corridor_ratio <= 0.05f) {
        return MapType::IRON_HARVEST_DOMAIN;
    }

    // 4) city.domain：中高障碍率、路口占比高、走廊不突出的大图。
    if (p.map_size >= 20000 &&
        p.obstacle_density >= 0.15f && p.obstacle_density <= 0.40f &&
        junction_ratio >= 0.80f && corridor_ratio <= 0.20f &&
        bottleneck_ratio <= 0.03f) {
        return MapType::CITY_DOMAIN;
    }

    // 5) 剩余的大型复杂图归到 game.domain。
    if (p.obstacle_density >= 0.60f ||
        (p.map_size >= 200000 && junction_ratio >= 0.90f && corridor_ratio <= 0.08f) ||
        p.map_size >= 20000) {
        return MapType::GAME_DOMAIN;
    }

    return MapType::RANDOM_DOMAIN;
}

// ================================================================
// 全局地图特征档案（单例，initialize 后可随处读取）
// ================================================================
inline MapProfile& map_profile() {
    static MapProfile instance;
    return instance;
}

} // namespace optsvr
