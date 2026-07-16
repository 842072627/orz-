#ifndef EXECUTOR_PARAMS_H
#define EXECUTOR_PARAMS_H
#include <string>
#include <cmath>

// 每个地图的 executor 方向锁参数
struct HeatLockParams {
    int predict_steps;       // 预判步数 (1..12)
    float decay_rate;        // 衰减系数 (1.0=无衰减, 0.9=每步90%)
    float threshold;         // plan预判总流量阈值
    bool bidirectional_filter;
    bool propagation;
    bool use_realtime;       // 使用实时occupancy锁
    float ema_threshold;     // 历史EMA阈值（0=使用threshold）
};

inline HeatLockParams get_executor_params(const std::string& map_name) {
    // map_name 格式：地图文件名，如 "orz900d.map", "scene_mp_2p_01.map"
    
    // orz：长路图，12步预判+衰减，提前识别远端拥堵
    if (map_name.find("orz") != std::string::npos) {
        return {12, 0.85f, 3.0f, true, true, false, 0.0f};
    }
    
    // iron harvest：超大图，12步+衰减，远距离分流
    if (map_name.find("scene_mp") != std::string::npos ||
        map_name.find("iron") != std::string::npos) {
        return {12, 0.90f, 5.0f, true, true, false, 0};
    }
    
    // warehouse：宽路网格，8步+轻度衰减
    if (map_name.find("warehouse") != std::string::npos ||
        map_name.find("fulfill") != std::string::npos) {
        return {8, 0.90f, 5.0f, true, true, false, 0};
    }
    
    // boston：城市图
    if (map_name.find("boston") != std::string::npos ||
        map_name.find("Boston") != std::string::npos ||
        map_name.find("bos") != std::string::npos) {
        return {8, 0.95f, 5.0f, true, true, false, 0};
    }
    
    // room：房间图，中等预判
    if (map_name.find("room") != std::string::npos) {
        return {8, 0.90f, 5.0f, true, true, false, 0};
    }
    
    // random：随机图，保守
    if (map_name.find("random") != std::string::npos) {
        return {5, 1.0f, 5.0f, true, true, false, 0};
    }
    
    // maze：1格走廊，方向锁无效，最小预判
    if (map_name.find("maze") != std::string::npos) {
        return {3, 1.0f, 10.0f, true, true, false, 0};
    }
    
    // 默认
    return {5, 1.0f, 5.0f, true, true, false, 0};
}

// 衰减权重计算（复用，避免重复 pow）
inline float decay_weight(int step, float decay_rate) {
    if (decay_rate >= 1.0f) return 1.0f;
    return std::pow(decay_rate, step);
}
#endif // EXECUTOR_PARAMS_H
