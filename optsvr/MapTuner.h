// optsvr/MapTuner.h — 地图类型识别 + 自动设置 schedule_mode（fullfill only）
#pragma once

#include "RuntimeConfig.h"
#include "MapProfile.h"
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>

namespace optsvr {

inline void load_map_tuning(const SharedEnvironment* env) {
    auto& c = cfg();
    auto profile = analyze_map(env);

    switch (profile.type) {

        case MapType::WAREHOUSE_DOMAIN:
            c.skip_test = false;
            c.schedule_mode = 1;   // GreedyHeap
            c.executor_mode = 3;   // DHExecutor
            c.planner_mode = 11;   // TestUsefulFlowDecay
            c.ware_direct = true;
            break;

        default:
            c.skip_test = true;
            std::cerr << "[MapTuner] SKIP_TEST (not warehouse): " << profile.type_to_string() << "\n";
            break;
    }

    c.override_from_env();

    if (c.debug_level >= 1) {
        std::cerr << "[MapTuner] " << profile.type_to_string()
                  << " -> schedule_mode=" << c.schedule_mode
                  << " planner_mode=" << c.planner_mode
                  << " executor_mode=" << c.executor_mode << "\n";
    }
}

} // namespace optsvr
