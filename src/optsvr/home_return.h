#pragma once
// home_return.h — 共享数据：agent 原始起点
// planner.cpp 初始化（setup_multistep_episode_state 首调用），Entry.cpp 使用
#include <vector>

inline std::vector<int> g_home_start_locs;  // [agent_id] = 原始起点 location
