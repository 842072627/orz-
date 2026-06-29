#pragma once
// home_return.h — 共享数据：agent 原始起点 + inside mask（interior ∪ guide zone）
// planner.cpp 初始化，Entry.cpp 使用
#include <vector>

inline std::vector<int>   g_home_start_locs;   // [agent_id] = 原始起点 location
inline std::vector<char>  g_home_inside_mask;  // [loc] = 1 表示 interior 或 guide zone 内
