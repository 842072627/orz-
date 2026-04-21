# MAPF Deep Research Notes

> 深度研究笔记 - 沉淀真正的知识，不是跑测试记录
> Last updated: 2026-04-21 12:30 GMT+8

---

## 核心问题

**Q: 为什么我们的代码 (total≈1.65) 与冠军 (total≈4.0) 差距 2.4x？**

**A: 架构设计差异** - 不是参数调优能解决的

---

## 冠军架构解析

### 1. Flow-guided A* (不是 TimeSpaceA*!)

**我之前误解了TimeSpaceA*** - 冠军的astar不是3D时空搜索！

**真正的算法**：
```
f = g + h + op_flow + all_vertex_flow
```

其中：
- `g` = 路径长度（从起点的步数）
- `h` = 启发式（到目标的曼哈顿距离）
- `op_flow` = 反向流量（反向穿过这条边的agent数）
- `all_vertex_flow` = 经过这个顶点的总流量

**关键公式**（search.cpp第86行）：
```cpp
temp_op = ( (flow[curr_id].d[d]+1) * flow[next].d[(d+2)%4]);
// 如果我们向东(d=0)，检查下一个位置有多少agent想向西(d=2)
```

**搜索仍是2D空间**，但启发式被动态flow信息丰富。

### 2. GlobalManager = LNS协调器

**不是简单的"全局状态管理"**，而是Large Neighborhood Search的并行协调框架：

```cpp
class GlobalManager {
    std::vector<std::shared_ptr<NeighborGenerator>> neighbor_generators;
    std::vector<std::shared_ptr<LocalOptimizer>> local_optimizers;
    PathTable path_table;                    // 路径缓存表
    std::vector<updating_queues>;             // 并行更新队列
    float sum_of_costs;                      // 全局成本
    int num_of_failures;                     // 失败计数
};
```

**LNS流程**：
1. GlobalManager协调多个agent子集(neighborhood)的并行重规划
2. 每个LocalOptimizer处理一个neighborhood
3. 路径缓存在PathTable中共享
4. 全局成本sum_of_costs被追踪用于判断收敛

**我们的差距**：我们的planner是"单线程"的frank_wolfe循环，冠军是"多线程"LNS并行。

### 3. PathTable = 冲突检测表

```cpp
class PathTable {
    vector<vector<int>> table; // [location][timestep] = agent_id
    vector<int> goals;         // [location] = timestep到达

    getConflictingAgents(agent_id, loc, to_time);  // O(1)查询冲突
    constrained(from, to, to_time);                // 检查边是否被占用
};
```

**这如何用于LNS**：
1. 每个LocalOptimizer重规划一个neighborhood
2. PathTable记录当前所有agent的位置/路径
3. 重规划时查询PathTable避免产生新冲突
4. 更新后写回PathTable供其他neighborhood使用

---

## 架构差距总结

| 组件 | 冠军 | 我们 | 性能影响 | 实现难度 |
|------|------|------|----------|----------|
| LNS并行协调 | ✅ | ❌ (单线程FW) | **巨大** | 极高 |
| PathTable | ✅ | ❌ | **大** | 高 |
| single-step plan | ✅ (每步replan) | ❌ (multi-step window) | 中 | 中 |
| rand() tie-breaking | ✅ | ❌ (deterministic) | 中 | 低 |
| higher_id exception | ✅ (有) | ❌ (无，更好) | 已验证我们更好 | - |
| flow-guided A* | ✅ | ✅ (已有) | - | - |
| 启发式数据结构 | vector | unordered_map | 中 | 低 |

---

## 已验证事实

1. **frank_wolfe essential**: 去掉后fulfill -71%
2. **LNS不能叠加在frank_wolfe后面**: 目标冲突
3. **is_large_map检查有道理**: 移除后fulfill -17%
4. **DEV-336 (deviation>5 recompute)**: 是当前最优的guide path更新策略
5. **threshold=5是经验最优**: 太高(2)或太低(3)都更差
6. **window_size不能随便改**: 减少导致fulfill -83%
7. **PIBT higher_id exception**: 我们没有反而更好
8. **rand() tie-breaking**: 比deterministic轻微改善

---

## 灾难性失败记录

| 方向 | 结果 | 原因 |
|------|------|------|
| A2 (LNS替换FW) | fulfill=0.338 | frank_wolfe是必须的 |
| B1 (window_size-1) | fulfill -83% | TPG依赖被破坏 |
| A7 (移除is_large_map) | fulfill -17% | is_large_map有道理 |

---

## frank_wolfe天花板

**核心发现**：frank_wolfe主循环架构天花板 ≈ 1.6-1.7

**原因**：
- frank_wolfe在plan()开始时分配flow并优化路径
- executor的window机制让guide path在多步内保持stale
- DEV-336通过deviation>5来缓解stale问题，但不是根本解决
- 冠军的架构是每步plan + 每步replan，不是multi-step window

**所有"叠加LNS"尝试都失败**：
- A1-A7: 全部失败
- A8: LNS destroy-repair循环 - total=1.620
- A9: PIBT higher_id - total=1.581
- A10: guide path 5→2 - total=1.590

---

## 下一步研究方向

**不是放弃** - 而是承认frank_wolfe天花板的现实，聚焦可改进的方向：

1. **PIBT决策质量** - 启发式准确性、tie-breaking策略
2. **启发式更新频率** - Dev-336是补丁，不是根本解决
3. **任务分配质量** - scheduler的贪心策略

**真正需要的是**：单步plan + 每步replan架构 - 但这需要完全重写executor

---

## 研究产出 (2026-04-21)

- commit `17c8a37` - Flow-guided A*真相
- commit `7040d47` - GlobalManager = LNS coordinator
- commit `519f390` - PathTable = conflict detection
- commit `e0927dd` - single-step vs multi-step architecture
- commit `5ece99f` - causalPIBT comparison + heuristics data structure

_Last updated: 2026-04-21 12:30 GMT+8_
