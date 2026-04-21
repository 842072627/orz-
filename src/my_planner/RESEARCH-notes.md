# MAPF Deep Research Notes

> 深度研究笔记 - 沉淀真正的知识，不是跑测试记录
> Last updated: 2026-04-21 14:00 GMT+8

---

## 核心问题

**Q: 为什么我们的代码 (total≈1.65) 与冠军 (total≈4.0) 差距 2.4x？**

**A: 架构设计差异** - 不是参数调优能解决的

---

## 冠军架构解析

### 真正的架构：LNS主循环

**关键发现**：冠军的Dist2Path代码被注释掉了！

```cpp
// Dist2PathHeuristicTable.h 第34-39行
// if (!lns.traj_dists.empty() && !lns.traj_dists[agent_idx].empty())
//     min_heuristic = get_dist_2_path(lns.traj_dists[agent_idx], lns.env, loc, &(lns.neighbors));
// else
```

**冠军只使用静态启发式表**，没有使用Dist2Path！

### 架构差距总结

| 组件 | 冠军 | 我们 | 关键差异 |
|------|------|------|----------|
| 主循环 | LNS | frank_wolfe | **根本差异** |
| 启发式 | 静态表 | Dist2Path | 我们的更优（但没用） |
| 重规划 | 每步完全重规划 | guide path stale | **根本差异** |
| 冲突处理 | LNS neighborhood repair | PIBT因果链 | 不同机制 |

### 核心结论

**启发式不是瓶颈**！冠军甚至没用Dist2Path，但他们的LNS主循环架构让他们的性能是我们的2.4倍。

---

## 灾难性失败记录

| 方向 | 结果 | 原因 |
|------|------|------|
| A2 (LNS替换FW) | fulfill=0.338 | frank_wolfe是必须的 |
| B1 (window_size-1) | fulfill -83% | TPG依赖被破坏 |
| A7 (移除is_large_map) | fulfill -17% | is_large_map有道理 |
| A11 | fulfill -89% | **在PIBT决策后修改guide path** |
| A12 | fulfill -100% | **O(n)遍历导致超时/死锁** |

---

## 已验证事实

1. **frank_wolfe essential**: 去掉后fulfill -71%
2. **LNS不能叠加在frank_wolfe后面**: 目标冲突
3. **is_large_map检查有道理**: 移除后fulfill -17%
4. **DEV-336**: 当前最优的guide path更新策略
5. **threshold=5是经验最优**: 太高或太低都更差
6. **window_size不能随便改**: 减少导致fulfill -83%
7. **启发式不是瓶颈**: 冠军甚至没用Dist2Path
8. **不要在PIBT决策后修改guide path**: 灾难
9. **不要在PIBT排序中遍历整个map**: 灾难
10. **frank_wolfe天花板 ≈ 1.6-1.7**: 架构限制

---

## 核心结论

**frank_wolfe + PIBT架构的天花板 ≈ 1.6-1.7**

唯一能突破的方向：**LNS主循环架构重构**

但这需要：
1. 数周工作时间
2. 集成LaCAM2初始化
3. 实现完整的LNS框架（GlobalManager, NeighborGenerator, LocalOptimizer）
4. 实现TimeSpaceA* (SIPP变体)

风险：极高（可能在重写过程中引入新问题）
回报：理论上100%+（突破1.7天花板）

---

## 研究产出 (2026-04-21)

- commit `17c8a37` - Flow-guided A*真相
- commit `7040d47` - GlobalManager = LNS coordinator
- commit `519f390` - PathTable = conflict detection
- commit `e0927dd` - single-step vs multi-step architecture
- commit `5ece99f` - causalPIBT comparison
- commit `ed59b73` - A11失败记录
- commit `8794697` - A12失败记录
- commit `48554eb` - **Dist2Path被注释！启发式不是瓶颈**

---

## LNS架构详细分析 (2026-04-21 14:20)

### PLNSSolver核心参数 (plns.h)

```cpp
int window_size_for_PATH=15;     // 路径规划window
int neighbor_size=8;             // neighborhood大小
destroy_strategy=RANDOMWALK;      // destroy策略
init_algo_name="LaCAM2";        // LaCAM2仅用于初始化
replan_algo_name="PP";          // replan用Prioritized Planning
int execution_window=1;          // **关键！每步完全replan**
```

### GlobalManager核心组件

| 组件 | 职责 |
|------|------|
| NeighborGenerator | 生成neighborhood（destroy哪些agent） |
| LocalOptimizer | 重新规划被destroy的agent |
| PathTable | O(1)冲突检测 |
| Agent管理 | 维护每个agent的路径和状态 |

### LNS主循环逻辑 (GlobalManager.run())

```
1. getInitialSolution(): 用LaCAM2生成初始解
2. LOOP (直到时间用完):
   a. 生成neighbors (NeighborGenerator):
      - destroy策略: RANDOMWALK
      - 随机选择neighbor_size个agent
      - 标记其路径为destroyed
   b. 重新规划 (LocalOptimizer):
      - 对每个destroyed agent用PIBT重新规划
      - 检查冲突 (PathTable)
   c. 接受/拒绝:
      - 如果新方案更优，接受
      - 否则拒绝，保留原方案
3. 返回最终actions
```

### champion的plan()流程 (planner.cpp)

```cpp
// 1. 用causal_pibt生成init_paths (multi_step)
auto multi_step_actions = causal_pibt->multi_step(window_size, ...);

// 2. 转换为init_paths格式
for each agent i:
    init_path[i] = 从multi_step_actions[i]生成

// 3. 用LNS改进init_paths
auto actions = plns->solve(goal_locations, init_paths, time_limit);

// 4. 返回actions
```

### 关键差异：execution_window=1

**我们的架构**:
```
plan() called once → frank_wolfe生成guide paths → executor消费window_size步
```

**冠军架构**:
```
plan() called every frame → LNS重新规划 → execution_window=1 → 每步fresh
```

### 重构需要的关键组件

1. **PLNSSolver**: 主调度器，管理LNS迭代
2. **GlobalManager**: 协调neighbors、replan、accept/reject
3. **NeighborGenerator**: 实现RANDOMWALK等destroy策略
4. **LocalOptimizer**: 实现PIBT/PP重新规划
5. **PathTable**: O(1)冲突检测表
6. **TimeSpaceAStarPlanner**: SIPP变体，用于复杂场景

--## A13 LNS重构进度 (2026-04-21 14:25)

### Day 1 里程碑 ✅
- [x] 创建LNS目录结构 (`inc/LNS/`, `src/LNS/`)
- [x] PathTable.h骨架完成（3493字节）
- [x] 项目编译成功 (lifelong.exe 3.7MB)
- [x] Baseline验证: total=1.652

### PathTable.h 实现状态
```cpp
class PathTable {
    std::vector<std::vector<int>> table; // [location][time] = agent_id
    std::vector<int> goals; // goal location -> timestep
    
    void insertPath(int agent_id, const vector<pair<int,int>>& path);
    bool constrained(int location, int timestep) const;
    bool edgeConstrained(int from, int to, int timestep) const;
    void getConflictingAgents(int location, int timestep, set<int>& agents) const;
    int getHoldingTime(int location, int earliest_timestep) const;
};
```

### Day 2 待办
- [ ] 创建NeighborGenerator骨架
- [ ] 创建LocalOptimizer骨架
- [ ] 集成到planner.cpp测试

---

_Last updated: 2026-04-21 14:25 GMT+8 - Day 1里程碑完成_
