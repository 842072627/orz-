# MAPF Deep Research Notes

> 深度研究笔记 - 沉淀真正的知识，不是跑测试记录
> Last updated: 2026-04-21 07:42 GMT+8

---

## 核心问题

**Q: 为什么我们的代码 (total≈1.61) 与冠军 (total≈4.0) 差距 2.5x？**

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

### 4. single-step vs multi-step 架构真相

**关键发现**：

```cpp
// Champion的plan() - 单步
void plan(int time_limit, vector<Action> & actions, SharedEnvironment* env) {
    // 单步actions，下个timestep立即再次调用plan()
    update_traj(...) + frank_wolfe(...) + PIBT(...)
}

// 我们的plan() - 多步 via executor window
void plan(int time_limit, vector<vector<Action>> & actions,
         SharedEnvironment* env, int num_steps) {
    // 多步actions，通过executor的window机制分解执行
    // window_size = min_planner_communication_time / ...
}
```

### 5. Dual-Timescale Architecture

这是比赛本身的设计，不是冠军的创新：

- **Planner**: 返回multi-step intent (如10步)
- **Executor**: 通过TPG(Time-Progress Graph)控制GO/STOP时序
- **Delay是核心机制**：每个tick有概率p触发delay
- Executor不能改变路径(where)，只能控制时序(when)
- 碰撞检测基于agent footprint (safety bubbles)，不是点碰撞

---

## 实验结论

### 架构天花板确认

经过27次实验，我们确认了**架构性天花板**：

| 瓶颈 | 影响 | 难度 |
|------|------|------|
| LNS并行协调 | 巨大 | 极高(需引入框架) |
| PathTable | 大 | 高(需数据结构重构) |
| single-step接口 | 中 | 中(需改plan接口) |
| rand() tie-breaking | 中 | 低(牺牲reproducibility) |

### 真正需要做的

1. **LNS/PLNS框架** - 这是冠军性能的核心
2. **PathTable实现** - O(1)冲突检测
3. **Single-step plan接口** - 与executor解耦

### 已验证失败的优化方向

所有"叠加LNS到现有frank_wolfe"的尝试都失败了：
- A1-A7: frank_wolfe + LNS叠加 - 全部失败
- A8: LNS destroy-repair - total=1.620
- A9: PIBT higher_id exception - total=1.581
- A10: guide path threshold调整 - 无改善
- A15: 基于拥塞率跳过frank_wolfe - fulfill暴跌至0.332
- A30: frank_wolfe迭代限制 - total=2.566 (-0.8%)
- A32: 运行时自适应早期终止 - total=2.576 (-1.2%)
- A41: frank_wolfe自适应deadline - total=2.556 (无改善)

---

## 附录：2026架构理解(修正)

### 误解纠正

**之前误解**：TimeSpaceA*是3D时空搜索

**正确理解**：冠军的astar是flow-guided A*，搜索空间仍是2D，但启发式被动态flow信息增强。

### 关键代码位置

| 功能 | 文件位置 |
|------|----------|
| Flow-guided启发式 | search.cpp:86 |
| GlobalManager | GlobalManager.cpp |
| PathTable | PathTable.cpp |
| PIBT | PIBT.cpp |

---

*Last update: 2026-04-21 23:38 GMT+8*
