# Team_RAPID LNS 算法研究

## 创建时间
2026-04-22 16:50

## 状态
进行中（重大发现！）

---

## 🎯 核心发现：邻居选择机制是关键！

### 我们的 frank_wolfe (my_flow.cpp:125-145)
```cpp
while (std::chrono::steady_clock::now() < timelimit){
    index = count % lns.env->num_of_agents;  // 遍历 ALL agents!
    a = replan_order[index].id;
    count++;
    // 处理每个 agent
    TimePoint agent_deadline = now + milliseconds(deadline_per_agent);
    if (!update_traj(lns, a, &agent_deadline)) { ... }
}
```

**问题：把时间分给 ALL agents，每个 agent 只有 `deadline_per_agent = remaining_ms / num_of_agents`**

### Team_RAPID 的 LNS
```cpp
neighbor_size = 8;  // 只选择8个邻居！
// 把时间集中在8个邻居上，而不是分散给所有人
```

### 性能影响

| 地图 | Agents | 我们的 deadline_per_agent | Team_RAPID neighbor_size |
|------|--------|---------------------------|---------------------------|
| room | 150 | ~6.6ms | 8 邻居 |
| maze | 40 | ~25ms | 8 邻居 |
| random | 400 | ~2.5ms | 8 邻居 |
| fulfill | 2500 | ~0.4ms | 8 邻居 |

---

## Team_RAPID 的 LNS 配置 (random-32-32-20.json)

```json
"LNS": {
    "sipp": true,              // ✅ SIPP 启用！
    "neighborSize": 8,         // 邻居大小
    "initAlgo": "LaCAM2",      // 初始化用 LaCAM2
    "replanAlgo": "PP",       // 重规划用 PP
    "destoryStrategy": "Adaptive",
    "initLNS": false,
    "pibtWindow": 5,
    "winPibtSoftmode": true,
    ...
}
```

---

## 邻居生成机制

### Team_RAPID 的邻居生成有两种策略：

#### 1. RandomWalk（随机游走）
```cpp
// NeighborGenerator.cpp:197
bool NeighborGenerator::generateNeighborByRandomWalk(Neighbor & neighbor, int idx) {
    if (neighbor_size >= (int)agents.size()) {
        // 如果 neighbor_size >= agents数量，选择所有agent
    }
    // 使用随机游走选择冲突的邻居
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
}
```

#### 2. Intersection（交叉口）
```cpp
// 在交叉口位置选择邻居
```

---

## ALNS（自适应大邻域搜索）销毁策略

Team_RAPID 使用 ALNS 策略选择销毁哪些代理：

```cpp
// NeighborGenerator.cpp:163
void NeighborGenerator::chooseDestroyHeuristicbyALNS() {
    int selected_neighbor = rouletteWheel();  // 轮盘赌选择
    switch (selected_neighbor) {
        case 0: destroy_strategy = RANDOMWALK; break;
        case 1: destroy_strategy = INTERSECTION; break;
    }
}
```

---

## 关键差距总结

| 功能 | Team_RAPID | 我们 | 优先级 |
|------|-----------|------|--------|
| **邻居选择** | **neighbor_size=8 智能选择** | ❌ 遍历所有 agent | **P0** |
| **时间分配** | **集中给8个邻居** | ❌ 分散给所有 agent | **P0** |
| SIPP | ✅ 启用 | ❌ 未实现 | P1 |
| ALNS 销毁策略 | RANDOMWALK/INTERSECTION | ❌ 未实现 | P1 |
| 并行执行 | OpenMP 并行 | ❌ 单线程 | P2 |
| LaCAM2 初始化 | ✅ 用于大规模 | ❌ 未实现 | P2 |

---

## 下一步（高优先级）

### 1. 修改 frank_wolfe 只选择部分邻居重规划
- 不是遍历所有 agent
- 而是根据冲突选择 neighbor_size=8 个邻居

### 2. 增加每个邻居的 deadline
- 不再是 `remaining_ms / num_of_agents`
- 而是 `remaining_ms / neighbor_size`

### 3. 实现邻居选择策略
- 简单版本：选择 deviation 最高的 N 个
- 完整版本：RandomWalk/Intersection 策略

---

## 参考文件

- `Best-Implementations-2024/Team_RAPID_3635cb44/src/LNS/LNSSolver.cpp`
- `Best-Implementations-2024/Team_RAPID_3635cb44/src/LNS/Parallel/NeighborGenerator.cpp`
- `Best-Implementations-2024/Team_RAPID_3635cb44/src/LNS/Parallel/GlobalManager.cpp`
- `Best-Implementations-2024/Team_RAPID_3635cb44/configs/random-32-32-20.json`
- `Best-Implementations-2024/Team_RAPID_3635cb44/inc/RHCR/main/SIPP.h`
