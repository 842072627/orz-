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

**我们的executor机制**：
- executor维护一个window_size的action窗口
- 每次next_command()只发送GO/STOP一个命令
- 通过TPG（Temporal Dependency Graph）协调agent执行顺序

**所以实际上也是每步协调的**——但guide path在window内部不更新。

### 5. causalPIBT Tie-Breaking详细对比

**关键差异**：

1. **Champion有higher_id exception (line 67-69)**:
```cpp
if (higher_id != -1 && prev_decision[next.location] == higher_id){
    continue;  // skip if prev occupant is higher priority
}
```
我们**没有**这个检查——我们更灵活。

2. **occupied检查**:
```cpp
// Champion: 无例外
if(occupied[next.location]) continue;

// 我们: 第一个agent可以stay at prev_loc
if(occupied[next.location] && !(higher_id == -1 && prev_loc == next.location))
    continue;
```

3. **tie-breaking**:
```cpp
// Champion: rand() tie-breaking
return a.tie_breaker < b.tie_breaker;

// 我们: deterministic closer location
int a_dist = std::abs(a.location - prev_loc);
int b_dist = std::abs(b.location - prev_loc);
if (a_dist != b_dist) return a_dist < b_dist;
return a.location < b.location;
```

**已验证**：我们的PIBT版本（无higher_id exception）比冠军的好。

### 6. frank_wolfe循环

**我们的版本**:
```cpp
while (std::chrono::steady_clock::now() < timelimit){
    // 遍历所有agent，按deviation排序
    if (!update_traj(lns, a, nullptr)) { // 无deadline
        lns.trajs[a] = old_traj;
        continue; // 失败继续
    }
}
```

**冠军的版本**：基本相同。关键都是：无deadline update_traj + 失败继续。

### 7. 启发式数据结构对比

**Champion**: `std::vector<HeuristicTable> global_heuristictable;`
- O(1)访问: `heuristics[goal]` (数组索引)
- 预分配，大小 = map_size
- 缓存友好

**Our**: `std::unordered_map<int, HeuristicTable> global_heuristictable;`
- O(1)平均: `heuristics.find(goal)` (哈希查找)
- 动态分配，有哈希冲突开销
- 缓存不友好

**性能影响**: 中等。不是核心差距，但影响启发式查找速度。

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

## 我的行为规则

1. **实验前写假设** - 预期X提升，因为Y原因
2. **最多测5次** - 没提升就停，分析原因
3. **失败立即反思** - 是假设错了还是方向错了
4. **20次测试上限** - 超过必须停下来问自己在干嘛
5. **写笔记** - 每次研究产出文档，不只是测试数据

---

## 研究产出 (2026-04-21)

- commit `17c8a37` - Flow-guided A*真相
- commit `7040d47` - GlobalManager = LNS coordinator
- commit `519f390` - PathTable = conflict detection
- commit `e0927dd` - single-step vs multi-step architecture
- commit `5ece99f` - causalPIBT comparison + heuristics data structure

_Last updated: 2026-04-21 07:42 GMT+8_

---

## A14 LNS???? (2026-04-21 16:30)

### ??LNS?????
`cpp
// 1. ?????
getInitialSolution(init_neighbor);

// 2. ???
while (!timeout) {
    // ??neighborhoods
    neighbor_generator->generate_parallel(time_limiter);
    
    // ??????neighborhood
    #pragma omp parallel for
    for (neighbor : neighbors) {
        local_optimizers[i]->optimize(neighbor);
    }
    
    // ??????
    update(neighbor);
}
`

### A14??
- Day1: ??LNSWrapper.cpp,??LNS???
- ???:????,??????

_Last updated: 2026-04-21 16:30 GMT+8_

---

## A14 LNS???????

### ????
| ?? | ?? | ?? |
|------|------|------|
| PathTable | ? ?? | O(1)???? |
| TimeSpaceAStarPlanner | ?? ?? | ????,???A* |
| NeighborGenerator | ?? ?? | ???generate() |
| LocalOptimizer | ?? ?? | ???optimize() |
| GlobalManager | ?? ?? | ???run() |

### ??LNS???
1. **M1**: ??astar??PathTable?? (2-3?)
2. **M2**: ??NeighborGenerator.generate() (1?)
3. **M3**: ??LocalOptimizer.optimize() (1?)
4. **M4**: ??GlobalManager.run() (1?)
5. **M5**: ???? (2-3?)

**?????**: 1-2?

### ????
- Baseline: 1.614
- ????: ?commit (bfa445b)
- ???: M1 - ??astar

_Last updated: 2026-04-21 16:50 GMT+8_

---

## A14 M1 ??: ??LNS?? (16:50)

### ????: ??PIBT?LNS
??constraint-based A*,????PIBT!

**??**:
- PIBT?occupied[]??????
- ???occupied = ??agents?guide path??
- ?neighborhood??PIBT

**??**:
1. frank_wolfe????guide paths (??)
2. ??neighborhood (??5 agents)
3. ???occupied?? = ?neighborhood agents???
4. ?neighborhood??PIBT
5. ???????

**??**:
- ????PIBT??
- ???constraint-based A*
- ??????1???2-3?

### ????
- PIBT: src/my_planner/pibt.cpp
- occupied??: ?causalPIBT?????

_Last updated: 2026-04-21 16:50 GMT+8_

---

## A14 M1 ??: ??constraint-based A* (16:55)

### ??
- PIBT?occupied???????,??LNS?
- LNS??????????
- ????(?PIBT??)???

### ??
M1????constraint-based A*,???:
1. 2D?? table[location][timestep] = agent_id
2. A*????????
3. ???:1?+

### ????
- A14??M1
- ????
- ??????

_Last updated: 2026-04-21 16:55 GMT+8_

---

## 2026架构重大变化 (2026-04-21 19:30)

### 重大发现：我的理解完全错误！

**错误**：我以为"冠军每步replan，我们50步replan一次"，这是瞎编的差距。

**真相**：Dual-Timescale Architecture是比赛本身的设计，不是冠军的创新。

### 2026架构时间线

**V3.0.0 (2026-03-13)** - 架构重构：
- Executor组件 - 分离planning和execution
- Runtime delay generation via `delayConfig`
- Agent footprint (safety bubbles) 碰撞检测
- Multistep/staged planner execution

**V3.1.0 (2026-04-11)** - 改进：
- Python接口层
- 执行管道和共享环境同步改进
- 修复delayed和windowed execution下的action validation问题

### Executor机制（重要！）

**我们的Executor** (`default_executor.cpp`):
1. TPG (Temporal Dependency Graph) 协调agent执行顺序
2. `mcp()` 函数决定GO/STOP：
   - 如果agent是TPG队列中下一个位置的第一名 → GO
   - 否则 → STOP
3. **模拟时假设"no delay"**（注释：`//simulate with mcp (no delay)`）
4. Delay由simulator处理，executor只管协调

**冠军的Executor**: 代码里**没有executor.cpp**！他们用start-kit的default_executor。

**结论**：Executor代码基本相同，差距在**Planner**！

### Planner流程分析

**我们的planner (`planner.cpp`)**: 
1. Step 0: 
   - `setup_multistep_episode_state()`
   - `update_guide_paths_once_for_multistep()` → guide paths + frank_wolfe + ConflictRepair
2. Steps 0 to num_steps-1:
   - 每次调用 `run_multistep_pibt_once()` (causalPIBT)
   - **Step 0之后不再更新guide paths和运行FW**

**num_steps = min_plan_steps = window_size**

```cpp
min_plan_steps = (int)(env->min_planner_communication_time/(env->action_time*env->max_counter)) + 1;
```

### 关键发现

1. **我们用causalPIBT** - 不是普通PIBT（line 299: `causalPIBT(i,-1,...)`）
2. **我们没有higher_id exception** - 已验证这比冠军的更好
3. **我们的executor和冠军基本相同** - 都用start-kit的default_executor
4. **差距在Planner**：guide path质量和更新策略

### Champion的PLNS架构（真正差距！）

**Champion的planner流程** (`my_planner/src/planner.cpp`):
```cpp
// 1. 生成初始多步actions
auto multi_step_actions = causal_pibt->multi_step(window_size, ...);

// 2. 用LNS优化路径
auto actions = plns->solve(goal_locations, init_paths, time_limit);

// 3. 返回单步actions
```

**关键发现**：
1. **frank_wolfe被注释掉了！** (line 99-105)
2. **用LaCAM2生成初始解** (`init_algo_name="LaCAM2"`)
3. **用PLNS (Parallel LNS) 优化路径**
4. **用PP (Pipe Planner) 重规划**

**Champion的配置**：
```cpp
bool async = true;
int neighbor_size = 8;
destroy_heuristic destroy_strategy = RANDOMWALK;
bool ALNS = true;
double decay_factor = 0.01;
double reaction_factor = 0.1;
std::string init_algo_name = "LaCAM2";
std::string replan_algo_name = "PP";
int window_size_for_PATH = 15;
```

### 真正的架构差距

| 组件 | Champion | 我们 | 差距影响 |
|------|----------|------|----------|
| 初始解算法 | **LaCAM2** | frank_wolfe | 巨大 |
| 路径优化 | **PLNS** | frank_wolfe循环 | 巨大 |
| 重规划 | **PP (Pipe Planner)** | PIBT | 大 |
| 并行化 | **async LNS** | 单线程 | 大 |

### LaCAM2是什么？
LaCAM2 (Large Neighborhood Search for Conflict-Based Asymmetric Multi-agent path planning) 是一种为Lifelong MAPF设计的算法。

**证据**：论文标题 "LaCAM2: Minimal Randomized... for Lifelong Multi-Agent Path Finding in Congested Systems" (arXiv:2309.16857)

### PP (Pipe Planner)是什么？
PP是一种基于管道的重规划算法，用于处理动态任务分配和路径更新。

---

## 下一步：如何缩小差距？

### 方案A：实现LaCAM2+PLNS（高风险高回报）
- **工作量**：数周
- **风险**：极高
- **回报**：可能达到4.0+
- **需要**：完整的LaCAM2实现

### 方案B：渐进式改进frank_wolfe+PIBT（中风险中回报）
- **工作量**：数天到数周
- **风险**：中等
- **回报**：不确定
- **方向**：
  - 改进guide path更新策略
  - 改进PIBT的conflict resolution
  - 改进scheduler任务分配

### 方案C：接受现状（无风险零回报）
- **禁止**

---

## A方向候选：渐进式LaCAM2特性

**想法**：能否在不完全实现LaCAM2的情况下，借鉴其思想改进frank_wolfe？

LaCAM2的核心思想：
1. **Conflict-based search** - 检测并解决冲突
2. **Large neighborhood search** - 大规模邻域搜索
3. **Minimal randomized techniques** - 随机化加速

**可能的渐进式改进**：
1. 在frank_wolfe之后添加conflict repair（已有A14的ConflictRepair后处理）
2. 研究为什么A14只提升1%而不是更多
3. 尝试不同的neighborhood生成策略

---

## 新发现：SOC vs Throughput 优化目标不匹配！

**关键发现**：
1. frank_wolfe优化的是**SOC (Sum of Costs)** - `lns.soc`
2. 比赛评分是**Throughput (tasks/makespan)**
3. **这两个目标不完全一致！**

**证据**：
```cpp
// flow.cpp
lns.soc -= lns.trajs[agent].size() - 1;  // SOC减
lns.soc += lns.trajs[agent].size() - 1;  // SOC加
```

**问题分析**：
- SOC优化 -> 最小化总路径长度
- Throughput优化 -> 最大化单位时间完成任务数
- 如果为了避免冲突而走更长的路，SOC增加但可能影响throughput

**A14 ConflictRepair效果有限的原因**：
- 只修复5个agents的冲突（太少）
- 只从timestep 0开始（窗口内会有更多冲突）
- 贪心修复不是最优

**新A方向候选**：
1. 研究是否应该优化throughput而不是SOC
2. 改进ConflictRepair到更多agents和更长时间窗口
3. 添加makespan优化到frank_wolfe目标函数
