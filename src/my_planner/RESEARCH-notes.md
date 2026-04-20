# MAPF Deep Research Notes

> 深度研究笔记 - 沉淀真正的知识，不是跑测试记录

---

## 核心问题

**Q: 为什么我们的代码 (total≈1.61) 与冠军 (total≈4.0) 差距 2.5x？**

**A: 架构设计差异** - 不是参数调优能解决的

---

## Team_Kitty_Knight 冠军架构解析

### 1. TimeSpaceA* (时空A*)

**什么是TimeSpaceA***：
- 普通A*在2D空间 (x, y) 搜索
- TimeSpaceA* 在3D空间 (x, y, t) 搜索，同时考虑空间和时间冲突
- 关键：它可以预见未来的冲突并回避

**我们的astar现状**：
```cpp
// 我们的astar只考虑空间位置
s_node astar(..., int start, int goal, ...) {
    // h = 空间启发式（不考虑时间）
    // 产生的path是静态的，不考虑其他agent的动态运动
}
```

**冠军的TimeSpaceA***：
- 需要 search_node.h 增加 time 维度
- 需要在搜索时考虑边的"拥塞度"随时空变化
- 产生的是"流动感知"的路径

**实现难点**：
1. 状态空间从 N×M 变成 N×M×T (T是时间)
2. 需要维护一张"时空边权重表"
3. 启发式也需要是时空的

### 2. GlobalManager (全局管理器)

**什么是GlobalManager**：
- 跨多个MAPF episode协调
- 我们只有单episode规划 (一次plan调用)
- 冠军在episode之间共享流量/拥塞信息

**我们的架构**：
```
plan() → 内部multi-step模拟 → 返回actions → 结束
         (没有跨episode记忆)
```

**冠军的架构**：
```
episode 1: plan() → GlobalManager记录流量状态
episode 2: plan() → GlobalManager查询历史流量 → 更智能的guide path
```

**实现难点**：
1. 需要持久化流状态到下一episode
2. 需要在Entry.cpp / TaskScheduler.cpp增加全局协调层
3. 与现有 planner.cpp / flow.cpp 接口对接

### 3. single-step vs multi-step

**我们的multi-step**：
```cpp
// 一次plan调用，内部模拟num_steps步
plan(time_limit, actions, env, num_steps) {
    update_guide_paths_once_for_multistep(...);  // 只计算一次
    for (step = 0; step < num_steps; step++) {
        run_multistep_pibt_once(...);  // guide path不更新
    }
}
```

**冠军的single-step**：
```cpp
// 每次plan调用只计算下一步
plan(time_limit, actions, env, num_steps=1) {
    update_traj(...) + frank_wolfe(...) + PIBT(...)
}
// 下个 timestep 再次调用 plan()
```

**差距**：我们的guide path在内部模拟时stale了，冠军始终用fresh的guide path。

### 4. PIBT tie-breaking

**我们**：
```cpp
// 确定性：closer location优先
if (a_dist != b_dist) return a_dist < b_dist;
```

**冠军**：
```cpp
// 随机性：rand() tie-breaking
return a.tie_breaker < b.tie_breaker;
```

**问题**：我们用确定性避免rand()，但可能牺牲了探索性。
**我的立场**：不能为了性能牺牲reproducibility——但这可能是差距来源之一。

### 5. 数据结构

**冠军**：vector[int] 预分配，O(1)访问
**我们**：unordered_map[int,int] 哈希查找，O(1)平均但有常量因子

**影响**：对启发式查找性能有影响，但不是核心差距。

---

## 架构差距总结

| 组件 | 冠军 | 我们 | 性能影响 | 实现难度 |
|------|------|------|----------|----------|
| TimeSpaceA* | ✅ | ❌ | **巨大** | 极高 |
| GlobalManager | ✅ | ❌ | **巨大** | 极高 |
| single-step | ✅ | ❌ (multi-step) | 大 | 高 |
| rand() PIBT | ✅ | ❌ (deterministic) | 中 | 低(但牺牲reproducibility) |
| vectorheuristics | ✅ | ❌ (unordered_map) | 中 | 中 |

---

## 真正的研究任务

### 任务1：理解TimeSpaceA*实现

**目标**：彻底理解champion的search.cpp如何在A*中考虑时空冲突

**步骤**：
1. 精读 champion/search.cpp 的 astar() 函数
2. 精读 champion/search_node.h 的节点结构
3. 理解 flow 加权如何在搜索中被考虑
4. 写一份详细的算法分析

**产出**：一份"TimeSpaceA*实现分析.md"

### 任务2：理解GlobalManager设计

**目标**：理解champion如何在episode间共享状态

**步骤**：
1. 查找champion代码中所有GlobalManager相关文件
2. 追踪planner.cpp如何与GlobalManager交互
3. 理解flow状态如何在episode间保持

**产出**：一份"GlobalManager设计分析.md"

### 任务3：系统对比差异

**目标**：量化我们的代码与champion的100+文件差异

**步骤**：
1. 对比我们default_planner/与champion的default_planner/
2. 标记每个文件的差异
3. 评估每个差异对性能的影响

**产出**：一份"代码差异分析.md"

---

## 我的行为规则

1. **实验前写假设** - 预期X提升，因为Y原因
2. **最多测5次** - 没提升就停，分析原因
3. **失败立即反思** - 是假设错了还是方向错了
4. **20次测试上限** - 超过必须停下来问自己在干嘛
5. **写笔记** - 每次研究产出文档，不只是测试数据

---

## 重大发现：这不是TimeSpaceA*!

我之前误解了。Champions的astar实际上**不是**时空A*（在3D时空中搜索）。

**真正的算法**：A* with flow-influenced cost function

```
f = g + h + op_flow + all_vertex_flow
```

其中：
- `g` = 路径长度（从起点的步数）
- `h` = 启发式（到目标的曼哈顿距离）
- `op_flow` = 这条边上的**反向流量**
- `all_vertex_flow` = 经过这个顶点的总流量

**关键公式**（search.cpp第86行）：
```cpp
temp_op = ( (flow[curr_id].d[d]+1) * flow[next].d[(d+2)%4]);
// 如果我们向东(d=0)，检查下一个位置有多少agent想向西(d=2)
```

**所以搜索仍然是2D空间**，但启发式被**动态流量信息**丰富了。

**这对我意味着什么**：
- 我之前高估了"TimeSpaceA*"的实现难度
- 冠军的flow引导A*实际上我们的代码已经在用（frank_wolfe + flow）
- 差距不在于TimeSpaceA*，而在于**如何更好的利用flow信息**

### 真正的差距：single-step vs multi-step

由于champion每次plan()只计算**一步**，然后立即重新规划，所以：
1. 每次guide path都是fresh的（基于最新的flow状态）
2. 每次frank_wolfe优化的flow也是当前的

我们的multi-step模拟：
1. 一次plan()计算guide path
2. 内部模拟num_steps步
3. Guide path和flow都是**陈旧的**


**这就是核心差距**。不是算法差异，是架构差异。

---

## PIBT Tie-Breaking分析

**关键发现**：search_node.h 中的 `cmp_less_f` 和 `cmp_less_of` 都用 `rand() % 2`

这意味着：
1. 冠军的A*搜索有**非确定性tie-breaking**
2. 这不是"bug"，而是故意的**探索性**
3. 我们的确定性方法可能**牺牲了探索性**

---

## 进度追踪

- [x] TimeSpaceA*实现分析.md — **已更正：不是TimeSpaceA*，是flow-guided A***
- [ ] GlobalManager设计分析.md
- [ ] 代码差异分析.md
- [x] **新增发现**：single-step vs multi-step架构差异是核心差距
- [x] **新增发现**：rand() tie-breaking在A*中被使用

---

_Last updated: 2026-04-21 07:17 GMT+8_
