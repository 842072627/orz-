# MAPF Deep Research Notes

> 深度研究笔记 - 沉淀真正的知识，不是跑测试记录
> Last updated: 2026-04-21 19:50 GMT+8

---

## 核心问题

**Q: 为什么我们的代码 (total≈1.668) 与冠军 (total≈4.0) 差距 2.4x？**

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

**搜索仍是2D空间**，但启发式被动态flow信息丰富。

### 2. GlobalManager = LNS协调器

**不是简单的"全局状态管理"**，而是Large Neighborhood Search的并行协调框架。

### 3. PathTable = 冲突检测表

O(1)查询冲突，LNS跨neighborhood协调关键。

### 4. LaCAM2 vs frank_wolfe

**Champion用LaCAM2 + PLNS**：
- `init_algo_name = "LaCAM2"`
- `replan_algo_name = "PP"`

**我们用frank_wolfe + PIBT**：
- frank_wolfe优化SOC
- PIBT做冲突解决

**frank_wolfe被Champion注释掉了！**

### 5. Scheduler的差距

**Champion有多种scheduler策略**：
```cpp
if (configs["scheduler"]=="matching") {
    schedule_plan_matching();  // Hungarian最优分配
} else if (configs["scheduler"]=="greedy_matching") {
    schedule_plan_greedy_matching();
}
```

**我们的scheduler只有贪心！**

---

## 2026架构（V3.0.0 - 重大变化）

### Dual-Timescale Architecture
- Planner返回multi-step intent
- Executor通过TPG控制GO/STOP时序
- **Delay是核心机制，不是干扰！**

### Runtime Delay机制
- `env->delay_event_distribution` ("bernoulli" or "poisson")
- `env->delay_time_distribution` ("uniform" or "gaussian")

---

## 已验证无效方向

| 实验 | 改动 | 结果 |
|------|------|------|
| A14 | ConflictRepair后处理 | 1.668 (+1%) |
| A23 | 扩展ConflictRepair到20 agents | 1.097 (-4%) |
| A24 | Greedy matching scheduler | 1.612 (-3%) |
| A25 | 冲突感知frank_wolfe排序 | 1.531 (-8%) |
| A26 | 简化版LaCAM2冲突重规划 | 1.614 (-3%) |

**结论**：所有小改动都失败，frank_wolfe + PIBT架构天花板 ≈ 1.6-1.7

### 失败模式总结
1. **ConflictRepair方向**：增加处理范围反而让性能下降
2. **Scheduler方向**：greedy matching不如per-agent贪心
3. **排序方向**：按冲突排序让问题更糟
4. **LaCAM2简化方向**：重新规划冲突agents引入更多不稳定

**核心问题**：frank_wolfe优化SOC（路径长度和），比赛评分是Throughput（单位时间完成任务数）。
这两个目标不一致，导致所有基于SOC优化的改进都无效。

**真正原因**：frank_wolfe是批量重规划架构，PIBT是运行时冲突解决。
这种组合无法实现真正的"每步重规划"，所以天花板是1.6-1.7。

---

## 当前最佳

- **Commit**: `3ae0134` (A14 ConflictRepair)
- **Result**: total=1.668 (random=0.523, fulfill=1.145)
- **GAP**: 2.4x (1.668 → 4.0+)

---

## A方向候选

### A方向1: 实现Hungarian Scheduler（高优先）
- **假设**：scheduler是瓶颈之一
- **证据**：Champion有Hungarian最优分配，我们只有贪心
- **工作量**：1-2天
- **风险**：中等

### A方向2: 实现LaCAM2 + PLNS（极高风险）
- **假设**：LaCAM2比frank_wolfe更适合Lifelong MAPF
- **证据**：Champion用LaCAM2达到4.0+
- **工作量**：数周
- **风险**：极高

---

## 下一步

选择A方向1：实现Hungarian Scheduler

**理由**：
1. 工作量可控（1-2天）
2. 有明确的证据（Champion代码）
3. 风险中等
