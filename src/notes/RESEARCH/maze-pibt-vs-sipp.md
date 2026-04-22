# maze 算法突破 - 关键发现 (2026-04-22 18:25)

## 核心结论

**maze 问题不是参数，而是算法本身。**

PIBT (Prioritized Iterative Best Priority) 无法有效处理 maze 的窄通道 (16.5%)。

## 为什么 PIBT 在 maze 上失败？

### PIBT 的工作原理
1. 按优先级顺序处理 agents
2. 每个 agent 选择最优移动方向
3. 如果与其他 agent 冲突，优先级低的让路

### maze 的问题
- **16.5% 窄通道** = 大量位置只有 1-2 个邻居
- 当多个 agent 同时进入窄通道时，PIBT 的"让路"机制失效
- 冲突解决策略过于简单，只考虑当前时间步

###PIB T vs SIPP 对比

| 特性 | PIBT | SIPP |
|------|------|------|
| 路径规划 | A* (最短路径) | 时间空间 A* |
| 冲突处理 | 隐式 (通过优先级) | 显式 (通过 interval) |
| 窄通道 | ❌ 无特殊处理 | ✅ 知道何时可以通行 |
| 规划 | 单步 | 多步 |

## Team_RAPID 的解决方案

### SIPP (Safe Interval Path Planning)

```cpp
// 核心概念：Interval - 某个位置在某个时间段可安全通行
struct Interval {
    int start_time;
    int end_time;  
};

// SIPP 搜索时会考虑：
// 1. 当前位置的可通行 interval
// 2. 相邻位置的 interval
// 3. 时间扩展的约束
```

### 关键优势

1. **知道何时可以通过窄通道**
   - 不是"能不能走"，而是"什么时候能走"
   
2. **预规划让路顺序**
   - Agent A 先过，Agent B 在 interval [5,10] 等候
   - Agent B 在 interval [10,15] 通过

3. **Reservation Table**
   - 记录所有位置的占用 interval
   - 搜索时检查冲突

## 参考代码

Team_RAPID 的 SIPP 实现：
- `inc/RHCR/main/SIPP.h` - SIPP 类
- `inc/RHCR/main/StateTimeAStar.h` - 时间空间 A*
- `inc/RHCR/main/ReservationTable.h` - 冲突表

## 下一步

1. 研究 Team_RAPID 的 SIPP 实现
2. 考虑两种方案：
   - **方案 A**: 实现 SIPP 作为 A* 的替代
   - **方案 B**: 修改 PIBT 使其考虑窄通道

## 笔记

maze 分数 0.090，差距 -88%。
参数调整全部失败（pibt_runtime=3,5,10 无效）。
需要算法层面的突破，不是参数调优。
