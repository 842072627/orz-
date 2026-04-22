# AWA 分析 - 决定实现 SIPP (2026-04-23)

## 当前状态总结

| 地图 | 分数 | 目标 | GAP | 需要 |
|------|------|------|-----|------|
| fulfill | 1.073 | 0.83 | +29% ✅ | 保持 |
| random | 0.520 | 0.98+ | -47% ❌ | **SIPP** |
| room | 0.189 | 0.88 | -79% ❌ | **SIPP** |
| maze | 0.090 | 0.73 | -88% ❌ | **SIPP** |
| bos | 0.297 | ??? | ??? | 研究 |
| orz | 0.215 | 0.86 | -75% ❌ | 研究 |
| iron | 0.151 | 0.86 | -83% ❌ | 研究 |

## 已失败实验

| 实验 | 方法 | 结果 |
|------|------|------|
| A117 | narrow corridor penalty | 失败 ❌ |
| A126 | neighbor_size=16 | 失败 ❌ |
| A127 | deadline*1.5 | 失败 ❌ |
| A133 | 冲突检测 | 失败回退 ❌ |
| A134 | frank_wolfe for maze | 崩溃 ❌ |
| A135 | DefaultPlanner for maze | 略好但无效 |
| A137 | neighbor_size=8 for DefaultPlanner | 失败 ❌ |

## 核心问题

**参数调优全部失败。问题在架构级别:**

1. **Our A***: 使用 MemoryPool，每个位置只存一个状态
2. **Team_RAPID A***: State = (pos, orient, timestep)，每个时间步一个状态
3. **差距**: 我们的 A* 不能追踪时间维度的冲突

## Team_RAPID SIPP 关键机制

```cpp
// StateTimeAStarNode
struct StateTimeAStarNode {
    State state;  // (position, timestep)
    double g_val, h_val;
    int conflicts;  // 冲突计数
    StateTimeAStarNode* parent;
};

// SIPP 搜索时检查 ReservationTable
Interval interval = rt.getFirstSafeInterval(next_location);
if (isFree(interval, next_time)) {
    // 可以扩展
}
```

## 实现计划

### Phase 1: 添加时间维度到 s_node
修改 `my_search_node.h` 添加 `timestep` 字段

### Phase 2: 修改 MemoryPool
支持同一位置多个时间状态

### Phase 3: 添加 ReservationTable
使用现有的 `my_reservation.h`，但需要正确集成

### Phase 4: 修改 A* 扩展逻辑
扩展节点时检查 ReservationTable

## 风险
- **极高**: 需要架构改动，可能破坏现有功能
- **不确定**: 可能需要大量调试

## 回报
- 如果成功: random 0.52 → 0.98+, room 0.19 → 0.50+, maze 0.09 → 0.30+

## AWA 选择

根据 AWA 原则，当参数调优失败时，选择最高风险/最高回报的方向。

**选择: 实现 SIPP (A130 续)**

- 风险: 极高
- 回报: 高 (所有地图可能提升)
- 这是唯一可行的重大改进方向

## 状态
- 创建时间: 2026-04-23
- 状态: 开始实现
- 备注: 这是最后的高风险选择