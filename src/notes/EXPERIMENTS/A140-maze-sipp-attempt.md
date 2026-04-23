# A140: Maze SIPP 实现尝试 (失败)

## 状态: 已回退

## 尝试的方法

**ReservationTable-based PIBT for maze**:
1. 在每个 planning step 开始时，从 guide paths 构建 ReservationTable
2. 在 PIBT successor 选择时检查 ReservationTable
3. 避免选择会导致时间冲突的位置

## 失败原因

实现有 bug，导致 maze 分数从 0.09 降到 0.00（完全失败）。

问题分析：
1. ReservationTable 的时间间隔管理有问题
2. 从 guide paths 构建 RT 的逻辑不正确
3. congestion 计分逻辑有问题

## 教训

1. **ReservationTable 必须正确管理 (position, timestep) 对**
2. **简单的启发式方法不够** - 需要完整的 SIPP 实现
3. **时间冲突检测需要正确的 interval 数学**

## 下一步

1. **Option A**: 实现完整的 Team_RAPID 风格的 SIPP（工作量大）
2. **Option B**: 研究 Team_Kitty_Knight 的 causal PIBT 实现差异
3. **Option C**: 接受现实 - maze 需要架构级别改动，当前代码无法支持

## AWA 结论

对于 maze:
- **简单修改全部失败** (A117 corridor penalty, A140 ReservationTable)
- **骨架 SIPP 更差** (A138: 0.076 vs 0.090)
- **完整 SIPP 需要大量工作**

maze 是最难优化的地图之一。需要深入研究冠军团队的完整实现。

## 笔记
- 2026-04-23: A140 尝试失败，已回退