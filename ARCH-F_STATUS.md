# ARCH-F 状态报告 (2026-04-13)

## 当前最佳基准
- **ARCH-D S1**: 3.555 (Quick Test) / 0.914 (3000-step)
- **目标**: 4.682 (差距 +1.127)

## ARCH-F 实验记录

| Round | 方向 | 状态 | 结果 |
|-------|------|------|------|
| R1 | Executor MCP 进度优先 | 失败-灾难 | 模拟超时（死锁） |
| R2 | TaskScheduler additive eagerness | 失败 | 模拟超时（公式过于激进） |

## 关键发现

1. **Executor 改动全部导致死锁**:
   - R306 逆向优先级（计算了但未使用，实际是 FIFO）
   - 近者优先（E1: 0.265 vs 0.481）
   - 远者优先（E2: 0.223 vs 0.481）
   - 进度优先（R1: 模拟超时）

2. **TaskScheduler 参数空间已饱和**:
   - min_plan_steps: *1.2 最优，*1.0/*1.5 都更差
   - 所有 P1 级别调整都未能超越 S1
   - additive eagerness 公式过于激进导致超时

3. **PIBT 时间调整有害**:
   - PIBT_RUNTIME_PER_100_AGENTS=2 更差（0.297 vs 0.321）
   - 原因：挤占 flow assignment 时间

## 核心问题
- TrajLNS 已集成 ✅
- 单次 PIBT 已最优 ✅
- **缺少迭代优化** ← 真正差距

## 建议下一步
实现「多次 PIBT 调用 + 选优」需要解决:
1. 如何扰动输入以获得不同 plan
2. 如何评估 plan 质量
3. 如何对应正确的 agent

或考虑完全重构 MAPFPlanner 使用 LaCAM2 核心思想
