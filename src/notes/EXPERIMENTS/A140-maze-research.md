# MAZE 算法研究 (A140)

## 状态: 研究中

## 分数现状

| 地图 | 分数 | 目标 | GAP |
|------|------|------|-----|
| maze | **0.090** | 0.73 | -88% |

## 问题分析

### maze 特征
- 32x32 grid, 40 agents
- 35% obstacles, 16.5% narrow passages
- 目标: 500 steps 内完成 365 tasks
- 当前: 500 steps 只完成 45 tasks (8x差距!)

### 为什么 PIBT 在 maze 上失败?

PIBT 是贪心分布式算法:
1. 每个 agent 独立选择最优动作
2. 不考虑全局协调
3. 在开阔空间: 有多种路径选择，贪心OK
4. 在窄通道: 多个 agents 争用同一通道，贪心导致死锁

### frank_wolfe 为何崩溃?
- A113/A134 确认: 即使保守配置也崩溃
- 可能原因: maze 结构导致流量优化不收敛
- 注意: fulfill (71% narrow) 用 frank_wolfe 成功

## 冠军方案研究

### Team_RAPID (LNS + LaCAM2 + SIPP)
```json
"LNS": {
    "sipp": true,
    "neighborSize": 8,
    "initAlgo": "LaCAM2",
    "replanAlgo": "PP"
}
```

**关键**: Team_RAPID 使用 SIPP 处理窄通道!

### SIPP vs PIBT

| 特性 | PIBT | SIPP |
|------|------|------|
| 冲突处理 | 优先级继承 | 时间区间调度 |
| 路径规划 | 贪心 | A* + safe intervals |
| 适用场景 | 开阔空间 | 窄通道 |
| 协调能力 | 弱 | 强 |

## AWA 分析

### 已失败方案
| 实验 | 方法 | 结果 |
|------|------|------|
| A113 | frank_wolfe for maze | 崩溃 ❌ |
| A117 | narrow corridor penalty | 0.038 (更差) ❌ |
| A134 | frank_wolfe 保守配置 | 崩溃 ❌ |
| A138 | skeleton SIPP | 0.076 (更差) ❌ |

### 可选方向

**A: 完整 SIPP 实现** (最高风险/最高回报)
- 需要: ReservationTable + Interval 计算 + 完整集成
- 预期收益: maze 可能从 0.09 → 0.40+
- 工作量: 很大

**B: 因果 PIBT 研究** (中等风险/中等回报)
- Team_Kitty_Knight 用 Causal PIBT
- 可能对窄通道更友好
- 需要研究其实现

**C: PIBT 改进** (较低风险/较低回报)
- 添加 look-ahead 冲突检测
- 修改 corridor 避让逻辑
- 可能提升有限

## 下一步

1. 研究 Team_Kitty_Knight 的 causal PIBT 实现
2. 考虑简化版 SIPP (只处理 corridor)
3. 或接受需要架构级别改动

## 笔记
- 2026-04-23: 创建