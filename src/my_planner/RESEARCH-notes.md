# MAPF Deep Research Notes

> 深度研究笔记 - 沉淀真正的知识，不是跑测试记录
> Last updated: 2026-04-21 14:00 GMT+8

---

## 核心问题

**Q: 为什么我们的代码 (total≈1.65) 与冠军 (total≈4.0) 差距 2.4x？**

**A: 架构设计差异** - 不是参数调优能解决的

---

## 冠军架构解析（最新发现）

### 真正的架构：LNS主循环，不是LaCAM2！

**我之前的误解**：以为冠军用LaCAM2作为主循环

**真相**：
- LaCAM2只是**初始化算法(initAlgo)**，不是主循环
- **真正的主循环是LNS**（Large Neighborhood Search）
- 配置：`lifelong_solver_name: "LNS"` (n_agents≥600时)

### LNS配置详解

```json
"LNS": {
    "initAlgo": "LaCAM2",        // 用LaCAM2初始化
    "replanAlgo": "PP",           // 用PP(PIBT)重规划
    "execution_window": 1,        // 每步执行！
    "pibtWindow": 5,              // PIBT窗口=5，类似我们的guide path
    "neighborSize": 8,            // LNS邻居大小
    "sipp": true,                 // 使用SIPP作为单agent求解器
}
```

### 关键参数对比

| 参数 | 冠军 | 我们 | 意义 |
|------|------|------|------|
| execution_window | **1** | 5 (window_size) | **每步replan！** |
| replanAlgo | PP (PIBT) | PIBT | 相同 |
| pibtWindow | 5 | 5 (DEV-336) | 相同机制 |
| main loop | **LNS** | frank_wolfe | **本质差距** |
| init | LaCAM2 | frank_wolfe | LaCAM2更强 |

---

## 架构差距总结

| 组件 | 冠军 | 我们 | 性能影响 | 实现难度 |
|------|------|------|----------|----------|
| **主循环** | LNS (每步replan) | frank_wolfe (多步window) | **巨大** | 极高 |
| 初始化 | LaCAM2 | frank_wolfe | 中 | 高 |
| 单agent求解 | SIPP | A* | 中 | 中 |
| 冲突避免 | LNS neighbor | PIBT | 中 | 高 |
| guide path | pibtWindow=5 | DEV-336 | - | 已实现 |

---

## 核心发现：frank_wolfe天花板

**frank_wolfe作为主循环的天花板 ≈ 1.6-1.7**

**原因**：
- frank_wolfe在plan()开始时分配flow并优化路径
- executor的window机制让guide path在多步内保持stale
- DEV-336通过deviation>5来缓解stale问题，但不是根本解决
- **冠军的架构是每步LNS重规划，不是multi-step window**

**所有"叠加LNS"尝试都失败**：
- A1-A7: 全部失败
- A8: LNS destroy-repair循环 - total=1.620
- A9: PIBT higher_id - total=1.581
- A10: guide path 5→2 - total=1.590
- **A11: 冲突检测guide path修复 - 灾难性失败 (fulfill -89%)**

---

## 灾难性失败记录

| 方向 | 结果 | 原因 |
|------|------|------|
| A2 (LNS替换FW) | fulfill=0.338 | frank_wolfe是必须的 |
| B1 (window_size-1) | fulfill -83% | TPG依赖被破坏 |
| A7 (移除is_large_map) | fulfill -17% | is_large_map有道理 |
| A11 (冲突检测guide path) | fulfill -89% | **破坏了PIBT决策的一致性** |

---

## A11失败根因分析

**失败的操作**：在`refresh_multistep_step_state`中检测冲突并修改guide path

**为什么失败**：
1. PIBT在step>0时使用当前的guide path做决策
2. 如果在PIBT决策后修改guide path，会导致不一致
3. PIBT假设guide path在整个step期间保持不变
4. 修改guide path会破坏这个假设，导致死锁或冲突

**教训**：
- **不要在PIBT决策后修改guide path**
- guide path的修改必须在PIBT运行之前完成
- 冲突解决应该是PIBT内部机制，而不是事后修复

---

## 已验证事实

1. **frank_wolfe essential**: 去掉后fulfill -71%
2. **LNS不能叠加在frank_wolfe后面**: 目标冲突（frank_wolfe优化SOC，LNS减少冲突）
3. **is_large_map检查有道理**: 移除后fulfill -17%
4. **DEV-336 (deviation>5 recompute)**: 是当前最优的guide path更新策略
5. **threshold=5是经验最优**: 太高(2)或太低(3)都更差
6. **window_size不能随便改**: 减少导致fulfill -83%
7. **PIBT higher_id exception**: 我们没有反而更好
8. **rand() tie-breaking**: 比deterministic轻微改善
9. **LaCAM2只是initAlgo**: 真正主循环是LNS，每步replan
10. **不要在PIBT决策后修改guide path**: 会导致灾难性的不一致

---

## A11方向：结论

**A11尝试失败的根本原因**：
- 试图在PIBT决策后修改guide path
- 这破坏了PIBT的一致性假设
- 导致fulfill从1.142暴跌到0.129

**真正需要做的**：
- 要么在PIBT决策前修改guide path
- 要么完全重构为LNS主循环

**当前状态**：
- 所有小改动都已验证失败
- 架构限制已确认
- 继续小改动没有意义

---

## 研究产出 (2026-04-21)

- commit `17c8a37` - Flow-guided A*真相
- commit `7040d47` - GlobalManager = LNS coordinator
- commit `519f390` - PathTable = conflict detection
- commit `e0927dd` - single-step vs multi-step architecture
- commit `5ece99f` - causalPIBT comparison + heuristics data structure
- commit `????` - **LaCAM2只是initAlgo，LNS才是主循环**
- commit `????` - **A11尝试失败：在PIBT后修改guide path导致灾难**

_Last updated: 2026-04-21 14:00 GMT+8_
