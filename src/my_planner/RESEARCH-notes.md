# MAPF Deep Research Notes

> 深度研究笔记 - 沉淀真正的知识，不是跑测试记录
> Last updated: 2026-04-21 13:50 GMT+8

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
| main loop | **LNS** | frank_wolfe | **本质差距** |

---

## 灾难性失败记录

| 方向 | 结果 | 原因 |
|------|------|------|
| A2 (LNS替换FW) | fulfill=0.338 | frank_wolfe是必须的 |
| B1 (window_size-1) | fulfill -83% | TPG依赖被破坏 |
| A7 (移除is_large_map) | fulfill -17% | is_large_map有道理 |
| **A11** | fulfill -89% (1.142→0.129) | **在PIBT决策后修改guide path导致不一致** |
| **A12** | fulfill -100% (1.142→0.000) | **冲突检测遍历整个map导致超时/死锁** |

---

## A11 & A12 失败根因分析

### A11失败
- **操作**：在`refresh_multistep_step_state`中检测冲突并修改guide path
- **根因**：在PIBT决策后修改guide path，破坏了PIBT的一致性假设
- **教训**：guide path的修改必须在PIBT运行之前完成

### A12失败
- **操作**：在PIBT successor排序中添加冲突检测
- **根因**：每次排序时遍历整个map（可能几万甚至几十万节点），导致超时或死锁
- **教训**：不要在热路径中进行O(n)复杂度的操作

---

## 已验证事实

1. **frank_wolfe essential**: 去掉后fulfill -71%
2. **LNS不能叠加在frank_wolfe后面**: 目标冲突
3. **is_large_map检查有道理**: 移除后fulfill -17%
4. **DEV-336 (deviation>5 recompute)**: 是当前最优的guide path更新策略
5. **threshold=5是经验最优**: 太高(2)或太低(3)都更差
6. **window_size不能随便改**: 减少导致fulfill -83%
7. **PIBT higher_id exception**: 我们没有反而更好
8. **rand() tie-breaking**: 比deterministic轻微改善
9. **不要在PIBT决策后修改guide path**: 会导致灾难
10. **不要在PIBT排序中遍历整个map**: 会导致超时

---

## 核心结论

**frank_wolfe + PIBT架构的天花板 ≈ 1.6-1.7**

所有尝试都失败了：
- A1-A7: frank_wolfe + LNS叠加 - 全部失败
- A8: LNS destroy-repair - total=1.620
- A9: PIBT higher_id - total=1.581
- A10: guide path 5→2 - total=1.590
- **A11: Guide path修改 - fulfill -89% (灾难)**
- **A12: PIBT排序修改 - fulfill -100% (灾难)**

**真正需要做的**：
- LNS主循环架构重构
- 但这需要数周工作

---

## 研究产出 (2026-04-21)

- commit `17c8a37` - Flow-guided A*真相
- commit `7040d47` - GlobalManager = LNS coordinator
- commit `519f390` - PathTable = conflict detection
- commit `e0927dd` - single-step vs multi-step architecture
- commit `5ece99f` - causalPIBT comparison + heuristics data structure
- commit `ed59b73` - A11失败记录
- commit `????` - **A12失败记录**

_Last updated: 2026-04-21 13:50 GMT+8_
