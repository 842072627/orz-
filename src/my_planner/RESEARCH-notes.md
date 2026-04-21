# MAPF Deep Research Notes

> 深度研究笔记 - 沉淀真正的知识，不是跑测试记录
> Last updated: 2026-04-21 14:00 GMT+8

---

## 核心问题

**Q: 为什么我们的代码 (total≈1.65) 与冠军 (total≈4.0) 差距 2.4x？**

**A: 架构设计差异** - 不是参数调优能解决的

---

## 冠军架构解析

### 真正的架构：LNS主循环

**关键发现**：冠军的Dist2Path代码被注释掉了！

```cpp
// Dist2PathHeuristicTable.h 第34-39行
// if (!lns.traj_dists.empty() && !lns.traj_dists[agent_idx].empty())
//     min_heuristic = get_dist_2_path(lns.traj_dists[agent_idx], lns.env, loc, &(lns.neighbors));
// else
```

**冠军只使用静态启发式表**，没有使用Dist2Path！

### 架构差距总结

| 组件 | 冠军 | 我们 | 关键差异 |
|------|------|------|----------|
| 主循环 | LNS | frank_wolfe | **根本差异** |
| 启发式 | 静态表 | Dist2Path | 我们的更优（但没用） |
| 重规划 | 每步完全重规划 | guide path stale | **根本差异** |
| 冲突处理 | LNS neighborhood repair | PIBT因果链 | 不同机制 |

### 核心结论

**启发式不是瓶颈**！冠军甚至没用Dist2Path，但他们的LNS主循环架构让他们的性能是我们的2.4倍。

---

## 灾难性失败记录

| 方向 | 结果 | 原因 |
|------|------|------|
| A2 (LNS替换FW) | fulfill=0.338 | frank_wolfe是必须的 |
| B1 (window_size-1) | fulfill -83% | TPG依赖被破坏 |
| A7 (移除is_large_map) | fulfill -17% | is_large_map有道理 |
| A11 | fulfill -89% | **在PIBT决策后修改guide path** |
| A12 | fulfill -100% | **O(n)遍历导致超时/死锁** |

---

## 已验证事实

1. **frank_wolfe essential**: 去掉后fulfill -71%
2. **LNS不能叠加在frank_wolfe后面**: 目标冲突
3. **is_large_map检查有道理**: 移除后fulfill -17%
4. **DEV-336**: 当前最优的guide path更新策略
5. **threshold=5是经验最优**: 太高或太低都更差
6. **window_size不能随便改**: 减少导致fulfill -83%
7. **启发式不是瓶颈**: 冠军甚至没用Dist2Path
8. **不要在PIBT决策后修改guide path**: 灾难
9. **不要在PIBT排序中遍历整个map**: 灾难
10. **frank_wolfe天花板 ≈ 1.6-1.7**: 架构限制

---

## 核心结论

**frank_wolfe + PIBT架构的天花板 ≈ 1.6-1.7**

唯一能突破的方向：**LNS主循环架构重构**

但这需要：
1. 数周工作时间
2. 集成LaCAM2初始化
3. 实现完整的LNS框架（GlobalManager, NeighborGenerator, LocalOptimizer）
4. 实现TimeSpaceA* (SIPP变体)

风险：极高（可能在重写过程中引入新问题）
回报：理论上100%+（突破1.7天花板）

---

## 研究产出 (2026-04-21)

- commit `17c8a37` - Flow-guided A*真相
- commit `7040d47` - GlobalManager = LNS coordinator
- commit `519f390` - PathTable = conflict detection
- commit `e0927dd` - single-step vs multi-step architecture
- commit `5ece99f` - causalPIBT comparison
- commit `ed59b73` - A11失败记录
- commit `8794697` - A12失败记录
- commit `????` - **Dist2Path被注释！启发式不是瓶颈**

_Last updated: 2026-04-21 14:00 GMT+8_
