# A13方向分析: LNS架构集成的最小可行方案

## 重大发现（2026-04-21 14:00）

### 冠军的Dist2Path被注释掉了！

查看`Dist2PathHeuristicTable.h`第34-39行：

```cpp
// if (!lns.traj_dists.empty() && !lns.traj_dists[agent_idx].empty())
//     min_heuristic = get_dist_2_path(lns.traj_dists[agent_idx], lns.env, loc, &(lns.neighbors));
// else
```

**冠军只使用静态启发式表**，没有使用Dist2Path！

### 结论

1. **启发式差异不是关键** - 冠军甚至没用Dist2Path
2. **架构差异才是关键** - LNS主循环 vs frank_wolfe主循环
3. 即使完美实现Dist2Path也不能解决问题

## 候选方案

### A13a: 完全重写LNS主循环
- 工作量：数周
- 风险：极高
- 潜在收益：100%+
- 前提：确认没有其他简单方案

### A13b: 尝试一个简单方案
- 既然启发式不是问题，那问题可能在其他地方
- 但14次实验都失败了，没有简单方案

## 决策

经过深入研究，结论很清楚：
1. 启发式不是瓶颈
2. frank_wolfe + PIBT架构有天花板
3. 唯一突破方案是LNS主循环

**状态：需要用户决定是否进行数周的LNS重写**
