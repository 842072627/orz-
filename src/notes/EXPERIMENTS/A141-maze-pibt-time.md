# A141: Maze PIBT Time Experiment (失败)

## 状态: 已回退

## 假设
增加 PIBT runtime 可以让 agents 在 narrow passages 中更好地协调，从而提高 maze 分数。

## 改动
```cpp
// my_planner.cpp - get_pibt_runtime_per_100_agents
if (name.find("maze") != std::string::npos) {
    return 10;  // 从 3 增加到 10
}
```

## 结果
- maze: 0.09 (无变化)

## 结论
❌ 增加 PIBT 时间对 maze 无效。

问题不是 planning 时间不足，而是 PIBT 算法本身不适合 narrow passages。PIBT 是贪心分布式算法，在窄通道中无法有效协调。

## 关键发现
- PIBT 时间从 3 增加到 10 没有任何影响
- 说明问题不是计算资源不足
- 是算法架构问题：PIBT 不适合窄通道地图

## 下一步
对于 maze，需要实现 SIPP 或类似的时间空间路径规划算法。简单的参数调整无效。

## 教训
- 不要在错误的假设上浪费时间
- 问题可能是算法层面的，不是参数层面的