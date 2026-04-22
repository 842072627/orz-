# maze 地图算法突破研究 (2026-04-22)

## 当前状态

- **maze 分数**: 0.090 (目标 0.727，差距 -88%)
- **问题**: 只有 40 agents 但分数最低，说明算法有问题

## 地图特征

| 特征 | 值 |
|------|-----|
| 地图大小 | 32x32 |
| 障碍物% | 35% |
| 狭窄度 | 16.5% |
| Agents | 40 |
| 特征 | 中等狭窄，需要 SIPP 处理窄通道 |

## 为什么分数这么低？

### 可能原因分析

1. **PIBT 实现问题**
   - 只用 40 个 agents，分数应该更高
   - Team_Kitty_Knight 用 PIBT 得到好成绩

2. **窄通道冲突**
   - maze 有 16.5% 窄通道
   - 普通 A* 无法有效处理时间冲突
   - SIPP 能找到让路方案

3. **frank_wolfe 被禁用**
   - `should_skip_frank_wolfe` 返回 true
   - 但 frank_wolfe 会崩溃（150 agents 就会超时）
   - 40 agents 应该不会超时？

### 时间检查

```cpp
// my_planner.cpp:37
bool should_skip_frank_wolfe(SharedEnvironment* env) {
    // Skip frank_wolfe for maps where it tends to timeout
    if (name.find("room") != std::string::npos ||
        name.find("maze") != std::string::npos ||  // <-- maze 被跳过
        name.find("iron") != std::string::npos ||
        name.find("orz") != std::string::npos) {
        return true;
    }
}
```

## SIPP 实现分析 (Team_RAPID)

### 核心概念

SIPP (Safe Interval Path Planning) 的关键：
- **Interval**: 某个位置在某个时间段可安全通行
- **窄通道**: 同一位置在不同时间被多个 agent 占用
- **让路**: SIPP 可以规划让路方案

### Team_RAPID 的 SIPP 配置

```json
"LNS": {
    "sipp": true,  // 启用 SIPP
    "neighborSize": 8,
    ...
}
```

### 关键文件

- `inc/RHCR/main/SIPP.h` - SIPP 实现
- `inc/RHCR/main/StateTimeAStar.h` - 带时间的状态空间 A*
- `inc/RHCR/main/ReservationTable.h` - 冲突表

## 实施计划

### Phase 1: 启用 frank_wolfe for maze (A113)
- 修改 `should_skip_frank_wolfe` 让 maze 用 frank_wolfe
- 测试 40 agents 是否会超时

### Phase 2: SIPP 实现 (A114)
- 参考 Team_RAPID 的 SIPP 实现
- 创建 `my_SIPP.h/cpp`
- 修改 `my_search.cpp` 使用 SIPP

### Phase 3: 优化 PIBT (A115)
- 检查 Team_Kitty_Knight 的 PIBT 参数
- 调整 window_size 或其他参数

## 风险

1. **frank_wolfe 超时**: 如果 40 agents 也会超时，需要别的方案
2. **SIPP 实现复杂**: 需要理解 interval 概念
3. **破坏其他地图**: 必须用 `env->map_name` 判断

## 参考资料

- Team_RAPID SIPP: `Best-Implementations-2024/Team_RAPID_3635cb44/inc/RHCR/main/SIPP.h`
- SIPP 论文: Safe interval path planning for dynamic environments
