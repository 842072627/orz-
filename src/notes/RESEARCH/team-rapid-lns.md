# Team_RAPID LNS 算法研究

## 创建时间
2026-04-22 16:50

## 状态
进行中

---

## 核心发现

### Team_RAPID 的 LNS 配置 (random-32-32-20.json)

```json
"LNS": {
    "sipp": true,              // ✅ SIPP 启用！
    "neighborSize": 8,         // 邻居大小
    "initAlgo": "LaCAM2",      // 初始化用 LaCAM2
    "replanAlgo": "PP",       // 重规划用 PP
    "destoryStrategy": "Adaptive",
    "initLNS": false,
    "pibtWindow": 5,
    "winPibtSoftmode": true,
    ...
}
```

### 关键参数

| 参数 | Team_RAPID 值 | 我们当前值 | 状态 |
|------|---------------|------------|------|
| sipp | true | ❌ 未实现 | 缺失 |
| neighborSize | 8 | 8 (但未使用) | 未启用 |
| initAlgo | LaCAM2 | 无 | 缺失 |
| replanAlgo | PP | PIBT | 可能差异 |
| destoryStrategy | Adaptive | 无 | 缺失 |

---

## 我们的 LNS 实现 vs Team_RAPID

### 我们的代码
- `my_flow.cpp` 的 `frank_wolfe()` 函数实现了类似 LNS 的逻辑
- `deadline_per_agent = max(5, remaining_ms / agent_count)` - 自适应 deadline
- 但缺少真正的 neighbor 机制

### Team_RAPID 的 LNS
- 使用 `Parallel::GlobalManager` 管理邻居
- `neighborSize=8` 选择8个邻居进行重规划
- `Adaptive` 销毁策略 - 智能选择要销毁的代理
- `SIPP` 路径规划 - 安全间隔路径

---

## 缺失的核心功能

1. **SIPP (Safe Interval Path Planning)**
   - 考虑时间窗口的安全路径
   - 避免动态障碍
   - 我们完全没有实现

2. **真正的 LNS Neighbor 机制**
   - 不是随机选邻居
   - 有 Adaptive 策略根据冲突选择邻居
   - `neighborSize=8` 控制每次重规划的邻居数量

3. **LaCAM2 初始化**
   - 大规模场景(10000+ agents)使用 LaCAM2
   - 小规模场景使用 LNS

---

## 下一步研究

1. 深入理解 SIPP 算法实现
2. 研究 GlobalManager 的邻居选择逻辑
3. 研究 Adaptive 销毁策略

---

## 参考文件

- `Best-Implementations-2024/Team_RAPID_3635cb44/src/LNS/LNSSolver.cpp`
- `Best-Implementations-2024/Team_RAPID_3635cb44/src/LNS/Parallel/GlobalManager.h`
- `Best-Implementations-2024/Team_RAPID_3635cb44/configs/random-32-32-20.json`
