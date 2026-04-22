# 冠军团队分析

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22
- 状态: 进行中

---

## 重要提醒

**云端冠军不是只有一个！**
比赛有多个团队的代码，每个团队用不同算法。

---

## 团队列表

| 团队 | 算法特点 | Random配置 |
|------|----------|------------|
| Team_Kitty_Knight | CausalPIBT + PLNS | causal_pibt |
| Team_RAPID | **LNS + LaCAM2** | LNS |
| Team_No_Man_Sky (3版本) | 待研究 | 待研究 |
| Team_SYSU-LCIS | 待研究 | 待研究 |

---

## Team_Kitty_Knight

### 基本信息
- 路径: `Best-Implementations-2024/Team_Kitty_Knight_da7d428b/`
- 特点: CausalPIBT + PLNS + TimeSpaceA* + GlobalManager

### Random 配置 (random-32-32-20.json)
```json
{
    "scheduler": "default",
    "planner": "causal_pibt",  // 不是 causal_pibt+plns!
    "map_weights_path": ""
}
```

### 关键发现
1. **frank_wolfe 被注释掉**: 代码中 frank_wolfe 完全没使用
2. **使用 Dist2PathHeuristicTable**: 快速启发式查询
3. **CausalPIBT 优先级继承**: 比普通 PIBT 更好

### 架构
```
CausalPIBT
├── Dist2PathHeuristicTable (启发式)
├── PLNSSolver (并行LNS) - 用于 causal_pibt+plns 模式
└── TimeSpaceAStar (时空A*)
```

---

## Team_RAPID (重要！)

### 基本信息
- 路径: `Best-Implementations-2024/Team_RAPID_3635cb44/`
- 特点: **LNS + LaCAM2 + RHCR + PBS + SIPP + ECBS**

### Random 配置 (random-32-32-20.json)
```json
{
    "lifelong_solver_name": [
        {"n_agents": 600, "value": "LNS"},
        {"n_agents": 1000000, "value": "LaCAM2"}
    ],
    "LNS": {
        "initAlgo": "LaCAM2",
        "replanAlgo": "PP",
        "destoryStrategy": "Adaptive",
        "neighborSize": 8,
        "pibtWindow": 5,
        "sipp": true
    }
}
```

### 关键发现
1. **对 random (400 agents) 使用 LNS 算法！**
2. LNS = Large Neighborhood Search
3. 使用 LaCAM2 作为初始化和重规划算法
4. SIPP 作为单智能体路径规划器

### LNS vs PIBT
- LNS: 邻域搜索，每次重规划多个智能体
- PIBT: 优先级继承，逐个智能体规划

### 对我们的意义
**Team_RAPID 用 LNS 在 random 上可能效果更好！**

---

## Team_No_Man_Sky

### 基本信息
- 路径: `Best-Implementations-2024/Team_No_Man_Sky_*/`
- 有3个版本: 44950d1b, e323feb6, faa15de9

### 待研究

---

## Team_SYSU-LCIS

### 基本信息
- 路径: `Best-Implementations-2024/Team_SYSU-LCIS_2d3d3529/`
- 待研究

---

## 算法对比

| 算法 | 描述 | 适用场景 |
|------|------|----------|
| CausalPIBT | 优先级继承 + 回溯 | 高密度地图 |
| LNS | 大邻域搜索 | 大规模问题 |
| LaCAM2 | 动态规划 | 长时规划 |
| PBS | 优先级树搜索 | 冲突解决 |
| SIPP | 安全intervals | 时空规划 |
| frank_wolfe | 流量优化 | 开放地形 |

---

## 待研究问题

1. [ ] Team_RAPID 的 LNS 在 local 测试效果如何
2. [ ] Team_No_Man_Sky 用什么算法
3. [ ] Team_SYSU-LCIS 用什么算法
4. [ ] 能否借鉴 LNS 改进我们的 planner
