# 地图分析笔记

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22
- 状态: 进行中

---

## 快速参考（必须记住）

| 地图 | 大小 | Agents | Planner | frank_wolfe | 分数 | 备注 |
|------|------|--------|---------|-------------|------|------|
| random | 64x64 | 400 | DefaultPlanner | N/A | 0.52 | MyPlanner崩溃 |
| bos | ~70k cells | 600 | DefaultPlanner | N/A | 0.31 | 待研究 |
| room | ~70k cells | 150 | MyPlanner | ❌ | 0.15 | tight地图 |
| maze | ~70k cells | 40 | MyPlanner | ❌ | 0.09 | tight地图 |
| fulfill | ~70k cells | 2500 | MyPlanner | ✅ | 1.07 | 我们的强项 |
| orz | ~70k cells | 1800 | MyPlanner | ❌ | 0.22 | tight地图 |
| iron | ~70k cells | 10000 | MyPlanner | ❌ | 0.15 | tight地图 |

---

## Random 地图

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22

### 基本信息
- 地图: random-64-64-10.map
- Agents: 400
- Steps: 300
- Planner: **DefaultPlanner** (MyPlanner 崩溃)

### 分数计算
```
score = tasks / makespan = 157 / 300 ≈ 0.523
```

### 已知问题
1. **MyPlanner 崩溃**: 原因不明，PIBT 实现差异
2. **frank_wolfe 有帮助**: 跳过 frank_wolfe 会变差

### 待研究
- [ ] 为什么 DefaultPlanner 可以，MyPlanner 不行
- [ ] Team_RAPID 用 LNS，效果如何
- [ ] Delay 机制如何利用

---

## Bos 地图

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22

### 基本信息
- 地图: city.domain
- Agents: 600
- Steps: 700
- Planner: **DefaultPlanner**

### 分数计算
```
score = tasks / makespan = 215 / 700 ≈ 0.307
```

### 问题
- 分数很低 (0.31)，是所有地图中第二差的
- 使用 DefaultPlanner，不是我们的强项

### 待研究
- [ ] 为什么分数这么低
- [ ] 是否可以切换到 MyPlanner

---

## Room 地图

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22

### 基本信息
- Agents: 150
- Steps: 400
- Planner: **MyPlanner** (跳过 frank_wolfe)

### 分数计算
```
score = tasks / makespan = 61 / 400 ≈ 0.152
```

### 已验证策略
- ✅ frank_wolfe 跳过（避免超时）

---

## Maze 地图

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22

### 基本信息
- Agents: 40
- Steps: 500
- Planner: **MyPlanner** (跳过 frank_wolfe)

### 分数计算
```
score = tasks / makespan = 45 / 500 ≈ 0.090
```

### 问题
- 分数最低 (0.09)
- Agents 很少但分数很差

### 待研究
- [ ] 为什么 agents 少但分数低
- [ ] 地图复杂度是否是问题

---

## Fulfill 地图

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22

### 基本信息
- 地图: warehouse.domain / fulfill-example_2500.json
- Agents: 2500
- Steps: 1000
- Planner: **MyPlanner** (使用 frank_wolfe)

### 分数计算
```
score = tasks / makespan = 1066 / 1000 ≈ 1.066
```

### 关键发现
- ✅ **我们的强项！超过所有冠军！**
- frank_wolfe 流量优化显著提升分数

### 已验证策略
- ✅ frank_wolfe 开启（流量优化有效）
- ✅ MyPlanner 比 DefaultPlanner 好

### 待研究
- [ ] 能否进一步提升到 1.2+
- [ ] 是否存在更优的参数配置

---

## Orz 地图

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22

### 基本信息
- Agents: 1800
- Steps: 400
- Planner: **MyPlanner** (跳过 frank_wolfe)

### 分数计算
```
score = tasks / makespan = 86 / 400 ≈ 0.215
```

### 已验证策略
- ✅ frank_wolfe 跳过（避免超时）

---

## Iron 地图

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22

### 基本信息
- Agents: 10000
- Steps: 700
- Planner: **MyPlanner** (跳过 frank_wolfe)

### 分数计算
```
score = tasks / makespan = 103 / 700 ≈ 0.147
```

### 问题
- Agents 很多 (10000) 但分数低
- 可能是 tight 地形导致冲突多

### 已验证策略
- ✅ frank_wolfe 跳过（避免超时）

---

## 下一步研究优先级

1. **🔥 Fulfill**: 我们的强项，进一步提升
2. **🔥 Random**: 理解崩溃原因
3. **🔸 Bos**: 理解低分原因
4. **🔸 Room/Maze/Iron/Orz**: frank_wolfe 替代方案
