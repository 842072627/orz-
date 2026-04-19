# MAPF 实验记录

> **唯一真实来源。每一轮实验（成功、失败、崩溃）都必须记录在这里。**
> **重要**：每个地图分数同等重要，没有"总分"概念。

---

## 测试步骤标准

| 地图 | 步数 |
|------|------|
| random | 300 |
| bos | 700 |
| room | 400 |
| maze | 500 |
| fulfill | 1000 |
| orz | 400 |
| iron | 700 |

---

## 核心基准

### Start-Kit 纯净基准 (commit f82e580)

| 地图 | 步数 | 分数 | tasks |
|------|------|------|-------|
| random | 300 | 0.523 | 157 |
| bos | 700 | 0.166 | 116 |
| room | 400 | 0.168 | 67 |
| maze | 500 | 0.094 | 47 |
| fulfill | 1000 | 0.507 | 507 |
| orz | 400 | 0.210 | 84 |
| iron | 700 | 0.153 | 107 |

---

## ARCH-AARCH 实验

### H1: +1 priority 移除

**来源**：Team_Kitty_Knight 的 causal_pibt.cpp
**机制**：移除 `refresh_multistep_step_state` 中的 `+1` priority increment，减少优先级膨胀

**验证结果 (Round 355-361)**：
| 地图 | Baseline | 平均 | 变化 |
|------|----------|------|------|
| fulfill | 0.546 | 0.733 | **+34%** |
| bos | 0.179 | 0.241 | **+35%** |
| room | 0.168 | 0.186 | +11% |
| random | 0.510 | 0.516 | +1% |
| orz | 0.215 | 0.217 | +1% |
| maze | 0.094 | 0.091 | -3% |
| iron | 0.150 | 0.147 | -2% |

**结论**：✅ 验证成功

---

### H2: causalPIBT 例外移除

**结果**：❌ fulfill -25%

---

### H3: LNS 集成

**状态**：⚠️ skeleton 创建，未集成

---

### H4: Dist2PathHeuristicTable wrapper

**状态**：⚠️ wrapper 创建，未集成

---

### H5: Frank-Wolfe 路径效率

**结果**：❌ iron=0.000 崩溃

---

### H6: 每步 quick FW

**结果**：❌ bos -14%, fulfill -12%

---

## 假设追踪

| H# | 假设 | 结果 | 状态 |
|----|------|------|------|
| H1 | +1 priority移除 | fulfill+34%, bos+35% | ✅ 成功 |
| H2 | causalPIBT例外 | fulfill-25% | ❌ 失败 |
| H3 | LNS集成 | skeleton未集成 | ⚠️ 未完成 |
| H4 | Dist2PathHeuristicTable | wrapper未集成 | ⚠️ 未完成 |
| H5 | Frank-Wolfe路径效率 | iron=0.000 | ❌ 崩溃 |
| H6 | 每步quick FW | bos-14%, fulfill-12% | ❌ 失败 |

---

## 架构差异分析

### Team_Kitty_Knight (9.039)
```
causal_pibt->multi_step() → LNS全局优化 → actions
```

### 我们当前架构 (H1 baseline)
```
plan() → 单次FW优化 → actions
```

**核心差距**：LNS在每步做全局优化，我们只在step 0做一次

---

## 待完成工作

- [ ] H3: 集成 LNS skeleton
- [ ] H4: 集成 Dist2PathHeuristicTable wrapper
