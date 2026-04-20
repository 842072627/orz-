# MAPF 实验记录

唯一真实来源。

---

## Start-Kit 纯净基准 (v3.1)

| 字段 | 值 |
|------|-----|
| **commit** | `483ca3d` |
| **Source MD5** | `63C6DDEFBB3F89F33769F82248AF6251` |
| **版本** | v3.1 Start-Kit |
| **测试日期** | 2026-04-20 |
| **测试次数** | 5次平均 |

### 5次测试结果

| Round | random | fulfill | total |
|-------|--------|---------|-------|
| 1 | 0.477 | 0.411 | 0.888 |
| 2 | 0.507 | 0.383 | 0.890 |
| 3 | 0.517 | 0.427 | 0.944 |
| 4 | 0.517 | 0.449 | 0.966 |
| 5 | 0.500 | 0.445 | 0.945 |

**平均值**: random=0.504, fulfill=0.423, total=0.927

**方差说明**: fulfill 方差极大 [0.383-0.449]，random 相对稳定 [0.477-0.517]

### 基线特征
- 原始 Start-Kit v3.1 代码
- planner.cpp: priority +1 increment（存在于第 157、203 行）
- 无任何优化修改

### 方差测试（Round 11-25，variance 脚本，共15次）

| Round | random | fulfill | total |
|-------|--------|---------|-------|
| 11 | 0.490 | 0.427 | 0.917 |
| 12 | 0.510 | 0.441 | 0.951 |
| 13 | 0.483 | 0.491 | 0.974 |
| 14 | 0.470 | 0.457 | 0.927 |
| 15 | 0.513 | 0.434 | 0.947 |
| 16 | 0.520 | 0.482 | 1.002 |
| 17 | 0.503 | 0.473 | 0.976 |
| 18 | 0.500 | 0.429 | 0.929 |
| 19 | 0.487 | 0.456 | 0.943 |
| 20 | 0.500 | 0.460 | 0.960 |
| 21 | 0.483 | 0.485 | 0.968 |
| 22 | 0.510 | 0.456 | 0.966 |
| 23 | 0.490 | 0.458 | 0.948 |
| 24 | 0.490 | 0.425 | 0.915 |
| 25 | 0.513 | 0.444 | 0.957 |

**方差测试平均（15次）**: random=0.497, fulfill=0.455, total=0.952
**fulfill 范围**: [0.425-0.491]，**最高未破 0.5**

### 综合基线（15次+5次=20次）
**random≈0.498, fulfill≈0.446, total≈0.944**

---

## 完整优化：4项融合架构

### 优化组件

| # | 组件 | 文件 | 效果 |
|---|------|------|------|
| 1 | ARCH-B R5 spatial scheduler | TaskScheduler.cpp | PICKUP=7.0, DEST=1.0, DECAY=0.95 |
| 2 | 无 +1 priority increment | planner.cpp | 防止优先级膨胀 |
| 3 | 无 higher_id exception | pibt.cpp | 移除因果PIBT中不必要的限制 |
| 4 | Dist2Path for warehouse | utils.cpp (is_large_map 90000→50000) | 启用路径距离启发式表 |

### ⚠️ MD5 Bug 修正
- 旧脚本 MD5 只算根目录文件，`my_planner/*.cpp` 从未被计入
- MD5 缓存全部失效（误报命中）
- 脚本已修复：`os.walk('.')` 递归包含所有子目录

### 测试结果（Dist2Path 关闭，MD5=BE7A67C4C146504F7729D41B9C032D28）

| Round | random | fulfill | total |
|-------|--------|---------|-------|
| 36 | 0.523 | 0.713 | 1.236 |

**组件 1+2+3 贡献**: random≈0.523, fulfill≈0.713, total≈**1.236**

### 测试结果（Dist2Path 开启，MD5=0E7B88593DAF0994550A76BAF5EDC236）

| Round | random | fulfill | total |
|-------|--------|---------|-------|
| 34 | 0.490 | 1.085 | 1.575 |
| 35 | 0.527 | 1.100 | 1.627 |
| 37 | 0.513 | 1.105 | 1.618 |

**全部4项融合**: random≈0.510, fulfill≈**1.097**, total≈**1.607**

### 贡献分解

| 阶段 | fulfill 提升 | 说明 |
|------|-------------|------|
| v3.1 基线 | 0.446 | 原始 Start-Kit |
| + 组件1+2+3 | 0.713 | scheduler + priority + higher_id |
| + 组件4 (Dist2Path) | **1.097** | **最大贡献** |

**最终**: random≈0.510, fulfill≈1.097, total≈**1.607**
**vs v3.1 基线 0.944 → +70.2%**

---

## 实验记录

### frank_wolfe deadline 移除 (commit b397bfb 之后)

| Round | random | fulfill | total |
|-------|--------|---------|-------|
| 84 | 0.513 | 1.119 | 1.632 |
| 85 | 0.503 | 1.109 | 1.612 |
| 86 | 0.520 | 1.102 | 1.622 |
| 87 | 0.533 | 1.085 | 1.618 |
| 88 | 0.477 | 1.114 | 1.591 |

**5次平均 (MD5=53E306DCFACF955EDED017FE88D669FC)**: random≈0.509, fulfill≈1.106, total≈1.615
**历史平均 (R71-83)**: random≈0.497, fulfill≈1.097, total≈1.594
**提升**: +2.1% total

**改动**: flow.cpp frank_wolfe 中 update_traj 不再传 deadline 参数 (nullptr)，让 A* 完整运行
**分析**: Team_Kitty_Knight 的 update_traj 不带 deadline，完整 A* 搜索产生更好的 guide paths，轻微改善

### pibt.cpp get_gp_h 统一 (fallback to heuristics for large maps)

| Round | random | fulfill | total |
|-------|--------|---------|-------|
| 95 | 0.497 | 1.149 | 1.646 |
| 96 | 0.510 | 1.126 | 1.636 |
| 97 | 0.463 | 1.072 | 1.535 |

**3次平均 (MD5=BA9586DA4E7DCD3B971D7FACD4692F00)**: random≈0.490, fulfill≈1.116, total≈1.606
**对比 frank_wolfe deadline 移除后 (R84-88)**: total≈1.615
**结论**: 无显著差异，结果在方差范围内

**改动**: 移除了 is_large_map 判断，统一 small/large map 的 get_gp_h 逻辑，都优先检查 traj_dists，然后 heuristics，然后 manhattan
**分析**: Team_Kitty_Knight 的 get_gp_h 不区分 is_large_map，统一使用相同逻辑

---

## 实验记录

