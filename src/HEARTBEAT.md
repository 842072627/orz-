# HEARTBEAT.md

## 当前状态 (2026-04-19 20:20) - 🔄 永远活跃

### 项目状态
- **代码路径**: `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627`
- **Git commit**: 628e8cd (TimeSpaceA* preparation)
- **测试**: round414 完成

---

## 📊 测试结果

### Round 412-414 vs 碰撞感知 (H8)
| Map | round412 | round413 | round414 | H8失败 |
|-----|----------|----------|----------|--------|
| random | 0.517 | 0.530 | 0.507 | 不稳定 |
| bos | 0.247 | 0.239 | 0.246 | 无提升 |
| room | 0.190 | 0.180 | 0.182 | 无提升 |
| maze | 0.102 | 0.100 | 0.090 | 无提升 |
| fulfill | 0.765 | 0.789 | 0.708 | 高方差 |
| orz | 0.228 | 0.210 | 0.217 | 无提升 |
| iron | 0.141 | 0.146 | 0.141 | 无提升 |
| **Total** | **2.190** | **2.194** | **2.091** | **回退** |

**结论**: 碰撞感知路径搜索 (H8) 无稳定提升，回退到 baseline

---

## ❌ 失败记录

| H# | 假设 | 结果 | 原因 |
|----|------|------|------|
| H1 | causalPIBT例外 | fulfill -25% | 移除关键例外 |
| H2 | PathTable碰撞检测 | fulfill -11% | LNS acceptance太严格 |
| H5 | Frank-Wolfe路径效率 | iron=0.000 | 崩溃 |
| H6 | 每步quick FW | bos -14%, fulfill -12% | 破坏路径稳定性 |
| H7 | run_complete_lns碰撞检测 | total -3% | A*不考虑多agent碰撞 |
| **H8** | **find_collision_free_path** | **total ~0** | **BFS不如A*，无稳定提升** |

### H8 碰撞感知路径搜索失败详情
实现了 `find_collision_free_path` 函数：
- 使用 BFS 在时间-空间搜索
- 检查 vertex/edge collision
- **问题**：不使用 flow heuristic，只用 manhattan distance
- **结果**：找到的路径可能更长，或 timeout
- **结论**：需要在 A* 框架内集成碰撞检查，不是单独做 BFS

---

## 🔬 下一步方向

### 方案A: TimeSpaceA* 集成 (P0) ⭐⭐⭐
**核心问题**：普通 A* 只考虑单个 agent，不考虑时间维度和多 agent 碰撞

Team_Kitty_Knight 的方法：
```
CausalPIBT.multi_step() → 初始路径（通过 PIBT 协调）
    ↓
PLNS.solve() + TimeSpaceA* → 碰撞感知优化
    ↓
无碰撞路径
```

**我们的差距**：
- Team_Kitty_Knight 用 TimeSpaceA* 在 LNS 重新优化时生成碰撞感知路径
- 我们用普通 A* 覆盖 PIBT 协调的路径
- 需要 TimeSpaceA* 来弥补这个差距

**需要的实现**：
1. TimeSpaceA* 在搜索时考虑其他 agent 的位置和时间
2. 生成碰撞感知的单 agent 路径
3. 在 LNS 重新优化时使用，而不是普通 A*

### 方案B: 调整 LNS 参数 (P1)
- neighbor_size
- 迭代次数
- ALNS 参数

### 方案C: 专注高权重地图 (P2)
- fulfill (0.765) 贡献最大
- bos (0.247) 第二
- 针对性优化这两个地图

---

## ✅ 当前最佳版本

- **Commit**: 628e8cd
- **方法**: H3-v4 + start_orientations + OpenMP
- **Total**: ~2.19 (稳定)
- **fulfill**: ~0.76 (稳定)

---

## 🏆 对比 Team_Kitty_Knight (9.039)

| 系统 | LNS每步? | 碰撞感知? | 方法 |
|------|---------|---------|------|
| Team_Kitty_Knight | ✅ | ✅ TimeSpaceA* | CausalPIBT + PLNS + TimeSpaceA* |
| 我们 (当前) | ❌ | ❌ | PIBT + LNS一次 + 普通A* |

**差距原因**：
- Team_Kitty_Knight 用 TimeSpaceA* 在 LNS 重新优化时生成碰撞感知路径
- 我们用普通 A* 覆盖 PIBT 协调的路径
- 需要 TimeSpaceA* 来弥补这个差距

**架构差距 (9.039 vs 2.19 = 4x gap)**:
1. Team_Kitty_Knight 的 CausalPIBT 做多步协调执行
2. PLNS 做全局优化，调用 TimeSpaceA* 生成碰撞感知路径
3. 我们只有一次 LNS，用普通 A*，没有真正的碰撞感知

---

## 📝 H8 实验教训

1. **碰撞检测在 LNS 层不工作** (H7)：检查已有路径的碰撞会导致过度拒绝
2. **碰撞感知 BFS 也不工作** (H8)：没有 flow heuristic，路径质量差
3. **真正需要的是 TimeSpaceA***：在 A* 搜索时集成碰撞约束

### 为什么碰撞感知在 A* 框架外不工作
- A* 的 flow heuristic 找到的是考虑整体 flow 的最短路径
- 单独的碰撞感知 BFS 找到的是不考虑 flow 的最短无碰撞路径
- 两者可能都很长或不理想
- **关键**：需要在 A* 框架内集成碰撞检查，同时保留 flow heuristic
