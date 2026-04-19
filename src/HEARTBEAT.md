# HEARTBEAT.md

## 当前状态 (2026-04-19 19:45) - 🔄 永远活跃

### 项目状态
- **代码路径**: `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627`
- **Git commit**: 628e8cd (TimeSpaceA* preparation)
- **测试**: round412 完成

---

## 📊 测试结果

### Round 412 (baseline确认)
| Map | Score | Tasks | Makespan |
|-----|-------|-------|----------|
| random | 0.517 | 155 | 300 |
| bos | 0.247 | 173 | 700 |
| room | 0.190 | 76 | 400 |
| maze | 0.102 | 51 | 500 |
| fulfill | 0.765 | 765 | 1000 |
| orz | 0.228 | 91 | 400 |
| iron | 0.141 | 99 | 700 |

**Total: 2.190** (H3-v4 baseline: 2.129, +3%)

### 历史高分记录
- **round409**: fulfill=0.807, total=2.225 (可能幸运值)
- **round408**: fulfill=0.796, total=2.171
- **round407**: fulfill=0.807, total=2.225

---

## ❌ 失败记录

| H# | 假设 | 结果 | 原因 |
|----|------|------|------|
| H1 | causalPIBT例外 | fulfill -25% | 移除关键例外 |
| H2 | PathTable碰撞检测 | fulfill -11% | LNS acceptance太严格 |
| H5 | Frank-Wolfe路径效率 | iron=0.000 | 崩溃 |
| H6 | 每步quick FW | bos -14%, fulfill -12% | 破坏路径稳定性 |
| **H7** | **run_complete_lns碰撞检测** | **fulfill -3%, total -3%** | **A*不考虑多agent碰撞，过度拒绝** |

### H7 碰撞检测失败详情
- 在 `run_complete_lns` 中检测 neighbor 内路径碰撞
- Vertex collision + Edge collision 检查
- **问题**：普通 A* 不知道其他 agent 位置，计算的"最短路径"可能和其他 agent 碰撞
- **结果**：拒绝正确的优化，只因为 neighbor 内两个新路径冲突
- **结论**：需要在路径搜索时就不产生碰撞，而不是事后检查

---

## 🔬 下一步方向

### 方案A: TimeSpaceA* 集成 (P0) ⭐
**核心问题**：普通 A* 只考虑单个 agent，不考虑时间维度和多 agent 碰撞

Team_Kitty_Knight 的方法：
```
CausalPIBT.multi_step() → 初始路径（通过 PIBT 协调）
    ↓
PLNS.solve() + TimeSpaceA* → 碰撞感知优化
    ↓
无碰撞路径
```

**我们的差距**：我们用普通 A* 重规划，会覆盖 PIBT 已验证的无碰撞路径

**需要的实现**：
1. TimeSpaceA* 在搜索时考虑其他 agent 的位置和时间
2. 生成碰撞感知的单 agent 路径
3. 在 LNS 重新优化时使用，而不是普通 A*

### 方案B: 简化碰撞解决 (P1)
在 PIBT 执行层解决碰撞，不在 LNS 层预防
- 当前 PIBT 已经有碰撞处理
- 可能已经足够

### 方案C: 调整 LNS 参数 (P2)
- neighbor_size
- 迭代次数
- ALNS 参数

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
