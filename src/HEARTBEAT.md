# HEARTBEAT.md

## 🚨 工作流偏好 (2026-04-20 更新)

### 核心原则 1：架构复杂度优先
> **永远不为调参浪费时间**
- P0 = 复杂架构实现
- P3 = 参数微调(禁止)

### 核心原则 2：复杂架构不轻易回退
> **改复杂架构时不要轻易回退**
- 失败后分析原因，保留代码
- 继续改，直到成功
- 回退 = 丢失分析机会

### 核心原则 3：切换方向是灾难
> **切换到完全不相关的方向是极其可怕的操作**
- 要学习各种架构，融合成更复杂的架构
- Team_Kitty_Knight = CausalPIBT + PLNS + TimeSpaceA* + GlobalManager
- 永远在当前架构上迭代，不放弃

---

## 当前状态 (2026-04-20 00:30)

### 项目状态
- **代码路径**: `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627`
- **Git commit**: 9eeca63 (H11: LNS RANDOMWALK with 8 neighbors)
- **测试**: round432 完成

### 测试结果 (round432-436)
| 地图 | round432 | round433 | round435 | round436 | 平均 |
|------|----------|----------|----------|----------|------|
| random | 0.523 | 0.497 | 0.510 | 0.517 | 0.512 |
| bos | 0.240 | 0.240 | 0.237 | 0.246 | 0.241 |
| room | 0.198 | 0.182 | 0.180 | 0.182 | 0.186 |
| maze | 0.094 | 0.092 | 0.090 | 0.088 | 0.091 |
| fulfill | 0.774 | 0.808 | 0.811 | 0.728 | 0.780 |
| orz | 0.217 | 0.223 | 0.217 | 0.212 | 0.217 |
| iron | 0.146 | 0.144 | 0.141 | 0.146 | 0.144 |
| **Total** | **2.192** | **2.186** | **2.186** | **2.119** | **2.171** |

---

## 架构分析

### Team_Kitty_Knight (9.039)
```
causal_pibt->multi_step() → LNS全局优化 → actions
```

### 我们当前架构
```
plan() → 单次FW优化 → LNS一次优化 → PIBT单步 → actions
```

**核心差距**：
1. Team_Kitty_Knight 用 `causal_pibt->multi_step()` 生成协调的多步路径
2. 我们的 PIBT 只做单步规划，没有多步协调
3. Team_Kitty_Knight 的 LNS 在每步运行，我们的 LNS 只在初始化时运行一次

---

## ❌ 失败记录

| H# | 假设 | 结果 | 原因 |
|----|------|------|------|
| H1 | priority管理 | ✅ +9% | Team_Kitty_Knight风格 |
| H2 | causalPIBT例外 | ❌ -25% | 移除关键例外 |
| H5 | Frank-Wolfe路径效率 | ❌ | 崩溃 |
| H6 | 每步quick FW | ❌ -12% | 破坏路径稳定性 |
| H7 | collision detection in LNS | ❌ -3% | A*不考虑多agent碰撞 |
| H8 | collision-aware BFS | ❌ | BFS无flow heuristic |
| H9 | collision-aware A* | ❌ -4% | PathTable不一致 |
| H10 | TimeSpaceA* LNS | ❌ 超时 | A* 太慢 |
| H11 | LNS RANDOMWALK 8 neighbors | ❌ 无提升 | LNS params not bottleneck |
| H12 | LNS CONGESTION strategy | ❌ 无提升 | 仍在方差范围内 |

---

## 🔬 H11 分析 (LNS 参数)

**改动**: RANDOM→RANDOMWALK, 4→8 neighbors

**结果**: fulfill=0.774 (同baseline), total=2.192 (vs 2.183 baseline)

**分析**: 
- LNS 参数不是瓶颈
- 架构差距才是核心问题
- 需要实现 multi_step() 协调规划

---

## 🚀 下一步行动

### 方案A: 实现 CausalPIBT.multi_step() (P0) ⭐⭐⭐
**核心问题**: 我们的 PIBT 不生成协调的多步路径

Team_Kitty_Knight 的方法：
1. `causal_pibt->multi_step()` 运行 PIBT 多次，每次更新状态
2. 生成多步协调路径（已避免碰撞）
3. PLNS 优化这些路径

**我们的实现方向**：
1. 在 planner.cpp 中实现 multi_step 版本
2. 让 PIBT 运行多次，每次更新状态
3. 然后用 LNS 优化

### 方案B: 提高 LNS 迭代效率 (P1)
- 更快的路径重规划
- 批量碰撞检测

### 方案C: 专注特定地图优化 (P2)
- room/maze 低分地图专项优化
