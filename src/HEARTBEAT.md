# HEARTBEAT.md

## 🚨 工作流偏好 (2026-04-19 更新)

### 核心原则 1：架构复杂度优先
> **永远不为调参浪费时间**
- P0 = 最复杂架构实现（融合所有冠军代码精华）
- P3 = 参数微调(禁止)

### 核心原则 2：复杂架构不轻易回退
> **改复杂架构时不要轻易回退**
- 失败后分析原因，保留代码
- 继续改，直到成功
- 回退 = 丢失分析机会

### 核心原则 3：切换方向是灾难
> **切换到完全不相关的方向是极其可怕的操作**
- 要融合更多复杂架构，不是切换方向
- Team_Kitty_Knight = CausalPIBT + PLNS + TimeSpaceA* + GlobalManager
- 永远在当前架构上迭代，不放弃

### 核心原则 4：博采众长
> **融合所有冠军代码精华**
- 第一步：Team_Kitty_Knight 架构
- 第二步：其他冠军代码精华

---

## 当前状态 (2026-04-20 06:10) - H19 ALNS

### 项目状态
- **代码路径**: `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627`
- **Git commit**: bfcd783 (H19 ALNS enabled, H20 disabled)
- **MD5**: 3837850 bytes
- **Strategy**: ALNS (RANDOM + RANDOMWALK)

### 架构融合进度

| 组件 | Team_Kitty_Knight | 我们 | 状态 |
|------|------------------|------|------|
| CausalPIBT | ✅ multi_step() | ❌ 无 | **待实现** |
| PLNS | ✅ 完整版 | ⚠️ 简化版 | 需增强 |
| TimeSpaceA* | ✅ 集成 | ✅ 已实现 | 完成 |
| GlobalManager | ✅ 完整版 | ⚠️ 简化版 | 需增强 |

---

## 最新测试结果

| Round | random | bos | room | maze | fulfill | orz | iron | Total |
|-------|--------|-----|------|------|---------|-----|------|-------|
| 470 | 0.503 | 0.244 | 0.195 | 0.092 | 0.842 | 0.215 | 0.140 | 2.231 |
| 471 | 0.507 | 0.239 | 0.182 | 0.100 | 0.764 | 0.212 | 0.147 | 2.151 |
| 475 | 0.507 | 0.234 | 0.185 | 0.094 | 0.768 | 0.223 | 0.141 | 2.152 |

---

## H19: ALNS with multiple strategies
- **Status**: ✅ Enabled
- **Result**: Neutral within variance (~2.15-2.23)
- **Approach**: ALNS selects between RANDOM and RANDOMWALK strategies based on weights

## H20: Coordinated multi-step paths (DISABLED)
- **Status**: ❌ DISABLED - caused crashes
- **Result**: Tests failed to produce output
- **Root cause**: State management issue - trajLNS.trajs became inconsistent
- **Analysis**: The approach is correct (use multi-step PIBT for coordination) but implementation corrupted state

---

## ❌ 失败记录 (H1-H20)

| H# | 假设 | 结果 | 原因 |
|----|------|------|------|
| H1 | priority管理 | ✅ +9% | Team_Kitty_Knight风格 |
| H2 | causalPIBT例外 | ❌ -25% | 移除关键例外 |
| H5 | Frank-Wolfe路径效率 | ❌ | 崩溃 |
| H6 | 每步quick FW | ❌ -12% | 破坏路径稳定性 |
| H7 | LNS碰撞检测 | ❌ -3% | LNS acceptance太严格 |
| H8 | BFS碰撞避免 | ❌ | BFS无flow heuristic |
| H9 | collision-aware A* | ❌ -4% | PathTable不一致 |
| H10 | TimeSpaceA* LNS | ❌ 超时 | 缺少 CausalPIBT 协调 |
| H11-H17 | LNS参数调优 | ❌ 无提升 | 确认LNS参数不是瓶颈 |
| **H18** | **run_multi_step_pibt** | **❌ 中性** | **无协调效益** |
| **H19** | **ALNS多策略** | **~中性** | **仍在方差范围** |
| **H20** | **多步协调路径** | **❌ 崩溃** | **状态管理问题** |

---

## 🚀 下一步行动

**H20 修复方向**:
1. 问题根源：trajLNS状态与env->start_states不一致
2. 修复方法：在run_multi_step_pibt后正确同步trajLNS状态
3. 或者：完全克隆causalPIBT状态，包括trajLNS

**或者切换方向**:
- 实现完整的CausalPIBT类，而不是修改现有run_multi_step_pibt
- 参考Team_Kitty_Knight的CausalPIBT实现

等待用户发送定时任务。
