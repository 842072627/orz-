# MAPF 实验记录

> **唯一真实来源**。每一轮实验（成功、失败、崩溃）都必须记录在这里。
> git commit 时同步更新。

---

## 全局状态面板

**当前 ARCH**：ARCH-N（R4完成，灾难性失败）
**当前轮次**：R4（round156完成，Entry revert）
**全局最佳**：ARCH-E R5 = **2.720**
**基准对比**：云端最高 9.44，本地最佳 2.720，差距 -6.72

**当前 ARCH 状态**：
- ARCH-L 方差测试完成（R1-R53），最佳 R24 = 2.719
- ARCH-M 失败，已回退
- ARCH-N R1=2.639, R2=2.590, R3=2.550, R4=2.463（revert）
- **Executor/Entry改动均灾难性失败，Scheduler改动均无效**
- 架构已充分探索，需要全新方向
- 下一步：深入研究 KITTY_KNIGHT WPPLPlanner（可能是规划器差异导致分数差距）

---

## 架构实验追踪

### ARCH-A — min-Task-ID 确定性策略
**最佳**：R6 = 2.489 | **状态**：✅ 饱和

| 轮次 | 配置 | 总分 | 备注 |
|------|------|------|------|
| R6 | 贪心+min-Task-ID确定性tie-break | 2.489 | ARCH-A最佳 |

---

### ARCH-B — 空间负载均衡 v1
**最佳**：R5 = 2.696 | **状态**：✅ 饱和

| 轮次 | 配置 | 总分 | fulfill | 备注 |
|------|------|------|---------|------|
| R5 | pickup=7.0, dest=1.0 | **2.696** | 0.830 | 最佳配置 |
| R6 | pickup=7.5, dest=0.5 | 2.674 | 0.815 | 略差 |
| R7 | pickup=7.2, dest=0.9 | 2.640 | 0.816 | interpolation错误 |
| R8 | DECAY=0.90 | 2.695 | 0.829 | 同R5 |
| R9 | REGION_GRID_SIZE=12 | 2.633 | 0.738 | fulfill下降 |
| R10 | R5 restored | 2.651 | 0.786 | 恢复R5基准 |

---

### ARCH-C — Auction Scheduler
**最佳**：0.0 | **状态**：✅ 饱和（灾难性失败）

| 轮次 | 配置 | 总分 | 备注 |
|------|------|------|------|
| R1 | Auction + agent-level congestion | **0.0** | only free_tasks considered，proposed_schedule未初始化 |

---

### ARCH-D — 参数空间探索
**最佳**：R2 = 2.625 | **状态**：✅ 饱和

| 轮次 | 配置 | 总分 | 备注 |
|------|------|------|------|
| R1 | pickup=8.5, dest=1.0, decay=0.88 | 2.469 | fulfill崩到0.592 |
| R2 | REGION_GRID_SIZE=20 | 2.625 | 粗粒度更差 |
| R3 | pickup=6.5, dest=2.5, decay=0.97 | 2.544 | dest=2.5过高 |
| R4 | 地图自适应参数 | 2.588 | 全局统一参数更好 |
| R5 | Swap-based redistribution | **0.0** | Bug导致崩溃 |

---

### ARCH-E — 空间负载均衡 v2
**最佳**：R5 = **2.720** | **状态**：✅ 饱和（全局最佳参考）

| 轮次 | 配置 | 总分 | fulfill | 备注 |
|------|------|------|---------|------|
| R1 | Multiplicative congestion | 2.542 | 0.666 | 短路径策略失败 |
| R2 | dest=0.8 | 2.682 | - | 接近R5 |
| R3 | pickup=7.5, dest=0.8 | 2.704 | 0.825 | 新纪录 |
| R4 | pickup=8.0, dest=0.8 | 2.653 | 0.789 | 回退 |
| R5 | pickup=7.3, dest=0.8 | **2.720** | 0.848 | **全局最佳** |
| R6 | pickup=7.2, dest=0.8 | 2.647 | 0.765 | 确认7.3为峰值 |
| R7 | pickup=7.4, dest=0.8 | 2.640 | 0.800 | 确认7.3为峰值 |
| R8 | dest=0.6 | 2.564 | 0.699 | dest太低 |
| R8b | dest=0.9 | 2.634 | 0.797 | dest略高 |

**确认参数**：`PICKUP=7.3, DEST=0.8, LEN=0.5, REGION=16, DECAY=0.95`

---

### ARCH-F — 非线性/细粒度惩罚
**最佳**：R3 = 2.695 | **状态**：✅ 饱和

| 轮次 | 配置 | 总分 | 备注 |
|------|------|------|------|
| R1 | Multi-tier non-linear penalty | **2.154** | fulfill崩到0.352 |
| R2 | REGION=24 | 2.657 | 不如16 |
| R3 | REGION=12 | 2.695 | 略差于16 |
| R5 | Distance-normalized log penalty | 2.435 | fulfill崩到0.575 |

---

### ARCH-G — 各种参数变体
**最佳**：R2 = 2.617 | **状态**：✅ 饱和

| 轮次 | 配置 | 总分 | 备注 |
|------|------|------|------|
| R1 | LNS-style persistent tracking | **0.0** | Crash |
| R2 | Map-adaptive by agent count | 2.617 | 低密度拖后腿 |
| R3 | REGION=8 | 2.607 | 过细粒度 |
| R5 | DECAY=0.99 | 2.634 | 记忆太长 |
| R6 | DEST_PENALTY=1.5 | 2.644 | dest惩罚过高 |

---

### ARCH-H — Nearest-K 策略
**最佳**：R2 = 2.442 | **状态**：✅ 饱和（灾难性失败）

| 轮次 | 配置 | 总分 | fulfill | 备注 |
|------|------|------|---------|------|
| R1 | K=5 nearest + congestion | 2.333 | 0.589 | 灾难性 |
| R2 | Light penalty (pickup=2.0) | 2.442 | 0.595 | 轻度惩罚失败 |

---

### ARCH-I — 规划器 + 混合探索
**最佳**：R13 = 2.678 | **状态**：✅ 饱和

| 轮次 | 配置 | 总分 | 备注 |
|------|------|------|------|
| R1 | TPG冲突后处理 | 2.628 | 规划器修改反效果 |
| R2 | Reservation时序惩罚 | 2.652 | 略微改善 |
| R3 | REGION=32 | 2.520 | 过细 |
| R4 | 规划窗口1.5x | 2.493 | 更长窗口降低fulfill |
| R6 | PICKUP=10.0 | 2.540 | 过犹不及 |
| R7 | PICKUP=6.0 | 2.569 | random好但fulfill差 |
| R9 | DECAY=0.90 | 2.634 | 慢速衰减略差 |
| R10 | DECAY=0.97 | 2.593 | 快速衰减也差 |
| R12 | 时间分配 1:3 | **1.921** | 灾难性失败 |
| R13 | LENGTH_WEIGHT=0.5 | **2.678** | 最佳ARCH-I |
| R14 | LENGTH=0.3 | 2.516 | 过轻 |
| R15 | LENGTH=0.7 | 2.652 | fulfill最高但其他差 |

---

### ARCH-J — Executor/调度器混合
**最佳**：R5 = 2.629 | **状态**：✅ 饱和

| 轮次 | 配置 | 结果 | 备注 |
|------|------|------|------|
| R1 | Priority-TPG Executor | **CRASHED** | 无输出崩溃 |
| R2 | Adaptive pickup by ratio | 2.591 | 无显著改善 |
| R3 | Map-specific tuning | 2.608 | 无改善 |
| R4 | LNS destroy-rebuild | **CRASHED** | Bug |
| R6 | 60:40 时间分配 | **1.909** | fulfill=0.184灾难性 |

---

### ARCH-K — Completion-time-aware
**最佳**：R1 = 2.634 | **状态**：✅ 饱和

| 轮次 | 配置 | 总分 | 备注 |
|------|------|------|------|
| R1 | completion_time aware | 2.634 | 无显著变化 |
| R2 | DEST_BOOST=1.2 | 2.590 | 无提升 |
| R3 | DEST=1.0, LEN=0.5 | 2.623 | 无提升 |
| R4 | PICKUP=7.0, DEST=1.0 | 2.582 | 无提升 |

---

### ARCH-L — Task-Age / 方差测试
**最佳**：R24 = 2.719 | **状态**：✅ 饱和

| 轮次 | 配置 | 总分 | fulfill | 备注 |
|------|------|------|---------|------|
| R1 | future-position-aware | 2.634 | 0.800 | - |
| R2 | future-position 0.5 | 2.611 | 0.764 | - |
| R3 | Executor Goal-Proximity | **0.0** | 0 | CRASHED |
| R4 | Task-Age 0.05/2.0 | 2.592 | 0.725 | - |
| R5 | Task-Age 0.1/3.0 | 2.612 | 0.763 | - |
| R6 | Task-Age 0.15/4.0 | 2.626 | 0.782 | - |
| R7 | PICKUP=8.0 | 2.523 | 0.678 | - |
| R10 | ARCH-E参数+age_bonus | 2.655 | 0.778 | - |
| R16 | ARCH-E参数(无age) | 2.683 | 0.839 | - |
| R21 | 方差测试 | 2.698 | 0.832 | - |
| R24 | 方差测试 | **2.719** | 0.847 | **最佳L** |
| R35 | 方差测试 | 2.715 | 0.862 | **最佳fulfill** |
| R51 | 方差测试 | 2.465 | 0.621 | 异常低分 |
| R52 | 方差测试 | 2.571 | 0.656 | 异常低分 |
| **R53** | **ARCH-E参数基线验证（round152）** | **2.664** | 0.803 | 方差正常范围内 |

**方差范围**：2.465 - 2.719（±5%）

---

### ARCH-M — Task Clustering / Agent Affinity
**最佳**：R4 = 2.551 | **状态**：✅ 失败（已回退）

| 轮次 | 配置 | 总分 | fulfill | 备注 |
|------|------|------|---------|------|
| R1 | Task Clustering + Agent Affinity | 2.210 | 0.578 | 灾难性 |
| R2 | Adjusted congestion (6.0->4.0) | 2.329 | 0.669 | 仍然差 |
| R3 | Agent density congestion | **1.099** | 0 | **SIGKILL** |
| R4 | DEST_PENALTY=1.0 | 2.551 | 0.688 | 差于DEST=0.8 |

---

### ARCH-N — Free Agent Proximity Penalty → 放弃
**最佳**：R1 = 2.639 | **状态**：✅ 放弃

| 轮次 | 配置 | 总分 | fulfill | 备注 |
|------|------|------|---------|------|
| R1 | Free Agent Proximity Penalty (weight=0.5) | **2.639** | 0.784 | round153 |
| R2 | Free Agent Proximity Penalty (weight=0.1) | 2.590 | 0.763 | round154，回归 |
| R3 | Baseline restore (ARCH-L code) | 2.550 | 0.691 | round155，方差正常 |
| R4 | Idle Agent Relocation (Entry.cpp) | **2.463** | 0.613 | round156，**灾难性失败，已revert** |

**R1-R4 分析**：
- R1 (weight=0.5): Total=2.639（-0.025 vs baseline）
- R2 (weight=0.1): Total=2.590（-0.049 vs baseline）
- R3 (baseline): Total=2.550（方差内）
- R4 (Idle Agent): Total=2.463（fulfill=0.613，灾难性，**已revert**）
- **结论：Free Agent Penalty 和 Idle Agent 方向均失败**

**ARCH-N 总结**：
- R1-R4 均无显著提升，方向失败
- 发现 ARCH-L baseline 本身有 ±5% 方差（2.465-2.719）
- Executor/Entry 改动均灾难性失败
- 需要新方向才能突破
- random: 0.694 (+0.003 vs baseline)
- bos: 0.338 (+0.008)
- room: 0.162 (-0.008)
- maze: 0.099 (-0.003)
- fulfill: 0.784 (-0.019)
- orz: 0.321 (-0.009)
- iron: 0.241 (+0.003)
- **Total: 2.639（-0.025 vs baseline 2.664）**

**R2 结果**：Total = 2.590（-0.049 vs baseline，方向确认失败）

**结论**：Free Agent Proximity Penalty 在 weight=0.5 和 0.1 均失败，weight=0.1 更差。方向放弃。

---

## 待实验队列

> **格式**：`方向 | 优先级 | 状态 | 所属ARCH | 备注`

| 方向 | 优先级 | 状态 | 所属ARCH | 备注 |
|------|--------|------|---------|------|
| **No Man Sky 冠军 Scheduler 分析** | P0 | ✅ 研究完成 | ARCH-N | 发现：MyScheduler多阶段，Kitty Knight用WPPLPlanner |
| Executor Wait-hint 优化 | P1 | ⏳ 待开始 | - | 低风险方向 |
| 多目标调度（throughput vs makespan） | P1 | ⏳ 待开始 | - | 需要先研究 |
| 延迟建模（delay-aware scheduling） | P2 | ⏳ 待开始 | - | 需要先研究 |

---

## 放弃方向登记

> **所有被放弃的方向必须记录在这里**。没有记录的放弃视为无效。

| 方向 | 所属ARCH | 放弃轮次 | 放弃原因 | 是否探索替代方案 | 备注 |
|------|---------|---------|---------|----------------|------|
| Task Clustering | ARCH-M | R1 | fulfill崩到0.578 | 是，调整参数后仍差 | 聚类逻辑与任务分配冲突 |
| LNS destroy-rebuild | ARCH-J | R4 | Crash | N | 状态管理复杂 |
| Nearest-K | ARCH-H | R2 | fulfill<0.6灾难性 | N | 距离优先在拥塞环境失败 |
| Multi-tier non-linear | ARCH-F | R1 | fulfill崩到0.352 | N | 非线性惩罚失衡 |
| Free Agent Proximity Penalty | ARCH-N | R1-R2 | fulfill下降，weight 0.5和0.1均失败 | 是，Executor Wait-hint、低风险 | 区域agents计数与实际冲突不相关 |
| Idle Agent Relocation | ARCH-N | R4 | fulfill=0.613灾难性，revert | N | 改变goal_location导致planner决策混乱 |
| Hungarian Assignment | ARCH-N | R5 | 卡死（超时） | N | O(n²) lookup 实现 bug |
| Randomized Agent Order | ARCH-N | R5b | 卡死（超时） | N | unordered_set erase 逻辑问题 |
| 时间分配 1:3 | ARCH-I | R12 | fulfill=0.184灾难性 | N | 调度器时间不足 |

---

## 云端基准对比（持续追踪）

| Map | ARCH-E (8.02) | Virtual Best (9.44) | 差距 | 优先级 |
|-----|---------------|---------------------|------|--------|
| bos | 0.840 | 1.000 | +0.160 | 中 |
| fulfill-A | 0.862 | 0.846 | -0.016 | 低 |
| fulfill-B | 0.781 | 0.640 | -0.141 | 低 |
| fulfill-C | 0.737 | 0.930 | +0.193 | 中 |
| iron | 0.698 | 0.760 | +0.062 | 低 |
| maze-A | 0.570 | 0.703 | +0.133 | **高** |
| maze-B | 0.746 | 0.899 | +0.153 | **高** |
| orz | 0.736 | 0.919 | +0.183 | **高** |
| rand-A | 0.865 | 0.911 | +0.046 | 低 |
| room-A | 0.472 | 0.904 | +0.432 | **最高** |
| room-B | 0.713 | 0.927 | +0.214 | 中 |
| **Total** | **8.02** | **9.44** | **+1.42** | |

**突破重点**：room-A（+0.432）、orz（+0.183）、maze-B（+0.153）

---

## 项目研究笔记

> 详细研究过程、诊断、灵感记录在：`项目研究笔记.md`

**失败实验备份**：所有被放弃的实验代码必须备份到 `../archive/ARCH-X_Ry/方向/`
- 备份内容：修改的文件快照 + 实验结果 + 失败原因记录
- 目的：用于后续混合架构研究，避免重复劳动
