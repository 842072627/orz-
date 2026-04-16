# HEARTBEAT.md

## 当前状态 (2026-04-16 22:30)

### 项目
- **路径**: `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627`
- **当前分支**: ARCH-L
- **当前代码**: PICKUP=7.3, DEST=0.8, LENGTH=0.5, REGION=16, DECAY=0.95

### 全局最佳
**ARCH-E R5: pickup=7.3, dest=0.8, Total=2.720**

### ARCH-L 实验总结 (R1-R21)

| Round | 方法 | Total | 结论 |
|-------|------|-------|------|
| R1 | Next-hop bonus=0.3 | 2.634 | 无显著变化 |
| R2 | Next-hop bonus=0.5 | 2.611 | 无提升 |
| R3 | Executor Goal-Proximity | 0.0 | CRASHED |
| R4 | Age bonus=0.05 | 2.592 | 无提升 |
| R5 | Age bonus=0.1 | 2.612 | 无提升 |
| R6 | Age bonus=0.15 | 2.626 | 无提升 |
| R7 | PICKUP=8.0 | 2.523 | 变差 |
| R8 | PICKUP=7.0, DEST=0.9 | 2.638 | 无提升 |
| R9 | PICKUP=7.2, DECAY=0.98 | 2.603 | 变差 |
| R10 | AGE=0.15 + ARCH-E params | 2.655 | 基准 |
| R11 | LENGTH=0.4 | 2.653 | 无提升 |
| R12 | LENGTH=0.45 | 2.638 | 无提升 |
| R13 | PICKUP=7.5 | 2.619 | 变差 |
| R14 | DEST=0.85 | 2.622 | 变差 |
| R15 | REGION=20 | 2.598 | 变差 |
| R16 | **NO age bonus + LENGTH=0.5** | **2.683** | 最佳 |
| R17 | age_bonus=0.15 (re-test) | 2.630 | age hurt |
| R18 | NO LENGTH_WEIGHT | 2.661 | LENGTH=0.5更好 |
| R19 | LENGTH=0.6 | 2.589 | 变差 |
| R20 | 55:45 time split | 1.987 | CATASTROPHIC |
| R21 | Variance test | **2.698** | 最佳! |

### ARCH-L 方差测试 (LENGTH=0.5最佳配置)
- R90: 2.683
- R93: 2.679
- R94: 2.680
- R97: 2.651
- R98: 2.675
- R99: 2.698 (最佳)

### 结论
- **架构饱和**: 所有Scheduler参数调优在2.56-2.70范围
- **LENGTH=0.5确实有效**: 比无LENGTH提升~0.02
- **Age bonus无效**: 会降低性能
- **时间分配50:50最优**: 任何改变都导致灾难
- **Executor修改全部crash**: 所有尝试都失败
- **最佳稳定分数**: ~2.68 (方差±0.02)
- **ARCH-E R5=2.720**: 可能为幸运跑分，在当前架构下难以复制

### 下一步
- 继续运行更多测试看能否突破2.72
- 考虑全新架构突破（但受4文件限制）
