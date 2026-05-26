# 🔴 比赛规则检查清单（每次修改代码前必须逐条核对！）

## 不可修改的文件（任何赛道，碰都不能碰）

**src/ 目录**
- `src/ActionModel.cpp` ❌
- `src/Evaluation.cpp` ❌
- `src/Logger.cpp` ❌（当前是本地 stub，提交前必须恢复）
- `src/States.cpp` ❌
- `src/driver.cpp` ❌
- `src/CompetitionSystem.cpp` ❌
- `src/Grid.cpp` ❌
- `src/common.cpp` ❌
- `src/TaskManager.cpp` ❌
- `src/Simulator.cpp` ❌
- `src/DelayGenerator.cpp` ❌

**inc/ 目录**
- `inc/ActionModel.h` ❌
- `inc/Evaluation.h` ❌
- `inc/Logger.h` ❌（当前是本地 stub，提交前必须恢复）
- `inc/SharedEnv.h` ❌
- `inc/Tasks.h` ❌
- `inc/CompetitionSystem.h` ❌
- `inc/Grid.h` ❌
- `inc/States.h` ❌
- `inc/common.h` ❌
- `inc/TaskManager.h` ❌
- `inc/Simulator.h` ❌
- `inc/DelayGenerator.h` ❌
- `inc/Counter.h` ❌
- `inc/Delay.h` ❌
- `inc/Plan.h` ❌

**default_planner/ 目录（所有文件都不可改！）**
- `default_planner/*.cpp` ❌
- `default_planner/*.h` ❌
- **！！！不要在 default_planner/ 里改任何东西！！！**
- 要改 planner 代码 → 复制到 `src/my_planner/` 改 my_ 版本

**python/ 目录**
- `python/common/MAPFbinding.cpp` ❌
- `python/common/opaque_types.h` ❌
- 等等所有 python/common/ 文件 ❌

**核心原则**：评价系统会在编译前用官方 start-kit 覆盖这些文件，改了也白改，还会编译失败！

## 可以修改的文件（Combined Track）

✅ `src/MAPFPlanner.cpp` — 规划器入口
✅ `src/TaskScheduler.cpp` — 调度器入口
✅ `src/Entry.cpp` — 执行器入口
✅ `src/Executor.cpp` — 执行器
✅ `src/strategy_*.h` — 调度策略
✅ `src/my_planner/*` — 自定义规划器（从 default_planner 复制改名）
✅ `src/config.h` — 策略配置
✅ `CMakeLists.txt` — 构建系统
✅ `compile.sh` — 编译脚本
✅ `apt.txt` / `pip.txt` — 依赖
✅ `exp.py` — 本地实验系统

## 运行时行为红线（同样违规！）

**规则原文：**
> "You must not alter, override, spoof, or otherwise distort the behavior of these protected functionalities in any manner, whether directly or indirectly through other modules, shared data structures, side effects, or runtime interactions."

**这意味着：**
- ❌ 不能通过 `env->task_pool` 插入自定义任务（TaskManager 管理此数据）
- ❌ 不能修改 `env->new_tasks`（TaskManager 管理）
- ❌ 不能修改 `env->goal_locations`（Entry 管理）
- ❌ 不能修改 `env->curr_task_schedule`（Scheduler 管理）
- ❌ 不能通过 shared data structures 间接干扰受保护模块

**可以通过调度器内部逻辑合法做的事：**
- ✅ 修改 `base_score()` 公式（在 `strategy_*.h` 中）
- ✅ 在 `MapFPlanner.cpp` 中调用 `MyPlanner::plan()` 使用自定义规划器
- ✅ 在 `src/my_planner/my_heuristics.cpp` 中修改 PIBT 的启发式函数
- ✅ 在 `src/my_planner/my_pibt.cpp` 中修改 causalPIBT 逻辑
- ✅ 在配置文件 `config.h` 中选择不同的策略

## ⚠️ 提交云端前的准备工作

1. **恢复 `inc/Logger.h` 和 `inc/Logger.cpp`** 为官方版本
2. **确保 `default_planner/` 没有任何修改**（git checkout -- default_planner/）
3. **确保 `inc/` 没有任何修改**（git checkout -- inc/）
4. **确保 `src/TaskManager.cpp` 没有修改**
5. 编译验证通过后再提交
