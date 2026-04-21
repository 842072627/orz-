# MAPF 工作流

---

## 目标

> **random + fulfill 总分达到 4.0 以上**

---

## 项目结构

| 文件 | 路径 |
|------|------|
| Workflow | `src/MAPF-workflow.md` |
| 实验记录 | `src/MAPF-records.md` |
| 研究笔记 | `src/my_planner/RESEARCH-notes.md` |
| 代码 | `src/my_planner/` |
| 备份 | `default_planner/` |
| 项目笔记 | `C:\Users\Administrator\Desktop\test-studio-mapf\项目笔记.md` |
| **冠军代码** | `Best-Implementations-2024/Team_Kitty_Knight_da7d428b/` |
| build | `build/lifelong.exe` |

---

## 测试脚本

| 脚本 | 用途 | 说明 |
|------|------|------|
| `run_mini_md5.ps1` | **默认测试** | random + fulfill，2个地图 |
| `run_quick_test_md5.ps1` | **全地图测试** | 7个地图全部测试，含MD5缓存 |
| `run_mini_variance.ps1` | 方差测试 | 非必要不使用 |
| `build.bat` | 编译 | 构建lifelong.exe |

- **默认**: `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\run_mini_md5.ps1` (random + fulfill)
- **全地图**: `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\run_quick_test_md5.ps1` (7地图)
- **方差**: `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\run_mini_variance.ps1` (非必要不使用)
- **build**: `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\build.bat`

结果目录: `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\round_mini\roundN\scores.txt`
全地图结果: `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\round\round1\scores.txt`

### 测试地图

**mini (random + fulfill)**:
- random
- fulfill

**全地图测试**:
- random
- bos
- room
- maze
- fulfill
- orz
- iron

---

## 核心接口文件（4个）

| 文件 | 职责 | 可重写函数 |
|------|------|-----------|
| `src/Entry.cpp` + `inc/Entry.h` | 入口协调 | `compute()`, `initialize()` |
| `src/TaskScheduler.cpp` + `inc/TaskScheduler.h` | 任务调度 | `plan()`, `initialize()` |
| `src/Executor.cpp` + `inc/Executor.h` | 执行控制 | `process_new_plan()`, `next_command()`, `initialize()` |
| `src/MAPFPlanner.cpp` + `inc/MAPFPlanner.h` | 路径规划 | `plan()`, `initialize()` |

---

## 算法实现（src/my_planner/）

| 文件 | 职责 |
|------|------|
| `planner.cpp` | PIBT路径规划 |
| `executor.cpp` | TPG执行控制 |
| `scheduler.cpp` | 贪心调度 |
| `pibt.cpp` | PIBT算法 |
| `flow.cpp` | 流优化 |
| `heuristics.cpp` | 启发式 |

---

## 修改规则

### ✅ 允许
- 优先在 `src/my_planner/` 修改算法
- 在`src`下新增 `.cpp/.h` 文件
- 修改 `CMakeLists.txt`
- 核心接口文件（4个）

### ❌ 禁止
- 修改 `default_planner/`（保持备份）
- 修改 `inc/` 头文件
- 未在允许范围内的其他一切文件

---

## 重要链接

| 资源 | 地址 |
|------|------|
| 比赛官网 | https://www.leagueofrobotrunners.org/submission |
| clawhub | https://clawhub.ai |

> ⚠️ 比赛官网用 React 编写，只能用浏览器访问。

---

## 冠军代码分析（已研究）

**Team_Kitty_Knight架构**：CausalPIBT + PLNS + TimeSpaceA* + GlobalManager

**关键发现**：
- 冠军的astar是 `f=g+h+op_flow` 的flow-guided A*，不是真正的3D时空搜索
- GlobalManager是LNS并行协调框架，不是简单的全局状态管理
- PathTable是O(1)冲突检测表，LNS跨neighborhood协调关键
- single-step vs multi-step：champion每步replan，我们window内stale

详细分析见：`src/my_planner/RESEARCH-notes.md`

---

## 当前方案（2026-04-21）

### 地图分流策略

由于room/maze/iron/orz地图会导致崩溃，采用按地图选择planner的策略：

| 地图类型 | Planner | 说明 |
|---------|---------|------|
| fulfill/warehouse | **MyPlanner** | 我们优化的代码 |
| 其他所有地图 | **DefaultPlanner** | starkit原始planner |

### 实现

`src/MAPFPlanner.cpp`中的`should_use_my_planner()`函数根据`env->map_name`判断。

### 测试结果

**全地图测试** (commit 38955c8):
```
random: 0.523 | bos: 0.310 | room: 0.180 | maze: 0.100
fulfill: 1.131 | orz: 0.220 | iron: 0.143
Total: 2.607
```

### 分数对比

| Map | 我们 | 云端冠军 | GAP |
|-----|------|----------|-----|
| random | 0.523 | 0.985 | -47% |
| bos | 0.310 | 1.000 | -69% |
| room | 0.180 | 0.880 | -80% |
| maze | 0.100 | 0.727 | -86% |
| fulfill | 1.131 | 0.825 | +37% ✅ |
| orz | 0.220 | 0.863 | -75% |
| iron | 0.143 | 0.860 | -83% |

**结论**：fulfill唯一超过冠军，其他地图差距巨大。