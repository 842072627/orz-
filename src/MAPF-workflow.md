# MAPF 工作流

---

## 核心原则

**永远有事做，永远研究驱动，永远记录完整。**

### 目标
> **random + fulfill 总分达到 4.0 以上**

### 精神
> 深入研究官网 + 冠军代码 + 本地代码，实现**根本性突破**，而不是表面调参。

---

## 🔄 研究循环

每次触发：
1. 读取 src/MAPF-records.md（当前状态）
2. 明确最重要任务
3. 执行实验（改代码 → 编译 → 测试 → 记录）
4. 反思：失败原因？新方向？
5. 继续下一个任务

---

## 测试规则

### 测试脚本
- **默认**: `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\run_mini_md5.ps1` (random + fulfill)
记录存在C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\round_mini 里面
- **方差**: `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\run_mini_variance.ps1` (非必要不使用)
build脚本： `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\build.bat`

### 分数记录
- **必须记录 commit ID**
- 两图分数同等重要
- commit 时必须带上分数

### MD5
- 程序自己计算，写入 scores.txt
- 禁止手动计算

---

## 核心接口文件（4个）

| 文件 | 职责 | 可重写函数 |
|------|------|-----------|
| `src/Entry.cpp` + `inc/Entry.h` | --- | `compute()`, `initialize()` |
| `src/TaskScheduler.cpp` + `inc/TaskScheduler.h` | --- | `plan()`, `initialize()` |
| `src/Executor.cpp` + `inc/Executor.h` | --- | `process_new_plan()`, `next_command()`, `initialize()` |
| `src/MAPFPlanner.cpp` + `inc/MAPFPlanner.h` | --- | `plan()`, `initialize()` |

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

## 关键路径

| 文件 | 路径 |
|------|------|
| Workflow | `src/MAPF-workflow.md` |
| 实验记录 | `src/MAPF-records.md` |
| 代码 | `src/my_planner/` |
| 备份 | `default_planner/` |
| 项目笔记 | `C:\Users\Administrator\Desktop\test-studio-mapf\项目笔记.md` |
| **冠军代码** | `Best-Implementations-2024/` |
| build | `build/lifelong.exe` |

> ⚠️ `Best-Implementations-2024/` 是 **2024年各地图世界冠军代码**，可借鉴算法思路。
