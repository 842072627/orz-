# MAPF 竞赛 — 研究驱动开发工作流

> **每一步都必须记录。放弃方向必须经过审批。无研究记录不写代码。**
> 本文档：`MAPF-workflow.md`
> **重要变更（2026-04-17）：不再使用 git branch，所有工作在 main 上进行，ARCH 标记仅存在于 commit message 中。**

---

## ⚠️ 文件修改规则（最优先，必须遵守）

### 可修改文件（Combined 赛道）

| 文件 | 职责 | 可重写函数 |
|------|------|-----------|
| `src/Entry.cpp` + `inc/Entry.h` | 双时间尺度协调者 | `compute()`, `initialize()` |
| `src/TaskScheduler.cpp` + `inc/TaskScheduler.h` | 任务分配 | `plan()`, `initialize()` |
| `src/Executor.cpp` + `inc/Executor.h` | 执行控制 | `process_new_plan()`, `next_command()`, `initialize()` |
| `src/MAPFPlanner.cpp` + `inc/MAPFPlanner.h` | 路径规划 | `plan()`, `initialize()` |

### ✅ 允许做的事
- **优先新增 `.cpp/.h` 文件**，而不是在4个文件里堆代码
- 添加新 `.cpp` / `.h` 文件到 `src/` 或 `inc/`
- 在预处理的 `initialize()` 阶段加载辅助数据到 `env->file_storage_path`
- 预计算启发式数据、地图分析数据等

### ❌ 绝对禁止
- 修改 `src/` 下除上述4个以外的所有文件
- 修改 `inc/` 下所有头文件
- 修改 `default_planner/` 下所有文件（包含默认调度器、规划器、执行器）
- 修改任何禁止文件，哪怕只是加一行注释

### 🔒 改前检查清单（每次写代码前必须核对）

每次写代码前，用 `git diff --stat` 确认实际修改范围：

```bash
# 在修改前后各运行一次，对比确认
git diff --stat HEAD
```

```
我的改动：
□ 我在改的文件是：________________
□ 这个文件在上面的"可修改文件"列表中吗？
□ git diff --stat 确认没有碰其他文件？

如果任何一个回答是"否" → 立即停手，重新检查
```

---

## 核心原则

1. **不研究，不动手** — 代码改变之前，必须有充分的研究支撑
2. **不放弃，无记录** — 每一个被跳过的方向必须记录"放弃原因"和"替代方案"
3. **不浅尝辄止** — 一个方向必须完成"最小完整验证"（4轮+）才能换方向
4. **用固定脚本** — 所有操作使用 `test-scripts/build.bat`、`run_quick_test.bat` 等固定脚本
5. **不乱改** — 只改4个允许的文件，可自由新增 `.cpp/.h`
6. **优先新增文件** — 鼓励新建 `.cpp/.h` 而不是堆代码到4个文件
7. **不建 branch** — 所有工作在 main branch，ARCH 标记仅在 commit message 中
8. **失败也要记录** — 架构失败后备份代码，用于后续混合架构研究

---

## 核心文件

| 文件 | 路径 | 说明 |
|------|------|------|
| MAPF-workflow.md | `../MAPF-workflow.md` | 本工作流定义 |
| MAPF-records.md | `MAPF-records.md` | 实验记录（每轮必更） |
| 项目研究笔记.md | `../项目研究笔记.md` | 研究过程、诊断、灵感、失败分析 |
| archive/ | `../archive/` | 失败实验的代码备份（用于混合架构研究） |

---

## ARCH 标记规则

> **ARCH-X 只是 commit message 中的标记，不是 git branch。**

- Commit message 格式：`[ARCH-X] Ry: <方向> → <总分>`
- 工作流程与之前完全相同，只是每次直接 commit 到 main
- 历史 ARCH（ARCH-E 2.720 等）已通过 commit message 记录，无需重建 branch

---

## 阶段总览

```
[0.自检] → [1.研究] → [2.思考] → [3.提案] → [4.审批] → [5.改前检查] → [6.动手] → [7.测试] → [8.记录] → [9.复盘] → [10.饱和]
            ↑                                                                                                                ↓
            ←←←←←←←←←←←←←← 如果失败 ←←←←←←←←←←←←←←
```

---

## 阶段 0：启动自检（每次迭代必须执行）

每次开始工作前，先读 `MAPF-records.md` 核对状态：

### 状态核对

1. **全局状态面板**：
   - "当前 ARCH" 和 "当前轮次" 是否与实际一致？
   - "全局最佳" 是否是最新值？（对比最新 scores.txt）

2. **待实验队列**：
   - "待实验队列" 中标记的状态是否准确？
   - 是否有遗漏的方向未记录？

3. **最新测试结果**：
   - 读取最新 round 目录的 scores.txt
   - 如果分数比"全局最佳"高 → 更新"全局最佳"
   - 如果状态面板有误 → 直接修正

### Workflow 完整性检查

- `MAPF-workflow.md` 是否存在且可读？
- `MAPF-records.md` 是否存在且可读？
- `项目研究笔记.md` 是否存在？（不存在则创建）
- `test-scripts/build.bat` 和 `run_quick_test.bat` 路径是否正确？

### 发现问题 → 直接修正

| 问题类型 | 行动 |
|---------|------|
| 状态过时 | 更新 MAPF-records.md |
| 路径错误 | 修正路径 |
| 环节缺失 | 补充 |
| 格式混乱 | 整理 |

**修正后在回传中说明"自检修正：XXX"**

---

## 阶段 1：研究（Research）

### 必读清单（每项完成后打 ✅）

**冠军代码**：
- [ ] `Best-Implementations-2024/Team_Kitty_Knight_da7d428b/` — 9.039，有完整 my_planner 源码
- [ ] `Best-Implementations-2024/Team_No_Man_Sky_*/` — 冠军调度器
- [ ] `LORR26_842072627/default_planner/scheduler.cpp` — 默认基准（理解逻辑，禁止修改）

**文档**：
- [ ] `LORR26_842072627/Input_Output_Format.md`
- [ ] `LORR26_842072627/Evaluation_Environment.md`
- [ ] `MAPF-records.md` — 历史实验记录
- [ ] `项目研究笔记.md` — 研究过程和诊断

### 研究记录格式（追加到项目研究笔记.md）

```markdown
## 研究：<方向>
**时间**：YYYY-MM-DD HH:MM - HH:MM

### 诊断
- 当前瓶颈：
- 假设：

### 冠军代码借鉴
- 发现：

### 实验设计
- 改动文件：
- 新增文件：
- 关键参数：
- 风险评估：
```

---

## 阶段 2：思考（Think）

在研究记录完成后，回答：

```markdown
### 逻辑推演

1. **假设是否成立？**
   - 支持证据（来自研究）：
   - 反面证据（来自研究）：

2. **最坏情况？**
   - Crash风险：高/中/低
   - 分数下降风险：高/中/低

3. **与历史对比？**
   - 类似方向：ARCH-X RY，失败原因：
   - 本次与之前的本质区别：

4. **新增文件？**（优先）
   - 是否需要新增 `.cpp/.h`？
   - 新增文件的职责是什么？
```

---

## 阶段 3-4：提案 → 审批（自动合并）

将以下内容追加到 `MAPF-records.md` 的"待实验队列"：

```markdown
| <方向> | P0/P1/P2 | 🔬研究中 | ARCH-X | <假设一句话> |
```

---

## 阶段 5：改前检查

**每次写代码之前，必须完成这个检查清单**：

```markdown
### 🔒 改前检查清单

**我计划改的文件**：
1. `src/_______________` — ✅ 在允许列表中
2. `src/_______________` — ✅ 在允许列表中
3. 新增 `src/_______________` / `inc/_______________` — ✅ 允许新增

**禁止文件检查**（确保没有碰到）：
- `default_planner/` — ❌ 没有碰
- `inc/` 下的禁止文件 — ❌ 没有碰
- `src/` 下的其他文件 — ❌ 没有碰

**git diff --stat 确认**：✅

**确认后开始写代码** ✅
```

---

## 阶段 6：动手（Implement）

### 代码规则

1. **一次只改一个方向**
2. **只改4个允许的文件** + 可自由新增 `.cpp/.h`
3. **禁止在禁止文件里加任何内容**（哪怕一行注释）
4. **优先新增文件**：当逻辑复杂 > 150行时，拆分成独立 `.cpp/.h`
5. **新增文件要声明职责**：在文件头部注释说明这个文件做什么
6. **提交信息格式**：
   ```
   [ARCH-X] Ry: <方向> → <总分> (<原因>)
   ```
7. **直接 commit 到 main，不创建 branch**

### 何时必须新增文件（强制）

| 情况 | 行动 |
|------|------|
| 调度逻辑复杂 > 150行 | 拆分成 helper.cpp/h |
| 需要新的数据结构（区域网格、历史记录） | 新增专门的 .h/.cpp |
| 不同子算法需要隔离 | 新增 submodule .h/.cpp |
| 注释解释不清楚的算法 | 新增 algorithm_name.h 单独文档 |

---

## 阶段 7：测试（固定脚本）

### 固定脚本

| 操作 | 脚本 |
|------|------|
| 编译 | `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\build.bat` |
| 测试 | `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\run_quick_test.bat` |
| 查看结果 | `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\round\roundXXX\` |

### ⏱️ 测试时长监控

**正常耗时**：约 10-20 分钟（7张图）

**⚠️ 10分钟无输出 → 超时警告**：
- 测试开始后超过10分钟无任何新输出 → 记录"超时警告"
- 继续等待但不中断，30分钟无输出才考虑终止

**⚠️ 重要：保证 7 张图完整跑完**

**7 图完整性检查**：
- 测试完成后检查 roundXXX 目录下是否有 7 个 .json 文件
- 如果 iron.json 缺失 → **实验无效**，排查原因

**测试结果路径**：`C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\round\roundXXX\`

### 流程

```
1. 研究完成 → 提案记录到 MAPF-records.md
2. ✅ 完成改前检查清单
3. 运行 build.bat
4. 如果编译失败 → 修复 → 返回步骤3
5. 运行 run_quick_test.bat
6. 等待测试完成（自动保存到 roundXXX）
7. ✅ 检查 7 个 .json 是否齐全
8. 解析 scores.txt → 记录到 MAPF-records.md
8. 评估：
   - 提升 >2% → git commit 到 main
   - 持平/下降 → 进入复盘
   - Crash → 记录崩溃、修复、回退
```

---

## 阶段 8：记录（Document）

**每轮实验必须有结果记录**，格式：

```markdown
### ✅ ARCH-X Ry — <方向> — 已完成

**假设**：（来自研究阶段）
**改动**：
- 文件：
- 新增文件：
- 关键参数/逻辑：

**结果**：
| Map | 本轮分数 | vs 基准 2.720 |
|-----|---------|--------------|
| **Total** | **X.XXX** | **±X.XXX** |

**分析**：
- 假设是否成立？
- 下一步方向：
```

---

## 阶段 9：复盘（Review）

**当结果未达预期时**，在继续之前必须完成：

```markdown
### ❌ ARCH-X Ry — <方向> — 未达预期

**预期**：Total ≥ 2.720 | **实际**：X.XXX

**失败分析**：
1. 假设验证：成立/不成立/部分成立，原因：
2. 哪个地图拖后腿：fulfill? room? maze?
3. 与历史对比：ARCH-X Ry 也失败过，共同原因：

**决定**：
- ⏳ 继续同一方向（调参）
- 🔄 换相关方向（不换ARCH）
- ❌ 放弃（必须记录原因+3个替代方案）

**失败实验备份**：
- 备份代码到 archive/ARCH-X_Ry/<方向>/
- 记录到 项目研究笔记.md
```

---

## 阶段 10：放弃方向登记 & 饱和判定

### 放弃方向登记

在 `MAPF-records.md` 的"放弃方向登记"中记录：

```markdown
| 方向 | ARCH | 放弃轮次 | 放弃原因 | 替代方案 |
|------|------|---------|---------|---------|
| XXX | ARCH-X | Ry | 具体原因 | 1. AAA 2. BBB 3. CCC |
```

**失败实验代码备份**（强制）：
- 将失败实验的代码备份到 `C:\Users\Administrator\Desktop\test-studio-mapf\archive\ARCH-X_Ry\<方向>\`
- 包括：修改的文件快照 + 实验结果 + 失败原因记录
- 目的：用于后续混合架构研究，避免重复劳动

**饱和判定**

ARCH 饱和必须满足：
1. **连续10轮无提升**（以2.720为基准）
2. 或者**所有P0-P1方向全部研究并记录失败原因**
3. 才能开新 ARCH（commit message 标记 `[ARCH-Y]`）

---

## 当前全局最佳

- **本地最佳**：ARCH-E R5 = **2.720**（pickup=7.3, dest=0.8, LENGTH=0.5）
- **云端基准**：8.02
- **云端最佳**：9.44
- **主要差距**：room-A（+0.432）、orz（+0.183）、maze-B（+0.153）

---

## 必读清单状态

```
[ ] KITTY_KNIGHT my_planner/ 源码（9.039）
[ ] No Man Sky TaskScheduler.cpp（9.842）
[ ] default_planner/scheduler.cpp（理解基准，禁止修改）
[ ] Input_Output_Format.md
[ ] Evaluation_Environment.md
[ ] MAPF-records.md（历史）
[ ] 项目研究笔记.md（当前研究状态）

→ 全部 ✅ 才能开始第一个实验
```
