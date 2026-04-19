# MAPF 研究工作流

> 本文件：`C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\src\MAPF-workflow.md`
> 核心原则：永远有事做，永远研究驱动，永远记录完整

---

## ⭐ 核心原则（必须背诵）

### 1. 测试记录规范
> **每条记录必须包含，否则无效：**
> - `commit ID` + `exe MD5` + `Round` + `日期` + `分数`
> - **exe MD5 相同 = 代码没变 = 禁止重复测试**（唯一例外：测方差）
> - 获取 MD5：`Get-FileHash build/lifelong.exe -Algorithm MD5`

### 2. 禁止重复测试
> **重复测试 = 浪费生命。**
> - 验证回退成功 → 看编译，不跑测试
> - 同一个 exe 跑超过 3 次 → 除非用户要求测方差，否则禁止

### 3. 未完成工作禁止开新 ARCH
> **H3/H4 没干完 = 禁止开新 ARCH。**
> - 开新 ARCH 前检查：所有 H# 完成？skeleton 集成？所有方向试过？

### 4. 禁止 checkout 旧 commit
> **"从 Start-Kit 开始" ≠ checkout 旧 commit。**
> - 应该在当前 HEAD 继续微调
> - checkout 会导致编译失败、文件混淆

### 5. 文件区分
> **轮次地图在 `test-scripts/round/roundXXX/`，不是 `example_problems/`。**

---

## 🔄 研究循环

```
每次触发：
1. 读取 src/MAPF-records.md（当前状态）
2. 明确最重要任务
3. 执行实验（改代码 → 编译 → 测试 → 记录）
4. 反思：失败原因？新方向？
5. 继续下一个任务
```

---

## 📊 信号判断

| 信号 | 标准 |
|------|------|
| **明确提升** | 低方差地图 +5% 或 高方差地图 +15% |
| **明确失败** | ≥2 地图 -5% |
| **信号不明** | 提升/下降在 5% 以内，需研究原因 |

---

## ⚠️ 危险信号（立即停止）

- 同一个方向跑 5+ 次，分数一直在 baseline 附近
- 发现假设根本上是错误的
- 未完成工作没干完就想开新 ARCH

---

## 🚫 绝对禁止

| 禁止项 | 原因 |
|--------|------|
| checkout 旧 commit | 编译失败、文件混淆 |
| 重复测试（MD5 相同） | 浪费时间 |
| 未完成工作开新 ARCH | 丢弃工作 |
| 混淆 example/轮次文件 | 测试结果无效 |

---

## 📁 关键文件（全路径）

| 文件 | 路径 |
|------|------|
| Workflow | `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\src\MAPF-workflow.md` |
| 实验记录 | `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\src\MAPF-records.md` |
| 项目笔记 | `C:\Users\Administrator\Desktop\test-studio-mapf\项目笔记.md` |
| 测试结果 | `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts\round\roundXXX\` |
| 代码 | `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\src\my_planner\` |
| 备份 | `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\default_planner\` |
| Team_Kitty_Knight | `C:\Users\Administrator\Desktop\test-studio-mapf\Best-Implementations-2024\Team_Kitty_Knight_da7d428b\` |
| Start-Kit 3.1 | `C:\Users\Administrator\Desktop\test-studio-mapf\Start_Kit_main\` |
| build 输出 | `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\build\` |
| lifelong.exe | `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\build\lifelong.exe` |

---

## 📝 记录格式

```
## [Round XXX] commit: XXXXXXX MD5: XXXXX

| 地图 | 分数 | tasks |
|------|------|-------|
| random | 0.xxx | xxx |
| ... | ... | ... |
```

---

## 🌐 重要参考

| 资源 | 地址 |
|------|------|
| 比赛官网 | https://www.mapf.info/ |
| clawhub（技能市场） | https://clawhub.ai |
| Team_Kitty_Knight 代码 | `C:\Users\Administrator\Desktop\test-studio-mapf\Best-Implementations-2024\Team_Kitty_Knight_da7d428b\` |

---

## 🔧 常见操作

### 编译测试
```bash
cd C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts
cmd /c build.bat
cmd /c run_quick_test.bat
```

### 获取 MD5
```powershell
Get-FileHash C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\build\lifelong.exe -Algorithm MD5
```

### 回退代码
```bash
cd C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627
git checkout <good-commit> -- .
git commit -m "revert to <commit>"
```

### 查看状态
```bash
cd C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627
git log --oneline -5
git status
```

---

## 🔬 附录：ARCH 探索指南

### 调试期
- 编译失败 → 继续调试
- 0 分 / crash → 继续调试
- **只追求让程序跑起来，不追求分数**

### 验证期
- 程序正常运行后才进入
- 1-2 次测试确认信号
- **信号明确后立即停止，不要收集更多数据**

### 饱和判定
- 10+ 实验，所有方向都失败 → 饱和
- 有 skeleton 没集成 → **不是饱和，是没干完**
- 6 个实验，H3/H4 未完成 → **不是饱和，是没干完**

---

## 🔬 附录：H# 假设追踪

| H# | 假设 | 结果 | 状态 |
|----|------|------|------|
| H1 | +1 priority 移除 | fulfill+34% | ✅ 成功 |
| H2 | causalPIBT 例外移除 | fulfill-25% | ❌ 失败 |
| H3 | LNS 集成 | skeleton 未集成 | ⚠️ 未完成 |
| H4 | Dist2PathHeuristicTable | wrapper 未集成 | ⚠️ 未完成 |
| H5 | Frank-Wolfe 路径效率 | iron=0.000 | ❌ 崩溃 |
| H6 | 每步 quick FW | bos-14% | ❌ 失败 |

---

## 🔬 附录：架构差距分析

### Team_Kitty_Knight (9.039)
```
causal_pibt->multi_step() → LNS全局优化 → actions
```

### 我们 (H1 baseline)
```
plan() → 单次FW优化 → actions
```

**核心差距**：LNS 在每步做全局优化，我们只在 step 0 做一次

---

## 📋 附录：检查清单

### 测试前
- [ ] commit ID 正确
- [ ] exe MD5 已记录
- [ ] 用的是 roundXXX/ 地图，不是 example_problems/

### 记录前
- [ ] 分数没有乱码
- [ ] commit + MD5 + Round + 日期齐全
- [ ] 每个地图单独记录

### 开新 ARCH 前
- [ ] 所有 H# 都完成（成功/失败/证伪）
- [ ] 所有 skeleton 都集成
- [ ] 所有方向都试过
- [ ] 真的没有新方向了

---

## 📋 附录：版本历史

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-04-19 | v4.2 | 补全文件全路径 |
| 2026-04-19 | v4.1 | 补回重要参考 |
| 2026-04-19 | v4.0 | 精简至核心原则 + 附录 |
| 2026-04-19 | v3.x | 多次错误教训叠加 |
| 2026-04-18 | v2.x | 初始版本 |
