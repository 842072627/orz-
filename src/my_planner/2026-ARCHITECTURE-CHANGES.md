# 2026竞赛架构重大变化 - 研究笔记

**日期**: 2026-04-21  
**状态**: 初步研究完成

---

## 核心发现：我的理解完全错了！

**之前的问题**：
- 我瞎编了一个"冠军每步replan，我们50步replan一次"的差距
- 在这个**不存在的差距**上做了20+次实验，全部失败
- 浪费了大量时间

**官网明确说明**：
- **Dual-Timescale Architecture** 是比赛本身的设计，不是冠军的创新
- Planning慢，Execution快
- Executor控制GO/STOP时序，不是Planner

---

## 2026架构变化时间线

### V3.0.0 (2026-03-13) - 架构重构

**新增**：
- `Execution`组件 - 分离planning和execution
- **Multistep / staged planner execution** - windowed plans
- **Runtime delay generation** via `delayConfig`
- Agent footprint (`agentSize`) 碰撞检测代替点碰撞
- `delayIntervals` 输出

**关键变化**：
- Simulator使用**executor-driven** planning/execution流程
- 不再直接消费返回的actions
- 新的时间控制：preprocessing time, initial planning time, **planner communication interval**, per-action execution time, executor plan-processing time

### V3.1.0 (2026-04-11) - 改进

**新增**：
- Python接口层 (`python/common`)
- 大型benchmark示例

**改进**：
- 执行管道和共享环境同步
- staged-plan执行和assignment/update顺序
- 修复race-condition和timeout-control问题
- 修复**delayed和windowed execution**下的action validation问题

---

## 关键概念澄清

### 1. Dual-Timescale Architecture

```
Planning Layer (慢)          Execution Layer (快)
─────────────────            ─────────────────────
planner.plan()   ──────▶   executor.next_command()
    │                         │
    │ 返回actions序列         │ 每tick发GO/STOP
    │                         │
    ▼                         ▼
┌─────────────────────────────────────┐
│         Windowed Plan               │
│  [action1, action2, ..., actionN]   │
└─────────────────────────────────────┘
              │
              ▼
      ┌─────────────────┐
      │  GO / STOP     │
      │  每tick一次     │
      └─────────────────┘
```

**关键点**：
- Planner返回multi-step intent（一段actions）
- Executor通过TPG (Temporal Dependency Graph) 控制时序
- 每tick发一次GO/STOP命令
- **Delay是核心机制，不是干扰**

### 2. Runtime Delay机制

**配置参数** (`delayConfig`)：
```json
{
  "seed": 42,
  "minDelay": 1,
  "maxDelay": 5,
  "eventModel": "bernoulli",    // or "poisson"
  "pDelay": 0.1,                // per-agent delay probability
  "durationModel": "uniform",    // or "gaussian"
  "gaussMeanRatio": 0.5,
  "gaussStdRatio": 0.1
}
```

**Event模型**：
- `bernoulli`: 每个agent独立采样是否delay
- `poisson`: 团队级别，rate = teamSize * pDelay

**Duration模型**：
- `uniform`: [minDelay, maxDelay]均匀分布
- `gaussian`: 高斯分布

**暴露给Planner的**：
- `env->delay_event_distribution` (String: "bernoulli" or "poisson")
- `env->delay_time_distribution` (String: "uniform" or "gaussian")
- **Numeric参数不暴露！**

### 3. Collision Detection - Agent Footprint

**旧模式** (2024前)：
- Point occupancy
- Vertex/edge collisions

**新模式** (2026)：
- Agent footprint (`agentSize`) 代替点
- Safety bubbles
- 重叠检测基于几何，不是grid位置

### 4. Executor的职责

**官网明确**：
> "The executor can reschedule temporally but cannot change where the robot moves."

**Executor能做的**：
- GO: 允许移动
- STOP: 强制停止等待

**Executor不能做的**：
- 改变路径（where）
- 只能在时序上调整（when）

---

## 我们的代码状态

**我们的版本**: v3.1.0 (2026-04-11)
- 基于最新的2026架构
- 已有Executor组件
- 使用TPG (Temporal Dependency Graph)
- 支持staged plans

**关键文件**：
- `src/Executor.cpp` - Executor入口
- `src/my_planner/default_executor.cpp` - TPG实现
- `inc/Delay.h` - Delay类型定义
- `inc/DelayGenerator.h` - Delay生成器（start-kit保护文件）

---

## 真正需要研究的问题

1. **我们的Executor是否正确处理delays？**
   - TPG模拟时假设"no delay"（`//simulate with mcp (no delay)`）
   - 但运行时会有delay触发STOP
   - 这部分是如何衔接的？

2. **我们的planner返回的plan质量如何？**
   - window_size是多少？
   - 是否考虑了未来的delays？

3. **冠军的LNS框架在这个新架构下如何工作？**
   - 是否与executor协调？
   - 如何处理windowed execution？

---

## 下一步

1. 深入研究我们的Executor代码，特别是：
   - `execute_process_new_plan()` - windowed simulation
   - `execute_next_command()` - GO/STOP decision
   - 如何处理runtime delay

2. 对比冠军的Executor实现

3. 理解评分机制 - 为什么我们1.668而冠军4.0+？

---

## 教训

1. **先理解问题，再做实验**
2. **不要瞎编差距**
3. **官网文档是第一手资料**
4. **比赛规则是根本**

---

**待完成**：
- [ ] 深入研究Executor的delay处理逻辑
- [ ] 检查window_size设置
- [ ] 对比冠军的default_executor实现
- [ ] 理解评分机制差异
