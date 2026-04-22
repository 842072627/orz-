# 核心策略想法

- 创建时间: 2026-04-22
- 最后更新: 2026-04-22
- 状态: **进行中**

---

## 核心发现（必须死记）

### 1. 地图判断用字符串包含
```cpp
std::string name = env->map_name;
std::transform(name.begin(), name.end(), name.begin(), ::tolower);
if (name.find("room") != std::string::npos) { ... }
```
**不要写死地图名！用字符串包含判断！**

### 2. 分数公式
```
score = numTaskFinished / makespan
```
不是直接读取 score 字段！

### 3. 云端冠军不是一个
有6个团队的代码，每个团队用不同算法：
- Team_Kitty_Knight: CausalPIBT + PLNS
- Team_RAPID: LNS + LaCAM2
- Team_No_Man_Sky: 不同版本
- Team_SYSU-LCIS: 待研究

### 4. 我们的优势地图
- **fulfill**: 1.066 (超过所有冠军)
- 其他地图暂时落后

---

## delay 相关策略（重点！）

### starkit 新版加入了 delay

starkit 的 Executor 有 delay 机制：
- delayConfig: 延迟配置
- pDelay: 延迟概率
- minDelay/maxDelay: 延迟范围

### 可能的优化方向

#### 策略1: Delay-Aware Planning
- 在规划时考虑 delay 造成的堵塞
- 预留额外时间窗口
- 避免在关键路径上堆积

#### 策略2: Delay-Resistant Paths
- 选择更鲁棒的路径而非最短路径
- 考虑绕行避免单点故障
- 利用 map 中的备用通道

#### 策略3: Executor 优化
- 调整 delay 参数下的执行策略
- 更积极的避让机制
- 对延迟建模进行补偿

#### 策略4: Priority Inheritance
- 利用 delay 机制实现优先级继承
- 关键任务获得优先执行权
- 减少堵塞造成的任务失败

---

## Executor 相关策略

### Executor 控制流程
1. `process_new_plan()` - 处理新计划
2. `next_command()` - 生成下一个命令
3. `initialize()` - 初始化

### 可能的优化方向

#### 策略1: 执行窗口调整
- 增加 execution_window 获得更多预判时间
- 平衡计算时间和执行质量

#### 策略2: 命令缓存优化
- 预计算多个时间步的命令
- 减少决策延迟

#### 策略3: 旋转优化
- 考虑旋转代价的路径平滑
- 减少不必要的转向

---

## Planner 相关策略

### 当前架构
- frank_wolfe: 流量优化（fulfill上有效）
- PIBT: 路径规划（基础算法）

### 已验证的发现

| 实验 | 描述 | 结果 | 原因 |
|------|------|------|------|
| A42 | 跳过 frank_wolfe for tight maps | ✅ | frank_wolfe 在 maze-like 地图上超时 |
| A44 | Random tie-breaking | ✅ +0.006 | 匹配 DefaultPlanner 行为 |
| A45 | MyPlanner for random (PIBT-only) | ❌ | 崩溃 |
| A46 | MIN_PIBT_TIME=100 | ❌ | 变差 |
| A47 | frank_wolfe skip for random | ❌ | 变差 |

### 地图分类策略

| 地图 | 推荐Planner | frank_wolfe | 备注 |
|------|------------|-------------|------|
| fulfill | MyPlanner | ✅ | 流量优化有效 |
| warehouse | MyPlanner | ✅ | 同上 |
| room | MyPlanner | ❌ | 跳过避免超时 |
| maze | MyPlanner | ❌ | 跳过避免超时 |
| iron | MyPlanner | ❌ | 跳过避免超时 |
| orz | MyPlanner | ❌ | 跳过避免超时 |
| random | DefaultPlanner | N/A | MyPlanner崩溃 |
| bos | DefaultPlanner | N/A | 待研究 |

---

## 待研究想法

### 高优先级
1. **Random 地图崩溃原因**: MyPlanner 在 random 上崩溃，DefaultPlanner 正常
2. **Team_RAPID 的 LNS 算法**: 对 random 使用 LNS，效果可能更好
3. **Delay 建模**: starkit 的 delay 机制如何利用

### 中优先级
4. **Bos 地图优化**: 为什么 bos 分数这么低 (0.307)
5. **Room/Maze/Iron/Orz 的 frank_wolfe 跳过**: 是否有更智能的判断方式
6. **多团队算法融合**: 借鉴其他团队的优点

### 低优先级（架构重构）
7. **CausalPIBT 实现**: 冠军的核心算法
8. **PLNS 并行框架**: 多线程邻域搜索
9. **PathTable 冲突检测**: O(1) 冲突检测
