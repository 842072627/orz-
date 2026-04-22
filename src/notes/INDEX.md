# MAPF 研究笔记索引

## ⚠️ 死记规则

1. **所有想法必须写进这里**，不许留在别处
2. **每次实验后立即记录**，不许"之后再写"
3. **笔记按类别归档**，不许混放
4. **查看笔记先看 INDEX**

---

## 目录结构

```
src/notes/
├── INDEX.md              # 本索引
├── EXPERIMENTS/          # 实验记录
│   ├── README.md         # 实验索引
│   └── A42-A47.md        # A42-A47实验详情
├── IDEAS/                # 创意想法
│   └── core-strategies.md # 核心策略（delay/executor/planner）
├── MAPS/                 # 地图分析
│   └── all-maps.md       # 7个地图分析
└── RESEARCH/             # 研究资料
    └── champion-teams.md # 冠军团队分析
```

---

## 核心绝对路径

| 用途 | 路径 |
|------|------|
| 项目根目录 | `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627` |
| 笔记文件夹 | `C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627\src\notes` |
| 测试脚本 | `C:\Users\Administrator\Desktop\test-studio-mapf\test-scripts` |
| 冠军代码 | `C:\Users\Administrator\Desktop\test-studio-mapf\Best-Implementations-2024` |

---

## 快速参考

### 分数公式
```
score = numTaskFinished / makespan
```

### 地图判断方式
```cpp
std::string name = env->map_name;
std::transform(name.begin(), name.end(), name.begin(), ::tolower);
if (name.find("room") != std::string::npos) { ... }
```

### 当前最佳分数 (commit 3f1c635)
| 地图 | 分数 | Planner |
|------|------|---------|
| random | 0.52 | DefaultPlanner |
| bos | 0.31 | DefaultPlanner |
| room | 0.15 | MyPlanner |
| maze | 0.09 | MyPlanner |
| fulfill | 1.07 | MyPlanner |
| orz | 0.22 | MyPlanner |
| iron | 0.15 | MyPlanner |
| **Total** | **2.50** | - |

### 冠军团队（不只一个！）
- Team_Kitty_Knight: CausalPIBT + PLNS
- **Team_RAPID**: LNS + LaCAM2
- Team_No_Man_Sky (3版本)
- Team_SYSU-LCIS

---

## 更新规则

1. **实验后立即写笔记**到 `EXPERIMENTS/`
2. **想法随时写**到 `IDEAS/`
3. **每周整理一次索引**
4. **笔记头部要有元数据**：
```markdown
# [标题]
- 创建时间: YYYY-MM-DD
- 最后更新: YYYY-MM-DD
- 状态: [进行中/已验证/已放弃]
```
