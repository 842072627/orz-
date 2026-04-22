# MAPF 研究笔记索引

## 目录结构

```
src/notes/
├── INDEX.md          # 本索引文件
├── EXPERIMENTS/     # 实验记录
│   ├── A42.md       # 跳过frank_wolfe for tight maps
│   ├── A44.md       # Random tie-breaking
│   └── README.md     # 实验记录索引
├── IDEAS/           # 创意想法
│   ├── delay-strategy.md    # delay策略
│   ├── executor-strategy.md # executor策略
│   └── planner-strategy.md  # planner策略
├── MAPS/           # 各地图分析
│   ├── random.md
│   ├── bos.md
│   ├── room.md
│   ├── maze.md
│   ├── fulfill.md
│   ├── orz.md
│   └── iron.md
└── RESEARCH/       # 研究资料
    ├── champion-teams.md    # 冠军团队分析
    └── algorithm-notes.md   # 算法笔记
```

## 更新规则

1. **所有想法必须写进这里**，不许留在别处
2. **每次实验后立即记录**，不许"之后再写"
3. **笔记按类别归档**，不许混放
4. **每周整理一次索引**，保持结构清晰

## 笔记格式

每个笔记文件头部要有：
```markdown
# [标题]
- 创建时间: YYYY-MM-DD
- 最后更新: YYYY-MM-DD
- 状态: [进行中/已验证/已放弃]
```
