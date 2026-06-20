// optsvr/Config.h — 编译期默认参数
#pragma once

namespace optsvr {

// ═══════════════════════════════════════════════════════════════════
// 全局控制
// ═══════════════════════════════════════════════════════════════════
constexpr bool  SKIP_TEST       = false;  // 默认不跳过（MapTuner 按地图类型设为 true）
constexpr int   DEBUG_LEVEL     = 0;       // 0=静默, 1=概要, 2=细节
constexpr int   NUM_THREADS     = 0;       // 0=自动检测

// ═══════════════════════════════════════════════════════════════════
// 模块选择
// ═══════════════════════════════════════════════════════════════════
// Scheduler: 0=Default, 1=GreedyHeap, 2=DHSchedule, 3=GameOpt, 4=TaskMatch, 5=DHFULLSchedule, 6=OptMazeSchedule, 7=OptRoom, 8=GameBalanceOpt, 9=GameSegmentFlow, 10=IronZone
constexpr int   SCHEDULE_MODE   = 2;
// Executor:  0=Default(官方), 1=Origin(MCP跟随链), 2=Hybrid, 3=DHExecutor(TPG+临时穿越), 4=RoomOptExecutor(连续STOP截断), 5=RoomExecutor(Room专用TPG), 6=GameExecutor(Game专用:对流检测+截断)
constexpr int   EXECUTOR_MODE   = 3;
// Planner:   0=Default, 1=FlowDecay, 2=GameOpt, 3=TurnAware, 4=EPIBT(轻量turn-cost), 5=IdleYield(空闲让路), 6=RoomPlanner(Room专用PIBT), 7=RoomTidalPlanner(Room潮汐门控), 8=GameNarrowPIBT, 9=OptMazePlanner
constexpr int   PLANNER_MODE    = 1;


// ═══════════════════════════════════════════════════════════════════
// GreedyHeap 专属（SCHEDULE_MODE=1）
// ═══════════════════════════════════════════════════════════════════
constexpr int   HEAP_REBUILD_PCT    = 45;      // rebuild 占时间预算百分比
constexpr int   HEAP_LNS_PCT        = 10;      // LNS 优化百分比
constexpr int   HEAP_SORT_K         = 500;     // 排序前 K 个候选
constexpr float HEAP_MAX_ASSIGN     = 1.0f;    // 分配比例（0.0~1.0, 1.0=全部分配, 0.4=只分配40%的agent）

// ═══════════════════════════════════════════════════════════════════
// TaskMatch 专属（SCHEDULE_MODE=4）
// ═══════════════════════════════════════════════════════════════════
constexpr int   SCORE_MODE          = 5;       // 打分公式：5 = 5*dist + task_len
constexpr int   CANDIDATE_TOPK      = 50;      // 大规模 TopK 候选数
constexpr float DIFFICULTY_WEIGHT   = 0.0f;    // score_mode=3 难度权重
constexpr float CONGESTION_DECAY    = 0.0f;    // 拥塞衰减（需 USE_HISTORY=true）
constexpr int   CONGESTION_GRID_CELL   = 4;    // 拥塞网格粒度
constexpr int   CONGESTION_GRID_RADIUS = 1;    // 拥塞查询邻域半径
constexpr bool  CONGESTION_USE_HISTORY = false; // true=衰减累积, false=每步清零重算

// ═══════════════════════════════════════════════════════════════════
// GreedyHeap/TaskMatch 共用（SCHEDULE_MODE=1 或 4）
// ═══════════════════════════════════════════════════════════════════
constexpr float HEAP_DIST_WEIGHT    = 5.0f;    // dist_to_start 权重
constexpr bool  REASSIGN_ENABLED    = true;    // 重分配未打开任务
constexpr float REASSIGN_KEEP_BIAS  = 6.0f;    // 重分配原配对偏好
constexpr int   REASSIGN_MIN_DIST   = 10;      // 距离保护阈值

// ═══════════════════════════════════════════════════════════════════
// GameOpt 专属（SCHEDULE_MODE=3）— 与 mode=1/4 完全独立
// ═══════════════════════════════════════════════════════════════════
// 旧版 GreedyHeapMatcher 参数（保留兼容）
constexpr float GAME_DIST_WEIGHT    = 16.0f;   // dist_to_start 权重
constexpr int   GAME_REBUILD_PCT    = 45;      // rebuild 占时间预算百分比
constexpr int   GAME_LNS_PCT        = 10;      // LNS 优化百分比
constexpr int   GAME_SORT_K         = 300;     // 排序前 K 个候选（R7最优）
constexpr bool  GAME_REASSIGN       = true;    // 重分配未打开任务
constexpr float GAME_KEEP_BIAS      = 10.0f;   // 重分配原配对偏好
constexpr int   GAME_MIN_DIST       = 10;      // 距离保护阈值
constexpr float GAME_CONG_DECAY     = 0.9f;    // 拥塞衰减
constexpr int   GAME_GRID_CELL      = 4;       // 拥塞网格粒度
constexpr int   GAME_GRID_RADIUS    = 1;       // 拥塞查询邻域半径
constexpr float GAME_CONG_WEIGHT    = 0.0f;    // 拥塞惩罚权重（0=关闭）
constexpr float GAME_DIST_POW       = 0.7f;    // 距离指数（R5-R7最优: 非线性打分）
constexpr float GAME_DEST_CONG_WEIGHT = 0.0f;  // 目的地拥塞惩罚权重（0=关闭）
constexpr float GAME_DEST_CONG_CAP  = 0.0f;    // 目的地负载上限（0=关闭cap模式，>0=超限指数惩罚）
constexpr float GAME_AGE_WEIGHT     = 0.0f;    // Task Age 饥饿度权重（0=关闭）
constexpr float GAME_COMP_WEIGHT    = 0.0f;    // Completion Potential 完成概率权重（0=关闭）
constexpr int   GAME_TOTAL_STEPS    = 5000;    // 总仿真步数（用于 completion 计算）
constexpr float GAME_ASSIGN_RATIO   = 1.0f;    // 部分分配比例（0.0~1.0, 1.0=全部分配）
constexpr int   GAME_IDLE_DIST      = 0;       // 空闲距离阈值（agent到最近task超过此距离不分配, 0=关闭）

// ═══════════════════════════════════════════════════════════════════
// GameBalanceOpt 专属（SCHEDULE_MODE=8）— 中心线调度器参数
// ═══════════════════════════════════════════════════════════════════
constexpr int   GAME_ZONE_HARD_CAP  = 15;      // zone 热区密度阈值
constexpr int   GAME_EXEMPT_MAIN    = 23;      // Layer1 main zone 豁免阈值
constexpr int   GAME_DENSITY_CAP    = 100;     // 默认打分中密度上限
constexpr int   GAME_DIR_THRESHOLD  = 10;      // 方向锁最低 agent 数
constexpr int   GAME_ROUTE_PENALTY  = 20;      // 密度惩罚起始阈值
constexpr int   GAME_EX_MAIN_CAP   = 35;       // Layer3 逃逸豁免主区上限
constexpr int   GAME_EX_NONMAIN_CAP = 80;      // Layer3 逃逸豁免非主区上限
constexpr float GAME_BAL_ASSIGN_RATIO = 1.0f;  // mode=8 分配比例（0.0~1.0, 1.0=不限制）



// ---- 已验证有效: 瓶颈宽度感知容量控制 ----
constexpr bool  GAME_WIDTH_AWARE      = true;    // (IDEA3+4最优)
constexpr int   GAME_NARROW_THRESHOLD = 4;       // 宽度<=此值视为窄瓶颈
constexpr float GAME_NARROW_CAP_RATIO = 0.5f;    // 窄瓶颈容量 = width * ratio

// ---- 已验证有效: PIBT 逆流惩罚 ----
constexpr bool  GAME_PIBT_COUNTER_FLOW = true;   // (IDEA3+4最优)
constexpr int   GAME_COUNTER_FLOW_COST = 20;     // 逆流额外代价 (cost=20最优)

// ---- 已验证有效: 紧迫度优先级（最后一公里加速）----
constexpr bool  GAME_URGENCY_PRIORITY  = true;   // 默认启用 (+3~5%)
constexpr float GAME_URGENCY_WEIGHT    = 10.0f;  // weight=10 最优
constexpr int   GAME_URGENCY_THRESHOLD = 50;     // thresh=50 最优

// ═══════════════════════════════════════════════════════════════════
// 新一轮自适应策略参数
// ═══════════════════════════════════════════════════════════════════

// ---- IDEA1: 搜索半径自适应扩展 ----
// 后期 zone 内无 free task 时，逐步扩大搜索范围
constexpr int   GAME_SEARCH_RADIUS     = 1;      // 默认搜索半径(zh_dist)
constexpr int   GAME_LATE_SEARCH_RADIUS = 3;     // 后半程搜索半径
constexpr float GAME_LATE_PHASE_RATIO  = 0.4f;   // step > total*ratio 进入后期模式

// ---- IDEA3: 后期zone约束放宽 ----
// 后半程自动放宽density_cap和zone_hard_cap
constexpr bool  GAME_LATE_RELAX        = false;  // true=启用后期放宽
constexpr int   GAME_LATE_DENSITY_CAP  = 200;    // 后期density_cap
constexpr int   GAME_LATE_ZONE_CAP     = 25;     // 后期zone_hard_cap

// ---- GameNarrowPIBT 提交默认策略 ----
// 0=关闭实验局部策略；23=R19-I2 density-gated wait pressure。
constexpr int   GAME_NARROW_PIBT_IDEA = 0;
constexpr int   GAME_NARROW_WAIT_BONUS = 2;
constexpr int   GAME_NARROW_DENSITY_THRESHOLD = 12;
constexpr int   GAME_NARROW_BASE_COUNTER_PCT = 100;
constexpr int   GAME_NARROW_LATE_START = 2500;
constexpr int   GAME_NARROW_LATE_EXTRA = 1;

// ---- IDEA5: Task Age排序优先 ----
// free_tasks按等待时间排序，优先分配久等task
constexpr bool  GAME_AGE_SORT         = false;   // true=启用age排序
constexpr float GAME_AGE_SORT_WEIGHT  = 0.5f;    // score加成: + weight * age/total_steps

// ═══════════════════════════════════════════════════════════════════
// Room 部分分配（SCHEDULE_MODE=5 的 Room 分支）
// ═══════════════════════════════════════════════════════════════════
// ASSIGN_AGENT_MODE:
//   0 = 关闭（全部分配）
//   1 = 动态筛选：按到最近K个task距离总和排序，取前ratio比例agent
//   2 = 静态随机：初始化时随机选ratio比例agent永久参与，其余永久待命
constexpr int   ROOM_ASSIGN_AGENT_MODE = 0;
constexpr float ROOM_ASSIGN_RATIO      = 1.0f;   // 0.0~1.0, 1.0=全部分配
constexpr int   ROOM_TOPK_TASKS        = 10;     // mode=1 每个agent采样最近K个task

// ═══════════════════════════════════════════════════════════════════
// OptRoom 专属（SCHEDULE_MODE=7）— 核心参数
// ═══════════════════════════════════════════════════════════════════
// ---- 核心打分 ----
constexpr float OPT_ROOM_DIST_FIRST_W    = 2.0f;   // first_weight (MapTuner验证最优)
constexpr float OPT_ROOM_DIST_OVERALL    = 0.9f;   // 路径总 cost 的全局系数
constexpr float OPT_ROOM_SHORT_TASK_W    = 1.0f;   // 短任务偏好权重
constexpr int   OPT_ROOM_TOTAL_TICKS     = 3000;   // 总仿真步数
constexpr int   OPT_ROOM_RECLAIM_MARGIN  = 1;      // reclaim 安全余量

// ---- 限流与匹配 ----
constexpr float OPT_ROOM_ASSIGN_RATIO    = 0.55f;  // 分配比例 (R12验证最优)
constexpr int   OPT_ROOM_MATCH_MODE      = 1;      // 1=greedy (R10验证最优)

// ---- 时间安全过滤 ----
constexpr float OPT_ROOM_TIME_SAFETY     = 1.5f;   // cost 乘数 (R10/R11验证最优)

// ---- 重分配 ----
constexpr int   OPT_ROOM_REASSIGN        = 0;      // 0=关闭, 1=启用重分配
constexpr int   OPT_ROOM_REASSIGN_MIN_DIST = 5;    // 重分配距离保护

// ═══════════════════════════════════════════════════════════════════
// OptMaze 专属（SCHEDULE_MODE=6 / PLANNER_MODE=9）
// MapTuner 会在 maze_40 上覆盖为 RT4 选定策略。
// ═══════════════════════════════════════════════════════════════════
constexpr int   OPT_MAZE_CELL_SIZE       = 2;
constexpr float OPT_MAZE_DIST_FIRST_W    = 11.0f;
constexpr float OPT_MAZE_DIST_OVERALL    = 0.7f;
constexpr float OPT_MAZE_DENSITY_START   = 0.0f;
constexpr float OPT_MAZE_DENSITY_END     = 0.0f;
constexpr int   OPT_MAZE_RECLAIM_MARGIN  = 1;
constexpr bool  OPT_MAZE_TIME_FILTER     = false;
constexpr int   OPT_MAZE_TOTAL_TICKS     = 1500;
constexpr int   OPT_MAZE_GREEDY          = 0;
constexpr int   OPT_MAZE_LOOKAHEAD       = 1;
constexpr float OPT_MAZE_LOOKAHEAD_W     = 0.3f;
constexpr float OPT_MAZE_CONGESTION_W    = 0.0f;
constexpr int   OPT_MAZE_DYNAMIC_FW      = 0;
constexpr float OPT_MAZE_FW_EARLY        = 6.0f;
constexpr float OPT_MAZE_FW_LATE         = 15.0f;
constexpr int   OPT_MAZE_LOOKAHEAD_TOPK  = 1;

constexpr bool  OPT_MAZE_ASSIGN_FLOW        = false;
constexpr float OPT_MAZE_ASSIGN_REVERSE_W   = 4.0f;
constexpr float OPT_MAZE_ASSIGN_ENTRY_W     = 1.5f;
constexpr float OPT_MAZE_ASSIGN_LANE_W      = 2.0f;
constexpr float OPT_MAZE_ASSIGN_ALIGN_BONUS = 0.0f;

constexpr bool  OPT_MAZE_PLANNER_HIGHWAY      = true;
constexpr bool  OPT_MAZE_PLANNER_SPLIT        = true;
constexpr bool  OPT_MAZE_PLANNER_TIDAL        = true;
constexpr bool  OPT_MAZE_PLANNER_GUIDE        = false;
constexpr bool  OPT_MAZE_PLANNER_LANE_AWARE   = false;
constexpr bool  OPT_MAZE_PLANNER_INTENT_PRIORITY = false;
constexpr bool  OPT_MAZE_PLANNER_INTENT_ENTRY    = true;
constexpr bool  OPT_MAZE_PLANNER_INTENT_REVERSE  = true;
constexpr bool  OPT_MAZE_PLANNER_INTENT_CORRIDOR_ONLY = false;
constexpr bool  OPT_MAZE_PLANNER_ENTRY_ARBITER = false;
constexpr bool  OPT_MAZE_PLANNER_ENTRY_DEMOTE  = true;
constexpr bool  OPT_MAZE_PLANNER_ENTRY_WINNER  = true;
constexpr bool  OPT_MAZE_PLANNER_TIDAL_MAJORITY = false;
constexpr int   OPT_MAZE_PLANNER_HIGHWAY_PENALTY = 240;
constexpr int   OPT_MAZE_PLANNER_SPLIT_PENALTY   = 240;
constexpr int   OPT_MAZE_PLANNER_TIDAL_PERIOD    = 80;
constexpr float OPT_MAZE_PLANNER_TIDAL_DELTA     = 20.0f;
constexpr int   OPT_MAZE_PLANNER_LANE_FLOW_PENALTY = 5;
constexpr int   OPT_MAZE_PLANNER_LANE_HEADON_PENALTY = 35;
constexpr int   OPT_MAZE_PLANNER_LANE_REVERSE_PENALTY = 20;
constexpr int   OPT_MAZE_PLANNER_LANE_WAIT_PENALTY = 12;
constexpr int   OPT_MAZE_PLANNER_LANE_REFUGE_BONUS = 12;
constexpr int   OPT_MAZE_PLANNER_INTENT_ENTRY_MIN = 2;
constexpr float OPT_MAZE_PLANNER_INTENT_DELTA = 8.0f;
constexpr int   OPT_MAZE_PLANNER_ENTRY_MIN = 2;
constexpr float OPT_MAZE_PLANNER_ENTRY_DELTA = 6.0f;

// ═══════════════════════════════════════════════════════════════════
// IronZone 专属（SCHEDULE_MODE=10）
// 大图低密度物流调度：按粗 zone 控制本地候选，减少跨图追任务。
// ═══════════════════════════════════════════════════════════════════
constexpr int   IRON_ZONE_ROWS       = 19;
constexpr int   IRON_ZONE_COLS       = 18;
constexpr int   IRON_ASSIGN_VARIANT  = 0;     // 0=continuous, 1=wave, 2=zone quota, 3=finish dispersion, 4=dynamic wave, 5=start-distance gate, 6=distance dominant, 7=fair zone quota, 8=completion-cost wave, 9=completion fixed70, 10=completion fixed75, 11=completion ramp70-80, 12=start fixed80, 13=base fixed70, 14=legacy early fixed70, 15=balanced fixed70, 16=opportunity reserve fixed70
constexpr int   IRON_LOCAL_RADIUS    = 1;
constexpr int   IRON_MAX_RADIUS      = 1;
constexpr int   IRON_CANDIDATE_TOPK  = 500;
constexpr float IRON_DIST_WEIGHT     = 1.8f;
constexpr float IRON_TASK_LEN_WEIGHT = 1.25f;
constexpr float IRON_CROSS_ZONE_W    = 60.0f;
constexpr float IRON_ENDPOINT_W      = 0.0f;
constexpr float IRON_AGE_WEIGHT      = 0.0f;
constexpr float IRON_ASSIGN_RATIO    = 1.0f;
constexpr int   IRON_ASSIGN_MAX_DIST = 0;   // agent→task1距离过滤(0=关闭)
constexpr float IRON_BALANCE_W       = 4.0f;
constexpr float IRON_MIGRATION_W     = 2.0f;
constexpr int   IRON_BALANCE_CAP     = 32;
constexpr int   IRON_SHORT_TASK_TOPK = 512;
constexpr int   IRON_STARVED_TASK_TOPK = 128;
constexpr int   IRON_STARVED_ZONE_THRESHOLD = 10;
constexpr int   IRON_LONG_TASK_TH    = 1250;
constexpr int   IRON_STATS_INTERVAL  = 10;
constexpr int   IRON_EARLY_PHASE_TICKS = 6000;
constexpr float IRON_EARLY_DIST_W      = 0.60f;
constexpr float IRON_EARLY_CROSS_W     = 24.0f;
constexpr float IRON_EARLY_LONG_W      = 0.20f;
constexpr bool  IRON_AGGRESSIVE_ASSIGN = true;
constexpr int   IRON_AGGRESSIVE_ITERS  = 10;
constexpr int   IRON_AGGRESSIVE_PROBE  = 64;
constexpr float IRON_AGGRESSIVE_REPLACE_MARGIN = 1.0f;
constexpr float IRON_OPPORTUNITY_RATIO = 0.03f;
constexpr int   IRON_OPPORTUNITY_DIST  = 50;
constexpr int   IRON_OPPORTUNITY_COMPLETION = 1350;
constexpr int   IRON_OPPORTUNITY_CROSS = 0;

// ═══════════════════════════════════════════════════════════════════
// Iron highway direction constraints（manual annotation based）
// ═══════════════════════════════════════════════════════════════════
constexpr int   IRON_HIGHWAY_ENABLED       = 1;       // 0=关闭, 1=开启
constexpr int   IRON_HIGHWAY_FORBID_COST   = 100000;  // 禁止方向A*代价
constexpr int   IRON_HIGHWAY_EXIT_COST     = 300;     // 出口区域禁止方向代价(比主高速小,方便进出)
constexpr float IRON_HIGHWAY_PIBT_BOOST    = 500.0f;  // highway上agent移动优先级提升(0=关)
constexpr int   IRON_HIGHWAY_SOUTH_PENALTY  = 35;      // 南口(1019)西行额外代价(平衡出口流量,0=关)
constexpr int   IRON_HIGHWAY_PIBT_PENALTY  = 0;       // PIBT禁止方向策略: 0=跳过(硬拦), >0=加入但代价加成(防死锁)
constexpr int   IRON_HIGHWAY_DEBUG         = 0;       // 0=静默, 1=打印
// ═══════════════════════════════════════════════════════════════════
constexpr int   IRON_LANE_ENABLED      = 0;       // 0=关闭, 1=开启红绿灯
constexpr int   IRON_LANE_GUARD_DIST   = 20;      // 统计agent的曼哈顿距离阈值
constexpr int   IRON_LANE_CORE_FLIP_TH = 10;      // core人数>此值冻锁不翻
constexpr float IRON_LANE_IDLE_BONUS   = 0.5f;    // 空闲agent引导加分（score越小越好）
constexpr int   IRON_LANE_FLOW_RATIO_FLIP = 2.5;    // 拦/放流比>=此值强制翻锁
constexpr int   IRON_LANE_VOTE_PERIOD  = 80;      // 每N tick重新投票
constexpr int   IRON_LANE_DEBUG       = 0;       // 0=关闭所有红绿灯打印, 1=打开debug打印
constexpr int   IRON_LANE_LRU_SORT    = 1;       // 0=顺序循环算guide path, 1=LRU排序
constexpr int   IRON_LANE_OUTSIDE_VOTE = 1;      // 0=全员投票(lane内+外来), 1=只有lane外agent投票
constexpr int   IRON_LANE_EXIT_BUFFER_RADIUS = 3;  // lane出口缓冲区半径(格)
constexpr float IRON_LANE_EXIT_PRIORITY    = 1000.0f; // lane内出去的agent排序+1000
constexpr float IRON_LANE_BUFFER_PENALTY   = 1000.0f; // buffer内挡路agent排序-1000

// ═══════════════════════════════════════════════════════════════════
// Iron future flow（64×64 zone 级别流量预测）
// ═══════════════════════════════════════════════════════════════════
constexpr float IRON_FUTURE_FLOW_W        = 1.0f;   // 加流权重(所有段等权)
constexpr float IRON_FUTURE_FLOW_K        = 1.0f;   // 流量基数乘子
constexpr float IRON_FUTURE_FLOW_DECAY    = 0.01f;  // 1/(1+d*decay)
constexpr int   IRON_FUTURE_FLOW_ZONE_ROWS = 64;    // ff内部zone网格行数
constexpr int   IRON_FUTURE_FLOW_ZONE_COLS = 64;    // ff内部zone网格列数
constexpr float IRON_FUTURE_FLOW_LOCK_TH  = 35.0f;  // 方向锁阈值(0=关): 逆流流量>=此值则拦
constexpr float IRON_FUTURE_FLOW_CONGEST_CAP = 300.0f; // 拥塞上限: zone总流>=此值则穿过的任务无论方向一律拦
// ═══════════════════════════════════════════════════════════════════
// ---- Planner: flow_scale覆盖(仅room) ----
constexpr int   OPT_ROOM_FLOW_SCALE = 0;           // 0=关闭(使用全局FLOW_DECAY_SCALE)
// ---- Planner: 后期强制replan ----
constexpr int   OPT_ROOM_LATE_FORCE_REPLAN = 0;    // 0=关闭(不强制replan)
constexpr int   OPT_ROOM_REPLAN_START_TICK = 2000;  // replan生效起始tick(默认2000=后期)

// ═══════════════════════════════════════════════════════════════════
// RoomTidalPlanner (Mode 7) 可调参数
// ═══════════════════════════════════════════════════════════════════
constexpr int   OPT_TIDAL_PERIOD = 80;              // 潮汐投票周期(tick)
constexpr int   OPT_TIDAL_GUIDE_BONUS = 30;         // 跟随引导路径下一步奖励
constexpr int   OPT_TIDAL_BACKTRACK_PENALTY = 45;   // 回退惩罚
constexpr int   OPT_TIDAL_BACKTRACK_STEP = 8;       // 回退每步额外惩罚
constexpr int   OPT_TIDAL_WAIT_PENALTY = 12;        // 原地等待惩罚
constexpr int   OPT_TIDAL_FORWARD_SKIP = 4;         // 跳步前进惩罚
constexpr int   OPT_TIDAL_OFF_PATH = 8;             // 偏离路径惩罚
constexpr int   OPT_TIDAL_PRIORITY_DELTA = 1000;    // 潮汐方向优先级差值
constexpr int   OPT_TIDAL_REFUGE_PENALTY = 120;     // 避难区入口惩罚
constexpr int   OPT_TIDAL_PIBT_RETRY_LIMIT = 1;    // PIBT重试次数(1=不重试)

// ---- Executor: yield相关策略 (R57/R58测试中) ----
// yield_highway: R57 avg=731.7(最优), R58 run3出现超时bug(待修复)
// pibt_yield: R58测试中
constexpr int   OPT_ROOM_EXEC_YIELD_HIGHWAY = 0;     // 通道让路(0=关闭,1=开启): R57验证有效但有deadlock风险
constexpr int   OPT_ROOM_EXEC_YIELD_THRESH = 3;      // yield触发阈值(连续STOP次数)
constexpr int   OPT_ROOM_PIBT_YIELD = 0;              // PIBT中STOP agent降优先级(0=关闭,1=开启)
constexpr int   OPT_ROOM_PIBT_YIELD_PENALTY = 500;    // STOP agent优先级减量

// ---- R59: Schedule层新策略 ----
// late_fw_phase: 后期fw调整(0=关闭,1=开启) — 后期优先近task冲分
constexpr int   OPT_ROOM_LATE_FW_PHASE = 0;           // 0=关闭
constexpr float OPT_ROOM_LATE_FW_VALUE = 3.0f;        // 后期fw值
constexpr int   OPT_ROOM_LATE_FW_START_TICK = 2000;   // 后期开始tick
// progress_boost: 已完成task多的agent分配优先级提升(0=关闭)
constexpr int   OPT_ROOM_PROGRESS_BOOST = 0;
constexpr float OPT_ROOM_PROGRESS_BOOST_W = 0.5f;     // 每个已完成task的cost减量

// ---- R59: Planner层新策略 ----
// corridor_forward: 通道中PIBT引导偏好前进方向(0=关闭)
constexpr int   OPT_TIDAL_CORRIDOR_FORWARD = 0;       // 0=关闭
constexpr int   OPT_TIDAL_CORRIDOR_FORWARD_BONUS = 15; // 通道前进方向heuristic奖励

// ═══════════════════════════════════════════════════════════════════
// FlowDecay 专属（PLANNER_MODE=1）
// ═══════════════════════════════════════════════════════════════════
constexpr float FLOW_DECAY       = 0.9f;      // 边流量衰减系数
constexpr int   FLOW_DECAY_SCALE = 100;        // 放大因子
constexpr bool  WARE_DIRECT      = false;      // Warehouse 通道定向策略开关
constexpr bool  OPT_PIBT_FORWARD_BREAK_TIE = true; // PIBT候选heuristic相等时优先FW
constexpr int   GUIDE_FLOW_OPPOSITE_W = 1;    // 逆流惩罚倍数(1=原版行为)
constexpr int   GUIDE_FLOW_SAME_W     = 3;    // 顺流奖励(0=关闭, >0时A*选人多方向)

// ═══════════════════════════════════════════════════════════════════
// 距离矩阵预计算（共用）
// ═══════════════════════════════════════════════════════════════════
constexpr bool  PRECOMPUTE_HEURISTIC = true;
constexpr int   MAX_PRECOMPUTE_CELLS = 100000000;
constexpr bool  TURN_AWARE_HEURISTIC = false;

// ═══════════════════════════════════════════════════════════════════
// Executor（Origin 逻辑在 src/Executor.cpp 中）
// ═══════════════════════════════════════════════════════════════════
constexpr int   ORIGIN_APPEND_STEPS = 0;       // 0=自动, >0=固定步数
constexpr bool  CONGESTION_ADAPTIVE_APPEND = false;

} // namespace optsvr
