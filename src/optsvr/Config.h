// optsvr/Config.h — 编译期默认参数
#pragma once

namespace optsvr {

// ═══════════════════════════════════════════════════════════════════
// 全局控制
// ═══════════════════════════════════════════════════════════════════
constexpr bool  SKIP_TEST       = true;   // 默认跳过，MapTuner 按地图类型设为 false
constexpr int   DEBUG_LEVEL     = 0;       // 0=静默, 1=概要, 2=细节
constexpr int   NUM_THREADS     = 0;       // 0=自动检测
constexpr bool  OPT_TIMING_TRACE = false;  // 每个规划周期输出 schedule/guide/pibt 耗时

// ═══════════════════════════════════════════════════════════════════
// 模块选择
// ═══════════════════════════════════════════════════════════════════
// Scheduler: 0=Default, 1=GreedyHeap, 2=DHSchedule, 3=GameOpt, 4=TaskMatch, 5=DHFULLSchedule, 6=OptMazeSchedule, 7=OptRoom, 8=GameBalanceOpt, 9=MazeOptScheduler(MAZEOpt默认schedule), 10=IronZone, 11=RandomRouteLoad
constexpr int   SCHEDULE_MODE   = 2;
// Executor:  0=Default(官方), 1=Origin(MCP跟随链), 2=Hybrid, 3=DHExecutor(TPG+临时穿越), 4=RoomOptExecutor(连续STOP截断), 5=RoomExecutor(Room专用TPG), 6=GameExecutor(Game专用:对流检测+截断), 7=IncrementalTpgExecutor(物理重建增量TPG), 8=ConservativeIncrementalTpgExecutor(9f9bf33保守版)
constexpr int   EXECUTOR_MODE   = 3;
// Planner:   0=Default, 1=FlowDecay, 2=GameOpt, 3=TurnAware, 4=EPIBT(轻量turn-cost), 5=IdleYield(空闲让路), 6=RoomPlanner(Room专用PIBT), 7=RoomTidalPlanner(Room潮汐门控), 8=MazeTidalPlanner(MAZEOpt默认planner封装), 10=RandomFlowEPIBT, 11=TestUsefulFlowDecay, 12=IronFlowDecay
constexpr int   PLANNER_MODE    = 11;


// ═══════════════════════════════════════════════════════════════════
// GreedyHeap 专属（SCHEDULE_MODE=1）
// ═══════════════════════════════════════════════════════════════════
constexpr int   HEAP_REBUILD_PCT    = 45;      // rebuild 占时间预算百分比
constexpr int   HEAP_LNS_PCT        = 10;      // LNS 优化百分比
constexpr int   HEAP_SORT_K         = 500;     // 排序前 K 个候选
constexpr float HEAP_MAX_ASSIGN     = 1.0f;    // 分配比例（0.0~1.0, 1.0=全部分配, 0.4=只分配40%的agent）
constexpr int   HEAP_DIST_DECAY_INTERVAL_STEPS = 0;   // 0=关闭；N步后一次性衰减dist_to_start权重
constexpr float HEAP_DIST_DECAY_FACTOR = 1.0f;         // 触发后乘一次的系数

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
// RandomRouteLoad 专属（SCHEDULE_MODE=11）
// 沿途 future-load 软惩罚。默认权重为 0，便于显式 ablation。
// ═══════════════════════════════════════════════════════════════════
constexpr float RANDOM_ROUTE_LOAD_WEIGHT      = 0.0f;  // 任务 route-load 额外代价权重
constexpr int   RANDOM_ROUTE_LOAD_BATCH_SIZE  = 64;    // 每批确认的匹配数
constexpr int   RANDOM_ROUTE_LOAD_GRID_CELL   = 4;     // random 64x64 -> 16x16 粗网格
constexpr int   RANDOM_ROUTE_LOAD_TIGHT_SLACK = 4;     // 近似最短路走廊的满权 extra
constexpr int   RANDOM_ROUTE_LOAD_LOOSE_SLACK = 12;    // 走廊外沿 extra，线性衰减到 0
constexpr float RANDOM_ROUTE_LOAD_DECAY       = 0.02f; // 后续段载入 future-load 的衰减
constexpr bool  RANDOM_ROUTE_LOAD_LENGTH_WEIGHTED_WRITE = true;  // future-load 写入乘以段长
constexpr float RANDOM_ROUTE_LOAD_WRITE_LENGTH_SCALE = 0.01f;     // 段长写入缩放，保持惩罚量级可控
constexpr bool  RANDOM_ROUTE_LOAD_DYNAMIC_SHORTEST = false;       // 每个batch按当前负载重算zone级最短路
constexpr float RANDOM_ROUTE_LOAD_DYNAMIC_PATH_WEIGHT = 0.1f;      // 动态最短路中 route-load 权重乘子

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

// ---- IDEA5: Task Age排序优先 ----
// free_tasks按等待时间排序，优先分配久等task
constexpr bool  GAME_AGE_SORT         = false;   // true=启用age排序
constexpr float GAME_AGE_SORT_WEIGHT  = 0.5f;    // score加成: + weight * age/total_steps

// ═══════════════════════════════════════════════════════════════════
// IronZone 专属（SCHEDULE_MODE=10）
// 大图低密度物流调度：按粗 zone 控制本地候选，减少跨图追任务。
// ═══════════════════════════════════════════════════════════════════
constexpr int   IRON_ZONE_ROWS       = 19;
constexpr int   IRON_ZONE_COLS       = 18;
constexpr int   IRON_MAX_RADIUS      = 1;
constexpr float IRON_DIST_WEIGHT     = 1.8f;
constexpr float IRON_TASK_LEN_WEIGHT = 1.25f;
constexpr int   IRON_LONG_TASK_TH    = 1250;
constexpr int   IRON_STATS_INTERVAL  = 10;

// ═══════════════════════════════════════════════════════════════════
// Iron highway direction constraints（manual annotation based）
// ═══════════════════════════════════════════════════════════════════
constexpr int   IRON_HIGHWAY_ENABLED       = 1;       // 0=关闭, 1=开启
constexpr int   IRON_HIGHWAY_FORBID_COST   = 100000;  // 禁止方向A*代价
constexpr int   IRON_HIGHWAY_EXIT_COST     = 300;     // 出口区域禁止方向代价(比主高速小,方便进出)
constexpr float IRON_HIGHWAY_PIBT_BOOST    = 1000.0f; // highway上agent移动优先级提升(0=关)
constexpr int   IRON_HIGHWAY_STALL_BOOST   = 100;     // 卡住每tick额外优先级(0=关)
constexpr int   IRON_HIGHWAY_STALL_MAX     = 5000;    // stall叠加上限
constexpr int   IRON_HIGHWAY_SOUTH_PENALTY = 15;      // 南口(1019)西行额外代价(平衡出口流量,0=关)
constexpr int   IRON_HIGHWAY_PIBT_PENALTY  = 0;       // PIBT禁止方向策略: 0=跳过(硬拦), >0=加入但代价加成
constexpr int   IRON_HIGHWAY_DEBUG         = 0;       // 0=静默, 1=打印
constexpr int   IRON_HIGHWAY_CLEAR_DECISION= 1;       // 每tick清空decision_残留(防2000tick卡死,0=关)
constexpr int   IRON_HIGHWAY_FORCE_MOVE    = 1;       // 单方向highway cell强制移动(0=关,1=forward真空时跳过occupied)
constexpr int   IRON_HIGHWAY_GUIDE_PENALTY = 100000;  // 入口引导惩罚(0=关): 93个guide cell错误方向代价
constexpr int   IRON_CORE_TASK_FILTER      = 1;       // 禁止分配任务到core非highway格(0=关,1=开启)
constexpr int   IRON_HIGHWAY_FREEZE_ASSIGN = 1;       // 禁止highway上的agent分配新任务(0=关,1=在highway上不分配)
constexpr int   IRON_HIGHWAY_SKIP_TASK     = 1;       // 过滤highway+guide上的任务(0=关,1=开启)

// Highway 非highway惩罚（让A*偏好走高速）
constexpr int   IRON_HIGHWAY_NON_HIGHWAY_PENALTY = 5;   // 非highway格子额外代价
constexpr int   IRON_HIGHWAY_ROW_PENALTY   = 250;   // passage row 逆流惩罚
constexpr int   IRON_HIGHWAY_COL_PENALTY   = 150;   // passage col 逆流惩罚

// IDEA/IRON lane traffic-light planner branch.
constexpr int   IRON_LANE_ENABLED      = 0;       // 0=关闭, 1=开启红绿灯
constexpr int   IRON_LANE_GUARD_DIST   = 20;      // 统计agent的曼哈顿距离阈值
constexpr int   IRON_LANE_CORE_FLIP_TH = 10;      // core人数>此值冻锁不翻
constexpr float IRON_LANE_IDLE_BONUS   = 0.5f;    // 保留原参数
constexpr int   IRON_LANE_FLOW_RATIO_FLIP = 2;    // 原 IDEA/IRON 写作 2.5，int 实际按 2 使用
constexpr int   IRON_LANE_VOTE_PERIOD  = 80;      // 每N tick重新投票
constexpr int   IRON_LANE_DEBUG        = 0;       // 0=关闭所有红绿灯打印, 1=打开debug打印
constexpr int   IRON_LANE_LRU_SORT     = 1;       // 0=顺序循环算guide path, 1=LRU排序
constexpr int   IRON_LANE_OUTSIDE_VOTE = 1;       // 0=全员投票, 1=只有lane外agent投票
constexpr int   IRON_LANE_EXIT_BUFFER_RADIUS = 3; // lane出口缓冲区半径
constexpr float IRON_LANE_EXIT_PRIORITY = 1000.0f;
constexpr float IRON_LANE_BUFFER_PENALTY = 1000.0f;

// ═══════════════════════════════════════════════════════════════════
// Iron future flow（64×64 zone 级别流量预测）
// ═══════════════════════════════════════════════════════════════════
constexpr float IRON_FUTURE_FLOW_W        = 1.0f;   // 加流权重(所有段等权)
constexpr float IRON_FUTURE_FLOW_K        = 1.0f;   // 流量基数乘子
constexpr float IRON_FUTURE_FLOW_DECAY    = 0.01f;  // 1/(1+d*decay)
constexpr int   IRON_FUTURE_FLOW_ZONE_ROWS = 64;    // ff内部zone网格行数
constexpr int   IRON_FUTURE_FLOW_ZONE_COLS = 64;    // ff内部zone网格列数
constexpr float IRON_FUTURE_FLOW_LOCK_TH  = 50.0f;  // 方向锁阈值(0=关闭逆流锁)
constexpr float IRON_FUTURE_FLOW_CONGEST_CAP = 300.0f; // 拥塞上限: zone总流>=此值则拦
constexpr float IRON_FUTURE_FLOW_IDLE_WEIGHT = 200.0f;  // idle relocate热度权重

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
// RoomPlanner / RoomExecutor 优化参数
// 默认值全部为"关闭"状态，需通过MapTuner或环境变量显式开启
// 关闭时: RoomExecutor(5)≡DHExecutor(3), RoomPlanner(6)≡OptPlanner(1)
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

// ═══════════════════════════════════════════════════════════════════
// FlowDecay 专属（PLANNER_MODE=1）
// ═══════════════════════════════════════════════════════════════════
constexpr float FLOW_DECAY       = 0.9f;      // 边流量衰减系数
constexpr float FLOW_DECAY_OPPOSITE = 0.99f;  // IDEA/IRON 逆流衰减系数
constexpr int   FLOW_DECAY_SCALE = 100;        // 放大因子
constexpr bool  OPT_PIBT_FORWARD_BREAK_TIE = true;
constexpr int   GUIDE_FLOW_OPPOSITE_W = 1;
constexpr int   GUIDE_FLOW_SAME_W = 1;        // IDEA/IRON Config.h 写作 int 1.3，实际等价为 1
// Fullfill highway
constexpr bool  FULLFILL_HIGHWAY_FORCE_MOVE = false; // PIBT单方向格强制驱离（warehouse窄道多易死锁，默认关）
constexpr bool  WARE_DIRECT      = false;      // Warehouse 通道定向策略开关
constexpr bool  DUMMY_GOAL_HOME  = false;      // idle agent 回原始起点（Entry.cpp 机制）
constexpr int   OPT_FLOW_TURN_PENALTY = 0;      // Flow guide path中每次转弯额外cost
constexpr int   OPT_FLOW_START_TURN_PENALTY = 2; // Flow guide第一步相对当前朝向的转弯cost
constexpr float OPT_FLOW_TURN_TIE = 0.1f;       // Flow guide path同分时的转弯tie-break
constexpr int   OPT_FLOW_VERTEX_COST_MODE = 4;  // 0=原版base,1=二次,2=指数,3=线性+指数,4=线性+二次尾部
constexpr int   OPT_FLOW_VERTEX_COST_THRESHOLD = 0;
constexpr int   OPT_FLOW_VERTEX_COST_WEIGHT = 45;

constexpr int   OPT_FLOW_VERTEX_COST_CAP = 100000;
constexpr float OPT_FLOW_VERTEX_COST_EXP_BASE = 3.0f;
constexpr int   OPT_FLOW_VERTEX_COST_LINEAR_END = 4;
constexpr float OPT_FLOW_VERTEX_SENS_DECAY = 0.92f;
constexpr int   OPT_FLOW_VERTEX_BASE_WEIGHT = 80;
constexpr bool  OPT_P1_FLOW_VERTEX_PENALTY_ENABLED = true; // P1: one switch for all added vertex-flow penalties
constexpr bool  OPT_FLOW_DYNAMIC_TIME_ENABLED = false; // FlowDecay: adapt PIBT reserve from previous actual time
constexpr int   OPT_FLOW_DYNAMIC_TIME_MARGIN_MS = 50;
constexpr int   OPT_EPIBT_FLOW_VERTEX_MODE = 4;
constexpr int   OPT_EPIBT_FLOW_VERTEX_THRESHOLD = 4;
constexpr float OPT_EPIBT_FLOW_VERTEX_WEIGHT = 1.0f;
constexpr int   OPT_EPIBT_FLOW_VERTEX_CAP = 1000;
constexpr float OPT_EPIBT_FLOW_VERTEX_EXP_BASE = 0.5f;
constexpr int   OPT_EPIBT_FLOW_VERTEX_LINEAR_END = 4;
constexpr int   OPT_EPIBT_FLOW_VERTEX_BASE_WEIGHT = 0;
constexpr bool  OPT_EPIBT_PARALLEL_GUIDE = false; // P10: parallel guide-path search

constexpr bool  OPT_DIST2PATH_PREWARM_ENABLED = false; // FlowDecay parallel guide-path distance cache prewarm
constexpr int   OPT_DIST2PATH_PREWARM_RADIUS = 3;   // FlowDecay PIBT guide-path distance cache prewarm radius
constexpr int   OPT_DIST2PATH_PREWARM_THREADS = 0;  // 0=hardware_concurrency-1
constexpr int   OPT_DIST2PATH_PREWARM_MIN_LEFT_MS = 50; // stop enqueueing near flow deadline

// PIBT cell-level historical occupancy penalty (currently wired for TestUsefulFlowDecay).
constexpr bool  OPT_PIBT_HISTORY_CONGESTION_ENABLED = false;
constexpr int   OPT_PIBT_HISTORY_WINDOW = 500;
constexpr float OPT_PIBT_HISTORY_FREE_THRESHOLD = 0.10f;
constexpr float OPT_PIBT_HISTORY_QUAD_THRESHOLD = 0.35f;
constexpr float OPT_PIBT_HISTORY_LINEAR_WEIGHT = 0.5f;
constexpr float OPT_PIBT_HISTORY_QUAD_WEIGHT = 2.0f;
constexpr float OPT_PIBT_HISTORY_CAP = 3.0f;
constexpr float OPT_PIBT_HISTORY_QUAD_POWER = 2.0f;
constexpr int   OPT_PIBT_HISTORY_SORT_MODE = 0; // 0=additive,1=tie-break,2=additive within best-base band
constexpr float OPT_PIBT_HISTORY_TIE_BASE_EPS = 0.0f;
constexpr bool  OPT_PIBT_HISTORY_APPLY_TO_WAIT = true;
constexpr int   OPT_PIBT_HISTORY_GOAL_EXEMPT_DIST = -1; // <0 disables; skip penalty when base h <= value
constexpr bool  OPT_PIBT_HISTORY_STATS_ENABLED = true;
constexpr int   OPT_PIBT_HISTORY_STATS_INTERVAL = 500;

// ═══════════════════════════════════════════════════════════════════
// 距离矩阵预计算（共用）
// ═══════════════════════════════════════════════════════════════════
constexpr bool  PRECOMPUTE_HEURISTIC = false;
constexpr int   MAX_PRECOMPUTE_CELLS = 100000000;
constexpr bool  TURN_AWARE_HEURISTIC = false;

// ═══════════════════════════════════════════════════════════════════
// Executor（Origin 逻辑在 src/Executor.cpp 中）
// ═══════════════════════════════════════════════════════════════════
constexpr int   ORIGIN_APPEND_STEPS = 0;       // 0=自动, >0=固定步数
constexpr bool  CONGESTION_ADAPTIVE_APPEND = false;

} // namespace optsvr
