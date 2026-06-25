#ifndef EXECUTOR_HEAT_LOCK_H
#define EXECUTOR_HEAT_LOCK_H

#include <unordered_map>
#include <array>
#include <vector>
#include "SharedEnv.h"
#include "ActionModel.h"

// Executor 方向锁：全量预判 + 热度衰减 + 备份豁免
// 
// 配置参数（可调）
constexpr float HEAT_LOCK_STEP_DECAY = 0.98f;    // 步数衰减
constexpr float HEAT_LOCK_HEAT_DECAY = 0.9f;     // 时间衰减
constexpr float HEAT_LOCK_THRESHOLD = 31.0f;     // 锁热阈值

// 方向 [E=0, S=1, W=2, N=3]
constexpr int DIR_OPP[4] = {2, 3, 0, 1};         // 对向映射

// 每 tick 数据（全局）
extern std::unordered_map<int, std::array<float, 4>> heat_map;       // [loc][dir] 累积热度
extern std::unordered_map<int, std::array<float, 4>> dir_cnt;        // [loc][dir] 原始计数
extern std::unordered_map<int, int> lock_dir;                        // [loc] 锁定方向, -1=未锁
extern std::unordered_map<int, int> backup_count;                    // [loc] 有多少 agent 会经过

// 初始化
void heat_lock_init();

// 每 tick 更新：预判全路径 → 热度衰减 → 锁决策
// env: SharedEnvironment，提供 staged_actions, system_states, map, cols, rows
void heat_lock_update(SharedEnvironment* env);

// 方向锁检查：agent 从 curr_loc 以 orient 方向移动时是否被拦
// 返回 true = 被锁拦（STOP），false = 放行
bool heat_lock_check(int agent_id, int curr_loc, int curr_orient, SharedEnvironment* env);

#endif
