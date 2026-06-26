#ifndef EXECUTOR_HEAT_LOCK_H
#define EXECUTOR_HEAT_LOCK_H

#include <unordered_map>
#include <array>
#include <vector>
#include "SharedEnv.h"
#include "ActionModel.h"

// 方向 [E=0, S=1, W=2, N=3]
constexpr int DIR_OPP[4] = {2, 3, 0, 1};

// 每 tick 数据（全局）
extern std::unordered_map<int, std::array<float, 4>> dir_cnt;
extern std::unordered_map<int, int> lock_dir;

void heat_lock_init();
void heat_lock_update(SharedEnvironment* env, const std::vector<std::vector<Action>>& staged_actions);
// 实时occupancy方向锁：基于当前每格的agent数量，不依赖plan预测
void heat_lock_update_realtime(SharedEnvironment* env);
// 历史流量EMA方向锁：基于agent实际走过的方向累积指数移动平均
void heat_lock_update_historical(SharedEnvironment* env);
// EMA外部访问（诊断用）
extern std::unordered_map<int, std::array<float, 4>> flow_ema;
bool heat_lock_check(int agent_id, int curr_loc, int curr_orient, SharedEnvironment* env);

// 链式反应诊断：追踪特定热点区域的agent轨迹
void heat_lock_diag_hotspots(SharedEnvironment* env);

#endif
