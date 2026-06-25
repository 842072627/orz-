#ifndef EXECUTOR_HEAT_LOCK_H
#define EXECUTOR_HEAT_LOCK_H

#include <unordered_map>
#include <array>
#include <vector>
#include "SharedEnv.h"
#include "ActionModel.h"

// Executor 方向锁：N步预判 + 即时流量 + 总流量阈值
constexpr int HEAT_LOCK_PREDICT_STEPS = 5;       // 预判步数
constexpr float HEAT_LOCK_THRESHOLD = 5.0f;       // 更高阈值

// 方向 [E=0, S=1, W=2, N=3]
constexpr int DIR_OPP[4] = {2, 3, 0, 1};

// 每 tick 数据（全局）
extern std::unordered_map<int, std::array<float, 4>> dir_cnt;
extern std::unordered_map<int, int> lock_dir;

void heat_lock_init();
void heat_lock_update(SharedEnvironment* env);
bool heat_lock_check(int agent_id, int curr_loc, int curr_orient, SharedEnvironment* env);

#endif
