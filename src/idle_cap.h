// idle_cap.h — 均匀空载容量控制
//
// 原理：ORZ 地图按列分 8 段，每段独立限制活跃 agent 比例
//       + shuffle 确保段内分配公平
// 效果：全图均匀空跑，避免热点垄断

#pragma once

#define IDLE_NUM_BUCKETS  8   // 桶数（ORZ 东西走廊列分 8 段）
#define IDLE_BOOT_TICKS 0   // 前 N tick 用小比例控流

namespace IdleCap {

// 计算每个桶的剩余名额
// budgets[bucket] = 该桶本轮还能分配几个任务
inline void compute_budgets(SharedEnvironment* env, int budgets[IDLE_NUM_BUCKETS]) {
    int cols_per = env->cols / IDLE_NUM_BUCKETS + 1;
    int active[IDLE_NUM_BUCKETS] = {0};
    int total[IDLE_NUM_BUCKETS] = {0};
    
    // 开机期用小比例控流，之后恢复正常
    float ratio = (env->curr_timestep < IDLE_BOOT_TICKS) ? 0.4f : IDLE_RATIO;

    for (int a = 0; a < env->num_of_agents; a++) {
        int loc = env->curr_states[a].location;
        int col = loc % env->cols;
        int b = col / cols_per;
        if (b >= IDLE_NUM_BUCKETS) b = IDLE_NUM_BUCKETS - 1;
        total[b]++;
        if (env->curr_task_schedule[a] >= 0) active[b]++;
    }

    for (int b = 0; b < IDLE_NUM_BUCKETS; b++) {
        int limit = std::max(8, (int)(total[b] * ratio));
        budgets[b] = std::max(0, limit - active[b]);
    }
}

// 获取 agent 所在的桶
inline int get_bucket(SharedEnvironment* env, int agent_id) {
    int col = env->curr_states[agent_id].location % env->cols;
    int cols_per = env->cols / IDLE_NUM_BUCKETS + 1;
    int b = col / cols_per;
    return (b >= IDLE_NUM_BUCKETS) ? (IDLE_NUM_BUCKETS - 1) : b;
}

} // namespace IdleCap
