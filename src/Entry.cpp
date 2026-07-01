#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"
#include "RuntimeConfig.h"
#include "PerfStats.h"
#include "home_return.h"
#include <algorithm>
#include <chrono>

// The initialize function will be called by competition system at the preprocessing stage.
// Implement the initialize functions of the planner and scheduler to load or compute auxiliary data.
// Note that, this function runs untill preprocess_time_limit (in milliseconds) is reached.
// This is an offline step, after it completes then evaluation begins.
void Entry::initialize(int preprocess_time_limit)
{
    optsvr::perf().init();
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

//The compute function will be called by competition system on each timestep.
//It computes:
//  1. a schedule that specifies which agent complete which task.
//  2. a next action that specifies how each agent should move in the next timestep.
//NB: the parameter time_limit is specified in milliseconds.
void Entry::compute(int time_limit, Plan & plan, std::vector<int> & proposed_schedule)
{

    // 首轮 compute 时 task_pool 已填充，用 task/agent 比例做 skip_test 延迟判断
    // static bool first_compute = true;
    // if (first_compute) {
    //     first_compute = false;
    //     float task_ratio = static_cast<float>(env->task_pool.size())
    //                      / std::max(1, env->num_of_agents);
    //     if (task_ratio > 1.05) {
    //         optsvr::cfg().skip_test = true;
    //     }
    // }
    //optsvr::cfg().skip_test = false;

    // 跳过当前算例：不做任务分配和路径规划，所有 agent 保持静止
    if (optsvr::cfg().skip_test) {
        // 必须设置正确大小的空 plan，否则 Simulator 的 validate_staged_actions_prefix 校验会抛异常
        plan.actions.resize(env->num_of_agents);
        return;
    }

    using clk = std::chrono::steady_clock;
    auto t0 = clk::now();

    //call the task scheduler to assign tasks to agents
    scheduler->plan(time_limit,proposed_schedule);

    auto t1 = clk::now();

    //then update the first unfinished errand/location of tasks for planner reference
    update_goal_locations(proposed_schedule);

    // 回家机制：idle agent 如果在 interior+guide 之外且出生在 interior 内，派 fake task 回家
    // 就像做真实任务一样，到家或接到真实任务后自动恢复
    if (optsvr::cfg().dummy_goal_home && !g_home_start_locs.empty()) {
        const int cols = env->cols;
        for (int i = 0; i < env->num_of_agents; i++) {
            // 已有真实任务 → 跳过
            if (!env->goal_locations[i].empty()) continue;
            if (i >= (int)g_home_start_locs.size()) continue;
            int home = g_home_start_locs[i];
            if (home < 0) continue;
            int hr = home / cols, hc = home % cols;
            // 出生在外部 → 不回家
            if (!(hc > 7 && hc < 488 && hr > 7 && hr < 131)) continue;
            // 当前在内部(含 guide zone) → 不需回家
            int cur = env->curr_states[i].location;
            if (cur < 0) continue;
            int cr = cur / cols, cc = cur % cols;
            if (cc >= 6 && cc <= 489 && cr >= 6 && cr <= 132) continue;
            // idle + 出生在内部 + 当前在外部 → 派 fake task 回家
            env->goal_locations[i].push_back({home, 0});
            { FILE* f = fopen("C:/Users/Administrator/Desktop/test-studio-mapf/big-final-test/fullfill-data/home_debug.log","a");
              if(f){fprintf(f,"HOME_TRIGGER tick=%d agent=%d home=(%d,%d) cur=(%d,%d)\n",
                         env->curr_timestep,i,hr,hc,cr,cc);fclose(f);} }
        }
    }

    // idle agent relocation: move idle agents toward low-flow zones (DISABLED)
    //scheduler->idle_relocate(proposed_schedule);

    //call the planner to compute the actions
    planner->plan(time_limit,plan);

    auto t2 = clk::now();

    // 记录性能统计
    auto us = [](auto d){ return std::chrono::duration_cast<std::chrono::microseconds>(d).count(); };
    optsvr::perf().record(us(t1-t0), us(t2-t1), us(t2-t0));

}

// Set the next goal locations for each agent based on the proposed schedule
void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    // record the proposed schedule so that we can tell the planner
    env->curr_task_schedule = proposed_schedule;

    // The first unfinished errand/location of each task is the next goal for the assigned agent.
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
    }
    return;
}