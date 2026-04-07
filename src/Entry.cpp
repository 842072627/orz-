#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"

// ==================== Entry::initialize ====================
void Entry::initialize(int preprocess_time_limit)
{
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

// ==================== Entry::compute ====================
void Entry::compute(int time_limit, Plan & plan, std::vector<int> & proposed_schedule)
{
    scheduler->plan(time_limit, proposed_schedule);
    update_goal_locations(proposed_schedule);
    planner->plan(time_limit, plan);
}

// ==================== Entry::update_goal_locations ====================
// 把调度结果翻译成 planner 的目标位置
// 
// 设计要点：
// 1. 每次只暴露任务的【下一个 errand】给 planner
//    原因：机器人到达 errand 后系统自动 idx_next_loc++，
//    下一轮 planner 自然看到新目标，实现动态响应
//
// 2. 空闲机器人：保持 goal_locations 为空
//    planner 的 dummy_goals 机制会让空闲机器人原地等待
//    不做漂移（warehouse_large_5000 上漂移代价太高，容易崩溃）
//    这个设计是安全的，不会引入额外冲突
void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    env->curr_task_schedule = proposed_schedule;

    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
    }
}
