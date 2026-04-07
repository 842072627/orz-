#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"

// ==================== Entry::initialize ====================
// 预处理阶段：先初始化 scheduler，再初始化 planner
// 顺序原因：scheduler 的 schedule_initialize 会预计算全局启发式表（Dijkstra），
//           这些表 planner 也会用到（get_h 函数依赖全局 heuristic table）
void Entry::initialize(int preprocess_time_limit)
{
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

// ==================== Entry::compute ====================
// 每个 planning episode 被调用一次
// 返回：proposed_schedule（哪个机器人做哪个任务）+ plan（每个机器人的一系列动作）
//
// 流程：调度 → 更新目标 → 规划
// 这三步的顺序很重要：必须先调度，planner 才知道每个机器人要去哪里
void Entry::compute(int time_limit, Plan & plan, std::vector<int> & proposed_schedule)
{
    // 第一步：任务调度 —— 把空闲机器人分配给未分配的任务
    scheduler->plan(time_limit, proposed_schedule);

    // 第二步：根据调度结果，更新每个机器人的目标位置
    // 这一步把 "agent→task" 映射转成 "agent→goal_location" 映射
    update_goal_locations(proposed_schedule);
    
    // 第三步：路径规划 —— 给定每个机器人的目标，计算无冲突的动作序列
    planner->plan(time_limit, plan);
}

// ==================== Entry::update_goal_locations ====================
// 核心：把调度结果翻译成 planner 能理解的目标位置
//
// 关键设计决策：
// 1. 每次只给 planner 暴露任务的【下一个 errand】（不是全部）
//    原因：机器人到达一个 errand 后，系统自动 idx_next_loc++，
//          下一轮 planner 自然会看到新目标。这样可以动态响应变化。
//
// 2. 空闲机器人的处理（改进！）
//    默认行为：空闲机器人没有目标 → planner 让它原地等待
//    改进行为：空闲机器人去最近的未分配任务位置 → 提前"靠近"工作区域
//    原因：来自冠军队的洞见——机器人提前移动到任务密集区域，
//          可以大幅减少正式分配后的行驶时间
//    代价：可能和有任务的机器人产生路径冲突（但 PIBT 会处理冲突）
//
// 3. 空闲机器人目标距离上限
//    如果最近的未分配任务太远（超过阈值），不值得移动
//    避免空闲机器人做无效长距离移动
void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    // 记录当前调度
    env->curr_task_schedule = proposed_schedule;

    // 第一遍：为已分配任务的机器人设置目标
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;  // 空闲机器人，留给第二遍处理

        // 取任务的下一个未完成 errand 作为目标
        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back(
            {env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed}
        );
    }

    // 第二遍：为空闲机器人设置"漂移目标"——去最近的未分配任务位置
    // 来自 RAPID 和 No Man Sky 的启发：让空闲机器人主动靠近工作区域
    // 
    // 为什么有效：
    // - 默认 planner 对空闲机器人设 dummy_goal=当前位置（原地不动）
    // - 如果机器人在角落，等它被分配任务时可能要走很远
    // - 让它提前往任务密集区移动，可以减少正式分配后的行驶时间
    //
    // 为什么安全：
    // - PIBT 会处理路径冲突（空闲机器人优先级低，会让路）
    // - 一旦机器人被分配任务，goal_locations 会被正式任务目标覆盖
    // - 如果没有未分配任务，机器人就原地等待（goal_locations 为空）
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        if (proposed_schedule[i] != -1) continue;  // 已有任务，跳过

        // 找最近的未分配任务
        int agent_loc = env->curr_states[i].location;
        int best_goal = -1;
        int best_dist = INT_MAX;

        for (auto& [tid, task] : env->task_pool) {
            if (task.agent_assigned != -1 || task.is_finished()) continue;
            
            int first_loc = task.locations.at(task.idx_next_loc);
            int dist = DefaultPlanner::get_h(env, agent_loc, first_loc);

            if (dist < best_dist) {
                best_dist = dist;
                best_goal = first_loc;
            }
        }

        // 只在距离合理时设置漂移目标（避免长距离无效移动）
        // 阈值30：大约是30步的行走距离，超过这个不值得
        // 如果距离太远，goal_locations 保持为空，planner 让机器人原地等
        if (best_goal != -1 && best_dist <= 30) {
            env->goal_locations[i].push_back({best_goal, env->curr_timestep});
        }
    }
}
