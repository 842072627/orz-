// optsvr/schedule/MazeOptScheduler/MazeOptScheduler.h
// Source: IDEA/MAZEOpt default TaskScheduler schedule method.
// DHFULL assignment before tick 3000, then min-cost-flow top80.
#pragma once

#include "SharedEnv.h"

#include <unordered_set>
#include <vector>

namespace optsvr {

class MazeOptScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms);
    void schedule(int time_limit_ms, std::vector<int>& proposed_schedule,
                  SharedEnvironment* env);

private:
    struct AgentTask {
        int task_id = -1;
        int min_task_dist = -1;
        double jam_when_assign = -1;
        int assign_moment = -1;
        int complete_moment = -1;
    };

    static constexpr int kSwitchTimestep = 3000;
    static constexpr double kTop80AssignmentRatio = 0.8;
    static constexpr int kStealDistanceThreshold = 10;
    static constexpr int kDensityCellSize = 2;

    SharedEnvironment* env_ = nullptr;
    std::vector<AgentTask> agent_task_;
    std::vector<int> task_agent_;
    std::unordered_set<int> free_agents_;
    std::unordered_set<int> free_tasks_;

    int num_task_finished_ = 0;
    int total_min_span_ = 0;
    int total_real_duration_ = 0;
    double total_jam_ = 0.0;

    void ensure_task_agent_size(int task_id);
    int scheduler_distance(int source, int target) const;
    int task_min_distance(int agent_id, int task_id) const;

    void lorr26_dhfull_scheduler(int time_limit_ms,
                                 std::vector<int>& proposed_schedule,
                                 double assignment_ratio = 1.0,
                                 bool use_min_cost_flow = false);
    void lorr26_dhfull_mincostflow_top80_scheduler(
        int time_limit_ms,
        std::vector<int>& proposed_schedule);
    void lorr26_dhfull_until_3000_then_mincostflow_top80_scheduler(
        int time_limit_ms,
        std::vector<int>& proposed_schedule);
};

} // namespace optsvr
