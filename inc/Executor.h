#pragma once
#include "ActionModel.h"
#include "SharedEnv.h"
#include "Plan.h"
#include "Config.h"
#include <deque>
#include <vector>
#include <string>

enum ProcessPlanType {APPEND_ONE, APPEND_ALL, APPEND_WINDOW};

class Executor
{
public:

    SharedEnvironment* env;
    Executor(SharedEnvironment* env): env(env){};
    Executor(){env = new SharedEnvironment();};
    virtual ~Executor(){delete env;};   
    
    virtual void initialize(int preprocess_time_limit);
    virtual vector<State> process_new_plan(int sync_time_limit, Plan & plan, vector<vector<Action>> & staged_actions);
    virtual void next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command);

    vector<list<int>> tpg;
    vector<int> previous_locations;
    vector<State> predicted_states;

    ProcessPlanType process_plan_type = APPEND_WINDOW;
    int window_size;
    int action_ceil_size;
    int max_counter_ = 10;
    int stage_action_max;
    int turn = 0;
    int total_push_action_count = 0;

    int following_chain_go_count_ = 0;
    long long following_chain_go_total_ = 0;

    long long empty_queue_exec_count_ = 0;

    int origin_extra_append_count_ = 0;
    long long origin_extra_append_total_ = 0;

    long long total_window_size_sum_ = 0;
    long long total_window_size_cnt_ = 0;

    double get_avg_window_size() const {
        return total_window_size_cnt_ > 0
            ? static_cast<double>(total_window_size_sum_) / static_cast<double>(total_window_size_cnt_)
            : 0.0;
    }

    int       staged_last_epoch_tick_ = 0;
    double    staged_sum_per_epoch_ = 0.0;
    int       staged_plan_cnt_per_epoch_ = 0;
    double    staged_sum_total_ = 0.0;
    int       staged_plan_cnt_total_ = 0;
    std::vector<double> staged_actions_epoch_avgs_;

    void record_staged_actions_stat(const std::vector<std::vector<Action>>& staged_actions);

    double get_avg_staged_actions() const {
        return staged_plan_cnt_total_ > 0
            ? staged_sum_total_ / staged_plan_cnt_total_
            : 0.0;
    }

    long long get_origin_extra_append_total() const {
        return origin_extra_append_total_;
    }

    bool is_valid_location(int location) const;
    bool contains_agent_in_tpg(int location, int agent_id) const;
    bool erase_agent_from_tpg(int location, int agent_id);

    void rebuild_from_system_states();

    vector<State> process_new_plan_official(int sync_time_limit,
                                            Plan& plan,
                                            vector<vector<Action>>& staged_actions);

    vector<State> process_new_plan_legacy(int sync_time_limit,
                                          Plan& plan,
                                          vector<vector<Action>>& staged_actions);

    bool official_process_initialized_ = false;

    bool mcp(int agent_id, vector<bool> & curr_decision, std::vector<ExecutionCommand> & agent_command);
    bool mcp_default_executor(int agent_id, std::vector<ExecutionCommand> & agent_command);
};