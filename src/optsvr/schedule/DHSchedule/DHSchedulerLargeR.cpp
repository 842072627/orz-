// optsvr/schedule/DHSchedule/DHSchedulerLargeR.cpp
// 来源：IDEA/Schedule-DH/TaskSchedulerLargeR.cpp（保留原始结构，用类封装）
// Room 大规模(num_of_agents > 300)：K-means 聚类 + 局部贪心
#include "DHScheduler.h"
#include "const.h"
#include "heuristics.h"
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <limits>

namespace optsvr {

std::pair<int, int> DHScheduler::get_task_location(int task_id) {
    if (env_->task_pool.find(task_id) != env_->task_pool.end() &&
        !env_->task_pool[task_id].locations.empty()) {
        int loc = env_->task_pool[task_id].locations[0];
        int row = loc / env_->cols;
        int col = loc % env_->cols;
        return {row, col};
    }
    return {0, 0};
}

double DHScheduler::euclidean_distance(const std::pair<int, int>& a, const std::pair<int, int>& b) {
    double dr = a.first - b.first;
    double dc = a.second - b.second;
    return std::sqrt(dr * dr + dc * dc);
}

void DHScheduler::cluster_tasks() {
    if (free_tasks_.empty()) {
        clusters_.clear();
        task_to_cluster_.clear();
        return;
    }

    int K = std::max(1, static_cast<int>(env_->num_of_agents / 2));
    K = std::min(K, static_cast<int>(free_tasks_.size()));

    std::vector<std::pair<int, int>> task_positions;
    std::vector<int> task_ids;
    for (int t_id : free_tasks_) {
        task_positions.push_back(get_task_location(t_id));
        task_ids.push_back(t_id);
    }

    clusters_.clear();
    clusters_.resize(K);

    std::vector<int> indices(task_ids.size());
    for (int i = 0; i < static_cast<int>(indices.size()); i++) indices[i] = i;
    std::shuffle(indices.begin(), indices.end(), rng_);

    for (int i = 0; i < K; i++) {
        clusters_[i].center = task_positions[indices[i]];
    }

    const int max_iterations = 10;
    for (int iter = 0; iter < max_iterations; iter++) {
        for (auto& cluster : clusters_) cluster.task_ids.clear();

        for (int i = 0; i < static_cast<int>(task_ids.size()); i++) {
            int best_cluster = 0;
            double min_dist = std::numeric_limits<double>::max();
            for (int c = 0; c < K; c++) {
                double dist = euclidean_distance(task_positions[i], clusters_[c].center);
                if (dist < min_dist) { min_dist = dist; best_cluster = c; }
            }
            clusters_[best_cluster].task_ids.push_back(task_ids[i]);
            task_to_cluster_[task_ids[i]] = best_cluster;
        }

        bool converged = true;
        for (int c = 0; c < K; c++) {
            if (clusters_[c].task_ids.empty()) continue;
            double sum_row = 0, sum_col = 0;
            for (int t_id : clusters_[c].task_ids) {
                auto pos = get_task_location(t_id);
                sum_row += pos.first; sum_col += pos.second;
            }
            std::pair<int, int> new_center = {
                static_cast<int>(sum_row / clusters_[c].task_ids.size()),
                static_cast<int>(sum_col / clusters_[c].task_ids.size())
            };
            if (new_center != clusters_[c].center) { converged = false; clusters_[c].center = new_center; }
        }
        if (converged) break;
    }
    last_clustered_free_tasks_count_ = free_tasks_.size();
}

int DHScheduler::find_nearest_cluster(int agent_id) {
    int agent_loc = env_->curr_states.at(agent_id).location;
    std::pair<int, int> agent_pos = {agent_loc / env_->cols, agent_loc % env_->cols};
    int best_cluster = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int c = 0; c < static_cast<int>(clusters_.size()); c++) {
        if (clusters_[c].task_ids.empty()) continue;
        double dist = euclidean_distance(agent_pos, clusters_[c].center);
        if (dist < min_dist) { min_dist = dist; best_cluster = c; }
    }
    return best_cluster;
}

void DHScheduler::planLargeR(int time_limit, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    free_agents_.insert(env_->new_freeagents.begin(), env_->new_freeagents.end());
    free_tasks_.insert(env_->new_tasks.begin(), env_->new_tasks.end());

    if (free_tasks_.size() != static_cast<size_t>(last_clustered_free_tasks_count_)) {
        cluster_tasks();
    }

    int min_task_i, min_task_makespan, dist, c_loc, count;

    auto it = free_agents_.begin();
    while (it != free_agents_.end()) {
        if (std::chrono::steady_clock::now() > endtime) break;
        int i = *it;

        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        // 首先尝试在最近聚类中寻找任务
        if (!clusters_.empty()) {
            int nearest_cluster = find_nearest_cluster(i);
            const auto& cluster_task_ids = clusters_[nearest_cluster].task_ids;

            for (int t_id : cluster_task_ids) {
                if (free_tasks_.find(t_id) == free_tasks_.end()) continue;
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;

                dist = 0;
                c_loc = env_->curr_states.at(i).location;
                for (int loc : env_->task_pool[t_id].locations) {
                    dist += DefaultPlanner::get_h(env_, c_loc, loc);
                    c_loc = loc;
                }
                if (dist < min_task_makespan) { min_task_i = t_id; min_task_makespan = dist; }
                count++;
            }
        }

        // 降级：在整个 free_tasks 中搜索
        if (min_task_i == -1) {
            for (int t_id : free_tasks_) {
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
                dist = 0;
                c_loc = env_->curr_states.at(i).location;
                for (int loc : env_->task_pool[t_id].locations) {
                    dist += DefaultPlanner::get_h(env_, c_loc, loc);
                    c_loc = loc;
                }
                if (dist < min_task_makespan) { min_task_i = t_id; min_task_makespan = dist; }
                count++;
            }
        }

        if (min_task_i != -1) {
            proposed_schedule[i] = min_task_i;
            it = free_agents_.erase(it);
            free_tasks_.erase(min_task_i);
            // 从聚类中移除已分配的任务
            if (task_to_cluster_.find(min_task_i) != task_to_cluster_.end()) {
                int cluster_id = task_to_cluster_[min_task_i];
                auto& cluster = clusters_[cluster_id];
                auto it_c = std::find(cluster.task_ids.begin(), cluster.task_ids.end(), min_task_i);
                if (it_c != cluster.task_ids.end()) cluster.task_ids.erase(it_c);
                task_to_cluster_.erase(min_task_i);
            }
        } else {
            proposed_schedule[i] = -1;
            it++;
        }
    }
}

} // namespace optsvr
