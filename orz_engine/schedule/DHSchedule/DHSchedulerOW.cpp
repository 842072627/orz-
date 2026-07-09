// optsvr/schedule/DHSchedule/DHSchedulerOW.cpp
// 来源：IDEA/Schedule-DH/TaskSchedulerOW.cpp（保留原始结构，用类封装）
// Open/Warehouse：QuadTree + 曼哈顿半径搜索 + 贪心
#include "DHScheduler.h"
#include "const.h"
#include "heuristics.h"
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>

namespace orz_engine {

// ============= QuadTree Implementation =============
DHScheduler::QuadTreeNode::QuadTreeNode(int x1, int y1, int x2, int y2, int lvl)
    : x_min(x1), y_min(y1), x_max(x2), y_max(y2), level(lvl), is_leaf(true) {
    children[0] = children[1] = children[2] = children[3] = nullptr;
}

DHScheduler::QuadTreeNode::~QuadTreeNode() {
    for (int i = 0; i < 4; ++i) delete children[i];
}

bool DHScheduler::QuadTreeNode::contains(int loc, int cols) const {
    int x = loc % cols; int y = loc / cols;
    return x >= x_min && x < x_max && y >= y_min && y < y_max;
}

int DHScheduler::QuadTreeNode::getChildIndex(int loc, int cols) const {
    int x = loc % cols; int y = loc / cols;
    int mid_x = (x_min + x_max) / 2; int mid_y = (y_min + y_max) / 2;
    if (y < mid_y) return (x < mid_x) ? 0 : 1;
    return (x < mid_x) ? 2 : 3;
}

void DHScheduler::QuadTreeNode::subdivide(int cols) {
    if (!is_leaf) return;
    is_leaf = false;
    int mid_x = (x_min + x_max) / 2; int mid_y = (y_min + y_max) / 2;
    children[0] = new QuadTreeNode(x_min, y_min, mid_x, mid_y, level + 1);
    children[1] = new QuadTreeNode(mid_x, y_min, x_max, mid_y, level + 1);
    children[2] = new QuadTreeNode(x_min, mid_y, mid_x, y_max, level + 1);
    children[3] = new QuadTreeNode(mid_x, mid_y, x_max, y_max, level + 1);
}

int DHScheduler::getLocationX(int loc) const { return map_cols_ > 0 ? loc % map_cols_ : 0; }
int DHScheduler::getLocationY(int loc) const { return map_cols_ > 0 ? loc / map_cols_ : 0; }

void DHScheduler::initQuadTree() {
    map_cols_ = env_->cols > 0 ? env_->cols : 100;
    map_rows_ = env_->rows > 0 ? env_->rows : 100;
    max_depth_ = 6;
    max_items_per_leaf_ = 8;
    if (quad_root_) delete quad_root_;
    quad_root_ = new QuadTreeNode(0, 0, map_cols_, map_rows_, 0);
}

void DHScheduler::insertAgentInternal(int agent_id, QuadTreeNode* node) {
    int loc = env_->curr_states.at(agent_id).location;
    if (!node->contains(loc, map_cols_)) return;
    if (node->is_leaf) {
        node->agent_indices.push_back(agent_id);
        int cur_items = (int)node->agent_indices.size() + (int)node->task_ids.size();
        if (node->level < max_depth_ && cur_items > max_items_per_leaf_) {
            std::vector<int> agents_backup = node->agent_indices;
            std::vector<int> tasks_backup = node->task_ids;
            node->agent_indices.clear(); node->task_ids.clear();
            node->subdivide(map_cols_);
            for (int a : agents_backup) { int l = env_->curr_states.at(a).location; int idx = node->getChildIndex(l, map_cols_); if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->agent_indices.push_back(a); }
            for (int t : tasks_backup) { if (!env_->task_pool.count(t)) continue; int loc_t = env_->task_pool.at(t).locations.front(); int idx = node->getChildIndex(loc_t, map_cols_); if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->task_ids.push_back(t); }
        }
    } else {
        int idx = node->getChildIndex(loc, map_cols_);
        if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->agent_indices.push_back(agent_id);
    }
}

void DHScheduler::insertTaskInternal(int task_id, QuadTreeNode* node) {
    if (!env_->task_pool.count(task_id)) return;
    int loc = env_->task_pool.at(task_id).locations.front();
    if (!node->contains(loc, map_cols_)) return;
    if (node->is_leaf) {
        node->task_ids.push_back(task_id);
        int cur_items = (int)node->agent_indices.size() + (int)node->task_ids.size();
        if (node->level < max_depth_ && cur_items > max_items_per_leaf_) {
            std::vector<int> agents_backup = node->agent_indices;
            std::vector<int> tasks_backup = node->task_ids;
            node->agent_indices.clear(); node->task_ids.clear();
            node->subdivide(map_cols_);
            for (int a : agents_backup) { int l = env_->curr_states.at(a).location; int idx = node->getChildIndex(l, map_cols_); if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->agent_indices.push_back(a); }
            for (int t : tasks_backup) { if (!env_->task_pool.count(t)) continue; int loc_t = env_->task_pool.at(t).locations.front(); int idx = node->getChildIndex(loc_t, map_cols_); if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->task_ids.push_back(t); }
        }
    } else {
        int idx = node->getChildIndex(loc, map_cols_);
        if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->task_ids.push_back(task_id);
    }
}

void DHScheduler::rebuildQuadTree() {
    if (quad_root_) delete quad_root_;
    quad_root_ = new QuadTreeNode(0, 0, map_cols_, map_rows_, 0);
    for (int agent_id : free_agents_) insertAgentInternal(agent_id, quad_root_);
    for (int task_id : free_tasks_) insertTaskInternal(task_id, quad_root_);
}

int DHScheduler::estimateTaskMakespan(int agent_id, int task_id) const {
    if (!env_->task_pool.count(task_id)) return INT_MAX;
    int dist = 0; int c_loc = env_->curr_states.at(agent_id).location;
    for (int loc : env_->task_pool.at(task_id).locations) {
        dist += DefaultPlanner::get_h(env_, c_loc, loc); c_loc = loc;
    }
    return dist;
}

void DHScheduler::collectTasksInRadius(int agent_loc, int radius, std::vector<int>& nearby_tasks) {
    nearby_tasks.clear();
    int ax = getLocationX(agent_loc); int ay = getLocationY(agent_loc);
    for (int task_id : free_tasks_) {
        if (!env_->task_pool.count(task_id)) continue;
        int tloc = env_->task_pool.at(task_id).locations.front();
        int tx = getLocationX(tloc); int ty = getLocationY(tloc);
        if (abs(tx - ax) + abs(ty - ay) <= radius) nearby_tasks.push_back(task_id);
    }
}

void DHScheduler::planOW(int time_limit, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    free_agents_.insert(env_->new_freeagents.begin(), env_->new_freeagents.end());
    free_tasks_.insert(env_->new_tasks.begin(), env_->new_tasks.end());

    rebuildQuadTree();

    int search_radius = 30;

    std::vector<int> agent_list(free_agents_.begin(), free_agents_.end());
    std::shuffle(agent_list.begin(), agent_list.end(), mt_);

    for (int i : agent_list) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (env_->curr_task_schedule[i] != -1) continue;

        int agent_loc = env_->curr_states.at(i).location;
        int best_task = -1; int best_makespan = INT_MAX;

        std::vector<int> candidate_tasks;
        collectTasksInRadius(agent_loc, search_radius, candidate_tasks);
        if (candidate_tasks.empty()) candidate_tasks.assign(free_tasks_.begin(), free_tasks_.end());

        int eval_count = 0; int max_eval = 200;
        for (int t_id : candidate_tasks) {
            if (!env_->task_pool.count(t_id)) continue;
            if (eval_count++ > max_eval) break;
            if (std::chrono::steady_clock::now() > endtime) break;
            int makespan = estimateTaskMakespan(i, t_id);
            if (makespan < best_makespan) { best_makespan = makespan; best_task = t_id; }
        }

        if (best_task != -1) {
            proposed_schedule[i] = best_task;
            free_agents_.erase(i); free_tasks_.erase(best_task);
        } else {
            proposed_schedule[i] = -1;
        }
    }
}

} // namespace orz_engine
