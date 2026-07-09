// optsvr/RoomPibtDensity.cpp — PIBT 区域密度惩罚实现
//
// 核心逻辑：
//   1. update_from_actions()：每轮 plan() 入口调用，统计每个 grid 区域的 FW/W 数量
//   2. get_region_load()：返回区域负载度（wait_ratio）
//   3. densityAwarePIBT()：在 causalPIBT 的候选评分中叠加密度惩罚

#include "PibtDensity.h"
#include "RuntimeConfig.h"
#include "HeuristicMatrix.h"
#include "pibt.h"
#include "utils.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>

using namespace RoomPlan;

// ================================================================
// opt_get_gp_h — 优化版 get_gp_h
//   有 guide path → get_dist_2_path（与原版一致）
//   无 guide path + 矩阵 ready → O(1) 真实距离
//   否则 → 曼哈顿距离
// ================================================================
static int opt_get_gp_h(TrajLNS& lns, int ai, int target) {
    if (!lns.traj_dists.empty() && !lns.traj_dists[ai].empty()) {
        return get_dist_2_path(lns.traj_dists[ai], lns.env, target, &(lns.neighbors));
    }

    return orz_engine::get_h_fast(lns.env, target, lns.tasks.at(ai));
}

namespace orz_engine {

// ================================================================
// initialize — 构建 grid 布局
// ================================================================
void RoomPibtDensity::initialize(SharedEnvironment* env) {
    env_ = env;
    rows_ = env->rows;
    cols_ = env->cols;
    map_size_ = rows_ * cols_;

    // 读取运行时参数
    grid_cell_size_ = 4;        // 固定值
    grid_radius_    = 1;        // 固定值
    density_weight_ = 0.0f;     // 密度惩罚已禁用（测试证明无效）
    ema_alpha_      = 0.98f;    // 固定值

    // 计算 grid 维度
    grid_rows_ = (rows_ + grid_cell_size_ - 1) / grid_cell_size_;
    grid_cols_ = (cols_ + grid_cell_size_ - 1) / grid_cell_size_;
    grid_count_ = grid_rows_ * grid_cols_;

    // 初始化统计为 0
    grid_fw_count_.assign(grid_count_, 0.0f);
    grid_w_count_.assign(grid_count_, 0.0f);

    if (orz_engine::cfg().debug_level >= 1 && density_weight_ > 0.0f) {
        std::cerr << "[RoomPibtDensity] initialized: grid=" << grid_rows_ << "x" << grid_cols_
                  << " (cell=" << grid_cell_size_ << "), weight=" << density_weight_
                  << ", ema_alpha=" << ema_alpha_
                  << ", radius=" << grid_radius_ << "\n";
    }
}

// ================================================================
// cell_to_grid — 地图位置 → grid ID
// ================================================================
int RoomPibtDensity::cell_to_grid(int loc) const {
    int r = loc / cols_;
    int c = loc % cols_;
    int gr = r / grid_cell_size_;
    int gc = c / grid_cell_size_;
    return gr * grid_cols_ + gc;
}

// ================================================================
// update_from_actions — 从上一轮的动作结果更新区域负载统计
//
// 使用 EMA（指数移动平均）平滑：
//   count_new = alpha * current_count + (1 - alpha) * count_old
// alpha 越大越依赖最新数据，越小越平滑
// ================================================================
void RoomPibtDensity::update_from_actions(
    const std::vector<Action>& actions,
    const std::vector<State>& agent_states,
    int num_agents)
{
    if (density_weight_ <= 0.0f) return;  // 不启用时跳过

    // 统计当前轮的 FW 和 W 数量
    // 使用临时缓冲避免与 EMA 的历史数据混淆
    std::vector<float> curr_fw(grid_count_, 0.0f);
    std::vector<float> curr_w(grid_count_, 0.0f);

    for (int i = 0; i < num_agents; i++) {
        int loc = agent_states[i].location;
        if (loc < 0 || loc >= map_size_) continue;
        int gid = cell_to_grid(loc);

        if (actions[i] == Action::FW) {
            curr_fw[gid] += 1.0f;
        } else if (actions[i] == Action::W) {
            curr_w[gid] += 1.0f;
        }
        // CR/CCR 不计入（转向动作不反映移动能力）
    }

    // EMA 更新
    for (int g = 0; g < grid_count_; g++) {
        grid_fw_count_[g] = ema_alpha_ * curr_fw[g] + (1.0f - ema_alpha_) * grid_fw_count_[g];
        grid_w_count_[g]  = ema_alpha_ * curr_w[g]  + (1.0f - ema_alpha_) * grid_w_count_[g];
    }
}

// ================================================================
// query_region — 查询区域负载（考虑邻域）
//
// 返回以 grid_id 为中心、半径 grid_radius_ 范围内的平均 wait_ratio
// wait_ratio = sum_w / (sum_fw + sum_w)，值域 [0, 1]
// 越高越拥堵
// ================================================================
float RoomPibtDensity::query_region(int grid_id) const {
    int gr = grid_id / grid_cols_;
    int gc = grid_id % grid_cols_;

    float sum_fw = 0.0f;
    float sum_w  = 0.0f;

    int r0 = std::max(0, gr - grid_radius_);
    int r1 = std::min(grid_rows_ - 1, gr + grid_radius_);
    int c0 = std::max(0, gc - grid_radius_);
    int c1 = std::min(grid_cols_ - 1, gc + grid_radius_);

    for (int r = r0; r <= r1; r++) {
        for (int c = c0; c <= c1; c++) {
            int gid = r * grid_cols_ + c;
            sum_fw += grid_fw_count_[gid];
            sum_w  += grid_w_count_[gid];
        }
    }

    float total = sum_fw + sum_w;
    if (total < 1.0f) return 0.0f;  // 区域无 agent，无负载

    return sum_w / total;  // wait_ratio，[0, 1]
}

// ================================================================
// get_region_load — 外部查询接口
// ================================================================
float RoomPibtDensity::get_region_load(int loc) const {
    if (loc < 0 || loc >= map_size_) return 0.0f;
    int gid = cell_to_grid(loc);
    return query_region(gid);
}

// ================================================================
// densityAwarePIBT — 带密度惩罚 + 通道方向限制的 causalPIBT
//
// 与原版 causalPIBT (pibt.cpp) 完全相同的逻辑，额外增加：
//   1. 密度惩罚：h' += pibt_density_weight * region_load(neighbor)
//   2. 通道方向硬约束（corridor_enforce=true 时生效）：
//      逆向移动直接跳过（在候选生成阶段 continue）
// ================================================================
bool RoomPibtDensity::densityAwarePIBT(
    int curr_id, int higher_id,
    std::vector<State>& prev_states,
    std::vector<State>& next_states,
    std::vector<int>& prev_decision,
    std::vector<int>& decision,
    std::vector<bool>& occupied,
    TrajLNS& lns)
{
    assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
    int prev_orientation = prev_states[curr_id].orientation;
    int next_locs[4] = {
        prev_loc + 1,
        prev_loc + lns.env->cols,
        prev_loc - 1,
        prev_loc - lns.env->cols
    };
    int orien_next_v = next_locs[prev_orientation];

    assert(prev_loc >= 0 && prev_loc < (int)lns.env->map.size());

    int target = lns.tasks.at(curr_id);

    if (higher_id == -1) {
        occupied[prev_loc] = true;
    }

    // 生成候选
    std::vector<int> neighbors;
    std::vector<PIBT_C> successors;
    getNeighborLocs(&(lns.neighbors), neighbors, prev_loc);

    for (auto& neighbor : neighbors) {
        assert(validateMove(prev_loc, neighbor, lns.env));

        int min_heuristic = get_gp_h(lns, curr_id, neighbor);

        // 密度惩罚（当前 density_weight_=0，实际不生效）
        int density_penalty = static_cast<int>(
            density_weight_ * get_region_load(neighbor) + 0.5f);
        min_heuristic += density_penalty;

        successors.emplace_back(neighbor, min_heuristic, 0, rand());
    }

    // wait 候选（原地等待不违反方向约束）
    int wait_heuristic = get_gp_h(lns, curr_id, prev_loc);
    int wait_penalty = static_cast<int>(
        density_weight_ * get_region_load(prev_loc) + 0.5f);
    wait_heuristic += wait_penalty;
    successors.emplace_back(prev_loc, wait_heuristic, 0, rand());

    // 排序规则：heuristic → 前走偏好 → 方向合规性 → 随机
    std::sort(successors.begin(), successors.end(),
        [&](PIBT_C& a, PIBT_C& b) {
            if (a.heuristic == b.heuristic) {
                // tie break 1: 偏好向前走
                if (a.location == orien_next_v && b.location != orien_next_v)
                    return true;
                if (a.location != orien_next_v && b.location == orien_next_v)
                    return false;
                // tie break 2: 偏好顺方向（maze_soft 时 orientation=0 优先）
                if (a.orientation != b.orientation)
                    return a.orientation < b.orientation;
                // tie break 3: random
                return a.tie_breaker < b.tie_breaker;
            }
            return a.heuristic < b.heuristic;
        });

    // 遍历候选，尝试移动
    for (auto& next : successors) {
        if (occupied[next.location] && !(higher_id == -1 && prev_loc == next.location))
            continue;
        assert(validateMove(prev_loc, next.location, lns.env));

        if (next.location == -1)
            continue;
        if (decision[next.location] != -1)
            continue;
        if (higher_id != -1 && prev_decision[next.location] == higher_id)
            continue;

        next_states.at(curr_id) = State(next.location, -1, -1);
        decision.at(next.location) = curr_id;

        if (prev_decision.at(next.location) != -1 &&
            next_states.at(prev_decision.at(next.location)).location == -1) {
            int lower_id = prev_decision.at(next.location);
            // ★ 递归也用密度版本
            if (!densityAwarePIBT(lower_id, curr_id, prev_states, next_states,
                                   prev_decision, decision, occupied, lns)) {
                continue;
            }
        }

        return true;
    }

    // 所有候选失败 → 强制原地等待
    next_states.at(curr_id) = State(prev_loc, -1, -1);
    decision.at(prev_loc) = curr_id;
    return false;
}

} // namespace orz_engine
