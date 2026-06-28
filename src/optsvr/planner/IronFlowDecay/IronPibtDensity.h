// optsvr/IronPibtDensity.h — PIBT 路径规划的区域密度惩罚
//
// 核心思路：
//   1. 维护 grid-level 统计，记录每个区域的成功 FW 数量和 W 数量
//   2. wait_ratio = wait_count / (fw_count + wait_count) 越高说明区域越拥堵
//   3. 在 PIBT 候选位置评分中叠加密度惩罚：
//      h' = get_gp_h(lns, ai, neighbor) + pibt_density_weight * region_load(neighbor)
//   4. 这样 agent 会自动偏好移向低负载区域
//
// 特点：
//   - 不修改 default_planner/ 下的受保护文件
//   - pibt_density_weight=0 时退化为原始 PIBT 行为（零开销）
//   - 区域负载使用指数移动平均（EMA），自动适应动态变化
#pragma once

#include "SharedEnv.h"
#include "TrajLNS.h"
#include "Types.h"
#include <vector>
#include <algorithm>
#include <cmath>

namespace optsvr {

class IronPibtDensity {
public:
    // ============ 初始化 ============
    // 在 IronFlowDecayPlanner::initialize 中调用一次
    void initialize(SharedEnvironment* env);

    // ============ 每步 PIBT 前调用 ============
    // 在 run_multistep_pibt_once 开始前调用（不需每个 multi-step 内都调用，
    // 只在 plan() 入口调用一次即可，因为 rollout 是模拟的）
    // 传入上一轮的 FW 和 W 动作统计，更新区域负载
    void update_from_actions(const std::vector<Action>& actions,
                             const std::vector<State>& agent_states,
                             int num_agents);

    // ============ 查询 ============
    // 返回位置 loc 所在区域的负载度（0.0~1.0，越高越拥堵）
    // 基于 wait_ratio = wait_count / (fw_count + wait_count)
    float get_region_load(int loc) const;

    // ============ 带密度惩罚的 causalPIBT ============
    // 替代 DefaultPlanner::causalPIBT，在候选位置评分中叠加密度惩罚
    // 接口与原版完全一致，额外使用内部的密度统计
    bool densityAwarePIBT(int curr_id, int higher_id,
                          std::vector<State>& prev_states,
                          std::vector<State>& next_states,
                          std::vector<int>& prev_decision,
                          std::vector<int>& decision,
                          std::vector<bool>& occupied,
                          IronFlowDecayPlan::TrajLNS& lns);

    // ============ 状态查询（调试） ============
    int grid_count() const { return grid_count_; }

private:
    SharedEnvironment* env_ = nullptr;

    // ---- Grid 参数 ----
    int rows_ = 0;
    int cols_ = 0;
    int map_size_ = 0;
    int grid_cell_size_ = 0;
    int grid_rows_ = 0;
    int grid_cols_ = 0;
    int grid_count_ = 0;

    // ---- 区域负载统计 ----
    // grid_fw_count_[g]  = 区域 g 中上一轮成功 FW 的 agent 数量（EMA 平滑）
    // grid_w_count_[g]   = 区域 g 中上一轮 W（等待）的 agent 数量（EMA 平滑）
    std::vector<float> grid_fw_count_;
    std::vector<float> grid_w_count_;

    // ---- 缓存的运行时参数 ----
    float density_weight_ = 0.0f;    // pibt_density_weight
    float ema_alpha_ = 0.0f;         // EMA 衰减因子
    int   grid_radius_ = 0;          // 查询半径

    // ---- 辅助 ----
    int cell_to_grid(int loc) const;
    float query_region(int grid_id) const;  // 考虑邻域的区域负载查询
};

} // namespace optsvr
