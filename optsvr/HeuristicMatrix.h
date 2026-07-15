// optsvr/HeuristicMatrix.h — 多线程预计算全量距离矩阵
//
// 两种模式（由 TURN_AWARE_HEURISTIC 开关控制，只计算/存储一份矩阵）：
//
// ---- 模式 A（默认，turn_aware=false）：location-only BFS ----
//   状态: location
//   矩阵: n_free × n_free × 2 bytes
//   查询: get(source, target) → 无转弯的 BFS 距离
//
// ---- 模式 B（turn_aware=true）：(location, orientation) BFS ----
//   状态: (location, orientation)，共 4 × n_free 个状态
//   矩阵: n_free × (4 × n_free) × 2 bytes = 4× 模式 A 的内存
//     行 = target（无方向，只需到达 target 位置即可）
//     列 = (source_location, source_orientation)
//   查询: get_oriented(source, source_orient, target) → 实际 action 步数（含转弯）
//
// 内存估算（模式 B）：
//   - warehouse_small (1277 free):  ~12.4 MB
//   - warehouse_large (38586 free): ~11.2 GB
//   - game_brc202d (43151 free):    ~14.0 GB
//
// 评测环境：128 GB RAM + 32 核 CPU + 30 分钟预处理
#pragma once

#include "SharedEnv.h"
#include <array>
#include <vector>
#include <cstdint>
#include <chrono>

namespace optsvr {

// 无效距离标记（对应 uint16_t 最大值）
constexpr uint16_t DIST_UNREACHABLE = 0xFFFF;

class HeuristicMatrix {
public:
    // 构建距离矩阵（在预处理阶段调用，使用多线程 BFS）
    // env: 共享环境
    // n_threads: 线程数，0=自动检测
    // deadline: 预处理截止时间
    // turn_aware: true=构建 (loc,orient) BFS; false=构建 location-only BFS
    void build(SharedEnvironment* env, int n_threads,
               std::chrono::steady_clock::time_point deadline,
               bool turn_aware = false);

    // ---- location-only 查询（模式 A） ----
    // 返回 source → target 的 BFS 距离（不含转弯）
    uint16_t get(int source, int target) const;

    // ---- orientation-aware 查询（模式 B） ----
    // 返回 (source, source_orient) → target 的精确 action 步数（含转弯）
    // 仅当 turn_aware_ == true 时有效
    uint16_t get_oriented(int source, int source_orient, int target) const;

    // 是否已完成预计算
    bool is_ready() const { return ready_; }

    // 当前矩阵是否为转弯感知模式
    bool is_turn_aware() const { return turn_aware_; }

    // 当前矩阵是否启用了静态方向硬约束
    bool is_direction_constrained() const { return direction_constrained_; }

    // 可通行 cell 数量
    int n_free_cells() const { return n_free_; }

    // 预计算覆盖的 target 数量
    int n_computed() const { return n_computed_; }

    // 预计算耗时（毫秒）
    int build_time_ms() const { return build_time_ms_; }

    // 地图列数（用于外部计算方向）
    int cols() const { return cols_; }

private:
    bool ready_ = false;
    bool turn_aware_ = false;
    bool direction_constrained_ = false;

    // 地图信息
    int map_size_ = 0;
    int rows_ = 0;
    int cols_ = 0;
    int n_free_ = 0;

    // cell_index → compressed_index (仅可通行 cell 有映射)
    std::vector<int> cell_to_idx_;

    // compressed_index → cell_index
    std::vector<int> idx_to_cell_;

    // 距离矩阵
    // 模式 A: matrix_[target_idx * n_free_ + source_idx]
    // 模式 B: matrix_[target_idx * n_states_ + source_state]
    //         其中 source_state = source_idx * 4 + orientation
    std::vector<uint16_t> matrix_;

    // 模式 B 的状态数 = 4 * n_free_
    int n_states_ = 0;

    // 邻居表
    std::vector<std::vector<int>> neighbors_;
    std::vector<std::vector<int>> reverse_neighbors_;

    // 静态方向硬约束：forbidden_dirs_[loc][dir] != 0 表示禁止从 loc 朝 dir 前进
    std::vector<std::array<unsigned char, 4>> forbidden_dirs_;
    int forbidden_edge_count_ = 0;

    // 统计
    int n_computed_ = 0;
    int build_time_ms_ = 0;

    // 模式 A：从一个 target cell 出发做反向 BFS（location-only）
    void bfs_from(int target_idx);

    // 模式 B：从一个 target cell 出发做反向 BFS（(loc,orient) 状态空间）
    void bfs_from_oriented(int target_idx);

    void build_direction_constraints(SharedEnvironment* env);
    bool forward_allowed(int from, int to) const;
};

// ================================================================
// 全局单例 + 便捷接口
// ================================================================

// 全局距离矩阵实例
HeuristicMatrix& heuristic_matrix();

// 便捷查询接口（统一入口）：
// - turn_aware 模式 + source_orient >= 0 → 精确 action 步数
// - 其他情况 → location-only BFS 距离或曼哈顿距离
// source_orient 默认 -1 表示不传朝向
int get_h_fast(SharedEnvironment* env, int source, int target, int source_orient = -1);

} // namespace optsvr
