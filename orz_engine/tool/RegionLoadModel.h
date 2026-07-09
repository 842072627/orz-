// optsvr/tool/RegionLoadModel.h
// Connected region abstraction for map-scale task scoring experiments.
#pragma once

#include "SharedEnv.h"
#include "nlohmann/json.hpp"

#include <climits>
#include <string>
#include <vector>

namespace orz_engine {

enum class RegionLoadMode {
    Adaptive,
    Cell,
};

struct RegionLoadConfig {
    bool enabled = false;
    RegionLoadMode mode = RegionLoadMode::Adaptive;
    int target_regions = 100;
    int max_regions = 256;
    int max_distance_regions = 512;
    bool build_tendency = true;
    int max_tendency_regions = 512;
    double tendency_decay = 1.0;
    bool tendency_size_weighted = false;
    std::string tendency_kind = "bottleneck";
    double bottleneck_narrow_weight = 2.0;
    double bottleneck_capacity_power = 2.0;
    std::string export_dir = "result/region_partition";

    static RegionLoadConfig from_env();
    std::string mode_name() const;
};

struct RegionInfo {
    int id = -1;
    int size = 0;
    int seed_loc = -1;
    double centroid_row = 0.0;
    double centroid_col = 0.0;
    int min_row = INT_MAX;
    int max_row = -1;
    int min_col = INT_MAX;
    int max_col = -1;
    int boundary_cells = 0;
};

class RegionLoadModel {
public:
    void clear();
    void initialize(const SharedEnvironment* env, const RegionLoadConfig& config);

    bool ready() const { return ready_; }
    int rows() const { return rows_; }
    int cols() const { return cols_; }
    int n_free() const { return n_free_; }
    int num_regions() const { return region_count_; }
    bool tendency_ready() const { return tendency_ready_; }
    const RegionLoadConfig& config() const { return config_; }

    int region_of(int loc) const;
    int region_distance(int region_a, int region_b) const;

    // Reserved for future task scoring. V1 only builds the abstraction.
    double segment_region_risk(int from_loc, int to_loc) const;

    const std::vector<int>& loc_to_region() const { return loc_to_region_; }
    const std::vector<RegionInfo>& regions() const { return regions_; }
    const std::vector<std::vector<int>>& region_adj() const { return region_adj_; }
    const std::vector<double>& region_tendency() const { return region_tendency_; }
    const std::vector<double>& region_tendency_normalized() const {
        return region_tendency_normalized_;
    }

    nlohmann::json to_json() const;
    bool export_json(const std::string& path) const;

private:
    RegionLoadConfig config_;
    bool ready_ = false;
    bool distances_ready_ = false;
    bool tendency_ready_ = false;
    int rows_ = 0;
    int cols_ = 0;
    int map_size_ = 0;
    int n_free_ = 0;
    int region_count_ = 0;
    std::string map_name_;

    std::vector<unsigned char> is_free_;
    std::vector<int> free_locs_;
    std::vector<int> loc_to_region_;
    std::vector<RegionInfo> regions_;
    std::vector<std::vector<int>> region_adj_;
    std::vector<std::vector<int>> region_dist_;
    std::vector<double> region_tendency_;
    std::vector<double> region_tendency_normalized_;
    std::vector<double> region_tendency_share_;
    std::vector<double> region_soft_tendency_;
    std::vector<double> region_soft_tendency_normalized_;
    std::vector<double> region_bottleneck_tendency_;
    std::vector<double> region_bottleneck_tendency_normalized_;
    std::vector<double> region_constriction_;
    std::vector<double> region_avg_free_degree_;
    double tendency_sum_ = 0.0;
    double tendency_max_ = 0.0;
    double tendency_pair_weight_sum_ = 0.0;
    long long tendency_pair_count_ = 0;

    int choose_region_count() const;
    void build_cell_regions();
    void build_adaptive_regions();
    std::vector<std::vector<int>> collect_components() const;
    std::vector<int> select_component_seeds(const std::vector<int>& cells, int k) const;
    void rebuild_region_stats();
    void build_adjacency();
    void compute_all_pairs_region_dist();
    void compute_region_tendency();
    void compute_region_constriction();
    void normalize_metric(const std::vector<double>& raw,
                          std::vector<double>& normalized) const;
    int bfs_region_distance(int region_a, int region_b) const;
    double route_tendency_risk(int region_a, int region_b) const;
    bool is_free_loc(int loc) const;
};

RegionLoadModel& region_load_model();

}  // namespace orz_engine
