// optsvr/planner/GameRegionDecomp/GameRegionDecompPlanner.h
// Planner mode for exporting a game-map decomposition into open regions,
// narrow passages, and turn gates. It delegates motion planning to GameOpt.
#pragma once

#include "ActionModel.h"
#include "SharedEnv.h"
#include "../GameOpt/GameOptPlanner.h"

#include <array>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace orz_engine {

class GameRegionDecompPlanner {
public:
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    enum class CellKind : unsigned char {
        Blocked = 0,
        Open = 1,
        NarrowH = 2,
        NarrowV = 3,
        TurnGate = 4,
    };

    enum class NodeType : unsigned char {
        OpenRegion = 1,
        NarrowSegment = 2,
        TurnGate = 3,
    };

    struct Node {
        int id = -1;
        NodeType type = NodeType::OpenRegion;
        int orientation = 0;  // 0=mixed/open, 1=horizontal, 2=vertical
        int size = 0;
        int r0 = 0;
        int c0 = 0;
        int r1 = 0;
        int c1 = 0;
        int min_clearance = 0;
        int min_width = 0;
        int capacity = 1;
        int centroid_r = 0;
        int centroid_c = 0;
        bool major_region = false;
        bool connects_regions = false;
        std::vector<int> connected_open_regions;
        std::vector<int> connected_major_open_regions;
        std::set<int> adjacent;
    };

    static constexpr int WIDTH_THRESHOLD = 4;
    static constexpr int MIN_RUN_LEN = 8;
    static constexpr int MIN_COMPONENT_SIZE = 4;
    static constexpr int DEFAULT_MAJOR_REGION_MIN_SIZE = 400;
    static constexpr int DENSITY_WINDOW_RADIUS = 4;
    static constexpr float LOCAL_DENSITY_THRESHOLD = 0.58f;

    SharedEnvironment* env_ = nullptr;
    std::unique_ptr<GameOptPlanner> fallback_;
    bool exported_ = false;

    int rows_ = 0;
    int cols_ = 0;
    int total_ = 0;
    int major_region_min_size_ = DEFAULT_MAJOR_REGION_MIN_SIZE;
    std::string output_dir_;
    std::string output_stem_;

    std::vector<unsigned char> free_;
    std::vector<int> degree_;
    std::vector<int> clearance_;
    std::vector<int> h_run_;
    std::vector<int> v_run_;
    std::vector<unsigned char> raw_kind_;
    std::vector<CellKind> cell_kind_;
    std::vector<int> component_id_;
    std::vector<Node> nodes_;

    void build_decomposition(const SharedEnvironment* env);
    void compute_degree();
    void compute_clearance();
    void compute_run_lengths();
    void classify_cells();
    void build_components();
    void build_adjacency();
    void classify_connectors();
    void export_outputs(const SharedEnvironment* env);

    bool in_bounds(int r, int c) const;
    int loc(int r, int c) const;
    int local_free_count(int r0, int c0, int r1, int c1) const;
    int estimate_min_width(const Node& node) const;

    std::array<unsigned char, 3> cell_color(int cell) const;
    std::vector<int> collect_open_regions_through_passage(int node_id) const;
    std::string node_type_name(NodeType type) const;
    std::string sanitize_stem(std::string name) const;
    std::string json_escape(const std::string& text) const;
    void write_ppm(const std::string& path, const SharedEnvironment* env) const;
    void write_svg(const std::string& path, const SharedEnvironment* env) const;
    void write_json(const std::string& path, const SharedEnvironment* env) const;
};

} // namespace orz_engine
