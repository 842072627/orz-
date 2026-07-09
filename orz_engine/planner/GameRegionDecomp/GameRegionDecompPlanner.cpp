// optsvr/planner/GameRegionDecomp/GameRegionDecompPlanner.cpp
// First-step game planner mode: decompose the map into macro traffic nodes and
// export visualization artifacts. Runtime planning is delegated to GameOpt.
#include "GameRegionDecompPlanner.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_map>

namespace orz_engine {

namespace {

constexpr int INF = 1 << 28;

const std::array<std::array<unsigned char, 3>, 12> OPEN_PALETTE = {{
    {{130, 178, 214}}, {{142, 199, 128}}, {{221, 176, 103}},
    {{178, 143, 209}}, {{211, 134, 147}}, {{113, 194, 186}},
    {{204, 205, 104}}, {{139, 165, 225}}, {{218, 150, 113}},
    {{153, 205, 169}}, {{201, 136, 196}}, {{126, 190, 220}},
}};

bool env_flag_enabled(const char* name, bool fallback)
{
    const char* v = std::getenv(name);
    if (v == nullptr) return fallback;
    std::string s(v);
    return s == "1" || s == "true" || s == "TRUE" || s == "on" || s == "ON";
}

int env_int_value(const char* name, int fallback)
{
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') return fallback;
    char* end = nullptr;
    const long parsed = std::strtol(v, &end, 10);
    if (end == v) return fallback;
    return static_cast<int>(std::max<long>(1, parsed));
}

unsigned char clamp_color(int value)
{
    return static_cast<unsigned char>(std::max(55, std::min(235, value)));
}

std::array<unsigned char, 3> open_region_color(int id)
{
    const auto base = OPEN_PALETTE[static_cast<size_t>(id) % OPEN_PALETTE.size()];
    const int variant = (id / static_cast<int>(OPEN_PALETTE.size())) % 5;
    const int delta[5] = {0, -28, 26, -14, 14};
    return {{
        clamp_color(static_cast<int>(base[0]) + delta[variant]),
        clamp_color(static_cast<int>(base[1]) + delta[(variant + 2) % 5]),
        clamp_color(static_cast<int>(base[2]) + delta[(variant + 4) % 5]),
    }};
}

} // namespace

void GameRegionDecompPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    env_ = env;
    build_decomposition(env);
    export_outputs(env);
    exported_ = true;

    if (env_flag_enabled("GAME_DECOMP_DELEGATE_GAMEOPT", true)) {
        fallback_ = std::make_unique<GameOptPlanner>();
        fallback_->initialize(preprocess_time_limit, env);
    }
}

void GameRegionDecompPlanner::plan(int time_limit,
                                   std::vector<std::vector<Action>>& actions,
                                   SharedEnvironment* env,
                                   int num_steps)
{
    if (fallback_) {
        fallback_->plan(time_limit, actions, env, num_steps);
        return;
    }

    actions.assign(env->num_of_agents, std::vector<Action>(std::max(0, num_steps), Action::W));
}

void GameRegionDecompPlanner::build_decomposition(const SharedEnvironment* env)
{
    rows_ = env->rows;
    cols_ = env->cols;
    total_ = rows_ * cols_;
    major_region_min_size_ =
        env_int_value("GAME_DECOMP_MAJOR_REGION_MIN_SIZE", DEFAULT_MAJOR_REGION_MIN_SIZE);

    free_.assign(total_, 0);
    for (int i = 0; i < total_; i++) {
        free_[i] = (env->map[i] == 0) ? 1 : 0;
    }

    compute_degree();
    compute_clearance();
    compute_run_lengths();
    classify_cells();
    build_components();
    build_adjacency();
    classify_connectors();
}

bool GameRegionDecompPlanner::in_bounds(int r, int c) const
{
    return r >= 0 && r < rows_ && c >= 0 && c < cols_;
}

int GameRegionDecompPlanner::loc(int r, int c) const
{
    return r * cols_ + c;
}

void GameRegionDecompPlanner::compute_degree()
{
    degree_.assign(total_, 0);
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            const int id = loc(r, c);
            if (!free_[id]) continue;
            int d = 0;
            for (int k = 0; k < 4; k++) {
                const int nr = r + dr[k];
                const int nc = c + dc[k];
                if (in_bounds(nr, nc) && free_[loc(nr, nc)]) d++;
            }
            degree_[id] = d;
        }
    }
}

void GameRegionDecompPlanner::compute_clearance()
{
    clearance_.assign(total_, INF);
    std::queue<int> q;

    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            const int id = loc(r, c);
            if (!free_[id]) {
                clearance_[id] = 0;
                q.push(id);
            } else if (r == 0 || c == 0 || r + 1 == rows_ || c + 1 == cols_) {
                clearance_[id] = 1;
                q.push(id);
            }
        }
    }

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    while (!q.empty()) {
        const int id = q.front();
        q.pop();
        const int r = id / cols_;
        const int c = id - r * cols_;
        const int nd = clearance_[id] + 1;
        for (int k = 0; k < 4; k++) {
            const int nr = r + dr[k];
            const int nc = c + dc[k];
            if (!in_bounds(nr, nc)) continue;
            const int nid = loc(nr, nc);
            if (nd < clearance_[nid]) {
                clearance_[nid] = nd;
                q.push(nid);
            }
        }
    }
}

void GameRegionDecompPlanner::compute_run_lengths()
{
    h_run_.assign(total_, 0);
    v_run_.assign(total_, 0);

    for (int r = 0; r < rows_; r++) {
        int c = 0;
        while (c < cols_) {
            if (!free_[loc(r, c)]) {
                c++;
                continue;
            }
            const int start = c;
            while (c < cols_ && free_[loc(r, c)]) c++;
            const int len = c - start;
            for (int cc = start; cc < c; cc++) h_run_[loc(r, cc)] = len;
        }
    }

    for (int c = 0; c < cols_; c++) {
        int r = 0;
        while (r < rows_) {
            if (!free_[loc(r, c)]) {
                r++;
                continue;
            }
            const int start = r;
            while (r < rows_ && free_[loc(r, c)]) r++;
            const int len = r - start;
            for (int rr = start; rr < r; rr++) v_run_[loc(rr, c)] = len;
        }
    }
}

int GameRegionDecompPlanner::local_free_count(int r0, int c0, int r1, int c1) const
{
    int count = 0;
    for (int r = r0; r <= r1; r++) {
        for (int c = c0; c <= c1; c++) {
            count += free_[loc(r, c)] ? 1 : 0;
        }
    }
    return count;
}

void GameRegionDecompPlanner::classify_cells()
{
    raw_kind_.assign(total_, 0); // 0=open/blocked, 1=H, 2=V, 3=mixed
    cell_kind_.assign(total_, CellKind::Blocked);

    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            const int id = loc(r, c);
            if (!free_[id]) continue;

            bool horizontal = v_run_[id] <= WIDTH_THRESHOLD && h_run_[id] >= MIN_RUN_LEN;
            bool vertical = h_run_[id] <= WIDTH_THRESHOLD && v_run_[id] >= MIN_RUN_LEN;

            if (!horizontal && !vertical) {
                const int wr = DENSITY_WINDOW_RADIUS;
                const int r0 = std::max(0, r - wr);
                const int r1 = std::min(rows_ - 1, r + wr);
                const int c0 = std::max(0, c - wr);
                const int c1 = std::min(cols_ - 1, c + wr);
                const int area = (r1 - r0 + 1) * (c1 - c0 + 1);
                const int free_count = local_free_count(r0, c0, r1, c1);
                const int min_span = std::min(h_run_[id], v_run_[id]);
                const int max_span = std::max(h_run_[id], v_run_[id]);
                const bool density_candidate =
                    free_count <= static_cast<int>(area * LOCAL_DENSITY_THRESHOLD) &&
                    min_span <= WIDTH_THRESHOLD * 2 &&
                    max_span >= MIN_RUN_LEN;

                if (density_candidate) {
                    if (h_run_[id] >= static_cast<int>(v_run_[id] * 1.25f)) horizontal = true;
                    else if (v_run_[id] >= static_cast<int>(h_run_[id] * 1.25f)) vertical = true;
                    else if (h_run_[id] >= v_run_[id]) horizontal = true;
                    else vertical = true;
                }
            }

            if (horizontal && vertical) raw_kind_[id] = 3;
            else if (horizontal) raw_kind_[id] = 1;
            else if (vertical) raw_kind_[id] = 2;
        }
    }

    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            const int id = loc(r, c);
            if (!free_[id]) {
                cell_kind_[id] = CellKind::Blocked;
                continue;
            }
            if (raw_kind_[id] == 0) {
                cell_kind_[id] = CellKind::Open;
                continue;
            }

            bool touches_orthogonal_junction = false;
            const int dr[4] = {-1, 1, 0, 0};
            const int dc[4] = {0, 0, -1, 1};
            for (int k = 0; k < 4; k++) {
                const int nr = r + dr[k];
                const int nc = c + dc[k];
                if (!in_bounds(nr, nc)) continue;
                const int nid = loc(nr, nc);
                const unsigned char nk = raw_kind_[nid];
                if (nk == 0 || nk == raw_kind_[id]) continue;
                if (degree_[id] >= 3 || degree_[nid] >= 3) {
                    touches_orthogonal_junction = true;
                    break;
                }
            }

            const bool turn_like = raw_kind_[id] == 3 ||
                                   touches_orthogonal_junction;
            if (turn_like) cell_kind_[id] = CellKind::TurnGate;
            else if (raw_kind_[id] == 1) cell_kind_[id] = CellKind::NarrowH;
            else if (raw_kind_[id] == 2) cell_kind_[id] = CellKind::NarrowV;
            else cell_kind_[id] = CellKind::Open;
        }
    }
}

int GameRegionDecompPlanner::estimate_min_width(const Node& node) const
{
    int best = INF;
    for (int r = node.r0; r <= node.r1; r++) {
        for (int c = node.c0; c <= node.c1; c++) {
            const int id = loc(r, c);
            if (component_id_[id] != node.id) continue;
            int width = std::min(h_run_[id], v_run_[id]);
            if (node.orientation == 1) width = v_run_[id];
            if (node.orientation == 2) width = h_run_[id];
            best = std::min(best, width);
        }
    }
    return best == INF ? 0 : best;
}

void GameRegionDecompPlanner::build_components()
{
    component_id_.assign(total_, -1);
    nodes_.clear();

    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};

    auto node_type_for_kind = [](CellKind kind) {
        if (kind == CellKind::TurnGate) return NodeType::TurnGate;
        if (kind == CellKind::NarrowH || kind == CellKind::NarrowV) return NodeType::NarrowSegment;
        return NodeType::OpenRegion;
    };

    for (int start = 0; start < total_; start++) {
        if (!free_[start] || component_id_[start] != -1) continue;
        const CellKind kind = cell_kind_[start];
        if (kind == CellKind::Blocked) continue;

        Node node;
        node.id = static_cast<int>(nodes_.size());
        node.type = node_type_for_kind(kind);
        node.orientation = kind == CellKind::NarrowH ? 1 : (kind == CellKind::NarrowV ? 2 : 0);
        node.r0 = node.r1 = start / cols_;
        node.c0 = node.c1 = start % cols_;
        node.min_clearance = INF;
        long long sum_r = 0;
        long long sum_c = 0;

        std::queue<int> q;
        q.push(start);
        component_id_[start] = node.id;
        while (!q.empty()) {
            const int id = q.front();
            q.pop();
            const int r = id / cols_;
            const int c = id - r * cols_;
            node.size++;
            sum_r += r;
            sum_c += c;
            node.r0 = std::min(node.r0, r);
            node.c0 = std::min(node.c0, c);
            node.r1 = std::max(node.r1, r);
            node.c1 = std::max(node.c1, c);
            node.min_clearance = std::min(node.min_clearance, clearance_[id]);

            for (int k = 0; k < 4; k++) {
                const int nr = r + dr[k];
                const int nc = c + dc[k];
                if (!in_bounds(nr, nc)) continue;
                const int nid = loc(nr, nc);
                if (!free_[nid] || component_id_[nid] != -1) continue;
                if (cell_kind_[nid] != kind) continue;
                component_id_[nid] = node.id;
                q.push(nid);
            }
        }

        if (node.size < MIN_COMPONENT_SIZE && node.type != NodeType::OpenRegion) {
            node.type = NodeType::TurnGate;
            node.orientation = 0;
        }
        node.min_width = estimate_min_width(node);
        if (node.type == NodeType::NarrowSegment) {
            node.capacity = std::max(1, std::min(4, node.min_width / 2));
        } else if (node.type == NodeType::TurnGate) {
            node.capacity = 1;
        } else {
            node.capacity = std::max(4, node.min_width);
        }
        if (node.min_clearance == INF) node.min_clearance = 0;
        if (node.size > 0) {
            node.centroid_r = static_cast<int>(sum_r / node.size);
            node.centroid_c = static_cast<int>(sum_c / node.size);
        }
        nodes_.push_back(std::move(node));
    }
}

void GameRegionDecompPlanner::build_adjacency()
{
    const int dr[4] = {-1, 1, 0, 0};
    const int dc[4] = {0, 0, -1, 1};
    for (int r = 0; r < rows_; r++) {
        for (int c = 0; c < cols_; c++) {
            const int id = loc(r, c);
            const int a = component_id_[id];
            if (a < 0) continue;
            for (int k = 0; k < 4; k++) {
                const int nr = r + dr[k];
                const int nc = c + dc[k];
                if (!in_bounds(nr, nc)) continue;
                const int b = component_id_[loc(nr, nc)];
                if (b < 0 || b == a) continue;
                nodes_[a].adjacent.insert(b);
            }
        }
    }
}

std::vector<int> GameRegionDecompPlanner::collect_open_regions_through_passage(int node_id) const
{
    std::set<int> open_regions;
    if (node_id < 0 || node_id >= static_cast<int>(nodes_.size())) return {};

    std::queue<int> q;
    std::vector<unsigned char> seen(nodes_.size(), 0);
    q.push(node_id);
    seen[node_id] = 1;

    int visited_passage_nodes = 0;
    while (!q.empty()) {
        const int cur = q.front();
        q.pop();
        visited_passage_nodes++;
        if (visited_passage_nodes > 256) break;

        for (int nb : nodes_[cur].adjacent) {
            if (nb < 0 || nb >= static_cast<int>(nodes_.size())) continue;
            const Node& other = nodes_[nb];
            if (other.type == NodeType::OpenRegion) {
                open_regions.insert(nb);
                continue;
            }
            if (seen[nb]) continue;
            // Passage-local flood: narrow segments and turn gates form one
            // bottleneck subgraph; open regions are collected but not crossed.
            seen[nb] = 1;
            q.push(nb);
        }
    }

    return std::vector<int>(open_regions.begin(), open_regions.end());
}

void GameRegionDecompPlanner::classify_connectors()
{
    for (Node& node : nodes_) {
        node.major_region = node.type == NodeType::OpenRegion &&
                            node.size >= major_region_min_size_;
        node.connects_regions = false;
        node.connected_open_regions.clear();
        node.connected_major_open_regions.clear();
    }

    for (Node& node : nodes_) {
        if (node.type == NodeType::OpenRegion) {
            node.connects_regions = node.major_region;
            node.connected_open_regions = {node.id};
            if (node.major_region) node.connected_major_open_regions = {node.id};
            continue;
        }
        node.connected_open_regions = collect_open_regions_through_passage(node.id);
        for (int region_id : node.connected_open_regions) {
            if (region_id < 0 || region_id >= static_cast<int>(nodes_.size())) continue;
            const Node& region = nodes_[region_id];
            if (region.type == NodeType::OpenRegion && region.major_region) {
                node.connected_major_open_regions.push_back(region_id);
            }
        }
        node.connects_regions = node.connected_major_open_regions.size() >= 2;
    }
}

std::string GameRegionDecompPlanner::sanitize_stem(std::string name) const
{
    if (name.empty()) name = "game_map";
    const size_t slash = name.find_last_of("/\\");
    if (slash != std::string::npos) name = name.substr(slash + 1);
    const size_t dot = name.find_last_of('.');
    if (dot != std::string::npos) name = name.substr(0, dot);
    for (char& ch : name) {
        const unsigned char u = static_cast<unsigned char>(ch);
        if (!std::isalnum(u) && ch != '-' && ch != '_') ch = '_';
    }
    return name.empty() ? "game_map" : name;
}

void GameRegionDecompPlanner::export_outputs(const SharedEnvironment* env)
{
    const char* out_dir = std::getenv("GAME_DECOMP_OUT_DIR");
    output_dir_ = out_dir != nullptr ? out_dir : "result/game_decomp";
    output_stem_ = sanitize_stem(env->map_name);
    std::filesystem::create_directories(output_dir_);

    const std::filesystem::path base = std::filesystem::path(output_dir_) / output_stem_;
    const std::string svg_path = base.string() + "_decomp.svg";
    const std::string ppm_path = base.string() + "_decomp.ppm";
    const std::string json_path = base.string() + "_decomp.json";

    write_svg(svg_path, env);
    write_ppm(ppm_path, env);
    write_json(json_path, env);

    int open_count = 0, major_open_count = 0, narrow_count = 0, turn_count = 0, connector_count = 0;
    int open_cells = 0, major_open_cells = 0, narrow_cells = 0, turn_cells = 0, connector_cells = 0;
    for (const Node& node : nodes_) {
        if (node.type == NodeType::OpenRegion) {
            open_count++;
            open_cells += node.size;
            if (node.major_region) {
                major_open_count++;
                major_open_cells += node.size;
            }
        } else if (node.type == NodeType::NarrowSegment) {
            narrow_count++;
            narrow_cells += node.size;
            if (node.connects_regions) {
                connector_count++;
                connector_cells += node.size;
            }
        } else {
            turn_count++;
            turn_cells += node.size;
        }
    }

    std::cerr << "[GameRegionDecomp] exported"
              << " nodes=" << nodes_.size()
              << " open=" << open_count << "/" << open_cells
              << " major_open=" << major_open_count << "/" << major_open_cells
              << " major_min_size=" << major_region_min_size_
              << " narrow=" << narrow_count << "/" << narrow_cells
              << " connector_narrow=" << connector_count << "/" << connector_cells
              << " turns=" << turn_count << "/" << turn_cells
              << " svg=" << svg_path
              << " json=" << json_path
              << "\n";
}

std::array<unsigned char, 3> GameRegionDecompPlanner::cell_color(int cell) const
{
    if (cell < 0 || cell >= total_ || !free_[cell]) return {{30, 30, 30}};
    const int cid = component_id_[cell];
    if (cid < 0 || cid >= static_cast<int>(nodes_.size())) return {{226, 226, 220}};
    const Node& node = nodes_[cid];
    if (node.type == NodeType::TurnGate) return {{224, 68, 78}};
    if (node.type == NodeType::NarrowSegment) {
        if (!node.connects_regions) return {{145, 145, 145}};
        return {{132, 0, 0}};
    }
    if (!node.major_region) return {{178, 178, 170}};
    return open_region_color(node.id);
}

void GameRegionDecompPlanner::write_ppm(const std::string& path,
                                        const SharedEnvironment* env) const
{
    std::vector<unsigned char> rgb(static_cast<size_t>(total_) * 3);
    for (int i = 0; i < total_; i++) {
        const auto color = cell_color(i);
        rgb[static_cast<size_t>(i) * 3 + 0] = color[0];
        rgb[static_cast<size_t>(i) * 3 + 1] = color[1];
        rgb[static_cast<size_t>(i) * 3 + 2] = color[2];
    }

    for (const State& state : env->curr_states) {
        const int id = state.location;
        if (id < 0 || id >= total_) continue;
        rgb[static_cast<size_t>(id) * 3 + 0] = 118;
        rgb[static_cast<size_t>(id) * 3 + 1] = 64;
        rgb[static_cast<size_t>(id) * 3 + 2] = 162;
    }

    std::ofstream out(path, std::ios::binary);
    out << "P6\n" << cols_ << " " << rows_ << "\n255\n";
    out.write(reinterpret_cast<const char*>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
}

void GameRegionDecompPlanner::write_svg(const std::string& path,
                                        const SharedEnvironment* env) const
{
    std::vector<std::array<unsigned char, 3>> colors(total_);
    for (int i = 0; i < total_; i++) colors[i] = cell_color(i);
    for (const State& state : env->curr_states) {
        const int id = state.location;
        if (id >= 0 && id < total_) colors[id] = {{118, 64, 162}};
    }

    auto same_color = [&](const std::array<unsigned char, 3>& a,
                          const std::array<unsigned char, 3>& b) {
        return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
    };

    std::ofstream out(path);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << cols_
        << "\" height=\"" << rows_ << "\" viewBox=\"0 0 " << cols_ << " " << rows_
        << "\" shape-rendering=\"crispEdges\">\n";
    out << "<title>Game region decomposition</title>\n";
    out << "<desc>Open regions use varied colors, connector passages are dark red, "
           "minor open pockets and internal narrow cells are muted gray, turn gates are bright red, "
           "agent starts are purple.</desc>\n";
    out << "<rect x=\"0\" y=\"0\" width=\"" << cols_ << "\" height=\"" << rows_
        << "\" fill=\"rgb(30,30,30)\"/>\n";

    const std::array<unsigned char, 3> blocked = {{30, 30, 30}};
    for (int r = 0; r < rows_; r++) {
        int c = 0;
        while (c < cols_) {
            const auto color = colors[loc(r, c)];
            if (same_color(color, blocked)) {
                c++;
                continue;
            }
            const int start = c;
            c++;
            while (c < cols_ && same_color(colors[loc(r, c)], color)) c++;
            out << "<rect x=\"" << start << "\" y=\"" << r
                << "\" width=\"" << (c - start) << "\" height=\"1\" fill=\"rgb("
                << static_cast<int>(color[0]) << ","
                << static_cast<int>(color[1]) << ","
                << static_cast<int>(color[2]) << ")\"/>\n";
        }
    }

    out << "</svg>\n";
}

std::string GameRegionDecompPlanner::node_type_name(NodeType type) const
{
    switch (type) {
        case NodeType::OpenRegion: return "open_region";
        case NodeType::NarrowSegment: return "narrow_segment";
        case NodeType::TurnGate: return "turn_gate";
    }
    return "unknown";
}

std::string GameRegionDecompPlanner::json_escape(const std::string& text) const
{
    std::ostringstream out;
    for (char ch : text) {
        switch (ch) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << ch; break;
        }
    }
    return out.str();
}

void GameRegionDecompPlanner::write_json(const std::string& path,
                                         const SharedEnvironment* env) const
{
    int free_cells = 0;
    int open_cells = 0;
    int major_open_cells = 0;
    int major_open_count = 0;
    int narrow_cells = 0;
    int turn_cells = 0;
    int connector_narrow_count = 0;
    int connector_narrow_cells = 0;
    int edge_count = 0;
    for (int i = 0; i < total_; i++) {
        if (!free_[i]) continue;
        free_cells++;
    }
    for (const Node& node : nodes_) {
        if (node.type == NodeType::OpenRegion) {
            open_cells += node.size;
            if (node.major_region) {
                major_open_count++;
                major_open_cells += node.size;
            }
        }
        else if (node.type == NodeType::NarrowSegment) {
            narrow_cells += node.size;
            if (node.connects_regions) {
                connector_narrow_count++;
                connector_narrow_cells += node.size;
            }
        }
        else if (node.type == NodeType::TurnGate) turn_cells += node.size;
    }
    for (const Node& node : nodes_) edge_count += static_cast<int>(node.adjacent.size());
    edge_count /= 2;

    std::ofstream out(path);
    out << "{\n";
    out << "  \"schema\": \"game_region_decomp_v1\",\n";
    out << "  \"map_name\": \"" << json_escape(env->map_name) << "\",\n";
    out << "  \"rows\": " << rows_ << ",\n";
    out << "  \"cols\": " << cols_ << ",\n";
    out << "  \"free_cells\": " << free_cells << ",\n";
    out << "  \"blocked_cells\": " << (total_ - free_cells) << ",\n";
    out << "  \"params\": {\n";
    out << "    \"width_threshold\": " << WIDTH_THRESHOLD << ",\n";
    out << "    \"min_run_len\": " << MIN_RUN_LEN << ",\n";
    out << "    \"density_window_radius\": " << DENSITY_WINDOW_RADIUS << ",\n";
    out << "    \"local_density_threshold\": " << LOCAL_DENSITY_THRESHOLD << ",\n";
    out << "    \"major_region_min_size\": " << major_region_min_size_ << "\n";
    out << "  },\n";
    out << "  \"cell_counts\": {\n";
    out << "    \"open\": " << open_cells << ",\n";
    out << "    \"major_open\": " << major_open_cells << ",\n";
    out << "    \"narrow\": " << narrow_cells << ",\n";
    out << "    \"turn_gate\": " << turn_cells << ",\n";
    out << "    \"connector_narrow\": " << connector_narrow_cells << "\n";
    out << "  },\n";
    out << "  \"graph\": {\n";
    out << "    \"node_count\": " << nodes_.size() << ",\n";
    out << "    \"edge_count\": " << edge_count << ",\n";
    out << "    \"major_open_count\": " << major_open_count << ",\n";
    out << "    \"connector_narrow_count\": " << connector_narrow_count << "\n";
    out << "  },\n";
    out << "  \"nodes\": [\n";
    for (size_t i = 0; i < nodes_.size(); i++) {
        const Node& node = nodes_[i];
        out << "    {\n";
        out << "      \"id\": " << node.id << ",\n";
        out << "      \"type\": \"" << node_type_name(node.type) << "\",\n";
        out << "      \"orientation\": " << node.orientation << ",\n";
        out << "      \"size\": " << node.size << ",\n";
        out << "      \"bbox\": [" << node.r0 << ", " << node.c0 << ", "
            << node.r1 << ", " << node.c1 << "],\n";
        out << "      \"min_clearance\": " << node.min_clearance << ",\n";
        out << "      \"min_width\": " << node.min_width << ",\n";
        out << "      \"capacity\": " << node.capacity << ",\n";
        out << "      \"centroid\": [" << node.centroid_r << ", " << node.centroid_c << "],\n";
        out << "      \"major_region\": " << (node.major_region ? "true" : "false") << ",\n";
        out << "      \"connects_regions\": " << (node.connects_regions ? "true" : "false") << ",\n";
        out << "      \"connected_open_regions\": [";
        for (size_t ri = 0; ri < node.connected_open_regions.size(); ri++) {
            if (ri > 0) out << ", ";
            out << node.connected_open_regions[ri];
        }
        out << "],\n";
        out << "      \"connected_major_open_regions\": [";
        for (size_t ri = 0; ri < node.connected_major_open_regions.size(); ri++) {
            if (ri > 0) out << ", ";
            out << node.connected_major_open_regions[ri];
        }
        out << "],\n";
        out << "      \"adjacent\": [";
        bool first = true;
        for (int nb : node.adjacent) {
            if (!first) out << ", ";
            first = false;
            out << nb;
        }
        out << "]\n";
        out << "    }" << (i + 1 == nodes_.size() ? "\n" : ",\n");
    }
    out << "  ]\n";
    out << "}\n";
}

} // namespace orz_engine
