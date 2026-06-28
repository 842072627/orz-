#include "RegionLoadModel.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_set>

namespace optsvr {

namespace {

bool env_flag_enabled(const char* name) {
    const char* v = std::getenv(name);
    if (v == nullptr) return false;
    std::string s(v);
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return s == "1" || s == "true" || s == "yes" || s == "on";
}

int env_int(const char* name, int fallback) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') return fallback;
    return std::max(0, std::atoi(v));
}

double env_double(const char* name, double fallback) {
    const char* v = std::getenv(name);
    if (v == nullptr || *v == '\0') return fallback;
    return std::max(0.0, std::atof(v));
}

std::string env_string(const char* name, const std::string& fallback) {
    const char* v = std::getenv(name);
    return (v == nullptr || *v == '\0') ? fallback : std::string(v);
}

RegionLoadMode parse_mode(const std::string& raw) {
    std::string s = raw;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    if (s == "cell" || s == "cells" || s == "full") return RegionLoadMode::Cell;
    return RegionLoadMode::Adaptive;
}

std::string lower_string(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return s;
}

}  // namespace

RegionLoadConfig RegionLoadConfig::from_env() {
    RegionLoadConfig cfg;
    cfg.enabled = env_flag_enabled("OPT_REGION_MODEL_ENABLE");
    cfg.mode = parse_mode(env_string("OPT_REGION_MODEL_MODE", "adaptive"));
    cfg.target_regions = env_int("OPT_REGION_TARGET", cfg.target_regions);
    cfg.max_regions = env_int("OPT_REGION_MAX_REGIONS", cfg.max_regions);
    cfg.max_distance_regions = env_int("OPT_REGION_MAX_DISTANCE_REGIONS",
                                        cfg.max_distance_regions);
    if (std::getenv("OPT_REGION_TENDENCY_ENABLE") != nullptr) {
        cfg.build_tendency = env_flag_enabled("OPT_REGION_TENDENCY_ENABLE");
    }
    cfg.max_tendency_regions = env_int("OPT_REGION_TENDENCY_MAX_REGIONS",
                                       cfg.max_tendency_regions);
    cfg.tendency_decay = env_double("OPT_REGION_TENDENCY_DECAY",
                                    cfg.tendency_decay);
    if (std::getenv("OPT_REGION_TENDENCY_SIZE_WEIGHTED") != nullptr) {
        cfg.tendency_size_weighted =
            env_flag_enabled("OPT_REGION_TENDENCY_SIZE_WEIGHTED");
    }
    cfg.tendency_kind = lower_string(env_string("OPT_REGION_TENDENCY_KIND",
                                                cfg.tendency_kind));
    cfg.bottleneck_narrow_weight = env_double("OPT_REGION_BOTTLENECK_NARROW_WEIGHT",
                                              cfg.bottleneck_narrow_weight);
    cfg.bottleneck_capacity_power = env_double("OPT_REGION_BOTTLENECK_CAPACITY_POWER",
                                               cfg.bottleneck_capacity_power);
    cfg.export_dir = env_string("OPT_REGION_EXPORT_DIR", cfg.export_dir);

    if (cfg.target_regions <= 0) cfg.target_regions = 100;
    if (cfg.max_regions <= 0) cfg.max_regions = 256;
    if (cfg.max_distance_regions <= 0) cfg.max_distance_regions = 512;
    if (cfg.max_tendency_regions <= 0) cfg.max_tendency_regions = 512;
    if (cfg.tendency_kind != "soft" && cfg.tendency_kind != "hybrid") {
        cfg.tendency_kind = "bottleneck";
    }
    return cfg;
}

std::string RegionLoadConfig::mode_name() const {
    return mode == RegionLoadMode::Cell ? "cell" : "adaptive";
}

RegionLoadModel& region_load_model() {
    static RegionLoadModel model;
    return model;
}

void RegionLoadModel::clear() {
    ready_ = false;
    distances_ready_ = false;
    tendency_ready_ = false;
    rows_ = cols_ = map_size_ = n_free_ = region_count_ = 0;
    tendency_sum_ = tendency_max_ = tendency_pair_weight_sum_ = 0.0;
    tendency_pair_count_ = 0;
    map_name_.clear();
    is_free_.clear();
    free_locs_.clear();
    loc_to_region_.clear();
    regions_.clear();
    region_adj_.clear();
    region_dist_.clear();
    region_tendency_.clear();
    region_tendency_normalized_.clear();
    region_tendency_share_.clear();
    region_soft_tendency_.clear();
    region_soft_tendency_normalized_.clear();
    region_bottleneck_tendency_.clear();
    region_bottleneck_tendency_normalized_.clear();
    region_constriction_.clear();
    region_avg_free_degree_.clear();
}

void RegionLoadModel::initialize(const SharedEnvironment* env,
                                 const RegionLoadConfig& config) {
    clear();
    config_ = config;
    if (env == nullptr) return;

    rows_ = env->rows;
    cols_ = env->cols;
    map_size_ = rows_ * cols_;
    map_name_ = env->map_name;
    if (rows_ <= 0 || cols_ <= 0 || map_size_ <= 0 ||
        (int)env->map.size() < map_size_) {
        return;
    }

    is_free_.assign(map_size_, 0);
    loc_to_region_.assign(map_size_, -1);
    free_locs_.reserve(map_size_);

    for (int loc = 0; loc < map_size_; ++loc) {
        if (env->map[loc] == 0) {
            is_free_[loc] = 1;
            free_locs_.push_back(loc);
        }
    }
    n_free_ = (int)free_locs_.size();
    if (n_free_ == 0) return;

    if (config_.mode == RegionLoadMode::Cell) {
        build_cell_regions();
    } else {
        build_adaptive_regions();
    }

    rebuild_region_stats();
    build_adjacency();
    compute_all_pairs_region_dist();
    compute_region_tendency();
    ready_ = true;
}

bool RegionLoadModel::is_free_loc(int loc) const {
    return loc >= 0 && loc < map_size_ && !is_free_.empty() && is_free_[loc] != 0;
}

int RegionLoadModel::choose_region_count() const {
    if (n_free_ <= 0) return 0;
    int k = std::max(1, config_.target_regions);
    k = std::min(k, std::max(1, config_.max_regions));
    k = std::min(k, n_free_);
    return k;
}

void RegionLoadModel::build_cell_regions() {
    region_count_ = n_free_;
    regions_.assign(region_count_, RegionInfo{});
    for (int rid = 0; rid < region_count_; ++rid) {
        int loc = free_locs_[rid];
        loc_to_region_[loc] = rid;
        regions_[rid].id = rid;
        regions_[rid].seed_loc = loc;
    }
}

std::vector<std::vector<int>> RegionLoadModel::collect_components() const {
    std::vector<std::vector<int>> components;
    std::vector<unsigned char> seen(map_size_, 0);
    std::deque<int> q;

    for (int start : free_locs_) {
        if (seen[start]) continue;
        components.emplace_back();
        auto& cells = components.back();
        seen[start] = 1;
        q.push_back(start);
        while (!q.empty()) {
            int cur = q.front();
            q.pop_front();
            cells.push_back(cur);
            int r = cur / cols_;
            int c = cur % cols_;
            const int nbs[4] = {
                (c + 1 < cols_) ? cur + 1 : -1,
                (r + 1 < rows_) ? cur + cols_ : -1,
                (c > 0) ? cur - 1 : -1,
                (r > 0) ? cur - cols_ : -1,
            };
            for (int nb : nbs) {
                if (nb >= 0 && is_free_loc(nb) && !seen[nb]) {
                    seen[nb] = 1;
                    q.push_back(nb);
                }
            }
        }
    }

    std::sort(components.begin(), components.end(),
              [](const auto& a, const auto& b) {
                  if (a.size() != b.size()) return a.size() > b.size();
                  return !a.empty() && !b.empty() && a.front() < b.front();
              });
    return components;
}

std::vector<int> RegionLoadModel::select_component_seeds(
    const std::vector<int>& cells, int k) const {
    std::vector<int> seeds;
    if (cells.empty() || k <= 0) return seeds;
    k = std::min(k, (int)cells.size());

    int min_r = rows_, max_r = -1, min_c = cols_, max_c = -1;
    for (int loc : cells) {
        int r = loc / cols_, c = loc % cols_;
        min_r = std::min(min_r, r); max_r = std::max(max_r, r);
        min_c = std::min(min_c, c); max_c = std::max(max_c, c);
    }

    int h = std::max(1, max_r - min_r + 1);
    int w = std::max(1, max_c - min_c + 1);
    int grid_r = std::max(1, (int)std::round(std::sqrt((double)k * h / w)));
    int grid_c = std::max(1, (k + grid_r - 1) / grid_r);
    while (grid_r * grid_c < k) ++grid_c;

    int buckets = grid_r * grid_c;
    std::vector<int> best_loc(buckets, -1);
    std::vector<long long> best_score(buckets, std::numeric_limits<long long>::max());

    for (int loc : cells) {
        int r = loc / cols_, c = loc % cols_;
        int br = std::min(grid_r - 1, std::max(0, (r - min_r) * grid_r / h));
        int bc = std::min(grid_c - 1, std::max(0, (c - min_c) * grid_c / w));
        int bid = br * grid_c + bc;
        double cr = min_r + (br + 0.5) * (double)h / grid_r;
        double cc = min_c + (bc + 0.5) * (double)w / grid_c;
        long long dr = (long long)std::llround((r - cr) * 1000.0);
        long long dc = (long long)std::llround((c - cc) * 1000.0);
        long long score = dr * dr + dc * dc;
        if (score < best_score[bid] ||
            (score == best_score[bid] && (best_loc[bid] < 0 || loc < best_loc[bid]))) {
            best_score[bid] = score;
            best_loc[bid] = loc;
        }
    }

    std::unordered_set<int> used;
    seeds.reserve(k);
    for (int loc : best_loc) {
        if (loc >= 0 && used.insert(loc).second) {
            seeds.push_back(loc);
            if ((int)seeds.size() == k) return seeds;
        }
    }

    for (int i = 0; (int)seeds.size() < k && i < k * 4; ++i) {
        int idx = (int)(((long long)i * (long long)cells.size() + cells.size() / 2) / k);
        idx = std::min((int)cells.size() - 1, std::max(0, idx));
        int loc = cells[idx];
        if (used.insert(loc).second) seeds.push_back(loc);
    }
    for (int loc : cells) {
        if ((int)seeds.size() == k) break;
        if (used.insert(loc).second) seeds.push_back(loc);
    }
    return seeds;
}

void RegionLoadModel::build_adaptive_regions() {
    auto components = collect_components();
    if (components.empty()) return;

    int target_k = choose_region_count();
    target_k = std::max(target_k, (int)components.size());
    target_k = std::min(target_k, n_free_);

    std::vector<int> comp_k(components.size(), 1);
    int remaining = target_k - (int)components.size();
    struct Fraction {
        int idx;
        double frac;
    };
    std::vector<Fraction> fractions;
    fractions.reserve(components.size());

    for (int i = 0; i < (int)components.size(); ++i) {
        double exact = (double)components[i].size() * target_k / std::max(1, n_free_);
        int add = std::max(0, (int)std::floor(exact) - 1);
        add = std::min(add, remaining);
        add = std::min(add, (int)components[i].size() - comp_k[i]);
        comp_k[i] += add;
        remaining -= add;
        fractions.push_back({i, exact - std::floor(exact)});
    }
    std::sort(fractions.begin(), fractions.end(),
              [](const Fraction& a, const Fraction& b) {
                  if (a.frac != b.frac) return a.frac > b.frac;
                  return a.idx < b.idx;
              });
    for (const auto& f : fractions) {
        if (remaining <= 0) break;
        int cap = (int)components[f.idx].size() - comp_k[f.idx];
        if (cap <= 0) continue;
        comp_k[f.idx]++;
        remaining--;
    }

    std::deque<int> q;
    region_count_ = 0;
    for (int ci = 0; ci < (int)components.size(); ++ci) {
        std::vector<int> seeds = select_component_seeds(components[ci], comp_k[ci]);
        for (int seed : seeds) {
            int rid = region_count_++;
            loc_to_region_[seed] = rid;
            q.push_back(seed);
        }
    }
    regions_.assign(region_count_, RegionInfo{});
    for (int loc = 0; loc < map_size_; ++loc) {
        int rid = loc_to_region_[loc];
        if (rid >= 0) {
            regions_[rid].id = rid;
            regions_[rid].seed_loc = loc;
        }
    }

    while (!q.empty()) {
        int cur = q.front();
        q.pop_front();
        int rid = loc_to_region_[cur];
        int r = cur / cols_;
        int c = cur % cols_;
        const int nbs[4] = {
            (c + 1 < cols_) ? cur + 1 : -1,
            (r + 1 < rows_) ? cur + cols_ : -1,
            (c > 0) ? cur - 1 : -1,
            (r > 0) ? cur - cols_ : -1,
        };
        for (int nb : nbs) {
            if (nb >= 0 && is_free_loc(nb) && loc_to_region_[nb] < 0) {
                loc_to_region_[nb] = rid;
                q.push_back(nb);
            }
        }
    }
}

void RegionLoadModel::rebuild_region_stats() {
    if (region_count_ <= 0) {
        regions_.clear();
        return;
    }
    regions_.assign(region_count_, RegionInfo{});
    std::vector<long long> sum_r(region_count_, 0), sum_c(region_count_, 0);

    for (int loc : free_locs_) {
        int rid = loc_to_region_[loc];
        if (rid < 0 || rid >= region_count_) continue;
        auto& info = regions_[rid];
        info.id = rid;
        if (info.seed_loc < 0) info.seed_loc = loc;
        info.size++;
        int r = loc / cols_, c = loc % cols_;
        sum_r[rid] += r;
        sum_c[rid] += c;
        info.min_row = std::min(info.min_row, r);
        info.max_row = std::max(info.max_row, r);
        info.min_col = std::min(info.min_col, c);
        info.max_col = std::max(info.max_col, c);
    }

    for (int loc : free_locs_) {
        int rid = loc_to_region_[loc];
        if (rid < 0 || rid >= region_count_) continue;
        int r = loc / cols_, c = loc % cols_;
        const int nbs[4] = {
            (c + 1 < cols_) ? loc + 1 : -1,
            (r + 1 < rows_) ? loc + cols_ : -1,
            (c > 0) ? loc - 1 : -1,
            (r > 0) ? loc - cols_ : -1,
        };
        bool boundary = false;
        for (int nb : nbs) {
            if (nb < 0 || !is_free_loc(nb) || loc_to_region_[nb] != rid) {
                boundary = true;
                break;
            }
        }
        if (boundary) regions_[rid].boundary_cells++;
    }

    for (auto& info : regions_) {
        if (info.size > 0) {
            info.centroid_row = (double)sum_r[info.id] / info.size;
            info.centroid_col = (double)sum_c[info.id] / info.size;
        } else {
            info.min_row = info.min_col = 0;
        }
    }
}

void RegionLoadModel::build_adjacency() {
    region_adj_.assign(region_count_, {});
    if (region_count_ <= 0) return;
    std::vector<std::unordered_set<int>> adj_sets(region_count_);

    for (int loc : free_locs_) {
        int r1 = loc_to_region_[loc];
        if (r1 < 0) continue;
        int row = loc / cols_, col = loc % cols_;
        const int nbs[2] = {
            (col + 1 < cols_) ? loc + 1 : -1,
            (row + 1 < rows_) ? loc + cols_ : -1,
        };
        for (int nb : nbs) {
            if (nb < 0 || !is_free_loc(nb)) continue;
            int r2 = loc_to_region_[nb];
            if (r2 >= 0 && r1 != r2) {
                adj_sets[r1].insert(r2);
                adj_sets[r2].insert(r1);
            }
        }
    }

    for (int r = 0; r < region_count_; ++r) {
        region_adj_[r].assign(adj_sets[r].begin(), adj_sets[r].end());
        std::sort(region_adj_[r].begin(), region_adj_[r].end());
    }
}

void RegionLoadModel::compute_all_pairs_region_dist() {
    distances_ready_ = false;
    region_dist_.clear();
    if (region_count_ <= 0 || region_count_ > config_.max_distance_regions) return;

    region_dist_.assign(region_count_, std::vector<int>(region_count_, INT_MAX));
    std::queue<int> q;
    for (int src = 0; src < region_count_; ++src) {
        auto& dist = region_dist_[src];
        dist[src] = 0;
        q.push(src);
        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            int nd = dist[cur] + 1;
            for (int nb : region_adj_[cur]) {
                if (dist[nb] == INT_MAX) {
                    dist[nb] = nd;
                    q.push(nb);
                }
            }
        }
    }
    distances_ready_ = true;
}

void RegionLoadModel::compute_region_tendency() {
    tendency_ready_ = false;
    tendency_sum_ = tendency_max_ = tendency_pair_weight_sum_ = 0.0;
    tendency_pair_count_ = 0;
    region_tendency_.assign(region_count_, 0.0);
    region_tendency_normalized_.assign(region_count_, 0.0);
    region_tendency_share_.assign(region_count_, 0.0);
    region_soft_tendency_.assign(region_count_, 0.0);
    region_soft_tendency_normalized_.assign(region_count_, 0.0);
    region_bottleneck_tendency_.assign(region_count_, 0.0);
    region_bottleneck_tendency_normalized_.assign(region_count_, 0.0);
    compute_region_constriction();

    if (!config_.build_tendency || region_count_ <= 0) return;
    if (!distances_ready_ || region_count_ > config_.max_tendency_regions) return;

    std::vector<std::vector<double>> sigma(region_count_,
                                           std::vector<double>(region_count_, 0.0));
    for (int src = 0; src < region_count_; ++src) {
        std::vector<int> dist(region_count_, INT_MAX);
        std::queue<int> q;
        dist[src] = 0;
        sigma[src][src] = 1.0;
        q.push(src);
        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            const int nd = dist[cur] + 1;
            for (int nb : region_adj_[cur]) {
                if (dist[nb] == INT_MAX) {
                    dist[nb] = nd;
                    q.push(nb);
                }
                if (dist[nb] == nd) {
                    sigma[src][nb] += sigma[src][cur];
                }
            }
        }
    }

    for (int s = 0; s < region_count_; ++s) {
        for (int t = 0; t < region_count_; ++t) {
            const int d0 = region_dist_[s][t];
            if (d0 == INT_MAX) continue;

            double pair_weight = 1.0;
            if (config_.tendency_size_weighted) {
                pair_weight = (double)regions_[s].size * (double)regions_[t].size;
            }
            if (pair_weight <= 0.0) continue;

            std::vector<double> soft_weights(region_count_, 0.0);
            double soft_sum = 0.0;
            for (int x = 0; x < region_count_; ++x) {
                const int dsx = region_dist_[s][x];
                const int dxt = region_dist_[x][t];
                if (dsx == INT_MAX || dxt == INT_MAX) continue;
                int extra = dsx + dxt - d0;
                if (extra < 0) extra = 0;
                const double w = std::exp(-config_.tendency_decay * (double)extra);
                soft_weights[x] = w;
                soft_sum += w;
            }
            if (soft_sum > 0.0) {
                for (int x = 0; x < region_count_; ++x) {
                    if (soft_weights[x] > 0.0) {
                        region_soft_tendency_[x] += pair_weight * soft_weights[x] / soft_sum;
                    }
                }
            }

            std::vector<double> bottleneck_weights(region_count_, 0.0);
            double bottleneck_sum = 0.0;
            const double denom = sigma[s][t];
            if (denom > 0.0) {
                for (int x = 0; x < region_count_; ++x) {
                    const int dsx = region_dist_[s][x];
                    const int dxt = region_dist_[x][t];
                    if (dsx == INT_MAX || dxt == INT_MAX || dsx + dxt != d0) continue;
                    double shortest_share = sigma[s][x] * sigma[x][t] / denom;
                    if (!std::isfinite(shortest_share) || shortest_share <= 0.0) continue;
                    const double narrow_factor =
                        1.0 + config_.bottleneck_narrow_weight * region_constriction_[x];
                    const double w = shortest_share * narrow_factor;
                    bottleneck_weights[x] = w;
                    bottleneck_sum += w;
                }
            }
            if (bottleneck_sum > 0.0) {
                for (int x = 0; x < region_count_; ++x) {
                    if (bottleneck_weights[x] > 0.0) {
                        region_bottleneck_tendency_[x] +=
                            pair_weight * bottleneck_weights[x] / bottleneck_sum;
                    }
                }
            }
            tendency_pair_weight_sum_ += pair_weight;
            tendency_pair_count_++;
        }
    }

    normalize_metric(region_soft_tendency_, region_soft_tendency_normalized_);
    normalize_metric(region_bottleneck_tendency_, region_bottleneck_tendency_normalized_);

    if (config_.tendency_kind == "soft") {
        region_tendency_ = region_soft_tendency_;
    } else if (config_.tendency_kind == "hybrid") {
        for (int i = 0; i < region_count_; ++i) {
            const double capacity_factor = std::pow(
                1.0 + config_.bottleneck_narrow_weight * region_constriction_[i],
                config_.bottleneck_capacity_power);
            region_tendency_[i] = 0.5 * region_soft_tendency_[i] +
                                  0.5 * region_bottleneck_tendency_[i] * capacity_factor;
        }
    } else {
        for (int i = 0; i < region_count_; ++i) {
            const double capacity_factor = std::pow(
                1.0 + config_.bottleneck_narrow_weight * region_constriction_[i],
                config_.bottleneck_capacity_power);
            region_tendency_[i] = region_bottleneck_tendency_[i] * capacity_factor;
        }
    }

    for (double v : region_tendency_) {
        tendency_sum_ += v;
        tendency_max_ = std::max(tendency_max_, v);
    }
    if (tendency_sum_ <= 0.0 || tendency_max_ <= 0.0) return;

    for (int i = 0; i < region_count_; ++i) {
        region_tendency_share_[i] = region_tendency_[i] / tendency_sum_;
        region_tendency_normalized_[i] = region_tendency_[i] / tendency_max_;
    }
    tendency_ready_ = true;
}

void RegionLoadModel::compute_region_constriction() {
    region_constriction_.assign(region_count_, 0.0);
    region_avg_free_degree_.assign(region_count_, 0.0);
    if (region_count_ <= 0) return;

    std::vector<double> local_sum(region_count_, 0.0);
    std::vector<int> counts(region_count_, 0);
    for (int loc : free_locs_) {
        int rid = loc_to_region_[loc];
        if (rid < 0 || rid >= region_count_) continue;
        int r = loc / cols_, c = loc % cols_;
        const int nbs[4] = {
            (c + 1 < cols_) ? loc + 1 : -1,
            (r + 1 < rows_) ? loc + cols_ : -1,
            (c > 0) ? loc - 1 : -1,
            (r > 0) ? loc - cols_ : -1,
        };
        int free_degree = 0;
        for (int nb : nbs) {
            if (nb >= 0 && is_free_loc(nb)) ++free_degree;
        }
        local_sum[rid] += std::max(0.0, (4.0 - (double)free_degree) / 3.0);
        region_avg_free_degree_[rid] += free_degree;
        counts[rid]++;
    }

    int max_graph_degree = 0;
    for (const auto& adj : region_adj_) {
        max_graph_degree = std::max(max_graph_degree, (int)adj.size());
    }
    for (int rid = 0; rid < region_count_; ++rid) {
        double local_constriction = 0.0;
        if (counts[rid] > 0) {
            local_constriction = local_sum[rid] / counts[rid];
            region_avg_free_degree_[rid] /= counts[rid];
        }
        double graph_constriction = 0.0;
        if (max_graph_degree > 0) {
            graph_constriction =
                1.0 - (double)region_adj_[rid].size() / (double)max_graph_degree;
        }
        region_constriction_[rid] =
            std::max(0.0, std::min(1.0, 0.65 * local_constriction + 0.35 * graph_constriction));
    }
}

void RegionLoadModel::normalize_metric(const std::vector<double>& raw,
                                       std::vector<double>& normalized) const {
    normalized.assign(raw.size(), 0.0);
    double mx = 0.0;
    for (double v : raw) mx = std::max(mx, v);
    if (mx <= 0.0) return;
    for (int i = 0; i < (int)raw.size(); ++i) normalized[i] = raw[i] / mx;
}

int RegionLoadModel::region_of(int loc) const {
    if (loc < 0 || loc >= (int)loc_to_region_.size()) return -1;
    return loc_to_region_[loc];
}

int RegionLoadModel::bfs_region_distance(int region_a, int region_b) const {
    if (region_a == region_b) return 0;
    if (region_a < 0 || region_b < 0 ||
        region_a >= region_count_ || region_b >= region_count_) {
        return INT_MAX;
    }
    std::vector<int> dist(region_count_, INT_MAX);
    std::queue<int> q;
    dist[region_a] = 0;
    q.push(region_a);
    while (!q.empty()) {
        int cur = q.front();
        q.pop();
        int nd = dist[cur] + 1;
        for (int nb : region_adj_[cur]) {
            if (dist[nb] != INT_MAX) continue;
            if (nb == region_b) return nd;
            dist[nb] = nd;
            q.push(nb);
        }
    }
    return INT_MAX;
}

int RegionLoadModel::region_distance(int region_a, int region_b) const {
    if (region_a < 0 || region_b < 0 ||
        region_a >= region_count_ || region_b >= region_count_) {
        return INT_MAX;
    }
    if (distances_ready_) return region_dist_[region_a][region_b];
    return bfs_region_distance(region_a, region_b);
}

double RegionLoadModel::segment_region_risk(int from_loc, int to_loc) const {
    return route_tendency_risk(region_of(from_loc), region_of(to_loc));
}

double RegionLoadModel::route_tendency_risk(int region_a, int region_b) const {
    if (!tendency_ready_ || !distances_ready_) return 0.0;
    if (region_a < 0 || region_b < 0 ||
        region_a >= region_count_ || region_b >= region_count_) {
        return 0.0;
    }

    const int d0 = region_dist_[region_a][region_b];
    if (d0 == INT_MAX) return 0.0;

    double weight_sum = 0.0;
    double risk = 0.0;
    for (int x = 0; x < region_count_; ++x) {
        const int dax = region_dist_[region_a][x];
        const int dxb = region_dist_[x][region_b];
        if (dax == INT_MAX || dxb == INT_MAX) continue;
        int extra = dax + dxb - d0;
        if (extra < 0) extra = 0;
        const double w = std::exp(-config_.tendency_decay * (double)extra);
        weight_sum += w;
        risk += w * region_tendency_normalized_[x];
    }
    return weight_sum > 0.0 ? risk / weight_sum : 0.0;
}

nlohmann::json RegionLoadModel::to_json() const {
    nlohmann::json j;
    j["schema"] = "optsvr.region_load_model.v2";
    j["map_name"] = map_name_;
    j["rows"] = rows_;
    j["cols"] = cols_;
    j["map_size"] = map_size_;
    j["n_free"] = n_free_;
    j["mode"] = config_.mode_name();
    j["target_regions"] = config_.target_regions;
    j["max_regions"] = config_.max_regions;
    j["max_distance_regions"] = config_.max_distance_regions;
    j["build_tendency"] = config_.build_tendency;
    j["max_tendency_regions"] = config_.max_tendency_regions;
    j["tendency_decay"] = config_.tendency_decay;
    j["tendency_size_weighted"] = config_.tendency_size_weighted;
    j["tendency_kind"] = config_.tendency_kind;
    j["bottleneck_narrow_weight"] = config_.bottleneck_narrow_weight;
    j["bottleneck_capacity_power"] = config_.bottleneck_capacity_power;
    j["num_regions"] = region_count_;
    j["distances_ready"] = distances_ready_;
    j["tendency_ready"] = tendency_ready_;
    j["loc_to_region"] = loc_to_region_;

    auto rjson = nlohmann::json::array();
    for (const auto& r : regions_) {
        nlohmann::json item = {
            {"id", r.id},
            {"size", r.size},
            {"seed_loc", r.seed_loc},
            {"centroid_row", r.centroid_row},
            {"centroid_col", r.centroid_col},
            {"min_row", r.min_row},
            {"max_row", r.max_row},
            {"min_col", r.min_col},
            {"max_col", r.max_col},
            {"boundary_cells", r.boundary_cells},
        };
        if (r.id >= 0 && r.id < (int)region_tendency_.size()) {
            item["tendency"] = region_tendency_[r.id];
            item["tendency_normalized"] = region_tendency_normalized_[r.id];
            item["tendency_share"] = region_tendency_share_[r.id];
            item["soft_tendency"] = region_soft_tendency_[r.id];
            item["soft_tendency_normalized"] = region_soft_tendency_normalized_[r.id];
            item["bottleneck_tendency"] = region_bottleneck_tendency_[r.id];
            item["bottleneck_tendency_normalized"] =
                region_bottleneck_tendency_normalized_[r.id];
            item["constriction"] = region_constriction_[r.id];
            item["avg_free_degree"] = region_avg_free_degree_[r.id];
        }
        rjson.push_back(std::move(item));
    }
    j["regions"] = std::move(rjson);
    j["adjacency"] = region_adj_;
    j["tendency"] = {
        {"ready", tendency_ready_},
        {"method", config_.tendency_kind},
        {"description", "soft uses exp(-decay * delta); bottleneck uses shortest-path share with a narrow-region multiplier, then normalizes each ordered (s,t) pair to one unit."},
        {"decay", config_.tendency_decay},
        {"size_weighted_pairs", config_.tendency_size_weighted},
        {"bottleneck_narrow_weight", config_.bottleneck_narrow_weight},
        {"bottleneck_capacity_power", config_.bottleneck_capacity_power},
        {"pair_count", tendency_pair_count_},
        {"pair_weight_sum", tendency_pair_weight_sum_},
        {"sum", tendency_sum_},
        {"max", tendency_max_},
        {"values", region_tendency_},
        {"normalized", region_tendency_normalized_},
        {"share", region_tendency_share_},
        {"soft_values", region_soft_tendency_},
        {"soft_normalized", region_soft_tendency_normalized_},
        {"bottleneck_values", region_bottleneck_tendency_},
        {"bottleneck_normalized", region_bottleneck_tendency_normalized_},
        {"constriction", region_constriction_},
        {"avg_free_degree", region_avg_free_degree_},
    };

    int unassigned_free = 0;
    int assigned_obstacle = 0;
    for (int loc = 0; loc < map_size_; ++loc) {
        if (is_free_loc(loc)) {
            if (loc_to_region_[loc] < 0) unassigned_free++;
        } else if (loc < (int)loc_to_region_.size() && loc_to_region_[loc] >= 0) {
            assigned_obstacle++;
        }
    }
    j["validation"] = {
        {"unassigned_free", unassigned_free},
        {"assigned_obstacle", assigned_obstacle},
    };
    return j;
}

bool RegionLoadModel::export_json(const std::string& path) const {
    if (path.empty()) return false;
    std::filesystem::path out_path(path);
    std::error_code ec;
    if (out_path.has_parent_path()) {
        std::filesystem::create_directories(out_path.parent_path(), ec);
        if (ec) {
            std::cerr << "[RegionLoadModel] failed to create export dir: "
                      << out_path.parent_path() << " error=" << ec.message() << "\n";
            return false;
        }
    }

    std::ofstream out(path);
    if (!out) {
        std::cerr << "[RegionLoadModel] failed to open export path: " << path << "\n";
        return false;
    }
    out << to_json().dump(2);
    return true;
}

}  // namespace optsvr
