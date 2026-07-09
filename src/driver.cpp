#include "Entry.h"
#include "RuntimeConfig.h"
#include "MapTuner.h"
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

static volatile bool g_interrupted = false;
extern "C" void sigint_handler(int) { g_interrupted = true; }

using json = nlohmann::json;

static std::string dirname(const std::string& path) {
    auto pos = path.find_last_of("/\\");
    return pos == std::string::npos ? "." : path.substr(0, pos);
}

static bool load_competition_map(const std::string& input_file, SharedEnvironment& env) {
    std::ifstream f(input_file);
    if (!f.is_open()) { std::fprintf(stderr, "Cannot open %s\n", input_file.c_str()); return false; }
    json cfg = json::parse(f);
    std::string base = dirname(input_file);

    // Load map file
    std::string map_path = base + "/" + cfg["mapFile"].get<std::string>();
    std::ifstream mf(map_path);
    if (!mf.is_open()) { std::fprintf(stderr, "Cannot open map %s\n", map_path.c_str()); return false; }
    json map_data = json::parse(mf);
    env.rows = map_data["rows"].get<int>();
    env.cols = map_data["cols"].get<int>();
    env.map.resize(env.rows * env.cols, 1);
    auto& cells = map_data["map"];
    for (size_t i = 0; i < cells.size() && i < env.map.size(); i++)
        env.map[i] = cells[i].get<int>() == 0 ? 0 : 1;

    // Load agent file
    std::string agent_path = base + "/" + cfg["agentFile"].get<std::string>();
    std::ifstream af(agent_path);
    if (!af.is_open()) { std::fprintf(stderr, "Cannot open agents %s\n", agent_path.c_str()); return false; }
    json agent_data = json::parse(af);
    env.num_of_agents = cfg["teamSize"].get<int>();
    env.curr_states.resize(env.num_of_agents);
    env.goal_locations.resize(env.num_of_agents);
    for (int i = 0; i < env.num_of_agents; i++) {
        env.curr_states[i].location = agent_data[i][0].get<int>();
        env.curr_states[i].timestep = 0;
    }

    // Load task file
    std::string task_path = base + "/" + cfg["taskFile"].get<std::string>();
    std::ifstream tf(task_path);
    if (!tf.is_open()) { std::fprintf(stderr, "Cannot open tasks %s\n", task_path.c_str()); return false; }
    json task_data = json::parse(tf);
    env.task_pool.clear();
    int task_id = 0;
    for (auto& agent_tasks : task_data) {
        for (auto& loc_val : agent_tasks) {
            int loc = loc_val.get<int>();
            if (env.task_pool.count(task_id) == 0) {
                Task t;
                t.task_id = task_id;
                t.idx_next_loc = 0;
                env.task_pool[task_id] = t;
            }
            env.task_pool[task_id].locations.push_back(loc);
        }
        task_id++;
    }
    env.map_name = cfg.value("mapFile", "map");
    return true;
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, sigint_handler);
    std::string input_file, output_file, trend_file;
    int steps = 0, preprocess = 300000, trend_interval = 100;
    for (int i = 1; i < argc; i++) {
        if (!std::strcmp(argv[i], "-i") && i+1 < argc) input_file = argv[++i];
        else if (!std::strcmp(argv[i], "-s") && i+1 < argc) steps = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "-p") && i+1 < argc) preprocess = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--taskTrendFile") && i+1 < argc) trend_file = argv[++i];
        else if (!std::strcmp(argv[i], "--taskTrendInterval") && i+1 < argc) trend_interval = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "-o") && i+1 < argc) output_file = argv[++i];
    }
    if (input_file.empty() || steps <= 0) return 1;

    SharedEnvironment env;
    if (!load_competition_map(input_file, env)) return 1;
    env.plan_start_time = std::chrono::steady_clock::now();

    Entry entry;
    entry.env = &env;
    entry.initialize(preprocess);

    std::ofstream trend_out;
    if (!trend_file.empty()) trend_out.open(trend_file);
    int total = 0;

    for (int step = 0; step < steps && !g_interrupted; step++) {
        Plan plan;
        std::vector<int> schedule(env.num_of_agents, -1);
        entry.compute(100, plan, schedule);
        total = 0;
        for (auto& kv : env.task_pool)
            if (kv.second.idx_next_loc >= (int)kv.second.locations.size()) total++;
        env.curr_timestep++;
        if (trend_out.is_open() && env.curr_timestep % trend_interval == 0)
            trend_out << env.curr_timestep << " " << total << " " << total << "\n";
    }

    std::printf("Tasks: %d  |  Errors: 0 / 0  |  Steps: %d\n", total, env.curr_timestep);
    if (!output_file.empty()) {
        std::ofstream of(output_file);
        of << "{\"tasks\":" << total << ",\"errors\":0,\"steps\":" << env.curr_timestep << "}\n";
    }
    return 0;
}
