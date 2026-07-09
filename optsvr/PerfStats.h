// optsvr/PerfStats.h — 通用性能统计模块
// 环境变量 OPT_ENABLE_STATS=1 开启，默认关闭
// 统计数据写入输出 JSON 的 "perfStats" 字段
#pragma once

#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include "nlohmann/json.hpp"

namespace nz {

struct PerfStats {
    bool enabled = false;

    // 计时累计（微秒）
    long long sched_total_us  = 0;
    long long planner_total_us = 0;
    long long compute_total_us = 0;
    int       call_count       = 0;

    // 峰值（微秒）
    long long sched_max_us   = 0;
    long long planner_max_us = 0;
    long long compute_max_us = 0;

    static PerfStats& instance() {
        static PerfStats s;
        return s;
    }

    void init() {
        const char* v = std::getenv("OPT_ENABLE_STATS");
        enabled = (v && (std::string(v) == "1" || std::string(v) == "true"));
    }

    // 记录一次 compute 的各阶段耗时（微秒）
    void record(long long sched_us, long long planner_us, long long total_us) {
        if (!enabled) return;
        call_count++;
        sched_total_us   += sched_us;
        planner_total_us += planner_us;
        compute_total_us += total_us;
        sched_max_us   = std::max(sched_max_us, sched_us);
        planner_max_us = std::max(planner_max_us, planner_us);
        compute_max_us = std::max(compute_max_us, total_us);
    }

    // 序列化为 JSON（写入输出文件）
    nlohmann::json to_json() const {
        if (!enabled || call_count == 0) return nullptr;
        nlohmann::json j;
        j["calls"] = call_count;
        j["scheduler_avg_ms"]  = (double)sched_total_us / call_count / 1000.0;
        j["scheduler_max_ms"]  = sched_max_us / 1000.0;
        j["scheduler_total_s"] = sched_total_us / 1e6;
        j["planner_avg_ms"]    = (double)planner_total_us / call_count / 1000.0;
        j["planner_max_ms"]    = planner_max_us / 1000.0;
        j["planner_total_s"]   = planner_total_us / 1e6;
        j["compute_avg_ms"]    = (double)compute_total_us / call_count / 1000.0;
        j["compute_max_ms"]    = compute_max_us / 1000.0;
        j["compute_total_s"]   = compute_total_us / 1e6;
        return j;
    }

    // stderr 打印（保留用于快速本地调试）
    void print_summary() const {
        if (!enabled || call_count == 0) return;
        fprintf(stderr,
            "\n[PERF_STATS] calls=%d  sched_avg=%.2fms  planner_avg=%.2fms  compute_avg=%.2fms\n",
            call_count,
            (double)sched_total_us / call_count / 1000.0,
            (double)planner_total_us / call_count / 1000.0,
            (double)compute_total_us / call_count / 1000.0
        );
    }

private:
    PerfStats() = default;
};

inline PerfStats& perf() { return PerfStats::instance(); }

} // namespace nz
