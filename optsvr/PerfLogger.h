// optsvr/PerfLogger.h — 性能日志统计器（纯计时版）
//
// 设计：
//   - 默认输出，无需开关
//   - 时间统计全部在 TaskScheduler::plan() 的调用处完成
//   - schedule 时间 = TaskScheduler::plan() 中 schedule 逻辑的耗时
//   - planner 时间 = 两次 TaskScheduler::plan() 调用之间的间隔减去 schedule 时间
//     （即 update_goal_locations + planner->plan() + 主线程等待/调度开销）
//   - 运行结束后通过 atexit 统一输出 final summary
//
// 调用时序（在 TaskScheduler 中）：
//   initialize:
//     on_init_start() → ... → on_init_end()
//   每轮 plan:
//     on_round_begin()  ← 结算上轮 planner 时间
//     ... schedule 逻辑 ...
//     on_schedule_done()  ← 记录本轮 schedule 时间
#pragma once

#include <chrono>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cfloat>
#include <atomic>
#include <csignal>

// 本地调试时定义此宏可启用 heuristic 退化统计输出
// 评测服务器上 default_planner/ 被覆盖，HeuristicStats.h 不存在，不要开启
#ifdef LORR_LOCAL_DEBUG
#include "HeuristicStats.h"
#endif

namespace nz {

using PerfClock = std::chrono::steady_clock;
using PerfTimePoint = PerfClock::time_point;

class PerfLogger {
public:
    static PerfLogger& instance() {
        static PerfLogger logger;
        return logger;
    }

    // ============ 初始化阶段 ============
    void on_init_start() {
        init_start_ = PerfClock::now();
    }

    void on_init_end() {
        init_elapsed_ms_ = elapsed_ms(init_start_);
    }

    // ============ 每轮 schedule 开始 ============
    void on_round_begin() {
        auto now = PerfClock::now();

        // 结算上一轮 planner 耗时
        if (has_prev_schedule_end_) {
            double plan_ms = std::chrono::duration<double, std::milli>(now - prev_schedule_end_).count();
            plan_count_++;
            total_plan_ms_ += plan_ms;
            if (plan_ms < min_plan_ms_) min_plan_ms_ = plan_ms;
            if (plan_ms > max_plan_ms_) max_plan_ms_ = plan_ms;
            if (plan_count_ == 1) first_plan_ms_ = plan_ms;
        }

        round_count_++;
        schedule_start_ = now;

        // 每轮重新注册信号处理器：
        // driver.cpp 在 simulate() 之前调用 signal(SIGINT, sigint_handler) 覆盖了我们的，
        // 这里每轮重新抢回 SIGINT 的处理权，确保 PerfLogger 能在被杀前输出。
        install_signal_handlers();
    }

    // ============ schedule 完成 ============
    void on_schedule_done() {
        auto now = PerfClock::now();
        double sched_ms = std::chrono::duration<double, std::milli>(now - schedule_start_).count();

        sched_count_++;
        total_sched_ms_ += sched_ms;
        if (sched_ms < min_sched_ms_) min_sched_ms_ = sched_ms;
        if (sched_ms > max_sched_ms_) max_sched_ms_ = sched_ms;
        if (sched_count_ == 1) first_sched_ms_ = sched_ms;

        // 标记 schedule 结束时间点
        prev_schedule_end_ = now;
        has_prev_schedule_end_ = true;
    }

    // ============ 最终输出 ============
    // 使用 fprintf(stderr) 而非 std::cerr —— fprintf 是异步信号安全的，
    // 在 SIGINT/SIGTERM handler 或 _exit 前调用也能正确输出。
    void print_final_summary() {
        if (printed_.exchange(true)) return;  // 原子操作防止重复

        // 如果从来没被调用过，跳过输出
        if (round_count_ == 0) return;

        double avg_sched = sched_count_ > 0 ? total_sched_ms_ / sched_count_ : 0.0;
        double avg_plan  = plan_count_  > 0 ? total_plan_ms_ / plan_count_ : 0.0;

        double disp_min_sched = sched_count_ > 0 ? min_sched_ms_ : 0.0;
        double disp_max_sched = sched_count_ > 0 ? max_sched_ms_ : 0.0;
        double disp_min_plan  = plan_count_  > 0 ? min_plan_ms_ : 0.0;
        double disp_max_plan  = plan_count_  > 0 ? max_plan_ms_ : 0.0;

        // 使用 fprintf 一次性输出，比 cerr 链式调用更安全
        fprintf(stderr,
            "\n"
            "============================================================\n"
            "                    PERFORMANCE SUMMARY                      \n"
            "============================================================\n"
            "  Init time          : %.1f ms\n"
            "  Rounds (calls)     : %d\n"
            "------------------------------------------------------------\n"
            "  Schedule calls     : %d\n"
            "  Schedule total     : %.1f ms\n"
            "  Schedule avg       : %.2f ms\n"
            "  Schedule min/max   : %.2f / %.2f ms\n"
            "  Schedule 1st round : %.2f ms\n"
            "------------------------------------------------------------\n"
            "  Planner calls      : %d\n"
            "  Planner total      : %.1f ms\n"
            "  Planner avg        : %.2f ms\n"
            "  Planner min/max    : %.2f / %.2f ms\n"
            "  Planner 1st round  : %.2f ms\n"
            "------------------------------------------------------------\n"
            "  Total compute time : %.1f ms\n"
            "============================================================\n\n",
            init_elapsed_ms_,
            round_count_,
            sched_count_,
            total_sched_ms_,
            avg_sched,
            disp_min_sched, disp_max_sched,
            first_sched_ms_,
            plan_count_,
            total_plan_ms_,
            avg_plan,
            disp_min_plan, disp_max_plan,
            first_plan_ms_,
            total_sched_ms_ + total_plan_ms_
        );
        fflush(stderr);

#ifdef LORR_LOCAL_DEBUG
        // 本地调试：追加 heuristic 退化统计
        DefaultPlanner::g_hstats.print_summary();
        fflush(stderr);
#endif
    }

private:
    // 信号处理器 —— 在 SIGINT/SIGTERM 时抢先输出 summary
    // driver.cpp 的 sigint_handler 调用 _exit(0) 不触发 atexit，
    // 我们先输出 summary，再把控制权交给原始 handler（saveResults）
    static void signal_handler(int sig) {
        PerfLogger::instance().print_final_summary();
        // 恢复之前保存的 handler 并重新发送信号
        auto prev = PerfLogger::instance().prev_sigint_handler_;
        if (prev && prev != SIG_DFL && prev != SIG_ERR) {
            prev(sig);  // 调用 driver.cpp 的 sigint_handler (saveResults + _exit)
        } else {
            std::signal(sig, SIG_DFL);
            std::raise(sig);
        }
    }

    PerfLogger() {
        // 注册 atexit 兜底（正常 exit(0) 路径）
        std::atexit([]() {
            PerfLogger::instance().print_final_summary();
        });
        // 注册信号处理器（_exit 路径前的最后机会）
        // 注意：driver.cpp 在 simulate 之前注册 SIGINT handler (sigint_handler)，
        // 而 PerfLogger 在 initialize 阶段就构造了，driver.cpp 后注册会覆盖我们的。
        // 解决方案：在 on_round_begin 中每次重新注册，确保我们的 handler 在链顶。
        install_signal_handlers();
    }

    void install_signal_handlers() {
        // 保存当前 SIGINT handler（可能是 driver.cpp 注册的 sigint_handler）
        auto prev = std::signal(SIGINT, signal_handler);
        if (prev != signal_handler) {
            prev_sigint_handler_ = prev;  // 记住 driver.cpp 的 handler
        }
        std::signal(SIGTERM, signal_handler);
    }

    double elapsed_ms(PerfTimePoint start) const {
        return std::chrono::duration<double, std::milli>(PerfClock::now() - start).count();
    }

    // 保存 driver.cpp 的原始 SIGINT handler（sigint_handler → saveResults + _exit）
    void (*prev_sigint_handler_)(int) = nullptr;

    // 初始化
    PerfTimePoint init_start_;
    double init_elapsed_ms_ = 0.0;

    // 轮次统计
    int round_count_ = 0;

    // Schedule 统计（全部用标量，避免 vector 被竞态破坏）
    PerfTimePoint schedule_start_;
    int sched_count_ = 0;
    double total_sched_ms_ = 0.0;
    double min_sched_ms_ = DBL_MAX;
    double max_sched_ms_ = 0.0;
    double first_sched_ms_ = 0.0;

    // Plan 统计
    PerfTimePoint prev_schedule_end_;
    bool has_prev_schedule_end_ = false;
    int plan_count_ = 0;
    double total_plan_ms_ = 0.0;
    double min_plan_ms_ = DBL_MAX;
    double max_plan_ms_ = 0.0;
    double first_plan_ms_ = 0.0;

    // 原子标记防止重复输出
    std::atomic<bool> printed_{false};
};

// 便捷访问
inline PerfLogger& perf() { return PerfLogger::instance(); }

} // namespace nz
