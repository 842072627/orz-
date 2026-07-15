#!/usr/bin/env python3
"""
extract_result.py — 从运行输出中提取 PERFORMANCE SUMMARY + 结果指标 → 生成格式化 JSON

用法:
    python3 result/extract_result.py \
        --stderr result/stderr.log \
        --output result/output.json \
        --input  example_problems/warehouse.domain/warehouse_small_200.json \
        [--env "OPT_SCORE_MODE=1" "OPT_GRID_RADIUS=2" ...] \
        [--dest  result/summary.json]

输出: 写入 --dest 指定的文件（默认 stdout），内容为带换行的 JSON
"""
import argparse
import json
import re
import sys
import os
from datetime import datetime
from extract_run_stats import parse_run_stats


def parse_perf_summary(stderr_path: str) -> dict:
    """从 stderr 日志中解析 PERFORMANCE SUMMARY 各字段"""
    perf = {}
    if not os.path.isfile(stderr_path):
        return perf

    with open(stderr_path, "r") as f:
        text = f.read()

    # 通用提取: "  Key name  : VALUE ms" 或 "  Key name  : VALUE"
    patterns = {
        "init_ms":           r"Init time\s*:\s*([\d.]+)\s*ms",
        "rounds":            r"Rounds \(calls\)\s*:\s*(\d+)",
        "schedule_calls":    r"Schedule calls\s*:\s*(\d+)",
        "schedule_total_ms": r"Schedule total\s*:\s*([\d.]+)\s*ms",
        "schedule_avg_ms":   r"Schedule avg\s*:\s*([\d.]+)\s*ms",
        "schedule_min_ms":   r"Schedule min/max\s*:\s*([\d.]+)",
        "schedule_max_ms":   r"Schedule min/max\s*:[\d.\s]+/\s*([\d.]+)\s*ms",
        "schedule_1st_ms":   r"Schedule 1st round\s*:\s*([\d.]+)\s*ms",
        "planner_calls":     r"Planner calls\s*:\s*(\d+)",
        "planner_total_ms":  r"Planner total\s*:\s*([\d.]+)\s*ms",
        "planner_avg_ms":    r"Planner avg\s*:\s*([\d.]+)\s*ms",
        "planner_min_ms":    r"Planner min/max\s*:\s*([\d.]+)",
        "planner_max_ms":    r"Planner min/max\s*:[\d.\s]+/\s*([\d.]+)\s*ms",
        "planner_1st_ms":    r"Planner 1st round\s*:\s*([\d.]+)\s*ms",
        "total_compute_ms":  r"Total compute time\s*:\s*([\d.]+)\s*ms",
    }

    for key, pat in patterns.items():
        m = re.search(pat, text)
        if m:
            val = m.group(1)
            # 整数字段用 int，其余 float
            if key in ("rounds", "schedule_calls", "planner_calls"):
                perf[key] = int(val)
            else:
                perf[key] = float(val)

    # --- 解析 Heuristic Degradation Stats ---
    hstats = {}
    # Planner get_gp_h: total=X | d2p=X(Y%) | bfs=X(Y%) | manhattan=X(Y%)
    m = re.search(
        r"Planner get_gp_h\s*:\s*total=(\d+)\s*\|\s*d2p=(\d+)\(([\d.]+)%\)\s*\|\s*bfs=(\d+)\(([\d.]+)%\)\s*\|\s*manhattan=(\d+)\(([\d.]+)%\)",
        text)
    if m:
        hstats["plan_gph_total"] = int(m.group(1))
        hstats["plan_d2p_count"] = int(m.group(2))
        hstats["plan_d2p_pct"] = float(m.group(3))
        hstats["plan_bfs_count"] = int(m.group(4))
        hstats["plan_bfs_pct"] = float(m.group(5))
        hstats["plan_manh_count"] = int(m.group(6))
        hstats["plan_manh_pct"] = float(m.group(7))

    # Sched get_h: total=X | hit=X(Y%) | expand=X(Y%) | miss=X(Y%)
    m = re.search(
        r"Sched\s+get_h\s*:\s*total=(\d+)\s*\|\s*hit=(\d+)\(([\d.]+)%\)\s*\|\s*expand=(\d+)\(([\d.]+)%\)\s*\|\s*miss=(\d+)\(([\d.]+)%\)",
        text)
    if m:
        hstats["sched_h_total"] = int(m.group(1))
        hstats["sched_hit_count"] = int(m.group(2))
        hstats["sched_hit_pct"] = float(m.group(3))
        hstats["sched_expand_count"] = int(m.group(4))
        hstats["sched_expand_pct"] = float(m.group(5))
        hstats["sched_miss_count"] = int(m.group(6))
        hstats["sched_miss_pct"] = float(m.group(7))

    if hstats:
        perf["heuristic_stats"] = hstats

    # 提取 RuntimeConfig 参数
    runtime_cfg = {}
    cfg_patterns = {
        "use_opt_scheduler":     r"use_opt_scheduler\s*=\s*(\S+)",
        "score_mode":            r"score_mode\s*=\s*(\S+)",
        "hungarian_threshold":   r"hungarian_threshold\s*=\s*(\S+)",
        "candidate_topk":        r"candidate_topk\s*=\s*(\S+)",
        "use_hungarian":         r"use_hungarian\s*=\s*(\S+)",
        "congestion_decay":      r"congestion_decay\s*=\s*(\S+)",
        "congestion_ttl":        r"congestion_ttl\s*=\s*(\S+)",
        "difficulty_weight":     r"difficulty_weight\s*=\s*(\S+)",
        "region_size":           r"region_size\s*=\s*(\S+)",
        "grid_cell_size":        r"grid_cell_size\s*=\s*(\S+)",
        "grid_radius":           r"grid_radius\s*=\s*(\S+)",
        "congestion_use_history": r"congestion_use_history\s*=\s*(\S+)",
        "total_steps":           r"total_steps\s*=\s*(\S+)",
    }
    for key, pat in cfg_patterns.items():
        m = re.search(pat, text)
        if m:
            val = m.group(1)
            # 尝试转为数字
            try:
                val = int(val)
            except ValueError:
                try:
                    val = float(val)
                except ValueError:
                    pass
            runtime_cfg[key] = val

    if runtime_cfg:
        perf["runtime_config"] = runtime_cfg

    # --- 解析 TIMING SUMMARY ---
    timing = {}
    timing_patterns = {
        "tasks_finished":    r"\[TIMING\]\s*tasks_finished=(\d+)",
        "init_ms":           r"\[TIMING\]\s*init_ms=([\d.]+)",
        "first_schedule_ms": r"\[TIMING\]\s*first_schedule_ms=([\d.]+)",
        "first_plan_ms":     r"\[TIMING\]\s*first_plan_ms=([\d.]+)",
        "avg_schedule_ms":   r"\[TIMING\]\s*avg_schedule_ms=([\d.]+)",
        "avg_plan_ms":       r"\[TIMING\]\s*avg_plan_ms=([\d.]+)",
        "avg_tick_ms":       r"\[TIMING\]\s*avg_tick_ms=([\d.]+)",
        "total_compute_ms":  r"\[TIMING\]\s*total_compute_ms=([\d.]+)",
        "plan_calls":        r"\[TIMING\]\s*plan_calls=(\d+)",
        "ticks":             r"\[TIMING\]\s*ticks=(\d+)",
        "avg_window_size":   r"\[TIMING\]\s*avg_window_size=([\d.]+)",
        "exec_window_size":  r"\[TIMING\]\s*exec_window_size=(\d+)",
        "avg_staged_actions": r"\[TIMING\]\s*avg_staged_actions=([\d.]+)",
        "origin_extra_append_total": r"\[TIMING\]\s*origin_extra_append_total=(\d+)",
        "flush_count":        r"\[TIMING\]\s*flush_count=(\d+)",
    }
    for key, pat in timing_patterns.items():
        m = re.search(pat, text)
        if m:
            val = m.group(1)
            if key in ("tasks_finished", "plan_calls", "ticks", "exec_window_size", "flush_count"):
                timing[key] = int(val)
            else:
                timing[key] = float(val)

    if timing:
        perf["timing"] = timing

    # 单独解析 staged_actions_per_1k_tick（逗号分隔数组）
    m = re.search(r"\[TIMING\]\s*staged_actions_per_1k_tick=([\d.,]+)", text)
    if m:
        vals = [float(v) for v in m.group(1).split(",") if v]
        if "timing" not in perf:
            perf["timing"] = {}
        perf["timing"]["staged_actions_per_1k_tick"] = vals

    return perf


def parse_epibt_flow_stats(stderr_path: str) -> dict:
    """解析 EPIBT flow/guide 重规划统计。"""
    if not os.path.isfile(stderr_path):
        return {}

    line_re = re.compile(
        r"\[EPIBTFlowStats\]\s+tick=(\d+)\s+calls=(\d+)\s+"
        r"avg_interval_tick=([\d.]+)\s+"
        r"cur\(need=(\d+),guide=(\d+),fw=(\d+),unique=(\d+),"
        r"unique_pct=([\d.]+)%,steps=(\d+)/(\d+),budget=(\d+)ms,"
        r"time=([\d.]+)\+([\d.]+)ms\)\s+"
        r"avg\(need=([\d.]+),guide=([\d.]+),fw=([\d.]+),unique=([\d.]+),"
        r"unique_pct=([\d.]+)%,steps=([\d.]+)/([\d.]+),budget=([\d.]+)ms,"
        r"time=([\d.]+)\+([\d.]+)ms\)\s+"
        r"peak\(need=(\d+),unique=(\d+)\)"
    )

    samples = []
    with open(stderr_path, "r") as f:
        for line in f:
            m = line_re.search(line)
            if not m:
                continue
            samples.append({
                "tick": int(m.group(1)),
                "calls": int(m.group(2)),
                "avg_interval_tick": float(m.group(3)),
                "cur_guide_required": int(m.group(4)),
                "cur_guide_replanned": int(m.group(5)),
                "cur_fw_replanned": int(m.group(6)),
                "cur_unique_replanned": int(m.group(7)),
                "cur_unique_pct": float(m.group(8)),
                "cur_produced_steps": int(m.group(9)),
                "cur_requested_steps": int(m.group(10)),
                "cur_flow_budget_ms": int(m.group(11)),
                "cur_guide_ms": float(m.group(12)),
                "cur_fw_ms": float(m.group(13)),
                "avg_guide_required": float(m.group(14)),
                "avg_guide_replanned": float(m.group(15)),
                "avg_fw_replanned": float(m.group(16)),
                "avg_unique_replanned": float(m.group(17)),
                "avg_unique_pct": float(m.group(18)),
                "avg_produced_steps": float(m.group(19)),
                "avg_requested_steps": float(m.group(20)),
                "avg_flow_budget_ms": float(m.group(21)),
                "avg_guide_ms": float(m.group(22)),
                "avg_fw_ms": float(m.group(23)),
                "peak_guide_required": int(m.group(24)),
                "peak_unique_replanned": int(m.group(25)),
            })

    if not samples:
        return {}

    stats = {
        "sample_count": len(samples),
        "last": samples[-1],
    }

    after_initial = [s for s in samples if s["tick"] > 0]
    if after_initial:
        def avg(key):
            return round(sum(s[key] for s in after_initial) / len(after_initial), 3)

        stats["sampled_after_initial"] = {
            "sample_count": len(after_initial),
            "avg_cur_guide_required": avg("cur_guide_required"),
            "min_cur_guide_required": min(s["cur_guide_required"] for s in after_initial),
            "max_cur_guide_required": max(s["cur_guide_required"] for s in after_initial),
            "avg_cur_fw_replanned": avg("cur_fw_replanned"),
            "avg_cur_unique_replanned": avg("cur_unique_replanned"),
            "avg_cur_guide_ms": avg("cur_guide_ms"),
            "avg_cur_fw_ms": avg("cur_fw_ms"),
        }

    return stats


def parse_output_json(output_path: str) -> dict:
    """从 lifelong 输出 JSON 中提取关键指标"""
    result = {}
    if not os.path.isfile(output_path):
        return result

    with open(output_path, "r") as f:
        data = json.load(f)

    keys = [
        "teamSize", "numTaskFinished", "makespan",
        "numPlannerErrors", "numScheduleErrors", "numEntryTimeouts",
    ]
    for k in keys:
        if k in data:
            result[k] = data[k]

    # 吞吐量
    if data.get("makespan", 0) > 0 and "numTaskFinished" in data:
        result["throughput"] = round(data["numTaskFinished"] / data["makespan"], 6)

    return result


def parse_input_json(input_path: str) -> dict:
    """从输入 JSON 中提取算例配置信息"""
    info = {}
    if not os.path.isfile(input_path):
        return info

    with open(input_path, "r") as f:
        data = json.load(f)

    info["input_file"] = input_path
    for k in ["mapFile", "agentFile", "taskFile", "teamSize", "numTasksReveal"]:
        if k in data:
            info[k] = data[k]

    return info


def parse_env_overrides(env_list: list) -> dict:
    """解析命令行传入的环境变量覆盖"""
    overrides = {}
    for item in (env_list or []):
        if "=" in item:
            k, v = item.split("=", 1)
            try:
                v = int(v)
            except ValueError:
                try:
                    v = float(v)
                except ValueError:
                    pass
            overrides[k] = v
    return overrides


def main():
    parser = argparse.ArgumentParser(description="Extract run results into formatted JSON")
    parser.add_argument("--stderr", required=True, help="Path to stderr log")
    parser.add_argument("--output", required=True, help="Path to lifelong output JSON")
    parser.add_argument("--input", required=True, help="Path to input problem JSON")
    parser.add_argument("--env", nargs="*", default=[], help="ENV overrides (e.g. OPT_SCORE_MODE=1)")
    parser.add_argument("--steps", type=int, default=None, help="Simulation steps")
    parser.add_argument("--scene", default=None, help="Scene name (short label)")
    parser.add_argument("--dest", default=None, help="Output JSON path (default: stdout)")
    parser.add_argument("--wall-time", type=float, default=None, help="Wall clock time in seconds")
    parser.add_argument("--no-run-stats", action="store_true",
                        help="Skip full actualPaths/events/tasks diagnostics")
    parser.add_argument("--movement-stats-only", action="store_true",
                        help="Only parse actualPaths movement diagnostics; skip task BFS diagnostics")
    args = parser.parse_args()

    summary = {
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }

    if args.scene:
        summary["scene"] = args.scene

    if args.steps is not None:
        summary["simulation_steps"] = args.steps

    # 算例信息
    input_info = parse_input_json(args.input)
    if input_info:
        summary["input"] = input_info

    # 输出结果
    output_info = parse_output_json(args.output)
    if output_info:
        summary["result"] = output_info

    if not args.no_run_stats:
        run_stats = parse_run_stats(
            args.output,
            args.input,
            include_task_stats=not args.movement_stats_only,
        )
        if run_stats:
            summary["run_stats"] = run_stats
            summary["run_stats_mode"] = "movement-only" if args.movement_stats_only else "full"
    else:
        summary["run_stats_skipped"] = True

    # PERFORMANCE SUMMARY
    perf = parse_perf_summary(args.stderr)
    if perf:
        summary["performance"] = perf

    epibt_flow_stats = parse_epibt_flow_stats(args.stderr)
    if epibt_flow_stats:
        summary["epibt_flow_stats"] = epibt_flow_stats

    # 将 timing 中的关键字段提升到 performance 直接层级
    # （映射字段名：TIMING SUMMARY 的 key → run_and_record.sh 使用的 key）
    timing = perf.get("timing", {})
    timing_to_perf = {
        "first_schedule_ms": "schedule_1st_ms",
        "avg_schedule_ms":   "schedule_avg_ms",
        "first_plan_ms":     "planner_1st_ms",
        "avg_plan_ms":       "planner_avg_ms",
        "avg_tick_ms":       "avg_tick_ms",
        "plan_calls":        "planner_calls",
        "total_compute_ms":  "total_compute_ms",
    }
    for src_key, dst_key in timing_to_perf.items():
        if src_key in timing and dst_key not in perf:
            perf[dst_key] = timing[src_key]

    # fallback: 如果 total_compute_ms 仍缺失，用 avg_tick * ticks 近似
    if "total_compute_ms" not in perf and "avg_tick_ms" in timing and "ticks" in timing:
        perf["total_compute_ms"] = round(timing["avg_tick_ms"] * timing["ticks"], 1)

    # 将 avg_window_size 和 exec_window_size 提升到 result 层，便于直接查看
    if "avg_window_size" in timing:
        if "result" not in summary:
            summary["result"] = {}
        summary["result"]["avg_window_size"] = timing["avg_window_size"]
    if "exec_window_size" in timing:
        if "result" not in summary:
            summary["result"] = {}
        summary["result"]["exec_window_size"] = timing["exec_window_size"]
    if "avg_staged_actions" in timing:
        if "result" not in summary:
            summary["result"] = {}
        summary["result"]["avg_staged_actions"] = timing["avg_staged_actions"]
    if "origin_extra_append_total" in timing:
        if "result" not in summary:
            summary["result"] = {}
        summary["result"]["origin_extra_append_total"] = timing["origin_extra_append_total"]
    if "staged_actions_per_1k_tick" in timing:
        if "result" not in summary:
            summary["result"] = {}
        summary["result"]["staged_actions_per_1k_tick"] = timing["staged_actions_per_1k_tick"]
    if "flush_count" in timing:
        if "result" not in summary:
            summary["result"] = {}
        summary["result"]["flush_count"] = timing["flush_count"]

    # 环境变量覆盖
    env_overrides = parse_env_overrides(args.env)
    if env_overrides:
        summary["env_overrides"] = env_overrides

    if args.wall_time is not None:
        summary["wall_time_sec"] = round(args.wall_time, 1)

    json_str = json.dumps(summary, indent=2, ensure_ascii=False)

    if args.dest:
        with open(args.dest, "w") as f:
            f.write(json_str + "\n")
        print(f"Result saved to: {args.dest}")
    else:
        print(json_str)


if __name__ == "__main__":
    main()
