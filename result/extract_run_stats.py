#!/usr/bin/env python3
"""Derived run-level diagnostics from LoRR output JSON."""

from collections import deque
import bisect
import json
import os
import re


_ACTION_RE = re.compile(r"\b([FWCRT])\s+(\d+)")
_CHUNK_RE = re.compile(r"\[\((\d+),[^)]*\):\(([^)]*)\)\]")


def _load_map(input_path):
    try:
        with open(input_path, "r") as f:
            inp = json.load(f)
        map_file = inp.get("mapFile")
        if not map_file:
            return None
        if not os.path.isabs(map_file):
            map_file = os.path.normpath(os.path.join(os.path.dirname(input_path), map_file))
        with open(map_file, "r") as f:
            lines = [line.rstrip("\n") for line in f]
    except Exception:
        return None

    rows = cols = None
    grid_start = None
    for i, line in enumerate(lines):
        parts = line.split()
        if len(parts) == 2 and parts[0] == "height":
            rows = int(parts[1])
        elif len(parts) == 2 and parts[0] == "width":
            cols = int(parts[1])
        elif line == "map":
            grid_start = i + 1
            break
    if rows is None or cols is None or grid_start is None:
        return None

    raw = lines[grid_start:grid_start + rows]
    if len(raw) != rows:
        return None
    passable = [ch not in ("@", "T", "O") for line in raw for ch in line[:cols]]
    return rows, cols, passable


def _bfs_distances(goal, rows, cols, passable):
    n = rows * cols
    dist = [-1] * n
    if goal < 0 or goal >= n or not passable[goal]:
        return dist
    q = deque([goal])
    dist[goal] = 0
    while q:
        loc = q.popleft()
        r, c = divmod(loc, cols)
        for nb in (loc - cols, loc + cols, loc - 1, loc + 1):
            if nb < 0 or nb >= n or dist[nb] >= 0 or not passable[nb]:
                continue
            nr, nc = divmod(nb, cols)
            if abs(nr - r) + abs(nc - c) != 1:
                continue
            dist[nb] = dist[loc] + 1
            q.append(nb)
    return dist


def _task_locs(task, cols):
    loc_pairs = task[2] if len(task) > 2 and isinstance(task[2], list) else []
    locs = []
    for i in range(0, len(loc_pairs) - 1, 2):
        locs.append(int(loc_pairs[i]) * cols + int(loc_pairs[i + 1]))
    return locs


def _path_dist(locs, map_data, cache):
    if len(locs) <= 1:
        return 0
    if map_data is None:
        return None
    rows, cols, passable = map_data
    total = 0
    for src, dst in zip(locs, locs[1:]):
        if dst not in cache:
            cache[dst] = _bfs_distances(dst, rows, cols, passable)
        d = cache[dst][src] if 0 <= src < len(cache[dst]) else -1
        if d < 0:
            return None
        total += d
    return total


def _manhattan_path_dist(locs, cols):
    total = 0
    for src, dst in zip(locs, locs[1:]):
        sr, sc = divmod(src, cols)
        dr, dc = divmod(dst, cols)
        total += abs(sr - dr) + abs(sc - dc)
    return total


def _parse_actual_schedule(schedule_entries):
    schedules = []
    for entry in schedule_entries or []:
        pairs = []
        if isinstance(entry, str) and entry:
            for part in entry.split(","):
                if ":" not in part:
                    continue
                ts, tid = part.split(":", 1)
                try:
                    pairs.append((int(ts), int(tid)))
                except ValueError:
                    continue
        schedules.append(pairs)
    return schedules


def _assignment_start(schedules, agent_id, task_id, complete_tick):
    if agent_id < 0 or agent_id >= len(schedules):
        return None
    best = None
    for ts, tid in schedules[agent_id]:
        if ts > complete_tick:
            break
        if tid == task_id:
            best = ts
        elif best is not None:
            best = None
    return best


def _movement_stats(data):
    if not data.get("actualPaths"):
        return {}
    action_ticks = {"F": 0, "W": 0, "C": 0, "R": 0, "T": 0}
    for path in data.get("actualPaths") or []:
        if not isinstance(path, str):
            continue
        for action, duration in _ACTION_RE.findall(path):
            action_ticks[action] = action_ticks.get(action, 0) + int(duration)

    total_action_ticks = sum(action_ticks.values())
    team_size = int(data.get("teamSize") or 0)
    makespan = int(data.get("makespan") or 0)
    agent_ticks = team_size * makespan
    max_counter = max(1, int(data.get("agentMaxCounter") or 1))

    stats = {
        "movement_action_ticks": action_ticks,
        "movement_total_action_ticks": total_action_ticks,
    }
    if total_action_ticks > 0:
        stats["avg_forward_action_ratio"] = round(action_ticks.get("F", 0) / total_action_ticks, 6)
        stats["avg_wait_action_ratio"] = round(action_ticks.get("W", 0) / total_action_ticks, 6)
        stats["avg_turn_action_ratio"] = round((action_ticks.get("C", 0) + action_ticks.get("R", 0)) / total_action_ticks, 6)
    if agent_ticks > 0:
        stats["avg_forward_ticks_per_agent_tick"] = round(action_ticks.get("F", 0) / agent_ticks, 6)
        stats["avg_cell_speed_est"] = round(action_ticks.get("F", 0) / (agent_ticks * max_counter), 6)
    return stats


def _forward_segments(path):
    segments = []
    if not isinstance(path, str):
        return segments
    for chunk_start, actions in _CHUNK_RE.findall(path):
        cursor = int(chunk_start)
        for action, duration_s in _ACTION_RE.findall(actions):
            duration = int(duration_s)
            if action == "F" and duration > 0:
                segments.append((cursor, cursor + duration))
            cursor += duration
    return segments


def _sum_forward_ticks(segments, start_tick, end_tick):
    if end_tick <= start_tick or not segments:
        return 0
    starts = [s for s, _ in segments]
    idx = max(0, bisect.bisect_left(starts, start_tick) - 1)
    total = 0
    while idx < len(segments):
        s, e = segments[idx]
        if s >= end_tick:
            break
        overlap = min(e, end_tick) - max(s, start_tick)
        if overlap > 0:
            total += overlap
        idx += 1
    return total


def _task_stats(data, input_path):
    if not data.get("tasks") or not data.get("events"):
        return {}
    map_data = _load_map(input_path)
    if map_data is None:
        cols = None
    else:
        _, cols, _ = map_data
    if cols is None:
        return {}

    tasks = {}
    dist_cache = {}
    for task in data.get("tasks") or []:
        if not isinstance(task, list) or len(task) < 3:
            continue
        try:
            task_id = int(task[0])
            revealed = int(task[1])
        except (TypeError, ValueError):
            continue
        locs = _task_locs(task, cols)
        hdist = _path_dist(locs, map_data, dist_cache)
        if hdist is None:
            hdist = _manhattan_path_dist(locs, cols)
        tasks[task_id] = {
            "revealed": revealed,
            "num_locs": len(locs),
            "heuristic_dist": hdist,
        }

    schedules = _parse_actual_schedule(data.get("actualSchedule"))
    forward_by_agent = [_forward_segments(path) for path in (data.get("actualPaths") or [])]
    max_counter = max(1, int(data.get("agentMaxCounter") or 1))
    first_events = {}
    completed = {}
    for event in data.get("events") or []:
        if not isinstance(event, list) or len(event) < 4:
            continue
        try:
            tick, agent_id, task_id, seq_id = map(int, event[:4])
        except (TypeError, ValueError):
            continue
        info = tasks.get(task_id)
        if not info or info["num_locs"] <= 0:
            continue
        if seq_id == 1 and task_id not in first_events:
            first_events[task_id] = (tick, agent_id)
        if seq_id >= info["num_locs"] and task_id not in completed:
            completed[task_id] = (tick, agent_id)

    n = len(completed)
    if n == 0:
        return {"completed_task_stats_count": 0}

    total_dist = 0
    total_reveal_ticks = 0
    total_assign_ticks = 0
    total_assigned_dist = 0
    assign_count = 0
    total_detour_ratio = 0.0
    total_extra_detour_ratio = 0.0
    total_internal_actual_dist = 0.0
    total_internal_shortest_dist = 0.0
    detour_count = 0
    for task_id, (tick, agent_id) in completed.items():
        info = tasks[task_id]
        total_dist += info["heuristic_dist"]
        total_reveal_ticks += max(0, tick - info["revealed"])
        start = _assignment_start(schedules, agent_id, task_id, tick)
        if start is not None:
            total_assign_ticks += max(0, tick - start)
            total_assigned_dist += info["heuristic_dist"]
            assign_count += 1

        first = first_events.get(task_id)
        if first is not None and first[1] == agent_id and info["heuristic_dist"] > 0:
            first_tick = first[0]
            if 0 <= agent_id < len(forward_by_agent) and tick > first_tick:
                fw_ticks = _sum_forward_ticks(forward_by_agent[agent_id], first_tick, tick)
                actual_dist = fw_ticks / max_counter
                ratio = actual_dist / info["heuristic_dist"]
                total_detour_ratio += ratio
                total_extra_detour_ratio += ratio - 1.0
                total_internal_actual_dist += actual_dist
                total_internal_shortest_dist += info["heuristic_dist"]
                detour_count += 1

    stats = {
        "completed_task_stats_count": n,
        "avg_completed_task_heuristic_dist": round(total_dist / n, 3),
        "avg_task_reveal_to_complete_ticks": round(total_reveal_ticks / n, 3),
    }
    if assign_count > 0:
        stats["assigned_task_stats_count"] = assign_count
        stats["avg_assigned_completed_task_heuristic_dist"] = round(total_assigned_dist / assign_count, 3)
        stats["avg_task_assign_to_complete_ticks"] = round(total_assign_ticks / assign_count, 3)
        stats["completed_task_heuristic_speed"] = round(total_assigned_dist / total_assign_ticks, 6) if total_assign_ticks > 0 else 0
    if detour_count > 0:
        detour_ratio = total_internal_actual_dist / total_internal_shortest_dist if total_internal_shortest_dist > 0 else 0
        stats["detour_task_stats_count"] = detour_count
        stats["task_internal_detour_ratio"] = round(detour_ratio, 6)
        stats["task_internal_extra_detour_ratio"] = round(detour_ratio - 1.0, 6)
        stats["avg_task_internal_detour_ratio"] = round(total_detour_ratio / detour_count, 6)
        stats["avg_task_internal_extra_detour_ratio"] = round(total_extra_detour_ratio / detour_count, 6)
        stats["avg_task_internal_actual_forward_dist"] = round(total_internal_actual_dist / detour_count, 3)
        stats["avg_task_internal_shortest_dist"] = round(total_internal_shortest_dist / detour_count, 3)
    return stats


def parse_run_stats(output_path, input_path, include_task_stats=True):
    if not os.path.isfile(output_path):
        return {}
    try:
        with open(output_path, "r") as f:
            data = json.load(f)
    except Exception:
        return {}

    stats = {}
    stats.update(_movement_stats(data))
    if include_task_stats:
        stats.update(_task_stats(data, input_path))
    return {k: v for k, v in stats.items() if v is not None}
