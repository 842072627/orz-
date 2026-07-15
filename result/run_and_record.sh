#!/usr/bin/env bash
# ================================================================
# run_and_record.sh — 运行测试，提取结果 JSON，追加到 benchmark_results.md
#
# 用法:
#   ./result/run_and_record.sh                                      # 默认 warehouse 1000步
#   ./result/run_and_record.sh --scene iron_harvest --steps 5000    # 指定场景步数
#   ./result/run_and_record.sh --env SCORE_MODE=1 --env GRID_RADIUS=2  # 覆盖 ENV 参数
#   ./result/run_and_record.sh --baseline                           # 关闭优化跑 baseline
#   ./result/run_and_record.sh --scene warehouse --steps 1000 --tag "v2.1 改进匹配"
#   ./result/run_and_record.sh --scene warehousegen --steps 5000 # 更接近线上 warehouse 版本
#   ./result/run_and_record.sh --scene city --steps 5000 --keep-log # 保留 stderr 日志
#   ./result/run_and_record.sh --scene city --steps 5000 --error    # 兼容旧写法，同 --keep-log
#   ./result/run_and_record.sh --scene room --steps 100 --viz      # 只生成可视化 JSON；另用 visualize_result.sh 打开
#   ./result/visualize_result.sh --scene room --plan latest --window 500 --agents 200
#   ./result/run_and_record.sh --log-level 3                        # 只显示 fatal 日志（关闭 warning）
#   ./result/run_and_record.sh --output-level 3                     # 输出 summary JSON，避免长跑生成巨大路径明细
#   ./result/run_and_record.sh --initial-plan 3000                  # 设置 initialPlanTimeLimit=3000ms
#   ./result/run_and_record.sh --plan-comm 2000                      # 设置 planCommTimeLimit=2000ms
#   ./result/run_and_record.sh --movement-stats                     # 只统计 actualPaths 移动率，不跑 task BFS
#   ./result/run_and_record.sh --stats                              # 启用 actualPaths/events 全量统计
# ================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# This copy always runs the submission binary from this repository. Test
# instances stay outside the lightweight submission and default to the sibling
# MAIN2026/example_problems directory; override with LORR_INPUT_ROOT if needed.
TRACK="SSFC"
PROJECT_ROOT="$REPO_ROOT"
if [ -n "${LORR_INPUT_ROOT:-}" ]; then
    INPUT_ROOT="$LORR_INPUT_ROOT"
elif [ -d "$REPO_ROOT/example_problems" ]; then
    INPUT_ROOT="$REPO_ROOT"
else
    INPUT_ROOT="$REPO_ROOT/../MAIN2026"
fi

BINARY="$PROJECT_ROOT/build/lifelong"
STEPS=1000
SCENE="warehouse"
ENV_OVERRIDES=""
TAG=""
BASELINE=false
KEEP_LOG=false
VIZ=false
VIZ_START=0
VIZ_END=""
VIZ_WINDOW=0
VIZ_AGENTS=""
VIZ_AGENT_START=0
VIZ_AGENT_STRIDE=1
LOG_LEVEL=1
OUTPUT_LEVEL=""
PLAN_TIME_LIMIT=""
INITIAL_PLAN_TIME_LIMIT=""
FULL_STATS=false
MOVEMENT_STATS=false

RESULTS_MD="$REPO_ROOT/result/benchmark_results.md"
# 追加 PID 后缀避免并行跑互相覆盖 tmp 文件
_PID_SFX="$$_$(date +%s%N 2>/dev/null || date +%s)"
TMP_OUTPUT="$REPO_ROOT/result/_tmp_output_${_PID_SFX}.json"
TMP_STDERR="$REPO_ROOT/result/_tmp_stderr_${_PID_SFX}.log"
RESULT_JSON="$REPO_ROOT/result/_tmp_summary_${_PID_SFX}.json"
# benchmark_results.md 追加时加 flock 保证互斥
_LOCK_FILE="$REPO_ROOT/result/.benchmark_results.lock"

# 场景别名 → 文件路径（基于 MAIN2026/example_problems 当前算例）
resolve_scene() {
    case "$1" in
        warehouse)       echo "$INPUT_ROOT/example_problems/warehouse.domain/fulfill-example_2500.json" ;;
        warehouse_large) echo "$INPUT_ROOT/example_problems/warehouse.domain/warehouse_large_5000.json" ;;
        warehouse_7500)  echo "$INPUT_ROOT/example_problems/warehouse.domain/fulfill-example_7500.json" ;;
        warehouse_10000) echo "$INPUT_ROOT/example_problems/warehouse.domain/fulfill-example_10000.json" ;;
        warehousegen|warehouse_gen|warehousegen_10000|warehouse_online|warehouse_online_10000) echo "$INPUT_ROOT/example_problems/warehouse.domain/fulfill-warehousegen-example_10000.json" ;;
        city)            echo "$INPUT_ROOT/example_problems/city.domain/bos-example_600.json" ;;
        city2000|city_2000|bos2000) echo "$INPUT_ROOT/example_problems/city.domain/bos-example_2000.json" ;;
        game)            echo "$INPUT_ROOT/example_problems/game.domain/orz-example_1800.json" ;;
        game4k)          echo "$INPUT_ROOT/example_problems/game.domain/orz-example_4000.json" ;;
        iron_harvest)    echo "$INPUT_ROOT/example_problems/iron_harvest.domain/iron-example_10000.json" ;;
        maze)            echo "$INPUT_ROOT/example_problems/maze.domain/maze-example_40.json" ;;
        maze50|maze_50)  echo "$INPUT_ROOT/example_problems/maze.domain/maze-example_50.json" ;;
        random)          echo "$INPUT_ROOT/example_problems/random.domain/random-example_400.json" ;;
        random1500|random_1500) echo "$INPUT_ROOT/example_problems/random.domain/random-example_1500.json" ;;
        randomA|random_a) echo "$INPUT_ROOT/example_problems/random.domain/randomA_instance_config.json" ;;
        room)            echo "$INPUT_ROOT/example_problems/room.domain/room-example_150.json" ;;
        room200|room_200) echo "$INPUT_ROOT/example_problems/room.domain/room-example_200.json" ;;
        *)               echo "$1" ;;
    esac
}

# 场景别名 → 短名
resolve_short_name() {
    case "$1" in
        warehouse)       echo "warehouse_fulfill_2500" ;;
        warehouse_large) echo "warehouse_large_5000" ;;
        warehouse_7500)  echo "warehouse_fulfill_7500" ;;
        warehouse_10000) echo "warehouse_fulfill_10000" ;;
        warehousegen|warehouse_gen|warehousegen_10000|warehouse_online|warehouse_online_10000) echo "warehousegen_fulfill_10000" ;;
        city)            echo "city_bos_600" ;;
        city2000|city_2000|bos2000) echo "city_bos_2000" ;;
        game)            echo "game_orz_1800" ;;
        game4k)          echo "game_orz_4000" ;;
        iron_harvest)    echo "iron_harvest_10000" ;;
        maze)            echo "maze_40" ;;
        maze50|maze_50)  echo "maze_50" ;;
        random)          echo "random_400" ;;
        random1500|random_1500) echo "random_1500" ;;
        randomA|random_a) echo "randomA_1500" ;;
        room)            echo "room_150" ;;
        room200|room_200) echo "room200" ;;
        *)               echo "$1" ;;
    esac
}

while [ $# -gt 0 ]; do
    case "$1" in
        --scene)     SCENE="$2"; shift 2 ;;
        --steps)     STEPS="$2"; shift 2 ;;
        --env)
            _envarg="$2"
            # 如果没有 =，自动补 =1（布尔开关快捷写法）
            if [[ "$_envarg" != *=* ]]; then
                _envarg="${_envarg}=1"
            fi
            _envkey="${_envarg%%=*}"
            _envval="${_envarg#*=}"
            _envkey="$(printf '%s' "$_envkey" | tr '[:lower:]' '[:upper:]')"
            # 如果参数已带 OPT_ / GAME_ / ROOM_ / IRON_ / HARD_ 前缀，则不再添加 OPT_
            if [[ "$_envkey" == OPT_* ]] || [[ "$_envkey" == GAME_* ]] || [[ "$_envkey" == ROOM_* ]] || [[ "$_envkey" == IRON_* ]] || [[ "$_envkey" == HARD_* ]]; then
                ENV_OVERRIDES="$ENV_OVERRIDES ${_envkey}=${_envval}"
            else
                ENV_OVERRIDES="$ENV_OVERRIDES OPT_${_envkey}=${_envval}"
            fi
            shift 2 ;;
        --baseline)  BASELINE=true; shift ;;
        --error|--keep-log) KEEP_LOG=true; shift ;;
        --no-keep-log) KEEP_LOG=false; shift ;;
        --viz)       VIZ=true; shift ;;
        --viz-start) VIZ_START="$2"; shift 2 ;;
        --viz-end)   VIZ_END="$2"; shift 2 ;;
        --viz-window) VIZ_WINDOW="$2"; shift 2 ;;
        --viz-agents|--viz-n) VIZ_AGENTS="$2"; shift 2 ;;
        --viz-agent-start) VIZ_AGENT_START="$2"; shift 2 ;;
        --viz-agent-stride|--viz-agent-step) VIZ_AGENT_STRIDE="$2"; shift 2 ;;
        --log-level)    LOG_LEVEL="$2"; shift 2 ;;
        --output-level) OUTPUT_LEVEL="$2"; shift 2 ;;
        --initial-plan) INITIAL_PLAN_TIME_LIMIT="$2"; shift 2 ;;
        --plan-comm)    PLAN_TIME_LIMIT="$2"; shift 2 ;;
        --no-stats|--quick-summary) FULL_STATS=false; MOVEMENT_STATS=false; shift ;;
        --movement-stats) FULL_STATS=false; MOVEMENT_STATS=true; shift ;;
        --stats|--full-stats) FULL_STATS=true; MOVEMENT_STATS=false; shift ;;
        --tag)          TAG="$2"; shift 2 ;;
        *)           echo "Unknown arg: $1"; exit 1 ;;
    esac
done

BINARY="$PROJECT_ROOT/build/lifelong"

INPUT_FILE=$(resolve_scene "$SCENE")
SCENE_NAME=$(resolve_short_name "$SCENE")

if [ ! -f "$INPUT_FILE" ]; then
    echo "Error: '$INPUT_FILE' not found"; exit 1
fi

if $BASELINE; then
    ENV_OVERRIDES="OPT_USE_OPT_SCHEDULER=0"
fi

# 去除前后空格
ENV_OVERRIDES=$(echo "$ENV_OVERRIDES" | xargs)

if ! $FULL_STATS && ! $MOVEMENT_STATS && ! $VIZ && [ -z "$OUTPUT_LEVEL" ]; then
    OUTPUT_LEVEL=3
fi

# 清理临时文件
rm -f "$TMP_OUTPUT" "$TMP_STDERR" "$RESULT_JSON"

echo "=========================================="
echo "  Track : $TRACK ($PROJECT_ROOT)"
echo "  Scene : $SCENE_NAME ($INPUT_FILE)"
echo "  Steps : $STEPS"
echo "  Env   : ${ENV_OVERRIDES:-<defaults>}"
echo "  Log   : level $LOG_LEVEL (1=all, 2=warn+fatal, 3=fatal only, 4=info+fatal)"
[ -n "$OUTPUT_LEVEL" ] && echo "  Out   : level $OUTPUT_LEVEL"
[ -n "$INITIAL_PLAN_TIME_LIMIT" ] && echo "  Init  : initialPlanTimeLimit=${INITIAL_PLAN_TIME_LIMIT}ms"
[ -n "$PLAN_TIME_LIMIT" ] && echo "  TimeL : planCommTimeLimit=${PLAN_TIME_LIMIT}ms"
if $FULL_STATS; then
    STATS_LABEL="full"
elif $MOVEMENT_STATS; then
    STATS_LABEL="movement-only"
else
    STATS_LABEL="summary-only"
fi
echo "  Stats : $STATS_LABEL"
[ "$VIZ" = true ] && echo "  Viz   : start=${VIZ_START}, ${VIZ_END:+end=${VIZ_END}, }window=${VIZ_WINDOW}, agents=${VIZ_AGENTS:-all}, agentStart=${VIZ_AGENT_START}, stride=${VIZ_AGENT_STRIDE}"
[ -n "$TAG" ] && echo "  Tag   : $TAG"
echo "=========================================="

# ---- 构造额外参数 ----
EXTRA_ARGS=""
[ -n "$INITIAL_PLAN_TIME_LIMIT" ] && EXTRA_ARGS="$EXTRA_ARGS -n $INITIAL_PLAN_TIME_LIMIT"
[ -n "$PLAN_TIME_LIMIT" ] && EXTRA_ARGS="$EXTRA_ARGS -t $PLAN_TIME_LIMIT"
[ -n "$OUTPUT_LEVEL" ] && EXTRA_ARGS="$EXTRA_ARGS -c $OUTPUT_LEVEL"

# ---- 运行 ----
START_SEC=$(date +%s)
RUN_EXIT=0
if [ -n "$ENV_OVERRIDES" ]; then
    env $ENV_OVERRIDES "$BINARY" -i "$INPUT_FILE" -s "$STEPS" -o "$TMP_OUTPUT" -d "$LOG_LEVEL" $EXTRA_ARGS 2> >(tee "$TMP_STDERR" >&2) || RUN_EXIT=$?
else
    "$BINARY" -i "$INPUT_FILE" -s "$STEPS" -o "$TMP_OUTPUT" -d "$LOG_LEVEL" $EXTRA_ARGS 2> >(tee "$TMP_STDERR" >&2) || RUN_EXIT=$?
fi
END_SEC=$(date +%s)
WALL_TIME=$((END_SEC - START_SEC))

if [ $RUN_EXIT -ne 0 ]; then
    echo "Warning: lifelong exited with code $RUN_EXIT (may be normal for some simulations)"
fi

# ---- 提取结果 JSON ----
EXTRACT_CMD="python3 $REPO_ROOT/result/extract_result.py \
    --stderr $TMP_STDERR \
    --output $TMP_OUTPUT \
    --input  $INPUT_FILE \
    --steps  $STEPS \
    --scene  $SCENE_NAME \
    --wall-time $WALL_TIME \
    --dest   $RESULT_JSON"

if [ -n "$ENV_OVERRIDES" ]; then
    EXTRACT_CMD="$EXTRACT_CMD --env $ENV_OVERRIDES"
fi
if ! $FULL_STATS && ! $MOVEMENT_STATS; then
    EXTRACT_CMD="$EXTRACT_CMD --no-run-stats"
elif $MOVEMENT_STATS; then
    EXTRACT_CMD="$EXTRACT_CMD --movement-stats-only"
fi

eval $EXTRACT_CMD

# ---- 输出结果 JSON 到终端 ----
echo ""
echo "=== Result JSON ==="
cat "$RESULT_JSON"
echo ""

# 追加行到 md 时加简单 mkdir 锁避免多进程冲突。
# 行号必须在锁内计算，否则并行结束时会拿到相同 idx。
while ! mkdir "$_LOCK_FILE" 2>/dev/null; do sleep 0.05; done
trap 'rmdir "$_LOCK_FILE" 2>/dev/null || true' EXIT

if [ ! -s "$RESULTS_MD" ]; then
    cat > "$RESULTS_MD" << 'EOF'
# Benchmark Results

| # | 场景 | agents | steps | 完成任务 | 吞吐量(t/tick) | ws | sched_1st(ms) | sched_avg(ms) | total(ms) | wall(s) | ENV覆盖 | 备注 |
|---|------|--------|-------|---------|---------------|--------|--------------|--------------|-----------|---------|---------|------|
EOF
fi

ROW=$(python3 << PYEOF
import json
d = json.load(open('$RESULT_JSON'))
r = d.get('result', {})
p = d.get('performance', {})
env_ov = d.get('env_overrides', {})
env_str = ' '.join(f'{k}={v}' for k, v in env_ov.items()) if env_ov else '-'
tag = '''$TAG''' if '''$TAG''' else '-'

try:
    with open('$RESULTS_MD') as f:
        lines = f.readlines()
    row_ids = []
    for line in lines:
        if not line.startswith('|') or line.startswith('|---') or line.startswith('| #'):
            continue
        parts = [p.strip() for p in line.strip().strip('|').split('|')]
        if parts and parts[0].isdigit():
            row_ids.append(int(parts[0]))
    idx = (max(row_ids) + 1) if row_ids else 0
except Exception:
    idx = 0

fields = [
    str(idx),
    d.get('scene', '-'),
    str(r.get('teamSize', '-')),
    str(d.get('simulation_steps', '-')),
    str(r.get('numTaskFinished', '-')),
    f"{r.get('throughput', 0):.4f}" if 'throughput' in r else '-',
    str(r.get('exec_window_size', '-')) if 'exec_window_size' in r else '-',
    f"{p.get('schedule_1st_ms', 0):.2f}" if 'schedule_1st_ms' in p else '-',
    f"{p.get('schedule_avg_ms', 0):.2f}" if 'schedule_avg_ms' in p else '-',
    f"{p.get('total_compute_ms', 0):.1f}" if 'total_compute_ms' in p else '-',
    str(d.get('wall_time_sec', '-')),
    env_str,
    tag,
]
print('| ' + ' | '.join(fields) + ' |')
PYEOF
)

echo "$ROW" >> "$RESULTS_MD"
rmdir "$_LOCK_FILE" 2>/dev/null || true
trap - EXIT

echo "=== Appended to $RESULTS_MD ==="
echo "$ROW"

# ---- 清理临时文件 ----
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
FINAL_STEM="${SCENE_NAME}_${STEPS}tick_${TIMESTAMP}_${_PID_SFX}"
FINAL_JSON="$REPO_ROOT/result/${FINAL_STEM}.json"
mv "$RESULT_JSON" "$FINAL_JSON"

if $KEEP_LOG; then
    FINAL_LOG="$REPO_ROOT/result/${FINAL_STEM}.stderr.log"
    mv "$TMP_STDERR" "$FINAL_LOG"
    echo ""
    echo "Result JSON: $FINAL_JSON"
    echo "Stderr  Log: $FINAL_LOG"
else
    rm -f "$TMP_STDERR"
    echo ""
    echo "Result JSON: $FINAL_JSON"
fi
# ---- 可视化支持 ----
if $VIZ; then
    # 从输入 JSON 中提取 mapFile 相对路径，拼接为绝对路径
    INPUT_DIR=$(dirname "$INPUT_FILE")
    MAP_REL=$(python3 -c "import json; print(json.load(open('$INPUT_FILE'))['mapFile'])")
    MAP_ABS="$INPUT_DIR/$MAP_REL"

    VIZ_JSON="$REPO_ROOT/result/${FINAL_STEM}_viz.json"
    # 使用 lifelong 的完整输出（包含 actualPaths/events 等 PlanViz 所需信息）
    if [ -f "$TMP_OUTPUT" ]; then
        cp "$TMP_OUTPUT" "$VIZ_JSON"
    else
        echo "Warning: lifelong output file not found, viz JSON not generated"
    fi
fi

rm -f "$TMP_OUTPUT"
echo "Benchmark MD: $RESULTS_MD"

if $VIZ && [ -f "$VIZ_JSON" ]; then
    VIZ_CMD=("$REPO_ROOT/result/visualize_result.sh" --scene "$SCENE" --plan "$VIZ_JSON" --start "$VIZ_START" --agent-start "$VIZ_AGENT_START" --agent-stride "$VIZ_AGENT_STRIDE")
    if [ -n "$VIZ_END" ]; then
        VIZ_CMD+=(--end "$VIZ_END")
    elif [ -n "$VIZ_WINDOW" ] && [ "$VIZ_WINDOW" -gt 0 ] 2>/dev/null; then
        VIZ_CMD+=(--window "$VIZ_WINDOW")
    fi
    if [ -n "$VIZ_AGENTS" ]; then
        VIZ_CMD+=(--agents "$VIZ_AGENTS")
    fi

    echo ""
    echo "=== Visualization Data ==="
    echo "Viz JSON : $VIZ_JSON"
    echo "Map file : $MAP_ABS"
    echo ""
    echo "Next command to visualize:"
    printf "  "
    printf "%q " "${VIZ_CMD[@]}"
    echo ""
fi
