#!/usr/bin/env bash
# ================================================================
# visualize_result.sh — 打开已有 *_viz.json 做 PlanViz Qt 可视化
#
# 用法:
#   ./result/visualize_result.sh --scene maze --plan result/maze_40_3000tick_xxx_viz.json
#   ./result/visualize_result.sh --scene iron_harvest --plan latest --window 2000 --agents 300
#   ./result/visualize_result.sh --scene warehousegen --plan latest --window 500 --agents 300
#   ./result/visualize_result.sh --map /path/to/map.map --plan /path/to/output_viz.json --start 500 --end 2500
#   ./result/visualize_result.sh --scene iron_harvest --plan latest --agents 300 --agent-start 5000 --agent-stride 10
# ================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -n "${LORR_INPUT_ROOT:-}" ]; then
    PROJECT_ROOT="$LORR_INPUT_ROOT"
elif [ -d "$REPO_ROOT/example_problems" ]; then
    PROJECT_ROOT="$REPO_ROOT"
else
    PROJECT_ROOT="$REPO_ROOT/../MAIN2026"
fi

SCENE=""
PLAN_JSON=""
MAP_PATH=""
START_TICK=0
END_TICK=""
WINDOW_TICKS=0
AGENTS=""
AGENT_START=0
AGENT_STRIDE=1
PRINT_ONLY=false

usage() {
    sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
}

resolve_scene() {
    case "$1" in
        warehouse)       echo "$PROJECT_ROOT/example_problems/warehouse.domain/fulfill-example_2500.json" ;;
        warehouse_large) echo "$PROJECT_ROOT/example_problems/warehouse.domain/warehouse_large_5000.json" ;;
        warehouse_7500)  echo "$PROJECT_ROOT/example_problems/warehouse.domain/fulfill-example_7500.json" ;;
        warehouse_10000) echo "$PROJECT_ROOT/example_problems/warehouse.domain/fulfill-example_10000.json" ;;
        warehousegen|warehouse_gen|warehousegen_10000|warehouse_online|warehouse_online_10000) echo "$PROJECT_ROOT/example_problems/warehouse.domain/fulfill-warehousegen-example_10000.json" ;;
        city)            echo "$PROJECT_ROOT/example_problems/city.domain/bos-example_600.json" ;;
        city2000|city_2000|bos2000) echo "$PROJECT_ROOT/example_problems/city.domain/bos-example_2000.json" ;;
        game)            echo "$PROJECT_ROOT/example_problems/game.domain/orz-example_1800.json" ;;
        game4k)          echo "$PROJECT_ROOT/example_problems/game.domain/orz-example_4000.json" ;;
        iron_harvest)    echo "$PROJECT_ROOT/example_problems/iron_harvest.domain/iron-example_10000.json" ;;
        maze)            echo "$PROJECT_ROOT/example_problems/maze.domain/maze-example_40.json" ;;
        maze50|maze_50)  echo "$PROJECT_ROOT/example_problems/maze.domain/maze-example_50.json" ;;
        random)          echo "$PROJECT_ROOT/example_problems/random.domain/random-example_400.json" ;;
        random1500|random_1500) echo "$PROJECT_ROOT/example_problems/random.domain/random-example_1500.json" ;;
        randomA|random_a) echo "$PROJECT_ROOT/example_problems/random.domain/randomA_instance_config.json" ;;
        room)            echo "$PROJECT_ROOT/example_problems/room.domain/room-example_150.json" ;;
        room200|room_200) echo "$PROJECT_ROOT/example_problems/room.domain/room-example_200.json" ;;
        *)               echo "$1" ;;
    esac
}

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
        --scene) SCENE="$2"; shift 2 ;;
        --plan) PLAN_JSON="$2"; shift 2 ;;
        --map) MAP_PATH="$2"; shift 2 ;;
        --start) START_TICK="$2"; shift 2 ;;
        --end) END_TICK="$2"; shift 2 ;;
        --window) WINDOW_TICKS="$2"; shift 2 ;;
        --agents|--n) AGENTS="$2"; shift 2 ;;
        --agent-start) AGENT_START="$2"; shift 2 ;;
        --agent-stride|--agent-step) AGENT_STRIDE="$2"; shift 2 ;;
        --print|--dry-run) PRINT_ONLY=true; shift ;;
        --help|-h) usage; exit 0 ;;
        *) echo "Unknown arg: $1"; usage; exit 1 ;;
    esac
done

if [ -z "$PLAN_JSON" ]; then
    echo "Error: --plan <json|latest> is required"
    usage
    exit 1
fi

if [ "$PLAN_JSON" = "latest" ]; then
    if [ -z "$SCENE" ]; then
        echo "Error: --plan latest requires --scene"
        exit 1
    fi
    SCENE_NAME=$(resolve_short_name "$SCENE")
    PLAN_JSON=$(SCENE_NAME="$SCENE_NAME" REPO_ROOT="$REPO_ROOT" python3 - <<'PY'
import glob
import os

root = os.environ["REPO_ROOT"]
scene = os.environ["SCENE_NAME"]
paths = glob.glob(os.path.join(root, "result", f"{scene}_*tick_*_viz.json"))
paths.sort(key=lambda p: os.path.getmtime(p), reverse=True)
print(paths[0] if paths else "")
PY
)
    if [ -z "$PLAN_JSON" ]; then
        echo "Error: no viz JSON found for scene '$SCENE' under $REPO_ROOT/result"
        exit 1
    fi
fi

if [ ! -f "$PLAN_JSON" ]; then
    echo "Error: plan JSON not found: $PLAN_JSON"
    exit 1
fi

if [ -z "$MAP_PATH" ]; then
    if [ -z "$SCENE" ]; then
        echo "Error: --map is required when --scene is not provided"
        exit 1
    fi
    INPUT_FILE=$(resolve_scene "$SCENE")
    if [ ! -f "$INPUT_FILE" ]; then
        echo "Error: scene input not found: $INPUT_FILE"
        exit 1
    fi
    INPUT_DIR=$(dirname "$INPUT_FILE")
    MAP_REL=$(python3 -c "import json; print(json.load(open('$INPUT_FILE'))['mapFile'])")
    MAP_PATH="$INPUT_DIR/$MAP_REL"
fi

if [ ! -f "$MAP_PATH" ]; then
    echo "Error: map file not found: $MAP_PATH"
    exit 1
fi

PLANVIZ_ROOT="${LORR_PLANVIZ_ROOT:-$REPO_ROOT/..}"
PLANVIZ_QT="$PLANVIZ_ROOT/planviz-qt/build-mac/planviz-qt"
if [ ! -x "$PLANVIZ_QT" ] && [ -x "$PLANVIZ_ROOT/planviz-qt/build/planviz-qt.exe" ]; then
    PLANVIZ_QT="$PLANVIZ_ROOT/planviz-qt/build/planviz-qt.exe"
fi
if [ ! -x "$PLANVIZ_QT" ]; then
    echo "Error: PlanViz Qt binary not found. Build it first:"
    echo "  cd $PLANVIZ_ROOT/planviz-qt && cmake --build build-mac --config Release -j 8"
    exit 1
fi

QT_ARGS=(--map "$MAP_PATH" --plan "$PLAN_JSON" --start "$START_TICK" --agent-start "$AGENT_START" --agent-stride "$AGENT_STRIDE")
if [ -n "$END_TICK" ]; then
    QT_ARGS+=(--end "$END_TICK")
elif [ -n "$WINDOW_TICKS" ] && [ "$WINDOW_TICKS" -gt 0 ] 2>/dev/null; then
    QT_ARGS+=(--window "$WINDOW_TICKS")
fi
if [ -n "$AGENTS" ]; then
    QT_ARGS+=(--n "$AGENTS")
fi

echo "=== PlanViz Qt ==="
echo "Plan   : $PLAN_JSON"
echo "Map    : $MAP_PATH"
echo "Window : start=$START_TICK, end=${END_TICK:-auto}, window=$WINDOW_TICKS"
echo "Agents : ${AGENTS:-all} from $AGENT_START stride $AGENT_STRIDE"
echo ""
printf "Command: "
printf "%q " "$PLANVIZ_QT" "${QT_ARGS[@]}"
echo ""

if ! $PRINT_ONLY; then
    exec "$PLANVIZ_QT" "${QT_ARGS[@]}"
fi
