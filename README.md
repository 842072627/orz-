# ORZ Game-only submission

This directory is a minimal LORR 2026 C++ submission for `game.domain`.
It contains only the active Game scheduler, planner, executor, their direct
dependencies, and the official local runner framework.

```bash
./compile.sh
./build/lifelong --inputFile ../MAIN2026/example_problems/game.domain/orz-example_1800.json \
  --preprocessTimeLimit 300000 --simulationTime 100 \
  --output ../result/orztest-smoke.json
```

The algorithm selection is intentionally fixed:

- `GameBalanceOptScheduler`
- `GameDirectionalPlanner`
- ORZ branch of `IronPlatoonExecutor`

Map structure is classified with the same `MapProfile` logic as MAIN2026.
`game.domain` runs the Game pipeline above; every other recognized domain sets
`skip_test=true` and safely returns stationary actions. `OPT_SKIP_TEST=0/1`
remains available as the highest-priority runtime override.

For a recorded local Game 4k run, use the same interface as MAIN2026:

```bash
./result/run_and_record.sh --scene game4k --steps 5000 --keep-log --viz \
  --env OPT_SKIP_TEST=0
```

The runner uses this repository's `build/lifelong`. To keep the submission
small, it reads test instances from the sibling `MAIN2026/example_problems`
directory by default; set `LORR_INPUT_ROOT` when the instances live elsewhere.
