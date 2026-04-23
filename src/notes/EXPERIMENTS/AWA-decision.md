# AWA Decision: Complete SIPP Implementation for maze

## Date: 2026-04-23 09:40

## AWA Principle: Choose A (Highest Risk/Highest Reward)

### Failed Experiments (All A directions exhausted)
| 实验 | 方法 | 结果 |
|------|------|------|
| A117 | corridor penalty | 0.038 (更差) ❌ |
| A134 | frank_wolfe保守 | crashed ❌ |
| A138 | skeleton SIPP | 0.076 vs 0.090 (更差) ❌ |
| A140 | ReservationTable PIBT | 0.00 (crashed) ❌ |
| A141 | 增加PIBT时间 | 无改进 ❌ |

### Current Baseline (Confirmed in A142)
| Map | Score | Target | Gap |
|-----|-------|--------|-----|
| maze | 0.090 | 0.73 | **-88%** ❌ |
| iron | 0.147 | 0.86 | -83% ❌ |
| room | 0.184 | 0.88 | -79% ❌ |
| orz | 0.216 | 0.86 | -75% ❌ |
| random | 0.510 | 0.98 | -48% ❌ |
| fulfill | 1.082 | 0.83 | **+30%** ✅ |

## Decision: A1 = Complete SIPP Implementation for maze

### Why SIPP is the RIGHT A direction:
1. **maze has the largest gap** - 0.09 vs 0.73 (-88%)
2. Team_RAPID uses `"sipp": true` in their config
3. PIBT fundamentally can't handle temporal conflicts in narrow corridors
4. Complete SIPP could unlock maze + potentially help other maps

### Why Previous SIPP Attempts Failed:
1. **A138**: Skeleton SIPP with empty ReservationTable - no real temporal avoidance
2. **A140**: ReservationTable PIBT - didn't properly integrate SIPP's safe interval concept

### Key Insight from Team_RAPID:
SIPP needs proper **Safe Interval Table (SIT)** - not just simple isFree() checks.

From their ReservationTable.h:
```cpp
list<tuple<int, int, int, bool, bool> > get_safe_intervals(int from, int to, int lower_bound, int upper_bound);
Interval get_first_safe_interval(size_t location);
bool find_safe_interval(Interval& interval, size_t location, int t_min);
```

The safe interval concept is critical: instead of just checking "is position free at time t?", SIPP asks "what are the safe time intervals for this position?"

### Implementation Plan:

**Phase 1: Study Team_RAPID's SIPP.cpp** (30 min)
- Understand how they compute safe intervals
- Understand how they integrate with ReservationTable

**Phase 2: Implement proper Safe Interval computation** (60 min)
- Create SafeIntervalTable based on their approach
- Compute safe intervals from current trajectories

**Phase 3: Integrate with TimeSpaceA*** (60 min)
- Use safe intervals to guide search
- Test on maze first (map-specific check)

**Phase 4: 10-round test on maze** (30 min)
- Compare with baseline 0.090
- If improved, extend to other maps

### Risk Assessment:
- **Risk Level**: Very High (multiple failures before)
- **Potential Reward**: maze 0.09 → 0.40+ (3x+ improvement)
- **Failure modes**: Still crashes like A134/A140, or worse than baseline like A138

### Commitment:
According to AWA: **Choose A, fail forward, never accept D**

I will make ONE serious attempt at complete SIPP implementation for maze. If this fails:
1. Document the failure thoroughly
2. Move to B direction (LaCAM2 research)
3. Accept that some maps need architectural changes beyond current scope

_Last updated: 2026-04-23 09:40 GMT+8_