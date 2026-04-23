# AWA Decision: Next Steps for MAPF Optimization

## Current Status (2026-04-23)

### Baseline Scores
| 地图 | 分数 | 目标 | GAP | 状态 |
|------|------|------|-----|------|
| fulfill | **1.019** | 0.83 | +23% | ✅ 已达标 |
| random | **0.527** | 0.98+ | -46% | 需算法突破 |
| room | **0.185** | 0.88 | -79% | 需算法突破 |
| maze | **0.090** | 0.73 | -88% | 需算法突破 |
| bos | **0.307** | ??? | ??? | 待研究 |
| orz | **0.200** | 0.86 | -77% | 待研究 |
| iron | **0.149** | 0.86 | -83% | 待研究 |

### Total: ~2.48 (Target: ~6.0 based on individual targets)

## AWA Analysis

### High-Risk/High-Reward Options

**A: Complete SIPP Implementation for maze**
- Risk: Very High (multiple attempts failed)
- Reward: Potential 0.09 → 0.40+ for maze
- Work: Large (needs ReservationTable + proper integration)

**B: LNS + LaCAM2 Research**
- Risk: Medium (need to understand complex implementation)
- Reward: Could help random, room, etc.
- Work: Large (Team_RAPID's implementation is complex)

**C: Corridor-Aware PIBT Modification**
- Risk: Medium (simpler than full SIPP)
- Reward: Unknown (may not be enough)
- Work: Medium

### Failed Experiments Summary
| 实验 | 方法 | 结果 |
|------|------|------|
| A117 | Corridor penalty | 0.038 (更差) ❌ |
| A134 | frank_wolfe for maze | 崩溃 ❌ |
| A138 | Skeleton SIPP | 0.076 (更差) ❌ |
| A140 | ReservationTable PIBT | 0.00 (崩溃) ❌ |

## Decision

Given that:
1. All simple modifications have failed
2. SIPP implementation has failed multiple times
3. The gap is huge (2.5 vs target 6.0)

**I will try Option C: Corridor-Aware PIBT for maze**

### Implementation Plan
1. Add simple corridor detection (neighbors <= 2)
2. For agents IN corridors, use a simple "claim and hold" strategy
3. For agents AT corridor entrances, wait for corridor to clear
4. This is simpler than full SIPP but addresses the core issue

### Why This Might Work
- Corridor deadlock is the likely cause of poor maze performance
- Full SIPP failed due to complexity
- Simple rule-based approach may capture most benefit
- Only affects maze (map-specific check)

## Next Steps
1. Implement corridor-aware modification for maze only
2. Test and verify
3. If fails, accept that maze needs full architectural changes

_Last updated: 2026-04-23 07:50 GMT+8_