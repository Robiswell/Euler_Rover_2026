# Compaction State

**Last updated:** 2026-03-22 ~16:30
**Task:** Add QUAD_CRUISE_SPEED to fix quad phase error in competition mode

## Goal
Fix CRITICAL quad gait -- phase error exceeds 29.9 deg overlap margin at CRUISE_SPEED=500. Added QUAD_CRUISE_SPEED=200 so quad runs slower in competition without penalizing tripod/wave.

## Edits APPLIED (3 edits to final_full_gait_test.py)
1. Line 1652: Added `QUAD_CRUISE_SPEED = 200` after CRUISE_SPEED
2. Line 2526: Changed `base_speed = CRUISE_SPEED` to `base_speed = QUAD_CRUISE_SPEED if self.terrain_gait == 2 else CRUISE_SPEED`
3. Line 2918: Changed `speed=400` to `speed=QUAD_CRUISE_SPEED` in fallback_quad_fwd

## Next steps
1. Commit these changes
2. Deploy to Pi
3. User tests with --competition
4. Pull telemetry and analyze

## Future (after QUAD_CRUISE_SPEED test)
- Change QUAD_IMPACT from 330/30 to 315/45 (270 deg air sweep, 42% motor margin)
- 12 hardcoded 345/15 pivot lines (low priority - pivots at speed=0)

## Anchors
- Latest commit: 5c9c56f (pre-edit)
- Gait file ~3734 lines (added 1 line)
- sim_terrain: 14/14, sim_nav: 12/12
