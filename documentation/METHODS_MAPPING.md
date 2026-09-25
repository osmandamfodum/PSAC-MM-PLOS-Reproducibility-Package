# Methods and scenarios

## Methods

- **B1:** direct target-centre navigation.
- **B2:** one fixed-offset service pose.
- **B3:** bounded multi-candidate planning and safety filtering without persistent target management.
- **P:** complete PSAC-MM, including persistent target lifecycle, multi-candidate evaluation, safety filtering, bounded replanning and bounded recovery.

## Scenarios

- **S1:** open/static configuration; dynamic obstacles are parked outside the active farm region.
- **S2:** constrained crop-row access.
- **S3:** constrained crop-row access with active patrol obstacles.

The precise runtime selection is implemented through the `UAV_UGV_SCENARIO` and `UAV_UGV_METHOD` environment variables and corresponding arguments in the retained runner and mission manager.
