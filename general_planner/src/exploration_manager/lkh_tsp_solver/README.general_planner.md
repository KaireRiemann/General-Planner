This directory imports the LKH TSP/ATSP solver wrapper used by FALCON:

https://github.com/HKUST-Aerial-Robotics/FALCON

Imported source:
- `falcon_planner/exploration_utils/src/lkh_tsp_solver`
- `falcon_planner/exploration_utils/include/lkh_tsp_solver`

`LKHmain.c` is intentionally excluded because `general_planner` calls the
library wrapper `solveTSPLKH()` from `lkh_interface.cpp` in-process.
