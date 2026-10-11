I need to run experiments to figure out if coverage path planning cost is related to object pose. 
We already have HGTSP implementation, so I want to use this as a baseline. 
First step is to tune parameters to improve repeatability. for example, i don't want from run to run the cost varies from 100 to 140. i want results to be repeatable. from my previous experience, you need multiple ik solutions per viewpoint to make results repeatable. 
so your first task is, experiment 1: find out the minimum parameters to make HGTSP costs repeatable (e.g. number of IK solutions, IK timeout, etc.) across different random seeds because i want to make my experiments quick. you can use a predefined viewpoint order like i saved in tmp. let's unify our terminology: use viewpoint order instead of route or other shit.
note: i don't care about surface sampling seed. i care about IK sampling seed. Use edge instead of leg
Summary: use the predefined viewpoint order saved in tmp and tun DP with different IK parameters.
Recommended config: max_solutions_per_candidate=4, ik_timeout=0.15, ik_retries_per_point=14, solve_restarts=3

(modification: different seeds causes variation around 10%. so from now on stick with 1 seed. Conclusion: seed doesn't matter)
Updated num_ik=5 and num_restart=3.

Now you've found the recommended config. 
experiment 2: Use this config, Sweep object x/y within +-15cm and solve for HGTSP. Only test positions where all viewpoints are reachable.
Keep viewpoint selection the same, and then record optimal position and its cost.
Summary: position matters. optimal position x~-=5cm.

experiment 3: at the optimal position you found, try different object orientation. Resample the same number of viewpoints at each orientation, and run HGTSP and compare costs.

Setup: position held at x=-5cm, y=0. At each orientation the 38 viewpoints are re-selected
(greedy set-cover) from the pool reachable at that orientation, then real HGTSP.

Findings:
- 5x5 roll/pitch grid (+-20deg): grid mean 83.4 = reference (0,0) 85.7. cv 17% across the
  grid, same as the single-run HGTSP noise. No smooth structure.
- Roll 0..180deg (30deg step): nothing beats roll 0 (cheapest of all 7).
- Y-rotation (pitch) 0..90deg, 5 runs/angle, min-of-5: 77 / 76 / 78 / 82. Flat within 6
  points; each angle's own 5 runs span 12-30 points.
- Coverage is pool-limited at ~63% everywhere, not a factor.

Summary: object orientation does NOT lower HGTSP cost at this position. Any effect is
smaller than the run-to-run noise (single-run HGTSP cv ~17%; identical config repeated 5x
still spans up to 30%). Keep the object upright.