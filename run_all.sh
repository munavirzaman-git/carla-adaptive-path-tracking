#!/bin/bash
# Reproduce every result in the README (about 110 runs, 2-3 h on a desktop GPU at -quality-level=Low).
# Prerequisites: CARLA 0.9.16 server running on $CARLA_HOST:$CARLA_PORT (default localhost:2000),
# CARLA_ROOT set (for agents.navigation), and the Python environment from requirements.txt active.
set -e
export PYTHONUNBUFFERED=1
cd "$(dirname "$0")"

python offline_check.py                                    # CARLA-free sanity check

echo "### 1/5 grip calibration (skid-pad)"
python skidpad.py --speed 14 --scales 1.0 0.3 0.15 0.08 0.04 | tee results_skidpad.txt

echo "### 2/5 baselines + robust MRAC presets on all study scenarios"
python run_experiments.py -c pure_pursuit stanley \
    adaptive_mrac_rob adaptive_mrac_rob_fixed adaptive_mrac_rob_prev adaptive_mrac_rob_prev_fixed

echo "### 3/5 original high-gain MRAC (delay-robustness comparison)"
DEF="nominal high_speed wet icy heavy_load steer_map_error noisy_delayed combined"
python run_experiments.py -c adaptive_mrac_cal adaptive_mrac_cal_fixed -s $DEF

echo "### 4/5 extra noise seeds"
for S in 1 2; do
  python run_experiments.py -c pure_pursuit stanley adaptive_mrac_rob_prev \
      -s noisy_delayed combined --noise-seed $S --tag _s$S
done

echo "### 5/5 figures and tables"
python make_figures.py
echo "### ALL DONE"
