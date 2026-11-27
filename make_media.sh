#!/bin/bash
# Build every image, GIF and video used in the README.
#   part 1 needs only results/ ; part 2 needs a running CARLA 0.9.16 server (about 15-25 min).
set -e
export PYTHONUNBUFFERED=1
cd "$(dirname "$0")"

echo "### 1/2 data-driven images and GIFs (no CARLA)"
python make_figures.py
python make_animations.py

echo "### 2/2 CARLA camera recordings"
python record_demo.py -s combined        -c stanley adaptive_mrac_rob_prev --t-end 45
python record_demo.py -s steer_map_error -c stanley ampc_ratio             --t-end 40
python record_demo.py -s noisy_delayed   -c adaptive_mrac_cal adaptive_mrac_rob --t-end 50
python record_demo.py -s dry_real_push   -c stanley adaptive_mrac_rob_prev --view top --t-end 25
python record_demo.py -s nominal         -c adaptive_mrac_rob_prev --view top --t-end 30
echo "### media done"; du -sh media
