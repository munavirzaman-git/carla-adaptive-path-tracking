"""Global configuration for the Town03 adaptive path-tracking study (CARLA 0.9.16)."""
import os

HOST = os.environ.get("CARLA_HOST", "localhost")
PORT = int(os.environ.get("CARLA_PORT", 2000))
TOWN = "Town03"
VEHICLE_BP = "vehicle.tesla.model3"
DT = 0.05                      # synchronous fixed step [s] (20 Hz control)

RESULTS_DIR = "results"
REF_FILE = os.path.join(RESULTS_DIR, "reference_town03.npz")

# Reference path generation
ROUTE_RESOLUTION = 2.0         # GlobalRoutePlanner sampling [m]
PATH_DS = 0.5                  # resampled spacing [m]
SMOOTH_WINDOW_M = 6.0          # moving-average window on x,y [m]
ROUTE_LENGTH_RANGE = (600.0, 1100.0)
ROUTE_TRIALS = 60
ROUTE_SEED = 7
KAPPA_SMOOTH_M = 5.0           # curvature smoothing window for control / speed profile [m]
KAPPA_MAX = 0.17               # reject routes tighter than ~5.9 m radius (checked on lightly smoothed kappa)

# Speed profile
A_LON_MAX = 2.0                # [m/s^2]
V_START = 2.0                  # [m/s]
V_END = 3.0                    # [m/s]

# Limits / termination
MAX_STEER_DEG = 40.0           # road-wheel angle limit used by all controllers
OFFTRACK_LIMIT = 5.0           # |e_y| [m] that counts as failure
T_MAX = 300.0                  # [s]
