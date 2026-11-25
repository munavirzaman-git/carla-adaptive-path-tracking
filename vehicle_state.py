"""CARLA-free state containers (so controllers can be tested offline).

Frame convention used everywhere outside sim_utils.py:
right-handed frame obtained from CARLA by  y -> -y,  yaw -> -yaw.
Steering angle delta > 0 turns LEFT.  e_y > 0 means LEFT of the path.
"""
import math
from dataclasses import dataclass, replace


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Geometry:
    wheelbase: float           # [m]
    rear_offset: float         # actor origin -> rear axle along heading [m] (negative)
    front_offset: float        # actor origin -> front axle along heading [m] (positive)
    max_wheel_steer_deg: float # CARLA steer=1.0 corresponds to this road-wheel angle


@dataclass
class State:
    t: float
    x: float      # actor origin (≈ vehicle centre)
    y: float
    psi: float    # heading [rad]
    v: float      # longitudinal speed [m/s]
    r: float      # yaw rate [rad/s]
    ay: float     # lateral acceleration, left positive [m/s^2]
    xr: float = 0.0  # rear axle centre
    yr: float = 0.0
    xf: float = 0.0  # front axle centre
    yf: float = 0.0


def with_pose(st: State, x: float, y: float, psi: float, geom: Geometry) -> State:
    c, s = math.cos(psi), math.sin(psi)
    return replace(st, x=x, y=y, psi=psi,
                   xr=x + geom.rear_offset * c, yr=y + geom.rear_offset * s,
                   xf=x + geom.front_offset * c, yf=y + geom.front_offset * s)
