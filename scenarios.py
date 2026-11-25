"""Road / driving conditions used in the study.

NOTE: CARLA weather is visual only. Grip is changed through each wheel's tire_friction
(friction_scale). CARLA's default tire_friction (3.5) gives about 2.2 g of lateral grip,
so the default-relative "wet" / "icy" scenarios are relative friction reductions, not
real wet or icy roads. The skid-pad calibration (skidpad.py) maps scale -> measured grip:
    1.0 -> 2.24 g, 0.30 -> 1.34 g, 0.15 -> 0.81 g, 0.08 -> 0.42 g, 0.04 -> 0.21 g
"""
from dataclasses import dataclass


@dataclass
class Scenario:
    name: str
    v_max_kmh: float = 40.0
    a_lat_max: float = 3.0            # used to build the speed profile [m/s^2]
    friction_scale: float = 1.0       # multiplies every wheel's tire_friction
    mass_scale: float = 1.0           # multiplies vehicle mass
    flat_steering_curve: bool = True  # False keeps CARLA's speed-dependent steering reduction
    pos_noise: float = 0.0            # GNSS-like position noise std [m]
    yaw_noise_deg: float = 0.0        # heading noise std [deg]
    steer_delay_steps: int = 0        # actuator delay in control steps (x DT)
    weather: str = "ClearNoon"        # carla.WeatherParameters preset name
    thr_max: float = 0.8              # throttle cap (crude traction control on low grip)
    brake_max: float = 1.0            # brake cap (avoid lock-up on low grip)


SCENARIOS = {sc.name: sc for sc in [
    # --- default-grip set ---------------------------------------------------------------
    Scenario("nominal", v_max_kmh=40),
    Scenario("high_speed", v_max_kmh=70, a_lat_max=4.5),
    Scenario("wet", v_max_kmh=50, friction_scale=0.55, weather="HardRainNoon"),
    Scenario("icy", v_max_kmh=35, a_lat_max=2.0, friction_scale=0.3, weather="WetCloudySunset"),
    Scenario("heavy_load", v_max_kmh=50, mass_scale=1.6),
    Scenario("steer_map_error", v_max_kmh=60, a_lat_max=4.0, flat_steering_curve=False),
    Scenario("noisy_delayed", v_max_kmh=50, pos_noise=0.10, yaw_noise_deg=0.5, steer_delay_steps=3),
    Scenario("combined", v_max_kmh=50, friction_scale=0.55, mass_scale=1.4,
             flat_steering_curve=False, pos_noise=0.05, yaw_noise_deg=0.3,
             steer_delay_steps=2, weather="MidRainyNoon"),
    # matched pairs: identical speed profile to nominal / dry_push, only grip differs
    Scenario("wet_matched", v_max_kmh=40, friction_scale=0.55, weather="HardRainNoon"),
    Scenario("icy_matched", v_max_kmh=40, friction_scale=0.3, weather="WetCloudySunset"),
    Scenario("dry_push", v_max_kmh=50, a_lat_max=4.5),
    Scenario("icy_push", v_max_kmh=50, a_lat_max=4.5, friction_scale=0.3, weather="WetCloudySunset"),
    # --- skid-pad calibrated grip (mu-aware speed profile) ------------------------------
    Scenario("dry_real", v_max_kmh=40, a_lat_max=2.6, friction_scale=0.17),        # mu ~ 0.9
    Scenario("dry_real_push", v_max_kmh=50, a_lat_max=4.0, friction_scale=0.17),
    # Calibrated wet / snow: defined for completeness, but NOT usable on Town03 -- CARLA's
    # tire_friction also removes cornering stiffness and traction (see README, Limitations).
    Scenario("wet_real", v_max_kmh=40, a_lat_max=1.5, friction_scale=0.095, thr_max=0.4,
             brake_max=0.5, weather="HardRainNoon"),                                  # mu ~ 0.5
    Scenario("snow_real", v_max_kmh=30, a_lat_max=0.75, friction_scale=0.05, thr_max=0.25,
             brake_max=0.3, weather="WetCloudySunset"),                               # mu ~ 0.25
]}

STUDY_SCENARIOS = [n for n in SCENARIOS if n not in ("wet_real", "snow_real")]
