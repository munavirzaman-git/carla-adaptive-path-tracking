"""CARLA helpers. The ONLY place where CARLA's left-handed frame is converted:
    x_rh = x,  y_rh = -y,  psi_rh = -radians(yaw),  r_rh = -radians(omega_z)
    CARLA steer = -delta / max_wheel_steer   (CARLA steer > 0 turns right)."""
import math

import carla

from vehicle_state import Geometry, State, with_pose


def connect(host, port, timeout=30.0):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    return client


def load_town(client, town):
    world = client.get_world()
    if not world.get_map().name.endswith(town):
        print(f"[sim] loading {town} ...")
        world = client.load_world(town)
    return world


def enable_sync(world, dt):
    original = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = dt
    s.substepping = True
    s.max_substep_delta_time = 0.01
    s.max_substeps = 10
    world.apply_settings(s)
    return original


def spawn_vehicle(world, bp_name, x, y, z, psi):
    bp = world.get_blueprint_library().find(bp_name)
    bp.set_attribute("role_name", "hero")
    tf = carla.Transform(carla.Location(x=float(x), y=float(-y), z=float(z) + 0.5),
                         carla.Rotation(yaw=float(-math.degrees(psi))))
    veh = world.try_spawn_actor(bp, tf)
    if veh is None:
        raise RuntimeError("Spawn failed at route start (occupied?). Clear the map or pick another route.")
    return veh


def configure_physics(vehicle, friction_scale=1.0, mass_scale=1.0, flat_steering_curve=True):
    pc = vehicle.get_physics_control()
    pc.mass = pc.mass * mass_scale
    wheels = pc.wheels
    for w in wheels:
        w.tire_friction = w.tire_friction * friction_scale
    pc.wheels = wheels
    if flat_steering_curve:
        pc.steering_curve = [carla.Vector2D(0.0, 1.0), carla.Vector2D(300.0, 1.0)]
    vehicle.apply_physics_control(pc)


def measure_geometry(vehicle):
    """Wheelbase and axle offsets from wheel positions (call after at least one tick)."""
    pc = vehicle.get_physics_control()
    tf = vehicle.get_transform()
    f, o = tf.get_forward_vector(), tf.location
    proj = [(w.position.x / 100.0 - o.x) * f.x + (w.position.y / 100.0 - o.y) * f.y for w in pc.wheels]
    front, rear = 0.5 * (proj[0] + proj[1]), 0.5 * (proj[2] + proj[3])
    L = front - rear
    if not 2.0 < L < 4.0:
        print(f"[sim] WARNING: odd wheelbase {L:.2f} m from physics; using Model 3 defaults")
        front, rear, L = 1.44, -1.44, 2.875
    max_steer = max(pc.wheels[0].max_steer_angle, pc.wheels[1].max_steer_angle)
    geom = Geometry(L, rear, front, max_steer)
    print(f"[sim] wheelbase {L:.3f} m, rear {rear:+.2f} m, front {front:+.2f} m, "
          f"max wheel steer {max_steer:.1f} deg, mass {pc.mass:.0f} kg, "
          f"tire_friction {pc.wheels[0].tire_friction:.2f}")
    return geom


def read_state(vehicle, geom, t):
    tf = vehicle.get_transform()
    vel, acc, w = vehicle.get_velocity(), vehicle.get_acceleration(), vehicle.get_angular_velocity()
    f, rt = tf.get_forward_vector(), tf.get_right_vector()
    v = vel.x * f.x + vel.y * f.y + vel.z * f.z
    ay = -(acc.x * rt.x + acc.y * rt.y + acc.z * rt.z)
    st = State(t=t, x=tf.location.x, y=-tf.location.y, psi=-math.radians(tf.rotation.yaw),
               v=v, r=-math.radians(w.z), ay=ay)
    return with_pose(st, st.x, st.y, st.psi, geom)


def delta_to_carla_steer(delta, geom):
    return max(-1.0, min(1.0, -math.degrees(delta) / geom.max_wheel_steer_deg))


def follow_spectator(world, vehicle, back=9.0, up=4.0):
    tf = vehicle.get_transform()
    f = tf.get_forward_vector()
    loc = tf.location - carla.Location(x=back * f.x, y=back * f.y) + carla.Location(z=up)
    world.get_spectator().set_transform(
        carla.Transform(loc, carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw)))


def draw_path(world, path, every=6, life=900.0):
    for i in range(0, path.n, every):
        world.debug.draw_point(carla.Location(x=float(path.x[i]), y=float(-path.y[i]),
                                              z=float(path.z[i]) + 0.3),
                               size=0.06, color=carla.Color(0, 200, 255), life_time=life)
