import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Robot geometry
wheel_diameter = 0.0463  # m
wheel_track = 0.055      # m (outside to outside)
wheel_radius = wheel_diameter / 2.0

def compute_paths(span, T):
    height = 1.0
    half_span = span / 2.0
    a = -height / (half_span ** 2)  # parabola: y = a*x^2 + height

    # Arc length integration via Simpson's rule
    def integrand_arc(x):
        return math.sqrt(1.0 + (2.0 * a * x) ** 2)

    n = 10000
    dx = span / n
    total_arc = 0.0
    for i in range(n + 1):
        x = -half_span + i * dx
        coeff = 4 if i % 2 != 0 else 2
        if i == 0 or i == n:
            coeff = 1
        total_arc += coeff * integrand_arc(x)
    L_center = (dx / 3.0) * total_arc

    # Heading change
    def slope(x):
        return 2.0 * a * x
    theta_start = math.atan(slope(-half_span))
    theta_end = math.atan(slope(half_span))
    theta_total = theta_end - theta_start  # radians

    # Wheel path lengths
    L_left = L_center - (wheel_track / 2.0) * theta_total
    L_right = L_center + (wheel_track / 2.0) * theta_total

    # Wheel rotations
    circumference = math.pi * wheel_diameter
    rot_left = L_left / circumference
    rot_right = L_right / circumference

    # Linear wheel velocities
    v_left = L_left / T
    v_right = L_right / T

    # Angular wheel velocities (rad/s)
    omega_left = v_left / wheel_radius
    omega_right = v_right / wheel_radius

    # Build x/y for centerline
    x_vals = [x for x in frange(-half_span, half_span, 0.002)]
    y_center = [a * x**2 + height for x in x_vals]

    # Offset left/right wheel paths along normal
    y_left, y_right = [], []
    for i, x in enumerate(x_vals):
        dy_dx = slope(x)
        norm_len = math.sqrt(1.0 + dy_dx**2)
        nx = -dy_dx / norm_len
        ny = 1.0 / norm_len
        y_left.append(y_center[i] + (wheel_track / 2.0) * ny)
        y_right.append(y_center[i] - (wheel_track / 2.0) * ny)

    return {
        "theta_start_deg": math.degrees(theta_start),
        "L_center_m": L_center,
        "L_left_m": L_left,
        "L_right_m": L_right,
        "rot_left": rot_left,
        "rot_right": rot_right,
        "v_left_mps": v_left,
        "v_right_mps": v_right,
        "omega_left_rads": omega_left,
        "omega_right_rads": omega_right,
        "x_vals": x_vals,
        "y_center": y_center,
        "y_left": y_left,
        "y_right": y_right
    }

def frange(start, stop, step):
    while start <= stop:
        yield start
        start += step

def animate_vehicle(data, T):
    x_vals = data["x_vals"]
    y_center = data["y_center"]
    y_left = data["y_left"]
    y_right = data["y_right"]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(x_vals, y_center, 'b', label='Center Path', linewidth=2)
    ax.plot(x_vals, y_left, 'r--', label='Left Wheel Path')
    ax.plot(x_vals, y_right, 'g--', label='Right Wheel Path')
    ax.axhline(0, color='black', linestyle='--', linewidth=1)
    ax.set_title("Vehicle Following Parabolic Path")
    ax.set_xlabel("Horizontal distance (m)")
    ax.set_ylabel("Height (m)")
    ax.legend()
    ax.grid(True)

    vehicle_body, = ax.plot([], [], 'k', linewidth=3)
    left_wheel, = ax.plot([], [], 'ro', markersize=8)
    right_wheel, = ax.plot([], [], 'go', markersize=8)

    num_frames = len(x_vals)
    interval = (T * 1000.0) / num_frames  # ms per frame

    def init():
        vehicle_body.set_data([], [])
        left_wheel.set_data([], [])
        right_wheel.set_data([], [])
        return vehicle_body, left_wheel, right_wheel

    def update(frame):
        x_c = x_vals[frame]
        y_c = y_center[frame]
        x_l = x_vals[frame]
        y_l = y_left[frame]
        x_r = x_vals[frame]
        y_r = y_right[frame]

        # Vehicle body as a line between wheel points
        vehicle_body.set_data([x_l, x_r], [y_l, y_r])

        # Wheels
        left_wheel.set_data([x_l], [y_l])
        right_wheel.set_data([x_r], [y_r])

        return vehicle_body, left_wheel, right_wheel

    ani = animation.FuncAnimation(
        fig, update, frames=num_frames, init_func=init,
        interval=interval, blit=True, repeat=False
    )
    plt.show()

if __name__ == "__main__":
    span = float(input("Enter track length in meters (7 - 10): "))
    T = float(input("Enter desired travel time in seconds: "))
    if 7 <= span <= 10:
        res = compute_paths(span, T)
        print(f"Starting angle: {res['theta_start_deg']:.4f} deg")
        print(f"Traversal time: {T} s")
        print(f"Left wheel angular velocity:  {res['omega_left_rads']:.4f} rad/s")
        print(f"Right wheel angular velocity: {res['omega_right_rads']:.4f} rad/s")
        print(f"Left wheel rotations:  {res['rot_left']:.4f}")
        print(f"Right wheel rotations: {res['rot_right']:.4f}")
        animate_vehicle(res, T)
    else:
        print("Please enter a length between 7 and 10 meters.")