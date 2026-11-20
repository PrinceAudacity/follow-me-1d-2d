import time
import numpy as np
from mbot_bridge.api import MBot


def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.
    Make sure you ignore any rays with length 0! Those are invalid.
    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.
    Returns:
        tuple: The length and angle of the shortest ray in the Lidar scan.
    """
    min_dist, min_angle = None, None
    # TODO: Find the length and angle of the shortest distance in the ray.
    ranges = np.array(ranges)
    thetas = np.array(thetas)

    # Ignore invalid (zero-length) rays.
    valid_idx = (ranges > 0).nonzero()
    valid_ranges = ranges[valid_idx]
    valid_thetas = thetas[valid_idx]

    if len(valid_ranges) == 0:
        return min_dist, min_angle

    # Find the index of the shortest valid ray.
    min_idx = np.argmin(valid_ranges)
    min_dist = valid_ranges[min_idx]
    min_angle = valid_thetas[min_idx]

    return min_dist, min_angle


robot = MBot()

setpoint = 0.5   # TODO: Pick your setpoint (meters).
Kp = 1.0         # TODO: Tune this proportional gain.
max_speed = 0.3  # Cap speed so the controller doesn't command something wild.

try:
    while True:
        # Read the latest Lidar scan.
        ranges, thetas = robot.read_lidar()

        # Get the distance and angle to the wall.
        dist_to_wall, angle_to_wall = find_min_dist(ranges, thetas)

        # TODO: Implement the 2D Follow Me controller
        # Hint: Look at your code from follow_1D
        # Hint: When you compute the velocity command, you might find the functions
        # np.sin(value) and np.cos(value) helpful!
        if dist_to_wall is not None:
            # Error is positive if we're farther than setpoint (drive toward wall),
            # negative if we're closer (drive away from it).
            error = dist_to_wall - setpoint
            speed = Kp * error
            speed = max(-max_speed, min(max_speed, speed))

            # Decompose the speed along the direction of the closest ray so the
            # robot drives straight at (or away from) the nearest point, not
            # just straight ahead.
            vx = speed * np.cos(angle_to_wall)
            vy = speed * np.sin(angle_to_wall)
        else:
            # No valid readings this scan — don't move.
            vx, vy = 0, 0

        # Command the robot: forward/lateral speed, no rotation.
        robot.drive(vx, vy, 0)

        # Optionally, sleep for a bit before reading a new scan.
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Stopping (Ctrl+C).")
finally:
    # Always stop the robot, whether we quit or something crashed.
    robot.stop()
