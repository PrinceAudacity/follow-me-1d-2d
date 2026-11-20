import time
import numpy as np
from mbot_bridge.api import MBot

# Get distance to wall
def find_fwd_dist(ranges, thetas, window=5):
    """Find the distance to the nearest object in front of the robot.
    Args:
        ranges (list): The ranges from the Lidar scan.
        thetas (list): The angles from the Lidar scan.
        window (int, optional): The window to average ranges over. Defaults to 5.
    Returns:
        float: The distance to the nearest obstacle in front of the robot.
    """
    # Grab the rays near the front of the scan.
    fwd_ranges = np.array(ranges[:window] + ranges[-window:])
    fwd_thetas = np.array(thetas[:window] + thetas[-window:])
    # Grab just the positive values.
    valid_idx = (fwd_ranges > 0).nonzero()
    fwd_ranges = fwd_ranges[valid_idx]
    fwd_thetas = fwd_thetas[valid_idx]
    # Compute forward distances.
    if len(fwd_ranges) == 0:
        return None  # No valid rays in front this scan.
    fwd_dists = fwd_ranges * np.cos(fwd_thetas)
    return np.mean(fwd_dists)  # Return the mean.


# Initialize a robot object.
robot = MBot()

setpoint = 0.5   # TODO: Pick your setpoint (meters). 0.5 m is a reasonable starting distance to hold from the wall.
Kp = 1.0         # TODO: Tune this proportional gain.
max_speed = 0.3  # Cap speed so the controller doesn't command something wild.

try:
    # Loop forever.
    while True:
        # Read the latest Lidar scan.
        ranges, thetas = robot.read_lidar()

        # Get the distance to the wall in front of the robot.
        dist_to_wall = find_fwd_dist(ranges, thetas)
        if dist_to_wall is None:
            robot.stop()
            time.sleep(0.1)
            continue

        # TODO: Implement the follow me controller to drive the robot based on
        # the distance to the wall in front.
        error = dist_to_wall - setpoint   # Positive error -> too far, drive forward.
        vx = Kp * error

        # Clamp the speed so it stays within a safe range.
        vx = max(-max_speed, min(max_speed, vx))

        # Command the robot: forward/backward speed, no strafe, no rotation.
        robot.drive(vx, 0, 0)

        # Optionally, sleep for a bit before reading a new scan.
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Stopping (Ctrl+C).")
finally:
    # Always stop the robot, whether we quit or something crashed.
    robot.stop()
