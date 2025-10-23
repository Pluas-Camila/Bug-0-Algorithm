from robot_systems.robot import HamBot
import time

# -----------------------------
# CONFIGURATION PARAMETERS
# -----------------------------

# Goal detection colors (pink shades)
GOAL_COLORS = [(255, 105, 180), (255, 192, 203), (255, 20, 147)]
COLOR_TOLERANCE = 0.12
AREA_THRESHOLD = 400  # minimum landmark area in pixels

# LIDAR thresholds (in mm)
FRONT_OBSTACLE_THRESHOLD = 300  # obstacle detected within 30 cm

# Motion parameters
FORWARD_SPEED = 50  # motor speed
TURN_SPEED = 30     # motor turn speed
TIME_STEP = 0.1     # control loop interval

# -----------------------------
# HELPER FUNCTIONS
# -----------------------------

def goal_visible(robot):
    """Check if the pink goal is visible in camera frame."""
    landmarks = robot.find_landmarks(area_threshold=AREA_THRESHOLD)
    return len(landmarks) > 0, landmarks

def distance_to_obstacle(lidar_data, angle=180):
    """Return distance to obstacle at given lidar angle (default: front)."""
    return lidar_data[angle]

def drive_toward_goal(robot, landmarks):
    """Proportional steering toward goal landmark in camera frame."""
    if not landmarks:
        robot.set_left_speed(FORWARD_SPEED)
        robot.set_right_speed(FORWARD_SPEED)
        return
    
    target = max(landmarks, key=lambda lm: lm.width * lm.height)
    img_center_x = 640 / 2  # assuming camera resolution width
    error_x = target.center_x - img_center_x

    # proportional steering
    Kp = 0.1
    turn_adjust = Kp * error_x
    left_speed = FORWARD_SPEED - turn_adjust
    right_speed = FORWARD_SPEED + turn_adjust

    # clamp speeds
    left_speed = max(min(left_speed, 100), -100)
    right_speed = max(min(right_speed, 100), -100)

    robot.set_left_speed(left_speed)
    robot.set_right_speed(right_speed)

def wall_following(robot, lidar_data):
    """Simple wall-following behavior."""
    left_dist = distance_to_obstacle(lidar_data, 90)
    right_dist = distance_to_obstacle(lidar_data, 270)
    front_dist = distance_to_obstacle(lidar_data, 180)

    if front_dist < FRONT_OBSTACLE_THRESHOLD:
        if left_dist > right_dist:
            robot.set_left_speed(-TURN_SPEED)
            robot.set_right_speed(TURN_SPEED)
        else:
            robot.set_left_speed(TURN_SPEED)
            robot.set_right_speed(-TURN_SPEED)
    else:
        robot.set_left_speed(FORWARD_SPEED / 2)
        robot.set_right_speed(FORWARD_SPEED / 2)

# -----------------------------
# MAIN LOOP
# -----------------------------

def main():
    robot = HamBot(lidar_enabled=True, camera_enabled=True)
    time.sleep(1)  # allow sensors to initialize

    # Set pink goal detection using built-in function
    robot.set_landmark_colors(GOAL_COLORS, tolerance=COLOR_TOLERANCE)

    state = "MG"  # start with Motion-to-Goal

    try:
        while True:
            lidar_data = robot.get_range_image()
            visible, landmarks = goal_visible(robot)

            if state == "MG":
                if distance_to_obstacle(lidar_data) < FRONT_OBSTACLE_THRESHOLD:
                    state = "WF"
                elif visible:
                    drive_toward_goal(robot, landmarks)
                else:
                    robot.set_left_speed(FORWARD_SPEED)
                    robot.set_right_speed(FORWARD_SPEED)

            elif state == "WF":
                wall_following(robot, lidar_data)
                if visible:
                    state = "MG"

            time.sleep(TIME_STEP)

    except KeyboardInterrupt:
        print("Stopping robot.")
    finally:
        robot.stop_motors()

# -----------------------------
# ENTRY POINT
# -----------------------------

if __name__ == "__main__":
    main()
