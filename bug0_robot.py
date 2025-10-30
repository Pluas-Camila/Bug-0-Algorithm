from robot_systems.robot import HamBot
from robot_systems.camera import Camera
import time

# -----------------------------
# CONFIGURATION PARAMETERS
# -----------------------------

# Goal detection colors (pink shades)
GOAL_COLORS = [(255, 0, 178), (255, 0, 203), (255, 0, 147)]
COLOR_TOLERANCE = 0.12
AREA_THRESHOLD = 400  # minimum landmark area in pixels
TARGET_WIDTH = 600
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
    try:
        landmarks = robot.camera.find_landmarks(AREA_THRESHOLD)
    except TypeError:
        landmarks = robot.camera.find_landmarks()
    return len(landmarks) > 0, landmarks

def distance_to_obstacle(lidar_data, angle=180):
    """Return distance to obstacle at given lidar angle (default: front)."""
    return lidar_data[angle]

def drive_toward_goal(robot, landmarks):
    """Proportional steering toward goal landmark in camera frame."""
    if not landmarks:
        robot.set_left_motor_speed(FORWARD_SPEED)
        robot.set_right_motor_speed(FORWARD_SPEED)
        return
    
    # Pick the largest landmark
    target = max(landmarks, key=lambda lm: lm.width * lm.height)
    img_center_x = 640 / 2  # assuming camera resolution width

    # Compatibility with older/newer Landmark API
    center_x = getattr(target, "center_x", getattr(target, "x", 0))

    # proportional steering
    Kp = 0.1
    error_x = center_x - img_center_x
    turn_adjust = Kp * error_x
    left_speed = max(min(FORWARD_SPEED - turn_adjust, 100), -100)
    right_speed = max(min(FORWARD_SPEED + turn_adjust, 100), -100)

    robot.set_left_motor_speed(left_speed)
    robot.set_right_motor_speed(right_speed)

def wall_following(robot, lidar_data):
    """Simple wall-following behavior."""
    left_dist = distance_to_obstacle(lidar_data, 90)
    right_dist = distance_to_obstacle(lidar_data, 270)
    front_dist = distance_to_obstacle(lidar_data, 180)

    if front_dist < FRONT_OBSTACLE_THRESHOLD:
        if left_dist > right_dist:
            robot.set_left_motor_speed(-TURN_SPEED)
            robot.set_right_motor_speed(TURN_SPEED)
        else:
            robot.set_left_motor_speed(TURN_SPEED)
            robot.set_right_motor_speed(-TURN_SPEED)
    else:
        robot.set_left_motor_speed(FORWARD_SPEED / 2)
        robot.set_right_motor_speed(FORWARD_SPEED / 2)

# -----------------------------
# MAIN LOOP
# -----------------------------

def main():
    robot = HamBot(lidar_enabled=True, camera_enabled=True)
    time.sleep(1)  # allow sensors to initialize

    # Configure camera to detect goal colors
    robot.camera.set_target_colors(GOAL_COLORS, tolerance=COLOR_TOLERANCE)

    state = "MG"  # start with Motion-to-Goal

    try:
        while True:
            lidar_data = robot.get_range_image()
            visible, landmarks = goal_visible(robot)

            # -----------------------------
            # GOAL REACHING LOGIC (NEW)
            # -----------------------------
            if visible:
                # Pick the largest visible landmark
                target = max(landmarks, key=lambda lm: lm.width * lm.height)
                width = getattr(target, "width", 0)

                if width >= TARGET_WIDTH:
                    # Landmark is large enough → goal reached
                    print(f"Goal reached! Landmark width = {width}")
                    robot.set_left_motor_speed(0)
                    robot.set_right_motor_speed(0)
                    break  # exit loop
                else:
                    # Drive toward the goal
                    drive_toward_goal(robot, landmarks)
                    continue  # skip wall-following while approaching goal

            # -----------------------------
            # BUG0 behavior
            # -----------------------------
            front_dist = distance_to_obstacle(lidar_data)
            if state == "MG":
                if front_dist < FRONT_OBSTACLE_THRESHOLD:
                    state = "WF"
                else:
                    robot.set_left_motor_speed(FORWARD_SPEED)
                    robot.set_right_motor_speed(FORWARD_SPEED)

            elif state == "WF":
                wall_following(robot, lidar_data)
                if visible:
                    state = "MG"

            time.sleep(TIME_STEP)

    except KeyboardInterrupt:
        print("Stopping robot.")
    finally:
        robot.stop_motors()
        robot.camera.stop()  # properly stop camera thread
        print("Robot stopped safely.")


# -----------------------------
# ENTRY POINT
# -----------------------------

if __name__ == "__main__":
    main()
