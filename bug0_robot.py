import time
import math
from robot_systems.robot import HamBot

# -----------------------------
# CONFIGURATION PARAMETERS
# -----------------------------
GOAL_COLOR = [(255, 255, 0)]   # Yellow cylinder
COLOR_TOLERANCE = 0.08
AREA_THRESHOLD = 500           # Minimum landmark size
LIDAR_FRONT_INDEX = 180
OBSTACLE_THRESHOLD = 300       # mm, stop distance in front
MOTOR_SPEED = 50               # Motor speed
STEP_DURATION = 0.2            # seconds per incremental move
GOAL_RADIUS = 0.25             # meters, success distance

# -----------------------------
# HELPER FUNCTIONS
# -----------------------------
def obstacle_detected(robot):
    """Check if there is an obstacle in front using LIDAR."""
    scan = robot.get_lidar_range_image()
    front_distance = scan[LIDAR_FRONT_INDEX]
    return front_distance < OBSTACLE_THRESHOLD

def goal_visible(robot):
    """Check if the yellow goal is visible in camera frame."""
    landmarks = robot.find_landmarks(area_threshold=AREA_THRESHOLD)
    return len(landmarks) > 0

def move_toward_goal(robot):
    """Drive forward by a small step."""
    robot.set_left_speed(-MOTOR_SPEED)   # left motor reversed
    robot.set_right_speed(MOTOR_SPEED)   # right motor forward
    time.sleep(STEP_DURATION)
    robot.stop_motors()

def follow_wall(robot):
    """Simple wall-following behavior (right-hand)."""
    scan = robot.get_lidar_range_image()
    front = scan[180]
    right = scan[270]
    
    if front < OBSTACLE_THRESHOLD:
        # Turn left if blocked in front
        robot.set_left_speed(-MOTOR_SPEED)
        robot.set_right_speed(-MOTOR_SPEED//2)
    elif right > OBSTACLE_THRESHOLD + 50:
        # Steer right to stay close to wall
        robot.set_left_speed(-MOTOR_SPEED)
        robot.set_right_speed(MOTOR_SPEED//2)
    else:
        # Drive straight
        robot.set_left_speed(-MOTOR_SPEED)
        robot.set_right_speed(MOTOR_SPEED)
    
    time.sleep(STEP_DURATION)
    robot.stop_motors()

def reached_goal(robot):
    """Check if robot is within GOAL_RADIUS of goal."""
    landmarks = robot.find_landmarks(area_threshold=AREA_THRESHOLD)
    return len(landmarks) > 0  # Can refine using distance estimate

# -----------------------------
# MAIN FUNCTION
# -----------------------------
def main():
    # Initialize robot with sensors enabled
    robot = HamBot(lidar_enabled=True, camera_enabled=True)
    
    # Set camera to detect goal color
    robot.set_landmark_colors(GOAL_COLOR, tolerance=COLOR_TOLERANCE)
    
    state = "MG"  # Motion to Goal

    try:
        while True:
            if reached_goal(robot):
                print("Goal Reached!")
                robot.stop_motors()
                break
            
            if state == "MG":
                if obstacle_detected(robot):
                    print("Obstacle detected! Switching to Wall-Following.")
                    state = "WF"
                else:
                    print("Motion to Goal...")
                    move_toward_goal(robot)
            
            elif state == "WF":
                if goal_visible(robot):
                    print("Line-of-sight to goal clear! Switching to Motion-to-Goal.")
                    state = "MG"
                else:
                    print("Wall Following...")
                    follow_wall(robot)
            
            time.sleep(0.05)  # small delay to prevent busy loop

    finally:
        # Stop everything safely
        robot.stop_motors()

# -----------------------------
# RUN MAIN
# -----------------------------
if __name__ == "__main__":
    main()
