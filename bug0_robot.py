import time
import math
from robot_systems.camera import Camera
from robot_systems.lidar import Lidar
from buildhat import Motor
#from robot_systems.robot import HamBot

# -----------------------------
# CONFIGURATION PARAMETERS
# -----------------------------
GOAL_COLOR = [(255, 255, 0)]   # Yellow cylinder
COLOR_TOLERANCE = 0.08
AREA_THRESHOLD = 500           # Minimum landmark size
LIDAR_FRONT_INDEX = 180
OBSTACLE_THRESHOLD = 300       # mm, stop distance in front
MOTOR_SPEED = 50               # Motor speed
STEP_DISTANCE = 0.1            # meters per move step
GOAL_RADIUS = 0.25             # meters, success distance

# -----------------------------
# INITIALIZE SENSORS & MOTORS
# -----------------------------
cam = Camera(fps=5)
cam.set_landmark_colors(GOAL_COLOR, tolerance=COLOR_TOLERANCE)

lidar = Lidar()
time.sleep(0.5)  # wait for first scan

left_motor = Motor('A')
right_motor = Motor('B')

# -----------------------------
# HELPER FUNCTIONS
# -----------------------------
def obstacle_detected():
    """Check if there is an obstacle in front."""
    scan = lidar.get_current_scan()
    front_distance = scan[LIDAR_FRONT_INDEX]
    return front_distance < OBSTACLE_THRESHOLD

def goal_visible():
    """Check if the yellow goal is visible in camera frame."""
    landmarks = cam.find_landmarks(area_threshold=AREA_THRESHOLD)
    return len(landmarks) > 0

def move_toward_goal():
    """Drive forward by a small step."""
    left_motor.start(-MOTOR_SPEED)   # left motor reversed
    right_motor.start(MOTOR_SPEED)   # right motor forward
    time.sleep(0.2)                  # step duration
    left_motor.stop()
    right_motor.stop()

def follow_wall():
    """Basic wall-following behavior (right-hand)."""
    # Get LIDAR distances
    scan = lidar.get_current_scan()
    front = scan[180]
    right = scan[270]
    
    # Simple proportional wall-following
    if front < OBSTACLE_THRESHOLD:
        # Turn left if blocked in front
        left_motor.start(-MOTOR_SPEED)
        right_motor.start(-MOTOR_SPEED//2)
    elif right > OBSTACLE_THRESHOLD + 50:
        # Steer right to stay close to wall
        left_motor.start(-MOTOR_SPEED)
        right_motor.start(MOTOR_SPEED//2)
    else:
        # Drive straight
        left_motor.start(-MOTOR_SPEED)
        right_motor.start(MOTOR_SPEED)
    
    time.sleep(0.2)
    left_motor.stop()
    right_motor.stop()

def reached_goal():
    """Check if robot is within GOAL_RADIUS of goal.
       Simplified: if landmark detected, assume reached."""
    landmarks = cam.find_landmarks(area_threshold=AREA_THRESHOLD)
    return len(landmarks) > 0  # Can be refined with distance estimate

# -----------------------------
# BUG 0 STATE MACHINE
# -----------------------------
state = "MG"  # Motion to Goal

try:
    while True:
        if reached_goal():
            print("Goal Reached!")
            left_motor.stop()
            right_motor.stop()
            break
        
        if state == "MG":
            if obstacle_detected():
                print("Obstacle detected! Switching to Wall-Following.")
                state = "WF"
            else:
                print("Motion to Goal...")
                move_toward_goal()
        
        elif state == "WF":
            if goal_visible():
                print("Line-of-sight to goal clear! Switching to Motion-to-Goal.")
                state = "MG"
            else:
                print("Wall Following...")
                follow_wall()
        
        time.sleep(0.05)  # small delay to prevent busy loop

finally:
    # STOP everything safely
    left_motor.stop()
    right_motor.stop()
    cam.stop_camera()
    lidar.stop_lidar()
