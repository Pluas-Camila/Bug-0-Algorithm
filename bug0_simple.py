#!/usr/bin/env python3
"""
bug0_simple.py - Simplified Bug Zero

Drive straight toward goal when visible.
Wall follow when blocked.
"""

import time
import numpy as np
from robot_systems.robot import HamBot

# ============================================================================
# TUNABLE PARAMETERS
# ============================================================================

# Goal detection
GOAL_RGB = (245, 0, 124)  # Pink cylinder
GOAL_TOL = 0.25           # ±25% tolerance
GOAL_AREA = 400           # minimum pixel area

# LIDAR indices (0=back, 90=left, 180=front, 270=right)
FRONT_CENTER = 180
FRONT_SECTOR = 20
LEFT_CENTER = 90
LEFT_SECTOR = 15
RIGHT_CENTER = 270
RIGHT_SECTOR = 15

# Distances (mm)
STOP_HARD = 150
GOAL_REACHED = 250
OBSTACLE_CLOSE = 400
FRONT_CLEAR = 600
TARGET_SIDE = 350
WALL_GONE = 1500

# Speeds
FORWARD_SPEED = 30
WALL_SPEED = 25
SEARCH_SPEED = 20
BACKUP_SPEED = -25
ALIGN_SPEED = 15          # Slow speed for alignment
CENTER_TOLERANCE = 40     # pixels - goal must be within this range of center (320±40)

# Wall following
WALL_P = 0.05
WALL_D = 0.02

# Timing
LOOP_DT = 0.1
SEARCH_TIME = 18

# Camera
CAM_WIDTH = 640

# ============================================================================
# STATES
# ============================================================================
STATE_SEARCH = "SEARCH"
STATE_GOAL_SEEK = "GOAL_SEEK"
STATE_WALL_FOLLOW = "WALL_FOLLOW"

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def get_sector_min(scan, center_idx, sector_width):
    """Get minimum valid distance in a sector."""
    start = (center_idx - sector_width) % 360
    end = (center_idx + sector_width) % 360
    
    if start < end:
        sector = scan[start:end+1]
    else:
        sector = scan[start:] + scan[:end+1]
    
    valid = [d for d in sector if d > 0]
    return min(valid) if len(valid) > 0 else None

# ============================================================================
# MAIN CONTROLLER
# ============================================================================

def main():
    print("[INIT] Starting Bug-0 Controller")
    
    # Initialize robot
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    print("[INIT] HamBot initialized")
    
    # Use built-in camera
    cam = bot.camera
    cam.set_target_colors([GOAL_RGB], tolerance=GOAL_TOL)
    print(f"[INIT] Camera configured for PINK goal {GOAL_RGB}")
    
    time.sleep(0.8)
    
    # State variables
    state = STATE_SEARCH
    search_timer = 0
    last_side_dist = None
    goal_lost_count = 0
    GOAL_LOST_THRESHOLD = 10
    
    print(f"[INIT] Starting in {state} state")
    print("[INIT] Press Ctrl+C to stop\n")
    
    try:
        while True:
            loop_start = time.time()
            
            # ================================================================
            # PERCEPTION
            # ================================================================
            
            scan = bot.get_range_image()
            if scan == -1:
                time.sleep(LOOP_DT)
                continue
            
            front_d = get_sector_min(scan, FRONT_CENTER, FRONT_SECTOR)
            left_d = get_sector_min(scan, LEFT_CENTER, LEFT_SECTOR)
            right_d = get_sector_min(scan, RIGHT_CENTER, RIGHT_SECTOR)
            
            landmarks = cam.find_landmarks(min_area=GOAL_AREA)
            goal_visible = len(landmarks) > 0
            
            f_str = f"{front_d:.0f}mm" if front_d else "N/A"
            l_str = f"{left_d:.0f}mm" if left_d else "N/A"
            r_str = f"{right_d:.0f}mm" if right_d else "N/A"
            
            # ================================================================
            # GOAL REACHED CHECK
            # ================================================================
            
            if goal_visible:
                lm = landmarks[0]
                if lm.width > 200 and front_d and front_d < GOAL_REACHED:
                    print(f"\n[SUCCESS] Goal reached!")
                    print(f"[SUCCESS] Width={lm.width}px, Front={front_d:.0f}mm")
                    bot.stop_motors()
                    break
            
            # ================================================================
            # EMERGENCY BACKUP
            # ================================================================
            
            if front_d and front_d <= STOP_HARD:
                print(f"[BACKUP] Too close {front_d:.0f}mm")
                bot.set_left_motor_speed(BACKUP_SPEED)
                bot.set_right_motor_speed(BACKUP_SPEED)
                time.sleep(0.6)
                bot.stop_motors()
                time.sleep(0.2)
                continue
            
            # ================================================================
            # STATE MACHINE
            # ================================================================
            
            if state == STATE_SEARCH:
                print(f"[{state}] timer={search_timer:.1f}s F={f_str} L={l_str} R={r_str} goal={goal_visible}")
                
                if goal_visible:
                    print(f"[TRANSITION] Goal found -> GOAL_SEEK")
                    state = STATE_GOAL_SEEK
                    search_timer = 0
                    goal_lost_count = 0
                    bot.stop_motors()
                    time.sleep(0.2)
                    continue
                
                # Rotate to search
                bot.set_left_motor_speed(-SEARCH_SPEED)
                bot.set_right_motor_speed(SEARCH_SPEED)
                search_timer += LOOP_DT
                
                # After full rotation, no goal -> wall follow
                if search_timer >= SEARCH_TIME:
                    print(f"[TRANSITION] 360° complete -> WALL_FOLLOW")
                    state = STATE_WALL_FOLLOW
                    search_timer = 0
                    last_side_dist = None
                    bot.stop_motors()
                    time.sleep(0.2)
                    continue
            
            elif state == STATE_GOAL_SEEK:
                print(f"[{state}] F={f_str} L={l_str} R={r_str} goal={goal_visible}")
                
                # Check if goal lost
                if not goal_visible:
                    goal_lost_count += 1
                    print(f"[GOAL_SEEK] Goal lost {goal_lost_count}/{GOAL_LOST_THRESHOLD}")
                    
                    if goal_lost_count >= GOAL_LOST_THRESHOLD:
                        print(f"[TRANSITION] Goal lost -> SEARCH")
                        state = STATE_SEARCH
                        search_timer = 0
                        goal_lost_count = 0
                        bot.stop_motors()
                        time.sleep(0.2)
                        continue
                    else:
                        # Keep going forward
                        time.sleep(LOOP_DT)
                        continue
                else:
                    goal_lost_count = 0
                
                lm = landmarks[0]
                goal_center_x = lm.x
                image_center = CAM_WIDTH / 2
                pixel_error = goal_center_x - image_center
                
                print(f"[GOAL_SEEK] Goal at ({lm.x}, {lm.y}) size {lm.width}x{lm.height}")
                print(f"[GOAL_SEEK] Pixel error: {pixel_error:.0f}px (center={image_center:.0f})")
                
                # Check for obstacle
                if front_d and front_d < OBSTACLE_CLOSE:
                    print(f"[TRANSITION] Obstacle {front_d:.0f}mm -> WALL_FOLLOW")
                    state = STATE_WALL_FOLLOW
                    last_side_dist = None
                    goal_lost_count = 0
                    bot.stop_motors()
                    time.sleep(0.2)
                    continue
                
                # Check if goal is centered
                if abs(pixel_error) < CENTER_TOLERANCE:
                    # GOAL IS CENTERED - DRIVE STRAIGHT
                    speed = FORWARD_SPEED
                    if lm.width > 100:
                        speed = FORWARD_SPEED * 0.7
                        print(f"[GOAL_SEEK] Slowing down, close to goal")
                    
                    print(f"[GOAL_SEEK] ALIGNED! Driving STRAIGHT at {speed:.1f} RPM")
                    bot.set_left_motor_speed(speed)
                    bot.set_right_motor_speed(speed)
                else:
                    # GOAL NOT CENTERED - ALIGN FIRST
                    if pixel_error > 0:
                        # Goal is to the RIGHT - turn right (slow right wheel)
                        print(f"[GOAL_SEEK] Goal RIGHT (+{pixel_error:.0f}px) - turning right slowly")
                        bot.set_left_motor_speed(ALIGN_SPEED)
                        bot.set_right_motor_speed(ALIGN_SPEED * 0.5)
                    else:
                        # Goal is to the LEFT - turn left (slow left wheel)
                        print(f"[GOAL_SEEK] Goal LEFT ({pixel_error:.0f}px) - turning left slowly")
                        bot.set_left_motor_speed(ALIGN_SPEED * 0.5)
                        bot.set_right_motor_speed(ALIGN_SPEED)
            
            elif state == STATE_WALL_FOLLOW:
                print(f"[{state}] F={f_str} L={l_str} R={r_str} goal={goal_visible}")
                
                # Goal visible and path clear?
                if goal_visible and front_d and front_d > FRONT_CLEAR:
                    print(f"[TRANSITION] Goal visible & clear -> GOAL_SEEK")
                    state = STATE_GOAL_SEEK
                    goal_lost_count = 0
                    bot.stop_motors()
                    time.sleep(0.2)
                    continue
                
                # Choose wall (prefer right)
                if right_d and right_d < WALL_GONE:
                    side_d = right_d
                    follow_right = True
                elif left_d and left_d < WALL_GONE:
                    side_d = left_d
                    follow_right = False
                else:
                    # Wall lost
                    print(f"[WALL_FOLLOW] No wall, searching...")
                    bot.set_left_motor_speed(-SEARCH_SPEED)
                    bot.set_right_motor_speed(SEARCH_SPEED)
                    time.sleep(1.0)
                    bot.stop_motors()
                    last_side_dist = None
                    continue
                
                # PD control
                error = side_d - TARGET_SIDE
                d_error = 0
                if last_side_dist:
                    d_error = (side_d - last_side_dist) / LOOP_DT
                last_side_dist = side_d
                
                pd = WALL_P * error + WALL_D * d_error
                
                # Speed modulation
                if front_d and front_d < 500:
                    speed = WALL_SPEED * max(0.4, front_d / 500)
                else:
                    speed = WALL_SPEED
                
                # Apply differential
                if follow_right:
                    left_rpm = speed + pd
                    right_rpm = speed - pd
                else:
                    left_rpm = speed - pd
                    right_rpm = speed + pd
                
                left_rpm = np.clip(left_rpm, -75, 75)
                right_rpm = np.clip(right_rpm, -75, 75)
                
                bot.set_left_motor_speed(left_rpm)
                bot.set_right_motor_speed(right_rpm)
            
            # ================================================================
            # LOOP TIMING
            # ================================================================
            
            elapsed = time.time() - loop_start
            sleep_time = LOOP_DT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Keyboard interrupt")
    
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("[SHUTDOWN] Stopping motors and camera...")
        bot.stop_motors()
        cam.stop()
        print("[SHUTDOWN] Complete")


if __name__ == "__main__":
    main()