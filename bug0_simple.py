#!/usr/bin/env python3
"""
Bug-0 Algorithm - Maze 1 & 2

Maze 1: Direct goal approach
Maze 2: 2×360° search → right wall follow → goal approach
"""

import time
import numpy as np
from robot_systems.robot import HamBot

# ============================================================================
# CONFIGURATION
# ============================================================================

# Goal detection
GOAL_RGB = (245, 0, 124)
GOAL_TOL = 0.25
GOAL_AREA = 400

# LIDAR (0=back, 90=left, 180=front, 270=right)
FRONT_CENTER = 180
FRONT_SECTOR = 20
RIGHT_CENTER = 270
RIGHT_SECTOR = 15

# Distances (mm)
GOAL_REACHED = 250
OBSTACLE_DETECT = 400
TARGET_WALL = 350
WALL_LOST = 1500

# Motor speeds
SEARCH_SPEED = 20
FORWARD_SPEED = 35
WALL_SPEED = 30
STEER_GAIN = 0.2
LEFT_BIAS = 1.10
TEST_MODE = True
WALL_P = 0.05
WALL_D = 0.02

# Camera
CAM_WIDTH = 640
CAM_CENTER = 320

# Timing
LOOP_DT = 0.1
ROTATION_COUNT = 0
TWO_ROTATIONS = 720  # 2 × 360 degrees
GOAL_LOST_THRESHOLD = 10  # Need 10 frames (1 sec) without goal to confirm lost

# ============================================================================
# HELPERS
# ============================================================================

def get_front_distance(scan):
    start = (FRONT_CENTER - FRONT_SECTOR) % 360
    end = (FRONT_CENTER + FRONT_SECTOR) % 360
    if start < end:
        sector = scan[start:end+1]
    else:
        sector = scan[start:] + scan[:end+1]
    valid = [d for d in sector if d > 0]
    return min(valid) if valid else None

def get_right_distance(scan):
    start = (RIGHT_CENTER - RIGHT_SECTOR) % 360
    end = (RIGHT_CENTER + RIGHT_SECTOR) % 360
    if start < end:
        sector = scan[start:end+1]
    else:
        sector = scan[start:] + scan[:end+1]
    valid = [d for d in sector if d > 0]
    return min(valid) if valid else None

# ============================================================================
# MAIN
# ============================================================================

def main():
    print("\n" + "="*50)
    print("BUG-0 ALGORITHM")
    print("="*50)
    
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    cam = bot.camera
    cam.set_target_colors([GOAL_RGB], tolerance=GOAL_TOL)
    time.sleep(0.5)
    
    print(f"✓ Robot initialized")
    print(f"✓ Looking for PINK goal: {GOAL_RGB}")
    print(f"✓ Left motor bias: {LEFT_BIAS}x")
    print("-"*50 + "\n")
    
    goal_found = False
    wall_following = False
    rotation_degrees = 0
    last_heading = None
    last_wall_dist = None
    goal_seen_ever = False
    goal_lost_count = 0
    
    try:
        while True:
            scan = bot.get_range_image()
            if scan == -1:
                time.sleep(LOOP_DT)
                continue
            
            front_d = get_front_distance(scan)
            right_d = get_right_distance(scan)
            landmarks = cam.find_landmarks(min_area=GOAL_AREA)
            goal_visible = len(landmarks) > 0
            
            # ============================================================
            # SEARCH PHASE - Track 2 full rotations by heading
            # ============================================================
            if not goal_found and not wall_following:
                # Get current heading
                current_heading = bot.get_heading()
                
                # Calculate rotation
                if last_heading is not None:
                    delta = current_heading - last_heading
                    # Handle wraparound (359° → 0°)
                    if delta > 180:
                        delta -= 360
                    elif delta < -180:
                        delta += 360
                    rotation_degrees += abs(delta)
                
                last_heading = current_heading
                
                # Track if we ever see goal during search
                if goal_visible:
                    goal_seen_ever = True
                    goal_lost_count = 0
                else:
                    if goal_seen_ever:
                        goal_lost_count += 1
                
                # If we saw goal but lost it for threshold frames, it's really gone
                if goal_seen_ever and goal_lost_count >= GOAL_LOST_THRESHOLD:
                    print(f"[LOST] Goal disappeared")
                    goal_seen_ever = False
                    goal_lost_count = 0
                
                print(f"[SEARCH] {rotation_degrees:.0f}° / 720° F={front_d if front_d else 'N/A'}mm")
                bot.set_left_motor_speed(-SEARCH_SPEED)
                bot.set_right_motor_speed(SEARCH_SPEED)
                
                # After 2 full rotations with NO stable goal detection
                if rotation_degrees >= TWO_ROTATIONS and not goal_visible:
                    print(f"\n[2×360° COMPLETE] No goal after {rotation_degrees:.0f}° rotation")
                    print(f"[WALL_FOLLOW] Starting careful right wall following\n")
                    wall_following = True
                    rotation_degrees = 0
                    last_heading = None
                    last_wall_dist = None
                    goal_seen_ever = False
                    bot.stop_motors()
                    time.sleep(0.3)
                
                # If goal is visible at end of search, lock on
                if rotation_degrees >= TWO_ROTATIONS and goal_visible:
                    print(f"\n[FOUND] Goal confirmed after {rotation_degrees:.0f}° rotation!\n")
                    goal_found = True
                    rotation_degrees = 0
                    last_heading = None
                    bot.stop_motors()
                    time.sleep(0.3)
                
                time.sleep(LOOP_DT)
                continue
            
            # ============================================================
            # WALL FOLLOWING PHASE - Careful and accurate
            # ============================================================
            if wall_following:
                if goal_visible:
                    print(f"\n[FOUND] Goal detected during wall follow!\n")
                    wall_following = False
                    goal_found = True
                    bot.stop_motors()
                    time.sleep(0.3)
                    continue
                
                # Check right wall
                if not right_d or right_d > WALL_LOST:
                    print(f"[WALL] No right wall, searching...")
                    bot.set_left_motor_speed(-SEARCH_SPEED)
                    bot.set_right_motor_speed(SEARCH_SPEED)
                    time.sleep(LOOP_DT)
                    last_wall_dist = None
                    continue
                
                # FRONT OBSTACLE HANDLING - Turn left gradually
                if front_d and front_d < 300:  # Close to front wall
                    print(f"[WALL] Front obstacle {front_d:.0f}mm - turning LEFT")
                    # Turn left in place until front clears
                    bot.set_left_motor_speed(-15)
                    bot.set_right_motor_speed(15)
                    time.sleep(LOOP_DT)
                    last_wall_dist = None  # Reset PD control
                    continue
                
                # PD control for smooth wall following
                error = right_d - TARGET_WALL
                d_error = 0
                if last_wall_dist:
                    d_error = (right_d - last_wall_dist) / LOOP_DT
                last_wall_dist = right_d
                
                pd = WALL_P * error + WALL_D * d_error
                
                # Slow and careful speed
                if front_d and front_d < 600:
                    # Slow down as approaching front obstacle
                    speed = WALL_SPEED * max(0.4, front_d / 600)
                else:
                    speed = WALL_SPEED
                
                # Right wall follow: positive error = too far, turn right
                left_rpm = speed + pd
                right_rpm = speed - pd
                
                left_rpm = np.clip(left_rpm, -75, 75)
                right_rpm = np.clip(right_rpm, -75, 75)
                
                print(f"[WALL] R={right_d:.0f}mm F={front_d if front_d else 'N/A'}mm L:{left_rpm:.1f} R:{right_rpm:.1f}")
                
                bot.set_left_motor_speed(left_rpm)
                bot.set_right_motor_speed(right_rpm)
                time.sleep(LOOP_DT)
                continue
            
            # ============================================================
            # GOAL DETECTED
            # ============================================================
            if goal_visible and not goal_found:
                goal_found = True
                lm = landmarks[0]
                print(f"\n[FOUND] Goal at ({lm.x}, {lm.y}) size {lm.width}x{lm.height}\n")
                bot.stop_motors()
                time.sleep(0.3)
                continue
            
            # ============================================================
            # GOAL APPROACH PHASE
            # ============================================================
            if not goal_visible:
                print(f"[WARNING] Lost goal!")
                bot.stop_motors()
                time.sleep(0.5)
                goal_found = False
                continue
            
            lm = landmarks[0]
            
            # SUCCESS CHECK - LIDAR DISTANCE ONLY
            if front_d and front_d < GOAL_REACHED:
                print(f"\n[SUCCESS] GOAL REACHED!")
                print(f"[SUCCESS] Distance: {front_d:.0f}mm ({front_d/1000:.2f}m)\n")
                bot.stop_motors()
                break
            
            # Obstacle check
            if front_d and front_d < OBSTACLE_DETECT and lm.width < 150:
                print(f"[OBSTACLE] Front blocked at {front_d}mm")
                bot.stop_motors()
                break
            
            # Steering calculation
            pixel_error = lm.x - CAM_CENTER
            error_normalized = pixel_error / CAM_CENTER
            
            # Speed adjustment based on goal size
            if lm.width > 120:
                speed = FORWARD_SPEED * 0.6
            elif lm.width > 80:
                speed = FORWARD_SPEED * 0.8
            else:
                speed = FORWARD_SPEED
            
            # Motor control
            if TEST_MODE:
                left_rpm = speed * LEFT_BIAS
                right_rpm = speed
            else:
                steer_correction = STEER_GAIN * error_normalized
                left_rpm = speed * (1.0 + steer_correction) * LEFT_BIAS
                right_rpm = speed * (1.0 - steer_correction)
            
            left_rpm = np.clip(left_rpm, -75, 75)
            right_rpm = np.clip(right_rpm, -75, 75)
            
            direction = "R" if pixel_error > 20 else "L" if pixel_error < -20 else "C"
            print(f"[DRIVE] Goal:({lm.x:3.0f},{lm.y:3.0f}) W:{lm.width:3.0f} "
                  f"Err:{pixel_error:+4.0f}{direction} L:{left_rpm:.1f} R:{right_rpm:.1f}")
            
            bot.set_left_motor_speed(left_rpm)
            bot.set_right_motor_speed(right_rpm)
            
            time.sleep(LOOP_DT)
    
    except KeyboardInterrupt:
        print("\n[STOP] User interrupted")
    
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        bot.stop_motors()
        cam.stop()
        print("\n[SHUTDOWN] Complete\n")


if __name__ == "__main__":
    main()