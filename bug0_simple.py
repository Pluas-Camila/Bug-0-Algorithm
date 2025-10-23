# bug0_pink_simple.py
# -------------------------------------------------------------
# Bug 0 (Pink-first) for HamBot:
#  - GOAL_SEEK: face & drive toward a pink landmark
#  - WALL_FOLLOW: very simple left-follow when blocked
#  - Prints explicit [GOAL DETECTED]/[GOAL LOST] messages
# -------------------------------------------------------------

import time, math, traceback
from robot_systems.robot import HamBot
from robot_systems.camera import Camera

# ---------------------------- Tunables ----------------------------

DT = 0.03  # control loop period

# LIDAR thresholds (meters)
STOP_HARD     = 0.15
TURN_TRIGGER  = 0.45
FRONT_CLEAR   = 0.70
TARGET_SIDE   = 0.30
SIDE_TOO_FAR  = 0.45
SIDE_TOO_NEAR = 0.22

# Motor speeds (RPM)
MAX_RPM       = 70.0
CRUISE_GOAL   = 26.0
CRUISE_WALL   = 22.0
ARC_FAST      = 22.0
ARC_SLOW      =  8.0
SEARCH_RPM    = 16.0
BACK_RPM      = 20.0
BACK_T        = 0.25

# Camera / pink detection
IMG_W, IMG_H  = 640, 480
PINKS = [
    (255, 105, 180),
    (255,  20, 147),
    (255,   0, 180),
    (255,   0, 255),
    (240,  50, 160),
]
BASE_TOL      = 0.10
MAX_TOL       = 0.20
MIN_AREA      = 200
NO_GOAL_WIDEN = 30
NO_GOAL_SHRINK= 60

HEADING_K     = 0.6
GOAL_LOCK_ERR = 0.18
GOAL_NEAR_H   = 220  # stop when bbox height suggests we’re close enough

# Indices for sectors
FRONT_S = slice(160, 200)
LEFT_S  = slice( 90, 116)
RIGHT_S = slice(244, 280)
BACK_S  = slice(355, 360)

# States
GOAL_SEEK   = 0
WALL_FOLLOW = 1

# ---------------------------- Helpers ----------------------------

def clamp(x, lo, hi): 
    return max(lo, min(hi, x))

def _median(xs):
    ys = sorted(xs)
    return ys[len(ys)//2] if ys else float("inf")

def safe_min(vals, fallback=float("inf")):
    good = []
    for v in vals:
        try:
            fv = float(v)
        except:
            continue
        if fv > 0.0 and math.isfinite(fv):
            good.append(fv)
    return min(good) if good else fallback

def normalize_scan_to_m(scan):
    """Auto-detect units (mm/cm/m) per frame and return meters."""
    if not scan or isinstance(scan, int):
        return [float("inf")] * 360
    samples = [v for v in scan if v > 0.0]
    if not samples:
        return [float("inf")] * len(scan)
    med = _median(samples[:min(200, len(samples))])
    if med > 50.0:   s = 0.001
    elif med > 5.0:  s = 0.01
    else:            s = 1.0
    out = []
    for v in scan:
        if v <= 0.0:
            out.append(float("inf"))
        else:
            m = v * s
            out.append(m if m < 20.0 else float("inf"))
    return out

def parse_boxes(lms):
    """Normalize find_landmarks() output into a list of (x,y,w,h)."""
    boxes = []
    if not lms:
        return boxes
    for lm in lms:
        if isinstance(lm, (list, tuple)) and len(lm) >= 4:
            x,y,w,h = lm[:4]
            boxes.append((int(x), int(y), int(w), int(h)))
        elif isinstance(lm, dict):
            if "bbox" in lm:
                x,y,w,h = lm["bbox"][:4]
                boxes.append((int(x), int(y), int(w), int(h)))
            elif "box" in lm:
                x,y,w,h = lm["box"][:4]
                boxes.append((int(x), int(y), int(w), int(h)))
            elif all(k in lm for k in ("x","y","w","h")):
                boxes.append((int(lm["x"]), int(lm["y"]), int(lm["w"]), int(lm["h"])))
    return boxes

def heading_error_from_box(box, img_w=IMG_W):
    """Normalized horizontal error in [-1, +1]. +err => target to RIGHT."""
    x, y, w, h = box
    cx = img_w * 0.5
    blob_cx = x + 0.5 * w
    pix_err = blob_cx - cx
    return clamp(pix_err / max(cx, 1.0), -1.0, 1.0)

def stop(bot):
    bot.set_left_motor_speed(0)
    bot.set_right_motor_speed(0)

def backup(bot, away_from_left=True):
    """Short reverse, then small pivot away from obstacle."""
    bot.set_left_motor_speed(-BACK_RPM)
    bot.set_right_motor_speed(-BACK_RPM)
    time.sleep(BACK_T)
    if away_from_left:
        bot.set_left_motor_speed(+SEARCH_RPM)
        bot.set_right_motor_speed(-SEARCH_RPM)
    else:
        bot.set_left_motor_speed(-SEARCH_RPM)
        bot.set_right_motor_speed(+SEARCH_RPM)
    time.sleep(0.20)
    stop(bot)

# ---------------------------- Main ----------------------------

def main():
    bot = HamBot(lidar_enabled=True, camera_enabled=True)
    cam = None
    tol = BASE_TOL
    no_goal_frames = 0
    yes_goal_frames = 0
    have_pink = False  # <-- NEW: detection state flag

    try:
        cam = Camera(fps=5, resolution=(IMG_W, IMG_H))
        cam.set_landmark_colors(PINKS, tolerance=tol)
        time.sleep(0.3)
    except Exception as e:
        print("[WARN] Camera init failed; running without vision.")
        print(e)
        print(traceback.format_exc())
        cam = None

    state = GOAL_SEEK
    print("[Bug0] start. Pink-first; pivots toward pink when tight; simple left wall-follow.")

    try:
        while True:
            # --- LIDAR ---
            scan = bot.get_range_image()
            if scan == -1 or not scan:
                print("[WARN] LIDAR not ready.")
                stop(bot)
                time.sleep(0.1)
                continue
            scan_m  = normalize_scan_to_m(scan)
            front_d = safe_min(scan_m[FRONT_S])
            left_d  = safe_min(scan_m[LEFT_S])

            # --- Backup if extremely close ---
            if front_d <= STOP_HARD:
                print(f"[BACKUP] front={front_d:.2f} m")
                backup(bot, away_from_left=True)
                state = WALL_FOLLOW
                continue

            # --- Camera (pink) ---
            pink_err = None
            pink_h   = None
            boxes    = None
            if cam is not None:
                try:
                    lms = cam.find_landmarks(area_threshold=MIN_AREA)
                    boxes = parse_boxes(lms)
                except Exception:
                    boxes = []

                if boxes:
                    best = max(boxes, key=lambda b: b[2]*b[3])
                    pink_err = heading_error_from_box(best, IMG_W)
                    pink_h   = best[3]

                    # NEW: one-time print when detection toggles ON
                    if not have_pink:
                        have_pink = True
                        x,y,w,h = best
                        print(f"[GOAL DETECTED] bbox={best}  err={pink_err:+.2f}  tol={tol:.2f}")

                    yes_goal_frames += 1
                    no_goal_frames = 0
                else:
                    # NEW: one-time print when detection toggles OFF
                    if have_pink:
                        have_pink = False
                        print("[GOAL LOST]")
                    no_goal_frames += 1
                    yes_goal_frames = 0

                # Adaptive tolerance
                if no_goal_frames >= NO_GOAL_WIDEN and tol < MAX_TOL:
                    tol = min(MAX_TOL, tol + 0.02)
                    cam.set_landmark_colors(PINKS, tolerance=tol)
                    no_goal_frames = 0
                    print(f"[CAM] widen tol -> {tol:.2f}")
                elif yes_goal_frames >= NO_GOAL_SHRINK and tol > BASE_TOL:
                    tol = max(BASE_TOL, tol - 0.02)
                    cam.set_landmark_colors(PINKS, tolerance=tol)
                    yes_goal_frames = 0
                    print(f"[CAM] shrink tol -> {tol:.2f}")

            # Per-loop camera heartbeat
            if cam is not None:
                if boxes:
                    print(f"[CAM] pink_boxes={len(boxes)}  err={None if pink_err is None else round(pink_err,2)}  h={pink_h}")
                else:
                    print("[CAM] no pink")

            # --- State Machine ---
            if state == GOAL_SEEK:
                if pink_err is not None:
                    # Close enough? stop
                    if pink_h is not None and pink_h >= GOAL_NEAR_H and front_d > STOP_HARD + 0.05:
                        print(f"[GOAL] reached (bbox_h={pink_h}) — stop")
                        stop(bot)
                        break

                    if front_d < TURN_TRIGGER:
                        # Pivot toward pink
                        turn = clamp(HEADING_K * pink_err * CRUISE_GOAL, -CRUISE_GOAL, CRUISE_GOAL)
                        left  = clamp(-turn, -MAX_RPM, MAX_RPM)
                        right = clamp(+turn, -MAX_RPM, MAX_RPM)
                        bot.set_left_motor_speed(left)
                        bot.set_right_motor_speed(right)
                        print(f"[GOAL_PIVOT] tight front={front_d:.2f} err={pink_err:+.2f}  L={left:.1f} R={right:.1f}")
                    else:
                        # Drive toward pink
                        steer = HEADING_K * pink_err * CRUISE_GOAL
                        steer = clamp(steer, -0.6*CRUISE_GOAL, 0.6*CRUISE_GOAL)
                        left  = clamp(CRUISE_GOAL + steer, -MAX_RPM, MAX_RPM)
                        right = clamp(CRUISE_GOAL - steer, -MAX_RPM, MAX_RPM)
                        bot.set_left_motor_speed(left)
                        bot.set_right_motor_speed(right)
                        print(f"[GOAL_SEEK] err={pink_err:+.2f} h={pink_h}  L={left:.1f} R={right:.1f}  front={front_d:.2f}")
                else:
                    # No goal in sight
                    if front_d > FRONT_CLEAR + 0.2:
                        bot.set_left_motor_speed(CRUISE_GOAL*0.55)
                        bot.set_right_motor_speed(CRUISE_GOAL*0.55)
                        print(f"[SEARCH-FWD] no pink; drift forward  front={front_d:.2f}")
                    else:
                        bot.set_left_motor_speed(-SEARCH_RPM)
                        bot.set_right_motor_speed(+SEARCH_RPM)
                        print("[SEARCH-SPIN] no pink; spinning to scan")

                    if front_d < TURN_TRIGGER:
                        print("[STATE] GOAL_SEEK → WALL_FOLLOW (blocked & no pink)")
                        state = WALL_FOLLOW
                        stop(bot)

            else:  # WALL_FOLLOW
                if (pink_err is not None) and (front_d > STOP_HARD + 0.05):
                    print("[STATE] WALL_FOLLOW → GOAL_SEEK (pink visible)")
                    state = GOAL_SEEK
                    stop(bot)
                    continue

                if front_d < TURN_TRIGGER:
                    bot.set_left_motor_speed(+ARC_FAST)
                    bot.set_right_motor_speed(+ARC_SLOW)
                    act = "arc right (front tight)"
                elif left_d > SIDE_TOO_FAR:
                    bot.set_left_motor_speed(+ARC_SLOW)
                    bot.set_right_motor_speed(+ARC_FAST)
                    act = "arc left (left far)"
                elif left_d < SIDE_TOO_NEAR:
                    bot.set_left_motor_speed(+ARC_FAST)
                    bot.set_right_motor_speed(+ARC_SLOW)
                    act = "arc right (left near)"
                else:
                    bot.set_left_motor_speed(CRUISE_WALL)
                    bot.set_right_motor_speed(CRUISE_WALL)
                    act = "cruise"

                print(f"[WALL_FOLLOW] {act} | front={front_d:.2f} left={left_d:.2f}")

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\n[Bug0] stopped by user.")
    finally:
        stop(bot)
        try:
            if cam is not None:
                cam.stop_camera()
        except Exception:
            pass

if __name__ == "__main__":
    main()
