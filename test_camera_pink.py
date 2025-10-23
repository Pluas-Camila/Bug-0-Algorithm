# test_camera_pink.py
# -------------------------------------------------------------
# Pink-only camera smoke test using HamBot's documented Camera API.
# Uses ONLY: Camera(...), get_image(), set_landmark_colors(), find_landmarks(), stop_camera()
# Prints detection state, largest bbox, heading error, and FPS.
# -------------------------------------------------------------
import time
import sys
import traceback
import numpy as np

from robot_systems.camera import Camera

# --- Config ---
FPS            = 8
RESOLUTION     = (640, 480)     # (W, H)
AREA_THRESHOLD = 500            # px^2 (filter tiny blobs)
TOLERANCE      = 0.10           # ±10% per channel
PRINT_FPS_EVERY = 2.0           # seconds

# A few pink/magenta shades
PINKS = [
    (255, 105, 180),  # hot pink
    (255,  20, 147),  # deep pink
    (255,   0, 255),  # magenta
    (240,  50, 160),  # vivid pink
]

def heading_error_from_center(cx, img_w):
    """
    Normalized horizontal error in [-1, +1] relative to image center.
    +err => target center is to the RIGHT of the image center.
    """
    cx_img  = img_w * 0.5
    pix_err = cx - cx_img
    return max(-1.0, min(1.0, pix_err / max(cx_img, 1.0)))

def require_methods(cam):
    """
    Ensure the installed Camera has the documented API.
    If not, print a clear message and exit gracefully.
    """
    missing = []
    for name in ("get_image", "set_landmark_colors", "find_landmarks", "stop_camera"):
        if not hasattr(cam, name):
            missing.append(name)
    if missing:
        print("[ERROR] Your installed robot_systems.camera.Camera is missing required methods:")
        for m in missing:
            print(f"        - {m}()")
        print("\nFix: update HamBot to a version that matches the documentation you pasted.")
        print("     Example: pip install -U --force-reinstall hambot-robot-systems (or your local package)")
        # Try to stop camera if it started internal threads
        if hasattr(cam, "stop_camera"):
            try: cam.stop_camera()
            except Exception: pass
        sys.exit(1)

def main():
    cam = None
    try:
        print(f"[INFO] Starting Camera {RESOLUTION[0]}x{RESOLUTION[1]} @ {FPS} fps ...")
        cam = Camera(fps=FPS, resolution=RESOLUTION)  # camera_index=0 default
        time.sleep(0.5)  # let background thread fill a frame

        require_methods(cam)

        # Optional sanity: ensure we can pull one frame (may be None on the first few tries)
        t_wait = time.time()
        img = None
        while (time.time() - t_wait) < 2.0:
            img = cam.get_image()
            if img is not None:
                break
            time.sleep(0.05)

        if img is None:
            print("[WARN] get_image() returned None after warm-up. Continuing anyway.")

        # Configure pink(s)
        cam.set_landmark_colors(PINKS, tolerance=TOLERANCE)
        print("[INFO] Looking ONLY for PINK. Press Ctrl+C to stop.")

        had_pink = False
        tfps = time.time()
        frames = 0

        while True:
            frame = cam.get_image()
            if frame is None:
                print("[NO FRAME] (camera warming up?)")
                time.sleep(0.05)
                continue

            # frame should be (H, W, 3) RGB per docs
            H, W = frame.shape[0], frame.shape[1]

            # Find pink landmarks through the library
            landmarks = cam.find_landmarks(area_threshold=AREA_THRESHOLD)

            frames += 1
            if landmarks:
                # Choose largest bbox by area
                best = max(landmarks, key=lambda lm: (lm.width * lm.height))
                err  = heading_error_from_center(best.center_x, W)
                if not had_pink:
                    print("[DETECTED] Pink landmark appears.")
                    had_pink = True
                print(f"[PINK] center=({best.center_x},{best.center_y}) size={best.width}x{best.height} "
                      f"area={best.width*best.height}  err={err:+.2f}")
            else:
                if had_pink:
                    print("[LOST] Pink landmark disappeared.")
                    had_pink = False
                print("[NO PINK] —")

            # FPS print
            if (time.time() - tfps) >= PRINT_FPS_EVERY:
                fps = frames / max(1e-6, (time.time() - tfps))
                print(f"[FPS] ~{fps:.1f}")
                tfps = time.time()
                frames = 0

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    except Exception:
        print("[ERROR] Camera test failed:")
        traceback.print_exc(file=sys.stdout)
    finally:
        if cam is not None and hasattr(cam, "stop_camera"):
            try:
                cam.stop_camera()
            except Exception:
                pass
        print("[INFO] Camera stopped.")

if __name__ == "__main__":
    main()
