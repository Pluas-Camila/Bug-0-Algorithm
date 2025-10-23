# test_camera_white.py
# -------------------------------------------------------------
# Robust WHITE-goal camera test for HamBot across Camera API variants.
# - Adapts to constructor (with/without resolution kw)
# - Uses set_landmark_colors if available, else tries find_landmarks variants
# - Prints DETECTED/LOST, largest bbox, heading error, and FPS
# -------------------------------------------------------------
import time
import sys
import argparse

try:
    from robot_systems.camera import Camera
except Exception as e:
    print(f"[FATAL] Could not import robot_systems.camera.Camera: {e}")
    sys.exit(1)

def parse_res(s: str):
    try:
        w, h = s.lower().split("x")
        return (int(w), int(h))
    except Exception:
        raise argparse.ArgumentTypeError("Resolution must look like 640x480")

def heading_error(center_x: int, img_width: int) -> float:
    if img_width <= 0:
        return 0.0
    half = img_width * 0.5
    return max(-1.0, min(1.0, (center_x - half) / max(half, 1.0)))

def try_construct_camera(fps: int, resolution):
    # Most-capable signature first, then fall back
    try:
        cam = Camera(fps=fps, resolution=resolution)
        print(f"[API] Camera(fps={fps}, resolution={resolution}) OK")
        return cam, resolution
    except TypeError:
        try:
            cam = Camera(fps=fps)
            print(f"[API] Camera(fps={fps}) OK (no 'resolution' kw)")
            return cam, resolution
        except Exception as e:
            print(f"[FATAL] Could not construct Camera: {e}")
            raise
    except Exception as e:
        print(f"[FATAL] Camera init failed: {e}")
        raise

class ColorDetector:
    """Adapts to camera API differences for color landmark detection."""
    def __init__(self, cam, rgb, tol, area_threshold):
        self.cam = cam
        self.rgb = tuple(int(x) for x in rgb)
        self.tol = float(tol)
        self.area = int(area_threshold)
        self.mode = None
        self._choose_mode()

    def _choose_mode(self):
        # Prefer set_landmark_colors if present
        if hasattr(self.cam, "set_landmark_colors"):
            try:
                self.cam.set_landmark_colors([self.rgb], tolerance=self.tol)
                self.mode = "configured_colors"
                print(f"[API] Using set_landmark_colors([...], tolerance={self.tol}) + find_landmarks(area_threshold=...)")
                return
            except Exception as e:
                print(f"[WARN] set_landmark_colors failed ({e}); trying direct find_landmarks variants.")

        fl = getattr(self.cam, "find_landmarks", None)
        if not callable(fl):
            print("[FATAL] Camera has no find_landmarks()")
            self.mode = None
            return

        # Try various signatures:
        try:
            _ = fl(target_rgb=self.rgb, tolerance=self.tol, area_threshold=self.area)
            self.mode = "kw_all"
            print("[API] Using find_landmarks(target_rgb=..., tolerance=..., area_threshold=...)")
            return
        except TypeError:
            pass
        except Exception:
            pass

        try:
            _ = fl(target_rgb=self.rgb, tolerance=self.tol)
            self.mode = "kw_rgb_tol"
            print("[API] Using find_landmarks(target_rgb=..., tolerance=...)")
            return
        except TypeError:
            pass
        except Exception:
            pass

        try:
            _ = fl(self.rgb)
            self.mode = "pos_rgb"
            print("[API] Using find_landmarks(<rgb_tuple>)")
            return
        except TypeError:
            pass
        except Exception:
            pass

        try:
            _ = fl()
            self.mode = "plain"
            print("[API] Using find_landmarks() (camera must have default color configured)")
            return
        except Exception:
            pass

        print("[FATAL] No compatible find_landmarks signature detected.")
        self.mode = None

    def detect(self):
        """Return list of landmarks (possibly empty)."""
        if self.mode is None:
            return []
        fl = self.cam.find_landmarks
        try:
            if self.mode == "configured_colors":
                try:
                    boxes = fl(area_threshold=self.area)
                except TypeError:
                    boxes = fl()
                    boxes = [lm for lm in boxes if getattr(lm, "width", 0) * getattr(lm, "height", 0) >= self.area]
                return boxes
            elif self.mode == "kw_all":
                return fl(target_rgb=self.rgb, tolerance=self.tol, area_threshold=self.area)
            elif self.mode == "kw_rgb_tol":
                boxes = fl(target_rgb=self.rgb, tolerance=self.tol)
                return [lm for lm in boxes if getattr(lm, "width", 0) * getattr(lm, "height", 0) >= self.area]
            elif self.mode == "pos_rgb":
                boxes = fl(self.rgb)
                return [lm for lm in boxes if getattr(lm, "width", 0) * getattr(lm, "height", 0) >= self.area]
            elif self.mode == "plain":
                boxes = fl()
                return [lm for lm in boxes if getattr(lm, "width", 0) * getattr(lm, "height", 0) >= self.area]
        except Exception as e:
            print(f"[WARN] find_landmarks call failed this tick: {e}")
            return []

def try_get_frame(cam):
    """Try common frame getters; return (frame, (W,H)) or (None, (None,None))."""
    for name in ("get_image", "get_frame"):
        fn = getattr(cam, name, None)
        if callable(fn):
            try:
                img = fn()
                if img is not None and hasattr(img, "shape") and len(img.shape) >= 2:
                    h, w = img.shape[0], img.shape[1]
                    return img, (w, h)
            except Exception:
                pass
    return None, (None, None)

def main():
    ap = argparse.ArgumentParser(description="HamBot WHITE-goal camera test (adaptive to API)")
    ap.add_argument("--fps", type=int, default=5, help="Camera FPS (default 5)")
    ap.add_argument("--res", type=parse_res, default=(640, 480), help="Resolution WxH (default 640x480)")
    ap.add_argument("--tol", type=float, default=0.15, help="Color tolerance (0..1). White often needs higher tol (default 0.15)")
    ap.add_argument("--area", type=int, default=600, help="Min bbox area (px^2) (default 600)")
    ap.add_argument("--rgb", type=str, default="255,255,255", help="Target RGB as R,G,B (default white 255,255,255)")
    args = ap.parse_args()

    try:
        TARGET_RGB = tuple(int(x) for x in args.rgb.split(","))
        assert len(TARGET_RGB) == 3 and all(0 <= c <= 255 for c in TARGET_RGB)
    except Exception:
        print("[ERR] --rgb must be 'R,G,B' with 0..255 values")
        sys.exit(1)

    cam = None
    try:
        print(f"[INFO] Starting camera {args.res[0]}x{args.res[1]} @ {args.fps} fps")
        cam, res_used = try_construct_camera(args.fps, args.res)
        time.sleep(0.6)  # warm-up

        detector = ColorDetector(cam, TARGET_RGB, args.tol, args.area)

        detected = False
        last_fps_t = time.time()
        frames = 0
        img_w = res_used[0]

        print(f"[INFO] Running WHITE-only detection. Ctrl+C to stop.")
        while True:
            frame, (w, h) = try_get_frame(cam)
            if w is not None:
                img_w = w

            lms = detector.detect()
            if lms:
                best = max(lms, key=lambda lm: getattr(lm, "width", 0) * getattr(lm, "height", 0))
                err = heading_error(getattr(best, "center_x", img_w//2), img_w or 1)
                if not detected:
                    print("[DETECTED] WHITE landmark appeared.")
                    detected = True
                print(f"[WHITE] center=({getattr(best,'center_x','?')},{getattr(best,'center_y','?')}) "
                      f"size={getattr(best,'width','?')}x{getattr(best,'height','?')} "
                      f"area={getattr(best,'width',0)*getattr(best,'height',0)} "
                      f"imgW={img_w} err={err:+.2f}")
            else:
                if detected:
                    print("[LOST] WHITE landmark disappeared.")
                    detected = False
                print("[NO WHITE] —")

            frames += 1
            now = time.time()
            if now - last_fps_t >= 2.0:
                fps = frames / max(1e-6, (now - last_fps_t))
                print(f"[FPS] ~{fps:.1f}")
                frames = 0
                last_fps_t = now

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    except Exception as e:
        print(f"[ERROR] {e}")
        print("[HINT] For white in mixed lighting, try: --tol 0.20 --area 900")
    finally:
        if cam is not None:
            try:
                cam.stop_camera()
            except Exception:
                pass
        print("[INFO] Camera stopped.")

if __name__ == "__main__":
    main()
