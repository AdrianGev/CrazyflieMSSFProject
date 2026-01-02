import cv2
import numpy as np
import time
import os
import world


class VisionObstacleUpdater:
    def __init__(
        self,
        cam_index=0,
        warp_w=640,
        warp_h=480,
        hit_frac=0.18,
        hit_frames=3,
        hold_ms=600,
        history=300,
        var_threshold=18,
        freeze_bg_when_blocked=True,
    ):
        if not os.path.exists("H.npy"):
            raise RuntimeError("Missing H.npy (run vision_calibrate.py)")

        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera")

        self.warp_w = warp_w
        self.warp_h = warp_h
        self.H = np.load("H.npy")

        self.grid_w = len(world.COLS)
        self.grid_h = len(world.ROWS)

        self.hit_frac = float(hit_frac)
        self.hit_frames = int(hit_frames)
        self.hold_ms = float(hold_ms)
        self.freeze_bg_when_blocked = bool(freeze_bg_when_blocked)

        self.bg = cv2.createBackgroundSubtractorMOG2(
            history=int(history),
            varThreshold=float(var_threshold),
            detectShadows=False
        )

        self.hit_streak = np.zeros((self.grid_h, self.grid_w), dtype=np.uint8)
        self.hold_until = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        self.prev_blocked = np.zeros((self.grid_h, self.grid_w), dtype=bool)
        self._blocked_any = False

    def _warp(self, frame):
        return cv2.warpPerspective(frame, self.H, (self.warp_w, self.warp_h))

    def _fgmask(self, warped):
        lr = 0 if (self.freeze_bg_when_blocked and self._blocked_any) else -1
        m = self.bg.apply(warped, learningRate=lr)
        m = cv2.medianBlur(m, 5)
        _, m = cv2.threshold(m, 200, 255, cv2.THRESH_BINARY)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k, iterations=1)
        m = cv2.morphologyEx(m, cv2.MORPH_DILATE, k, iterations=1)
        return m

    def _mask_to_blocked(self, fg):
        t = time.time() * 1000.0
        H, W = fg.shape
        cw, ch = W / self.grid_w, H / self.grid_h

        blocked = np.zeros((self.grid_h, self.grid_w), dtype=bool)

        for gy in range(self.grid_h):
            y0, y1 = int(gy * ch), int((gy + 1) * ch)
            for gx in range(self.grid_w):
                x0, x1 = int(gx * cw), int((gx + 1) * cw)
                cell = fg[y0:y1, x0:x1]
                frac = (cell > 0).mean()

                if frac >= self.hit_frac:
                    self.hit_streak[gy, gx] = min(255, self.hit_streak[gy, gx] + 1)
                else:
                    self.hit_streak[gy, gx] = 0

                if self.hit_streak[gy, gx] >= self.hit_frames:
                    self.hold_until[gy, gx] = t + self.hold_ms

                if self.hold_until[gy, gx] > t:
                    blocked[gy, gx] = True

        self._blocked_any = bool(blocked.any())
        return blocked

    def _neighbors4_labels(self, label):
        """Drone cell + its 4-neighbors, all as labels, clipped to grid bounds."""
        x, y = world.label_to_xy(label)
        out = {label}
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(world.COLS) and 0 <= ny < len(world.ROWS):
                out.add(world.xy_to_label(nx, ny))
        return out

    def step(self, start_label, goal_label, drone_label=None, debug=False):
        """
        Updates world.grid based on camera motion.
        Returns (added_labels, removed_labels).
        """
        ok, frame = self.cap.read()
        if not ok:
            return set(), set()

        warped = self._warp(frame)
        fg = self._fgmask(warped)
        blocked = self._mask_to_blocked(fg)

        if debug:
            cv2.imshow("vision_warped", warped)
            cv2.imshow("vision_fg", fg)
            cv2.waitKey(1)

        # --- ignore set: start/goal + drone cell + drone 4-neighbors ---
        ignore = {start_label, goal_label}
        if drone_label is not None:
            ignore |= self._neighbors4_labels(drone_label)

        added, removed = set(), set()

        changed = blocked ^ self.prev_blocked
        if changed.any():
            ys, xs = np.where(changed)
            for y, x in zip(ys, xs):
                lbl = world.xy_to_label(int(x), int(y))
                if lbl in ignore:
                    continue
                if blocked[y, x]:
                    world.set_obstacle(lbl)
                    added.add(lbl)
                else:
                    world.clear_obstacle(lbl)
                    removed.add(lbl)

        self.prev_blocked = blocked
        return added, removed

    def close(self):
        try:
            self.cap.release()
        except Exception:
            pass
        try:
            cv2.destroyWindow("vision_warped")
            cv2.destroyWindow("vision_fg")
        except Exception:
            pass
