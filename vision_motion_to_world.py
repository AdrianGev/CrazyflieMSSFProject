import cv2
import numpy as np
import time
import os
import world


# Grid-aligned warp dimensions: cellPx * grid size
CELL_PX = 80


def draw_grid_overlay(img, rows, cols, color=(0, 255, 0), thickness=1):
    """Draw grid lines on an image. Modifies img in-place and returns it."""
    h, w = img.shape[:2]
    cell_w = w / cols
    cell_h = h / rows

    # Vertical lines
    for k in range(cols + 1):
        x = int(k * cell_w)
        cv2.line(img, (x, 0), (x, h), color, thickness)

    # Horizontal lines
    for k in range(rows + 1):
        y = int(k * cell_h)
        cv2.line(img, (0, y), (w, y), color, thickness)

    # Label corners for orientation check: (0,0) top-right
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "(0,0)", (w - 50, 15), font, 0.4, color, 1)
    cv2.putText(img, f"({cols-1},{rows-1})", (5, h - 5), font, 0.4, color, 1)

    return img


def draw_blocked_overlay(img, blocked, rows, cols, color=(0, 0, 255), alpha=0.4):
    """Draw semi-transparent rectangles on blocked cells. Modifies img in-place."""
    h, w = img.shape[:2]
    cell_w = w / cols
    cell_h = h / rows

    overlay = img.copy()
    for gy in range(rows):
        for gx in range(cols):
            if blocked[gy, gx]:
                x0 = int(gx * cell_w)
                y0 = int(gy * cell_h)
                x1 = int((gx + 1) * cell_w)
                y1 = int((gy + 1) * cell_h)
                cv2.rectangle(overlay, (x0, y0), (x1, y1), color, -1)

    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    return img


class VisionObstacleUpdater:
    def __init__(
        self,
        cam_index=0,
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

        self.grid_w = len(world.COLS)
        self.grid_h = len(world.ROWS)

        # Warp size derived from grid dimensions (swapped for horizontal debug view)
        self.warp_w = self.grid_h * CELL_PX
        self.warp_h = self.grid_w * CELL_PX

        self.H = np.load("H.npy")

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

        # Track dynamic cells to clear them properly without nuking static walls
        self.prev_dynamic_cells = set()

        # Warmup: let background model learn before detecting obstacles
        self._warmup_frames = 30
        self._frame_count = 0

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
        H, W = fg.shape
        # Warp is swapped: W corresponds to grid_h (7), H corresponds to grid_w (4)
        cw, ch = W / self.grid_h, H / self.grid_w

        blocked = np.zeros((self.grid_h, self.grid_w), dtype=bool)

        # Don't detect obstacles during warmup (let background model learn)
        self._frame_count += 1
        if self._frame_count < self._warmup_frames:
            self._blocked_any = False
            return blocked

        # Iterate in display order (rows=grid_w, cols=grid_h)
        # Debug display: 7 cols (x) × 4 rows (y), with (0,0) at top-right
        # GUI: 4 cols (A-D) × 7 rows (1-7), with A1 at top-left
        # Mapping: debug_col -> GUI_row, debug_row -> GUI_col (reversed)
        for disp_row in range(self.grid_w):  # 4 rows in display (0-3)
            y0, y1 = int(disp_row * ch), int((disp_row + 1) * ch)
            for disp_col in range(self.grid_h):  # 7 cols in display (0-6)
                x0, x1 = int(disp_col * cw), int((disp_col + 1) * cw)
                cell = fg[y0:y1, x0:x1]
                frac = (cell > 0).mean()

                # 50% threshold: if half or more of the cell is white, it's blocked
                if frac >= 0.4:
                    # Debug (0,0) is top-right, GUI (0,0) is top-left
                    # Debug col 0 is rightmost, col 6 is leftmost
                    # GUI row 0 is top, row 6 is bottom
                    # So: debug col 6 (left) -> GUI row 0 (top), debug col 0 (right) -> GUI row 6 (bottom)
                    gui_row = (self.grid_h - 1) - disp_col  # flip: debug left -> GUI top
                    gui_col = disp_row  # debug row 0 (top) -> GUI col 0 (A, leftmost)
                    blocked[gui_row, gui_col] = True

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
            # Draw grid + blocked overlay on both debug windows
            # Debug display: 7 cols x 4 rows, (0,0) at top-right
            # blocked[gui_row, gui_col] needs to map to debug display
            # debug_display[disp_row, disp_col] where disp_col=gui_row, disp_row=(3-gui_col)
            debug_rows = self.grid_w  # 4
            debug_cols = self.grid_h  # 7
            
            # Create debug blocked array with correct mapping
            # Reverse of: gui_row = (grid_h-1) - disp_col, gui_col = disp_row
            blocked_debug = np.zeros((debug_rows, debug_cols), dtype=bool)
            for gui_row in range(self.grid_h):
                for gui_col in range(self.grid_w):
                    if blocked[gui_row, gui_col]:
                        disp_row = gui_col
                        disp_col = (self.grid_h - 1) - gui_row
                        blocked_debug[disp_row, disp_col] = True

            warped_debug = warped.copy()
            draw_blocked_overlay(warped_debug, blocked_debug, debug_rows, debug_cols)
            draw_grid_overlay(warped_debug, debug_rows, debug_cols)

            fg_debug = cv2.cvtColor(fg, cv2.COLOR_GRAY2BGR)
            draw_blocked_overlay(fg_debug, blocked_debug, debug_rows, debug_cols)
            draw_grid_overlay(fg_debug, debug_rows, debug_cols)

            cv2.imshow("vision_warped", warped_debug)
            cv2.imshow("vision_fg", fg_debug)
            cv2.waitKey(1)

        # --- ignore set: start/goal + drone cell + drone 4-neighbors ---
        ignore = {start_label, goal_label}
        if drone_label is not None:
            ignore |= self._neighbors4_labels(drone_label)

        # Build new dynamic cells set
        # blocked[gui_row, gui_col] where gui_row=0-6, gui_col=0-3
        # xy_to_label(x, y) where x=col (0-3), y=row (0-6)
        new_dynamic_cells = set()
        for gui_row in range(self.grid_h):  # rows 0-6
            for gui_col in range(self.grid_w):  # cols 0-3
                if blocked[gui_row, gui_col]:
                    lbl = world.xy_to_label(gui_col, gui_row)
                    if lbl not in ignore:
                        new_dynamic_cells.add(lbl)

        # Clear old dynamic cells (only if they're not static walls)
        for lbl in self.prev_dynamic_cells:
            if lbl not in new_dynamic_cells:
                x, y = world.label_to_xy(lbl)
                # Only clear if it was set by us (check if currently blocked)
                if world.grid[y][x] == 1:
                    world.clear_obstacle(lbl)

        # Set new dynamic cells
        for lbl in new_dynamic_cells:
            if lbl not in self.prev_dynamic_cells:
                world.set_obstacle(lbl)

        # Compute added/removed for caller
        added = new_dynamic_cells - self.prev_dynamic_cells
        removed = self.prev_dynamic_cells - new_dynamic_cells

        self.prev_dynamic_cells = new_dynamic_cells
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
