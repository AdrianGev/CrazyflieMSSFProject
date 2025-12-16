import tkinter as tk
from tkinter import ttk, messagebox

import world
# gooey
COLS = world.COLS
ROWS = world.ROWS
grid = world.grid
label_to_xy = world.label_to_xy
xy_to_label = world.xy_to_label
set_obstacle = world.set_obstacle
clear_obstacle = world.clear_obstacle
reset_grid = world.reset_grid

from v1basic import plan_v1
from v2deadline import plan_v2
from v3neural import plan_v3

try:
    from commands import execute_path_on_cf
    HAVE_CF = True
except Exception:
    HAVE_CF = False

CELL_SIZE = 40  # pixels per grid cell
GRID_COLS = len(COLS)      # 4
GRID_ROWS = len(ROWS)      # 12

WINDOW_BG = "#f4f4f4"


class PathfindingGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Smarter Paths – Grid Planner")
        self.root.configure(bg=WINDOW_BG)

        self.start_label = "A1"
        self.goal_label = "D12"
        self.current_planner = tk.StringVar(value="v1")
        self.deadline_ms = tk.DoubleVar(value=20.0)
        self.current_path_labels = []

        self.drag_mode = None

        self._build_controls()
        self._build_canvas()
        self.redraw_grid()

    def _build_controls(self):
        ctrl_frame = ttk.Frame(self.root, padding=10)
        ctrl_frame.grid(row=0, column=0, sticky="ew")
        ctrl_frame.columnconfigure(0, weight=1)

        planner_label = ttk.Label(ctrl_frame, text="Planner:")
        planner_label.grid(row=0, column=0, sticky="w")

        self.planner_combo = ttk.Combobox(
            ctrl_frame,
            textvariable=self.current_planner,
            values=["v1 (basic A*)", "v2 (deadline A*)", "v3 (neural A*)"],
            state="readonly",
            width=25,
        )
        self.planner_combo.bind("<<ComboboxSelected>>", self._on_planner_selected)
        self.planner_combo.current(0)  # default to v1
        self.planner_combo.grid(row=0, column=1, sticky="w", padx=5)

        deadline_label = ttk.Label(ctrl_frame, text="Deadline (ms):")
        deadline_label.grid(row=0, column=2, sticky="e", padx=(15, 0))
        deadline_entry = ttk.Entry(ctrl_frame, textvariable=self.deadline_ms, width=8)
        deadline_entry.grid(row=0, column=3, sticky="w", padx=5)

        self.start_var = tk.StringVar(value=f"Start: {self.start_label}")
        self.goal_var = tk.StringVar(value=f"Goal: {self.goal_label}")

        start_lbl = ttk.Label(ctrl_frame, textvariable=self.start_var)
        start_lbl.grid(row=1, column=0, sticky="w", pady=(8, 0))

        goal_lbl = ttk.Label(ctrl_frame, textvariable=self.goal_var)
        goal_lbl.grid(row=1, column=1, sticky="w", pady=(8, 0))

        run_btn = ttk.Button(ctrl_frame, text="Run planner", command=self.run_planner)
        run_btn.grid(row=1, column=2, padx=5, pady=(8, 0))

        reset_btn = ttk.Button(ctrl_frame, text="Reset grid", command=self.reset_world)
        reset_btn.grid(row=1, column=3, padx=5, pady=(8, 0))

        if HAVE_CF:
            fly_btn = ttk.Button(
                ctrl_frame, text="Fly on Crazyflie", command=self.fly_path
            )
        else:
            fly_btn = ttk.Button(
                ctrl_frame,
                text="Fly on Crazyflie",
                command=self.fly_path,
                state="disabled",
            )
        fly_btn.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(8, 0))

        help_text = (
            "Click = toggle obstacle   |   Shift+Click = set Start   |   "
            "Ctrl+Click = set Goal   |   Drag Start/Goal to move them"
        )
        help_lbl = ttk.Label(ctrl_frame, text=help_text)
        help_lbl.grid(row=3, column=0, columnspan=4, sticky="w", pady=(8, 0))

    def _build_canvas(self):
        canvas_width = GRID_COLS * CELL_SIZE
        canvas_height = GRID_ROWS * CELL_SIZE

        self.canvas = tk.Canvas(
            self.root,
            width=canvas_width,
            height=canvas_height,
            bg="white",
            highlightthickness=0,
        )
        self.canvas.grid(row=1, column=0, padx=10, pady=10)

        self.canvas.bind("<ButtonPress-1>", self.on_canvas_press)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)

    def _on_planner_selected(self, event=None):
        text = self.planner_combo.get()
        if text.startswith("v1"):
            self.current_planner.set("v1")
        elif text.startswith("v2"):
            self.current_planner.set("v2")
        else:
            self.current_planner.set("v3")

    def redraw_grid(self):
        self.canvas.delete("all")

        path_set = set(self.current_path_labels)

        for y in range(GRID_ROWS):
            for x in range(GRID_COLS):
                x0 = x * CELL_SIZE
                y0 = y * CELL_SIZE
                x1 = x0 + CELL_SIZE
                y1 = y0 + CELL_SIZE

                label = xy_to_label(x, y)
                cell_val = grid[y][x]

                if label == self.start_label:
                    fill = "#6bd26b"  # green
                elif label == self.goal_label:
                    fill = "#ff6b6b"  # red
                elif cell_val == 1:
                    fill = "#333333"  # obstacle
                elif label in path_set:
                    fill = "#ffd66b"  # yellow for path
                else:
                    fill = "#ffffff"  # free

                self.canvas.create_rectangle(
                    x0, y0, x1, y1,
                    fill=fill,
                    outline="#aaaaaa",
                )

                self.canvas.create_text(
                    x0 + CELL_SIZE / 2,
                    y0 + CELL_SIZE / 2,
                    text=label,
                    font=("TkDefaultFont", 8),
                )

    def _event_to_cell(self, event):
        col = event.x // CELL_SIZE
        row = event.y // CELL_SIZE
        if not (0 <= col < GRID_COLS and 0 <= row < GRID_ROWS):
            return None, None, None
        label = xy_to_label(col, row)
        return col, row, label

    def on_canvas_press(self, event):
        col, row, label = self._event_to_cell(event)
        if label is None:
            return

        if (event.state & 0x0001):
            old = self.start_label
            self.start_label = label
            self.start_var.set(f"Start: {self.start_label}")
            self.current_path_labels = []
            print(f"[GUI] Start changed (Shift+Click): {old} -> {label}")
            self.redraw_grid()
            self.drag_mode = None
            return

        if (event.state & 0x0004):
            old = self.goal_label
            self.goal_label = label
            self.goal_var.set(f"Goal: {self.goal_label}")
            self.current_path_labels = []
            print(f"[GUI] Goal changed (Ctrl+Click): {old} -> {label}")
            self.redraw_grid()
            self.drag_mode = None
            return

        if label == self.start_label:
            self.drag_mode = "start"
            print(f"[GUI] Begin dragging start from {label}")
        elif label == self.goal_label:
            self.drag_mode = "goal"
            print(f"[GUI] Begin dragging goal from {label}")
        else:
            self.drag_mode = None
            if grid[row][col] == 0:
                set_obstacle(label)
                print(f"[GUI] Obstacle added at {label}")
            else:
                clear_obstacle(label)
                print(f"[GUI] Obstacle removed at {label}")
            self.current_path_labels = []
            self.redraw_grid()

    def on_canvas_drag(self, event):
        if self.drag_mode not in ("start", "goal"):
            return

        col, row, label = self._event_to_cell(event)
        if label is None:
            return

        if self.drag_mode == "start":
            if label == self.start_label or label == self.goal_label:
                return
            old = self.start_label

            if grid[row][col] == 1:
                clear_obstacle(label)
                print(f"[GUI] Obstacle removed at {label} to place start")

            self.start_label = label
            self.start_var.set(f"Start: {self.start_label}")
            self.current_path_labels = []
            print(f"[GUI] Start moved (drag): {old} -> {label}")
            self.redraw_grid()

        elif self.drag_mode == "goal":
            if label == self.goal_label or label == self.start_label:
                return
            old = self.goal_label

            if grid[row][col] == 1:
                clear_obstacle(label)
                print(f"[GUI] Obstacle removed at {label} to place goal")

            self.goal_label = label
            self.goal_var.set(f"Goal: {self.goal_label}")
            self.current_path_labels = []
            print(f"[GUI] Goal moved (drag): {old} -> {label}")
            self.redraw_grid()

    def on_canvas_release(self, event):
        if self.drag_mode in ("start", "goal"):
            print(f"[GUI] End drag ({self.drag_mode})")
        self.drag_mode = None

    def reset_world(self):
        reset_grid(0)
        self.current_path_labels = []
        print("[GUI] Grid reset (all cells free)")
        self.redraw_grid()

    def run_planner(self):
        planner = self.current_planner.get()
        try:
            deadline = float(self.deadline_ms.get())
        except ValueError:
            messagebox.showerror("Invalid deadline", "Deadline must be a number (milliseconds).")
            return

        print(f"[GUI] Running planner={planner}, start={self.start_label}, goal={self.goal_label}")

        if planner == "v1":
            path = plan_v1(self.start_label, self.goal_label)
            hit_deadline = False
        elif planner == "v2":
            path, hit_deadline = plan_v2(
                self.start_label, self.goal_label, deadline_ms=deadline
            )
        else:
            path, hit_deadline = plan_v3(
                self.start_label, self.goal_label, deadline_ms=deadline
            )

        if path is None or len(path) == 0:
            self.current_path_labels = []
            self.redraw_grid()
            print("[GUI] No path found.")
            messagebox.showwarning("No path", "No path found (or empty path).")
            return

        self.current_path_labels = path
        self.redraw_grid()

        msg = f"[GUI] Path length: {len(path)}"
        if planner in ("v2", "v3"):
            msg += f" | Hit deadline? {'YES' if hit_deadline else 'NO'}"
        print(msg)

    def fly_path(self):
        if not HAVE_CF:
            messagebox.showerror(
                "Crazyflie not available",
                "Crazyflie libraries not imported. Run this on your drone laptop or install cflib.",
            )
            return

        if not self.current_path_labels or len(self.current_path_labels) < 2:
            messagebox.showwarning(
                "No path", "No valid path to fly. Run a planner first."
            )
            return

        if not messagebox.askyesno(
            "Confirm flight",
            f"Fly path with {len(self.current_path_labels)} cells on Crazyflie?",
        ):
            return

        print(f"[GUI] Executing path on Crazyflie: {self.current_path_labels}")
        try:
            planner = self.current_planner.get()
            compress = planner in ("v2", "v3")
            execute_path_on_cf(self.current_path_labels, compress=compress)
            print("[GUI] Crazyflie path execution finished.")
        except Exception as e:
            print("[GUI] Crazyflie error:", e)
            messagebox.showerror("Crazyflie error", f"Error during flight:\n{e}")


def main():
    root = tk.Tk()
    app = PathfindingGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()