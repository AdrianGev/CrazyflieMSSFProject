COLS = ["A", "B", "C", "D"]
ROWS = list(range(1, 13))  # 1..12

# grid[y][x], y = row index (0..11), x = col index (0..3)
# 0 = free, 1 = obstacle
grid = [[0 for _ in COLS] for _ in ROWS]
# wrld

def label_to_xy(label: str):
    """
    "A1" -> (0, 0)
    "D12" -> (3, 11)
    """
    col_letter = label[0].upper()
    row_number = int(label[1:])
    x = COLS.index(col_letter)
    y = row_number - 1
    return (x, y)


def xy_to_label(x: int, y: int) -> str:
    """
    (0, 0) -> "A1"
    (3, 11) -> "D12"
    """
    col = COLS[x]
    row = y + 1
    return f"{col}{row}"


def set_obstacle(label: str):
    x, y = label_to_xy(label)
    grid[y][x] = 1


def clear_obstacle(label: str):
    x, y = label_to_xy(label)
    grid[y][x] = 0


def reset_grid(value: int = 0):
    """Set entire grid to a value (0 = all free, 1 = all blocked)."""
    for y in range(len(grid)):
        for x in range(len(grid[0])):
            grid[y][x] = value