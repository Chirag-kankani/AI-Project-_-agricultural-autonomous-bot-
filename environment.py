import random

CELL_SAFE = 0             
CELL_POTATO_ROW = 1       
CELL_CROP_BUFFER = 2      
CELL_STATIC_OBSTACLE = 3
CELL_UNKNOWN_OBSTACLE = 4 

class FieldGrid:
    """
    Represents a 2D grid potato field with crop rows, buffer zones,
    static/unknown obstacles, and crop monitoring checkpoints.
    """
    def __init__(self, rows=12, cols=16, buffer_penalty=4.0):
        self.rows = rows
        self.cols = cols
        self.buffer_penalty = buffer_penalty
        
        self.grid = [[CELL_SAFE for _ in range(cols)] for _ in range(rows)]
        
        self.start = (1, 1)
        self.goal = (rows - 2, cols - 2)
        
        self.checkpoints = []
        
        self.unknown_obstacles = []

    def in_bounds(self, r, c):
        """Check if (r, c) is within grid boundaries."""
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_passable(self, r, c, robot_view=False):
        """
        Check if a cell is passable.
        - POTATO_ROW and STATIC_OBSTACLE are never passable.
        - If robot_view=True, UNKNOWN_OBSTACLE is treated as passable
          because the robot does not yet know it exists.
        """
        if not self.in_bounds(r, c):
            return False
        
        cell = self.grid[r][c]
        if cell == CELL_POTATO_ROW or cell == CELL_STATIC_OBSTACLE:
            return False
        if cell == CELL_UNKNOWN_OBSTACLE:
            return True if robot_view else False
        return True

    def get_step_cost(self, r, c, use_crop_safety=True):
        """
        Calculates cost of stepping into cell (r, c).
        - Safe alley cell: base cost = 1.0
        - Buffer cell: 1.0 + buffer_penalty (if crop safety enabled)
        """
        base_cost = 1.0
        if not self.in_bounds(r, c):
            return float('inf')
        
        if use_crop_safety and self.grid[r][c] == CELL_CROP_BUFFER:
            return base_cost + self.buffer_penalty
        return base_cost

    def get_neighbors(self, r, c, robot_view=False):
        """
        Return valid orthogonal 4-directional neighboring cells:
        Ordered: Right (0, 1), Down (1, 0), Up (-1, 0), Left (0, -1).
        This balanced ordering produces intuitive shortest-path exploration for BFS
        and clearly exhibits deep exploration & backtracking for DFS.
        """
        neighbors = []
        actions = [(0, 1), (1, 0), (-1, 0), (0, -1)]
        for dr, dc in actions:
            nr, nc = r + dr, c + dc
            if self.is_passable(nr, nc, robot_view=robot_view):
                neighbors.append((nr, nc))
        return neighbors

    def add_crop_buffers(self):
        """
        Automatically mark cells adjacent (8-neighborhood) to potato rows
        as CROP_BUFFER cells (unless they are already potato rows or obstacles).
        """
        for r in range(self.rows):
            for c in range(self.cols):
                if self.grid[r][c] == CELL_POTATO_ROW:
                    for dr in [-1, 0, 1]:
                        for dc in [-1, 0, 1]:
                            nr, nc = r + dr, c + dc
                            if self.in_bounds(nr, nc):
                                if self.grid[nr][nc] == CELL_SAFE:
                                    self.grid[nr][nc] = CELL_CROP_BUFFER

    def add_checkpoint(self, r, c):
        """Adds a crop inspection checkpoint if valid and within bounds."""
        if self.in_bounds(r, c) and self.is_passable(r, c):
            if (r, c) not in self.checkpoints and (r, c) != self.start and (r, c) != self.goal:
                self.checkpoints.append((r, c))
                return True
        return False

    def get_suggested_checkpoints(self, count=2):
        """
        Intelligently finds realistic monitoring checkpoint locations:
        Alley cells immediately adjacent to potato rows (for camera/soil inspection).
        """
        candidates = []
        for r in range(1, self.rows - 1):
            for c in range(1, self.cols - 1):
                if self.is_passable(r, c) and (r, c) != self.start and (r, c) != self.goal:
                    # Check if next to a potato row
                    adjacent_to_crop = any(
                        self.in_bounds(r+dr, c+dc) and self.grid[r+dr][c+dc] == CELL_POTATO_ROW
                        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                    )
                    if adjacent_to_crop:
                        candidates.append((r, c))
        
        # Pick spaced out candidates
        if len(candidates) <= count:
            return candidates
        step = len(candidates) // count
        return [candidates[i * step] for i in range(count)]

    @classmethod
    def create_default_demo_field(cls):
        """
        Creates a realistic, structured potato field layout (12x16).
        - Parallel horizontal potato beds with designated crossing gaps.
        - Selected crop monitoring checkpoints for plant inspection (Task 1).
        - Clear headlands and perimeter roads for turning/safe transit.
        - Demonstrates clear distinction between unweighted and Crop-Safe A*.
        - Placed unknown obstacle on the top perimeter alley at (0, 10).
        """
        field = cls(rows=12, cols=16, buffer_penalty=4.0)
        field.start = (1, 1)
        field.goal = (10, 14)

        for c in range(2, 14):
            field.grid[3][c] = CELL_POTATO_ROW
            field.grid[6][c] = CELL_POTATO_ROW
            field.grid[9][c] = CELL_POTATO_ROW

        field.grid[3][4] = CELL_SAFE
        field.grid[6][11] = CELL_SAFE
        field.grid[9][7] = CELL_SAFE

        field.add_crop_buffers()

        static_obs = [(1, 7), (5, 14), (8, 1)]
        for r, c in static_obs:
            if (r, c) != field.start and (r, c) != field.goal:
                field.grid[r][c] = CELL_STATIC_OBSTACLE

        field.grid[4][8] = CELL_UNKNOWN_OBSTACLE
        field.unknown_obstacles.append((4, 8))

        field.checkpoints = [(3, 4), (6, 11)]

        field.grid[field.start[0]][field.start[1]] = CELL_SAFE
        field.grid[field.goal[0]][field.goal[1]] = CELL_SAFE

        return field

    @classmethod
    def create_custom_field(cls, rows, cols, num_obstacles, start=None, goal=None, checkpoints=None):
        """
        Creates a custom field grid with user-specified dimensions, obstacle count,
        and farmer-selected inspection checkpoints.
        """
        field = cls(rows=rows, cols=cols, buffer_penalty=4.0)
        
        field.start = start if start and field.in_bounds(*start) else (0, 0)
        field.goal = goal if goal and field.in_bounds(*goal) else (rows - 1, cols - 1)

        # Place potato rows every 3 rows if space permits
        if rows >= 7 and cols >= 6:
            for r in range(2, rows - 2, 3):
                gap_col_1 = max(1, cols // 3)
                gap_col_2 = min(cols - 2, 2 * (cols // 3))
                for c in range(2, cols - 2):
                    if c != gap_col_1 and c != gap_col_2:
                        field.grid[r][c] = CELL_POTATO_ROW

        field.add_crop_buffers()

        placed_obstacles = 0
        attempts = 0
        max_attempts = rows * cols * 10
        
        while placed_obstacles < num_obstacles and attempts < max_attempts:
            attempts += 1
            r = random.randint(0, rows - 1)
            c = random.randint(0, cols - 1)
            if (r, c) == field.start or (r, c) == field.goal:
                continue
            if field.grid[r][c] in [CELL_SAFE, CELL_CROP_BUFFER]:
                field.grid[r][c] = CELL_STATIC_OBSTACLE
                placed_obstacles += 1

        attempts = 0
        while attempts < max_attempts:
            attempts += 1
            r = random.randint(0, rows - 1)
            c = random.randint(0, cols - 1)
            if (r, c) == field.start or (r, c) == field.goal:
                continue
            if field.grid[r][c] in [CELL_SAFE, CELL_CROP_BUFFER]:
                field.grid[r][c] = CELL_UNKNOWN_OBSTACLE
                field.unknown_obstacles.append((r, c))
                break

        if checkpoints is not None:
            for cp in checkpoints:
                field.add_checkpoint(*cp)
        else:
            suggested = field.get_suggested_checkpoints(count=4)
            field.checkpoints = suggested

        field.grid[field.start[0]][field.start[1]] = CELL_SAFE
        field.grid[field.goal[0]][field.goal[1]] = CELL_SAFE

        return field
