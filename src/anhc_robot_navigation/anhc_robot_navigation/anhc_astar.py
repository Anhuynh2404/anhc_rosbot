"""
╔══════════════════════════════════════════════════════════════════╗
║              ANHC A* PATH PLANNING ALGORITHM                     ║
║                                                                  ║
║  Pure Python implementation — zero ROS dependencies.            ║
║  This makes it independently testable and replaceable           ║
║  (swap with D*, RRT, or RL policy without touching ROS nodes).  ║
║                                                                  ║
║  Algorithm: A* (Hart, Nilsson, Raphael — 1968)                  ║
║                                                                  ║
║  Data structures:                                                ║
║    - Open set  : heapq (min-heap on f = g + h)                  ║
║    - Closed set: set of (row, col) tuples                        ║
║    - came_from : dict mapping node → parent node                 ║
║    - g_score   : dict mapping node → cost from start            ║
║                                                                  ║
║  Heuristic: Euclidean distance (admissible + consistent)        ║
║  Movements: 8-directional (N,S,E,W + 4 diagonals)              ║
╚══════════════════════════════════════════════════════════════════╝
"""

import heapq
import math
from typing import List, Tuple, Optional, Dict


# ── Type aliases ────────────────────────────────────────────────
GridCell  = Tuple[int, int]          # (row, col)
Grid      = List[List[int]]          # 0 = free, 1 = obstacle
Path      = List[GridCell]           # ordered list of cells


# ════════════════════════════════════════════════════════════════
# NODE
# Wraps a grid cell with f, g, h scores for the priority queue.
# We store f first so heapq sorts by f automatically.
# ════════════════════════════════════════════════════════════════
class AStarNode:
    """
    Represents one cell in the A* search tree.

    Attributes:
        row, col : grid coordinates
        g        : cost from start to this node (exact)
        h        : heuristic estimate from this node to goal
        f        : total estimated cost  f = g + h
        parent   : reference to parent AStarNode (for path reconstruction)
    """

    __slots__ = ('row', 'col', 'g', 'h', 'f', 'parent')

    def __init__(self, row: int, col: int,
                 g: float = 0.0, h: float = 0.0,
                 parent: Optional['AStarNode'] = None):
        self.row    = row
        self.col    = col
        self.g      = g
        self.h      = h
        self.f      = g + h
        self.parent = parent

    # ── Comparison for heapq (min-heap on f, break ties on h) ──
    def __lt__(self, other: 'AStarNode') -> bool:
        if self.f == other.f:
            return self.h < other.h   # tie-break: prefer node closer to goal
        return self.f < other.f

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, AStarNode):
            return False
        return self.row == other.row and self.col == other.col

    def __hash__(self) -> int:
        return hash((self.row, self.col))

    def __repr__(self) -> str:
        return f'Node({self.row},{self.col} f={self.f:.2f})'


# ════════════════════════════════════════════════════════════════
# HEURISTICS
# All heuristics must be admissible (never overestimate true cost)
# for A* to guarantee the optimal path.
# ════════════════════════════════════════════════════════════════
class Heuristic:
    """
    Collection of heuristic functions.
    All take (current_cell, goal_cell) and return a float cost estimate.
    """

    @staticmethod
    def euclidean(a: GridCell, b: GridCell) -> float:
        """
        Straight-line distance.
        Admissible for 8-directional movement with diagonal cost √2.
        Best choice when diagonal moves are allowed.
        """
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    @staticmethod
    def manhattan(a: GridCell, b: GridCell) -> float:
        """
        Sum of absolute differences.
        Admissible ONLY for 4-directional movement.
        Faster to compute but suboptimal for 8-dir movement.
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def diagonal(a: GridCell, b: GridCell) -> float:
        """
        Chebyshev / Octile distance.
        Exact cost for 8-directional movement:
          - cardinal moves cost 1.0
          - diagonal moves cost √2 ≈ 1.414
        Most accurate heuristic for our use case.
        """
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)


# ════════════════════════════════════════════════════════════════
# A* PLANNER
# ════════════════════════════════════════════════════════════════
class AStarPlanner:
    """
    A* path planner on a 2D occupancy grid.

    The grid is a List[List[int]] where:
        0 = free cell
        1 = occupied / obstacle
        Values > 0 but < OBSTACLE_THRESHOLD = inflation zone

    Usage:
        planner = AStarPlanner(grid, resolution=0.1, inflation_radius=2)
        path = planner.plan((start_row, start_col), (goal_row, goal_col))
    """

    # Cells with value >= this threshold are treated as obstacles
    OBSTACLE_THRESHOLD: int = 50     # matches nav_msgs/OccupancyGrid convention

    # 8-directional movement: (delta_row, delta_col, move_cost)
    # Cardinal  = cost 1.0
    # Diagonal  = cost √2
    DIRECTIONS: List[Tuple[int, int, float]] = [
        (-1,  0, 1.000),   # North
        ( 1,  0, 1.000),   # South
        ( 0,  1, 1.000),   # East
        ( 0, -1, 1.000),   # West
        (-1,  1, 1.414),   # North-East
        (-1, -1, 1.414),   # North-West
        ( 1,  1, 1.414),   # South-East
        ( 1, -1, 1.414),   # South-West
    ]

    def __init__(self,
                 grid: Grid,
                 resolution: float = 0.1,
                 inflation_radius: int = 2,
                 heuristic: str = 'diagonal'):
        """
        Args:
            grid             : 2D occupancy grid [rows][cols]
            resolution       : meters per cell (for logging/scaling)
            inflation_radius : cells to inflate around obstacles (robot safety margin)
            heuristic        : 'euclidean' | 'manhattan' | 'diagonal'
        """
        self.resolution       = resolution
        self.inflation_radius = inflation_radius

        # Select heuristic function
        h_map = {
            'euclidean': Heuristic.euclidean,
            'manhattan': Heuristic.manhattan,
            'diagonal' : Heuristic.diagonal,
        }
        if heuristic not in h_map:
            raise ValueError(f'Unknown heuristic: {heuristic}. '
                             f'Choose from {list(h_map.keys())}')
        self._heuristic = h_map[heuristic]

        # Inflate obstacles for robot safety clearance
        self.grid = self._inflate_obstacles(grid, inflation_radius)

        self.rows = len(self.grid)
        self.cols = len(self.grid[0]) if self.rows > 0 else 0

        # Statistics (populated after each plan() call)
        self.stats: Dict = {}

    # ── Public API ───────────────────────────────────────────────
    def plan(self,
             start: GridCell,
             goal: GridCell) -> Optional[Path]:
        """
        Run A* from start to goal on the occupancy grid.

        Args:
            start : (row, col) of start cell
            goal  : (row, col) of goal  cell

        Returns:
            List of (row, col) from start to goal (inclusive),
            or None if no path exists.
        """
        # ── Validate inputs ─────────────────────────────────────
        if not self._in_bounds(start):
            raise ValueError(f'Start {start} is out of grid bounds '
                             f'({self.rows}×{self.cols})')
        if not self._in_bounds(goal):
            raise ValueError(f'Goal {goal} is out of grid bounds '
                             f'({self.rows}×{self.cols})')
        if self._is_obstacle(start):
            raise ValueError(f'Start {start} is inside an obstacle')
        if self._is_obstacle(goal):
            raise ValueError(f'Goal {goal} is inside an obstacle')
        if start == goal:
            return [start]

        # ── Initialize data structures ───────────────────────────
        #
        # open_heap : min-heap of AStarNode, sorted by f = g + h
        # open_set  : set of (row,col) for O(1) membership test
        # closed    : set of (row,col) already expanded
        # g_score   : best known cost from start to each cell
        #
        h0         = self._heuristic(start, goal)
        start_node = AStarNode(start[0], start[1], g=0.0, h=h0)

        open_heap: List[AStarNode] = [start_node]
        open_set:  Dict[GridCell, AStarNode] = {start: start_node}
        closed:    set = set()

        # g_score default = infinity (undiscovered)
        g_score: Dict[GridCell, float] = {start: 0.0}

        nodes_expanded = 0

        # ── Main A* Loop ─────────────────────────────────────────
        while open_heap:

            # Pop node with lowest f score
            current = heapq.heappop(open_heap)
            current_cell = (current.row, current.col)

            # Skip stale entries (node was re-added with better g)
            if current_cell in closed:
                continue

            # ── Goal reached → reconstruct path ─────────────────
            if current_cell == goal:
                path = self._reconstruct_path(current)
                self.stats = {
                    'path_length_cells': len(path),
                    'path_length_meters': len(path) * self.resolution,
                    'nodes_expanded': nodes_expanded,
                    'nodes_in_open': len(open_set),
                }
                return path

            # Mark as expanded
            closed.add(current_cell)
            nodes_expanded += 1

            # ── Expand neighbors ─────────────────────────────────
            for d_row, d_col, move_cost in self.DIRECTIONS:

                nb_row  = current.row + d_row
                nb_col  = current.col + d_col
                nb_cell = (nb_row, nb_col)

                # Skip: out of bounds, obstacle, or already expanded
                if not self._in_bounds(nb_cell):
                    continue
                if self._is_obstacle(nb_cell):
                    continue
                if nb_cell in closed:
                    continue

                # ── Corner cutting check ─────────────────────────
                # Prevent diagonal moves that squeeze between two
                # diagonal obstacles (causes robot to clip corners)
                if abs(d_row) == 1 and abs(d_col) == 1:
                    if (self._is_obstacle((current.row + d_row, current.col)) or
                            self._is_obstacle((current.row, current.col + d_col))):
                        continue

                # ── Compute tentative g score ────────────────────
                tentative_g = current.g + move_cost

                # If we found a better path to nb_cell, update
                if tentative_g < g_score.get(nb_cell, math.inf):
                    g_score[nb_cell] = tentative_g
                    h = self._heuristic(nb_cell, goal)
                    nb_node = AStarNode(
                        nb_row, nb_col,
                        g=tentative_g, h=h,
                        parent=current
                    )
                    heapq.heappush(open_heap, nb_node)
                    open_set[nb_cell] = nb_node

        # Open set exhausted — no path found
        self.stats = {
            'path_length_cells': 0,
            'nodes_expanded': nodes_expanded,
            'result': 'NO_PATH',
        }
        return None

    def update_grid(self, new_grid: Grid) -> None:
        """
        Hot-swap the grid without recreating the planner.
        Called by map_handler when a new occupancy map arrives.
        """
        self.grid = self._inflate_obstacles(new_grid, self.inflation_radius)
        self.rows = len(self.grid)
        self.cols = len(self.grid[0]) if self.rows > 0 else 0

    # ── Private helpers ──────────────────────────────────────────
    def _in_bounds(self, cell: GridCell) -> bool:
        """Return True if cell is within grid dimensions."""
        r, c = cell
        return 0 <= r < self.rows and 0 <= c < self.cols

    def _is_obstacle(self, cell: GridCell) -> bool:
        """Return True if cell value meets obstacle threshold."""
        r, c = cell
        return self.grid[r][c] >= self.OBSTACLE_THRESHOLD

    @staticmethod
    def _reconstruct_path(node: AStarNode) -> Path:
        """
        Walk parent pointers from goal back to start,
        then reverse to get start→goal order.
        """
        path: Path = []
        current: Optional[AStarNode] = node
        while current is not None:
            path.append((current.row, current.col))
            current = current.parent
        path.reverse()
        return path

    @staticmethod
    def _inflate_obstacles(grid: Grid, radius: int) -> Grid:
        """
        Inflate obstacles by `radius` cells in all directions.

        This creates a safety margin so the robot center never
        gets closer than `radius * resolution` meters to an obstacle.

        Uses a simple square dilation (fast, good enough for nav).
        For smoother inflation, replace with circular kernel.

        Args:
            grid   : original occupancy grid
            radius : inflation radius in cells

        Returns:
            New inflated grid (original is not modified)
        """
        if radius <= 0:
            return [row[:] for row in grid]   # deep copy

        rows = len(grid)
        cols = len(grid[0]) if rows > 0 else 0

        # Deep copy
        inflated = [row[:] for row in grid]

        # For each obstacle cell, mark surrounding cells
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] >= AStarPlanner.OBSTACLE_THRESHOLD:
                    for dr in range(-radius, radius + 1):
                        for dc in range(-radius, radius + 1):
                            nr, nc = r + dr, c + dc
                            if 0 <= nr < rows and 0 <= nc < cols:
                                # Use 100 (fully occupied in nav_msgs convention)
                                inflated[nr][nc] = 100

        return inflated


# ════════════════════════════════════════════════════════════════
# STANDALONE TEST (run: python3 anhc_astar.py)
# ════════════════════════════════════════════════════════════════
if __name__ == '__main__':

    # 10×20 grid — 0=free, 100=wall
    # Robot goes from (5,0) to (5,19), wall forces it to go around
    W = 100
    test_grid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, W, W, W, W, W, W, W, W, W, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, W, 0, 0, 0, 0, 0, 0, 0, W, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, W, 0, 0, 0, 0, 0, 0, 0, W, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, W, 0, 0, 0, 0, 0, 0, 0, W, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, W, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, W, W, W, W, W, W, W, W, W, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]

    planner = AStarPlanner(test_grid, resolution=0.1, inflation_radius=0)
    path = planner.plan(start=(5, 0), goal=(5, 19))

    if path:
        print(f'✅ Path found! {len(path)} cells')
        print(f'   Stats: {planner.stats}')

        # Visualize on grid
        vis = [row[:] for row in test_grid]
        for r, c in path:
            vis[r][c] = 50   # mark path

        print('\nGrid (S=start, G=goal, *=path, #=wall):')
        symbols = {0: '.', 50: '*', 100: '#'}
        for ri, row in enumerate(vis):
            line = ''
            for ci, val in enumerate(row):
                if (ri, ci) == (5, 0):
                    line += 'S '
                elif (ri, ci) == (5, 19):
                    line += 'G '
                else:
                    line += symbols.get(val, '? ') + ' '
            print(line)
    else:
        print('❌ No path found')
