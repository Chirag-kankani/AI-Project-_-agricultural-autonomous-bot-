import collections
import heapq
import time
import math
import itertools

class SearchResult:
    """
    Holds the output and performance metrics of a search algorithm.
    """
    def __init__(self, name, path=None, visited_order=None, nodes_expanded=0, 
                 path_cost=0.0, execution_time_ms=0.0, success=False, segment_paths=None):
        self.name = name
        self.path = path if path is not None else []
        self.visited_order = visited_order if visited_order is not None else []
        self.nodes_expanded = nodes_expanded
        self.path_cost = path_cost
        self.execution_time_ms = execution_time_ms
        self.success = success
        self.segment_paths = segment_paths if segment_paths is not None else []
        self.path_length = len(self.path) - 1 if self.success and len(self.path) > 1 else 0

    def __repr__(self):
        status = "Success" if self.success else "Failed"
        return (f"[{self.name}] Status: {status} | Path Length: {self.path_length} steps | "
                f"Cost: {self.path_cost:.2f} | Expanded: {self.nodes_expanded} | "
                f"Time: {self.execution_time_ms:.3f} ms")


# Heuristic Functions

def manhattan_distance(pos, goal):
    """
    Manhattan distance: h = |r1 - r2| + |c1 - c2|
    Admissible and consistent for 4-directional grid movement.
    """
    return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])

def euclidean_distance(pos, goal):
    """
    Euclidean straight-line distance: h = sqrt((r1 - r2)^2 + (c1 - c2)^2)
    Also admissible for grid movement.
    """
    return math.sqrt((pos[0] - goal[0])**2 + (pos[1] - goal[1])**2)


# Path Cost & Reconstruction Helpers

def reconstruct_path(came_from, current):
    """Backtracks parent pointers from goal to start to build the path."""
    path = [current]
    while current in came_from and came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def compute_path_cost(field, path, use_crop_safety=True):
    """Calculates total cost along a path based on cell costs."""
    if not path or len(path) <= 1:
        return 0.0
    cost = 0.0
    for node in path[1:]: # Starting cell has cost 0
        cost += field.get_step_cost(node[0], node[1], use_crop_safety=use_crop_safety)
    return cost


# 1. Breadth-First Search (BFS)

def bfs(field, start, goal, robot_view=False):
    """
    Breadth-First Search (Uninformed).
    Uses a FIFO Queue. Guarantees finding the shortest path in terms of steps.
    """
    start_time = time.perf_counter()
    
    queue = collections.deque([start])
    visited = set([start])
    came_from = {start: None}
    visited_order = []
    nodes_expanded = 0

    success = False
    
    while queue:
        current = queue.popleft()
        nodes_expanded += 1
        visited_order.append(current)

        if current == goal:
            success = True
            break

        for neighbor in field.get_neighbors(current[0], current[1], robot_view=robot_view):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    
    path = reconstruct_path(came_from, goal) if success else []
    total_cost = compute_path_cost(field, path) if success else 0.0

    return SearchResult(
        name="BFS",
        path=path,
        visited_order=visited_order,
        nodes_expanded=nodes_expanded,
        path_cost=total_cost,
        execution_time_ms=elapsed_ms,
        success=success
    )


# 2. Depth-First Search (DFS)

def dfs(field, start, goal, robot_view=False):
    """
    Depth-First Search (Uninformed).
    Uses a LIFO Stack. Explores deep branches first; not guaranteed optimal.
    """
    start_time = time.perf_counter()
    
    stack = [start]
    visited = set([start])
    came_from = {start: None}
    visited_order = []
    nodes_expanded = 0

    success = False

    while stack:
        current = stack.pop()
        nodes_expanded += 1
        visited_order.append(current)

        if current == goal:
            success = True
            break

        for neighbor in field.get_neighbors(current[0], current[1], robot_view=robot_view):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                stack.append(neighbor)

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0

    path = reconstruct_path(came_from, goal) if success else []
    total_cost = compute_path_cost(field, path) if success else 0.0

    return SearchResult(
        name="DFS",
        path=path,
        visited_order=visited_order,
        nodes_expanded=nodes_expanded,
        path_cost=total_cost,
        execution_time_ms=elapsed_ms,
        success=success
    )


# 3. Greedy Best-First Search (GBFS)

def greedy_best_first_search(field, start, goal, heuristic=manhattan_distance, robot_view=False):
    """
    Greedy Best-First Search (Informed).
    Priority queue ordered strictly by h(n) = heuristic estimate to goal.
    Fast, but can be suboptimal as it ignores accumulated path cost g(n).
    """
    start_time = time.perf_counter()

    frontier = []
    counter = 0 # Tie-breaker
    heapq.heappush(frontier, (heuristic(start, goal), counter, start))
    
    visited = set([start])
    came_from = {start: None}
    visited_order = []
    nodes_expanded = 0

    success = False

    while frontier:
        h_val, _, current = heapq.heappop(frontier)
        nodes_expanded += 1
        visited_order.append(current)

        if current == goal:
            success = True
            break

        for neighbor in field.get_neighbors(current[0], current[1], robot_view=robot_view):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                counter += 1
                heapq.heappush(frontier, (heuristic(neighbor, goal), counter, neighbor))

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0

    path = reconstruct_path(came_from, goal) if success else []
    total_cost = compute_path_cost(field, path) if success else 0.0

    return SearchResult(
        name="Greedy Best-First",
        path=path,
        visited_order=visited_order,
        nodes_expanded=nodes_expanded,
        path_cost=total_cost,
        execution_time_ms=elapsed_ms,
        success=success
    )


# 4. A* Search

def a_star_search(field, start, goal, heuristic=manhattan_distance, 
                  use_crop_safety=True, robot_view=False, algorithm_label="A*"):
    """
    A* Search (Informed).
    Priority queue ordered by f(n) = g(n) + h(n).
    g(n): exact cost to reach node n from start.
    h(n): heuristic estimate from n to goal.
    Guaranteed to find the optimal least-cost path when h(n) is admissible & consistent.
    """
    start_time = time.perf_counter()

    frontier = []
    counter = 0
    heapq.heappush(frontier, (0.0 + heuristic(start, goal), counter, start))

    g_score = {start: 0.0}
    came_from = {start: None}
    visited = set()
    visited_order = []
    nodes_expanded = 0

    success = False

    while frontier:
        f_val, _, current = heapq.heappop(frontier)

        if current in visited:
            continue

        visited.add(current)
        nodes_expanded += 1
        visited_order.append(current)

        if current == goal:
            success = True
            break

        for neighbor in field.get_neighbors(current[0], current[1], robot_view=robot_view):
            step_cost = field.get_step_cost(neighbor[0], neighbor[1], use_crop_safety=use_crop_safety)
            tentative_g = g_score[current] + step_cost

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f_score = tentative_g + heuristic(neighbor, goal)
                counter += 1
                heapq.heappush(frontier, (f_score, counter, neighbor))

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0

    path = reconstruct_path(came_from, goal) if success else []
    total_cost = g_score.get(goal, 0.0) if success else 0.0

    return SearchResult(
        name=algorithm_label,
        path=path,
        visited_order=visited_order,
        nodes_expanded=nodes_expanded,
        path_cost=total_cost,
        execution_time_ms=elapsed_ms,
        success=success
    )


# Multi-Checkpoint Mission Path Planning (Agricultural Monitoring)

def plan_mission_path(field, waypoints, search_algo_fn, **kwargs):
    """
    Executes a complete agricultural monitoring mission visiting all waypoints:
    waypoints = [Start, Checkpoint_1, Checkpoint_2, ..., Destination].
    Computes sequential path legs and stitches them into a continuous route.
    """
    if len(waypoints) < 2:
        return SearchResult("Empty Mission", success=False)
    
    if len(waypoints) == 2:
        return search_algo_fn(field, waypoints[0], waypoints[1], **kwargs)

    combined_path = [waypoints[0]]
    total_cost = 0.0
    total_expanded = 0
    total_time = 0.0
    all_visited = []
    segment_paths = []
    algo_name = ""

    for i in range(len(waypoints) - 1):
        src = waypoints[i]
        dst = waypoints[i + 1]
        leg_res = search_algo_fn(field, src, dst, **kwargs)
        algo_name = leg_res.name

        if not leg_res.success:
            return SearchResult(algo_name, success=False)

        segment_paths.append(leg_res.path)
        combined_path.extend(leg_res.path[1:])
        total_cost += leg_res.path_cost
        total_expanded += leg_res.nodes_expanded
        total_time += leg_res.execution_time_ms
        all_visited.extend(leg_res.visited_order)

    return SearchResult(
        name=algo_name,
        path=combined_path,
        visited_order=all_visited,
        nodes_expanded=total_expanded,
        path_cost=total_cost,
        execution_time_ms=total_time,
        success=True,
        segment_paths=segment_paths
    )

# Farmer's Optimal Checkpoint Tour Optimizer (TSP for Agriculture)

def optimize_farmer_checkpoint_tour(start, checkpoints, goal):
    """
    Calculates the optimal visiting order of checkpoints to minimize
    total travel distance and battery consumption for the farmer.
    Uses Manhattan distance estimation across permutations (exact for K <= 7).
    """
    if not checkpoints or len(checkpoints) <= 1:
        return list(checkpoints)

    best_tour = list(checkpoints)
    best_dist = float('inf')

    for perm in itertools.permutations(checkpoints):
        route = [start] + list(perm) + [goal]
        dist = sum(manhattan_distance(route[i], route[i+1]) for i in range(len(route)-1))
        if dist < best_dist:
            best_dist = dist
            best_tour = list(perm)

    return best_tour
