"""
replanner.py - Obstacle Detection and Online Replanning Simulation (Task 5)
Supports multi-checkpoint agricultural monitoring missions.
Author: AI Assignment Group
"""

import copy
from algorithms import a_star_search, manhattan_distance, compute_path_cost, plan_mission_path

class ReplanEvent:
    """Records details of when an obstacle was detected and route replanned."""
    def __init__(self, step_number, robot_pos, obstacle_pos, old_path_remaining, new_path):
        self.step_number = step_number
        self.robot_pos = robot_pos
        self.obstacle_pos = obstacle_pos
        self.old_path_remaining = old_path_remaining
        self.new_path = new_path

class ReplanningSimulationResult:
    """Holds the full output of the dynamic online replanning journey."""
    def __init__(self):
        self.initial_path = []
        self.final_trajectory = []
        self.replan_events = []
        self.replan_count = 0
        self.total_steps = 0
        self.total_cost = 0.0
        self.destination_reached = False
        self.detected_obstacles = []
        self.visited_checkpoints = []

def simulate_robot_journey(field, waypoints=None, sensor_range=1, use_crop_safety=True):
    """
    Simulates the robot's journey visiting all crop monitoring checkpoints and destination:
    1. Initial planning: Robot plans complete route through all checkpoints without knowing hidden obstacles.
    2. Step-by-step traversal: At each step, proximity sensor scans surrounding cells.
    3. Detection: If an unknown obstacle is spotted, robot registers it in internal map.
    4. Replanning: If the obstacle blocks the remaining path, A* replans from current position
       to the current target checkpoint and remaining waypoints.
    5. Success when all checkpoints and final destination are reached!
    """
    sim_result = ReplanningSimulationResult()

    if waypoints is None:
        waypoints = [field.start] + list(field.checkpoints) + [field.goal]

    # Robot's internal map copy where unknown obstacles are initially treated as passable
    robot_field = copy.deepcopy(field)

    # Step 1: Initial Mission Path Planning
    initial_search = plan_mission_path(
        robot_field, 
        waypoints, 
        a_star_search,
        heuristic=manhattan_distance,
        use_crop_safety=use_crop_safety,
        robot_view=True
    )

    if not initial_search.success:
        print("[Replanner] No initial path could be found through all checkpoints!")
        return sim_result

    sim_result.initial_path = list(initial_search.path)
    current_planned_path = list(initial_search.path)

    # Waypoints to visit in sequence (index 1 to end)
    remaining_targets = list(waypoints[1:])

    # Step 2: Begin Movement
    current_pos = robot_field.start
    trajectory = [current_pos]
    path_index = 0
    total_cost = 0.0

    print(f"\n[Journey Start] Robot starting at {current_pos}. Monitoring Mission with {len(waypoints)-2} checkpoints.")
    print(f"Waypoints sequence: {waypoints}")
    print(f"Initial route length: {len(current_planned_path)-1} steps.")

    while current_pos != field.goal:
        # Check if current_pos reached the next checkpoint
        if remaining_targets and current_pos == remaining_targets[0]:
            reached_target = remaining_targets.pop(0)
            sim_result.visited_checkpoints.append(reached_target)
            if reached_target != field.goal:
                print(f" -> [CHECKPOINT REACHED] Successfully inspected crop monitoring station at {reached_target}!")

        if current_pos == field.goal:
            break

        # Look ahead to next waypoint
        if path_index + 1 >= len(current_planned_path):
            print("[Replanner Error] Ran out of planned path without reaching goal.")
            break

        # Move to next cell
        path_index += 1
        next_pos = current_planned_path[path_index]
        
        # SENSOR SCAN: Scan cells within sensor_range
        obstacle_detected_this_step = False
        detected_pos = None

        for dr in range(-sensor_range, sensor_range + 1):
            for dc in range(-sensor_range, sensor_range + 1):
                if abs(dr) + abs(dc) <= sensor_range:
                    sr, sc = current_pos[0] + dr, current_pos[1] + dc
                    if field.in_bounds(sr, sc):
                        if (sr, sc) in field.unknown_obstacles and (sr, sc) not in sim_result.detected_obstacles:
                            print(f" -> [SENSOR ALERT] Unknown obstacle detected at ({sr}, {sc}) from current position {current_pos}!")
                            sim_result.detected_obstacles.append((sr, sc))
                            robot_field.grid[sr][sc] = 3 # Mark as known static obstacle
                            obstacle_detected_this_step = True
                            detected_pos = (sr, sc)

        # Check if the detected obstacle blocks the remaining planned path
        remaining_path = current_planned_path[path_index:]
        is_blocked = any(obs in remaining_path for obs in sim_result.detected_obstacles)

        if is_blocked:
            print(f" -> [BLOCKED] Remaining route is blocked by obstacle at {detected_pos}! Initiating Online A* Replanning...")
            
            # Replan from current position through all remaining targets
            sub_waypoints = [current_pos] + remaining_targets
            replan_search = plan_mission_path(
                robot_field,
                sub_waypoints,
                a_star_search,
                heuristic=manhattan_distance,
                use_crop_safety=use_crop_safety,
                robot_view=False
            )

            if not replan_search.success:
                print(" -> [Replanner Error] Replanning failed: no alternative route exists to remaining targets!")
                break

            sim_result.replan_count += 1
            event = ReplanEvent(
                step_number=len(trajectory),
                robot_pos=current_pos,
                obstacle_pos=detected_pos,
                old_path_remaining=remaining_path,
                new_path=list(replan_search.path)
            )
            sim_result.replan_events.append(event)

            # Update planned path to the new path
            current_planned_path = list(replan_search.path)
            path_index = 0
            print(f" -> [REPLAN SUCCESS] Alternative crop-safe path found ({len(current_planned_path)-1} remaining steps). Resuming inspection.")
            continue

        # Execute safe move to next_pos
        step_cost = field.get_step_cost(next_pos[0], next_pos[1], use_crop_safety=use_crop_safety)
        total_cost += step_cost
        current_pos = next_pos
        trajectory.append(current_pos)

    # Check final destination
    if remaining_targets and current_pos == remaining_targets[0]:
        reached_target = remaining_targets.pop(0)
        sim_result.visited_checkpoints.append(reached_target)

    sim_result.final_trajectory = trajectory
    sim_result.total_steps = len(trajectory) - 1
    sim_result.total_cost = total_cost
    sim_result.destination_reached = (current_pos == field.goal)

    if sim_result.destination_reached:
        print(f"[Mission Complete] All checkpoints visited and reached destination at {current_pos}! "
              f"Total steps: {sim_result.total_steps}, Total cost: {sim_result.total_cost:.2f}, Replans: {sim_result.replan_count}\n")
    else:
        print(f"[Mission Incomplete] Stopped at {current_pos}.\n")

    return sim_result
