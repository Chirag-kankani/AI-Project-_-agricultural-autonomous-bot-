"""
main.py - Main Interactive Program with Live Step-by-Step Animations & Crop Checkpoints
Author: AI Assignment Group
"""

import sys
import os
from environment import (
    FieldGrid, 
    CELL_SAFE, 
    CELL_POTATO_ROW, 
    CELL_CROP_BUFFER, 
    CELL_STATIC_OBSTACLE, 
    CELL_UNKNOWN_OBSTACLE
)
from algorithms import (
    bfs, 
    dfs, 
    greedy_best_first_search, 
    a_star_search, 
    manhattan_distance,
    plan_mission_path,
    optimize_farmer_checkpoint_tour
)
from replanner import simulate_robot_journey
from visualizer import (
    visualize_all_algorithms, 
    animate_search_process, 
    animate_replanning_journey
)

def print_banner():
    banner = """
================================================================================
     AI-BASED CROP-SAFE PATH PLANNING FOR AUTONOMOUS POTATO-FIELD ROBOT
     Precision Agriculture Monitoring: Crop Checkpoints & Multi-Goal Tour
     Algorithms: BFS | DFS | Greedy Best-First | A* (Crop-Safe & Standard)
================================================================================
"""
    print(banner)


def print_ascii_grid(field, show_unknown=False):
    """Prints a friendly ASCII representation of the field in terminal."""
    symbols = {
        CELL_SAFE: " . ",
        CELL_POTATO_ROW: "[P]",
        CELL_CROP_BUFFER: " ~ ",
        CELL_STATIC_OBSTACLE: " # ",
        CELL_UNKNOWN_OBSTACLE: " ? " if not show_unknown else " ! "
    }
    
    cp_map = {pos: f"C{i+1}" for i, pos in enumerate(field.checkpoints)}
    
    print("\n" + "=" * 60)
    print(f" FIELD GRID MAP ({field.rows} rows x {field.cols} cols)")
    print(" S: Start | G: Destination | C1,C2: Crop Checkpoints | [P]: Potato Row | #: Obstacle")
    print("=" * 60)
    
    header = "    " + "".join(f"{c:4d}" for c in range(field.cols))
    print(header)
    print("   +" + "----" * field.cols + "+")

    for r in range(field.rows):
        row_str = f"{r:2d} |"
        for c in range(field.cols):
            if (r, c) == field.start:
                row_str += " S  "
            elif (r, c) == field.goal:
                row_str += " G  "
            elif (r, c) in cp_map:
                row_str += f"[{cp_map[(r, c)]}]"
            else:
                cell_type = field.grid[r][c]
                if not show_unknown and cell_type == CELL_UNKNOWN_OBSTACLE:
                    cell_type = CELL_SAFE
                sym = symbols.get(cell_type, " . ")
                row_str += f" {sym} "[:4]
        row_str += "|"
        print(row_str)
    print("   +" + "----" * field.cols + "+")
    print(f"Start: {field.start} | Goal: {field.goal}")
    if field.checkpoints:
        print(f"Crop Inspection Checkpoints to Visit: {field.checkpoints}")
    print()


def compute_all_algorithms(field):
    """Executes all algorithms on the given field through all checkpoints and returns results."""
    waypoints = [field.start] + list(field.checkpoints) + [field.goal]

    res_bfs = plan_mission_path(field, waypoints, bfs, robot_view=False)
    res_dfs = plan_mission_path(field, waypoints, dfs, robot_view=False)
    res_gbfs = plan_mission_path(field, waypoints, greedy_best_first_search, heuristic=manhattan_distance, robot_view=False)
    res_astar_std = plan_mission_path(field, waypoints, a_star_search, heuristic=manhattan_distance, 
                                      use_crop_safety=False, robot_view=False, algorithm_label="A* (Standard)")
    res_astar_safe = plan_mission_path(field, waypoints, a_star_search, heuristic=manhattan_distance, 
                                       use_crop_safety=True, robot_view=False, algorithm_label="A* (Crop-Safe)")

    return [res_bfs, res_dfs, res_gbfs, res_astar_std, res_astar_safe]


def print_comparison_table(all_results):
    """Prints the performance evaluation table in the terminal."""
    print("\n" + "=" * 88)
    print(f"{'ALGORITHM PERFORMANCE COMPARISON TABLE':^88}")
    print("=" * 88)
    print(f"{'Algorithm':<22} | {'Reached':<8} | {'Path Steps':<12} | {'Path Cost':<12} | {'Expanded Nodes':<16} | {'Time (ms)':<10}")
    print("-" * 88)
    for res in all_results:
        reached_str = "Yes" if res.success else "No"
        print(f"{res.name:<22} | {reached_str:<8} | {res.path_length:<12} | {res.path_cost:<12.2f} | {res.nodes_expanded:<16} | {res.execution_time_ms:<10.3f}")
    print("=" * 88)


def run_animated_demo(field):
    """
    Demonstrates each algorithm step-by-step with live animation in sequence,
    followed by the final side-by-side comparison figure!
    """
    print("\n>>> STARTING STEP-BY-STEP ANIMATED DEMONSTRATION <<<")
    print(f"Robot will visit Start -> {len(field.checkpoints)} Crop Checkpoints -> Destination.")
    print("Each algorithm will open an animated window showing search expansion and inspection path.")
    print("When an algorithm finishes, close its window to proceed to the next one.\n")

    results = compute_all_algorithms(field)

    for idx, res in enumerate(results, 1):
        print(f"\n--- [{idx}/{len(results)}] Animating: {res.name} ---")
        print(f"Path Steps: {res.path_length} | Cost: {res.path_cost:.1f} | Nodes Expanded: {res.nodes_expanded}")
        print("Watch the search tree expand and the robot visit the checkpoints in the animation window...")
        animate_search_process(field, res, pause_sec=0.02)

    # Print summary table
    print_comparison_table(results)

    # Show final side-by-side comparison image
    print("\n>>> Showing Final Side-by-Side Comparison Plot of All Algorithms...")
    visualize_all_algorithms(field, results, save_path="simulation_comparison.png", show_plot=True)


def run_single_algorithm_interactive(field):
    """Lets the user select one specific algorithm to animate."""
    waypoints = [field.start] + list(field.checkpoints) + [field.goal]

    print("\nSelect an algorithm to animate step-by-step:")
    print("  [1] Breadth-First Search (BFS)")
    print("  [2] Depth-First Search (DFS)")
    print("  [3] Greedy Best-First Search (GBFS)")
    print("  [4] Standard A* (Crop Safety Disabled)")
    print("  [5] Crop-Safe A* (Crop Buffer Penalty Enabled)")

    sub_choice = input("Enter choice (1-5): ").strip()

    if sub_choice == "1":
        res = plan_mission_path(field, waypoints, bfs)
    elif sub_choice == "2":
        res = plan_mission_path(field, waypoints, dfs)
    elif sub_choice == "3":
        res = plan_mission_path(field, waypoints, greedy_best_first_search)
    elif sub_choice == "4":
        res = plan_mission_path(field, waypoints, a_star_search, use_crop_safety=False, algorithm_label="A* (Standard)")
    elif sub_choice == "5":
        res = plan_mission_path(field, waypoints, a_star_search, use_crop_safety=True, algorithm_label="A* (Crop-Safe)")
    else:
        print("Invalid selection.")
        return

    print(f"\n--- Animating {res.name} ---")
    print(res)
    animate_search_process(field, res, pause_sec=0.03)


def run_replanning_animation(field):
    """Simulates and animates Task 5: Online obstacle detection & replanning."""
    print("\n" + "=" * 70)
    print(" TASK 5: SENSOR-BASED OBSTACLE DETECTION & ONLINE A* REPLANNING")
    print("=" * 70)
    print(f"Unknown Obstacles placed in field: {field.unknown_obstacles}")
    print(f"Checkpoints to inspect: {field.checkpoints}")
    print("The robot starts with no prior knowledge of these dynamic obstacles.")
    print("Watch the robot move cell-by-cell with its active sensor radar.\n")

    sim_result = simulate_robot_journey(field, sensor_range=1, use_crop_safety=True)

    print("-" * 70)
    print(f"REPLANNING EXPERIMENT RESULTS:")
    print(f" - Destination Reached        : {'Yes' if sim_result.destination_reached else 'No'}")
    print(f" - Checkpoints Inspected      : {sim_result.visited_checkpoints}")
    print(f" - Total Physical Steps Taken : {sim_result.total_steps}")
    print(f" - Total Trajectory Cost      : {sim_result.total_cost:.2f}")
    print(f" - Replanning Operations Count: {sim_result.replan_count}")
    print(f" - Detected Obstacle Location : {sim_result.detected_obstacles}")
    print("-" * 70)

    print("\n>>> Launching Step-by-Step Live Journey Animation...")
    animate_replanning_journey(field, sim_result, pause_sec=0.12, save_path="replanning_simulation.png")


def parse_checkpoints_input(input_str):
    """Parses user string of form 'r1 c1, r2 c2' into list of tuples [(r1, c1), (r2, c2)]."""
    cps = []
    if not input_str.strip():
        return None
    raw_pairs = input_str.split(",")
    for p in raw_pairs:
        tokens = p.strip().split()
        if len(tokens) == 2:
            cps.append((int(tokens[0]), int(tokens[1])))
    return cps


def interactive_menu():
    """Main interactive terminal interface."""
    print_banner()

    # Load default field initially
    current_field = FieldGrid.create_default_demo_field()

    while True:
        print("\n" + "=" * 62)
        print("                         MAIN MENU")
        print("=" * 62)
        print("  [1] Run Full Animated Demo (Visiting Crop Checkpoints -> Final Comparison)")
        print("  [2] Select a Specific Algorithm to Animate (BFS / DFS / GBFS / A*)")
        print("  [3] Animate Task 5 Online Obstacle Detection & Replanning")
        print("  [4] Configure Farmer Field (Custom Size, Obstacles & Checkpoints)")
        print("  [5] Quick Benchmark Mode (Prints Table & Saves Plots without animation)")
        print("  [0] Exit")
        print("=" * 62)

        choice = input("\nEnter your choice (0-5): ").strip()

        if choice == "1":
            print_ascii_grid(current_field, show_unknown=False)
            run_animated_demo(current_field)
            run_replanning_animation(current_field)

        elif choice == "2":
            print_ascii_grid(current_field, show_unknown=False)
            run_single_algorithm_interactive(current_field)

        elif choice == "3":
            print_ascii_grid(current_field, show_unknown=False)
            run_replanning_animation(current_field)

        elif choice == "4":
            print("\n>>> Farmer Custom Potato Field Configuration <<<")
            try:
                rows = int(input("Enter number of rows (e.g. 10 to 25) [Default 12]: ") or "12")
                cols = int(input("Enter number of columns (e.g. 12 to 30) [Default 16]: ") or "16")
                num_obstacles = int(input("Enter number of static obstacles [Default 6]: ") or "6")

                s_input = input("Enter start as 'row col' or press Enter for (0, 0): ").strip()
                start = tuple(map(int, s_input.split())) if s_input else (0, 0)

                g_input = input(f"Enter goal as 'row col' or press Enter for ({rows-1}, {cols-1}): ").strip()
                goal = tuple(map(int, g_input.split())) if g_input else (rows - 1, cols - 1)

                print("\n--- Crop Inspection Checkpoints Setup ---")
                print("Enter coordinates of monitoring stations for soil/crop inspection.")
                print("Format: 'row col, row col' (e.g. '3 4, 6 11')")
                cp_input = input("Enter checkpoints or press Enter for optimal recommended stations: ").strip()
                parsed_cps = parse_checkpoints_input(cp_input)

                current_field = FieldGrid.create_custom_field(
                    rows=rows, 
                    cols=cols, 
                    num_obstacles=num_obstacles, 
                    start=start, 
                    goal=goal,
                    checkpoints=parsed_cps
                )

                if current_field.checkpoints:
                    opt_q = input(f"Optimize checkpoint visiting order for minimum battery/distance? (y/n) [Default y]: ").strip().lower()
                    if opt_q != "n":
                        optimized_tour = optimize_farmer_checkpoint_tour(
                            current_field.start, 
                            current_field.checkpoints, 
                            current_field.goal
                        )
                        current_field.checkpoints = optimized_tour
                        print(f"Optimal Farmer Inspection Tour: {current_field.checkpoints}")

                print_ascii_grid(current_field, show_unknown=False)

                anim_opt = input("Run live animations on this custom field? (y/n) [Default y]: ").strip().lower()
                if anim_opt != "n":
                    run_animated_demo(current_field)
                else:
                    results = compute_all_algorithms(current_field)
                    print_comparison_table(results)
                    visualize_all_algorithms(current_field, results, save_path="simulation_comparison.png", show_plot=True)

            except ValueError as e:
                print(f"Invalid input: {e}. Please enter integer numbers.")

        elif choice == "5":
            print("\n>>> Running Quick Benchmark Mode (No Animation Delays) <<<")
            print_ascii_grid(current_field, show_unknown=False)
            results = compute_all_algorithms(current_field)
            print_comparison_table(results)
            visualize_all_algorithms(current_field, results, save_path="simulation_comparison.png", show_plot=True)
            run_replanning_animation(current_field)

        elif choice == "0":
            print("Exiting program. Good luck with your project presentation!")
            sys.exit(0)
        else:
            print("Invalid option. Please enter a number between 0 and 5.")

if __name__ == "__main__":
    interactive_menu()
