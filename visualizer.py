"""
visualizer.py - Rich Graphical Visualization and Step-by-Step Live Animation
Supports Agricultural Crop Monitoring Checkpoints, Wavefront Gradient & Online Replanning.
Author: AI Assignment Group
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap, BoundaryNorm

# Color scheme definitions
COLOR_SAFE = "#FFFFFF"           # White: Free alley
COLOR_POTATO = "#2E7D32"         # Forest Green: Potato plant row
COLOR_BUFFER = "#FFF3CD"         # Soft Amber / Yellow: Crop buffer danger zone
COLOR_STATIC_OBS = "#424242"     # Charcoal: Static obstacle (stones/tools)
COLOR_UNKNOWN_OBS = "#D32F2F"    # Crimson: Unknown dynamic obstacle
COLOR_CHECKPOINT = "#00E5FF"     # Cyan: Agricultural crop monitoring checkpoint

CUSTOM_COLORS = [COLOR_SAFE, COLOR_POTATO, COLOR_BUFFER, COLOR_STATIC_OBS, COLOR_UNKNOWN_OBS]
CUSTOM_CMAP = ListedColormap(CUSTOM_COLORS)
CUSTOM_NORM = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5, 4.5], CUSTOM_CMAP.N)

def render_field_base(ax, field, show_unknown=True):
    """Draws the base grid cells with colors, gridlines, Start, Checkpoints, and Destination."""
    grid_matrix = np.zeros((field.rows, field.cols), dtype=int)
    for r in range(field.rows):
        for c in range(field.cols):
            val = field.grid[r][c]
            if not show_unknown and val == 4:
                val = 0
            grid_matrix[r, c] = val

    ax.imshow(grid_matrix, cmap=CUSTOM_CMAP, norm=CUSTOM_NORM, origin="upper")

    # Grid line overlays
    ax.set_xticks(np.arange(-0.5, field.cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, field.rows, 1), minor=True)
    ax.grid(which="minor", color="#CFD8DC", linestyle="-", linewidth=0.7)
    ax.tick_params(which="minor", size=0)

    # Major ticks
    ax.set_xticks(range(0, field.cols, 2))
    ax.set_yticks(range(0, field.rows, 2))
    ax.tick_params(labelsize=8)

    # Mark Start (Green circle) and Goal (Gold star)
    sr, sc = field.start
    gr, gc = field.goal
    ax.scatter(sc, sr, marker="o", color="#00C853", s=180, edgecolors="black", linewidths=1.5, zorder=5, label="Start")
    ax.scatter(gc, gr, marker="*", color="#FFD600", s=250, edgecolors="black", linewidths=1.5, zorder=5, label="Destination")
    ax.text(sc, sr, "S", color="white", fontweight="bold", ha="center", va="center", fontsize=8, zorder=6)
    ax.text(gc, gr, "G", color="black", fontweight="bold", ha="center", va="center", fontsize=8, zorder=6)

    # Mark Crop Monitoring Checkpoints (Cyan diamonds with labels CP1, CP2, ...)
    for idx, (cr, cc) in enumerate(field.checkpoints, 1):
        ax.scatter(cc, cr, marker="D", color=COLOR_CHECKPOINT, s=170, edgecolors="black", linewidths=1.5, zorder=5)
        ax.text(cc, cr, f"{idx}", color="black", fontweight="bold", ha="center", va="center", fontsize=8, zorder=6)


def create_legend_handles(include_explored=True, include_path=True, include_robot=False):
    """Generates standard legend patches for clarity."""
    patches = [
        mpatches.Patch(color=COLOR_POTATO, label="Potato Row (Forbidden)"),
        mpatches.Patch(color=COLOR_BUFFER, label="Crop Buffer Zone (Penalty = +4.0)"),
        mpatches.Patch(color=COLOR_CHECKPOINT, label="Crop Checkpoint (Inspection)"),
        mpatches.Patch(color=COLOR_STATIC_OBS, label="Static Obstacle (Stone/Tool)"),
        mpatches.Patch(color=COLOR_UNKNOWN_OBS, label="Unknown Obstacle (Dynamic)"),
    ]
    if include_explored:
        patches.append(mpatches.Patch(color="#00BCD4", label="Explored (Wavefront Gradient)"))
    if include_path:
        patches.append(mpatches.Patch(color="#1565C0", label="Planned Safe Route"))
    if include_robot:
        patches.append(mpatches.Patch(color="#E91E63", label="Robot Position"))
    return patches


# =====================================================================
# Step-by-Step Animation for a Single Search Algorithm
# =====================================================================

def animate_search_process(field, result, pause_sec=0.02, save_path=None):
    """
    Animates the step-by-step execution of a search algorithm:
    1. Exploration Phase: Shows nodes being visited layer-by-layer.
    2. Path Tracing Phase: Traces the final path through checkpoints to goal.
    """
    fig, ax = plt.subplots(figsize=(10.5, 6.5))
    render_field_base(ax, field, show_unknown=True)

    title_obj = ax.set_title(f"Initializing {result.name} Search...", fontsize=12, fontweight="bold", pad=12)
    handles = create_legend_handles(include_explored=True, include_path=True, include_robot=True)
    ax.legend(handles=handles, loc="upper right", bbox_to_anchor=(1.35, 1.0), fontsize=8.5)

    plt.tight_layout()
    plt.ion()
    plt.show()

    total_visited = len(result.visited_order)
    batch_size = max(1, total_visited // 45)

    explored_c = []
    explored_r = []
    scatter_explored = None

    # --- Phase 1: Search Node Expansion Animation (with Wavefront Gradient) ---
    for i in range(0, total_visited, batch_size):
        chunk = result.visited_order[i:i + batch_size]
        for node in chunk:
            explored_r.append(node[0])
            explored_c.append(node[1])

        if scatter_explored:
            scatter_explored.remove()
        
        node_colors = [idx / max(1, total_visited - 1) for idx in range(len(explored_r))]
        scatter_explored = ax.scatter(explored_c, explored_r, c=node_colors, cmap="cool", 
                                      s=45, alpha=0.75, marker="s", zorder=3)
        
        current_node = chunk[-1]
        title_obj.set_text(f"[{result.name}] Expanding Search Wavefront...\n"
                           f"Current Node: {current_node} | Nodes Expanded: {min(i + batch_size, total_visited)}/{total_visited}")
        fig.canvas.draw_idle()
        plt.pause(pause_sec)

    # --- Phase 2: Final Path Tracing Animation ---
    if result.success and result.path:
        title_obj.set_text(f"[{result.name}] Route Planned! Tracing Inspection Mission...")
        fig.canvas.draw_idle()
        plt.pause(0.3)

        path_c = []
        path_r = []
        robot_scatter = None
        line_obj = None

        cp_set = set(field.checkpoints)

        for idx, (r, c) in enumerate(result.path):
            path_r.append(r)
            path_c.append(c)

            if line_obj:
                line_obj.remove()
            line_obj, = ax.plot(path_c, path_r, color="#1565C0", linewidth=3.5, zorder=4)

            if robot_scatter:
                robot_scatter.remove()
            robot_scatter = ax.scatter(c, r, marker="o", color="#E91E63", s=130, edgecolors="white", linewidths=1.5, zorder=7)

            if (r, c) in cp_set:
                cp_num = field.checkpoints.index((r, c)) + 1
                title_obj.set_text(f"[{result.name}] [CHECKPOINT {cp_num} INSPECTED!]\n"
                                   f"Scanning crop health at ({r}, {c}) | Step {idx}/{result.path_length}")
                fig.canvas.draw_idle()
                plt.pause(0.5)
            else:
                title_obj.set_text(f"[{result.name}] Tracing Mission Route...\n"
                                   f"Step {idx}/{result.path_length} | Robot at ({r}, {c})")
                fig.canvas.draw_idle()
                plt.pause(pause_sec * 2)

        # Final Summary
        title_obj.set_text(
            f"[{result.name}] MISSION COMPLETED!\n"
            f"Steps: {result.path_length} | Cost: {result.path_cost:.1f} | "
            f"Expanded: {result.nodes_expanded} | Time: {result.execution_time_ms:.2f} ms"
        )
    else:
        title_obj.set_text(f"[{result.name}] FAILED: No path found through all checkpoints!")

    fig.canvas.draw_idle()

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"[Animation Snapshot] Saved to {save_path}")

    plt.ioff()
    print(f"\n[Animation Finished] {result.name} - Close the window to proceed.")
    plt.show()


# =====================================================================
# Step-by-Step Animation for Task 5: Dynamic Obstacle & Replanning
# =====================================================================

def animate_replanning_journey(field, sim_result, pause_sec=0.15, save_path="replanning_simulation.png"):
    """
    Animates the physical robot journey with real-time sensor radar detection:
    1. Shows initial planned path through all checkpoints.
    2. Robot moves step-by-step with an expanding sensor bubble.
    3. When the unknown obstacle is detected, triggers an alert, turns obstacle red,
       erases blocked path, draws newly replanned route, and continues safely!
    """
    fig, ax = plt.subplots(figsize=(11, 7))
    render_field_base(ax, field, show_unknown=False)

    title_obj = ax.set_title("Task 5: Initial Planned Route (Robot unaware of hidden obstacle)",
                             fontsize=12, fontweight="bold", pad=12)

    # Draw initial path
    initial_c = [p[1] for p in sim_result.initial_path]
    initial_r = [p[0] for p in sim_result.initial_path]
    init_line, = ax.plot(initial_c, initial_r, color="#90CAF9", linestyle="--", linewidth=2.5, label="Initial Planned Route", zorder=3)

    handles = create_legend_handles(include_explored=False, include_path=True, include_robot=True)
    patches = handles + [
        mpatches.Patch(color="#90CAF9", label="Initial Route (Unaware)"),
        mpatches.Patch(color="#00897B", label="Traversed Path"),
        mpatches.Patch(color="#F4511E", label="Replanned Safe Route"),
    ]
    ax.legend(handles=patches, loc="upper right", bbox_to_anchor=(1.35, 1.0), fontsize=8.5)

    plt.tight_layout()
    plt.ion()
    plt.show()
    plt.pause(0.8)

    traversed_c = []
    traversed_r = []
    traj_line = None
    robot_marker = None
    sensor_patch = None
    replanned_line = None

    replan_steps = {event.step_number: event for event in sim_result.replan_events}
    cp_set = set(field.checkpoints)

    for step_idx, pos in enumerate(sim_result.final_trajectory):
        r, c = pos
        traversed_r.append(r)
        traversed_c.append(c)

        # Update traversed path
        if traj_line:
            traj_line.remove()
        traj_line, = ax.plot(traversed_c, traversed_r, color="#00897B", linewidth=3.5, zorder=4)

        # Update robot position
        if robot_marker:
            robot_marker.remove()
        robot_marker = ax.scatter(c, r, marker="o", color="#E91E63", s=140, edgecolors="white", linewidths=1.8, zorder=8)

        # Draw local sensor radar bubble (1 cell radius)
        if sensor_patch:
            sensor_patch.remove()
        sensor_patch = plt.Circle((c, r), radius=1.3, color="#FF4081", fill=False, linestyle=":", linewidth=1.5, zorder=7)
        ax.add_patch(sensor_patch)

        if (r, c) in cp_set:
            cp_idx = field.checkpoints.index((r, c)) + 1
            title_obj.set_text(f"[CHECKPOINT {cp_idx} INSPECTED!] Robot inspecting potato crops at ({r}, {c})")
            fig.canvas.draw_idle()
            plt.pause(0.5)
        else:
            title_obj.set_text(f"Robot Journey: Step {step_idx}/{sim_result.total_steps} | Position ({r}, {c}) | Sensor Active")
            fig.canvas.draw_idle()
            plt.pause(pause_sec)

        # Check if replan event occurs at this step
        if step_idx in replan_steps:
            event = replan_steps[step_idx]
            obs_r, obs_c = event.obstacle_pos

            # Reveal the obstacle!
            ax.scatter(obs_c, obs_r, marker="s", color=COLOR_UNKNOWN_OBS, s=160, edgecolors="black", linewidths=1.5, zorder=6)
            ax.scatter(c, r, marker="X", color="#D50000", s=220, zorder=9)

            title_obj.set_text(f"[ALERT: OBSTACLE DETECTED at ({obs_r}, {obs_c})!]\n"
                               f"Route Blocked! Initiating Online A* Replanning...")
            fig.canvas.draw_idle()
            plt.pause(1.2)

            # Draw newly replanned route
            new_c = [p[1] for p in event.new_path]
            new_r = [p[0] for p in event.new_path]
            if replanned_line:
                replanned_line.remove()
            replanned_line, = ax.plot(new_c, new_r, color="#F4511E", linestyle="-.", linewidth=3.0, zorder=5)

            title_obj.set_text(f"[REPLAN SUCCESS] Alternative Safe Path Found! Resuming...")
            fig.canvas.draw_idle()
            plt.pause(1.0)

    # Journey completed
    if sensor_patch:
        sensor_patch.remove()

    title_obj.set_text(
        f"[MISSION COMPLETE] All Checkpoints Visited & Reached Destination!\n"
        f"Total Steps: {sim_result.total_steps} | Cost: {sim_result.total_cost:.1f} | Replans: {sim_result.replan_count}"
    )
    fig.canvas.draw_idle()

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"[Replanning Plot Saved] Saved to {save_path}")

    plt.ioff()
    print(f"\n[Replanning Animation Finished] Close the window to proceed.")
    plt.show()


# =====================================================================
# Side-by-Side Final Comparison Plot (Clear, Beautiful & No Text Overlap)
# =====================================================================

def plot_single_path_panel(ax, field, result, line_color="#1E88E5", panel_subtitle=""):
    """Renders a single algorithm's explored search tree with a wavefront gradient and final path."""
    render_field_base(ax, field, show_unknown=True)

    # Plot explored nodes with expansion-order gradient
    if result.visited_order:
        n_visited = len(result.visited_order)
        explored_r = [node[0] for node in result.visited_order]
        explored_c = [node[1] for node in result.visited_order]
        norm_indices = [idx / max(1, n_visited - 1) for idx in range(n_visited)]
        ax.scatter(explored_c, explored_r, c=norm_indices, cmap="cool", s=32, alpha=0.75, marker="s", zorder=3)

    # Plot final path line
    if result.success and result.path:
        path_r = [node[0] for node in result.path]
        path_c = [node[1] for node in result.path]
        ax.plot(path_c, path_r, color=line_color, linewidth=3.0, marker="o", markersize=3.5, zorder=4)

    # 2-Line formatted clean title
    title_text = (f"{result.name} {panel_subtitle}\n"
                  f"Steps: {result.path_length} | Cost: {result.path_cost:.1f} | "
                  f"Expanded: {result.nodes_expanded} | Time: {result.execution_time_ms:.2f} ms")
    ax.set_title(title_text, fontsize=9.5, fontweight="bold", pad=8)


def visualize_all_algorithms(field, results, save_path="simulation_comparison.png", show_plot=True):
    """
    Creates a multi-panel visual figure comparing all search algorithms side-by-side.
    Formatted with spacious subplots to prevent any title or tick label overlap.
    """
    n_algos = len(results)
    cols = min(3, n_algos)
    rows = (n_algos + cols - 1) // cols

    fig, axes = plt.subplots(rows, cols, figsize=(5.8 * cols, 5.2 * rows))
    if n_algos == 1:
        axes = np.array([axes])
    axes = axes.flatten()

    colors = ["#1565C0", "#8E24AA", "#EF6C00", "#00897B", "#D81B60", "#3949AB"]
    subtitles = [
        "(Wavefront Expansion)",
        "(Wandering & Backtrack)",
        "(Heuristic-Guided)",
        "(Unweighted Shortest)",
        "(Crop-Safe Optimal)"
    ]

    for idx, res in enumerate(results):
        ax = axes[idx]
        color = colors[idx % len(colors)]
        sub = subtitles[idx] if idx < len(subtitles) else ""
        plot_single_path_panel(ax, field, res, line_color=color, panel_subtitle=sub)

    for idx in range(n_algos, len(axes)):
        fig.delaxes(axes[idx])

    plt.subplots_adjust(left=0.05, right=0.95, top=0.90, bottom=0.12, wspace=0.22, hspace=0.48)

    handles = create_legend_handles(include_explored=True, include_path=True, include_robot=False)
    fig.legend(handles=handles, loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, 0.02), frameon=True)
    
    cp_info = f"({len(field.checkpoints)} Monitoring Checkpoints Visited)" if field.checkpoints else "(Direct Path)"
    fig.suptitle(f"Potato Field Robot: AI Search Algorithms Comparison {cp_info}", fontsize=14, fontweight="bold", y=0.97)

    if save_path:
        plt.savefig(save_path, dpi=200, bbox_inches="tight")
        print(f"[Visualization Saved] Final comparison plot saved to -> {save_path}")

    if show_plot:
        plt.show()
    plt.close()
