# Comprehensive Project Documentation
## AI-Based Crop-Safe Path Planning for an Autonomous Potato-Field Robot

---

## 1. Project Overview & Background

In precision agriculture, autonomous agricultural field robots are deployed for monitoring crop health, soil moisture assessment, canopy disease detection, and selective treatment. In a potato field, the robot must navigate between crop rows, **visit selected monitoring locations (inspection checkpoints)**, and avoid damaging potato plants.

Furthermore, real agricultural fields contain physical hazards such as stones, tools, and irrigation equipment. Some hazards are known prior to departure, while others appear unpredictably and must be detected online using onboard sensors.

This project implements an intelligent path-planning agent using core Artificial Intelligence search paradigms:
1. **Breadth-First Search (BFS)** - Uninformed Search
2. **Depth-First Search (DFS)** - Uninformed Search
3. **Greedy Best-First Search (GBFS)** - Informed Search
4. **A\* Search** - Informed Search with Crop-Safe Cost Optimization
5. **Multi-Checkpoint Agricultural Mission Planning & Farmer Tour Optimization** - Guiding the robot through all farmer-designated monitoring stations.
6. **Dynamic Obstacle Detection & Online A\* Replanning** - Online execution with local sensor simulation.

---

## 2. Formal AI Problem Formulation (Task 2)

According to Russell & Norvig's Artificial Intelligence framework, the path planning task is formally formulated as follows:

### 2.1 State Space ($S$)
- The field is modeled as a 2D discrete grid of size $M \times N$, where each cell is indexed by coordinates $(r, c)$ such that:
  $$S = \{ (r, c) \mid 0 \le r < M,\; 0 \le c < N \}$$
- Each state represents the robot's physical grid location in the potato field.

### 2.2 Initial State ($S_0$) and Monitoring Checkpoints ($\mathcal{C}$)
- The designated deployment location: $S_0 = (r_{\text{start}}, c_{\text{start}}) \in S$.
- The farmer's crop monitoring checkpoints:
  $$\mathcal{C} = \{ C_1, C_2, \dots, C_K \} \subset S$$
  representing specific field inspection stations (e.g., soil moisture probes, canopy cameras, pest traps).
- In the default demonstration field:
  - $S_0 = (1, 1)$
  - $C_1 = (3, 4)$ (Soil Moisture & Root Sensor Station Alpha)
  - $C_2 = (6, 11)$ (Canopy Blight Inspection Camera Station Beta)
  - $S_{\text{goal}} = (10, 14)$ (Farm shed / charging depot)

### 2.3 Actions ($A(s)$)
- The robot moves orthogonally across the grid:
  $$A(s) \subseteq \{ \text{RIGHT } (0, 1),\; \text{DOWN } (1, 0),\; \text{UP } (-1, 0),\; \text{LEFT } (0, -1) \}$$
- Diagonal movements are forbidden to prevent wheel clipping of potato root zones.

### 2.4 Multi-Checkpoint Mission Goal Test
- The mission goal is satisfied when all checkpoints in $\mathcal{C}$ have been visited in sequence and the robot reaches $S_{\text{goal}}$:
  $$\text{IsMissionComplete}(s, \mathcal{C}_{\text{remaining}}) \iff s = S_{\text{goal}} \land \mathcal{C}_{\text{remaining}} = \emptyset$$

### 2.5 Farmer Tour Optimization (Agricultural TSP)
- To conserve robot battery and minimize traversal time, the visiting order of arbitrary farmer-specified checkpoints is optimized:
  $$\text{Tour}^* = \arg\min_{\pi \in \text{Perm}(\mathcal{C})} \left[ \text{dist}(S_0, \pi_1) + \sum_{i=1}^{K-1} \text{dist}(\pi_i, \pi_{i+1}) + \text{dist}(\pi_K, S_{\text{goal}}) \right]$$

### 2.6 Path-Cost Function ($c(s, a, s')$)
- Unlike simple shortest-path problems where every step has unit cost $1$, agricultural navigation must prioritize crop safety:
  $$c(s, a, s') = \text{base\_step\_cost} + \text{safety\_penalty}(s')$$
  - **Open Safe Alley**: $\text{base\_step\_cost} = 1.0$, $\text{safety\_penalty} = 0.0 \implies \text{Cost} = 1.0$
  - **Crop-Protection Buffer Zone**: $\text{base\_step\_cost} = 1.0$, $\text{safety\_penalty} = 4.0 \implies \text{Cost} = 5.0$
  - **Potato Plant / Obstacle**: $\text{Cost} = \infty$ (Strictly forbidden).

---

## 3. Field Environment & Cell Taxonomy (Task 1 & Task 4)

The field environment is structured into five distinct cell types:

| Cell ID | Name | Color in Visualizer | Passable? | Step Cost | Description |
| :---: | :--- | :--- | :---: | :---: | :--- |
| **0** | **Safe Alley** | White (`#FFFFFF`) | **Yes** | $1.0$ | Open driving corridors between crop beds and headlands. |
| **1** | **Potato Crop Row** | Dark Green (`#2E7D32`) | **No** | $\infty$ | Actual potato crop beds. Never traversable by the robot. |
| **2** | **Crop Buffer Zone** | Amber / Yellow (`#FFF3CD`) | **Yes** | $5.0$ | Sensitivity perimeter around potato rows. Passable only when necessary. |
| **3** | **Static Obstacle** | Dark Charcoal (`#424242`) | **No** | $\infty$ | Known physical hazards: large stones, irrigation hydrants, toolboxes. |
| **4** | **Unknown Obstacle** | Bright Red (`#D32F2F`) | **Dynamic** | $\infty$ | Hazard absent from initial map, discovered during transit via sensors. |

### Crop Protection Logic (Task 4)
Potato plants have shallow root architectures and delicate leaf canopies. Driving immediately adjacent to a crop row risks soil compaction and canopy clipping. Therefore:
1. Every cell adjacent (8-connected neighborhood) to a `Potato Crop Row` is designated as a `Crop Buffer Zone`.
2. A safety penalty is added to the transition cost of entering buffer cells.
3. As demonstrated in our benchmark, **Standard A\*** ignores this penalty and takes risky tight shortcuts, whereas **Crop-Safe A\*** takes open perimeter alleys to preserve crop integrity.

---

## 4. Search Algorithms Implemented (Task 3)

### 4.1 Breadth-First Search (BFS)
- **Strategy**: Explores the search space level-by-level in expanding concentric rings.
- **Data Structure**: First-In-First-Out (FIFO) queue (`collections.deque`).
- **Completeness**: Yes (finite state space).
- **Optimality**: Finds the path with the minimum number of steps (unweighted shortest path). However, it does not minimize path cost when cell traversal costs vary.
- **Time Complexity**: $O(b^d)$, where $b \le 4$ is branching factor and $d$ is depth of goal.
- **Space Complexity**: $O(b^d)$ (must keep all nodes in memory).

### 4.2 Depth-First Search (DFS)
- **Strategy**: Follows a single branch as deeply as possible before backtracking.
- **Data Structure**: Last-In-First-Out (LIFO) stack (`list` with `.pop()`).
- **Completeness**: Yes in finite graph with cycle checking (`visited` set).
- **Optimality**: **Not optimal**. DFS wanders erratically along field borders and frequently generates unnecessarily long paths.
- **Time Complexity**: $O(b^m)$, where $m$ is maximum search depth.
- **Space Complexity**: $O(b \cdot m)$ (linear memory advantage).

### 4.3 Greedy Best-First Search (GBFS)
- **Strategy**: Guided solely by the heuristic estimate $h(n)$ of remaining distance to the goal.
- **Data Structure**: Min-Priority Queue (`heapq`) ordered by $h(n)$.
- **Completeness**: Yes in finite spaces with graph search.
- **Optimality**: **Not optimal**. Because it ignores the accumulated cost $g(n)$, it greedily dives toward the goal coordinates and can get funneled into high-cost buffer zones or obstacle pockets.
- **Performance**: Expands significantly fewer nodes than uninformed search.

### 4.4 A\* Search
- **Strategy**: Combines the actual cost to reach a state $g(n)$ with the estimated remaining cost $h(n)$:
  $$f(n) = g(n) + h(n)$$
  - $g(n)$: Accumulated cost from $S_0$ to node $n$, incorporating crop safety penalties.
  - $h(n)$: Heuristic estimate from node $n$ to $S_{\text{goal}}$.
- **Data Structure**: Min-Priority Queue (`heapq`) ordered by $f(n)$.
- **Completeness**: Yes.
- **Optimality**: Guaranteed to return the lowest-cost path provided that $h(n)$ is **admissible** and **consistent**.

---

## 5. Heuristic Function Design & Theoretical Properties

For grid pathfinding with 4-directional orthogonal movement ($UP, DOWN, LEFT, RIGHT$), we use the **Manhattan Distance**:
$$h((r, c), (r_g, c_g)) = |r - r_g| + |c - c_g|$$

### 5.1 Admissibility Proof
A heuristic $h(n)$ is admissible if it never overestimates the true cost $h^*(n)$ to reach the goal:
$$h(n) \le h^*(n), \quad \forall n$$
- On an obstacle-free grid with unit step cost $1$, the minimum possible steps required to move from $(r, c)$ to $(r_g, c_g)$ without diagonal steps is $|r - r_g| + |c - c_g|$.
- Since obstacles, crop rows, and buffer penalties can only increase the actual cost ($h^*(n) \ge h(n)$), Manhattan distance never overestimates the true cost. Hence, $h(n)$ is strictly **admissible**.

### 5.2 Consistency (Monotonicity) Proof
A heuristic is consistent if for every node $n$ and every neighbor $n'$ generated by action $a$:
$$h(n) \le c(n, a, n') + h(n')$$
- By the triangle inequality of absolute values:
  $$|r - r_g| \le |r - r'| + |r' - r_g|$$
  $$|c - c_g| \le |c - c'| + |c' - c_g|$$
- Summing these gives:
  $$h(n) \le (|r - r'| + |c - c'|) + h(n')$$
- Since a single orthogonal move changes coordinates by exactly $1$ unit, $|r - r'| + |c - c'| = 1$.
- Since the minimum step cost $c(n, a, n') \ge 1.0$:
  $$h(n) \le 1.0 + h(n') \le c(n, a, n') + h(n')$$
- Therefore, the Manhattan heuristic is **consistent**, guaranteeing that $A^*$ will never need to re-open closed nodes.

---

## 6. Obstacle Detection & Online Replanning (Task 5)

In real agricultural operations, fields are dynamic. A tool may be dropped, or an irrigation pipe may burst after the initial route is computed.

### 6.1 Detection Mechanism
1. The robot starts with an internal map that includes static obstacles and crop rows, but lacks knowledge of dynamic/unknown obstacles.
2. The robot computes an initial mission plan using $A^*$.
3. As the robot traverses each step, its simulated proximity sensor scans a radius of $r = 1$ cell in all cardinal directions.
4. When an unknown obstacle enters the sensor horizon, the robot:
   - Registers the obstacle coordinates in its internal map ($S_{\text{known\_obstacles}} \leftarrow S_{\text{known\_obstacles}} \cup \{(r_{\text{obs}}, c_{\text{obs}})\}$)
   - Inspects whether its remaining planned route intersects the detected obstacle.

### 6.2 Replanning Execution
- If the remaining route is unobstructed, the robot continues without delay.
- If the obstacle blocks the planned route, the robot halts at its current position $S_{\text{current}}$ and immediately executes an online $A^*$ replan:
  $$\text{Plan}_{\text{new}} = A^*(S_{\text{current}}, S_{\text{goal}}, \text{Map}_{\text{updated}})$$
- The robot logs the replan event, updates its path, and continues seamlessly to the destination.

### 6.3 Real-Time Visual Animation Engine
To provide intuitive pedagogical demonstrations for classroom and professor evaluation, `visualizer.py` implements a 2-phase live animation engine using Matplotlib:
1. **Search Expansion Phase**: Animates the frontier and visited set in real-time as the algorithm explores nodes across the grid.
2. **Path Tracing Phase**: Once the goal is reached, a moving robot marker traces the discovered path step-by-step from start to goal.
3. **Dynamic Sensor & Replanning Animation**: Shows the robot's physical movement with a live circular sensor radar ($r = 1$). When an unknown hazard is sensed, an alert is triggered, the hazard turns red, the blocked route is marked, and the newly replanned safe path is rendered dynamically.

---

## 7. Assumptions Made in this Project

To ensure academic rigor and clarity, the following explicit assumptions are documented:

1. **Discrete Grid Abstraction**: The continuous agricultural field is discretized into uniform square grid cells ($1 \times 1$ unit per cell).
2. **Deterministic Kinematics**: Actions are deterministic (no wheel slip, dead reckoning errors, or actuator failures).
3. **Orthogonal 4-Connected Movement**: Robot moves in 4 cardinal directions ($N, S, E, W$). Diagonal movement is avoided to prevent cutting corners into crop plants.
4. **Static Crop Rows**: Potato plants do not change position during the mission execution.
5. **Sensor Horizon**: The robot has an omnidirectional short-range proximity sensor with a sensing radius of $1$ grid cell.
6. **Zero-Latency Replanning**: Computational replanning takes negligible time compared to robot mechanical displacement.
7. **Complete Detection Reliability**: The onboard sensor has $100\%$ precision within its sensing bubble (no false positives or misses).

---

## 8. Benchmark Evaluation & Performance Comparison (Task 6)

The following benchmark was gathered from the standardized $12 \times 16$ Potato Field Demonstration environment visiting **all crop monitoring checkpoints** ($C_1=(3, 4)$, $C_2=(6, 11)$) and Destination $(10, 14)$:

| Algorithm | Goal Reached | Path Length (Steps) | Path Cost | Expanded Nodes | Execution Time (ms) | Key Characteristics |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **BFS** | Yes | **22** | 94.00 | 139 | ~0.30 ms | Minimizes step count; expands in uniform concentric breadth wavefront. |
| **DFS** | Yes | 208 | 672.00 | 241 | ~0.50 ms | Suboptimal wander; delves deep down perimeter; extensive backtracking. |
| **Greedy Best-First** | Yes | **22** | 94.00 | **25** | **~0.08 ms** | Fastest node expansion; narrow heuristic-guided beam towards checkpoints. |
| **A\* (Standard)** | Yes | **22** | 22.00 | 35 | ~0.15 ms | Considers step cost = 1; ignores crop buffer safety. |
| **A\* (Crop-Safe)** | Yes | 24 | 88.00 | 194 | ~0.80 ms | **Optimal crop safety: routes through open alleys; avoids crop hazards!** |

### Online Replanning Experiment Results (Task 5)
- **Initial Planned Mission Path**: 24 steps visiting $S_0 \to C_1 \to C_2 \to S_{\text{goal}}$.
- **Checkpoint 1 Inspected**: Robot safely reaches and monitors $C_1(3, 4)$.
- **Sensor Detection Point**: While traveling towards Checkpoint 2, robot at $(4, 7)$ senses an unexpected hazard at $(4, 8)$ directly on its corridor.
- **Route Status**: Corridor blocked; A\* online replanning triggered.
- **Replanning Operations**: **1 replan**.
- **Replanned Sub-Path**: 15 steps circumventing the hazard to reach $C_2(6, 11)$ and $S_{\text{goal}}(10, 14)$.
- **Mission Status**: **All checkpoints inspected and successfully reached destination $(10, 14)$!**

---

## 9. Professor / Viva Voce Q&A Guide

Prepare for your evaluation with these direct answers to questions your professor is likely to ask:

### Q1: Why did you choose Manhattan distance over Euclidean distance as your heuristic?
> **Answer**: Because our robot is constrained to orthogonal 4-directional movements (Up, Down, Left, Right). In an orthogonal grid without diagonals, Euclidean straight-line distance underestimates the actual step count, making it less informative. Manhattan distance matches the exact metric of grid movement, making it perfectly admissible and consistent while providing a tighter bound to guide the search faster.

### Q2: How does your project implement "Crop-Safe Planning"?
> **Answer**: We divided the field into safe alleys, impassable potato rows, and a "Crop Buffer Zone" surrounding each crop row. We modified the path-cost function $c(s, a, s')$ by adding a penalty (+4.0) whenever the robot enters a buffer cell. While standard algorithms (BFS, GBFS, Standard A\*) cut right through narrow crop gaps to shave off 2 or 3 steps, our Crop-Safe A\* recognizes that entering buffer zones carries high cost, guiding the robot to take safer, wider open alleys.

### Q3: Why does Greedy Best-First Search expand fewer nodes than A\*, but produce a worse path?
> **Answer**: Greedy Best-First Search orders nodes strictly by $h(n)$ (distance to goal), so it marches straight towards the destination without looking back at the cost it has accumulated. A\* orders nodes by $f(n) = g(n) + h(n)$, which balances the goal distance with the actual cost incurred. Therefore, GBFS is fast but greedy, whereas A\* guarantees the optimal path.

### Q4: How does your obstacle detection and replanning simulation work without real hardware?
> **Answer**: We simulate an onboard proximity sensor with a sensing radius of 1 cell. The robot begins with an initial map where the dynamic obstacle is hidden. As the robot traverses each step of its planned path, the sensor scans surrounding cells. When the unknown obstacle enters the sensor radius, the robot updates its internal belief map and triggers an online A\* search from its current coordinate to the destination.

### Q5: What makes this code modular and not hard-coded?
> **Answer**: The code is separated into modular Python files (`environment.py`, `algorithms.py`, `replanner.py`, `visualizer.py`, and `main.py`). The field dimensions, start and goal coordinates, and obstacle count can all be entered dynamically by the user via the interactive terminal menu. The algorithms accept any general grid environment object implementing `is_passable()` and `get_step_cost()`.
