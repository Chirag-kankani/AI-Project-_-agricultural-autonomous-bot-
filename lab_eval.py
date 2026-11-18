"""
lab_eval.py - Cryptarithmetic Puzzle Solver (SEND + MORE = MONEY)
Constraint Satisfaction Problem (CSP) Approach
Features: Backtracking Search, Forward Checking, AC-3 Constraint Propagation, MRV, and LCV.
Author: Chirag Kankani
"""

import time
import copy

# =====================================================================
# 1. CSP Definition: Variables, Domains, and Constraints
# =====================================================================

VARIABLES = ['S', 'E', 'N', 'D', 'M', 'O', 'R', 'Y']

def get_initial_domains():
    """
    Returns initial domains for all variables:
    - S, M in {1..9} because leading digits cannot be zero.
    - E, N, D, O, R, Y in {0..9}.
    """
    domains = {}
    for var in VARIABLES:
        if var in ['S', 'M']:
            domains[var] = list(range(1, 10)) # Leading zeros forbidden
        else:
            domains[var] = list(range(0, 10))
    return domains


def check_partial_arithmetic(assignment):
    """
    Verifies column-wise arithmetic constraints as soon as constituent
    variables are assigned. Enables early pruning before full assignment.
    Columns (from right to left):
      Col 0 (Units):     D + E = Y + 10 * C1
      Col 1 (Tens):      N + R + C1 = E + 10 * C2
      Col 2 (Hundreds):  E + O + C2 = N + 10 * C3
      Col 3 (Thousands): S + M + C3 = O + 10 * C4
      Col 4 (Ten-Thous): C4 = M
    """
    # Leading digit non-zero check
    if assignment.get('S') == 0 or assignment.get('M') == 0:
        return False

    # Col 0: Units column (D, E, Y)
    if 'D' in assignment and 'E' in assignment and 'Y' in assignment:
        if (assignment['D'] + assignment['E']) % 10 != assignment['Y']:
            return False

    # Col 1: Tens column (N, R, E) - requires C1 from Col 0
    if 'D' in assignment and 'E' in assignment and 'N' in assignment and 'R' in assignment:
        c1 = (assignment['D'] + assignment['E']) // 10
        if (assignment['N'] + assignment['R'] + c1) % 10 != assignment['E']:
            return False

    # Col 2: Hundreds column (E, O, N) - requires C2 from Col 1
    if 'D' in assignment and 'E' in assignment and 'N' in assignment and 'R' in assignment and 'O' in assignment:
        c1 = (assignment['D'] + assignment['E']) // 10
        c2 = (assignment['N'] + assignment['R'] + c1) // 10
        if (assignment['E'] + assignment['O'] + c2) % 10 != assignment['N']:
            return False

    # Col 3 & 4: Thousands & Ten-thousands (S, M, O) - requires C3 from Col 2
    if ('D' in assignment and 'E' in assignment and 'N' in assignment and 'R' in assignment 
            and 'O' in assignment and 'S' in assignment and 'M' in assignment):
        c1 = (assignment['D'] + assignment['E']) // 10
        c2 = (assignment['N'] + assignment['R'] + c1) // 10
        c3 = (assignment['E'] + assignment['O'] + c2) // 10
        
        # S + M + C3 = O + 10 * C4
        if (assignment['S'] + assignment['M'] + c3) % 10 != assignment['O']:
            return False
        c4 = (assignment['S'] + assignment['M'] + c3) // 10
        # Ten-thousands column: C4 must equal M
        if c4 != assignment['M']:
            return False

    return True


def verify_solution(assignment):
    """Verifies that all 8 variables are assigned uniquely and satisfy SEND + MORE = MONEY."""
    if len(assignment) != 8:
        return False
    if len(set(assignment.values())) != 8:
        return False
    
    s = assignment['S']
    e = assignment['E']
    n = assignment['N']
    d = assignment['D']
    m = assignment['M']
    o = assignment['O']
    r = assignment['R']
    y = assignment['Y']

    if s == 0 or m == 0:
        return False

    send = 1000 * s + 100 * e + 10 * n + d
    more = 1000 * m + 100 * o + 10 * r + e
    money = 10000 * m + 1000 * o + 100 * n + 10 * e + y

    return send + more == money


# =====================================================================
# 2. Constraint Propagation: AC-3 Algorithm & Structural Pruning
# =====================================================================

def ac3_constraint_propagation(domains):
    """
    Applies Constraint Propagation (AC-3 style) to eliminate inconsistent values:
    1. Structural Cryptarithmetic Deduction:
       - SEND < 10000 and MORE < 10000 => SEND + MORE < 20000.
       - Since MONEY >= 10000, M must be 1 (leading carry = 1).
       - Therefore, Domain(M) = {1}.
    2. Arc Consistency on AllDifferent:
       - Since M = 1, value 1 is removed from all other 7 variables.
    3. Structural Deduction on Thousands column:
       - S + M + C3 = 10 * M + O => S + 1 + C3 = 10 + O => S + C3 = 9 + O.
       - Since S <= 9 and M=1 (so O != 1), O must be 0!
       - Therefore, Domain(O) = {0}.
       - Value 0 is removed from all other variables.
    4. S bound deduction:
       - S + C3 = 9 => S in {8, 9}.
    Returns the reduced, arc-consistent domains.
    """
    pruned_domains = {var: list(vals) for var, vals in domains.items()}

    # Deduce M = 1
    pruned_domains['M'] = [1]
    for var in pruned_domains:
        if var != 'M' and 1 in pruned_domains[var]:
            pruned_domains[var].remove(1)

    # Deduce O = 0
    pruned_domains['O'] = [0]
    for var in pruned_domains:
        if var != 'O' and 0 in pruned_domains[var]:
            pruned_domains[var].remove(0)

    # Deduce S in {8, 9}
    pruned_domains['S'] = [val for val in pruned_domains['S'] if val in [8, 9]]

    return pruned_domains


# =====================================================================
# 3. Heuristics: MRV and LCV
# =====================================================================

def select_unassigned_variable_mrv(assignment, domains, var_order):
    """
    Minimum Remaining Values (MRV) Heuristic:
    Selects the unassigned variable with the fewest remaining legal values in its domain.
    Tie-breaking: Uses the original column-wise order.
    """
    unassigned = [v for v in var_order if v not in assignment]
    return min(unassigned, key=lambda v: len(domains[v]))


def order_domain_values_lcv(var, assignment, domains):
    """
    Least Constraining Value (LCV) Heuristic:
    Orders domain values in ascending order of how many choices they eliminate
    from the domains of the remaining unassigned variables.
    Prefers values that leave the maximum flexibility for other variables.
    """
    def count_conflicts(val):
        conflicts = 0
        for other_var in VARIABLES:
            if other_var not in assignment and other_var != var:
                if val in domains[other_var]:
                    conflicts += 1
        return conflicts

    return sorted(domains[var], key=count_conflicts)


# =====================================================================
# 4. Forward Checking (FC)
# =====================================================================

def forward_check(var, val, assignment, domains):
    """
    Forward Checking:
    When var is assigned val, removes val from the domains of all other
    unassigned variables (AllDifferent constraint).
    Returns (pruned_domains, is_consistent).
    If any unassigned variable's domain becomes empty, returns (None, False).
    """
    new_domains = {}
    for other_var, other_vals in domains.items():
        if other_var not in assignment and other_var != var:
            pruned = [v for v in other_vals if v != val]
            if len(pruned) == 0:
                return None, False # Domain wipeout! Prune branch early.
            new_domains[other_var] = pruned
        else:
            new_domains[other_var] = list(other_vals)
    return new_domains, True


# =====================================================================
# 5. Core CSP Solver Engine
# =====================================================================

class CSPSolverMetrics:
    """Tracks performance metrics of a search run."""
    def __init__(self, name):
        self.name = name
        self.nodes_explored = 0
        self.backtracks = 0
        self.execution_time_ms = 0.0
        self.assignment = None
        self.verified = False


def solve_cryptarithmetic(use_fc=False, use_mrv=False, use_lcv=False, use_ac3=False, name="Solver"):
    """
    Solves SEND + MORE = MONEY under specific heuristic and inference configurations.
    - use_fc: Forward Checking
    - use_mrv: Minimum Remaining Values variable selection
    - use_lcv: Least Constraining Value value ordering
    - use_ac3: Pre-search AC-3 Constraint Propagation
    """
    metrics = CSPSolverMetrics(name)
    domains = get_initial_domains()

    # Step A: Apply Constraint Propagation if requested
    if use_ac3:
        domains = ac3_constraint_propagation(domains)

    # Column-wise variable ordering: Units (D, E, Y) -> Tens (N, R) -> Hundreds (O) -> Thousands (S, M)
    column_var_order = ['D', 'E', 'Y', 'N', 'R', 'O', 'S', 'M']

    start_time = time.perf_counter()

    def backtrack(assignment, cur_domains):
        metrics.nodes_explored += 1

        # Base case: All variables assigned
        if len(assignment) == len(VARIABLES):
            if verify_solution(assignment):
                return assignment
            return None

        # 1. Variable Selection: MRV vs. Column Order
        if use_mrv:
            var = select_unassigned_variable_mrv(assignment, cur_domains, column_var_order)
        else:
            var = next(v for v in column_var_order if v not in assignment)

        # 2. Value Ordering: LCV vs. Ascending Domain Order
        if use_lcv:
            values = order_domain_values_lcv(var, assignment, cur_domains)
        else:
            values = cur_domains[var]

        # 3. Try each candidate value
        for val in values:
            # AllDifferent constraint check
            if val in assignment.values():
                continue

            # Tentative assignment
            new_assignment = dict(assignment)
            new_assignment[var] = val

            # Arithmetic column consistency check
            if not check_partial_arithmetic(new_assignment):
                continue

            # 4. Inference: Forward Checking
            if use_fc:
                pruned_domains, success = forward_check(var, val, new_assignment, cur_domains)
                if not success:
                    metrics.backtracks += 1
                    continue
            else:
                pruned_domains = cur_domains

            # Recursive search call
            result = backtrack(new_assignment, pruned_domains)
            if result is not None:
                return result

            metrics.backtracks += 1

        return None

    solution = backtrack({}, domains)
    elapsed_ms = (time.perf_counter() - start_time) * 1000.0

    metrics.execution_time_ms = elapsed_ms
    metrics.assignment = solution
    metrics.verified = verify_solution(solution) if solution else False

    return metrics


# =====================================================================
# 6. Display & Comparison Helpers
# =====================================================================

def display_arithmetic_puzzle(assignment):
    """Prints a beautiful, formatted display of the puzzle and its arithmetic verification."""
    s = assignment['S']
    e = assignment['E']
    n = assignment['N']
    d = assignment['D']
    m = assignment['M']
    o = assignment['O']
    r = assignment['R']
    y = assignment['Y']

    send = 1000 * s + 100 * e + 10 * n + d
    more = 1000 * m + 100 * o + 10 * r + e
    money = 10000 * m + 1000 * o + 100 * n + 10 * e + y

    print("\n" + "=" * 55)
    print("      CRYPTARITHMETIC PUZZLE ARITHMETIC VERIFICATION")
    print("=" * 55)
    print(" Letter Assignment Mapping:")
    mapping_str = ", ".join([f"{k} = {v}" for k, v in sorted(assignment.items())])
    print(f" {mapping_str}\n")
    print(" Vertical Arithmetic Column Alignment:")
    print(f"       S E N D   -->     {s} {e} {n} {d}   (SEND  = {send})")
    print(f"     + M O R E   -->   + {m} {o} {r} {e}   (MORE  = {more})")
    print(f"     ---------         ---------")
    print(f"     M O N E Y   -->   {m} {o} {n} {e} {y}   (MONEY = {money})")
    print("=" * 55)
    print(f" Verification Equation: {send} + {more} == {money}")
    if send + more == money:
        print(" Result Status: [VERIFIED CORRECT] Satisfies all constraints!")
    else:
        print(" Result Status: [FAILED]")
    print("=" * 55 + "\n")


def run_all_experiments():
    """Runs all 5 configurations and outputs a comparative evaluation table."""
    print("=" * 80)
    print("   ARTIFICIAL INTELLIGENCE LAB ASSESSMENT: CRYPTARITHMETIC CSP SOLVER")
    print("   Problem: SEND + MORE = MONEY")
    print("   Heuristics: MRV, LCV | Inference: Forward Checking (FC), AC-3 Propagation")
    print("=" * 80)

    configs = [
        ("1. Pure Backtracking (Baseline)", False, False, False, False),
        ("2. Backtracking + Forward Checking", True, False, False, False),
        ("3. Backtracking + FC + MRV", True, True, False, False),
        ("4. Backtracking + FC + MRV + LCV", True, True, True, False),
        ("5. AC-3 Constraint Propagation + FC + MRV + LCV", True, True, True, True)
    ]

    all_metrics = []

    print("\nExecuting Search Algorithms...\n")
    for label, fc, mrv, lcv, ac3 in configs:
        metric = solve_cryptarithmetic(use_fc=fc, use_mrv=mrv, use_lcv=lcv, use_ac3=ac3, name=label)
        all_metrics.append(metric)

    # Print Comparison Table
    print("=" * 90)
    print(f"{'ALGORITHM PERFORMANCE COMPARISON TABLE':^90}")
    print("=" * 90)
    print(f"{'Algorithm Configuration':<46} | {'Nodes Explored':<15} | {'Backtracks':<10} | {'Time (ms)':<10}")
    print("-" * 90)
    for m in all_metrics:
        print(f"{m.name:<46} | {m.nodes_explored:<15} | {m.backtracks:<10} | {m.execution_time_ms:<10.3f}")
    print("=" * 90)

    # Display final verified arithmetic solution
    display_arithmetic_puzzle(all_metrics[-1].assignment)

    # Summary commentary for viva
    print("KEY CONCEPTS & OBSERVATIONS FOR VIVA EVALUATION:")
    print(" 1. Variables & Domains: 8 variables (S, E, N, D, M, O, R, Y). S, M in {1..9}; others in {0..9}.")
    print(" 2. AllDifferent: Enforced across all 8 letters so no digit is assigned twice.")
    print(" 3. Forward Checking (FC): Whenever a digit is assigned, it is instantly pruned from remaining")
    print("    unassigned variables. If any domain wipes out, the branch is immediately pruned.")
    print(" 4. Minimum Remaining Values (MRV): Prioritizes the most constrained variable (fewest legal choices)")
    print("    which detects failures earliest (Fail-First Principle).")
    print(" 5. Least Constraining Value (LCV): Orders values by which rules out the fewest choices for neighbors,")
    print("    maximizing the chances of finding a solution quickly without dead ends.")
    print(" 6. AC-3 / Constraint Propagation: Deduces M=1, O=0, and S in {8, 9} before search even begins,")
    print("    drastically shrinking the state space!\n")

if __name__ == "__main__":
    run_all_experiments()
