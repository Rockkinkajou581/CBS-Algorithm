"""
Benchmark CBS on random 32x32 grids to find how many agents it solves, and how fast.

Sweeps agent count upward, running several random instances at each count, and
reports the success rate within a wall-clock budget plus the solve-time spread.

    python3 bench.py                     # defaults: 32x32, 20% obstacles, 20 trials
    python3 bench.py --trials 30 --timeout 30

Solve times are reported only over instances that actually solved; the success
rate is reported separately, because CBS runtime is heavy tailed and averaging a
timeout in as though it were a measurement would be dishonest.
"""

import argparse
import random
import signal
import statistics
import time
from collections import deque

from mapf.grid import Grid
from mapf.high_level import conflict_based_search


class Timeout(Exception):
    pass


def _alarm(signum, frame):
    raise Timeout()


def largest_component(grid):
    """Free cells of the biggest connected region, so every instance is reachable."""
    seen, best = set(), []
    for x in range(grid.width):
        for y in range(grid.height):
            start = (x, y)
            if start in seen or not grid.is_free(start):
                continue
            comp, q = [], deque([start])
            seen.add(start)
            while q:
                cell = q.popleft()
                comp.append(cell)
                for nb in grid.neighbors(cell):
                    if nb not in seen:
                        seen.add(nb)
                        q.append(nb)
            if len(comp) > len(best):
                best = comp
    return best


def make_instance(rng, size, density, num_agents):
    """Random grid + random start/goal per agent, all inside one connected region."""
    cells = [(x, y) for x in range(size) for y in range(size)]
    obstacles = frozenset(rng.sample(cells, int(density * len(cells))))
    grid = Grid(width=size, height=size, obstacles=obstacles)

    free = largest_component(grid)
    if len(free) < 2 * num_agents:
        return None, None

    starts = rng.sample(free, num_agents)
    goals = rng.sample(free, num_agents)
    return grid, {i: (starts[i], goals[i]) for i in range(num_agents)}


def run_trial(grid, agents, timeout):
    """Returns (outcome, seconds). Outcome is solved / timeout / unsolvable / error."""
    signal.signal(signal.SIGALRM, _alarm)
    signal.setitimer(signal.ITIMER_REAL, timeout)
    t0 = time.perf_counter()
    try:
        solution = conflict_based_search(grid, agents)
        elapsed = time.perf_counter() - t0
        return ("solved" if solution is not None else "unsolvable"), elapsed
    except Timeout:
        return "timeout", timeout
    except Exception as exc:
        return f"error: {type(exc).__name__}: {exc}", time.perf_counter() - t0
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)


def verify(grid, agents, solution):
    """Independent check: right endpoints, legal moves, no vertex or edge conflict."""
    for agent, (start, goal) in agents.items():
        path = solution[agent]
        if path[0] != start or path[-1] != goal:
            return False
        for a, b in zip(path, path[1:]):
            if b != a and b not in grid.neighbors(a):
                return False

    horizon = max(len(p) for p in solution.values())
    at = lambda p, t: p[t] if t < len(p) else p[-1]
    for t in range(horizon):
        seen = {}
        for agent, path in solution.items():
            cell = at(path, t)
            if cell in seen:
                return False
            seen[cell] = agent
        for a1, p1 in solution.items():
            for a2, p2 in solution.items():
                if a1 < a2 and at(p1, t) == at(p2, t + 1) and at(p2, t) == at(p1, t + 1):
                    return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=int, default=32)
    ap.add_argument("--density", type=float, default=0.20, help="obstacle fraction")
    ap.add_argument("--trials", type=int, default=20, help="instances per agent count")
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds per instance")
    ap.add_argument("--start", type=int, default=2)
    ap.add_argument("--step", type=int, default=2)
    ap.add_argument("--max-agents", type=int, default=40)
    ap.add_argument("--stop-below", type=float, default=0.80,
                    help="halt the sweep once success rate drops under this")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    print(f"{args.size}x{args.size} grid, {args.density:.0%} obstacles, "
          f"{args.trials} instances per agent count, {args.timeout:g}s budget, seed {args.seed}\n")
    print(f"{'agents':>7} {'solved':>8} {'median':>9} {'p95':>9} {'max':>9}  {'notes':<24}")
    print("-" * 72)

    last_good = None
    for n in range(args.start, args.max_agents + 1, args.step):
        rng = random.Random(args.seed + n * 1000)
        times, outcomes, bad = [], [], 0

        for _ in range(args.trials):
            grid, agents = make_instance(rng, args.size, args.density, n)
            if grid is None:
                continue
            outcome, elapsed = run_trial(grid, agents, args.timeout)
            outcomes.append(outcome)
            if outcome == "solved":
                times.append(elapsed)
                sol = conflict_based_search(grid, agents)
                if not verify(grid, agents, sol):
                    bad += 1

        solved = outcomes.count("solved")
        total = len(outcomes)
        rate = solved / total if total else 0.0

        note = []
        if any(o.startswith("error") for o in outcomes):
            note.append(next(o for o in outcomes if o.startswith("error"))[:20])
        if outcomes.count("unsolvable"):
            note.append(f"{outcomes.count('unsolvable')} unsolvable")
        if bad:
            note.append(f"{bad} INVALID")

        if times:
            p95 = sorted(times)[min(len(times) - 1, int(0.95 * len(times)))]
            print(f"{n:>7} {solved:>3}/{total:<4} {statistics.median(times):>8.3f}s "
                  f"{p95:>8.3f}s {max(times):>8.3f}s  {', '.join(note):<24}")
        else:
            print(f"{n:>7} {solved:>3}/{total:<4} {'—':>9} {'—':>9} {'—':>9}  {', '.join(note):<24}")

        if rate >= args.stop_below and times:
            last_good = (n, rate, statistics.median(times),
                         sorted(times)[min(len(times) - 1, int(0.95 * len(times)))])
        else:
            print(f"\nsuccess rate {rate:.0%} fell below {args.stop_below:.0%}, stopping sweep.")
            break

    if last_good:
        n, rate, med, p95 = last_good
        print(f"\nLargest agent count solving >={args.stop_below:.0%} of instances: {n}")
        print(f"  median {med:.2f}s, p95 {p95:.2f}s, {rate:.0%} of {args.trials} random instances")
        fmt = lambda v: f"{v:.2f}" if v >= 0.1 else f"{v:.3f}"
        print(f"\n  N = {n}, X = {fmt(p95)}  (p95 — the defensible 'under X seconds')")
        print(f"  or X = {fmt(med)} if you quote the median instead")
    else:
        print("\nNo agent count met the success threshold. Loosen --timeout or --density.")


if __name__ == "__main__":
    main()
