"""
Render a GIF of CBS resolving a single conflict: the root's shortest paths collide,
the tree branches into two constrained children, and one replan comes back clean.

    python3 viz.py            # writes docs/cbs-conflict.gif

Replays the search through the same find_conflict / generate_one_node / space_time_astar
the solver uses, so the animation cannot drift from the algorithm. Needs Pillow only.
"""

import heapq
import itertools

from PIL import Image, ImageDraw, ImageFont

from mapf.grid import Grid
from mapf.high_level import CTNode, cost, find_conflict, generate_one_node
from mapf.low_level import space_time_astar

W, H = 840, 470
GX, GY, CELL = 36, 74, 58
TX, TY = 372, 74

BG      = (255, 255, 255)
PANEL   = (247, 248, 250)
LINE    = (213, 218, 224)
INK     = (31, 36, 48)
MUTED   = (107, 114, 128)
WALL    = (46, 52, 64)
A0      = (47, 111, 237)
A1      = (232, 131, 58)
RED     = (229, 72, 77)
GREEN   = (34, 150, 94)

AGENT_COLOR = {0: A0, 1: A1}
SUB = 7          # interpolation frames per timestep
FRAME_MS = 70


def font(size, bold=False):
    for path in ("/System/Library/Fonts/Supplemental/Arial Bold.ttf" if bold
                 else "/System/Library/Fonts/Supplemental/Arial.ttf",
                 "/System/Library/Fonts/Helvetica.ttc",
                 "/Library/Fonts/Arial.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default(size=size)


F_TITLE = font(19, bold=True)
F_BODY  = font(14)
F_SMALL = font(12)
F_NODE  = font(12, bold=True)


def trace(grid, agents):
    """Run CBS, recording each expansion and the children it produced."""
    counter = itertools.count()
    root_sol = {a: space_time_astar(grid, s, g, a, set(), set(), 0)
                for a, (s, g) in agents.items()}
    root = CTNode(set(), root_sol, cost(root_sol))
    open_, steps = [(root.cost, next(counter), root)], []

    while open_:
        _, _, node = heapq.heappop(open_)
        conflict = find_conflict(node.solution)
        kids = []
        if conflict is not None:
            for is_a in (True, False):
                child = CTNode(set(), dict(), 0)
                if generate_one_node(grid, agents, conflict, is_a, child, node):
                    added = next(iter(child.constraints - node.constraints))
                    kids.append((child, added))
                    heapq.heappush(open_, (child.cost, next(counter), child))
        steps.append({"node": node, "conflict": conflict, "kids": kids})
        if conflict is None:
            break
    return steps


def cell_px(x, y):
    return GX + x * CELL, GY + y * CELL


def at(path, t):
    return path[t] if t < len(path) else path[-1]


def draw_grid(d, grid, solution, positions, conflict_cell, agents):
    d.rectangle([GX - 6, GY - 6, GX + grid.width * CELL + 6, GY + grid.height * CELL + 6],
                fill=PANEL, outline=LINE)
    for x in range(grid.width):
        for y in range(grid.height):
            px, py = cell_px(x, y)
            box = [px + 1, py + 1, px + CELL - 1, py + CELL - 1]
            if not grid.is_free((x, y)):
                d.rectangle(box, fill=WALL)
            else:
                d.rectangle(box, fill=BG, outline=LINE)

    # goals as hollow rings
    for a, (_, goal) in agents.items():
        px, py = cell_px(*goal)
        d.ellipse([px + 14, py + 14, px + CELL - 14, py + CELL - 14],
                  outline=AGENT_COLOR[a], width=3)

    # planned route as a faint trail
    if solution:
        for a, path in solution.items():
            pts = [(cell_px(x, y)[0] + CELL // 2, cell_px(x, y)[1] + CELL // 2)
                   for x, y in path]
            if len(pts) > 1:
                d.line(pts, fill=AGENT_COLOR[a] + (0,), width=3)

    if conflict_cell is not None:
        px, py = cell_px(*conflict_cell)
        d.rectangle([px + 2, py + 2, px + CELL - 2, py + CELL - 2], outline=RED, width=4)

    # agents, nudged apart when they share a cell so the collision stays visible
    same = len({(round(p[0], 2), round(p[1], 2)) for p in positions.values()}) < len(positions)
    for i, (a, (fx, fy)) in enumerate(sorted(positions.items())):
        cx = GX + fx * CELL + CELL // 2
        cy = GY + fy * CELL + CELL // 2
        if same:
            cx += -7 if i == 0 else 7
            cy += -7 if i == 0 else 7
        r = 15
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=AGENT_COLOR[a],
                  outline=BG, width=2)
        d.text((cx, cy), str(a), font=F_NODE, fill=BG, anchor="mm")


def node_box(d, cx, cy, title, sub, state):
    """state: 'idle' | 'active' | 'conflict' | 'solved'"""
    w, h = 152, 48
    box = [cx - w // 2, cy - h // 2, cx + w // 2, cy + h // 2]
    edge = {"idle": LINE, "active": INK, "conflict": RED, "solved": GREEN}[state]
    fill = {"idle": BG, "active": PANEL, "conflict": (253, 240, 240),
            "solved": (238, 248, 242)}[state]
    d.rounded_rectangle(box, radius=7, fill=fill, outline=edge,
                        width=3 if state != "idle" else 1)
    d.text((cx, cy - 9), title, font=F_NODE, fill=INK, anchor="mm")
    d.text((cx, cy + 10), sub, font=F_SMALL,
           fill=RED if state == "conflict" else MUTED, anchor="mm")


def draw_tree(d, root_cost, conflict, kids, phase, chosen):
    d.rectangle([TX - 6, TY - 6, TX + 432, TY + 280], fill=PANEL, outline=LINE)
    d.text((TX + 213, TY + 16), "constraint tree", font=F_BODY, fill=MUTED, anchor="mm")

    rcx, rcy = TX + 213, TY + 62
    node_box(d, rcx, rcy, "root", f"cost {root_cost}",
             "conflict" if phase >= 1 else "active")

    if phase >= 2 and kids:
        for i, (child, added) in enumerate(kids):
            ccx = TX + 105 + i * 216
            ccy = TY + 190
            d.line([(rcx, rcy + 24), (rcx, rcy + 55), (ccx, rcy + 55), (ccx, ccy - 24)],
                   fill=LINE if chosen not in (i,) else INK, width=2)
            label = f"a{added.agent} ≠ {added.loc} @t{added.time}"
            state = "idle"
            if phase >= 3 and chosen == i:
                state = "solved" if phase >= 4 else "active"
            node_box(d, ccx, ccy, label, f"cost {child.cost}", state)

    if phase >= 4:
        d.text((TX + 213, TY + 258), "no conflicts — solution is optimal",
               font=F_SMALL, fill=GREEN, anchor="mm")
    elif phase >= 1 and conflict:
        a, b, loc, t = conflict
        d.text((TX + 213, TY + 258),
               f"vertex conflict: agents {a} and {b} at {loc}, t={t}",
               font=F_SMALL, fill=RED, anchor="mm")


def frame(grid, agents, solution, positions, conflict_cell, root_cost,
          conflict, kids, phase, chosen, heading, caption):
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.text((36, 24), "Conflict-Based Search", font=F_TITLE, fill=INK, anchor="lm")
    d.text((36, 46), heading, font=F_SMALL, fill=MUTED, anchor="lm")
    draw_grid(d, grid, solution, positions, conflict_cell, agents)
    draw_tree(d, root_cost, conflict, kids, phase, chosen)
    d.line([(36, 400), (W - 36, 400)], fill=LINE, width=1)
    d.text((36, 424), caption, font=F_BODY, fill=INK, anchor="lm")
    return img


def animate(frames, grid, agents, solution, root_cost, conflict, kids,
            phase, chosen, heading, caption, stop_at=None, freeze=0):
    horizon = max(len(p) for p in solution.values())
    last = stop_at if stop_at is not None else horizon - 1
    for t in range(last):
        for s in range(SUB):
            f = s / SUB
            pos = {}
            for a, path in solution.items():
                x0, y0 = at(path, t)
                x1, y1 = at(path, t + 1)
                pos[a] = (x0 + (x1 - x0) * f, y0 + (y1 - y0) * f)
            frames.append(frame(grid, agents, solution, pos, None, root_cost,
                                conflict, kids, phase, chosen, heading,
                                caption.format(t=t)))
    pos = {a: at(p, last) for a, p in solution.items()}
    cc = at(solution[0], last) if freeze and stop_at is not None else None
    for _ in range(freeze):
        frames.append(frame(grid, agents, solution, pos, cc, root_cost,
                            conflict, kids, phase, chosen, heading,
                            caption.format(t=last)))
    return pos


def main():
    grid = Grid(width=5, height=5, obstacles=frozenset({(0, 0), (4, 0), (0, 4), (4, 4)}))
    agents = {0: ((0, 2), (4, 2)), 1: ((2, 0), (2, 4))}

    steps = trace(grid, agents)
    root, conflict, kids = steps[0]["node"], steps[0]["conflict"], steps[0]["kids"]
    solved = steps[-1]["node"]
    chosen = next(i for i, (c, _) in enumerate(kids)
                  if c.solution == solved.solution)

    print(f"expansions={len(steps)} conflict={conflict} "
          f"root_cost={root.cost} solved_cost={solved.cost}")

    frames = []
    _, _, cloc, ct = conflict

    # Act 1 - unconstrained shortest paths run straight into each other
    animate(frames, grid, agents, root.solution, root.cost, None, [], 0, None,
            "step 1 of 3  ·  plan each agent independently",
            "Shortest path per agent, ignoring the others.", stop_at=ct)
    for _ in range(18):
        frames.append(frame(grid, agents, root.solution,
                            {a: at(p, ct) for a, p in root.solution.items()},
                            cloc, root.cost, conflict, [], 1, None,
                            "step 1 of 3  ·  plan each agent independently",
                            f"Both agents want {cloc} at t={ct}. That is a vertex conflict."))

    # Act 2 - branch on the conflict
    for _ in range(22):
        frames.append(frame(grid, agents, root.solution,
                            {a: at(p, ct) for a, p in root.solution.items()},
                            cloc, root.cost, conflict, kids, 2, None,
                            "step 2 of 3  ·  branch on the conflict",
                            "Two children: forbid one agent or the other from that cell."))

    # Act 3 - replan the constrained agent
    animate(frames, grid, agents, solved.solution, root.cost, conflict, kids, 3, chosen,
            "step 3 of 3  ·  replan under the new constraint",
            "Agent 0 waits one step, then goes. Cost rises 8 to 9.")
    for _ in range(30):
        frames.append(frame(grid, agents, solved.solution,
                            {a: p[-1] for a, p in solved.solution.items()},
                            None, root.cost, conflict, kids, 4, chosen,
                            "step 3 of 3  ·  replan under the new constraint",
                            f"Conflict free at cost {solved.cost}, and provably optimal."))

    out = "docs/cbs-conflict.gif"
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=FRAME_MS, loop=0, optimize=True)
    print(f"wrote {out}  ({len(frames)} frames)")
    frames[len(frames) // 2].save("docs/frame-mid.png")
    frames[-1].save("docs/frame-final.png")


if __name__ == "__main__":
    main()
