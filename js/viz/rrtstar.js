/* RRT* visualization: like RRT, but each new node picks the lowest-cost
   nearby parent and rewires nearby nodes through itself when that shortens
   their path — converging toward an optimal tree, not just any tree.

   q_new is shown as a hollow gold candidate for one step before it's
   resolved (added via its best parent, or discarded on collision) exactly
   like the RRT demo; the highlighted pseudocode line on an "add" step
   reflects whether a rewire actually happened that step, not just that a
   node was added. The "bidirectional" toggle grows two independent RRT*
   trees (each with its own cost/rewire bookkeeping) from q_start and
   q_goal, terminating the moment either tree's newest node can connect
   straight to the other tree, instead of growing one tree to the goal. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  const START_COLOR = "rgba(111,168,220,0.9)";
  const GOAL_TREE_COLOR = "rgba(31,156,138,0.9)";
  const CANDIDATE = "#e2b06a";
  const REJECT = "#c23b3b";
  const REWIRE = "#b7532c";
  const SAMPLE_COLOR = "#9b7fd4";

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
  function segmentFree(world, a, b, step) {
    step = step || 4;
    const d = dist(a, b), n = Math.max(1, Math.ceil(d / step));
    for (let i = 1; i <= n; i++) {
      const t = i / n;
      if (!world.isFree(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return false;
    }
    return true;
  }
  function randPt(world, rng) {
    return { x: world.margin * 0.3 + rng() * (world.width - world.margin * 0.6), y: world.margin * 0.3 + rng() * (world.height - world.margin * 0.6) };
  }
  function nearestIdx(nodes, pt) {
    let best = 0, bestD = Infinity;
    for (let i = 0; i < nodes.length; i++) { const d = dist(nodes[i], pt); if (d < bestD) { bestD = d; best = i; } }
    return { idx: best, d: bestD };
  }
  function propagateCost(nodes, pIdx, delta) {
    for (let k = 0; k < nodes.length; k++) {
      if (nodes[k].parent === pIdx) { nodes[k].cost += delta; propagateCost(nodes, k, delta); }
    }
  }

  // pseudocode line indices (0-based) -- see vizDefs.rrtstar.pseudocode below
  const L_EXTEND = 0, L_EXAMINE = 1, L_PARENT = 2, L_REWIRE = 3;

  const STEP = 16, RADIUS = 46, GOAL_BIAS = 0.08;

  // One RRT* iteration against `nodes` (mutated in place). Pushes a
  // "candidate" event, then either "blocked" or "add" (carrying `rewires`).
  // Returns the new node's index, or -1 if the extension was blocked.
  function rrtStarIterate(nodes, treeIdx, sample, world, events) {
    const { idx: nearest, d: bestD } = nearestIdx(nodes, sample);
    const d = Math.max(1e-6, bestD);
    const t = Math.min(STEP, d) / d;
    const newPt = { x: nodes[nearest].x + (sample.x - nodes[nearest].x) * t, y: nodes[nearest].y + (sample.y - nodes[nearest].y) * t };

    events.push({ type: "candidate", from: { x: nodes[nearest].x, y: nodes[nearest].y }, pt: newPt, sample: { x: sample.x, y: sample.y }, tree: treeIdx, line: L_EXTEND });

    if (!segmentFree(world, nodes[nearest], newPt)) {
      events.push({ type: "blocked", from: { x: nodes[nearest].x, y: nodes[nearest].y }, to: newPt, line: L_EXTEND });
      return -1;
    }

    const nearIdxs = [];
    for (let i = 0; i < nodes.length; i++) if (dist(nodes[i], newPt) < RADIUS) nearIdxs.push(i);
    let bestParent = nearest, bestCost = nodes[nearest].cost + dist(nodes[nearest], newPt);
    for (const i of nearIdxs) {
      if (i === nearest) continue;
      if (!segmentFree(world, nodes[i], newPt)) continue;
      const c = nodes[i].cost + dist(nodes[i], newPt);
      if (c < bestCost) { bestCost = c; bestParent = i; }
    }

    const newIdx = nodes.length;
    nodes.push({ x: newPt.x, y: newPt.y, parent: bestParent, cost: bestCost });

    const rewires = [];
    for (const i of nearIdxs) {
      if (i === bestParent) continue;
      if (!segmentFree(world, newPt, nodes[i])) continue;
      const c = bestCost + dist(newPt, nodes[i]);
      if (c < nodes[i].cost - 0.01) {
        const delta = c - nodes[i].cost;
        rewires.push(i);
        nodes[i].parent = newIdx;
        nodes[i].cost = c;
        propagateCost(nodes, i, delta);
      }
    }
    events.push({ type: "add", idx: newIdx, tree: treeIdx, parent: bestParent, rewires, line: rewires.length ? L_REWIRE : L_PARENT });
    return newIdx;
  }

  function computeRRTStar(world, rng, bidirectional) {
    const events = [];

    if (!bidirectional) {
      const MAX_ITER = 1600;
      const nodes = [{ x: world.start.x, y: world.start.y, parent: -1, cost: 0 }];
      let success = false, goalIdx = -1;

      for (let iter = 0; iter < MAX_ITER; iter++) {
        const sample = rng() < GOAL_BIAS ? world.goal : randPt(world, rng);
        const newIdx = rrtStarIterate(nodes, 0, sample, world, events);
        if (newIdx === -1) continue;
        const newPt = nodes[newIdx];
        if (dist(newPt, world.goal) < STEP * 1.3 && segmentFree(world, newPt, world.goal)) {
          const goalCost = newPt.cost + dist(newPt, world.goal);
          if (goalIdx === -1) {
            nodes.push({ x: world.goal.x, y: world.goal.y, parent: newIdx, cost: goalCost });
            goalIdx = nodes.length - 1;
            events.push({ type: "add", idx: goalIdx, tree: 0, parent: newIdx, rewires: [], isGoal: true, line: L_PARENT });
            success = true;
          }
        }
        if (success && iter > MAX_ITER * 0.55) break; // keep improving briefly, then stop
      }
      if (!success) events.push({ type: "fail" });
      return { mode: "single", trees: [{ nodes, color: START_COLOR }], events, success, goalIdx, connS: -1, connG: -1 };
    }

    // ---- bidirectional: two independent RRT* trees grow toward each other ----
    const MAX_ITER = 1600;
    const treeS = [{ x: world.start.x, y: world.start.y, parent: -1, cost: 0 }];
    const treeG = [{ x: world.goal.x, y: world.goal.y, parent: -1, cost: 0 }];
    const trees = [treeS, treeG];
    let success = false, connS = -1, connG = -1;

    for (let iter = 0; iter < MAX_ITER; iter++) {
      const ext = trees[0].length <= trees[1].length ? 0 : 1;
      const other = 1 - ext;
      const arr = trees[ext];
      const sample = randPt(world, rng);
      const newIdx = rrtStarIterate(arr, ext, sample, world, events);
      if (newIdx === -1) continue;
      const newPt = arr[newIdx];
      const oarr = trees[other];
      const { idx: nB, d: dB } = nearestIdx(oarr, newPt);
      if (dB < STEP * 1.3 && segmentFree(world, newPt, oarr[nB])) {
        events.push({ type: "connect", treeA: ext, idxA: newIdx, treeB: other, idxB: nB, line: L_REWIRE });
        success = true;
        connS = ext === 0 ? newIdx : nB;
        connG = ext === 0 ? nB : newIdx;
        break;
      }
    }
    if (!success) events.push({ type: "fail" });
    return { mode: "bidirectional", trees: [{ nodes: treeS, color: START_COLOR }, { nodes: treeG, color: GOAL_TREE_COLOR }], events, success, goalIdx: -1, connS, connG };
  }

  // Replays add/rewire events up through `idx` so the tree renders exactly
  // as it existed after that many steps, not the final structure.
  function currentParents(tree, events, idx, treeTag) {
    const p = tree.nodes.map(() => undefined);
    p[0] = -1;
    for (let k = 0; k <= idx && k < events.length; k++) {
      const e = events[k];
      if (e.tree !== treeTag) continue;
      if (e.type === "add") {
        p[e.idx] = e.parent;
        (e.rewires || []).forEach((r) => { p[r] = e.idx; });
      }
    }
    return p;
  }

  function pathTo(data, idx) {
    if (data.mode === "single") {
      if (data.goalIdx === -1) return null;
      const tree = data.trees[0];
      const parents = currentParents(tree, data.events, idx, 0);
      if (parents[data.goalIdx] === undefined) return null;
      const path = []; let c = data.goalIdx;
      while (c !== -1 && c !== undefined) { path.push(tree.nodes[c]); c = parents[c]; }
      return path.reverse();
    }
    if (data.connS === -1) return null;
    // only valid once the "connect" event itself has been reached
    const connectIdx = data.events.findIndex((e) => e.type === "connect");
    if (connectIdx === -1 || idx < connectIdx) return null;
    const treeS = data.trees[0], treeG = data.trees[1];
    const pS_parents = currentParents(treeS, data.events, idx, 0);
    const pG_parents = currentParents(treeG, data.events, idx, 1);
    const pS = []; let c = data.connS;
    while (c !== -1 && c !== undefined) { pS.push(treeS.nodes[c]); c = pS_parents[c]; }
    pS.reverse();
    const pG = []; c = data.connG;
    while (c !== -1 && c !== undefined) { pG.push(treeG.nodes[c]); c = pG_parents[c]; }
    return pS.concat(pG);
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });

    ctx.save();
    ctx.lineWidth = 1.3;
    data.trees.forEach((tree, ti) => {
      const parents = currentParents(tree, data.events, idx, ti);
      ctx.strokeStyle = tree.color;
      for (let i = 1; i < tree.nodes.length; i++) {
        const p = parents[i];
        if (p === undefined || p === null || p === -1) continue;
        const n = tree.nodes[i], pn = tree.nodes[p];
        ctx.beginPath(); ctx.moveTo(pn.x, pn.y); ctx.lineTo(n.x, n.y); ctx.stroke();
      }
    });
    ctx.restore();

    const cur = idx >= 0 ? data.events[idx] : null;
    if (cur && cur.type === "candidate") {
      if (cur.sample) {
        ctx.save();
        ctx.strokeStyle = SAMPLE_COLOR; ctx.lineWidth = 1.6;
        const r = 4.5, p = cur.sample;
        ctx.beginPath(); ctx.moveTo(p.x - r, p.y); ctx.lineTo(p.x + r, p.y);
        ctx.moveTo(p.x, p.y - r); ctx.lineTo(p.x, p.y + r); ctx.stroke();
        ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.stroke();
        ctx.restore();
      }
      ctx.save();
      ctx.setLineDash([2, 2]); ctx.strokeStyle = CANDIDATE; ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.moveTo(cur.from.x, cur.from.y); ctx.lineTo(cur.pt.x, cur.pt.y); ctx.stroke();
      ctx.setLineDash([]);
      ctx.strokeStyle = CANDIDATE; ctx.lineWidth = 1.8;
      ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 5, 0, Math.PI * 2); ctx.stroke();
      ctx.restore();
    } else if (cur && cur.type === "blocked") {
      ctx.save();
      ctx.strokeStyle = REJECT; ctx.setLineDash([3, 3]); ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.moveTo(cur.from.x, cur.from.y); ctx.lineTo(cur.to.x, cur.to.y); ctx.stroke();
      ctx.restore();
    } else if (cur && cur.type === "add" && cur.rewires && cur.rewires.length) {
      ctx.save();
      ctx.strokeStyle = REWIRE; ctx.lineWidth = 2;
      const tree = data.trees[cur.tree];
      const n = tree.nodes[cur.idx];
      cur.rewires.forEach((r) => { const rn = tree.nodes[r]; ctx.beginPath(); ctx.moveTo(n.x, n.y); ctx.lineTo(rn.x, rn.y); ctx.stroke(); });
      ctx.restore();
    } else if (cur && cur.type === "connect") {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 2.3;
      const a = data.trees[cur.treeA].nodes[cur.idxA], b = data.trees[cur.treeB].nodes[cur.idxB];
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      ctx.restore();
    }

    const path = pathTo(data, idx);
    if (path) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 3;
      ctx.beginPath();
      path.forEach((p, k) => { k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, params, width, height, world: sharedWorld }) {
    const bidirectional = !!(params && params.bidirectional);
    const world = sharedWorld || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const data = computeRRTStar(world, rng, bidirectional);
    let idx = -1;
    const total = data.events.length;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, total - 1);
        const done = idx >= total - 1;
        const e = data.events[idx];
        const treeName = (t) => (data.mode === "bidirectional" ? (t === 0 ? "T_start" : "T_goal") : "the tree");
        let note;
        if (e.type === "candidate") note = `Sampled q_rand (purple), extended toward it from ${treeName(e.tree)} -> q_new. Testing collision...`;
        else if (e.type === "blocked") note = "Extension blocked by an obstacle — discarded.";
        else if (e.type === "add" && e.isGoal) note = "First connection to q_goal found — the tree keeps rewiring briefly to improve it.";
        else if (e.type === "add") {
          note = `Examined nodes within radius ${RADIUS}px of q_new, chose the lowest-cost collision-free parent — added to ${treeName(e.tree)}.`;
          if (e.rewires && e.rewires.length) note += ` Rewired ${e.rewires.length} nearby node(s) through it — shorter paths found (orange).`;
        } else if (e.type === "connect") note = "T_start and T_goal made contact — trees connected! Path found.";
        else note = "Iteration budget exhausted — try Generate new.";
        if (done && data.success && data.mode === "single") {
          const g = data.trees[0].nodes[data.goalIdx];
          note += ` Final path cost ≈ ${g.cost.toFixed(0)}px.`;
        }
        return { done, note, line: e.line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.rrtstar = {
    title: "RRT*",
    badge: "§8.5.3 / book §7.2.2",
    subtitle: "RRT plus rewiring: every new node picks the lowest-cost nearby parent, and can steal nearby nodes from the tree if that shortens their path.",
    width: 560, height: 360,
    legend: [
      { color: START_COLOR, label: "tree edge (from start)" },
      { color: GOAL_TREE_COLOR, label: "tree edge (from goal, bidirectional)" },
      { color: SAMPLE_COLOR, label: "random sample q_rand (this step only)" },
      { color: CANDIDATE, label: "candidate q_new (untested)" },
      { color: REJECT, label: "blocked extension" },
      { color: REWIRE, label: "rewire (this step)" },
      { color: "#2f8f5b", label: "current best path" },
    ],
    params: [
      { key: "bidirectional", label: "bidirectional (grow T_start and T_goal)", type: "checkbox", value: false },
    ],
    pseudocode: [
      "sample, extend to q_new exactly as in RRT",
      "examine existing nodes within a shrinking radius of q_new (not just the single nearest)",
      "choose best parent: connect q_new to whichever nearby node gives the lowest cost-from-root",
      "rewire: re-parent any nearby node through q_new if that would lower ITS cost-from-root",
    ],
    makeSim,
    pythonCode: `
def rrt_star(start, goal, is_free, step=16, radius=46, max_iter=1600):
    nodes = [Node(start, parent=None, cost=0)]

    for _ in range(max_iter):
        sample = biased_random_point(goal)
        nearest = min(nodes, key=lambda n: dist(n.pos, sample))
        new_pt = steer(nearest.pos, sample, step)
        if not segment_free(nearest.pos, new_pt, is_free):
            continue

        near = [n for n in nodes if dist(n.pos, new_pt) < radius]

        # pick the lowest-cost collision-free parent, not just the nearest
        best_parent = min(near, key=lambda n: n.cost + dist(n.pos, new_pt)
                           if segment_free(n.pos, new_pt, is_free) else float("inf"))
        new_node = Node(new_pt, parent=best_parent,
                         cost=best_parent.cost + dist(best_parent.pos, new_pt))
        nodes.append(new_node)

        # rewire: would routing a neighbor through new_node be cheaper?
        for n in near:
            new_cost = new_node.cost + dist(new_pt, n.pos)
            if new_cost < n.cost and segment_free(new_pt, n.pos, is_free):
                propagate_cost_to_descendants(n, new_cost - n.cost)
                n.parent, n.cost = new_node, new_cost

    return best_path_to(goal, nodes)   # cost keeps improving as more samples arrive

# bidirectional variant: grow two independent RRT* trees (T_start, T_goal),
# each with its own cost/rewire bookkeeping, and try connecting the newest
# node straight to the other tree's nearest node after every extension.
`,
  };
})();
