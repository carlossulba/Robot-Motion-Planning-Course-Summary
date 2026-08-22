/* RRT* visualization: like RRT, but each new node picks the lowest-cost
   nearby parent and rewires nearby nodes through itself when that shortens
   their path — converging toward an optimal tree, not just any tree. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

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

  function computeRRTStar(world, rng) {
    const STEP = 16, RADIUS = 46, GOAL_BIAS = 0.08, MAX_ITER = 1600;
    const nodes = [{ x: world.start.x, y: world.start.y, parent: -1, cost: 0 }];
    const events = [];
    let success = false, goalIdx = -1;

    function propagateCost(pIdx, delta) {
      for (let k = 0; k < nodes.length; k++) {
        if (nodes[k].parent === pIdx) { nodes[k].cost += delta; propagateCost(k, delta); }
      }
    }

    for (let iter = 0; iter < MAX_ITER; iter++) {
      const sample = rng() < GOAL_BIAS
        ? world.goal
        : { x: world.margin * 0.3 + rng() * (world.width - world.margin * 0.6), y: world.margin * 0.3 + rng() * (world.height - world.margin * 0.6) };
      let nearest = 0, bestD = Infinity;
      for (let i = 0; i < nodes.length; i++) { const d = dist(nodes[i], sample); if (d < bestD) { bestD = d; nearest = i; } }
      const d = Math.max(1e-6, bestD);
      const t = Math.min(STEP, d) / d;
      const newPt = { x: nodes[nearest].x + (sample.x - nodes[nearest].x) * t, y: nodes[nearest].y + (sample.y - nodes[nearest].y) * t };

      if (!segmentFree(world, nodes[nearest], newPt)) { events.push({ type: "blocked", from: { x: nodes[nearest].x, y: nodes[nearest].y }, to: newPt }); continue; }

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
          propagateCost(i, delta);
        }
      }
      events.push({ type: "add", idx: newIdx, parent: bestParent, rewires });

      if (dist(newPt, world.goal) < STEP * 1.3 && segmentFree(world, newPt, world.goal)) {
        const goalCost = bestCost + dist(newPt, world.goal);
        if (goalIdx === -1) {
          nodes.push({ x: world.goal.x, y: world.goal.y, parent: newIdx, cost: goalCost });
          goalIdx = nodes.length - 1;
          events.push({ type: "goal", idx: goalIdx });
          success = true;
        }
      }
      if (success && iter > MAX_ITER * 0.55) break; // keep improving briefly, then stop
    }
    if (!success) events.push({ type: "fail" });

    return { nodes, events, success, goalIdx };
  }

  function currentParents(data, idx) {
    // replay add/rewire events in order so step k shows the tree exactly as
    // it existed after that step, not the final structure
    const p2 = data.nodes.map(() => undefined);
    p2[0] = -1;
    for (let k = 0; k <= idx && k < data.events.length; k++) {
      const e = data.events[k];
      if (e.type === "add") {
        p2[e.idx] = e.parent;
        (e.rewires || []).forEach((r) => { p2[r] = e.idx; });
      } else if (e.type === "goal") {
        p2[e.idx] = data.nodes[e.idx].parent;
      }
    }
    return p2;
  }

  function pathTo(data, parents, idx) {
    if (data.goalIdx === -1 || parents[data.goalIdx] === undefined) return null;
    const path = []; let c = data.goalIdx;
    while (c !== -1 && c !== undefined) { path.push(c); c = parents[c]; }
    return path.reverse();
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });
    const parents = currentParents(data, idx);
    ctx.save();
    ctx.strokeStyle = "rgba(111,168,220,0.7)";
    ctx.lineWidth = 1.3;
    for (let i = 1; i < data.nodes.length; i++) {
      const p = parents[i];
      if (p === undefined || p === null || p === -1) continue;
      const n = data.nodes[i], pn = data.nodes[p];
      ctx.beginPath(); ctx.moveTo(pn.x, pn.y); ctx.lineTo(n.x, n.y); ctx.stroke();
    }
    ctx.restore();

    const cur = data.events[idx];
    if (cur && cur.type === "blocked") {
      ctx.save();
      ctx.strokeStyle = "#c23b3b"; ctx.setLineDash([3, 3]); ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.moveTo(cur.from.x, cur.from.y); ctx.lineTo(cur.to.x, cur.to.y); ctx.stroke();
      ctx.restore();
    }
    if (cur && cur.type === "add" && cur.rewires && cur.rewires.length) {
      ctx.save();
      ctx.strokeStyle = "#b7532c"; ctx.lineWidth = 2;
      const n = data.nodes[cur.idx];
      cur.rewires.forEach((r) => { const rn = data.nodes[r]; ctx.beginPath(); ctx.moveTo(n.x, n.y); ctx.lineTo(rn.x, rn.y); ctx.stroke(); });
      ctx.restore();
    }

    const path = pathTo(data, parents, idx);
    if (path && (idx >= data.events.length - 1 || (cur && cur.type === "add" && cur.rewires && cur.rewires.length))) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 3;
      ctx.beginPath();
      path.forEach((id, k) => { const p = data.nodes[id]; k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height }) {
    const world = makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const data = computeRRTStar(world, rng);
    let idx = -1;
    const total = data.events.length;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, total - 1);
        const done = idx >= total - 1;
        const e = data.events[idx];
        let note;
        if (e.type === "add") {
          note = `Node ${e.idx} added via its lowest-cost nearby parent.`;
          if (e.rewires && e.rewires.length) note += ` Rewired ${e.rewires.length} nearby node(s) through it — shorter paths found (orange).`;
        } else if (e.type === "blocked") note = "Extension blocked by an obstacle — discarded.";
        else if (e.type === "goal") note = "First connection to q_goal found — the tree keeps rewiring briefly to improve it.";
        else note = "Iteration budget exhausted — try Generate new.";
        if (done && data.success) {
          const g = data.nodes[data.goalIdx];
          note += ` Final path cost ≈ ${g.cost.toFixed(0)}px.`;
        }
        return { done, note };
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
      { color: "rgba(111,168,220,0.9)", label: "tree edge" },
      { color: "#b7532c", label: "rewire (this step)" },
      { color: "#2f8f5b", label: "current best path" },
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
`,
  };
})();
