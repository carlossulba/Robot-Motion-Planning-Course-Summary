/* RRT visualization: grow a tree from q_start by repeatedly steering from the
   nearest tree node toward a random (occasionally goal-biased) sample. */
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

  function computeRRT(world, rng) {
    const STEP = 16, GOAL_BIAS = 0.08, MAX_ITER = 3000;
    const nodes = [{ x: world.start.x, y: world.start.y, parent: -1 }];
    const events = [];
    let success = false, goalIdx = -1;

    for (let iter = 0; iter < MAX_ITER; iter++) {
      const sample = rng() < GOAL_BIAS
        ? world.goal
        : { x: world.margin * 0.3 + rng() * (world.width - world.margin * 0.6), y: world.margin * 0.3 + rng() * (world.height - world.margin * 0.6) };

      let nearest = 0, bestD = Infinity;
      for (let i = 0; i < nodes.length; i++) { const d = dist(nodes[i], sample); if (d < bestD) { bestD = d; nearest = i; } }
      const d = Math.max(1e-6, bestD);
      const t = Math.min(STEP, d) / d;
      const newPt = { x: nodes[nearest].x + (sample.x - nodes[nearest].x) * t, y: nodes[nearest].y + (sample.y - nodes[nearest].y) * t };

      if (!segmentFree(world, nodes[nearest], newPt)) {
        events.push({ type: "blocked", from: { x: nodes[nearest].x, y: nodes[nearest].y }, to: newPt });
        continue;
      }
      nodes.push({ x: newPt.x, y: newPt.y, parent: nearest });
      const newIdx = nodes.length - 1;
      events.push({ type: "add", idx: newIdx });

      if (dist(newPt, world.goal) < STEP * 1.3 && segmentFree(world, newPt, world.goal)) {
        nodes.push({ x: world.goal.x, y: world.goal.y, parent: newIdx });
        goalIdx = nodes.length - 1;
        events.push({ type: "goal", idx: goalIdx });
        success = true;
        break;
      }
    }
    if (!success) events.push({ type: "fail" });

    let path = null;
    if (success) {
      path = []; let c = goalIdx;
      while (c !== -1) { path.push(c); c = nodes[c].parent; }
      path.reverse();
    }
    return { nodes, events, path, success };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });
    ctx.save();
    ctx.strokeStyle = "rgba(111,168,220,0.7)";
    ctx.lineWidth = 1.3;
    for (let k = 0; k <= idx && k < data.events.length; k++) {
      const e = data.events[k];
      if (e.type === "add" || e.type === "goal") {
        const n = data.nodes[e.idx], p = data.nodes[n.parent];
        ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(n.x, n.y); ctx.stroke();
      }
    }
    ctx.restore();

    const cur = data.events[idx];
    if (cur && cur.type === "blocked") {
      ctx.save();
      ctx.strokeStyle = "#c23b3b";
      ctx.setLineDash([3, 3]);
      ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.moveTo(cur.from.x, cur.from.y); ctx.lineTo(cur.to.x, cur.to.y); ctx.stroke();
      ctx.restore();
    }

    if (data.path && idx >= data.events.length - 1) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 3;
      ctx.beginPath();
      data.path.forEach((id, k) => { const p = data.nodes[id]; k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height }) {
    const world = makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const data = computeRRT(world, rng);
    let idx = -1;
    const total = data.events.length;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, total - 1);
        const done = idx >= total - 1;
        const e = data.events[idx];
        let note;
        if (e.type === "add") note = `Extended the tree toward a random sample — node ${e.idx} added (${data.nodes.length - 1} nodes so far).`;
        else if (e.type === "blocked") note = "Extension blocked by an obstacle — discarded, no node added.";
        else if (e.type === "goal") note = `Within reach of q_goal with a clear line of sight — connected! Path uses ${data.path.length} nodes.`;
        else note = "Iteration budget exhausted without reaching the goal — try Generate new.";
        return { done, note };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.rrt = {
    title: "RRT (Rapidly-Exploring Random Tree)",
    badge: "§8.5.1 / book §7.2.2",
    subtitle: "Grow a tree from q_start: sample a point, extend from the nearest tree node by a fixed step, keep it if collision-free.",
    width: 560, height: 360,
    legend: [
      { color: "rgba(111,168,220,0.9)", label: "tree edge" },
      { color: "#c23b3b", label: "blocked extension (this step)" },
      { color: "#2f8f5b", label: "path to goal" },
    ],
    makeSim,
    pythonCode: `
def rrt(start, goal, is_free, step=16, goal_bias=0.08, max_iter=3000):
    nodes = [Node(start, parent=None)]

    for _ in range(max_iter):
        sample = goal if random() < goal_bias else random_free_point()
        nearest = min(nodes, key=lambda n: dist(n.pos, sample))
        direction = normalize(sample - nearest.pos)
        new_pt = nearest.pos + direction * min(step, dist(nearest.pos, sample))

        if not segment_free(nearest.pos, new_pt, is_free):
            continue                                    # discard, try again

        new_node = Node(new_pt, parent=nearest)
        nodes.append(new_node)

        if dist(new_pt, goal) < step * 1.3 and segment_free(new_pt, goal, is_free):
            goal_node = Node(goal, parent=new_node)
            return trace_back(goal_node)                # success

    return None                                          # iteration budget exhausted
`,
  };
})();
