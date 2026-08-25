/* RRT visualization: grow a tree from q_start by repeatedly steering from the
   nearest tree node toward a random (occasionally goal-biased) sample.

   q_new is always shown as a hollow gold candidate marker for one step
   before it's resolved -- either added (solid edge) or discarded (a red
   dashed flash) -- so the collision-check decision is visible, not just its
   outcome. The "bidirectional" toggle switches to growing T_start and
   T_goal toward each other, terminating the moment they touch instead of
   when T_start reaches q_goal. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  const START_COLOR = "rgba(111,168,220,0.9)";
  const GOAL_TREE_COLOR = "rgba(31,156,138,0.9)";
  const CANDIDATE = "#e2b06a";
  const REJECT = "#c23b3b";
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

  // pseudocode line indices (0-based) -- see vizDefs.rrt.pseudocode below
  const L_SAMPLE = 0, L_NEAREST = 1, L_EXTEND = 2, L_ADD = 3, L_GOAL = 4;

  function computeRRT(world, rng, bidirectional) {
    const STEP = 16, GOAL_BIAS = 0.08, MAX_ITER = 3000;
    const events = [];

    if (!bidirectional) {
      const nodes = [{ x: world.start.x, y: world.start.y, parent: -1 }];
      let success = false, goalIdx = -1;

      for (let iter = 0; iter < MAX_ITER; iter++) {
        const sample = rng() < GOAL_BIAS ? world.goal : randPt(world, rng);
        const { idx: nearest, d: bestD } = nearestIdx(nodes, sample);
        const d = Math.max(1e-6, bestD);
        const t = Math.min(STEP, d) / d;
        const newPt = { x: nodes[nearest].x + (sample.x - nodes[nearest].x) * t, y: nodes[nearest].y + (sample.y - nodes[nearest].y) * t };

        events.push({ type: "candidate", from: { x: nodes[nearest].x, y: nodes[nearest].y }, pt: newPt, sample: { x: sample.x, y: sample.y }, tree: 0, line: L_EXTEND });

        if (!segmentFree(world, nodes[nearest], newPt)) {
          events.push({ type: "blocked", from: { x: nodes[nearest].x, y: nodes[nearest].y }, to: newPt, line: L_ADD });
          continue;
        }
        nodes.push({ x: newPt.x, y: newPt.y, parent: nearest });
        const newIdx = nodes.length - 1;
        events.push({ type: "add", idx: newIdx, tree: 0, line: L_ADD });

        if (dist(newPt, world.goal) < STEP * 1.3 && segmentFree(world, newPt, world.goal)) {
          nodes.push({ x: world.goal.x, y: world.goal.y, parent: newIdx });
          goalIdx = nodes.length - 1;
          events.push({ type: "add", idx: goalIdx, tree: 0, isGoal: true, line: L_GOAL });
          success = true;
          break;
        }
      }
      if (!success) events.push({ type: "fail" });

      let path = null;
      if (success) {
        path = []; let c = goalIdx;
        while (c !== -1) { path.push(nodes[c]); c = nodes[c].parent; }
        path.reverse();
      }
      return { mode: "single", trees: [{ nodes, color: START_COLOR }], events, path, success };
    }

    // ---- bidirectional: T_start (tree 0) and T_goal (tree 1) grow toward each other ----
    const treeS = [{ x: world.start.x, y: world.start.y, parent: -1 }];
    const treeG = [{ x: world.goal.x, y: world.goal.y, parent: -1 }];
    const trees = [treeS, treeG];
    let success = false, connS = -1, connG = -1;

    for (let iter = 0; iter < MAX_ITER; iter++) {
      const ext = trees[0].length <= trees[1].length ? 0 : 1;
      const other = 1 - ext;
      const arr = trees[ext];
      const sample = randPt(world, rng);
      const { idx: nearest, d: bestD } = nearestIdx(arr, sample);
      const d = Math.max(1e-6, bestD);
      const t = Math.min(STEP, d) / d;
      const newPt = { x: arr[nearest].x + (sample.x - arr[nearest].x) * t, y: arr[nearest].y + (sample.y - arr[nearest].y) * t };

      events.push({ type: "candidate", from: { x: arr[nearest].x, y: arr[nearest].y }, pt: newPt, sample: { x: sample.x, y: sample.y }, tree: ext, line: L_EXTEND });

      if (!segmentFree(world, arr[nearest], newPt)) {
        events.push({ type: "blocked", from: { x: arr[nearest].x, y: arr[nearest].y }, to: newPt, line: L_ADD });
        continue;
      }
      arr.push({ x: newPt.x, y: newPt.y, parent: nearest });
      const newIdx = arr.length - 1;
      events.push({ type: "add", idx: newIdx, tree: ext, line: L_ADD });

      const oarr = trees[other];
      const { idx: nB, d: dB } = nearestIdx(oarr, newPt);
      if (dB < STEP * 1.3 && segmentFree(world, newPt, oarr[nB])) {
        events.push({ type: "connect", treeA: ext, idxA: newIdx, treeB: other, idxB: nB, line: L_GOAL });
        success = true;
        connS = ext === 0 ? newIdx : nB;
        connG = ext === 0 ? nB : newIdx;
        break;
      }
    }
    if (!success) events.push({ type: "fail" });

    let path = null;
    if (success) {
      const pS = []; let c = connS;
      while (c !== -1) { pS.push(treeS[c]); c = treeS[c].parent; }
      pS.reverse();
      const pG = []; c = connG;
      while (c !== -1) { pG.push(treeG[c]); c = treeG[c].parent; }
      path = pS.concat(pG);
    }
    return { mode: "bidirectional", trees: [{ nodes: treeS, color: START_COLOR }, { nodes: treeG, color: GOAL_TREE_COLOR }], events, path, success };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });

    ctx.save();
    ctx.lineWidth = 1.3;
    for (let k = 0; k <= idx && k < data.events.length; k++) {
      const e = data.events[k];
      if (e.type === "add") {
        const tr = data.trees[e.tree].nodes;
        const n = tr[e.idx], p = tr[n.parent];
        ctx.strokeStyle = data.trees[e.tree].color;
        ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(n.x, n.y); ctx.stroke();
      } else if (e.type === "connect") {
        const a = data.trees[e.treeA].nodes[e.idxA], b = data.trees[e.treeB].nodes[e.idxB];
        ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 2.3;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        ctx.lineWidth = 1.3;
      }
    }
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
      ctx.strokeStyle = REJECT;
      ctx.setLineDash([3, 3]);
      ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.moveTo(cur.from.x, cur.from.y); ctx.lineTo(cur.to.x, cur.to.y); ctx.stroke();
      ctx.restore();
    }

    if (data.path && idx >= data.events.length - 1) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 3;
      ctx.beginPath();
      data.path.forEach((p, k) => { k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, params, width, height, world: sharedWorld }) {
    const bidirectional = !!(params && params.bidirectional);
    const world = sharedWorld || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const data = computeRRT(world, rng, bidirectional);
    let idx = -1;
    const total = data.events.length;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, total - 1);
        const done = idx >= total - 1;
        const e = data.events[idx];
        let note;
        const treeName = (t) => (data.mode === "bidirectional" ? (t === 0 ? "T_start" : "T_goal") : "the tree");
        if (e.type === "candidate") note = `Sampled q_rand (purple), steered from the nearest node of ${treeName(e.tree)} toward it by step size dq -> q_new. Testing collision...`;
        else if (e.type === "blocked") note = "Extension blocked by an obstacle — q_new discarded, no node added.";
        else if (e.type === "add" && e.isGoal) note = `Within reach of q_goal with a clear line of sight — connected! Path uses ${data.path.length} nodes.`;
        else if (e.type === "add") note = `Collision-free — q_new added to ${treeName(e.tree)} (${data.trees[e.tree].nodes.length} nodes so far).`;
        else if (e.type === "connect") note = `T_start and T_goal made contact — trees connected! Path uses ${data.path.length} nodes.`;
        else note = "Iteration budget exhausted without reaching the goal — try Generate new.";
        return { done, note, line: e.line };
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
      { color: START_COLOR, label: "tree edge (from start)" },
      { color: GOAL_TREE_COLOR, label: "tree edge (from goal, bidirectional)" },
      { color: SAMPLE_COLOR, label: "random sample q_rand (this step only)" },
      { color: CANDIDATE, label: "candidate q_new (untested)" },
      { color: REJECT, label: "blocked extension" },
      { color: "#2f8f5b", label: "path to goal" },
    ],
    params: [
      { key: "bidirectional", label: "bidirectional (grow T_start and T_goal)", type: "checkbox", value: false },
    ],
    pseudocode: [
      "sample random q_rand (occasionally q_goal -- goal bias)",
      "find the nearest existing tree node q_near",
      "extend toward q_rand by at most step size dq -> q_new",
      "add q_new if the segment is collision-free",
      "if q_new is close enough to the goal: connect, done",
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

# bidirectional variant: grow T_start and T_goal toward each other, alternating
# which tree extends (smaller tree first), and try connecting to the other
# tree's nearest node after every successful extension -- terminate on contact.
`,
  };
})();
