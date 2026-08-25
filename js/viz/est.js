/* EST (Expansive-Spaces Trees) visualization: unlike RRT (steer from the
   NEAREST node toward a random sample), EST picks an existing node with
   probability inversely related to its local neighbor density -- favoring
   sparse regions of the tree so it expands evenly into open space -- then
   samples a candidate nearby and keeps it ONLY IF a local planner connects
   it to the picked node (PRM instead keeps every collision-free sample
   unconditionally).

   The candidate is always shown hollow for one step before it resolves
   (added, or discarded on a failed local-planner check). The
   "bidirectional" toggle grows T_start and T_goal as two EST trees and
   merges them the moment a newly added node in one tree lands near enough
   to connect straight to the other. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  const START_COLOR = "rgba(111,168,220,0.9)";
  const GOAL_TREE_COLOR = "rgba(31,156,138,0.9)";
  const CANDIDATE = "#e2b06a";
  const REJECT = "#c23b3b";

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
  function nearestIdx(nodes, pt) {
    let best = 0, bestD = Infinity;
    for (let i = 0; i < nodes.length; i++) { const d = dist(nodes[i], pt); if (d < bestD) { bestD = d; best = i; } }
    return { idx: best, d: bestD };
  }
  // Pick a node with probability inversely related to its local neighbor
  // density (a lightly-populated node is more likely to be picked, biasing
  // growth into open space rather than piling up where the tree is dense).
  function pickDensityWeighted(nodes, rng, R) {
    const weights = nodes.map((n) => {
      let cnt = 0;
      for (const m of nodes) if (m !== n && dist(n, m) < R) cnt++;
      return 1 / (cnt + 1);
    });
    const total = weights.reduce((a, b) => a + b, 0);
    let r = rng() * total;
    for (let i = 0; i < weights.length; i++) { r -= weights[i]; if (r <= 0) return i; }
    return weights.length - 1;
  }
  function sampleNear(node, rng, radius) {
    const r = radius * Math.sqrt(rng());
    const a = rng() * Math.PI * 2;
    return { x: node.x + Math.cos(a) * r, y: node.y + Math.sin(a) * r };
  }

  // pseudocode line indices (0-based) -- see vizDefs.est.pseudocode below
  const L_PICK = 0, L_SAMPLE = 1, L_ADD = 2, L_MERGE = 3;
  const NEIGHBOR_R = 42, SAMPLE_R = 26, GOAL_BIAS = 0.07, MERGE_R = 22, MAX_ITER = 900;

  function computeEST(world, rng, bidirectional) {
    const events = [];

    if (!bidirectional) {
      const nodes = [{ x: world.start.x, y: world.start.y, parent: -1 }];
      let success = false, goalIdx = -1;

      for (let iter = 0; iter < MAX_ITER; iter++) {
        const pickIdx = pickDensityWeighted(nodes, rng, NEIGHBOR_R);
        const picked = nodes[pickIdx];
        events.push({ type: "pick", idx: pickIdx, tree: 0, line: L_PICK });
        const cand = rng() < GOAL_BIAS ? { x: world.goal.x, y: world.goal.y } : sampleNear(picked, rng, SAMPLE_R);
        events.push({ type: "candidate", from: { x: picked.x, y: picked.y }, pt: cand, tree: 0, line: L_SAMPLE });

        const valid = world.isFree(cand.x, cand.y) && segmentFree(world, picked, cand);
        if (!valid) { events.push({ type: "reject", from: { x: picked.x, y: picked.y }, pt: cand, line: L_ADD }); continue; }
        nodes.push({ x: cand.x, y: cand.y, parent: pickIdx });
        const newIdx = nodes.length - 1;
        events.push({ type: "add", idx: newIdx, tree: 0, line: L_ADD });

        if (dist(cand, world.goal) < SAMPLE_R * 0.9 && segmentFree(world, cand, world.goal)) {
          nodes.push({ x: world.goal.x, y: world.goal.y, parent: newIdx });
          goalIdx = nodes.length - 1;
          events.push({ type: "add", idx: goalIdx, tree: 0, isGoal: true, line: L_ADD });
          success = true;
          break;
        }
      }
      if (!success) events.push({ type: "fail" });
      let path = null;
      if (success) { path = []; let c = goalIdx; while (c !== -1) { path.push(nodes[c]); c = nodes[c].parent; } path.reverse(); }
      return { mode: "single", trees: [{ nodes, color: START_COLOR }], events, path, success };
    }

    // ---- bidirectional: T_start and T_goal, each grown EST-style ----
    const treeS = [{ x: world.start.x, y: world.start.y, parent: -1 }];
    const treeG = [{ x: world.goal.x, y: world.goal.y, parent: -1 }];
    const trees = [treeS, treeG];
    let success = false, connS = -1, connG = -1;

    for (let iter = 0; iter < MAX_ITER; iter++) {
      const ext = trees[0].length <= trees[1].length ? 0 : 1;
      const other = 1 - ext;
      const arr = trees[ext];
      const pickIdx = pickDensityWeighted(arr, rng, NEIGHBOR_R);
      const picked = arr[pickIdx];
      events.push({ type: "pick", idx: pickIdx, tree: ext, line: L_PICK });
      const cand = sampleNear(picked, rng, SAMPLE_R);
      events.push({ type: "candidate", from: { x: picked.x, y: picked.y }, pt: cand, tree: ext, line: L_SAMPLE });

      const valid = world.isFree(cand.x, cand.y) && segmentFree(world, picked, cand);
      if (!valid) { events.push({ type: "reject", from: { x: picked.x, y: picked.y }, pt: cand, line: L_ADD }); continue; }
      arr.push({ x: cand.x, y: cand.y, parent: pickIdx });
      const newIdx = arr.length - 1;
      events.push({ type: "add", idx: newIdx, tree: ext, line: L_ADD });

      const oarr = trees[other];
      const { idx: nB, d: dB } = nearestIdx(oarr, cand);
      if (dB < MERGE_R && segmentFree(world, cand, oarr[nB])) {
        events.push({ type: "connect", treeA: ext, idxA: newIdx, treeB: other, idxB: nB, line: L_MERGE });
        success = true;
        connS = ext === 0 ? newIdx : nB;
        connG = ext === 0 ? nB : newIdx;
        break;
      }
    }
    if (!success) events.push({ type: "fail" });
    let path = null;
    if (success) {
      const pS = []; let c = connS; while (c !== -1) { pS.push(treeS[c]); c = treeS[c].parent; } pS.reverse();
      const pG = []; c = connG; while (c !== -1) { pG.push(treeG[c]); c = treeG[c].parent; }
      path = pS.concat(pG);
    }
    return { mode: "bidirectional", trees: [{ nodes: treeS, color: START_COLOR }, { nodes: treeG, color: GOAL_TREE_COLOR }], events, path, success };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });

    ctx.save();
    ctx.lineWidth = 1.2;
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
        ctx.lineWidth = 1.2;
      }
    }
    ctx.restore();

    const cur = idx >= 0 ? data.events[idx] : null;
    if (cur && cur.type === "pick") {
      const tr = data.trees[cur.tree].nodes;
      const p = tr[cur.idx];
      ctx.save();
      ctx.strokeStyle = data.trees[cur.tree].color; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(p.x, p.y, 7, 0, Math.PI * 2); ctx.stroke();
      ctx.restore();
    } else if (cur && cur.type === "candidate") {
      ctx.save();
      ctx.setLineDash([2, 2]); ctx.strokeStyle = CANDIDATE; ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.moveTo(cur.from.x, cur.from.y); ctx.lineTo(cur.pt.x, cur.pt.y); ctx.stroke();
      ctx.setLineDash([]);
      ctx.strokeStyle = CANDIDATE; ctx.lineWidth = 1.8;
      ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 5, 0, Math.PI * 2); ctx.stroke();
      ctx.restore();
    } else if (cur && cur.type === "reject") {
      ctx.save();
      ctx.setLineDash([3, 3]); ctx.strokeStyle = REJECT; ctx.lineWidth = 1.4;
      ctx.beginPath(); ctx.moveTo(cur.from.x, cur.from.y); ctx.lineTo(cur.pt.x, cur.pt.y); ctx.stroke();
      ctx.setLineDash([]);
      ctx.strokeStyle = REJECT; ctx.lineWidth = 1.8;
      const r = 5, p = cur.pt;
      ctx.beginPath(); ctx.moveTo(p.x - r, p.y - r); ctx.lineTo(p.x + r, p.y + r);
      ctx.moveTo(p.x + r, p.y - r); ctx.lineTo(p.x - r, p.y + r); ctx.stroke();
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
    const data = computeEST(world, rng, bidirectional);
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
        if (e.type === "pick") note = `Picked an existing node of ${treeName(e.tree)} — sparser neighborhoods are more likely to be picked.`;
        else if (e.type === "candidate") note = "Sampled a candidate near the picked node. Testing the local planner...";
        else if (e.type === "reject") note = "Local planner failed (candidate or connecting segment in collision) — discarded, unlike PRM this candidate is NOT kept.";
        else if (e.type === "add" && e.isGoal) note = `Candidate reached q_goal directly — connected! Path uses ${data.path.length} nodes.`;
        else if (e.type === "add") note = `Local planner connected — candidate added to ${treeName(e.tree)} (${data.trees[e.tree].nodes.length} nodes so far).`;
        else if (e.type === "connect") note = `New node landed close enough to merge with the other tree — T_start and T_goal connected! Path uses ${data.path.length} nodes.`;
        else note = "Iteration budget exhausted without reaching the goal — try Generate new.";
        return { done, note, line: e.line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.est = {
    title: "EST (Expansive-Spaces Trees)",
    badge: "§8.4.2 / book §7.2.1",
    subtitle: "Grow a tree by picking sparsely-covered nodes to expand from, keeping a nearby candidate only if a local planner connects it back.",
    width: 560, height: 360,
    legend: [
      { color: START_COLOR, label: "tree edge (from start)" },
      { color: GOAL_TREE_COLOR, label: "tree edge (from goal, bidirectional)" },
      { color: CANDIDATE, label: "candidate (untested)" },
      { color: REJECT, label: "rejected (local planner failed)" },
      { color: "#2f8f5b", label: "path to goal" },
    ],
    params: [
      { key: "bidirectional", label: "bidirectional (grow T_start and T_goal)", type: "checkbox", value: false },
    ],
    pseudocode: [
      "pick an existing node with probability inversely related to local neighbor density",
      "sample a candidate configuration near the picked node",
      { text: "add it ONLY IF a local planner connects it to the picked node (unlike PRM, which keeps every collision-free sample unconditionally)", indent: 0 },
      "if bidirectional: merge trees by connecting a newly added node in one to its nearest neighbors in the other",
    ],
    makeSim,
    pythonCode: `
def est_step(tree, is_free, sample_disk, neighbor_radius=42, sample_radius=26):
    weights = [1 / (count_neighbors(tree, n, neighbor_radius) + 1) for n in tree]
    picked = weighted_choice(tree, weights)          # sparsely-covered nodes favored

    candidate = sample_disk(picked, sample_radius)
    if not is_free(candidate) or not segment_free(picked, candidate, is_free):
        return None                                  # local planner failed -- discard

    return Node(candidate, parent=picked)             # kept only because it connects

# bidirectional variant: grow T_start and T_goal with est_step() in turn; after
# every successful add, try connecting the new node to the OTHER tree's
# nearest node -- merge (done) the moment that connection is collision-free.
`,
  };
})();
