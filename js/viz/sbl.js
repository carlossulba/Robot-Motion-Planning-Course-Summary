/* SBL (Single-query, Bi-directional, Lazy collision-checking) visualization:
   two EST-style trees (T_start, T_goal) grow by density-weighted node
   picking exactly like EST, but the edge to each new candidate is added
   WITHOUT a collision check (lazy -- drawn dashed here, since it's
   unverified). Only when a start-tree node and a goal-tree node land near
   each other does SBL propose a full candidate start->goal path through
   both trees' unverified edges, and only THEN checks every edge along that
   path. If any edge fails, the proposal is discarded and growth continues
   -- unlike PRM/EST/RRT, which validate every edge (or point) the instant
   it's created. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  const START_COLOR = "rgba(111,168,220,0.9)";
  const GOAL_TREE_COLOR = "rgba(31,156,138,0.9)";
  const CANDIDATE = "#e2b06a";
  const REJECT = "#c23b3b";
  const PROPOSE = "#e2b06a";

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
  function walkUp(tree, idx) {
    const arr = []; let c = idx;
    while (c !== -1) { arr.push(tree[c]); c = tree[c].parent; }
    return arr; // idx ... root
  }

  // pseudocode line indices (0-based) -- see vizDefs.sbl.pseudocode below
  const L_PICK = 0, L_LAZY_ADD = 1, L_PROPOSE = 2, L_VALIDATE = 3;
  const NEIGHBOR_R = 30, SAMPLE_R = 44, CONNECT_R = 34, MAX_ITER = 1400;

  function computeSBL(world, rng) {
    const events = [];
    const treeS = [{ x: world.start.x, y: world.start.y, parent: -1 }];
    const treeG = [{ x: world.goal.x, y: world.goal.y, parent: -1 }];
    const trees = [treeS, treeG];
    let success = false, path = null;

    for (let iter = 0; iter < MAX_ITER; iter++) {
      const ext = trees[0].length <= trees[1].length ? 0 : 1;
      const other = 1 - ext;
      const arr = trees[ext];
      const pickIdx = pickDensityWeighted(arr, rng, NEIGHBOR_R);
      const picked = arr[pickIdx];
      events.push({ type: "pick", idx: pickIdx, tree: ext, line: L_PICK });
      const cand = sampleNear(picked, rng, SAMPLE_R);
      events.push({ type: "candidate", from: { x: picked.x, y: picked.y }, pt: cand, tree: ext, line: L_LAZY_ADD });

      if (!world.isFree(cand.x, cand.y)) {
        events.push({ type: "reject", from: { x: picked.x, y: picked.y }, pt: cand, line: L_LAZY_ADD });
        continue;
      }
      // Added WITHOUT checking the connecting segment -- lazy. Drawn dashed
      // (unverified) until it's part of a fully-validated path.
      arr.push({ x: cand.x, y: cand.y, parent: pickIdx });
      const newIdx = arr.length - 1;
      events.push({ type: "add", idx: newIdx, tree: ext, line: L_LAZY_ADD });

      const oarr = trees[other];
      const { idx: nB, d: dB } = nearestIdx(oarr, cand);
      if (dB < CONNECT_R) {
        const sIdx = ext === 0 ? newIdx : nB;
        const gIdx = ext === 0 ? nB : newIdx;
        const startPart = walkUp(treeS, sIdx).reverse(); // start ... sIdx
        const goalPart = walkUp(treeG, gIdx);             // gIdx ... goal
        const full = startPart.concat(goalPart);
        events.push({ type: "propose", pts: full.slice(), line: L_PROPOSE });

        let failEdge = null;
        for (let i = 0; i < full.length - 1; i++) {
          if (!segmentFree(world, full[i], full[i + 1])) { failEdge = [full[i], full[i + 1]]; break; }
        }
        if (!failEdge) {
          events.push({ type: "validated", pts: full.slice(), line: L_VALIDATE });
          success = true;
          path = full;
          break;
        }
        events.push({ type: "path-reject", edge: failEdge, line: L_VALIDATE });
      }
    }
    if (!success) events.push({ type: "fail" });
    return { trees: [{ nodes: treeS, color: START_COLOR }, { nodes: treeG, color: GOAL_TREE_COLOR }], events, path, success };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });

    ctx.save();
    ctx.setLineDash([4, 3]);
    ctx.lineWidth = 1.1;
    for (let k = 0; k <= idx && k < data.events.length; k++) {
      const e = data.events[k];
      if (e.type === "add") {
        const tr = data.trees[e.tree].nodes;
        const n = tr[e.idx], p = tr[n.parent];
        ctx.strokeStyle = data.trees[e.tree].color;
        ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(n.x, n.y); ctx.stroke();
      }
    }
    ctx.setLineDash([]);
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
      ctx.strokeStyle = REJECT; ctx.lineWidth = 1.8;
      const r = 5, p = cur.pt;
      ctx.beginPath(); ctx.moveTo(p.x - r, p.y - r); ctx.lineTo(p.x + r, p.y + r);
      ctx.moveTo(p.x + r, p.y - r); ctx.lineTo(p.x - r, p.y + r); ctx.stroke();
      ctx.restore();
    } else if (cur && cur.type === "propose") {
      ctx.save();
      ctx.setLineDash([5, 3]); ctx.strokeStyle = PROPOSE; ctx.lineWidth = 2.2;
      ctx.beginPath();
      cur.pts.forEach((p, k) => { k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    } else if (cur && cur.type === "path-reject") {
      ctx.save();
      ctx.strokeStyle = REJECT; ctx.lineWidth = 2.6; ctx.setLineDash([]);
      const [a, b] = cur.edge;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      ctx.restore();
    }

    if (data.path && idx >= 0 && (data.events[idx].type === "validated" || idx >= data.events.length - 1) && data.success) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 3; ctx.setLineDash([]);
      ctx.beginPath();
      data.path.forEach((p, k) => { k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height, world: sharedWorld }) {
    const world = sharedWorld || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const data = computeSBL(world, rng);
    let idx = -1;
    const total = data.events.length;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, total - 1);
        const done = idx >= total - 1;
        const e = data.events[idx];
        const treeName = (t) => (t === 0 ? "T_start" : "T_goal");
        let note;
        if (e.type === "pick") note = `Picked an existing node of ${treeName(e.tree)} (density-weighted, as in EST).`;
        else if (e.type === "candidate") note = "Sampled a nearby candidate configuration.";
        else if (e.type === "reject") note = "Candidate itself is in collision — discarded.";
        else if (e.type === "add") note = `Added to ${treeName(e.tree)} WITHOUT checking the connecting edge yet (lazy — dashed).`;
        else if (e.type === "propose") note = "A start-tree and goal-tree node landed close together — proposing this full start-to-goal path.";
        else if (e.type === "validated") note = `Every edge along the proposed path is collision-free — path found! Uses ${data.path.length} nodes.`;
        else if (e.type === "path-reject") note = "One edge along the proposed path is blocked — discard this path (not the whole tree) and keep growing.";
        else note = "Iteration budget exhausted without finding a valid path — try Generate new.";
        return { done, note, line: e.line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.sbl = {
    title: "SBL (Single-query, Bi-directional, Lazy)",
    badge: "§8.4.2 / book §7.2.3",
    subtitle: "Two EST-style trees grow with unverified edges; only when start and goal trees touch is the whole candidate path checked for collisions.",
    width: 560, height: 360,
    legend: [
      { color: START_COLOR, label: "T_start edge (unverified, dashed)" },
      { color: GOAL_TREE_COLOR, label: "T_goal edge (unverified, dashed)" },
      { color: CANDIDATE, label: "candidate (untested)" },
      { color: REJECT, label: "rejected candidate / edge" },
      { color: "#2f8f5b", label: "validated path" },
    ],
    pseudocode: [
      "pick an existing node (density-weighted, as in EST)",
      "sample and add a candidate WITHOUT checking the edge for collisions yet (lazy)",
      "when a start-tree node and goal-tree node become close, propose a candidate connecting path",
      { text: "only now check every edge along that candidate path for collisions -- if any edge fails, discard it and keep growing instead of re-checking everything", indent: 0 },
    ],
    makeSim,
    pythonCode: `
def sbl_step(tree, other_tree, is_free, connect_radius=34):
    picked = weighted_choice_by_sparsity(tree)         # density-weighted, as in EST
    candidate = sample_near(picked)
    if not is_free(candidate):
        return None
    node = Node(candidate, parent=picked)              # edge added WITHOUT a collision check
    tree.append(node)

    nearest_other = nearest(other_tree, candidate)
    if dist(candidate, nearest_other) < connect_radius:
        path = chain_to_root(node) + chain_to_root(nearest_other)   # candidate path
        if all(segment_free(a, b, is_free) for a, b in pairs(path)):
            return path                                 # only now checked, all at once
        # else: discard this proposal, keep growing -- don't re-check the whole tree
    return None
`,
  };
})();
