/* SRT (Sampling-Based Roadmap of Trees) visualization: instead of one
   roadmap of point milestones (PRM) or one tree from q_start (RRT), SRT
   grows several small trees from random roots ("add trees"), then tries to
   connect each tree to its nearest and a couple of random neighboring
   trees using the same tree-planner logic used to grow them ("add edges"),
   growing both trees a little further toward each other if a direct
   connection is blocked. Two trees merge into one roadmap component the
   moment any configuration in one connects to any configuration in the
   other -- q_start and q_goal are just two more trees that need to end up
   in the same component. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  const CANDIDATE = "#e2b06a";
  const REJECT = "#c23b3b";
  const MERGE = "#b7532c";

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
  function sampleFreePt(world, rng) {
    for (let tries = 0; tries < 300; tries++) {
      const p = randPt(world, rng);
      if (world.isFree(p.x, p.y, 2)) return p;
    }
    return { x: world.width / 2, y: world.height / 2 };
  }
  function makeUF(n) {
    const parent = Array.from({ length: n }, (_, i) => i);
    function find(x) { while (parent[x] !== x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
    function union(a, b) { a = find(a); b = find(b); if (a !== b) parent[a] = b; }
    return { find, union };
  }
  function treeColor(i) { return `hsl(${Math.round((i * 360) / 6) % 360}, 62%, 50%)`; }
  function treeHalo(i) { return `hsla(${Math.round((i * 360) / 6) % 360}, 70%, 65%, 0.16)`; }

  // pseudocode line indices (0-based) -- see vizDefs.srt.pseudocode below
  const L_ADD_TREES = 0, L_ADD_EDGES = 1, L_MERGE = 2;
  const STEP = 14;

  // Grows `nodes` (already seeded with the root at index 0) up to K nodes
  // using a small RRT-style tree planner; pushes a "grow" event per new node.
  function growTree(world, rng, nodes, K, treeIdx, events, retry) {
    let guard = 0;
    while (nodes.length < K && guard < K * 15) {
      guard++;
      const sample = randPt(world, rng);
      const { idx: nearest, d: bestD } = nearestIdx(nodes, sample);
      const d = Math.max(1e-6, bestD), t = Math.min(STEP, d) / d;
      const newPt = { x: nodes[nearest].x + (sample.x - nodes[nearest].x) * t, y: nodes[nearest].y + (sample.y - nodes[nearest].y) * t };
      if (!world.inBounds(newPt.x, newPt.y) || !segmentFree(world, nodes[nearest], newPt)) continue;
      nodes.push({ x: newPt.x, y: newPt.y, parent: nearest });
      events.push({ type: "grow", tree: treeIdx, idx: nodes.length - 1, retry: !!retry, line: L_ADD_TREES });
    }
  }

  function growToward(world, rng, nodes, fromIdx, target, treeIdx, events) {
    const from = nodes[fromIdx];
    const d = dist(from, target);
    if (d < 1e-6) return fromIdx;
    const t = Math.min(STEP, d) / d;
    const newPt = { x: from.x + (target.x - from.x) * t, y: from.y + (target.y - from.y) * t };
    if (!world.inBounds(newPt.x, newPt.y) || !segmentFree(world, from, newPt)) return fromIdx;
    nodes.push({ x: newPt.x, y: newPt.y, parent: fromIdx });
    const newIdx = nodes.length - 1;
    events.push({ type: "grow", tree: treeIdx, idx: newIdx, retry: true, line: L_ADD_TREES });
    return newIdx;
  }

  function nearestTreePair(treesArr, i, j) {
    let bestD = Infinity, bestA = 0, bestB = 0;
    treesArr[i].nodes.forEach((na, ai) => {
      treesArr[j].nodes.forEach((nb, bi) => {
        const d = dist(na, nb);
        if (d < bestD) { bestD = d; bestA = ai; bestB = bi; }
      });
    });
    return { a: bestA, b: bestB, d: bestD };
  }

  function computeSRT(world, rng) {
    const M = 6, K = 11;
    const events = [];
    const treesArr = [];

    for (let i = 0; i < M; i++) {
      const root = sampleFreePt(world, rng);
      const nodes = [{ x: root.x, y: root.y, parent: -1 }];
      events.push({ type: "root", tree: i, idx: 0, line: L_ADD_TREES });
      growTree(world, rng, nodes, K, i, events, false);
      treesArr.push({ nodes, color: treeColor(i), halo: treeHalo(i), isTree: true });
    }
    const startTreeIdx = M, goalTreeIdx = M + 1;
    treesArr.push({ nodes: [{ x: world.start.x, y: world.start.y, parent: -1 }], color: "#2b6cb0" });
    treesArr.push({ nodes: [{ x: world.goal.x, y: world.goal.y, parent: -1 }], color: "#2f8f5b" });
    events.push({ type: "root", tree: startTreeIdx, idx: 0, line: L_ADD_TREES, isEndpoint: true });
    events.push({ type: "root", tree: goalTreeIdx, idx: 0, line: L_ADD_TREES, isEndpoint: true });

    const total = treesArr.length;
    const uf = makeUF(total);
    const mergeEdges = [];

    // candidate pairs: each tree's nearest neighbor + one random neighbor
    const pairSet = new Set();
    const candidatePairs = [];
    function addPair(i, j) {
      if (i === j) return;
      const key = i < j ? `${i}-${j}` : `${j}-${i}`;
      if (pairSet.has(key)) return;
      pairSet.add(key);
      candidatePairs.push([i, j]);
    }
    for (let i = 0; i < total; i++) {
      let bestJ = -1, bestD = Infinity;
      for (let j = 0; j < total; j++) {
        if (j === i) continue;
        const { d } = nearestTreePair(treesArr, i, j);
        if (d < bestD) { bestD = d; bestJ = j; }
      }
      if (bestJ !== -1) addPair(i, bestJ);
      let rj = Math.floor(rng() * total);
      if (rj === i) rj = (rj + 1) % total;
      addPair(i, rj);
    }
    candidatePairs.forEach((p) => { p[2] = nearestTreePair(treesArr, p[0], p[1]).d; });
    candidatePairs.sort((a, b) => a[2] - b[2]);

    function treeLabel(i) {
      if (i === startTreeIdx) return "the start component";
      if (i === goalTreeIdx) return "the goal component";
      return `tree ${i + 1}`;
    }

    candidatePairs.forEach(([i, j]) => {
      if (uf.find(i) === uf.find(j)) return; // already the same roadmap component
      let { a, b } = nearestTreePair(treesArr, i, j);
      let A = treesArr[i].nodes[a], B = treesArr[j].nodes[b];
      events.push({ type: "edge-attempt", treeA: i, a, treeB: j, b, labelA: treeLabel(i), labelB: treeLabel(j), line: L_ADD_EDGES });

      if (segmentFree(world, A, B)) {
        uf.union(i, j);
        mergeEdges.push({ treeA: i, a, treeB: j, b });
        events.push({ type: "merge", treeA: i, a, treeB: j, b, labelA: treeLabel(i), labelB: treeLabel(j), line: L_MERGE });
        return;
      }
      events.push({ type: "edge-blocked", treeA: i, a, treeB: j, b, line: L_ADD_EDGES });

      const a2 = growToward(world, rng, treesArr[i].nodes, a, B, i, events);
      const b2 = growToward(world, rng, treesArr[j].nodes, b, A, j, events);
      A = treesArr[i].nodes[a2]; B = treesArr[j].nodes[b2];
      events.push({ type: "edge-attempt", treeA: i, a: a2, treeB: j, b: b2, labelA: treeLabel(i), labelB: treeLabel(j), line: L_ADD_EDGES });
      if (segmentFree(world, A, B)) {
        uf.union(i, j);
        mergeEdges.push({ treeA: i, a: a2, treeB: j, b: b2 });
        events.push({ type: "merge", treeA: i, a: a2, treeB: j, b: b2, labelA: treeLabel(i), labelB: treeLabel(j), line: L_MERGE });
      } else {
        events.push({ type: "edge-blocked", treeA: i, a: a2, treeB: j, b: b2, line: L_ADD_EDGES });
      }
    });

    const connected = uf.find(startTreeIdx) === uf.find(goalTreeIdx);
    let path = null, pathLen = 0;
    if (connected) {
      const offsets = []; let acc = 0;
      treesArr.forEach((t) => { offsets.push(acc); acc += t.nodes.length; });
      const totalNodes = acc;
      const adj = Array.from({ length: totalNodes }, () => []);
      treesArr.forEach((t, ti) => {
        t.nodes.forEach((n, ni) => {
          if (n.parent !== -1) {
            const u = offsets[ti] + ni, v = offsets[ti] + n.parent;
            const w = dist(n, t.nodes[n.parent]);
            adj[u].push([v, w]); adj[v].push([u, w]);
          }
        });
      });
      mergeEdges.forEach((e) => {
        const u = offsets[e.treeA] + e.a, v = offsets[e.treeB] + e.b;
        const w = dist(treesArr[e.treeA].nodes[e.a], treesArr[e.treeB].nodes[e.b]);
        adj[u].push([v, w]); adj[v].push([u, w]);
      });
      const S = offsets[startTreeIdx], G = offsets[goalTreeIdx];
      const dd = new Array(totalNodes).fill(Infinity), prev = new Array(totalNodes).fill(-1), vis = new Array(totalNodes).fill(false);
      dd[S] = 0;
      for (let it = 0; it < totalNodes; it++) {
        let u = -1, best = Infinity;
        for (let ii = 0; ii < totalNodes; ii++) if (!vis[ii] && dd[ii] < best) { best = dd[ii]; u = ii; }
        if (u === -1) break;
        vis[u] = true;
        for (const [v, w] of adj[u]) if (dd[u] + w < dd[v]) { dd[v] = dd[u] + w; prev[v] = u; }
      }
      if (dd[G] < Infinity) {
        function globalToPoint(g) {
          for (let ti = treesArr.length - 1; ti >= 0; ti--) if (g >= offsets[ti]) return treesArr[ti].nodes[g - offsets[ti]];
        }
        const idxPath = [G]; let c = G;
        while (c !== S) { c = prev[c]; idxPath.push(c); }
        idxPath.reverse();
        path = idxPath.map(globalToPoint);
        pathLen = dd[G];
      }
    }
    events.push({ type: "search", connected, line: -1 });

    return { trees: treesArr, events, path, pathLen, connected, M, startTreeIdx, goalTreeIdx };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });

    const revealed = data.trees.map(() => new Set());
    const revealedMerges = [];
    for (let k = 0; k <= idx && k < data.events.length; k++) {
      const e = data.events[k];
      if (e.type === "root" || e.type === "grow") revealed[e.tree].add(e.idx);
      if (e.type === "merge") revealedMerges.push(e);
    }

    // pastel halos + within-tree edges + nodes for the M generated trees
    data.trees.forEach((tree, ti) => {
      if (!tree.isTree) return;
      const rset = revealed[ti];
      if (!rset.size) return;
      ctx.save();
      ctx.fillStyle = tree.halo;
      rset.forEach((ni) => { const n = tree.nodes[ni]; ctx.beginPath(); ctx.arc(n.x, n.y, 15, 0, Math.PI * 2); ctx.fill(); });
      ctx.restore();
    });
    data.trees.forEach((tree, ti) => {
      const rset = revealed[ti];
      if (!rset.size) return;
      ctx.save();
      ctx.strokeStyle = tree.color; ctx.lineWidth = 1.2;
      rset.forEach((ni) => {
        const n = tree.nodes[ni];
        if (n.parent !== -1 && rset.has(n.parent)) {
          const p = tree.nodes[n.parent];
          ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(n.x, n.y); ctx.stroke();
        }
      });
      ctx.fillStyle = tree.color;
      if (tree.isTree) {
        rset.forEach((ni) => { const n = tree.nodes[ni]; ctx.beginPath(); ctx.arc(n.x, n.y, 2.6, 0, Math.PI * 2); ctx.fill(); });
      }
      ctx.restore();
    });

    // merge edges, permanent once reached
    ctx.save();
    ctx.strokeStyle = MERGE; ctx.lineWidth = 2.4; ctx.setLineDash([6, 4]);
    revealedMerges.forEach((e) => {
      const a = data.trees[e.treeA].nodes[e.a], b = data.trees[e.treeB].nodes[e.b];
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    });
    ctx.restore();

    const cur = idx >= 0 ? data.events[idx] : null;
    if (cur && cur.type === "edge-attempt") {
      const a = data.trees[cur.treeA].nodes[cur.a], b = data.trees[cur.treeB].nodes[cur.b];
      ctx.save();
      ctx.setLineDash([3, 3]); ctx.strokeStyle = CANDIDATE; ctx.lineWidth = 1.8;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      ctx.restore();
    } else if (cur && cur.type === "edge-blocked") {
      const a = data.trees[cur.treeA].nodes[cur.a], b = data.trees[cur.treeB].nodes[cur.b];
      ctx.save();
      ctx.setLineDash([3, 3]); ctx.strokeStyle = REJECT; ctx.lineWidth = 1.8;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      ctx.restore();
    }

    if (data.path && idx >= 0 && (data.events[idx].type === "search" && data.connected)) {
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
    const data = computeSRT(world, rng);
    let idx = -1;
    const total = data.events.length;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, total - 1);
        const done = idx >= total - 1;
        const e = data.events[idx];
        let note;
        if (e.type === "root") note = e.isEndpoint ? `${e.tree === data.startTreeIdx ? "q_start" : "q_goal"} treated as its own single-node component to be merged into the roadmap.` : `Sampled a root uniformly in free space for tree ${e.tree + 1} of ${data.M}.`;
        else if (e.type === "grow") note = e.retry ? `Growing tree ${e.tree + 1} one step further, toward a neighboring tree, to try to bridge a blocked connection.` : `Growing tree ${e.tree + 1} from its root with a tree planner (RRT-style extension).`;
        else if (e.type === "edge-attempt") note = `Attempting to connect ${e.labelA} and ${e.labelB} via their closest pair of nodes.`;
        else if (e.type === "edge-blocked") note = "Blocked by an obstacle — grow both trees a little further toward each other and retry.";
        else if (e.type === "merge") note = `Connected! ${e.labelA} and ${e.labelB} merge into one roadmap component (thick dashed edge).`;
        else note = data.connected ? `q_start and q_goal ended up in the same roadmap component — path found, length ≈ ${data.pathLen.toFixed(0)}px.` : "q_start and q_goal are still in different components after this pass — try Generate new.";
        return { done, note, line: e.line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.srt = {
    title: "SRT (Sampling-Based Roadmap of Trees)",
    badge: "§8.6 / book §7.2.1",
    subtitle: "Grow several small trees from random roots, then connect nearby trees with the same tree planner — merging into one roadmap wherever two trees touch.",
    width: 560, height: 360,
    legend: [
      { color: "hsl(200,62%,50%)", label: "tree node/edge (distinct color per tree)" },
      { color: CANDIDATE, label: "connection attempt (untested)" },
      { color: REJECT, label: "blocked attempt" },
      { color: MERGE, label: "merge (trees connected)" },
      { color: "#2f8f5b", label: "start-to-goal path" },
    ],
    pseudocode: [
      "add trees: sample a root uniformly in free space, then grow a tree from it with a tree planner (EST/RRT)",
      { text: "add edges: for each tree, find its closest and a few random neighboring trees; attempt to connect via the same tree planner, growing both trees further if needed", indent: 0 },
      "two trees merge into one roadmap component as soon as any configuration in one connects to any configuration in the other",
    ],
    makeSim,
    pythonCode: `
def build_srt(is_free, sample_free, n_trees=6, nodes_per_tree=11):
    trees = [grow_tree(sample_free(), is_free, nodes_per_tree) for _ in range(n_trees)]
    trees += [Tree(start), Tree(goal)]              # start/goal are single-node trees too

    components = UnionFind(len(trees))
    for i, tree in enumerate(trees):
        candidates = [nearest_tree(trees, i)] + random_sample(other_trees(trees, i), k=1)
        for j in candidates:
            if components.find(i) == components.find(j):
                continue
            a, b = closest_pair(tree, trees[j])
            if not segment_free(a, b, is_free):
                a = extend_toward(tree, a, b, is_free)      # grow both trees a step closer
                b = extend_toward(trees[j], b, a, is_free)
            if segment_free(a, b, is_free):
                components.union(i, j)                       # merge into one component

    if components.find(start_tree) == components.find(goal_tree):
        return shortest_path(trees, components, start, goal)
`,
  };
})();
