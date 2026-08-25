/* PRM visualization: sample free milestones, connect each to its k nearest
   neighbors with a collision-free local planner, then search the roadmap.

   Every candidate is shown before it's resolved: a hollow gold marker for a
   just-sampled point (or a dashed gold line for a just-proposed edge), then
   the very next step either solidifies it (accepted) or flashes it red
   (rejected) before moving on -- so the accept/reject decision is visible,
   not just its outcome. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  const CANDIDATE = "#e2b06a";
  const REJECT = "#c23b3b";

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
  function segmentFree(world, a, b, step) {
    step = step || 4;
    const d = dist(a, b), n = Math.max(1, Math.ceil(d / step));
    for (let i = 1; i < n; i++) {
      const t = i / n;
      if (!world.isFree(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return false;
    }
    return true;
  }
  function randPt(world, rng) {
    return { x: world.margin * 0.3 + rng() * (world.width - world.margin * 0.6), y: world.margin * 0.3 + rng() * (world.height - world.margin * 0.6) };
  }

  // pseudocode line indices (0-based) -- see vizDefs.prm.pseudocode below
  const L_SAMPLE = 0, L_NEIGHBORS = 1, L_ATTEMPT = 2, L_ADD_EDGE = 3, L_DENSE = 4, L_QUERY = 5;

  function computePRM(world, rng, N, k) {
    const milestones = [];
    const events = [];
    let attempts = 0;
    while (milestones.length < N && attempts < N * 8) {
      attempts++;
      const p = randPt(world, rng);
      events.push({ type: "candidate", pt: p, line: L_SAMPLE });
      if (world.isFree(p.x, p.y, 2)) {
        milestones.push(p);
        events.push({ type: "accept", pt: p, line: L_SAMPLE, milestoneIdx: milestones.length - 1 });
      } else {
        events.push({ type: "reject", pt: p, line: L_SAMPLE });
      }
    }
    events.push({ type: "dense", line: L_DENSE });

    const nodes = [{ ...world.start }, { ...world.goal }, ...milestones];
    const edgeSet = new Set();
    const edges = [];
    const adj = nodes.map(() => []);
    nodes.forEach((n, i) => {
      const others = nodes.map((m, j) => [j, dist(n, m)]).filter(([j]) => j !== i).sort((a, b) => a[1] - b[1]).slice(0, k);
      others.forEach(([j, w]) => {
        const key = i < j ? `${i}-${j}` : `${j}-${i}`;
        if (edgeSet.has(key)) return;
        edgeSet.add(key);
        events.push({ type: "edge-candidate", a: i, b: j, line: L_ATTEMPT });
        if (segmentFree(world, n, nodes[j])) {
          edges.push([i, j, w]);
          adj[i].push([j, w]); adj[j].push([i, w]);
          events.push({ type: "edge-accept", a: i, b: j, edgeIdx: edges.length - 1, line: L_ADD_EDGE });
        } else {
          events.push({ type: "edge-reject", a: i, b: j, line: L_ADD_EDGE });
        }
      });
    });

    const dd = new Array(nodes.length).fill(Infinity), prev = new Array(nodes.length).fill(-1), vis = new Array(nodes.length).fill(false);
    dd[0] = 0;
    for (let it = 0; it < nodes.length; it++) {
      let u = -1, best = Infinity;
      for (let i = 0; i < nodes.length; i++) if (!vis[i] && dd[i] < best) { best = dd[i]; u = i; }
      if (u === -1) break;
      vis[u] = true;
      for (const [v, w] of adj[u]) if (dd[u] + w < dd[v]) { dd[v] = dd[u] + w; prev[v] = u; }
    }
    let path = null;
    if (dd[1] < Infinity) { path = [1]; let c = 1; while (c !== 0) { c = prev[c]; path.push(c); } path.reverse(); }
    events.push({ type: "search", line: L_QUERY });

    // prefix counts so draw() can render cumulative accepted state in O(1)
    const milestoneCountAt = new Array(events.length);
    const edgeCountAt = new Array(events.length);
    let mc = 0, ec = 0;
    events.forEach((e, i) => {
      if (e.type === "accept") mc++;
      if (e.type === "edge-accept") ec++;
      milestoneCountAt[i] = mc; edgeCountAt[i] = ec;
    });

    return { nodes, milestones, edges, path, pathLen: dd[1], events, milestoneCountAt, edgeCountAt, N };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });
    const mRev = idx >= 0 ? data.milestoneCountAt[idx] : 0;
    const eRev = idx >= 0 ? data.edgeCountAt[idx] : 0;

    ctx.save();
    ctx.strokeStyle = "rgba(111,168,220,0.5)";
    ctx.lineWidth = 1;
    for (let k = 0; k < eRev; k++) {
      const [i, j] = data.edges[k];
      ctx.beginPath(); ctx.moveTo(data.nodes[i].x, data.nodes[i].y); ctx.lineTo(data.nodes[j].x, data.nodes[j].y); ctx.stroke();
    }
    ctx.restore();

    ctx.fillStyle = "#6a4fb0";
    for (let k = 0; k < mRev; k++) {
      const p = data.milestones[k];
      ctx.beginPath(); ctx.arc(p.x, p.y, 3, 0, Math.PI * 2); ctx.fill();
    }

    const cur = idx >= 0 ? data.events[idx] : null;
    if (cur) {
      if (cur.type === "candidate") {
        ctx.save();
        ctx.setLineDash([2, 2]); ctx.strokeStyle = CANDIDATE; ctx.lineWidth = 1.6;
        ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 5, 0, Math.PI * 2); ctx.stroke();
        ctx.restore();
      } else if (cur.type === "reject") {
        ctx.save();
        ctx.strokeStyle = REJECT; ctx.lineWidth = 1.8;
        const r = 5;
        ctx.beginPath(); ctx.moveTo(cur.pt.x - r, cur.pt.y - r); ctx.lineTo(cur.pt.x + r, cur.pt.y + r);
        ctx.moveTo(cur.pt.x + r, cur.pt.y - r); ctx.lineTo(cur.pt.x - r, cur.pt.y + r); ctx.stroke();
        ctx.restore();
      } else if (cur.type === "edge-candidate") {
        const a = data.nodes[cur.a], b = data.nodes[cur.b];
        ctx.save();
        ctx.setLineDash([3, 3]); ctx.strokeStyle = CANDIDATE; ctx.lineWidth = 1.6;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        ctx.restore();
      } else if (cur.type === "edge-reject") {
        const a = data.nodes[cur.a], b = data.nodes[cur.b];
        ctx.save();
        ctx.setLineDash([3, 3]); ctx.strokeStyle = REJECT; ctx.lineWidth = 1.8;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        ctx.restore();
      }
    }

    if (idx >= 0 && data.events[idx].type === "search" && data.path) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 3;
      ctx.beginPath();
      data.path.forEach((pIdx, k) => { const p = data.nodes[pIdx]; k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    }
    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height, world: sharedWorld }) {
    const world = sharedWorld || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const N = 55, k = 6;
    const data = computePRM(world, rng, N, k);
    let idx = -1;
    const total = data.events.length;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, total - 1);
        const done = idx >= total - 1;
        const e = data.events[idx];
        let note;
        if (e.type === "candidate") note = "Sampled a candidate configuration — testing whether it's collision-free.";
        else if (e.type === "accept") note = `Free — kept as milestone ${e.milestoneIdx + 1} of up to ${data.N}.`;
        else if (e.type === "reject") note = "In collision — discarded, not added to the roadmap.";
        else if (e.type === "dense") note = `Roadmap has ${data.milestones.length} milestones — dense enough. Connecting each to its ${k} nearest neighbors next.`;
        else if (e.type === "edge-candidate") note = "Attempting to connect a candidate pair — testing straight-line collision (local planner).";
        else if (e.type === "edge-accept") note = "Collision-free — edge added to the roadmap.";
        else if (e.type === "edge-reject") note = "Blocked by an obstacle — edge discarded.";
        else note = data.path ? `Roadmap complete. Shortest path over the graph ≈ ${data.pathLen.toFixed(0)}px through ${data.path.length} nodes.` : "Roadmap complete, but q_start and q_goal ended up in different components — try Generate new, or imagine adding more milestones.";
        return { done, note, line: e.line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.prm = {
    title: "PRM (Probabilistic Roadmap)",
    badge: "§8.3.1 / book §7.1",
    subtitle: "Sample free milestones, connect each to its k nearest neighbors, then search the resulting graph.",
    width: 560, height: 360,
    legend: [
      { color: "#6a4fb0", label: "milestone" },
      { color: "rgba(111,168,220,0.8)", label: "roadmap edge" },
      { color: CANDIDATE, label: "candidate (untested)" },
      { color: REJECT, label: "rejected" },
      { color: "#2f8f5b", label: "shortest path" },
    ],
    pseudocode: [
      "sample a random free configuration -> candidate milestone",
      "find its k-nearest (or radius-based) neighbor milestones",
      "attempt to connect each candidate pair with a local planner (straight-line collision check)",
      "add the edge if collision-free",
      { text: "repeat until the roadmap is “dense enough”", indent: 0 },
      "query: connect q_start, q_goal to the roadmap, search",
    ],
    makeSim,
    pythonCode: `
def build_prm(sample_free, is_free, n=55, k=6):
    milestones = [sample_free() for _ in range(n)]     # rejection-sample free configurations

    edges = []
    for i, a in enumerate(milestones):
        neighbors = k_nearest(milestones, a, k)
        for j in neighbors:
            if segment_free(a, milestones[j], is_free):  # local planner: discretized line check
                edges.append((i, j, dist(a, milestones[j])))

    return milestones, edges

def query(milestones, edges, start, goal):
    graph = connect(milestones, edges, start, goal)     # link start/goal into the roadmap
    return dijkstra(graph, start, goal)
`,
  };
})();
