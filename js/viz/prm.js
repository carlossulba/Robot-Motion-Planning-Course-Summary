/* PRM visualization: sample free milestones, connect each to its k nearest
   neighbors with a collision-free local planner, then search the roadmap. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

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

  function computePRM(world, N, k) {
    const milestones = [];
    let attempts = 0;
    while (milestones.length < N && attempts < N * 8) {
      attempts++;
      const p = { x: world.margin * 0.3 + Math.random() * (world.width - world.margin * 0.6), y: world.margin * 0.3 + Math.random() * (world.height - world.margin * 0.6) };
      if (world.isFree(p.x, p.y, 2)) milestones.push(p);
    }
    const nodes = [{ ...world.start }, { ...world.goal }, ...milestones];
    const edgeSet = new Set();
    const edges = [];
    const adj = nodes.map(() => []);
    nodes.forEach((n, i) => {
      const others = nodes.map((m, j) => [j, dist(n, m)]).filter(([j]) => j !== i).sort((a, b) => a[1] - b[1]).slice(0, k);
      others.forEach(([j, w]) => {
        const key = i < j ? `${i}-${j}` : `${j}-${i}`;
        if (edgeSet.has(key)) return;
        if (segmentFree(world, n, nodes[j])) {
          edgeSet.add(key);
          edges.push([i, j, w]);
          adj[i].push([j, w]); adj[j].push([i, w]);
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

    return { nodes, milestones, edges, path, pathLen: dd[1] };
  }

  function draw(ctx, world, data, milestonesRevealed, edgesRevealed, showPath) {
    world.draw(ctx, { alpha: 0.9 });
    ctx.save();
    ctx.strokeStyle = "rgba(111,168,220,0.5)";
    ctx.lineWidth = 1;
    for (let k = 0; k < edgesRevealed && k < data.edges.length; k++) {
      const [i, j] = data.edges[k];
      ctx.beginPath(); ctx.moveTo(data.nodes[i].x, data.nodes[i].y); ctx.lineTo(data.nodes[j].x, data.nodes[j].y); ctx.stroke();
    }
    ctx.restore();
    ctx.fillStyle = "#6a4fb0";
    for (let k = 0; k < milestonesRevealed && k < data.milestones.length; k++) {
      const p = data.milestones[k];
      ctx.beginPath(); ctx.arc(p.x, p.y, 3, 0, Math.PI * 2); ctx.fill();
    }
    if (showPath && data.path) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 3;
      ctx.beginPath();
      data.path.forEach((idx, k) => { const p = data.nodes[idx]; k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    }
    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height }) {
    const world = makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const N = 55, k = 6;
    const data = computePRM(world, N, k);
    let idx = 0;
    const mSteps = data.milestones.length, eSteps = data.edges.length;
    const total = mSteps + eSteps + 1;
    return {
      draw(ctx) {
        const mRev = Math.min(idx, mSteps);
        const eRev = Math.max(0, Math.min(idx - mSteps, eSteps));
        draw(ctx, world, data, mRev, eRev, idx >= mSteps + eSteps);
      },
      step() {
        idx = Math.min(idx + 1, total);
        const done = idx >= total;
        let note;
        if (idx <= mSteps) note = `Sampling milestone ${idx} of ${mSteps} (rejected in-collision samples not shown).`;
        else if (idx <= mSteps + eSteps) note = `Connecting milestones to their ${k} nearest neighbors — testing straight-line collision (edge ${idx - mSteps} of ${eSteps}).`;
        else note = data.path ? `Roadmap complete. Shortest path over the graph ≈ ${data.pathLen.toFixed(0)}px through ${data.path.length} nodes.` : "Roadmap complete, but q_start and q_goal ended up in different components — try Generate new, or imagine adding more milestones.";
        return { done, note };
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
      { color: "#2f8f5b", label: "shortest path" },
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
