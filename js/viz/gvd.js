/* GVD / Voronoi roadmap visualization: extract the Generalized Voronoi
   Diagram as a graph (via the Brushfire distance transform), connect start
   and goal onto it, and search the graph for a maximum-clearance path.

   Reuses brushfire.js's computeBrushfire/draw (window.RMP.brushfireCore)
   directly rather than recomputing the distance transform -- and, since the
   GVD comes out of Brushfire, the demo now visibly plays the Brushfire
   ring-propagation phase before revealing the skeleton extracted from it. */
(function () {
  "use strict";
  const { makeWorld, brushfireCore } = window.RMP;

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

  function computeGvd(world, connectivity) {
    const { cellSize } = world;
    const bf = brushfireCore.computeBrushfire(world, connectivity || 4);
    const gvdCells = bf.gvd.map(([i, j]) => ({ i, j, x: (i + 0.5) * cellSize, y: (j + 0.5) * cellSize }));

    // graph over GVD cells (8-connectivity) + start/goal connectors
    const n = gvdCells.length;
    const adj = Array.from({ length: n + 2 }, () => []);
    const S = n, G = n + 1;
    const keyToIdx = new Map(gvdCells.map((c, k) => [c.j * world.cols + c.i, k]));
    const REACH = 2; // bridge small grid-discretization gaps in the skeleton
    gvdCells.forEach((c, k) => {
      for (let dj = -REACH; dj <= REACH; dj++) {
        for (let di = -REACH; di <= REACH; di++) {
          if (di === 0 && dj === 0) continue;
          const kk = keyToIdx.get((c.j + dj) * world.cols + (c.i + di));
          if (kk !== undefined) adj[k].push([kk, dist(c, gvdCells[kk])]);
        }
      }
    });
    let nearS = -1, dS = Infinity, nearG = -1, dG = Infinity;
    gvdCells.forEach((c, k) => {
      const ds = dist(c, world.start), dg = dist(c, world.goal);
      if (ds < dS) { dS = ds; nearS = k; }
      if (dg < dG) { dG = dg; nearG = k; }
    });
    if (nearS >= 0) { adj[S].push([nearS, dS]); adj[nearS].push([S, dS]); }
    if (nearG >= 0) { adj[G].push([nearG, dG]); adj[nearG].push([G, dG]); }

    // Dijkstra S -> G
    const dd = new Array(n + 2).fill(Infinity), prev = new Array(n + 2).fill(-1), vis = new Array(n + 2).fill(false);
    dd[S] = 0;
    for (let it = 0; it < n + 2; it++) {
      let u = -1, best = Infinity;
      for (let i = 0; i < n + 2; i++) if (!vis[i] && dd[i] < best) { best = dd[i]; u = i; }
      if (u === -1) break;
      vis[u] = true;
      for (const [v, w] of adj[u]) if (dd[u] + w < dd[v]) { dd[v] = dd[u] + w; prev[v] = u; }
    }
    let path = null;
    if (dd[G] < Infinity) {
      path = [G]; let cur = G;
      while (cur !== S) { cur = prev[cur]; path.push(cur); }
      path.reverse();
    }
    const pointOf = (id) => id === S ? world.start : id === G ? world.goal : gvdCells[id];

    return { gvdCells, path, pointOf, length: dd[G], brushfire: bf };
  }

  function draw(ctx, world, data, phase, brushfireRings, cellsRevealed, showConnectors, showPath) {
    // Always render the Brushfire ring coloring first -- fully revealed once
    // we're past the replay phase -- and layer the GVD skeleton on top of it.
    // The ring/heatmap coloring must stay visible throughout, not get wiped
    // the moment GVD marking starts.
    const bfRevealed = phase === "brushfire" ? brushfireRings : data.brushfire.layers.length;
    brushfireCore.draw(ctx, world, data.brushfire.layers, data.brushfire.gvd, bfRevealed, false);
    if (phase === "brushfire") return;

    ctx.save();
    ctx.fillStyle = "#b7532c";
    for (let k = 0; k < cellsRevealed; k++) {
      const c = data.gvdCells[k];
      ctx.beginPath();
      ctx.arc(c.x, c.y, 1.6, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();

    if (showConnectors) {
      ctx.save();
      ctx.strokeStyle = "#8892a0";
      ctx.setLineDash([3, 3]);
      ctx.lineWidth = 1.2;
      const nearS = data.path ? data.pointOf(data.path[1]) : null;
      const nearG = data.path ? data.pointOf(data.path[data.path.length - 2]) : null;
      if (nearS) { ctx.beginPath(); ctx.moveTo(world.start.x, world.start.y); ctx.lineTo(nearS.x, nearS.y); ctx.stroke(); }
      if (nearG) { ctx.beginPath(); ctx.moveTo(world.goal.x, world.goal.y); ctx.lineTo(nearG.x, nearG.y); ctx.stroke(); }
      ctx.restore();
    }

    if (showPath && data.path) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      data.path.forEach((id, k) => { const p = data.pointOf(id); if (k === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  // pseudocode line indices (0-based)
  const L_INIT = 0, L_PROPAGATE = 1, L_MARK = 2, L_CONNECT = 3, L_SEARCH = 4;

  function makeSim({ rng, params, width, height, world }) {
    const conn = params && params.connectivity ? 8 : 4;
    const w = world || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3), cellSize: 6 });
    const data = computeGvd(w, conn);
    const bfRings = data.brushfire.layers.length;
    let idx = 0;
    // phase A: bfRings steps replaying the Brushfire propagation
    // phase B: 1 step revealing the whole GVD skeleton at once (a single
    //          completed pass, mirroring how Brushfire's own reveal batches
    //          a whole ring per step rather than cell-by-cell)
    // phase C: 1 step showing start/goal connectors
    // phase D: 1 step showing the searched max-clearance path
    const total = bfRings + 3;

    return {
      draw(ctx) {
        if (idx <= bfRings) {
          draw(ctx, w, data, "brushfire", idx, 0, false, false);
        } else {
          const skIdx = idx - bfRings; // 1 = marked, 2 = connectors, 3 = path
          const cellsRevealed = skIdx >= 1 ? data.gvdCells.length : 0;
          draw(ctx, w, data, "gvd", 0, cellsRevealed, skIdx >= 2, skIdx >= 3);
        }
      },
      step() {
        idx = Math.min(idx + 1, total);
        const done = idx >= total;
        let note, line;
        if (idx <= bfRings) {
          note = idx === 1
            ? `Init: marking obstacle-adjacent cells as the Brushfire seed frontier.`
            : `Replaying Brushfire's distance-transform propagation (ring ${idx} of ${bfRings}) — the GVD is extracted from this.`;
          line = idx === 1 ? L_INIT : L_PROPAGATE;
        } else {
          const skIdx = idx - bfRings;
          if (skIdx === 1) {
            note = `Marking all ${data.gvdCells.length} GVD cells at once — free cells where two distinct waves met at equal value.`;
            line = L_MARK;
          } else if (skIdx === 2) {
            note = "Skeleton complete. Connect q_start and q_goal to their nearest GVD point.";
            line = L_CONNECT;
          } else {
            note = data.path ? `Searching the GVD graph: max-clearance path found, length ≈ ${data.length.toFixed(0)}px.` : "No GVD path found connecting start and goal (disconnected roadmap in this layout).";
            line = L_SEARCH;
          }
        }
        return { done, note, line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.computeGvd = computeGvd;
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.gvd = {
    title: "Generalized Voronoi Diagram (roadmap)",
    badge: "§6.2 / book §5.2",
    subtitle: "Extracted from the Brushfire distance transform, then used as a roadmap: depart start, search the skeleton, arrive at goal.",
    width: 480, height: 320,
    legend: [
      { color: "rgb(214,69,43)", label: "Brushfire ring (replay)" },
      { color: "#b7532c", label: "GVD skeleton" },
      { color: "#8892a0", label: "depart/arrive connector" },
      { color: "#2f8f5b", label: "max-clearance path" },
    ],
    params: [
      { key: "connectivity", label: "8-connectivity", type: "checkbox", value: false },
    ],
    pseudocode: [
      "grid init: free cells <- 0, obstacle cells <- 1",
      "propagate distance-to-nearest-obstacle (Brushfire)",
      "mark a cell as GVD if two distinct waves meet there at equal value",
      "connect q_start, q_goal onto the nearest GVD point",
      "search the GVD graph for a path (max-clearance route)",
    ],
    makeSim,
    pythonCode: `
def gvd_roadmap(grid, start, goal, connectivity=4):
    dist, label = brushfire(grid, connectivity)        # see the Brushfire demo
    gvd = gvd_cells(grid, dist, label)                  # equidistant cells -> the skeleton

    graph = build_adjacency(gvd, connectivity=8)
    s_node = nearest(gvd, start)
    g_node = nearest(gvd, goal)
    graph.add_edge("start", s_node, weight=dist_to(start, s_node))
    graph.add_edge("goal", g_node, weight=dist_to(goal, g_node))

    return dijkstra(graph, "start", "goal")             # max-clearance path
`,
  };
})();
