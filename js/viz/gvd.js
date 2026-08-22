/* GVD / Voronoi roadmap visualization: extract the Generalized Voronoi
   Diagram as a graph (via the Brushfire distance transform), connect start
   and goal onto it, and search the graph for a maximum-clearance path. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

  function computeGvd(world) {
    const { cols, rows, grid, cellSize } = world;
    const idx = (i, j) => j * cols + i;
    const distArr = new Int32Array(cols * rows).fill(-1);
    const label = new Int32Array(cols * rows).fill(-1);
    let frontier = [];
    for (let j = 0; j < rows; j++) for (let i = 0; i < cols; i++) {
      if (grid[idx(i, j)] !== 1) continue;
      const nbrs = [[i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1]];
      if (!nbrs.some(([ni, nj]) => ni >= 0 && nj >= 0 && ni < cols && nj < rows && grid[idx(ni, nj)] === 0)) continue;
      const cx = (i + 0.5) * cellSize, cy = (j + 0.5) * cellSize;
      const srcId = world.obstacles.indexOf(world.nearestObstacle(cx, cy));
      distArr[idx(i, j)] = 0; label[idx(i, j)] = srcId; frontier.push([i, j]);
    }
    while (frontier.length) {
      const next = [];
      for (const [i, j] of frontier) {
        const here = idx(i, j);
        for (const [ni, nj] of [[i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1]]) {
          if (ni < 0 || nj < 0 || ni >= cols || nj >= rows) continue;
          const k = idx(ni, nj);
          if (grid[k] !== 0 || distArr[k] !== -1) continue;
          distArr[k] = distArr[here] + 1; label[k] = label[here]; next.push([ni, nj]);
        }
      }
      frontier = next;
    }

    const gvdSet = new Set();
    for (let j = 0; j < rows; j++) for (let i = 0; i < cols; i++) {
      const k = idx(i, j);
      if (grid[k] !== 0 || label[k] < 0) continue;
      for (const [ni, nj] of [[i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1]]) {
        if (ni < 0 || nj < 0 || ni >= cols || nj >= rows) continue;
        const k2 = idx(ni, nj);
        if (grid[k2] === 0 && label[k2] >= 0 && label[k2] !== label[k]) { gvdSet.add(k); break; }
      }
    }
    const gvdCells = Array.from(gvdSet).map((k) => ({ i: k % cols, j: Math.floor(k / cols), x: (k % cols + 0.5) * cellSize, y: (Math.floor(k / cols) + 0.5) * cellSize }));

    // graph over GVD cells (8-connectivity) + start/goal connectors
    const n = gvdCells.length;
    const adj = Array.from({ length: n + 2 }, () => []);
    const S = n, G = n + 1;
    const keyToIdx = new Map(gvdCells.map((c, k) => [c.j * cols + c.i, k]));
    const REACH = 2; // bridge small grid-discretization gaps in the skeleton
    gvdCells.forEach((c, k) => {
      for (let dj = -REACH; dj <= REACH; dj++) {
        for (let di = -REACH; di <= REACH; di++) {
          if (di === 0 && dj === 0) continue;
          const kk = keyToIdx.get((c.j + dj) * cols + (c.i + di));
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

    return { gvdCells, path, pointOf, length: dd[G] };
  }

  function draw(ctx, world, data, cellsRevealed, showConnectors, showPath) {
    world.draw(ctx, { alpha: 0.9 });
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

  function makeSim({ rng, width, height }) {
    const world = makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3), cellSize: 6 });
    const data = computeGvd(world);
    const revealStep = Math.max(1, Math.floor(data.gvdCells.length / 60));
    let idx = 0;
    const cellSteps = Math.ceil(data.gvdCells.length / revealStep);
    const total = cellSteps + 2; // + connectors step + path step
    return {
      draw(ctx) {
        const cellsRevealed = Math.min(idx * revealStep, data.gvdCells.length);
        draw(ctx, world, data, cellsRevealed, idx >= cellSteps, idx >= cellSteps + 1);
      },
      step() {
        idx = Math.min(idx + 1, total);
        const done = idx >= total;
        let note;
        if (idx < cellSteps) note = `Extracting the GVD skeleton from the distance transform (${Math.min(idx * revealStep, data.gvdCells.length)} of ${data.gvdCells.length} cells).`;
        else if (idx === cellSteps) note = "Skeleton complete. Connect q_start and q_goal to their nearest GVD point.";
        else note = data.path ? `Searching the GVD graph: max-clearance path found, length ≈ ${data.length.toFixed(0)}px.` : "No GVD path found connecting start and goal (disconnected roadmap in this layout).";
        return { done, note };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.computeGvd = computeGvd;
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.gvd = {
    title: "Generalized Voronoi Diagram (roadmap)",
    badge: "§5.2 / book §5.2",
    subtitle: "Extracted from the Brushfire distance transform, then used as a roadmap: depart start, search the skeleton, arrive at goal.",
    width: 560, height: 360,
    legend: [
      { color: "#b7532c", label: "GVD skeleton" },
      { color: "#8892a0", label: "depart/arrive connector" },
      { color: "#2f8f5b", label: "max-clearance path" },
    ],
    makeSim,
    pythonCode: `
def gvd_roadmap(grid, start, goal):
    dist, label = brushfire(grid)                    # see the Potential Functions page
    gvd = gvd_cells(grid, dist, label)                # equidistant cells -> the skeleton

    graph = build_adjacency(gvd, connectivity=8)
    s_node = nearest(gvd, start)
    g_node = nearest(gvd, goal)
    graph.add_edge("start", s_node, weight=dist_to(start, s_node))
    graph.add_edge("goal", g_node, weight=dist_to(goal, g_node))

    return dijkstra(graph, "start", "goal")           # max-clearance path
`,
  };
})();
