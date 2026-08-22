/* Visibility Graph visualization: nodes = start, goal, and every obstacle
   vertex; edges connect any two nodes with an unobstructed line of sight.
   Ends by running Dijkstra over the graph to extract the shortest path. */
(function () {
  "use strict";
  const { makeWorld, geom } = window.RMP;

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
  function pointsEqual(a, b, eps) { return dist(a, b) < (eps || 0.75); }
  function orient(p, q, r) { return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x); }
  function onSeg(p, q, r) {
    return Math.min(p.x, r.x) - 1e-6 <= q.x && q.x <= Math.max(p.x, r.x) + 1e-6 &&
      Math.min(p.y, r.y) - 1e-6 <= q.y && q.y <= Math.max(p.y, r.y) + 1e-6;
  }
  function segSegIntersect(a, b, c, d) {
    if (pointsEqual(a, c) || pointsEqual(a, d) || pointsEqual(b, c) || pointsEqual(b, d)) return false;
    const d1 = orient(c, d, a), d2 = orient(c, d, b), d3 = orient(a, b, c), d4 = orient(a, b, d);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return true;
    if (d1 === 0 && onSeg(c, a, d)) return true;
    if (d2 === 0 && onSeg(c, b, d)) return true;
    if (d3 === 0 && onSeg(a, c, b)) return true;
    if (d4 === 0 && onSeg(a, d, b)) return true;
    return false;
  }

  function segmentBlocked(a, b, obstacles) {
    const mid = { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
    for (const o of obstacles) {
      const n = o.pts.length;
      for (let i = 0; i < n; i++) {
        const p = { x: o.pts[i][0], y: o.pts[i][1] }, q = { x: o.pts[(i + 1) % n][0], y: o.pts[(i + 1) % n][1] };
        if (segSegIntersect(a, b, p, q)) return true;
      }
      if (geom.pointInPoly(mid.x, mid.y, o.pts)) return true;
    }
    return false;
  }

  // ---- reduced/simplified visibility graph: keep only tangent lines ----
  // A line through A and B is a tangent of a polygon at one of its own
  // vertices V (V === A or V === B) iff every other vertex of that polygon
  // lies weakly on one side of the line (the polygon doesn't straddle it).
  // Start/goal are point-obstacles: trivially "all on one side" (no other
  // vertices to check), so edges from start/goal are kept whenever the one
  // real obstacle endpoint (if any) passes the test. Two nodes on the SAME
  // obstacle are excluded outright -- the reduced graph only keeps the
  // common tangents *between* obstacles (supporting: both obstacles on the
  // same side; separating: one on each side -- both cases pass this test).
  function sideOfLine(ax, ay, bx, by, px, py) {
    const cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
    return cross > 1e-6 ? 1 : cross < -1e-6 ? -1 : 0;
  }

  function isTangentAt(pts, ax, ay, bx, by) {
    let sawPos = false, sawNeg = false;
    for (const [x, y] of pts) {
      const s = sideOfLine(ax, ay, bx, by, x, y);
      if (s > 0) sawPos = true; else if (s < 0) sawNeg = true;
      if (sawPos && sawNeg) return false;
    }
    return true;
  }

  function isReducedGraphEdge(nodeA, nodeB, obstacles) {
    const oa = nodeA.kind === "vertex" ? obstacles[nodeA.obs] : null;
    const ob = nodeB.kind === "vertex" ? obstacles[nodeB.obs] : null;
    if (oa && ob && oa === ob) return false; // same-obstacle vertex pair: not an inter-obstacle tangent
    if (oa && !isTangentAt(oa.pts, nodeA.x, nodeA.y, nodeB.x, nodeB.y)) return false;
    if (ob && !isTangentAt(ob.pts, nodeA.x, nodeA.y, nodeB.x, nodeB.y)) return false;
    return true;
  }

  function dijkstra(nodes, adj, s, t) {
    const dd = new Array(nodes.length).fill(Infinity);
    const prev = new Array(nodes.length).fill(-1);
    const visited = new Array(nodes.length).fill(false);
    dd[s] = 0;
    for (let iter = 0; iter < nodes.length; iter++) {
      let u = -1, best = Infinity;
      for (let i = 0; i < nodes.length; i++) if (!visited[i] && dd[i] < best) { best = dd[i]; u = i; }
      if (u === -1) break;
      visited[u] = true;
      for (const [v, w] of adj[u]) if (dd[u] + w < dd[v]) { dd[v] = dd[u] + w; prev[v] = u; }
    }
    if (dd[t] === Infinity) return null;
    const path = [t];
    let cur = t;
    while (cur !== s) { cur = prev[cur]; path.push(cur); }
    return { path: path.reverse(), length: dd[t] };
  }

  function computeGraph(world, reduced) {
    const nodes = [
      { x: world.start.x, y: world.start.y, kind: "start" },
      { x: world.goal.x, y: world.goal.y, kind: "goal" },
    ];
    world.obstacles.forEach((o, oi) => o.pts.forEach((p, vi) => nodes.push({ x: p[0], y: p[1], kind: "vertex", obs: oi, v: vi })));

    const edges = [];
    const adj = nodes.map(() => []);
    let fullEdgeCount = 0;
    for (let i = 0; i < nodes.length; i++) {
      for (let j = i + 1; j < nodes.length; j++) {
        if (segmentBlocked(nodes[i], nodes[j], world.obstacles)) continue;
        fullEdgeCount++;
        if (reduced && !isReducedGraphEdge(nodes[i], nodes[j], world.obstacles)) continue;
        const w = dist(nodes[i], nodes[j]);
        edges.push([i, j, w]);
        adj[i].push([j, w]);
        adj[j].push([i, w]);
      }
    }
    const sp = dijkstra(nodes, adj, 0, 1);
    return { nodes, edges, sp, fullEdgeCount };
  }

  function draw(ctx, world, data, edgesRevealed, pathRevealed) {
    world.draw(ctx, { alpha: 0.9 });
    ctx.save();
    ctx.strokeStyle = "rgba(111,168,220,0.55)";
    ctx.lineWidth = 1;
    for (let k = 0; k < edgesRevealed; k++) {
      const [i, j] = data.edges[k];
      ctx.beginPath();
      ctx.moveTo(data.nodes[i].x, data.nodes[i].y);
      ctx.lineTo(data.nodes[j].x, data.nodes[j].y);
      ctx.stroke();
    }
    ctx.restore();

    data.nodes.forEach((n, i) => {
      if (i < 2) return;
      ctx.beginPath();
      ctx.fillStyle = "#8892a0";
      ctx.arc(n.x, n.y, 3, 0, Math.PI * 2);
      ctx.fill();
    });

    if (pathRevealed && data.sp) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      data.sp.path.forEach((idx, k) => {
        const n = data.nodes[idx];
        if (k === 0) ctx.moveTo(n.x, n.y); else ctx.lineTo(n.x, n.y);
      });
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, params, width, height, world }) {
    const reduced = !!(params && params.reduced);
    const w = world || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3), polygonsOnly: true, maxR: 30 });
    const data = computeGraph(w, reduced);
    let idx = 0;
    const total = data.edges.length + 1;
    return {
      draw(ctx) {
        const edgesRevealed = Math.min(idx, data.edges.length);
        draw(ctx, w, data, edgesRevealed, idx >= data.edges.length);
      },
      step() {
        idx = Math.min(idx + 1, total);
        const done = idx >= total;
        let note;
        const kind = reduced ? "reduced (tangent-only)" : "full";
        if (idx < data.edges.length) note = `Testing node pair ${idx} of ${data.edges.length}+ (${kind} graph): unobstructed line of sight — edge added.`;
        else if (data.sp) note = `Graph complete (${data.edges.length} edges${reduced ? `, vs. ${data.fullEdgeCount} in the full graph` : ""}). Dijkstra's shortest path over the graph has length ≈ ${data.sp.length.toFixed(0)}px, using ${data.sp.path.length} nodes.`;
        else note = `Graph complete (${data.edges.length} edges), but q_start and q_goal ended up in different components — no path exists.`;
        return { done, note };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.visibilitygraph = {
    title: "Visibility Graph",
    badge: "§6.1 / book §5.1",
    subtitle: "Nodes = start, goal, and every obstacle vertex. Edges connect any two nodes with a clear line of sight. Shortest path via Dijkstra.",
    width: 480, height: 320,
    legend: [
      { color: "rgba(111,168,220,0.8)", label: "visibility edge" },
      { color: "#2f8f5b", label: "shortest path" },
      { color: "#8892a0", label: "obstacle vertex" },
    ],
    params: [
      { key: "reduced", label: "Reduced (tangent-only) graph", type: "checkbox", value: false },
    ],
    makeSim,
    pythonCode: `
def visibility_graph(start, goal, obstacles):
    """obstacles: list of polygons (each a CCW list of (x, y) vertices)."""
    nodes = [start, goal] + [v for poly in obstacles for v in poly]
    edges = []

    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if not segment_blocked(nodes[i], nodes[j], obstacles):
                w = dist(nodes[i], nodes[j])
                edges.append((i, j, w))

    return nodes, edges

def segment_blocked(a, b, obstacles):
    mid = midpoint(a, b)
    for poly in obstacles:
        for p, q in polygon_edges(poly):
            if segments_properly_intersect(a, b, p, q):
                return True
        if point_in_polygon(mid, poly):     # segment passes through the interior
            return True
    return False

def shortest_path(nodes, edges, start_idx, goal_idx):
    return dijkstra(nodes, edges, start_idx, goal_idx)   # standard graph shortest path
`,
  };
})();
