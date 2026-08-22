/* Trapezoidal Decomposition visualization: vertical lines extended from every
   obstacle vertex slice free space into trapezoids; path via the adjacency
   graph, connecting cell centroids through the midpoints of shared edges. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  function scanIntervals(world, x, step) {
    step = step || 2;
    const ivs = [];
    let open = null;
    for (let y = 1; y <= world.height - 1; y += step) {
      const free = world.isFree(x, y);
      if (free && open === null) open = y;
      if (!free && open !== null) { ivs.push([open, y - step]); open = null; }
    }
    if (open !== null) ivs.push([open, world.height - 1]);
    return ivs;
  }

  function overlap(a, b) { return Math.min(a[1], b[1]) - Math.max(a[0], b[0]); }

  function computeTrapezoids(world) {
    const eps = 1.5;
    const xs = new Set([world.margin * 0.4, world.width - world.margin * 0.4]);
    world.obstacles.forEach((o) => o.pts.forEach((p) => xs.add(p[0])));
    const bx = Array.from(xs).sort((a, b) => a - b).filter((x, i, arr) => i === 0 || x - arr[i - 1] > 3);

    const slabs = [];
    for (let k = 0; k < bx.length - 1; k++) {
      const xL = bx[k] + eps, xR = bx[k + 1] - eps;
      if (xR <= xL) continue;
      const left = scanIntervals(world, xL);
      const right = scanIntervals(world, xR);
      slabs.push({ x0: bx[k], x1: bx[k + 1], xL, xR, left, right, cells: left.map((iv, m) => ({ m, left: iv, right: right[m] || iv })) });
    }

    // adjacency across slab boundaries via interval overlap
    const cellRef = (k, m) => ({ k, m });
    const edges = []; // {a:{k,m}, b:{k,m}, port:{x,y}}
    for (let k = 0; k < slabs.length - 1; k++) {
      slabs[k].cells.forEach((ca, m) => {
        slabs[k + 1].cells.forEach((cb, m2) => {
          const ov = overlap(ca.right, cb.left);
          if (ov > 1) {
            const y0 = Math.max(ca.right[0], cb.left[0]), y1 = Math.min(ca.right[1], cb.left[1]);
            edges.push({ a: cellRef(k, m), b: cellRef(k + 1, m2), port: { x: slabs[k].x1, y: (y0 + y1) / 2 } });
          }
        });
      });
    }

    function centroid(k, m) {
      const c = slabs[k].cells[m];
      return { x: (slabs[k].xL + slabs[k].xR) / 2, y: (c.left[0] + c.left[1] + c.right[0] + c.right[1]) / 4 };
    }
    function locate(pt) {
      for (let k = 0; k < slabs.length; k++) {
        if (pt.x >= slabs[k].x0 && pt.x <= slabs[k].x1) {
          for (let m = 0; m < slabs[k].cells.length; m++) {
            const c = slabs[k].cells[m];
            const yLo = Math.min(c.left[0], c.right[0]), yHi = Math.max(c.left[1], c.right[1]);
            if (pt.y >= yLo - 3 && pt.y <= yHi + 3) return { k, m };
          }
        }
      }
      return null;
    }

    const startCell = locate(world.start), goalCell = locate(world.goal);
    let route = null;
    if (startCell && goalCell) {
      const nodeKey = (r) => `${r.k}:${r.m}`;
      const adj = new Map();
      const addEdge = (a, b, edge) => {
        const ka = nodeKey(a);
        if (!adj.has(ka)) adj.set(ka, []);
        adj.get(ka).push({ to: b, edge });
      };
      edges.forEach((e) => { addEdge(e.a, e.b, e); addEdge(e.b, e.a, e); });
      const startKey = nodeKey(startCell), goalKey = nodeKey(goalCell);
      const visited = new Set([startKey]);
      const prevMap = new Map();
      const queue = [startKey];
      while (queue.length) {
        const cur = queue.shift();
        if (cur === goalKey) break;
        for (const { to, edge } of adj.get(cur) || []) {
          const k2 = nodeKey(to);
          if (!visited.has(k2)) { visited.add(k2); prevMap.set(k2, { from: cur, edge }); queue.push(k2); }
        }
      }
      if (visited.has(goalKey)) {
        const chain = [];
        let cur = goalKey;
        while (cur !== startKey) { const { from, edge } = prevMap.get(cur); chain.push(edge); cur = from; }
        chain.reverse();
        route = chain;
      }
    }

    return { slabs, edges, startCell, goalCell, route, centroid };
  }

  function draw(ctx, world, data, slabsRevealed, showLocate, showPath) {
    world.draw(ctx, { alpha: 0.9 });
    ctx.save();
    for (let k = 0; k < slabsRevealed && k < data.slabs.length; k++) {
      const s = data.slabs[k];
      s.cells.forEach((c, m) => {
        ctx.beginPath();
        ctx.moveTo(s.xL, c.left[0]); ctx.lineTo(s.xR, c.right[0]);
        ctx.lineTo(s.xR, c.right[1]); ctx.lineTo(s.xL, c.left[1]);
        ctx.closePath();
        ctx.fillStyle = (m % 2 === 0) ? "rgba(111,168,220,0.16)" : "rgba(111,168,220,0.28)";
        ctx.fill();
        ctx.strokeStyle = "rgba(111,168,220,0.7)";
        ctx.lineWidth = 1;
        ctx.stroke();
      });
      ctx.strokeStyle = "#8892a0";
      ctx.setLineDash([3, 2]);
      ctx.beginPath(); ctx.moveTo(s.x1, 2); ctx.lineTo(s.x1, world.height - 2); ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.restore();

    if (showLocate && data.startCell && data.goalCell) {
      ctx.save();
      ctx.fillStyle = "#b7532c";
      [data.centroid(data.startCell.k, data.startCell.m), data.centroid(data.goalCell.k, data.goalCell.m)].forEach((p) => {
        ctx.beginPath(); ctx.arc(p.x, p.y, 3, 0, Math.PI * 2); ctx.fill();
      });
      ctx.restore();
    }

    if (showPath && data.route) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(world.start.x, world.start.y);
      let cur = data.startCell;
      ctx.lineTo(data.centroid(cur.k, cur.m).x, data.centroid(cur.k, cur.m).y);
      data.route.forEach((edge) => {
        ctx.lineTo(edge.port.x, edge.port.y);
        const nextCell = (edge.a.k === cur.k && edge.a.m === cur.m) ? edge.b : edge.a;
        ctx.lineTo(data.centroid(nextCell.k, nextCell.m).x, data.centroid(nextCell.k, nextCell.m).y);
        cur = nextCell;
      });
      ctx.lineTo(world.goal.x, world.goal.y);
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height }) {
    const world = makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3), polygonsOnly: true, maxR: 28 });
    const data = computeTrapezoids(world);
    let idx = 0;
    const total = data.slabs.length + 2;
    return {
      draw(ctx) {
        const slabsRevealed = Math.min(idx, data.slabs.length);
        draw(ctx, world, data, slabsRevealed, idx >= data.slabs.length, idx >= data.slabs.length + 1);
      },
      step() {
        idx = Math.min(idx + 1, total);
        const done = idx >= total;
        let note;
        if (idx < data.slabs.length) note = `Slab ${idx} of ${data.slabs.length}: vertical extensions dropped from each vertex slice this strip into trapezoids.`;
        else if (idx === data.slabs.length) note = "Decomposition complete. Locate the trapezoid containing q_start and the one containing q_goal.";
        else note = data.route ? `Path found through ${data.route.length + 1} adjacent trapezoids, connecting centroids via the shared-edge midpoints.` : "q_start and q_goal ended up in disconnected trapezoids — no path exists.";
        return { done, note };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.trapezoidal = {
    title: "Trapezoidal Decomposition",
    badge: "§5.2.1 / book §6.1",
    subtitle: "A vertical line is extended from every obstacle vertex, slicing free space into trapezoids. Path = adjacency-graph search + centroid/midpoint connectors.",
    width: 560, height: 360,
    legend: [
      { color: "rgba(111,168,220,0.5)", label: "trapezoid cell" },
      { color: "#b7532c", label: "start/goal cell centroid" },
      { color: "#2f8f5b", label: "resulting path" },
    ],
    makeSim,
    pythonCode: `
def trapezoidal_decomposition(obstacles, width, height):
    xs = sorted({x for poly in obstacles for x, y in poly})
    slabs = []
    for x0, x1 in zip(xs, xs[1:]):
        left = free_intervals(x0 + eps, height)      # scan a vertical line
        right = free_intervals(x1 - eps, height)
        cells = [{"left": l, "right": r} for l, r in zip(left, right)]
        slabs.append({"x0": x0, "x1": x1, "cells": cells})

    edges = []                                        # adjacency across slab boundaries
    for k in range(len(slabs) - 1):
        for m, ca in enumerate(slabs[k]["cells"]):
            for m2, cb in enumerate(slabs[k + 1]["cells"]):
                if intervals_overlap(ca["right"], cb["left"]):
                    edges.append(((k, m), (k + 1, m2)))

    return slabs, edges

def find_path(slabs, edges, start, goal):
    start_cell, goal_cell = locate(slabs, start), locate(slabs, goal)
    route = bfs(edges, start_cell, goal_cell)          # graph search over the adjacency graph
    return [start] + [centroid_and_port(c) for c in route] + [goal]
`,
  };
})();
