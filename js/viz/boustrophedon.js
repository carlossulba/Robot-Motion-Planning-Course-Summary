/* Boustrophedon (Morse) Decomposition visualization: slab boundaries are
   only placed at "critical points" -- vertices where the number of free-space
   intervals along the sweep actually changes -- giving far fewer, coarser
   cells than a full trapezoidal decomposition of the same scene. */
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

  function computeBoustrophedon(world) {
    const eps = 2;
    const allVertX = Array.from(new Set(world.obstacles.flatMap((o) => o.pts.map((p) => p[0])))).sort((a, b) => a - b);
    const critical = new Set([world.margin * 0.4, world.width - world.margin * 0.4]);
    allVertX.forEach((x) => {
      const before = scanIntervals(world, Math.max(1, x - eps)).length;
      const after = scanIntervals(world, Math.min(world.width - 1, x + eps)).length;
      if (before !== after) critical.add(x);
    });
    const bx = Array.from(critical).sort((a, b) => a - b).filter((x, i, arr) => i === 0 || x - arr[i - 1] > 4);

    const SAMPLES = 6;
    const slabs = [];
    for (let k = 0; k < bx.length - 1; k++) {
      const x0 = bx[k] + eps, x1 = bx[k + 1] - eps;
      if (x1 <= x0) continue;
      const xsSample = Array.from({ length: SAMPLES }, (_, s) => x0 + (x1 - x0) * (s / (SAMPLES - 1)));
      const scans = xsSample.map((x) => scanIntervals(world, x));
      const count = Math.min(...scans.map((s) => s.length));
      const cells = [];
      for (let m = 0; m < count; m++) {
        cells.push({ m, tops: scans.map((s) => s[m][0]), bots: scans.map((s) => s[m][1]), left: scans[0][m], right: scans[scans.length - 1][m] });
      }
      slabs.push({ x0: bx[k], x1: bx[k + 1], xsSample, cells });
    }

    const edges = [];
    for (let k = 0; k < slabs.length - 1; k++) {
      slabs[k].cells.forEach((ca, m) => {
        slabs[k + 1].cells.forEach((cb, m2) => {
          const ov = overlap(ca.right, cb.left);
          if (ov > 1) {
            const y0 = Math.max(ca.right[0], cb.left[0]), y1 = Math.min(ca.right[1], cb.left[1]);
            edges.push({ a: { k, m }, b: { k: k + 1, m: m2 }, port: { x: slabs[k].x1, y: (y0 + y1) / 2 } });
          }
        });
      });
    }

    function centroid(k, m) {
      const c = slabs[k].cells[m];
      const xs = slabs[k].xsSample;
      return { x: (xs[0] + xs[xs.length - 1]) / 2, y: (c.tops.reduce((a, b) => a + b, 0) + c.bots.reduce((a, b) => a + b, 0)) / (c.tops.length * 2) };
    }
    function locate(pt) {
      for (let k = 0; k < slabs.length; k++) {
        if (pt.x >= slabs[k].x0 && pt.x <= slabs[k].x1) {
          for (let m = 0; m < slabs[k].cells.length; m++) {
            const c = slabs[k].cells[m];
            const yLo = Math.min(...c.tops), yHi = Math.max(...c.bots);
            if (pt.y >= yLo - 3 && pt.y <= yHi + 3) return { k, m };
          }
        }
      }
      return null;
    }

    const startCell = locate(world.start), goalCell = locate(world.goal);
    let route = null;
    if (startCell && goalCell) {
      const key = (r) => `${r.k}:${r.m}`;
      const adj = new Map();
      const add = (a, b, e) => { const kk = key(a); if (!adj.has(kk)) adj.set(kk, []); adj.get(kk).push({ to: b, edge: e }); };
      edges.forEach((e) => { add(e.a, e.b, e); add(e.b, e.a, e); });
      const sK = key(startCell), gK = key(goalCell);
      const visited = new Set([sK]), prevMap = new Map(), queue = [sK];
      while (queue.length) {
        const cur = queue.shift();
        if (cur === gK) break;
        for (const { to, edge } of adj.get(cur) || []) {
          const k2 = key(to);
          if (!visited.has(k2)) { visited.add(k2); prevMap.set(k2, { from: cur, edge }); queue.push(k2); }
        }
      }
      if (visited.has(gK)) {
        const chain = []; let cur = gK;
        while (cur !== sK) { const { from, edge } = prevMap.get(cur); chain.push(edge); cur = from; }
        route = chain.reverse();
      }
    }

    return { slabs, edges, startCell, goalCell, route, centroid, nCritical: bx.length - 2 };
  }

  function draw(ctx, world, data, slabsRevealed, showLocate, showPath) {
    world.draw(ctx, { alpha: 0.9 });
    ctx.save();
    for (let k = 0; k < slabsRevealed && k < data.slabs.length; k++) {
      const s = data.slabs[k];
      s.cells.forEach((c, m) => {
        ctx.beginPath();
        s.xsSample.forEach((x, si) => { const y = c.tops[si]; si === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y); });
        for (let si = s.xsSample.length - 1; si >= 0; si--) ctx.lineTo(s.xsSample[si], c.bots[si]);
        ctx.closePath();
        ctx.fillStyle = (m % 2 === 0) ? "rgba(183,83,44,0.16)" : "rgba(183,83,44,0.28)";
        ctx.fill();
        ctx.strokeStyle = "rgba(183,83,44,0.75)";
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
        const nxt = (edge.a.k === cur.k && edge.a.m === cur.m) ? edge.b : edge.a;
        ctx.lineTo(data.centroid(nxt.k, nxt.m).x, data.centroid(nxt.k, nxt.m).y);
        cur = nxt;
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
    const data = computeBoustrophedon(world);
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
        if (idx < data.slabs.length) note = `Slab ${idx} of ${data.slabs.length} (only ${data.nCritical} critical points found — far fewer than one per vertex).`;
        else if (idx === data.slabs.length) note = "Decomposition complete. Locate the cell containing q_start and the one containing q_goal.";
        else note = data.route ? `Path found through ${data.route.length + 1} cells — typically fewer, larger cells than trapezoidal decomposition of the same scene.` : "q_start and q_goal ended up in disconnected cells — no path exists.";
        return { done, note };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.boustrophedon = {
    title: "Boustrophedon (Morse) Decomposition",
    badge: "§5.2.2 / book §6.2",
    subtitle: "A slab boundary is only placed where the number of free-space intervals actually changes — far coarser than trapezoidal decomposition.",
    width: 560, height: 360,
    legend: [
      { color: "rgba(183,83,44,0.5)", label: "cell" },
      { color: "#b7532c", label: "start/goal cell centroid" },
      { color: "#2f8f5b", label: "resulting path" },
    ],
    makeSim,
    pythonCode: `
def boustrophedon_decomposition(obstacles, width, height):
    vertex_xs = sorted({x for poly in obstacles for x, y in poly})

    critical = [x for x in vertex_xs
                if len(free_intervals(x - eps, height)) != len(free_intervals(x + eps, height))]
    # only keep x-values where the *count* of free intervals changes --
    # a plain pass-through vertex changes the boundary shape but not the count

    boundaries = [0, *critical, width]
    slabs = []
    for x0, x1 in zip(boundaries, boundaries[1:]):
        samples = [free_intervals(x, height) for x in linspace(x0 + eps, x1 - eps, 6)]
        cells = list(zip(*samples))                  # cell m = interval m across all samples
        slabs.append({"x0": x0, "x1": x1, "cells": cells})

    return slabs   # adjacency + path search: same as trapezoidal decomposition
`,
  };
})();
