/* Quadtree (approximate cell decomposition) visualization: recursively split
   a mixed free/occupied region into 4 quadrants until each leaf is uniformly
   free, uniformly occupied, or a minimum size is reached. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  const MAX_DEPTH = 6, MIN_SIZE = 11, MIN_SPLIT_DEPTH = 3;

  function classify(world, r) {
    // sample density independent of region size, so large early regions don't
    // "get lucky" and miss a small obstacle between sparse sample points
    const nx = Math.max(3, Math.min(9, Math.round(r.w / 14)));
    const ny = Math.max(3, Math.min(9, Math.round(r.h / 14)));
    let free = 0, total = 0;
    for (let a = 0; a < nx; a++) for (let b = 0; b < ny; b++) {
      const x = r.x + (r.w * (a + 0.5)) / nx, y = r.y + (r.h * (b + 0.5)) / ny;
      total++;
      if (world.isFree(x, y)) free++;
    }
    if (free === total) return "free";
    if (free === 0) return "occupied";
    return "mixed";
  }

  function buildQuadtree(world) {
    const root = { x: 0, y: 0, w: world.width, h: world.height, depth: 0 };
    const events = [];
    const leaves = [];
    let queue = [root];
    while (queue.length) {
      const next = [];
      for (const r of queue) {
        const cls = classify(world, r);
        const forceSplit = r.depth < MIN_SPLIT_DEPTH;
        if ((cls === "mixed" || forceSplit) && r.depth < MAX_DEPTH && r.w > MIN_SIZE && r.h > MIN_SIZE) {
          events.push({ type: "split", r, cls });
          const hw = r.w / 2, hh = r.h / 2;
          const children = [
            { x: r.x, y: r.y, w: hw, h: hh, depth: r.depth + 1 },
            { x: r.x + hw, y: r.y, w: hw, h: hh, depth: r.depth + 1 },
            { x: r.x, y: r.y + hh, w: hw, h: hh, depth: r.depth + 1 },
            { x: r.x + hw, y: r.y + hh, w: hw, h: hh, depth: r.depth + 1 },
          ];
          next.push(...children);
        } else {
          const finalCls = cls === "mixed" ? "occupied" : cls; // conservative at resolution limit
          const leaf = { ...r, cls: finalCls, wasMixed: cls === "mixed", id: leaves.length };
          leaves.push(leaf);
          events.push({ type: "leaf", leaf });
        }
      }
      queue = next;
    }

    // adjacency among free leaves (touching edges)
    const freeLeaves = leaves.filter((l) => l.cls === "free");
    const adj = new Map(freeLeaves.map((l) => [l.id, []]));
    for (let i = 0; i < freeLeaves.length; i++) {
      for (let j = i + 1; j < freeLeaves.length; j++) {
        const a = freeLeaves[i], b = freeLeaves[j];
        const touchX = a.x < b.x + b.w && b.x < a.x + a.w;
        const touchY = a.y < b.y + b.h && b.y < a.y + a.h;
        const edgeTouchV = Math.abs(a.x - (b.x + b.w)) < 1 || Math.abs(b.x - (a.x + a.w)) < 1;
        const edgeTouchH = Math.abs(a.y - (b.y + b.h)) < 1 || Math.abs(b.y - (a.y + a.h)) < 1;
        if ((edgeTouchV && touchY) || (edgeTouchH && touchX)) {
          const w = Math.hypot((a.x + a.w / 2) - (b.x + b.w / 2), (a.y + a.h / 2) - (b.y + b.h / 2));
          adj.get(a.id).push([b.id, w]);
          adj.get(b.id).push([a.id, w]);
        }
      }
    }

    function locate(pt) { return freeLeaves.find((l) => pt.x >= l.x && pt.x <= l.x + l.w && pt.y >= l.y && pt.y <= l.y + l.h); }
    const startLeaf = locate(world.start), goalLeaf = locate(world.goal);
    let path = null;
    if (startLeaf && goalLeaf) {
      const dist = new Map(freeLeaves.map((l) => [l.id, Infinity]));
      const prev = new Map();
      dist.set(startLeaf.id, 0);
      const visited = new Set();
      while (visited.size < freeLeaves.length) {
        let u = -1, best = Infinity;
        for (const l of freeLeaves) if (!visited.has(l.id) && dist.get(l.id) < best) { best = dist.get(l.id); u = l.id; }
        if (u === -1) break;
        visited.add(u);
        if (u === goalLeaf.id) break;
        for (const [v, w] of adj.get(u) || []) if (dist.get(u) + w < dist.get(v)) { dist.set(v, dist.get(u) + w); prev.set(v, u); }
      }
      if (dist.get(goalLeaf.id) < Infinity) {
        const chain = [goalLeaf.id]; let cur = goalLeaf.id;
        while (cur !== startLeaf.id) { cur = prev.get(cur); chain.push(cur); }
        path = chain.reverse().map((id) => freeLeaves.find((l) => l.id === id));
      }
    }

    return { events, leaves, startLeaf, goalLeaf, path };
  }

  const CLS_COLOR = { free: "rgba(47,143,91,0.14)", occupied: "rgba(66,72,82,0.55)" };
  const CLS_STROKE = { free: "rgba(47,143,91,0.55)", occupied: "rgba(40,44,50,0.6)" };

  function draw(ctx, world, data, revealCount, showPath) {
    ctx.save();
    ctx.fillStyle = "#eef0f2";
    ctx.fillRect(0, 0, world.width, world.height);
    ctx.restore();
    for (let i = 0; i < revealCount && i < data.events.length; i++) {
      const e = data.events[i];
      if (e.type !== "leaf") continue;
      const l = e.leaf;
      ctx.fillStyle = CLS_COLOR[l.cls];
      ctx.fillRect(l.x, l.y, l.w, l.h);
      ctx.strokeStyle = CLS_STROKE[l.cls];
      ctx.lineWidth = 1;
      ctx.strokeRect(l.x + 0.5, l.y + 0.5, l.w - 1, l.h - 1);
    }
    world.draw(ctx, { fill: "rgba(58,63,71,0.001)", stroke: "#20242b", alpha: 1 });

    if (showPath && data.path) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(world.start.x, world.start.y);
      data.path.forEach((l) => ctx.lineTo(l.x + l.w / 2, l.y + l.h / 2));
      ctx.lineTo(world.goal.x, world.goal.y);
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  // pseudocode line indices (0-based)
  const L_ROOT = 0, L_STOP = 1, L_SPLIT = 3, L_LIMIT = 4, L_SEARCH = 5;

  function makeSim({ rng, width, height, world }) {
    const w = world || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const data = buildQuadtree(w);
    const leafEventCount = data.events.length;
    let idx = 0;
    const total = leafEventCount + 1;
    return {
      draw(ctx) { draw(ctx, w, data, Math.min(idx, leafEventCount), idx >= leafEventCount); },
      step() {
        idx = Math.min(idx + 1, total);
        const done = idx >= total;
        const e = data.events[Math.min(idx, leafEventCount) - 1];
        let note, line;
        if (idx === 1) {
          line = L_ROOT;
        } else if (e && e.type === "split") {
          line = L_SPLIT;
        } else if (e && e.type === "leaf") {
          line = e.leaf.wasMixed ? L_LIMIT : L_STOP;
        } else {
          line = L_SEARCH;
        }
        if (idx < leafEventCount) {
          note = e && e.type === "split"
            ? (e.cls === "mixed" ? "Region is mixed (both free and occupied) — split into 4 quadrants." : "Still coarse — split into 4 quadrants to resolve more detail.")
            : `Region is uniformly ${e ? e.leaf.cls : ""} — stop subdividing, keep as a leaf.`;
        } else if (idx === leafEventCount) {
          note = `Quadtree complete: ${data.leaves.filter((l) => l.cls === "free").length} free leaves, ${data.leaves.filter((l) => l.cls === "occupied").length} occupied. Locating start/goal leaves and searching the adjacency graph.`;
          line = L_SEARCH;
        } else {
          note = data.path ? `Path found through ${data.path.length} leaves.` : "q_start and q_goal fall in disconnected regions — no path found at this resolution.";
          line = L_SEARCH;
        }
        return { done, note, line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.quadtree = {
    title: "Quadtree Decomposition",
    badge: "§7.4 / book §6.2",
    subtitle: "Recursively split a mixed region into 4 quadrants until each leaf is uniformly free or uniformly occupied — an approximate decomposition.",
    width: 560, height: 360,
    legend: [
      { color: "rgba(47,143,91,0.35)", label: "free leaf" },
      { color: "rgba(66,72,82,0.7)", label: "occupied leaf" },
      { color: "#2f8f5b", label: "path via leaf centroids" },
    ],
    pseudocode: [
      "start with one cell covering the whole free space",
      "if a cell is fully free or fully occupied: stop splitting it",
      "if a cell is mixed (partly free, partly occupied):",
      { text: "split it into 4 equal quadrants, recurse on each", indent: 1 },
      "stop recursing at a resolution limit even if still mixed",
      "search the resulting free-cell adjacency graph",
    ],
    makeSim,
    pythonCode: `
def build_quadtree(region, is_free, max_depth=6, min_size=11):
    cls = classify(region, is_free)        # "free" / "occupied" / "mixed" via a 3x3 sample grid

    if cls != "mixed" or region.depth >= max_depth or region.w <= min_size:
        leaf_cls = "occupied" if cls == "mixed" else cls   # conservative at resolution limit
        return Leaf(region, leaf_cls)

    return Node(region, children=[build_quadtree(q, is_free, max_depth, min_size)
                                   for q in split_into_4(region)])

def find_path(tree, start, goal):
    free_leaves = [l for l in tree.leaves() if l.cls == "free"]
    graph = build_adjacency(free_leaves)              # leaves sharing an edge
    return dijkstra(graph, locate(free_leaves, start), locate(free_leaves, goal))
`,
  };
})();
