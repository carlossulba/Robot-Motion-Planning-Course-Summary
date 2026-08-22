/* ===========================================================
   gridflood.js — shared multi-source BFS / flood-fill grid
   propagation, used by Brushfire, Wave-Front, and (via
   brushfire.js) GVD. Factored out so all three share one
   propagation loop and one 4-/8-connectivity switch instead of
   each re-implementing the same BFS ring-by-ring expansion.
   =========================================================== */
(function (global) {
  "use strict";

  function neighbors4(i, j) { return [[i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1]]; }
  function neighbors8(i, j) {
    return [
      [i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1],
      [i - 1, j - 1], [i + 1, j - 1], [i - 1, j + 1], [i + 1, j + 1],
    ];
  }
  function neighborsFor(connectivity) { return connectivity === 8 ? neighbors8 : neighbors4; }

  // Multi-source BFS layer propagation over a `grid` (Uint8Array, 0 = free,
  // 1 = blocked), from `seedCells` = [[i,j,label], ...] (label optional).
  // Returns { val (Int32Array, -1 = never reached), label (Int32Array or
  // null), layers: [[ [i,j], ... ], ...] } where layers[0] is the seed
  // frontier itself and layers[k] is the k-th ring outward from it.
  function bfsLayers(cols, rows, grid, seedCells, opts) {
    opts = opts || {};
    const nbrFn = neighborsFor(opts.connectivity || 4);
    const idx = (i, j) => j * cols + i;
    const seedValue = opts.seedValue === undefined ? 0 : opts.seedValue;
    const val = new Int32Array(cols * rows).fill(-1);
    const trackLabel = !!opts.trackLabel;
    const label = trackLabel ? new Int32Array(cols * rows).fill(-1) : null;

    let frontier = [];
    for (const s of seedCells) {
      const i = s[0], j = s[1], lab = s[2];
      const k = idx(i, j);
      if (val[k] !== -1) continue; // duplicate seed
      val[k] = seedValue;
      if (label) label[k] = lab === undefined ? -1 : lab;
      frontier.push([i, j]);
    }

    const layers = [frontier];
    let d = seedValue;
    while (frontier.length) {
      const next = [];
      for (const [i, j] of frontier) {
        const here = idx(i, j);
        for (const [ni, nj] of nbrFn(i, j)) {
          if (ni < 0 || nj < 0 || ni >= cols || nj >= rows) continue;
          const k = idx(ni, nj);
          if (grid[k] !== 0 || val[k] !== -1) continue;
          val[k] = val[here] + 1;
          if (label) label[k] = label[here];
          next.push([ni, nj]);
        }
      }
      d++;
      if (next.length) layers.push(next);
      frontier = next;
    }
    return { val, label, layers };
  }

  // Builds a heat-color lookup from [t, [r,g,b]] stops (t ascending in [0,1]).
  function heatColorFactory(stops) {
    return function heatColor(t) {
      for (let i = 1; i < stops.length; i++) {
        if (t <= stops[i][0]) {
          const [t0, c0] = stops[i - 1], [t1, c1] = stops[i];
          const f = (t - t0) / (t1 - t0 || 1);
          const c = c0.map((v, k) => Math.round(v + (c1[k] - v) * f));
          return `rgb(${c[0]},${c[1]},${c[2]})`;
        }
      }
      const last = stops[stops.length - 1][1];
      return `rgb(${last[0]},${last[1]},${last[2]})`;
    };
  }

  global.RMP = global.RMP || {};
  global.RMP.gridFlood = { bfsLayers, heatColorFactory, neighbors4, neighbors8, neighborsFor };
})(window);
