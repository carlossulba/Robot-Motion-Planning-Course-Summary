/* Brushfire visualization: multi-source grid distance transform.
   Every "step" reveals one full ring (BFS layer) propagating from every
   obstacle simultaneously; where two differently-sourced rings meet at
   equal distance, that's a Generalized Voronoi Diagram (GVD) cell.

   computeBrushfire/heatColor/draw are exposed as window.RMP.brushfireCore
   so gvd.js can reuse this exact distance-transform + rendering instead of
   recomputing it -- the GVD literally comes out of Brushfire. */
(function () {
  "use strict";
  const { makeWorld, gridFlood } = window.RMP;

  function computeBrushfire(world, connectivity) {
    const { cols, rows, grid, cellSize } = world;
    const idx = (i, j) => j * cols + i;

    const seeds = [];
    for (let j = 0; j < rows; j++) {
      for (let i = 0; i < cols; i++) {
        if (grid[idx(i, j)] !== 1) continue;
        const nbrs = [[i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1]];
        const touchesFree = nbrs.some(([ni, nj]) => ni >= 0 && nj >= 0 && ni < cols && nj < rows && grid[idx(ni, nj)] === 0);
        if (!touchesFree) continue;
        const cx = (i + 0.5) * cellSize, cy = (j + 0.5) * cellSize;
        const obs = world.nearestObstacle(cx, cy);
        seeds.push([i, j, world.obstacles.indexOf(obs)]);
      }
    }

    const { val: dist, label, layers } = gridFlood.bfsLayers(cols, rows, grid, seeds, {
      connectivity: connectivity || 4,
      trackLabel: true,
    });

    // GVD cells: free cells with a differently-labeled free neighbor
    // (always checked via 4-neighbors -- the skeleton is a thin boundary
    // between labeled regions regardless of the propagation connectivity).
    const gvd = [];
    for (let j = 0; j < rows; j++) {
      for (let i = 0; i < cols; i++) {
        const k = idx(i, j);
        if (grid[k] !== 0 || label[k] < 0) continue;
        for (const [ni, nj] of [[i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1]]) {
          if (ni < 0 || nj < 0 || ni >= cols || nj >= rows) continue;
          const k2 = idx(ni, nj);
          if (grid[k2] === 0 && label[k2] >= 0 && label[k2] !== label[k]) { gvd.push([i, j]); break; }
        }
      }
    }

    return { layers, gvd, dist, label, maxDist: layers.length };
  }

  const heatColor = gridFlood.heatColorFactory([
    [0.00, [214, 69, 43]], [0.35, [214, 150, 43]], [0.65, [120, 160, 90]], [1.00, [59, 110, 160]],
  ]);

  function draw(ctx, world, layers, gvd, revealCount, showGvd) {
    const { cellSize } = world;
    ctx.save();
    ctx.fillStyle = "#eef0f2";
    ctx.fillRect(0, 0, world.width, world.height);
    const maxD = layers.length || 1;
    for (let d = 0; d < revealCount && d < layers.length; d++) {
      ctx.fillStyle = heatColor(d / maxD);
      for (const [i, j] of layers[d]) ctx.fillRect(i * cellSize, j * cellSize, cellSize + 0.5, cellSize + 0.5);
    }
    if (showGvd) {
      ctx.fillStyle = "#1b2027";
      for (const [i, j] of gvd) ctx.fillRect(i * cellSize + cellSize * 0.25, j * cellSize + cellSize * 0.25, cellSize * 0.5, cellSize * 0.5);
    }
    ctx.restore();
    world.draw(ctx, { fill: "#3a3f47", stroke: "#20242b", alpha: 1 });
    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  // pseudocode line indices (0-based)
  const L_INIT = 0, L_PROPAGATE_HEAD = 1, L_PROPAGATE_BODY = 2, L_DONE = 3;

  function makeSim({ rng, params, width, height, world }) {
    const conn = params && params.connectivity ? 8 : 4;
    const w = world || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3), cellSize: 7 });
    const { layers, gvd } = computeBrushfire(w, conn);
    let idx = 0; // number of layers revealed
    const totalSteps = layers.length + 1; // +1 final step reveals GVD
    return {
      draw(ctx) { draw(ctx, w, layers, gvd, idx, idx >= layers.length); },
      step() {
        idx = Math.min(idx + 1, totalSteps);
        const done = idx >= totalSteps;
        let note, line;
        if (idx === 1) {
          note = `Init: marking every obstacle cell touching free space as the seed frontier (${layers[0].length} cells).`;
          line = L_INIT;
        } else if (idx < layers.length) {
          note = `Propagating ring ${idx} of ${layers.length} outward from every obstacle boundary simultaneously (${conn}-connectivity).`;
          line = idx === 2 ? L_PROPAGATE_HEAD : L_PROPAGATE_BODY;
        } else {
          note = `Every free cell has a distance value. Dark cells mark the Generalized Voronoi Diagram — where two different obstacles' rings met at equal distance.`;
          line = L_DONE;
        }
        return { done, note, line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.brushfireCore = { computeBrushfire, draw, heatColor };
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.brushfire = {
    title: "Brushfire",
    badge: "§5.1 / book §4.3.2",
    subtitle: "A distance transform computed by multi-source BFS on a grid — the standard way to build a GVD from raster data.",
    width: 480, height: 320,
    legend: [
      { color: "rgb(214,69,43)", label: "near an obstacle" },
      { color: "rgb(59,110,160)", label: "far from all obstacles" },
      { color: "#1b2027", label: "GVD (rings met)" },
    ],
    params: [
      { key: "connectivity", label: "8-connectivity", type: "checkbox", value: false },
    ],
    pseudocode: [
      "Init: every obstacle cell and outer-boundary cell <- 1",
      "Propagate: repeat until every free cell has a value:",
      { text: "each free cell <- 1 + min(neighbor values)   [multi-source BFS]", indent: 1 },
      "done: cell value = distance to nearest obstacle",
    ],
    makeSim,
    pythonCode: `
from collections import deque

def brushfire(grid, connectivity=4):
    """grid[j][i] == 1 for obstacle cells. Returns (dist, label) grids;
    label[j][i] identifies which obstacle's wave reached that cell first."""
    rows, cols = len(grid), len(grid[0])
    dist = [[-1] * cols for _ in range(rows)]
    label = [[-1] * cols for _ in range(rows)]
    q = deque()

    for j in range(rows):
        for i in range(cols):
            if grid[j][i] == 1 and touches_free(grid, i, j):
                dist[j][i] = 0
                label[j][i] = nearest_obstacle_id(i, j)
                q.append((i, j))

    neighbors = neighbors8 if connectivity == 8 else neighbors4
    while q:
        i, j = q.popleft()
        for ni, nj in neighbors(i, j, cols, rows):
            if grid[nj][ni] == 0 and dist[nj][ni] == -1:
                dist[nj][ni] = dist[j][i] + 1
                label[nj][ni] = label[j][i]
                q.append((ni, nj))

    return dist, label

def gvd_cells(grid, dist, label):
    """Cells adjacent to a differently-labeled free cell lie on the GVD."""
    return [(i, j) for i, j in free_cells(grid)
            if any(label[nj][ni] not in (-1, label[j][i])
                   for ni, nj in neighbors4(i, j, len(grid[0]), len(grid)))]
`,
  };
})();
