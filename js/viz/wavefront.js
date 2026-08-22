/* Wave-Front visualization: single-source BFS numbering from the GOAL cell.
   Seeding at the goal (not the start) is what makes the planner immune to
   local minima: every free cell ends up with a value that strictly decreases
   toward the goal, so a robot starting anywhere can reach the goal by pure
   greedy descent -- no need to recompute anything for a different start. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  function computeWaveFront(world) {
    const { cols, rows, grid, cellSize } = world;
    const idx = (i, j) => j * cols + i;
    const val = new Int32Array(cols * rows).fill(0); // 1 = obstacle, 0 = unvisited free, >=2 = wave number
    for (let k = 0; k < grid.length; k++) if (grid[k] === 1) val[k] = 1;

    const sc = world.cellOf(world.start.x, world.start.y);
    const gc = world.cellOf(world.goal.x, world.goal.y);
    val[idx(gc.i, gc.j)] = 2;
    let frontier = [[gc.i, gc.j]];
    const layers = [frontier];

    while (frontier.length) {
      const next = [];
      for (const [i, j] of frontier) {
        const here = val[idx(i, j)];
        for (const [ni, nj] of [[i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1]]) {
          if (ni < 0 || nj < 0 || ni >= cols || nj >= rows) continue;
          const k = idx(ni, nj);
          if (val[k] !== 0) continue;
          val[k] = here + 1;
          next.push([ni, nj]);
        }
      }
      if (next.length) layers.push(next);
      frontier = next;
    }

    const sv = val[idx(sc.i, sc.j)];
    let path = [];
    if (sv >= 2) {
      let cur = [sc.i, sc.j];
      path.push(cur);
      while (val[idx(cur[0], cur[1])] > 2) {
        const [i, j] = cur;
        const want = val[idx(i, j)] - 1;
        let nextCell = null;
        for (const [ni, nj] of [[i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1], [i - 1, j - 1], [i + 1, j - 1], [i - 1, j + 1], [i + 1, j + 1]]) {
          if (ni < 0 || nj < 0 || ni >= cols || nj >= rows) continue;
          if (val[idx(ni, nj)] === want) { nextCell = [ni, nj]; break; }
        }
        if (!nextCell) break;
        cur = nextCell;
        path.push(cur);
      }
    }

    return { val, layers, path, reachable: sv >= 2, cellSize, cols, rows };
  }

  function heatColor(t) {
    const stops = [[0.0, [214, 150, 43]], [0.5, [120, 160, 90]], [1.0, [59, 110, 160]]];
    for (let i = 1; i < stops.length; i++) {
      if (t <= stops[i][0]) {
        const [t0, c0] = stops[i - 1], [t1, c1] = stops[i];
        const f = (t - t0) / (t1 - t0 || 1);
        const c = c0.map((v, k) => Math.round(v + (c1[k] - v) * f));
        return `rgb(${c[0]},${c[1]},${c[2]})`;
      }
    }
    return "rgb(59,110,160)";
  }

  function draw(ctx, world, data, layersRevealed, pathRevealed) {
    const { cellSize } = world;
    ctx.save();
    ctx.fillStyle = "#eef0f2";
    ctx.fillRect(0, 0, world.width, world.height);
    const maxD = data.layers.length || 1;
    for (let d = 0; d < layersRevealed && d < data.layers.length; d++) {
      ctx.fillStyle = heatColor(d / maxD);
      for (const [i, j] of data.layers[d]) ctx.fillRect(i * cellSize, j * cellSize, cellSize + 0.5, cellSize + 0.5);
    }
    ctx.restore();
    world.draw(ctx, { fill: "#3a3f47", stroke: "#20242b", alpha: 1 });

    if (pathRevealed > 0 && data.path.length) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      const n = Math.min(pathRevealed, data.path.length);
      for (let k = 0; k < n; k++) {
        const [i, j] = data.path[k];
        const x = (i + 0.5) * cellSize, y = (j + 0.5) * cellSize;
        if (k === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height }) {
    const world = makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3), cellSize: 7 });
    const data = computeWaveFront(world);
    let idx = 0;
    const totalLayerSteps = data.layers.length;
    const totalSteps = totalLayerSteps + (data.reachable ? data.path.length : 1);
    return {
      draw(ctx) {
        const layersRevealed = Math.min(idx, totalLayerSteps);
        const pathRevealed = Math.max(0, idx - totalLayerSteps);
        draw(ctx, world, data, layersRevealed, pathRevealed);
      },
      step() {
        idx = Math.min(idx + 1, totalSteps);
        const done = idx >= totalSteps;
        let note;
        if (idx < totalLayerSteps) {
          note = `Numbering ring ${idx} of ${totalLayerSteps} outward from q<sub>goal</sub> (value ${idx + 1}).`;
        } else if (data.reachable) {
          note = "Walking downhill from q<sub>start</sub>: at each cell, step to any neighbor numbered exactly one lower.";
          if (done) note = `Path extracted — length ${data.path.length - 1} grid steps. The start cell's number (minus one) gives the shortest path length in this grid's connectivity, and this downhill walk works from any starting cell.`;
        } else {
          note = "Every reachable free cell has been numbered and q<sub>start</sub> never got one — no path exists.";
        }
        return { done, note };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.wavefront = {
    title: "Wave-Front Planner",
    badge: "§4.2 / book §4.5",
    subtitle: "BFS numbering outward from the goal; any start's number gives the path length, found by walking downhill.",
    width: 560, height: 360,
    legend: [
      { color: "rgb(214,150,43)", label: "close to goal" },
      { color: "rgb(59,110,160)", label: "far from goal" },
      { color: "#2f8f5b", label: "extracted path" },
    ],
    makeSim,
    pythonCode: `
from collections import deque

def wavefront(grid, start, goal):
    """grid[j][i] == 1 for obstacles. Seeds the flood fill at the GOAL, so the
    resulting field lets a robot starting anywhere reach the goal by walking
    downhill -- that is what makes the planner immune to local minima.
    Returns the path as a list of (i, j) cells, or None if unreachable."""
    rows, cols = len(grid), len(grid[0])
    val = [[1 if grid[j][i] else 0 for i in range(cols)] for j in range(rows)]
    gi, gj = goal
    val[gj][gi] = 2
    q = deque([(gi, gj)])

    while q:
        i, j = q.popleft()
        for ni, nj in neighbors4(i, j, cols, rows):
            if val[nj][ni] == 0:
                val[nj][ni] = val[j][i] + 1
                q.append((ni, nj))

    si, sj = start
    if val[sj][si] < 2:
        return None                      # never numbered -> unreachable

    path = [(si, sj)]
    i, j = si, sj
    while val[j][i] > 2:
        want = val[j][i] - 1
        i, j = next(n for n in neighbors8(i, j, cols, rows) if val[n[1]][n[0]] == want)
        path.append((i, j))
    return path                          # already ordered start -> goal
`,
  };
})();
