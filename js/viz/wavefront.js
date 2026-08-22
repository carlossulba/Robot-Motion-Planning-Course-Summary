/* Wave-Front visualization: single-source BFS numbering from the GOAL cell.
   Seeding at the goal (not the start) is what makes the planner immune to
   local minima: every free cell ends up with a value that strictly decreases
   toward the goal, so a robot starting anywhere can reach the goal by pure
   greedy descent -- no need to recompute anything for a different start. */
(function () {
  "use strict";
  const { makeWorld, gridFlood } = window.RMP;

  function computeWaveFront(world, connectivity) {
    const { cols, rows, grid, cellSize } = world;
    const idx = (i, j) => j * cols + i;
    const nbrFn = gridFlood.neighborsFor(connectivity);

    const sc = world.cellOf(world.start.x, world.start.y);
    const gc = world.cellOf(world.goal.x, world.goal.y);

    const { val, layers } = gridFlood.bfsLayers(cols, rows, grid, [[gc.i, gc.j]], {
      connectivity, seedValue: 2,
    });

    const sv = val[idx(sc.i, sc.j)];
    let path = [];
    if (sv >= 2) {
      let cur = [sc.i, sc.j];
      path.push(cur);
      while (val[idx(cur[0], cur[1])] > 2) {
        const [i, j] = cur;
        const want = val[idx(i, j)] - 1;
        let nextCell = null;
        for (const [ni, nj] of nbrFn(i, j)) {
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

  const heatColor = gridFlood.heatColorFactory([[0.0, [214, 150, 43]], [0.5, [120, 160, 90]], [1.0, [59, 110, 160]]]);

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

  // pseudocode line indices (0-based)
  const L_SEED = 0, L_PROPAGATE = 2, L_EXTRACT = 3, L_EXTRACT_DONE = 4;

  function makeSim({ rng, params, width, height, world }) {
    const conn = params && params.connectivity ? 8 : 4;
    const w = world || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3), cellSize: 7 });
    const data = computeWaveFront(w, conn);
    let idx = 0;
    const totalLayerSteps = data.layers.length;
    const totalSteps = totalLayerSteps + (data.reachable ? data.path.length : 1);
    return {
      draw(ctx) {
        const layersRevealed = Math.min(idx, totalLayerSteps);
        const pathRevealed = Math.max(0, idx - totalLayerSteps);
        draw(ctx, w, data, layersRevealed, pathRevealed);
      },
      step() {
        idx = Math.min(idx + 1, totalSteps);
        const done = idx >= totalSteps;
        let note, line;
        if (idx === 1) {
          note = `Seed: q_goal's cell <- 2 (obstacle cells already <- 1).`;
          line = L_SEED;
        } else if (idx < totalLayerSteps) {
          note = `Numbering ring ${idx} of ${totalLayerSteps} outward from q<sub>goal</sub> (value ${idx + 1}, ${conn}-connectivity).`;
          line = L_PROPAGATE;
        } else if (data.reachable) {
          const atGoalStep = done;
          note = "Walking downhill from q<sub>start</sub>: at each cell, step to a strictly smaller neighbor value.";
          line = L_EXTRACT;
          if (atGoalStep) {
            note = `Path extracted — length ${data.path.length - 1} grid steps, reaching q<sub>goal</sub>. The start cell's number (minus one) gives the shortest path length in this grid's connectivity, and this downhill walk works from any starting cell.`;
            line = L_EXTRACT_DONE;
          }
        } else {
          note = "Every reachable free cell has been numbered and q<sub>start</sub> never got one — no path exists.";
          line = L_PROPAGATE;
        }
        return { done, note, line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.wavefront = {
    title: "Wave-Front Planner",
    badge: "§4.2 / book §4.5",
    subtitle: "BFS numbering outward from the goal; any start's number gives the path length, found by walking downhill.",
    width: 480, height: 320,
    legend: [
      { color: "rgb(214,150,43)", label: "close to goal" },
      { color: "rgb(59,110,160)", label: "far from goal" },
      { color: "#2f8f5b", label: "extracted path" },
    ],
    params: [
      { key: "connectivity", label: "8-connectivity", type: "checkbox", value: false },
    ],
    pseudocode: [
      "seed: goal cell <- 2, obstacle cells <- 1",
      "Propagate: repeat until every reachable free cell has a value:",
      { text: "free neighbor of a wave-k cell <- k + 1", indent: 1 },
      "extract path: from start, repeatedly step to a strictly",
      { text: "smaller neighbor value, until reaching the goal", indent: 1 },
    ],
    makeSim,
    pythonCode: `
from collections import deque

def wavefront(grid, start, goal, connectivity=4):
    """grid[j][i] == 1 for obstacles. Seeds the flood fill at the GOAL, so the
    resulting field lets a robot starting anywhere reach the goal by walking
    downhill -- that is what makes the planner immune to local minima.
    Returns the path as a list of (i, j) cells, or None if unreachable."""
    rows, cols = len(grid), len(grid[0])
    val = [[1 if grid[j][i] else 0 for i in range(cols)] for j in range(rows)]
    gi, gj = goal
    val[gj][gi] = 2
    q = deque([(gi, gj)])

    neighbors = neighbors8 if connectivity == 8 else neighbors4
    while q:
        i, j = q.popleft()
        for ni, nj in neighbors(i, j, cols, rows):
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
        i, j = next(n for n in neighbors(i, j, cols, rows) if val[n[1]][n[0]] == want)
        path.append((i, j))
    return path                          # already ordered start -> goal
`,
  };
})();
