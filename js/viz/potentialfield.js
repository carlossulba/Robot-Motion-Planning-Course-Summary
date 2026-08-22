/* Potential Field visualization: conic-quadratic attractive field + repulsive
   fields around obstacles, robot follows the numerical gradient downhill.
   Deliberately NOT a navigation function, so it can (and sometimes does)
   get stuck in a local minimum -- that failure mode is the whole point. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  const RHO0 = 55, DSTAR = 90;

  function Uatt(p, goal, xi) {
    const d = Math.hypot(p.x - goal.x, p.y - goal.y);
    return d <= DSTAR ? 0.5 * xi * d * d : DSTAR * xi * d - 0.5 * xi * DSTAR * DSTAR;
  }

  function Urep(p, world, eta) {
    let u = 0;
    for (const o of world.obstacles) {
      const d = o.type === "circle"
        ? Math.max(0.001, Math.hypot(p.x - o.cx, p.y - o.cy) - o.r)
        : Math.max(0.001, window.RMP.geom.distToPoly(p.x, p.y, o.pts));
      if (d < RHO0) u += 0.5 * eta * Math.pow(1 / d - 1 / RHO0, 2);
    }
    return Math.min(u, 4000);
  }

  function U(p, world, xi, eta) { return Uatt(p, world.goal, xi) + Urep(p, world, eta); }

  function numGrad(p, world, xi, eta) {
    const h = 1.5;
    const dx = (U({ x: p.x + h, y: p.y }, world, xi, eta) - U({ x: p.x - h, y: p.y }, world, xi, eta)) / (2 * h);
    const dy = (U({ x: p.x, y: p.y + h }, world, xi, eta) - U({ x: p.x, y: p.y - h }, world, xi, eta)) / (2 * h);
    return { x: dx, y: dy, mag: Math.hypot(dx, dy) };
  }

  function computePath(world, xi, eta) {
    const STEP = 4;
    const events = [{ x: world.start.x, y: world.start.y, note: "Start. The robot follows −∇U(q) (steepest descent) at every step." }];
    let pos = { x: world.start.x, y: world.start.y };
    let guard = 0, stallCount = 0;
    while (guard++ < 900) {
      const d = Math.hypot(pos.x - world.goal.x, pos.y - world.goal.y);
      if (d < STEP * 1.3) { events.push({ x: world.goal.x, y: world.goal.y, phase: "reached", note: "Reached q<sub>goal</sub> — global minimum of U found." }); return { events, success: true }; }
      const g = numGrad(pos, world, xi, eta);
      if (g.mag < 0.06) {
        stallCount++;
        if (stallCount > 3) { events.push({ x: pos.x, y: pos.y, phase: "stuck", note: "∇U ≈ 0 here but this isn't q<sub>goal</sub> — stuck in a local minimum, the classic failure mode of potential fields." }); return { events, success: false }; }
      } else stallCount = 0;
      const dir = g.mag > 1e-6 ? { x: -g.x / g.mag, y: -g.y / g.mag } : { x: 0, y: 0 };
      const next = { x: pos.x + dir.x * STEP, y: pos.y + dir.y * STEP };
      if (!world.isFree(next.x, next.y)) { events.push({ x: pos.x, y: pos.y, phase: "stuck", note: "Gradient step would enter an obstacle (numerical edge case near a corner) — stopping." }); return { events, success: false }; }
      pos = next;
      events.push({ x: pos.x, y: pos.y, phase: "descent", note: "Descending −∇U(q)." });
    }
    events.push({ x: pos.x, y: pos.y, phase: "stuck", note: "Iteration limit reached." });
    return { events, success: false };
  }

  function buildHeatmap(world, xi, eta) {
    const cell = 9;
    const cols = Math.ceil(world.width / cell), rows = Math.ceil(world.height / cell);
    const vals = new Float32Array(cols * rows);
    let max = 0;
    for (let j = 0; j < rows; j++) for (let i = 0; i < cols; i++) {
      const v = Math.log(1 + U({ x: (i + 0.5) * cell, y: (j + 0.5) * cell }, world, xi, eta));
      vals[j * cols + i] = v;
      if (v > max) max = v;
    }
    return { vals, cols, rows, cell, max };
  }

  function heatColor(t) {
    const stops = [[0, [59, 110, 160]], [0.45, [120, 160, 90]], [0.75, [214, 150, 43]], [1, [193, 60, 60]]];
    for (let i = 1; i < stops.length; i++) {
      if (t <= stops[i][0]) {
        const [t0, c0] = stops[i - 1], [t1, c1] = stops[i];
        const f = (t - t0) / (t1 - t0 || 1);
        const c = c0.map((v, k) => Math.round(v + (c1[k] - v) * f));
        return `rgb(${c[0]},${c[1]},${c[2]})`;
      }
    }
    return "rgb(193,60,60)";
  }

  function draw(ctx, world, heat, events, idx) {
    for (let j = 0; j < heat.rows; j++) for (let i = 0; i < heat.cols; i++) {
      ctx.fillStyle = heatColor(heat.vals[j * heat.cols + i] / (heat.max || 1));
      ctx.fillRect(i * heat.cell, j * heat.cell, heat.cell + 0.5, heat.cell + 0.5);
    }
    world.draw(ctx, { fill: "#2c2f36", stroke: "#14161a", alpha: 1 });

    ctx.save();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2.6;
    ctx.beginPath();
    for (let i = 1; i <= idx; i++) {
      if (i === 1) ctx.moveTo(events[0].x, events[0].y);
      ctx.lineTo(events[i].x, events[i].y);
    }
    ctx.stroke();
    ctx.restore();

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
    const cur = events[idx];
    const color = cur.phase === "stuck" ? "#c23b3b" : cur.phase === "reached" ? "#2f8f5b" : "#ffffff";
    ctx.save();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(cur.x, cur.y, 5.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#14161a";
    ctx.lineWidth = 1.2;
    ctx.stroke();
    ctx.restore();
  }

  function makeSim({ rng, params, width, height, world }) {
    const xi = (params && params.xi) || 1.1;
    const eta = (params && params.eta) || 2200;
    const w = world || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const heat = buildHeatmap(w, xi, eta);
    const { events, success } = computePath(w, xi, eta);
    let idx = 0;
    return {
      draw(ctx) { draw(ctx, w, heat, events, idx); },
      step() {
        idx = Math.min(idx + 1, events.length - 1);
        const done = idx >= events.length - 1;
        let note = events[idx].note;
        if (done && success) note += " Reaching the goal isn't guaranteed for a plain potential field — click Generate new a few times and watch for local minima.";
        return { done, note };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.potentialfield = {
    title: "Attractive/Repulsive Potential Field",
    badge: "§4.1 / book §4.1",
    subtitle: "Gradient descent on U = U_att + U_rep. Unlike a navigation function, this field can (and sometimes does) trap the robot in a local minimum.",
    width: 560, height: 360,
    legend: [
      { color: "rgb(59,110,160)", label: "low potential" },
      { color: "rgb(193,60,60)", label: "high potential (near obstacle)" },
      { color: "#ffffff", label: "descent path" },
    ],
    params: [
      { key: "xi", label: "attractive gain ξ", type: "range", min: 0.3, max: 3, step: 0.1, value: 1.1 },
      { key: "eta", label: "repulsive gain η", type: "range", min: 400, max: 6000, step: 100, value: 2200 },
    ],
    makeSim,
    pythonCode: `
def U_att(q, goal, xi=1.1, d_star=90):
    d = dist(q, goal)
    if d <= d_star:
        return 0.5 * xi * d**2                      # quadratic near the goal
    return d_star * xi * d - 0.5 * xi * d_star**2    # conic (linear) far away

def U_rep(q, obstacles, eta=2200, rho0=55):
    total = 0
    for obs in obstacles:
        d = max(1e-3, distance_to(q, obs))
        if d < rho0:
            total += 0.5 * eta * (1 / d - 1 / rho0) ** 2
    return total

def gradient_descent(start, goal, obstacles, step=4, max_iters=900):
    pos, path = start, [start]
    for _ in range(max_iters):
        if dist(pos, goal) < step * 1.3:
            return path + [goal]                     # success

        g = numerical_gradient(lambda q: U_att(q, goal) + U_rep(q, obstacles), pos)
        if norm(g) < 0.06:
            return None                               # stuck: local minimum

        pos = pos - step * normalize(g)                # steepest descent
        path.append(pos)
    return None
`,
  };
})();
