/* Potential Field visualization: attractive + repulsive fields, robot follows
   the numerical gradient downhill. Deliberately NOT a navigation function, so
   it can (and sometimes does) get stuck in a local minimum -- that failure
   mode is the whole point, and is reliably reproducible via the occasional
   concave "horseshoe" obstacle (book Fig. 4.10's classic trap). */
(function () {
  "use strict";
  const { makeWorld, finishWorld } = window.RMP;

  const RHO0 = 55, DSTAR = 90;

  // ---- stepping constants -------------------------------------------------
  // The robot takes a step of size min(GLOBAL_MAX, ALPHA*|grad|, 0.6*clearance)
  // in the direction of steepest descent, instead of a fixed-length step in a
  // normalized direction. This is what actually lets xi/eta drive *speed* (not
  // just steering angle), and is what makes the conic/quadratic/piecewise
  // attractive potentials behave differently near the goal (see refMag below).
  const BASE_STEP = 4;      // "cruising" step size, matches the old fixed STEP
  const GLOBAL_MAX = 10;    // hard safety cap on any single step
  const REACH = 1.8;        // capture radius around q_goal
  const STALL_EPS = 0.06;   // |grad| this small = essentially a fixed point
  const PROGRESS_WINDOW = 14, PROGRESS_EPS = 1.2; // "no real progress lately" detector
  const RANDOM_STEP = 14, RANDOM_BURST = 6, RANDOM_WALK_CAP = 500;
  const MAIN_GUARD = 3600;
  const HORSESHOE_CHANCE = 0.4;

  // ---- attractive / repulsive potentials -----------------------------------
  function Uatt(p, goal, xi, type) {
    const d = Math.hypot(p.x - goal.x, p.y - goal.y);
    if (type === "conic") return xi * d;
    if (type === "quadratic") return 0.5 * xi * d * d;
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
    return u; // no artificial cap: log-scaled heatmap and the adaptive step
              // clamp both handle arbitrarily large values fine, and a value
              // cap here used to flatten the gradient right next to an
              // obstacle -- exactly where it must NOT go flat.
  }

  function U(p, world, xi, eta, type) { return Uatt(p, world.goal, xi, type) + Urep(p, world, eta); }

  function numGrad(p, world, xi, eta, type) {
    const h = 1.5;
    const dx = (U({ x: p.x + h, y: p.y }, world, xi, eta, type) - U({ x: p.x - h, y: p.y }, world, xi, eta, type)) / (2 * h);
    const dy = (U({ x: p.x, y: p.y + h }, world, xi, eta, type) - U({ x: p.x, y: p.y - h }, world, xi, eta, type)) / (2 * h);
    return { x: dx, y: dy, mag: Math.hypot(dx, dy) };
  }

  // Reference |grad U_att| used to normalize step size across the three
  // attractive-potential modes so they share the same "cruising" pace despite
  // having differently-scaled formulas: piecewise/quadratic's far-field slope
  // is xi*DSTAR (that's literally the piecewise formula's linear branch);
  // conic's slope is the constant xi everywhere. See computePath for how this
  // plays out: piecewise caps at BASE_STEP far away then decelerates smoothly
  // inside d*; quadratic keeps accelerating past that cap (unbounded gradient
  // -> bigger strides far from the goal, capped only by GLOBAL_MAX for
  // stability) and also decelerates smoothly near the goal (same formula as
  // piecewise there); conic never decelerates, so it can overshoot q_goal and
  // needs a corrective step right at the end -- the textbook "chatter".
  function refMag(xi, type) { return type === "conic" ? xi : xi * DSTAR; }

  // ---- deliberate horseshoe/local-minimum-trap world -----------------------
  function makeHorseshoePolygon(cx, cy, outerR, innerR, gapAngle, gapWidth, n) {
    const a0 = gapAngle + gapWidth / 2;
    const sweep = Math.PI * 2 - gapWidth;
    const pts = [];
    for (let k = 0; k <= n; k++) { const a = a0 + (k / n) * sweep; pts.push([cx + Math.cos(a) * outerR, cy + Math.sin(a) * outerR]); }
    for (let k = n; k >= 0; k--) { const a = a0 + (k / n) * sweep; pts.push([cx + Math.cos(a) * innerR, cy + Math.sin(a) * innerR]); }
    return { type: "poly", pts, cx, cy };
  }

  // Book Fig. 4.10's classic trap: the goal sits outside a near-complete ring
  // wall, the start sits inside the bowl the wall encloses. Pulling straight
  // toward the goal drives the robot into the inner wall surface; repulsion
  // there exactly cancels attraction well before it could find its way around
  // to the (deliberately off-axis) gap -- a genuine local minimum, not a
  // dead end, since the field really does go to (near) zero there.
  function makeTrapWorld(rng, width, height) {
    const cx = 110 + rng() * 120;
    const cy = 90 + rng() * 180;
    const outerR = 50 + rng() * 15;
    const innerR = outerR * (0.55 + rng() * 0.15);
    const goal = {
      x: Math.min(width - 34, cx + outerR + 60 + rng() * 40),
      y: Math.max(34, Math.min(height - 34, cy + (rng() - 0.5) * 60)),
    };
    const side = rng() < 0.5 ? -1 : 1;
    const gapAngle = side * (1.3 + rng() * 1.0); // 74-131 deg off the goal direction -- never facing the goal directly
    const gapWidth = (36 + rng() * 20) * Math.PI / 180;
    const wall = makeHorseshoePolygon(cx, cy, outerR, innerR, gapAngle, gapWidth, 22);
    const startR = innerR * (0.15 + rng() * 0.35);
    const startA = rng() * Math.PI * 2;
    const start = { x: cx + Math.cos(startA) * startR, y: cy + Math.sin(startA) * startR };
    const world = finishWorld({ width, height, margin: 34, obstacles: [wall], start, goal, cellSize: 6 });
    world.isTrap = true;
    return world;
  }

  function makeDemoWorld(rng, width, height) {
    if (rng() < HORSESHOE_CHANCE) return makeTrapWorld(rng, width, height);
    return makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
  }

  // ---- path computation -----------------------------------------------------
  function computePath(world, xi, eta, type, rng, randomWalk) {
    const ALPHA = BASE_STEP / refMag(xi, type);
    const events = [{ x: world.start.x, y: world.start.y, phase: "start", note: "Start. The robot follows −∇U(q) (steepest descent) at every step." }];
    let pos = { x: world.start.x, y: world.start.y };
    let guard = 0, randomUsed = 0;
    let distHistory = [];

    while (guard++ < MAIN_GUARD) {
      const d = Math.hypot(pos.x - world.goal.x, pos.y - world.goal.y);
      if (d < REACH) {
        events.push({ x: world.goal.x, y: world.goal.y, phase: "reached", note: "Reached q<sub>goal</sub> — global minimum of U found." });
        return { events, success: true };
      }
      const g = numGrad(pos, world, xi, eta, type);
      distHistory.push(d);
      if (distHistory.length > PROGRESS_WINDOW) distHistory.shift();
      const noProgress = distHistory.length === PROGRESS_WINDOW && (distHistory[0] - Math.min(...distHistory)) < PROGRESS_EPS;

      let next = null, blocked = false;
      if (g.mag >= STALL_EPS && !noProgress) {
        const clearance = world.distToNearestObstacle(pos.x, pos.y);
        const stepMag = Math.min(GLOBAL_MAX, ALPHA * g.mag, Math.max(0.4, clearance * 0.6));
        const dir = { x: -g.x / g.mag, y: -g.y / g.mag };
        next = { x: pos.x + dir.x * stepMag, y: pos.y + dir.y * stepMag };
        blocked = !world.isFree(next.x, next.y);
      }

      if (g.mag < STALL_EPS || noProgress || blocked) {
        if (randomWalk && randomUsed < RANDOM_WALK_CAP) {
          for (let b = 0; b < RANDOM_BURST && randomUsed < RANDOM_WALK_CAP; b++) {
            const ang = rng() * Math.PI * 2;
            const nx = pos.x + Math.cos(ang) * RANDOM_STEP, ny = pos.y + Math.sin(ang) * RANDOM_STEP;
            if (world.isFree(nx, ny)) {
              pos = { x: nx, y: ny };
              events.push({ x: pos.x, y: pos.y, phase: "randomwalk", note: "Stuck in a local minimum (∇U ≈ 0) — taking small random steps to try to escape." });
              randomUsed++;
            }
          }
          distHistory = [];
          continue;
        }
        const hint = world.isTrap && !randomWalk ? " This is a concave horseshoe obstacle — try “random walk when stuck” below." : "";
        const base = blocked && g.mag >= STALL_EPS && !noProgress
          ? "Gradient step would enter an obstacle (numerical edge case near a corner) — stopping."
          : "∇U ≈ 0 here but this isn't q<sub>goal</sub> — stuck in a local minimum, the classic failure mode of potential fields.";
        events.push({ x: pos.x, y: pos.y, phase: "stuck", note: base + hint });
        return { events, success: false };
      }

      pos = next;
      events.push({ x: pos.x, y: pos.y, phase: "descent", note: "Descending −∇U(q)." });
    }
    events.push({ x: pos.x, y: pos.y, phase: "stuck", note: "Iteration limit reached." });
    return { events, success: false };
  }

  // The attractive potential's gradient spans the whole canvas and grows
  // with distance from the goal, so on the heatmap alone it visually
  // dominates the color range -- each obstacle's own repulsive bump is
  // real and correctly centered on that obstacle (see Urep above, which
  // sums a per-obstacle term keyed on distance to *that* obstacle, not to
  // the goal), but it's easy to mistake the field for "just distance from
  // goal" at a glance. This samples world.distToNearestObstacle on a fine
  // grid and keeps every sample that straddles the ρ0 influence radius
  // (a sign change vs. a right/down neighbor), producing a dotted contour
  // that hugs each obstacle's actual footprint -- circle or polygon alike
  // -- at exactly the boundary where Urep's contribution starts. Drawn as
  // a bright dotted ring in draw(), this makes "repulsion is centered
  // here, at each obstacle" visually unambiguous instead of needing to
  // read the heatmap colors closely.
  function buildRho0Contour(world) {
    const cell = 4;
    const cols = Math.ceil(world.width / cell) + 1;
    const rows = Math.ceil(world.height / cell) + 1;
    const d = new Float32Array(cols * rows);
    for (let j = 0; j < rows; j++) for (let i = 0; i < cols; i++) {
      d[j * cols + i] = world.distToNearestObstacle(i * cell, j * cell);
    }
    const pts = [];
    for (let j = 0; j < rows; j++) for (let i = 0; i < cols; i++) {
      const v = d[j * cols + i];
      const right = i + 1 < cols ? d[j * cols + i + 1] : v;
      const down = j + 1 < rows ? d[(j + 1) * cols + i] : v;
      if ((v - RHO0) * (right - RHO0) < 0 || (v - RHO0) * (down - RHO0) < 0) {
        pts.push(i * cell, j * cell);
      }
    }
    return pts;
  }

  function buildHeatmap(world, xi, eta, type) {
    const cell = 9;
    const cols = Math.ceil(world.width / cell), rows = Math.ceil(world.height / cell);
    const vals = new Float32Array(cols * rows);
    let max = 0;
    for (let j = 0; j < rows; j++) for (let i = 0; i < cols; i++) {
      const v = Math.log(1 + U({ x: (i + 0.5) * cell, y: (j + 0.5) * cell }, world, xi, eta, type));
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

  function draw(ctx, world, heat, rho0Pts, events, idx) {
    for (let j = 0; j < heat.rows; j++) for (let i = 0; i < heat.cols; i++) {
      ctx.fillStyle = heatColor(heat.vals[j * heat.cols + i] / (heat.max || 1));
      ctx.fillRect(i * heat.cell, j * heat.cell, heat.cell + 0.5, heat.cell + 0.5);
    }
    world.draw(ctx, { fill: "#2c2f36", stroke: "#14161a", alpha: 1 });

    // ρ0 influence-radius contour: a dotted ring hugging every obstacle,
    // marking exactly where each obstacle's own repulsive term switches on.
    ctx.save();
    ctx.fillStyle = "#ff4fd8";
    for (let k = 0; k < rho0Pts.length; k += 2) {
      ctx.beginPath();
      ctx.arc(rho0Pts[k], rho0Pts[k + 1], 1.3, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();

    ctx.save();
    ctx.lineWidth = 2.6;
    ctx.beginPath();
    let dashed = null;
    for (let i = 1; i <= idx; i++) {
      const rw = events[i].phase === "randomwalk";
      if (dashed === null || dashed !== rw) {
        if (dashed !== null) ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(events[i - 1].x, events[i - 1].y);
        ctx.strokeStyle = rw ? "#e0a339" : "#ffffff";
        ctx.setLineDash(rw ? [4, 3] : []);
        dashed = rw;
      }
      ctx.lineTo(events[i].x, events[i].y);
    }
    if (dashed !== null) ctx.stroke();
    ctx.restore();

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
    const cur = events[idx];
    const color = cur.phase === "stuck" ? "#c23b3b" : cur.phase === "reached" ? "#2f8f5b" : cur.phase === "randomwalk" ? "#e0a339" : "#ffffff";
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

  // ---- formula side panel ---------------------------------------------------
  function fmt(v, maxFrac) {
    return Number(v).toLocaleString(undefined, { maximumFractionDigits: maxFrac === undefined ? 2 : maxFrac });
  }

  function attFormulaHTML(type) {
    if (type === "conic") {
      return `<div><strong>U<sub>att</sub></strong>(q) = ξ · d</div>
        <div style="color:var(--code-comment);font-size:0.82em;margin-top:0.15em;">conic: bounded gradient, but direction-discontinuous (chatter) right at the goal</div>`;
    }
    if (type === "quadratic") {
      return `<div><strong>U<sub>att</sub></strong>(q) = &#189; ξ d&sup2;</div>
        <div style="color:var(--code-comment);font-size:0.82em;margin-top:0.15em;">quadratic: unbounded gradient far away, smooth stop at the goal</div>`;
    }
    return `<div><strong>U<sub>att</sub></strong>(q) =</div>
      <div style="padding-left:0.9em;">&#189; ξ d&sup2; &nbsp;<span style="color:var(--code-comment)">d ≤ d*</span></div>
      <div style="padding-left:0.9em;">d*ξd − &#189;ξ(d*)&sup2; &nbsp;<span style="color:var(--code-comment)">d &gt; d*</span></div>
      <div style="color:var(--code-comment);font-size:0.82em;margin-top:0.15em;">piecewise: quadratic near the goal, conic (capped) far away</div>`;
  }

  function formulaHTML(xi, eta, type) {
    const dstarLine = type === "piecewise" ? ` &nbsp; d* = ${DSTAR}` : "";
    return `
      ${attFormulaHTML(type)}
      <div style="margin-top:0.7em;"><strong>U<sub>rep</sub></strong>(q) =</div>
      <div style="padding-left:0.9em;">&#189; η (1/D(q) − 1/ρ₀)&sup2; &nbsp;<span style="color:var(--code-comment)">D(q) ≤ ρ₀</span></div>
      <div style="padding-left:0.9em;">0 &nbsp;<span style="color:var(--code-comment)">D(q) &gt; ρ₀</span></div>
      <div style="margin-top:0.7em;padding-top:0.5em;border-top:1px solid rgba(255,255,255,0.15);font-size:0.82em;color:var(--code-comment);">
        ξ = ${fmt(xi)} &nbsp; η = ${fmt(eta, 0)} &nbsp; ρ₀ = ${RHO0}${dstarLine}
      </div>
      <div style="margin-top:0.3em;font-size:0.78em;color:var(--code-comment);">U = U<sub>att</sub> + U<sub>rep</sub>, d = dist(q,q<sub>goal</sub>), D(q) = dist to nearest obstacle</div>
    `;
  }

  // ---- sim lifecycle ----------------------------------------------------------
  function makeSim({ rng, params, width, height, world }) {
    const xi = (params && params.xi) || 1.1;
    const eta = (params && params.eta) || 2200;
    const attType = (params && params.attType) || "piecewise";
    const randomWalk = !!(params && params.randomWalk);
    const w = world || makeDemoWorld(rng, width, height);
    const heat = buildHeatmap(w, xi, eta, attType);
    const rho0Pts = buildRho0Contour(w);
    const { events, success } = computePath(w, xi, eta, attType, rng, randomWalk);
    let idx = 0;
    return {
      sidePanelHTML: formulaHTML(xi, eta, attType),
      draw(ctx) { draw(ctx, w, heat, rho0Pts, events, idx); },
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
      { color: "#e0a339", label: "random walk (when stuck)" },
      { color: "#ff4fd8", label: "ρ₀ influence radius (repulsion boundary, per obstacle)" },
    ],
    params: [
      { key: "xi", label: "attractive gain ξ", type: "range", scale: "log", min: 0.03, max: 40, value: 1.1 },
      { key: "eta", label: "repulsive gain η", type: "range", scale: "log", min: 150, max: 150000, value: 2200 },
      {
        key: "attType", label: "U_att shape", type: "select", value: "piecewise",
        options: [
          { value: "conic", label: "Conic" },
          { value: "quadratic", label: "Quadratic" },
          { value: "piecewise", label: "Piecewise" },
        ],
      },
      { key: "randomWalk", label: "random walk when stuck", type: "checkbox", value: false },
    ],
    sidePanel: { title: "Potential function" },
    makeSim,
    pythonCode: `
def U_att(q, goal, xi=1.1, shape="piecewise", d_star=90):
    d = dist(q, goal)
    if shape == "conic":
        return xi * d                                # bounded gradient, chatters at the goal
    if shape == "quadratic":
        return 0.5 * xi * d**2                        # unbounded gradient far away, smooth stop
    if d <= d_star:
        return 0.5 * xi * d**2                         # piecewise: quadratic near the goal
    return d_star * xi * d - 0.5 * xi * d_star**2       # piecewise: conic (capped) far away

def U_rep(q, obstacles, eta=2200, rho0=55):
    total = 0
    for obs in obstacles:
        d = max(1e-3, distance_to(q, obs))
        if d < rho0:
            total += 0.5 * eta * (1 / d - 1 / rho0) ** 2
    return total

def gradient_descent(start, goal, obstacles, random_walk=False, max_iters=3600):
    pos, path = start, [start]
    for _ in range(max_iters):
        if dist(pos, goal) < reach:
            return path + [goal]                       # success

        g = numerical_gradient(lambda q: U_att(q, goal) + U_rep(q, obstacles), pos)
        if norm(g) < eps or no_recent_progress(path):
            if random_walk:
                pos = pos + random_direction() * step    # small random kick, then resume descent
                path.append(pos)
                continue
            return None                                 # stuck: local minimum

        step_len = min(max_step, alpha * norm(g), 0.6 * clearance(pos, obstacles))
        pos = pos - step_len * normalize(g)               # step scales with |grad| -> real speed control
        path.append(pos)
    return None
`,
  };
})();
