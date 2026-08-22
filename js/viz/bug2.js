/* Bug2 visualization: greedy strategy — leave as soon as the fixed m-line is
   re-crossed closer to the goal than the hit point. */
(function () {
  "use strict";
  const { makeBugWorld, geom } = window.RMP;

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
  function distToLine(p, a, b) {
    const dx = b.x - a.x, dy = b.y - a.y, len = Math.hypot(dx, dy) || 1;
    return Math.abs((p.x - a.x) * dy - (p.y - a.y) * dx) / len;
  }

  // pseudocode line indices (0-based) -- see vizDefs.bug2.pseudocode below
  const L_MLINE = 0, L_TO_GOAL = 1, L_HIT = 2, L_FOLLOW = 3, L_LEAVE = 4, L_NO_PATH = 7, L_REACHED = 8;

  function computeBug2(world) {
    const STEP = 4, EPS = 3, LINE_TOL = 3;
    const mA = world.start, mB = world.goal;
    const events = [{ x: world.start.x, y: world.start.y, phase: "start", line: L_MLINE, note: "Start at q<sub>start</sub>. The m-line (start→goal) is fixed for the whole run." }];
    let pos = { x: world.start.x, y: world.start.y };
    const visited = new Set();
    let guard = 0;

    while (guard++ < 6000) {
      const toGoal = { x: world.goal.x - pos.x, y: world.goal.y - pos.y };
      const d = Math.hypot(toGoal.x, toGoal.y);
      if (d < STEP) {
        events.push({ x: world.goal.x, y: world.goal.y, phase: "reached", line: L_REACHED, note: "Reached q<sub>goal</sub> — path complete." });
        return { events, success: true };
      }
      const dir = { x: toGoal.x / d, y: toGoal.y / d };
      const next = { x: pos.x + dir.x * STEP, y: pos.y + dir.y * STEP };
      if (world.isFree(next.x, next.y)) {
        pos = next;
        events.push({ x: pos.x, y: pos.y, phase: "to_goal", line: L_TO_GOAL, note: "Motion-to-goal: heading straight for q<sub>goal</sub>." });
        continue;
      }
      const qH = { x: pos.x, y: pos.y };
      const dGoalH = dist(qH, world.goal);
      events.push({ x: qH.x, y: qH.y, phase: "hit", line: L_HIT, note: "Hit point q<sub>H</sub>. Follow the boundary until the m-line is crossed closer to the goal." });
      const obs = world.nearestObstacle(next.x, next.y);
      if (!obs || visited.has(obs)) {
        events.push({ x: qH.x, y: qH.y, phase: "stuck", line: L_NO_PATH, note: "Re-hit the departure point on the m-line without ever getting closer — no path exists." });
        return { events, success: false };
      }
      visited.add(obs);
      const boundary = geom.traceObstacleBoundary(obs, qH, STEP, EPS, Infinity);
      let found = false, arcWalked = 0;
      for (let i = 1; i < boundary.length; i++) {
        const p = boundary[i];
        events.push({ x: p.x, y: p.y, phase: "circumnav", line: L_FOLLOW, note: "Following the boundary, watching for the m-line." });
        arcWalked += STEP;
        if (arcWalked > STEP * 3 && distToLine(p, mA, mB) < LINE_TOL && dist(p, world.goal) < dGoalH) {
          pos = { x: p.x, y: p.y };
          events.push({ x: pos.x, y: pos.y, phase: "leave", line: L_LEAVE, note: "Back on the m-line, closer to the goal than q<sub>H</sub> — leave the boundary now." });
          found = true;
          break;
        }
      }
      if (!found) {
        events.push({ x: qH.x, y: qH.y, phase: "stuck", line: L_NO_PATH, note: "Circled the whole obstacle without a valid m-line crossing — no path exists." });
        return { events, success: false };
      }
    }
    events.push({ x: pos.x, y: pos.y, phase: "stuck", line: L_NO_PATH, note: "Iteration limit reached." });
    return { events, success: false };
  }

  function pathLength(events) {
    let L = 0;
    for (let i = 1; i < events.length; i++) L += dist(events[i - 1], events[i]);
    return L;
  }

  const PHASE_COLOR = {
    start: "#2b6cb0", to_goal: "#2b6cb0", hit: "#c23b3b",
    circumnav: "#b7532c", leave: "#2f8f5b", reached: "#2f8f5b", stuck: "#c23b3b",
  };

  function draw(ctx, world, events, idx) {
    world.draw(ctx);
    ctx.save();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = "#8892a0";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(world.start.x, world.start.y);
    ctx.lineTo(world.goal.x, world.goal.y);
    ctx.stroke();
    ctx.restore();

    for (let i = 1; i <= idx; i++) {
      ctx.beginPath();
      ctx.strokeStyle = PHASE_COLOR[events[i].phase] || "#2b6cb0";
      ctx.lineWidth = 2.4;
      ctx.moveTo(events[i - 1].x, events[i - 1].y);
      ctx.lineTo(events[i].x, events[i].y);
      ctx.stroke();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");

    const cur = events[idx];
    ctx.save();
    ctx.fillStyle = PHASE_COLOR[cur.phase] || "#2b6cb0";
    ctx.beginPath();
    ctx.arc(cur.x, cur.y, 5.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  function makeSim({ rng, width, height, world }) {
    const w = world || makeBugWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const { events, success } = computeBug2(w);
    let idx = 0;
    return {
      draw(ctx) { draw(ctx, w, events, idx); },
      step() {
        idx = Math.min(idx + 1, events.length - 1);
        const done = idx >= events.length - 1;
        let note = events[idx].note;
        if (done) {
          const L = pathLength(events.slice(0, idx + 1));
          const d0 = dist(w.start, w.goal);
          note += success ? ` Total path length ≈ ${L.toFixed(0)}px vs. straight-line d(q<sub>start</sub>,q<sub>goal</sub>) ≈ ${d0.toFixed(0)}px.` : "";
        }
        return { done, note, line: events[idx].line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.bug2 = {
    title: "Bug2",
    badge: "§3.3 / book §2.1",
    subtitle: "Greedy strategy: leave the boundary the instant you cross the fixed start–goal m-line closer to the goal.",
    width: 480, height: 320,
    legend: [
      { color: "#2b6cb0", label: "motion-to-goal" },
      { color: "#b7532c", label: "boundary-following" },
      { color: "#2f8f5b", label: "leave / reached" },
      { color: "#c23b3b", label: "hit point / stuck" },
    ],
    pseudocode: [
      "m-line = fixed line from q_start to q_goal",
      "move toward q_goal along the m-line",
      "if hit an obstacle boundary at q_H:",
      { text: "follow the boundary", indent: 1 },
      { text: "leave as soon as back on the m-line, closer to goal than q_H", indent: 1 },
      { text: "resume motion-to-goal from that leave point", indent: 1 },
      "if q_H is re-encountered:",
      { text: "report: no path exists", indent: 1 },
      "if q_goal reached: done",
    ],
    makeSim,
    pythonCode: `
def bug2(start, goal, is_free, step=0.05):
    """Bug2: leave the boundary as soon as the fixed m-line is crossed
    closer to the goal than the hit point."""
    m_line = Line(start, goal)          # fixed for the whole run
    pos, path = start, [start]
    visited_obstacles = set()

    while dist(pos, goal) > step:
        direction = normalize(goal - pos)
        nxt = pos + direction * step

        if is_free(nxt):
            pos = nxt
            path.append(pos)
            continue

        # --- hit point ---
        q_hit = pos
        d_hit = dist(q_hit, goal)
        obstacle = nearest_obstacle(nxt)
        if obstacle in visited_obstacles:
            return None                  # no path exists
        visited_obstacles.add(obstacle)

        for p in trace_boundary(obstacle, q_hit, step):   # walk the boundary
            path.append(p)
            if m_line.distance(p) < tol and dist(p, goal) < d_hit:
                pos = p                  # leave immediately
                break
        else:
            return None                  # full loop, never got closer

    path.append(goal)
    return path
`,
  };
})();
