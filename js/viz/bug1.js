/* Bug1 visualization: exhaustive circumnavigation, leave from the closest boundary point. */
(function () {
  "use strict";
  const { makeBugWorld, geom } = window.RMP;

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

  // pseudocode line indices (0-based) -- see vizDefs.bug1.pseudocode below
  const L_TO_GOAL = 0, L_HIT = 1, L_CIRCUMNAV = 2, L_RETURN = 3, L_LEAVE = 4, L_NO_PATH = 6, L_REACHED = 7;

  function computeBug1(world) {
    const STEP = 4, EPS = 3;
    const events = [{ x: world.start.x, y: world.start.y, phase: "start", line: L_TO_GOAL, note: "Start at q<sub>start</sub>, heading straight for q<sub>goal</sub> along the m-line." }];
    let pos = { x: world.start.x, y: world.start.y };
    const visited = new Set();
    let guard = 0, totalPerimeterUsed = 0;

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
        events.push({ x: pos.x, y: pos.y, phase: "to_goal", line: L_TO_GOAL, note: "Motion-to-goal: moving straight toward q<sub>goal</sub>." });
        continue;
      }
      const qH = { x: pos.x, y: pos.y };
      events.push({ x: qH.x, y: qH.y, phase: "hit", line: L_HIT, note: "Hit point q<sub>H</sub> — obstacle detected. Circumnavigate the entire boundary once." });
      const obs = world.nearestObstacle(next.x, next.y);
      if (!obs || visited.has(obs)) {
        events.push({ x: qH.x, y: qH.y, phase: "stuck", line: L_NO_PATH, note: "Re-hit an already-fully-circled obstacle with no progress — no path exists." });
        return { events, success: false };
      }
      visited.add(obs);
      const boundary = geom.traceObstacleBoundary(obs, qH, STEP, EPS, Infinity);
      let qL = boundary[0], minD = Infinity, leaveIdx = 0;
      boundary.forEach((p, i) => { const dd = dist(p, world.goal); if (dd < minD) { minD = dd; qL = p; leaveIdx = i; } });
      for (let i = 1; i < boundary.length; i++) {
        events.push({ x: boundary[i].x, y: boundary[i].y, phase: "circumnav", line: L_CIRCUMNAV, note: "Circumnavigating — recording distance-to-goal at every boundary point." });
        totalPerimeterUsed += STEP;
      }
      events.push({ x: qL.x, y: qL.y, phase: "found", line: L_RETURN, note: "Full loop complete: q<sub>L</sub> is the boundary point closest to q<sub>goal</sub> seen during the loop." });
      // retrace via whichever direction is actually shorter, not always forward
      const forwardSteps = leaveIdx;
      const backwardSteps = boundary.length - 1 - leaveIdx;
      if (backwardSteps < forwardSteps) {
        for (let i = boundary.length - 2; i >= leaveIdx; i--) {
          events.push({ x: boundary[i].x, y: boundary[i].y, phase: "return", line: L_RETURN, note: "Retracing the boundary (shorter direction) back to the leave point q<sub>L</sub>." });
          totalPerimeterUsed += STEP;
        }
      } else {
        for (let i = 1; i <= leaveIdx; i++) {
          events.push({ x: boundary[i].x, y: boundary[i].y, phase: "return", line: L_RETURN, note: "Retracing the boundary back to the leave point q<sub>L</sub>." });
          totalPerimeterUsed += STEP;
        }
      }
      pos = { x: qL.x, y: qL.y };
      events.push({ x: pos.x, y: pos.y, phase: "leave", line: L_LEAVE, note: "At leave point q<sub>L</sub> — resume straight-line motion toward q<sub>goal</sub>." });
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
    start: "#2b6cb0", to_goal: "#2b6cb0", hit: "#c23b3b", circumnav: "#b7532c",
    found: "#b7532c", return: "#8a5a2c", leave: "#2f8f5b", reached: "#2f8f5b", stuck: "#c23b3b",
  };

  function draw(ctx, world, events, idx) {
    world.draw(ctx);
    // m-line
    ctx.save();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = "#8892a0";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(world.start.x, world.start.y);
    ctx.lineTo(world.goal.x, world.goal.y);
    ctx.stroke();
    ctx.restore();

    // traced path, colored by phase
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
    const { events, success } = computeBug1(w);
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
          note += success
            ? ` Total path length ≈ ${L.toFixed(0)}px vs. straight-line d(q<sub>start</sub>,q<sub>goal</sub>) ≈ ${d0.toFixed(0)}px.`
            : "";
        }
        return { done, note, line: events[idx].line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.bug1 = {
    title: "Bug1",
    badge: "§3.2 / book §2.1",
    subtitle: "Exhaustive strategy: circle the whole obstacle once, remember the best exit point, then retrace to it.",
    width: 480, height: 320,
    legend: [
      { color: "#2b6cb0", label: "motion-to-goal" },
      { color: "#b7532c", label: "circumnavigate" },
      { color: "#8a5a2c", label: "retrace to q_L" },
      { color: "#2f8f5b", label: "leave / reached" },
      { color: "#c23b3b", label: "hit point / stuck" },
    ],
    pseudocode: [
      "move toward q_goal along the m-line",
      "if hit an obstacle boundary:",
      { text: "circumnavigate the ENTIRE boundary, tracking distance-to-goal", indent: 1 },
      { text: "return to q_L = boundary point closest to goal", indent: 1 },
      { text: "resume motion-to-goal from q_L", indent: 1 },
      "if q_L -> q_goal re-intersects the same obstacle:",
      { text: "report: no path exists", indent: 1 },
      "if q_goal reached: done",
    ],
    makeSim,
    pythonCode: `
def bug1(start, goal, is_free, step=0.05):
    """Bug1: circle the whole obstacle, then go to the closest point to goal."""
    pos, path = start, [start]
    visited_obstacles = set()

    while dist(pos, goal) > step:
        direction = normalize(goal - pos)
        nxt = pos + direction * step

        if is_free(nxt):
            pos = nxt
            path.append(pos)
            continue

        # --- hit point: circle the whole obstacle once ---
        q_hit = pos
        obstacle = nearest_obstacle(nxt)
        if obstacle in visited_obstacles:
            return None  # revisited without progress -> no path exists
        visited_obstacles.add(obstacle)

        boundary = trace_boundary(obstacle, q_hit, step)   # one full loop
        q_leave = min(boundary, key=lambda p: dist(p, goal))

        path += boundary                                   # full circumnavigation
        path += boundary[:boundary.index(q_leave) + 1]      # retrace to q_leave
        pos = q_leave

    path.append(goal)
    return path
`,
  };
})();
