/* Bug0 visualization: the naive strategy -- head for the goal, and the
   instant an obstacle is hit, follow its boundary only until stepping
   straight toward the goal is locally free again, then leave immediately.
   No memory of distance-to-goal (Bug1) and no fixed m-line invariant
   (Bug2) -- just "can I go straight now?", checked at every boundary
   point. That's exactly why it has no termination guarantee: it can leave
   too early and spiral, or hand off between two obstacles forever. */
(function () {
  "use strict";
  const { makeBugWorld, geom } = window.RMP;

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

  // pseudocode line indices (0-based) -- see vizDefs.bug0.pseudocode below
  const L_TO_GOAL = 0, L_HIT = 1, L_FOLLOW = 2, L_LEAVE = 3, L_NO_PATH = 5, L_REACHED = 6;

  function computeBug0(world) {
    const STEP = 4, EPS = 3;
    const events = [{ x: world.start.x, y: world.start.y, phase: "start", line: L_TO_GOAL, note: "Start at q<sub>start</sub>, heading straight for q<sub>goal</sub>." }];
    let pos = { x: world.start.x, y: world.start.y };
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
        events.push({ x: pos.x, y: pos.y, phase: "to_goal", line: L_TO_GOAL, note: "Motion-to-goal: moving straight toward q<sub>goal</sub>." });
        continue;
      }

      // hit point: follow the boundary with NO other bookkeeping -- leave
      // the instant a straight step toward the goal is locally free again.
      const qH = { x: pos.x, y: pos.y };
      events.push({ x: qH.x, y: qH.y, phase: "hit", line: L_HIT, note: "Hit point q<sub>H</sub> — obstacle detected. Follow the boundary until the direct line to goal is locally unobstructed." });
      const obs = world.nearestObstacle(next.x, next.y);
      if (!obs) { events.push({ x: qH.x, y: qH.y, phase: "stuck", line: L_NO_PATH, note: "No obstacle found (numerical edge case) — stopping." }); return { events, success: false }; }

      const boundary = geom.traceObstacleBoundary(obs, qH, STEP, EPS, Infinity);
      let left = false;
      for (let i = 1; i < boundary.length; i++) {
        const p = boundary[i];
        events.push({ x: p.x, y: p.y, phase: "follow", line: L_FOLLOW, note: "Boundary-following — checking, at every point, whether a step straight toward q<sub>goal</sub> is free." });
        const dG = dist(p, world.goal);
        if (dG < STEP) { pos = { x: p.x, y: p.y }; left = true; break; }
        const gdir = { x: (world.goal.x - p.x) / dG, y: (world.goal.y - p.y) / dG };
        const probe = { x: p.x + gdir.x * STEP, y: p.y + gdir.y * STEP };
        if (world.isFree(probe.x, probe.y)) {
          pos = { x: p.x, y: p.y };
          events.push({ x: pos.x, y: pos.y, phase: "leave", line: L_LEAVE, note: "Direct step toward q<sub>goal</sub> is free — leave immediately (no check on whether this is actually a good leave point)." });
          left = true;
          break;
        }
      }
      if (!left) {
        events.push({ x: pos.x, y: pos.y, phase: "stuck", line: L_NO_PATH, note: "Circled the whole obstacle without ever finding a locally-clear direction to the goal. In general Bug0 has no termination guarantee — it can spiral or hand off between obstacles forever; this demo stops after one full lap." });
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
    follow: "#b7532c", leave: "#2f8f5b", reached: "#2f8f5b", stuck: "#c23b3b",
  };

  function draw(ctx, world, events, idx) {
    world.draw(ctx);

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
    const { events, success } = computeBug0(w);
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
  window.RMP.vizDefs.bug0 = {
    title: "Bug0",
    badge: "§3.1",
    subtitle: "Naive strategy: head for the goal, and the moment an obstacle blocks the way, follow its boundary only until a straight step toward the goal is locally free again.",
    width: 480, height: 320,
    legend: [
      { color: "#2b6cb0", label: "motion-to-goal" },
      { color: "#b7532c", label: "boundary-following" },
      { color: "#2f8f5b", label: "leave / reached" },
      { color: "#c23b3b", label: "hit point / stuck" },
    ],
    pseudocode: [
      "move toward q_goal",
      "if hit an obstacle boundary at q_H:",
      { text: "follow the boundary until a direct step toward q_goal is locally free", indent: 1 },
      { text: "leave immediately -- no check on whether this is a good leave point", indent: 1 },
      "if the whole boundary is circled with no such direction:",
      { text: "no termination guarantee -- may loop forever (this demo stops after one lap)", indent: 1 },
      "if q_goal reached: done",
    ],
    makeSim,
    pythonCode: `
def bug0(start, goal, is_free, step=0.05):
    """Bug0: the naive strategy -- leave the boundary the INSTANT a direct
    step toward the goal is locally free again. No memory of distance-to-
    goal (Bug1) and no fixed m-line invariant (Bug2), so completeness is
    not guaranteed: it can leave too early and spiral, or hand off between
    two obstacles forever."""
    pos, path = start, [start]

    while dist(pos, goal) > step:
        direction = normalize(goal - pos)
        nxt = pos + direction * step

        if is_free(nxt):
            pos = nxt
            path.append(pos)
            continue

        # --- hit point: follow the boundary with no other bookkeeping ---
        q_hit = pos
        for p in trace_boundary(nearest_obstacle(nxt), q_hit, step):
            path.append(p)
            probe = p + normalize(goal - p) * step
            if is_free(probe):
                pos = p                      # leave immediately, first opening wins
                break
        else:
            return None                      # full lap, no opening -- may not
                                              # even be a real "no path" case,
                                              # just this run's bad luck
    path.append(goal)
    return path
`,
  };
})();
