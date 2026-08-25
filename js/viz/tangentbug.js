/* Tangent Bug visualization: finite-range radial sensor, heads toward the
   sensed boundary point that minimizes heuristic distance dist(robot,p)+dist(p,goal),
   and leaves boundary-following as soon as a better sensed "reach" point beats
   the best distance-to-goal seen so far while following ("d_reach < d_followed"). */
(function () {
  "use strict";
  const { makeBugWorld } = window.RMP;
  const { traceObstacleBoundary } = window.RMP.geom;

  const RANGE = 95, N_RAYS = 36, RAY_STEP = 3;

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
  function norm(a, b) { const d = dist(a, b) || 1; return { x: (b.x - a.x) / d, y: (b.y - a.y) / d }; }

  function lineOfSightClear(world, a, b) {
    const d = dist(a, b);
    if (d < 1e-6) return true;
    // Fine sampling matters here specifically: right after leaving a
    // boundary, the ray toward the goal can run nearly tangent to the very
    // obstacle just circled, grazing a thin sliver of it. A coarser step
    // (e.g. every 3px) can hop clean over that sliver and report "clear"
    // when the goal-ward path actually cuts back into the obstacle.
    const steps = Math.ceil(d / 1);
    for (let i = 1; i < steps; i++) {
      const t = i / steps;
      if (!world.isFree(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return false;
    }
    return true;
  }

  // A single STEP is short, but checking only its destination point (as a
  // contact-sensor Bug would) can still let it clip a thin obstacle sliver
  // in between -- same failure mode as lineOfSightClear above, just at
  // STEP scale instead of sensor-RANGE scale. Check the whole segment.
  function stepFree(world, a, b) {
    return world.isFree(b.x, b.y) && lineOfSightClear(world, a, b);
  }

  // One point per ray: the obstacle it hits, or the point at full sensor
  // range if that direction is clear. Range-limited points matter just as
  // much as hit points -- they're what let the robot "see past" an
  // isolated obstacle into open space toward the goal.
  function sensePoints(world, pos) {
    const pts = [];
    for (let k = 0; k < N_RAYS; k++) {
      const ang = (k / N_RAYS) * Math.PI * 2;
      const dx = Math.cos(ang), dy = Math.sin(ang);
      let hitR = null;
      for (let r = RAY_STEP; r <= RANGE; r += RAY_STEP) {
        const x = pos.x + dx * r, y = pos.y + dy * r;
        if (!world.isFree(x, y)) { hitR = r - RAY_STEP * 0.4; break; }
      }
      const r = hitR === null ? RANGE : hitR;
      pts.push({ x: pos.x + dx * r, y: pos.y + dy * r, blocked: hitR !== null });
    }
    return pts;
  }

  function bestHeuristicPoint(pts, from, goal) {
    let best = null, bestH = Infinity;
    for (const p of pts) {
      const h = dist(from, p) + dist(p, goal);
      if (h < bestH) { bestH = h; best = p; }
    }
    return { best, bestH };
  }

  // pseudocode line indices (0-based) -- see vizDefs.tangentbug.pseudocode below
  const L_SENSE = 0, L_MOTION = 2, L_LOCALMIN = 4, L_FOLLOW = 6, L_LEAVE = 8, L_REACHED = 9;

  function computeTangentBug(world) {
    const STEP = 4, EPS = 3, H_EPS = 1.5, MIN_FOLLOW_STEPS = 3;
    const events = [{ x: world.start.x, y: world.start.y, phase: "start", rays: [], line: L_SENSE, note: "Start at q<sub>start</sub>, equipped with a finite-range radial sensor (dashed circle)." }];
    let pos = { x: world.start.x, y: world.start.y };
    let guard = 0;
    // h(p) achieved by the previous tangential step -- reset to Infinity every
    // time we leave tangential motion (straight-line motion-to-goal, or just
    // left boundary-following), so each new episode gets a fresh baseline.
    let prevBestH = Infinity;
    let lastPhase = "start";

    while (guard++ < 5000) {
      const distG = dist(pos, world.goal);
      if (distG < STEP * 1.2) {
        events.push({ x: world.goal.x, y: world.goal.y, phase: "reached", rays: [], line: L_REACHED, note: "Reached q<sub>goal</sub> — path complete." });
        return { events, success: true };
      }
      const losEnd = distG <= RANGE ? world.goal
        : { x: pos.x + (world.goal.x - pos.x) / distG * RANGE, y: pos.y + (world.goal.y - pos.y) / distG * RANGE };
      const pts = sensePoints(world, pos);

      if (lineOfSightClear(world, pos, losEnd)) {
        const dir = norm(pos, world.goal);
        const next = { x: pos.x + dir.x * STEP, y: pos.y + dir.y * STEP };
        if (stepFree(world, pos, next)) {
          pos = next;
          prevBestH = Infinity;
          lastPhase = "to_goal";
          events.push({ x: pos.x, y: pos.y, phase: "to_goal", rays: pts, line: L_MOTION, note: "Goal side clear within sensor range — heading straight for q<sub>goal</sub>." });
          continue;
        }
      }

      // blocked within range: steer toward whichever sensed point minimizes
      // h(p) = dist(robot,p) + dist(p,goal), re-sensing at every step, for as
      // long as h keeps decreasing -- exactly like the pseudocode below.
      if (lastPhase !== "tangent") prevBestH = Infinity;
      const { best, bestH } = bestHeuristicPoint(pts, pos, world.goal);
      if (best && bestH < prevBestH - H_EPS) {
        const dir = norm(pos, best);
        const next = { x: pos.x + dir.x * STEP, y: pos.y + dir.y * STEP };
        if (stepFree(world, pos, next)) {
          pos = next;
          prevBestH = bestH;
          lastPhase = "tangent";
          events.push({ x: pos.x, y: pos.y, phase: "tangent", rays: pts, line: L_MOTION, note: `Heading toward the sensed point minimizing h(p) = dist(robot,p)+dist(p,goal) &mdash; h is still decreasing (h&asymp;${bestH.toFixed(0)}px).` });
          continue;
        }
        // best point is physically unreachable this step (right at the
        // boundary) -- can't make further sensor-driven progress either.
      }

      // h(p) stopped decreasing (or no further progress is physically
      // possible): this is the local minimum -- begin boundary-following,
      // continuously re-sensing.
      const qH = { x: pos.x, y: pos.y };
      const obs = world.nearestObstacle(pos.x, pos.y);
      lastPhase = "hit";
      events.push({ x: qH.x, y: qH.y, phase: "hit", rays: pts, line: L_LOCALMIN, note: "h(p) stopped decreasing &mdash; local minimum reached. Begin boundary-following, continuously re-sensing." });
      if (!obs) { events.push({ x: qH.x, y: qH.y, phase: "stuck", rays: [], note: "No obstacle found (numerical edge case) — stopping." }); return { events, success: false }; }

      const boundary = traceObstacleBoundary(obs, qH, STEP, EPS, Infinity);
      let dFollowedMin = dist(qH, world.goal);
      let left = false;
      for (let i = 1; i < boundary.length; i++) {
        const p = boundary[i];
        const pts2 = sensePoints(world, p);
        events.push({ x: p.x, y: p.y, phase: "follow", rays: pts2, line: L_FOLLOW, note: "Following the boundary; comparing d<sub>reach</sub> (best sensed shortcut) to d<sub>followed</sub> (closest point seen so far)." });
        // d_reach must reflect genuinely NEW information -- open space, or a
        // different obstacle -- not just "the same wall I'm already circling,
        // sensed a few px ahead of me". Blocked points on the obstacle
        // currently being followed are exactly what the boundary trace
        // itself will visit next; counting them (with a travel-cost term
        // that's ~0 since they're right next to p) made every step look
        // like a "shortcut" purely from ordinary forward progress, so the
        // demo left almost immediately, indistinguishable from Bug0. Only
        // points that reveal something beyond this obstacle -- clear range,
        // or a DIFFERENT obstacle's surface -- count as a real shortcut.
        let dReach = Infinity;
        for (const q of pts2) {
          if (q.blocked && world.nearestObstacle(q.x, q.y) === obs) continue;
          dReach = Math.min(dReach, dist(q, world.goal));
        }
        if (dist(p, world.goal) <= RANGE && lineOfSightClear(world, p, world.goal)) dReach = Math.min(dReach, dist(p, world.goal));
        // Require a few real steps of boundary-following before trusting a
        // "shortcut" -- right next to the hit point, a momentary gap in the
        // sensor readings (e.g. grazing a sharp corner) can look like an
        // opening that isn't really walkable, causing the robot to leave,
        // immediately fail to make progress, and re-hit at nearly the same
        // spot -- an infinite micro-loop. A short cooldown gives the
        // geometry a chance to settle before the leave condition can fire.
        if (i >= MIN_FOLLOW_STEPS && dReach < dFollowedMin - 2) {
          pos = { x: p.x, y: p.y };
          lastPhase = "leave";
          events.push({ x: pos.x, y: pos.y, phase: "leave", rays: pts2, line: L_LEAVE, note: "d<sub>reach</sub> &lt; d<sub>followed</sub> — a shortcut is now visible. Leave the boundary." });
          left = true;
          break;
        }
        // update AFTER comparing, so d_followed reflects prior history, not
        // the current point itself (else direct line-of-sight can only ever
        // tie d_followed, never beat it, and the leave condition never fires)
        dFollowedMin = Math.min(dFollowedMin, dist(p, world.goal));
      }
      if (!left) {
        events.push({ x: pos.x, y: pos.y, phase: "stuck", rays: [], note: "Circled the whole obstacle without ever sensing a shortcut — no path exists." });
        return { events, success: false };
      }
    }
    events.push({ x: pos.x, y: pos.y, phase: "stuck", rays: [], note: "Iteration limit reached." });
    return { events, success: false };
  }

  function pathLength(events) {
    let L = 0;
    for (let i = 1; i < events.length; i++) L += dist(events[i - 1], events[i]);
    return L;
  }

  const PHASE_COLOR = {
    start: "#2b6cb0", to_goal: "#2b6cb0", tangent: "#6a4fb0", hit: "#c23b3b",
    follow: "#b7532c", leave: "#2f8f5b", reached: "#2f8f5b", stuck: "#c23b3b",
  };

  function draw(ctx, world, events, idx) {
    world.draw(ctx);
    const cur = events[idx];

    // sensor range + rays
    ctx.save();
    ctx.strokeStyle = "rgba(111,168,220,0.55)";
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.arc(cur.x, cur.y, RANGE, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.strokeStyle = "rgba(111,168,220,0.4)";
    (cur.rays || []).filter((p) => p.blocked).forEach((p) => {
      ctx.beginPath();
      ctx.moveTo(cur.x, cur.y);
      ctx.lineTo(p.x, p.y);
      ctx.stroke();
    });
    ctx.restore();

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

    ctx.save();
    ctx.fillStyle = PHASE_COLOR[cur.phase] || "#2b6cb0";
    ctx.beginPath();
    ctx.arc(cur.x, cur.y, 5.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  function makeSim({ rng, width, height, world }) {
    const w = world || makeBugWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const { events, success } = computeTangentBug(w);
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
          note += success ? ` Total path length ≈ ${L.toFixed(0)}px vs. straight-line ≈ ${d0.toFixed(0)}px.` : "";
        }
        return { done, note, line: events[idx].line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.tangentbug = {
    title: "Tangent Bug",
    badge: "§3.4 / book §2.2",
    subtitle: "A finite-range radial sensor lets the robot shortcut around obstacles instead of fully committing to boundary-following.",
    width: 480, height: 320,
    legend: [
      { color: "#2b6cb0", label: "motion-to-goal" },
      { color: "#6a4fb0", label: "heading to sensed point" },
      { color: "#b7532c", label: "boundary-following" },
      { color: "#2f8f5b", label: "leave / reached" },
      { color: "#c23b3b", label: "hit point / stuck" },
    ],
    pseudocode: [
      "sense boundary/discontinuity points within range R",
      "h(p) = dist(robot, p) + dist(p, q_goal)",
      "motion-to-goal: move toward the sensed point minimizing h(p)",
      "if h(p) stops decreasing (local minimum reached):",
      { text: "switch to boundary-following", indent: 1 },
      "boundary-following: track d_followed (best dist-to-goal seen on this obstacle)",
      { text: "and d_reach (best dist-to-goal currently visible)", indent: 1 },
      "if d_reach < d_followed:",
      { text: "leave boundary, resume motion-to-goal", indent: 1 },
      "if q_goal reached: done",
    ],
    makeSim,
    pythonCode: `
def tangent_bug(start, goal, sense, is_free, range_=95, step=0.05, h_eps=1.5, min_follow_steps=3):
    """Tangent Bug: finite-range radial sensor picks the sensed point that
    minimizes heuristic h(p) = dist(robot, p) + dist(p, goal)."""
    pos, path = start, [start]

    def line_of_sight_clear(a, b):
        return not any(not is_free(pt) for pt in sample_segment(a, b))

    prev_best_h = float("inf")   # reset whenever a new tangential episode starts

    while dist(pos, goal) > step:
        los_end = goal if dist(pos, goal) <= range_ else pos + normalize(goal - pos) * range_

        if line_of_sight_clear(pos, los_end):
            nxt = pos + normalize(goal - pos) * step
            if is_free(nxt):
                pos = nxt
                path.append(pos)
                prev_best_h = float("inf")                # fresh episode next time we're blocked
                continue

        # --- blocked: keep heading toward whichever sensed point minimizes
        # h(p), re-sensing every step, for as long as h keeps decreasing ---
        sensed = sense(pos, range_)                       # radial scan
        target = min(sensed, key=lambda p: dist(pos, p) + dist(p, goal))
        best_h = dist(pos, target) + dist(target, goal)
        if best_h < prev_best_h - h_eps:
            nxt = pos + normalize(target - pos) * step
            if is_free(nxt):
                pos = nxt
                path.append(pos)
                prev_best_h = best_h
                continue

        # --- h(p) stopped decreasing: local minimum -> boundary-following,
        # still re-sensing at every point for a shortcut ---
        q_hit = pos
        obstacle = nearest_obstacle(pos)
        d_followed = dist(q_hit, goal)
        for i, p in enumerate(trace_boundary(obstacle, q_hit, step)):
            path.append(p)
            d_followed = min(d_followed, dist(p, goal))
            sensed = sense(p, range_)
            # only points that reveal something BEYOND this obstacle count as
            # a real shortcut -- blocked points still on the obstacle being
            # followed are just the wall itself, sensed a step ahead
            reach_candidates = [q for q in sensed if not (q.blocked and nearest_obstacle(q) is obstacle)]
            d_reach = min((dist(q, goal) for q in reach_candidates), default=float("inf"))
            if line_of_sight_clear(p, goal) and dist(p, goal) <= range_:
                d_reach = min(d_reach, dist(p, goal))
            # require a few real steps before trusting a "shortcut" -- right
            # at the hit point a momentary gap can look walkable but isn't,
            # which otherwise causes leave -> immediate re-hit forever
            if i >= min_follow_steps and d_reach < d_followed:
                pos = p                                    # shortcut found
                prev_best_h = float("inf")                  # fresh episode next time we're blocked
                break
        else:
            return None                                    # no shortcut, no path

    path.append(goal)
    return path
`,
  };
})();
