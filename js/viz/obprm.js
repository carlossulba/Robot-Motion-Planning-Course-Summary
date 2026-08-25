/* OBPRM (Obstacle-Based PRM) visualization: instead of PRM's uniform
   rejection sampling, every milestone is grown FROM a colliding seed --
   step along a random direction until free, then binary-search the segment
   between the colliding and free points to converge on the boundary. That
   bias toward obstacle surfaces is exactly where narrow passages live.

   Once the milestones are built this way, they're connected into a roadmap
   and queried exactly like plain PRM (see prm.js) -- OBPRM only changes how
   milestones are sampled, not what's done with them afterward. */
(function () {
  "use strict";
  const { makeWorld } = window.RMP;

  const SEED_COLOR = "#c23b3b";
  const FREE_COLOR = "#2b6cb0";
  const CANDIDATE = "#e2b06a";
  const MILESTONE_COLOR = "#7a4fb0";
  const REJECT = "#c23b3b";

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
  function segmentFree(world, a, b, step) {
    step = step || 4;
    const d = dist(a, b), n = Math.max(1, Math.ceil(d / step));
    for (let i = 1; i < n; i++) {
      const t = i / n;
      if (!world.isFree(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return false;
    }
    return true;
  }
  function randPt(world, rng) {
    return { x: world.margin * 0.3 + rng() * (world.width - world.margin * 0.6), y: world.margin * 0.3 + rng() * (world.height - world.margin * 0.6) };
  }

  // pseudocode line indices (0-based) -- see vizDefs.obprm.pseudocode below
  const L_SEED = 0, L_DIRECTION = 1, L_BISECT = 2, L_MILESTONE = 3;

  // Build one milestone: find a colliding seed, walk a random direction to a
  // free point, binary-search the boundary between them. Pushes every
  // intermediate attempt as an event so the process is fully visible.
  function buildOneMilestone(world, rng, events) {
    const RAY_STEP = 10, MAX_RAY_STEPS = 30, BISECT_TOL = 1.5, MAX_BISECT_ITERS = 10;

    let seed = null;
    for (let tries = 0; tries < 300; tries++) {
      const p = randPt(world, rng);
      if (!world.isFree(p.x, p.y)) { seed = p; break; }
    }
    if (!seed) return null; // pathological world (almost no obstacle area) -- skip this attempt
    events.push({ type: "seed", pt: seed, line: L_SEED });

    const angle = rng() * Math.PI * 2;
    const dir = { x: Math.cos(angle), y: Math.sin(angle) };
    let collidingPt = seed, freePt = null;
    for (let s = 1; s <= MAX_RAY_STEPS; s++) {
      const p = { x: seed.x + dir.x * RAY_STEP * s, y: seed.y + dir.y * RAY_STEP * s };
      if (world.inBounds(p.x, p.y) && world.isFree(p.x, p.y)) {
        freePt = p;
        events.push({ type: "ray-free", pt: p, from: collidingPt, line: L_DIRECTION });
        break;
      }
      if (world.inBounds(p.x, p.y)) {
        collidingPt = p;
        events.push({ type: "ray-step", pt: p, line: L_DIRECTION });
      } else {
        break; // walked out of the canvas without finding free space
      }
    }
    if (!freePt) return null; // never found a free point along this ray -- skip this attempt

    // Binary search until the bracket is within BISECT_TOL of the boundary --
    // "close enough" is fine, we don't need to nail the boundary exactly.
    // hi is always free by construction, so stopping early still guarantees
    // the returned milestone is sampled outside the obstacle.
    let lo = collidingPt, hi = freePt; // lo: colliding, hi: free
    let it = 0;
    while (it < MAX_BISECT_ITERS && dist(lo, hi) > BISECT_TOL) {
      const mid = { x: (lo.x + hi.x) / 2, y: (lo.y + hi.y) / 2 };
      events.push({ type: "bisect-candidate", pt: mid, line: L_BISECT });
      if (world.isFree(mid.x, mid.y)) {
        hi = mid;
        events.push({ type: "bisect-free", pt: mid, line: L_BISECT });
      } else {
        lo = mid;
        events.push({ type: "bisect-colliding", pt: mid, line: L_BISECT });
      }
      it++;
    }
    events.push({ type: "milestone", pt: hi, line: L_MILESTONE });
    return hi;
  }

  function computeOBPRM(world, rng, N, k) {
    const events = [];
    const milestones = [];
    let guard = 0;
    while (milestones.length < N && guard < N * 4) {
      guard++;
      const m = buildOneMilestone(world, rng, events);
      if (m) milestones.push(m);
    }
    events.push({ type: "dense" });

    const nodes = [{ ...world.start }, { ...world.goal }, ...milestones];
    const edgeSet = new Set();
    const edges = [];
    const adj = nodes.map(() => []);
    nodes.forEach((n, i) => {
      const others = nodes.map((m, j) => [j, dist(n, m)]).filter(([j]) => j !== i).sort((a, b) => a[1] - b[1]).slice(0, k);
      others.forEach(([j, w]) => {
        const key = i < j ? `${i}-${j}` : `${j}-${i}`;
        if (edgeSet.has(key)) return;
        edgeSet.add(key);
        events.push({ type: "edge-candidate", a: i, b: j });
        if (segmentFree(world, n, nodes[j])) {
          edges.push([i, j, w]);
          adj[i].push([j, w]); adj[j].push([i, w]);
          events.push({ type: "edge-accept", a: i, b: j });
        } else {
          events.push({ type: "edge-reject", a: i, b: j });
        }
      });
    });

    const dd = new Array(nodes.length).fill(Infinity), prev = new Array(nodes.length).fill(-1), vis = new Array(nodes.length).fill(false);
    dd[0] = 0;
    for (let it = 0; it < nodes.length; it++) {
      let u = -1, best = Infinity;
      for (let i = 0; i < nodes.length; i++) if (!vis[i] && dd[i] < best) { best = dd[i]; u = i; }
      if (u === -1) break;
      vis[u] = true;
      for (const [v, w] of adj[u]) if (dd[u] + w < dd[v]) { dd[v] = dd[u] + w; prev[v] = u; }
    }
    let path = null;
    if (dd[1] < Infinity) { path = [1]; let c = 1; while (c !== 0) { c = prev[c]; path.push(c); } path.reverse(); }
    events.push({ type: "search" });

    const milestoneCountAt = new Array(events.length);
    const edgeCountAt = new Array(events.length);
    let mc = 0, ec = 0;
    events.forEach((e, i) => {
      if (e.type === "milestone") mc++;
      if (e.type === "edge-accept") ec++;
      milestoneCountAt[i] = mc; edgeCountAt[i] = ec;
    });

    return { nodes, milestones, edges, path, pathLen: dd[1], events, milestoneCountAt, edgeCountAt, N };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });
    const mRev = idx >= 0 ? data.milestoneCountAt[idx] : 0;
    const eRev = idx >= 0 ? data.edgeCountAt[idx] : 0;

    ctx.save();
    ctx.strokeStyle = "rgba(111,168,220,0.5)";
    ctx.lineWidth = 1;
    for (let k = 0; k < eRev; k++) {
      const [i, j] = data.edges[k];
      ctx.beginPath(); ctx.moveTo(data.nodes[i].x, data.nodes[i].y); ctx.lineTo(data.nodes[j].x, data.nodes[j].y); ctx.stroke();
    }
    ctx.restore();

    ctx.fillStyle = MILESTONE_COLOR;
    for (let k = 0; k < mRev; k++) {
      const p = data.milestones[k];
      ctx.beginPath(); ctx.arc(p.x, p.y, 3.2, 0, Math.PI * 2); ctx.fill();
    }

    const cur = idx >= 0 ? data.events[idx] : null;
    if (cur) {
      if (cur.type === "seed") {
        ctx.save();
        ctx.fillStyle = SEED_COLOR;
        ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 4, 0, Math.PI * 2); ctx.fill();
        ctx.restore();
      } else if (cur.type === "ray-step") {
        ctx.save();
        ctx.fillStyle = SEED_COLOR; ctx.globalAlpha = 0.75;
        ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 3, 0, Math.PI * 2); ctx.fill();
        ctx.restore();
      } else if (cur.type === "ray-free") {
        ctx.save();
        ctx.setLineDash([2, 2]); ctx.strokeStyle = FREE_COLOR; ctx.lineWidth = 1.2;
        ctx.beginPath(); ctx.moveTo(cur.from.x, cur.from.y); ctx.lineTo(cur.pt.x, cur.pt.y); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = FREE_COLOR;
        ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 4, 0, Math.PI * 2); ctx.fill();
        ctx.restore();
      } else if (cur.type === "bisect-candidate") {
        ctx.save();
        ctx.strokeStyle = CANDIDATE; ctx.lineWidth = 1.6;
        ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 4.5, 0, Math.PI * 2); ctx.stroke();
        ctx.restore();
      } else if (cur.type === "bisect-colliding") {
        ctx.save();
        ctx.fillStyle = SEED_COLOR; ctx.globalAlpha = 0.85;
        ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 3, 0, Math.PI * 2); ctx.fill();
        ctx.restore();
      } else if (cur.type === "bisect-free") {
        ctx.save();
        ctx.fillStyle = FREE_COLOR; ctx.globalAlpha = 0.85;
        ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 3, 0, Math.PI * 2); ctx.fill();
        ctx.restore();
      } else if (cur.type === "milestone") {
        ctx.save();
        ctx.fillStyle = MILESTONE_COLOR;
        ctx.beginPath(); ctx.arc(cur.pt.x, cur.pt.y, 5, 0, Math.PI * 2); ctx.fill();
        ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.4; ctx.stroke();
        ctx.restore();
      } else if (cur.type === "edge-candidate") {
        const a = data.nodes[cur.a], b = data.nodes[cur.b];
        ctx.save();
        ctx.setLineDash([3, 3]); ctx.strokeStyle = CANDIDATE; ctx.lineWidth = 1.6;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        ctx.restore();
      } else if (cur.type === "edge-reject") {
        const a = data.nodes[cur.a], b = data.nodes[cur.b];
        ctx.save();
        ctx.setLineDash([3, 3]); ctx.strokeStyle = REJECT; ctx.lineWidth = 1.8;
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
        ctx.restore();
      }
    }

    if (idx >= 0 && data.events[idx].type === "search" && data.path) {
      ctx.save();
      ctx.strokeStyle = "#2f8f5b"; ctx.lineWidth = 3;
      ctx.beginPath();
      data.path.forEach((pIdx, k) => { const p = data.nodes[pIdx]; k === 0 ? ctx.moveTo(p.x, p.y) : ctx.lineTo(p.x, p.y); });
      ctx.stroke();
      ctx.restore();
    }
    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height, world: sharedWorld }) {
    const world = sharedWorld || makeWorld(rng, { width, height, nObstacles: 3 + Math.floor(rng() * 3) });
    const N = 30, k = 6;
    const data = computeOBPRM(world, rng, N, k);
    let idx = -1;
    const total = data.events.length;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, total - 1);
        const done = idx >= total - 1;
        const e = data.events[idx];
        let note;
        if (e.type === "seed") note = "Found a colliding seed configuration (inside a C-obstacle).";
        else if (e.type === "ray-step") note = "Stepping along a random direction from the seed — still in collision.";
        else if (e.type === "ray-free") note = "Reached a free configuration — now binary-search the segment for the boundary.";
        else if (e.type === "bisect-candidate") note = "Binary search: testing the midpoint of the colliding/free bracket.";
        else if (e.type === "bisect-colliding") note = "Midpoint is in collision — it becomes the new colliding endpoint (bracket narrows).";
        else if (e.type === "bisect-free") note = "Midpoint is free — it becomes the new free endpoint (bracket narrows).";
        else if (e.type === "milestone") note = `Converged on the obstacle boundary — milestone ${data.milestoneCountAt[idx]} of up to ${data.N} accepted.`;
        else if (e.type === "dense") note = `${data.milestones.length} boundary-biased milestones built. Connecting each to its ${k} nearest neighbors next (same as plain PRM).`;
        else if (e.type === "edge-candidate") note = "Attempting to connect a candidate pair — testing straight-line collision (local planner).";
        else if (e.type === "edge-accept") note = "Collision-free — edge added to the roadmap.";
        else if (e.type === "edge-reject") note = "Blocked by an obstacle — edge discarded.";
        else note = data.path ? `Roadmap complete. Shortest path over the graph ≈ ${data.pathLen.toFixed(0)}px through ${data.path.length} nodes.` : "Roadmap complete, but q_start and q_goal ended up in different components — try Generate new.";
        return { done, note, line: e.line };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.obprm = {
    title: "OBPRM (Obstacle-Based PRM)",
    badge: "§8.3.2 / book §7.1",
    subtitle: "Bias milestones toward obstacle surfaces: find a colliding seed, walk to a free point, binary-search the boundary between them.",
    width: 560, height: 360,
    legend: [
      { color: MILESTONE_COLOR, label: "milestone (on boundary)" },
      { color: "rgba(111,168,220,0.8)", label: "roadmap edge" },
      { color: SEED_COLOR, label: "colliding seed / ray step" },
      { color: FREE_COLOR, label: "free endpoint" },
      { color: CANDIDATE, label: "bisection candidate" },
      { color: "#2f8f5b", label: "shortest path" },
    ],
    pseudocode: [
      "find a colliding configuration (inside a C-obstacle)",
      "pick a random direction; step along it until reaching a free configuration",
      "binary-search the segment between the colliding and free points until close enough to the boundary (free endpoint stays free)",
      "the boundary point becomes the new milestone",
    ],
    makeSim,
    pythonCode: `
def obprm_milestone(is_free, sample_anywhere, ray_step=10, bisect_tol=1.5, max_iters=10):
    seed = sample_anywhere()
    while is_free(seed):                        # rejection-sample a COLLIDING seed
        seed = sample_anywhere()

    direction = random_unit_vector()
    colliding, free = seed, None
    p = seed
    while free is None:
        p = p + direction * ray_step
        if is_free(p):
            free = p
        else:
            colliding = p                        # keep marching until free space is found

    lo, hi = colliding, free                     # binary search converges on the boundary
    it = 0
    while it < max_iters and dist(lo, hi) > bisect_tol:  # "close enough" is fine
        mid = (lo + hi) / 2
        if is_free(mid):
            hi = mid
        else:
            lo = mid
        it += 1
    return hi                                     # boundary-adjacent free milestone, still guaranteed free

# Roadmap connection and query proceed exactly like plain PRM (see prm.js) --
# OBPRM only changes how each milestone is generated.
`,
  };
})();
