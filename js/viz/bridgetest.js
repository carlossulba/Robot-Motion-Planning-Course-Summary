/* Bridge test: sample q1; if it happens to be free, discard it immediately
   (the test only fires from a colliding seed). Sample q2 at a Gaussian
   offset from q1; if q2 is also in collision, test the segment's midpoint --
   keep it as a milestone only if that midpoint is collision-free. A midpoint
   that survives this test literally "bridges" the thin gap between the two
   obstacle regions q1 and q2 landed in, which is exactly what a narrow
   passage looks like from the inside. */
(function () {
  "use strict";
  const { makeNarrowPassageWorld } = window.RMP;

  function gauss(rng) {
    let u = 0, v = 0;
    while (u === 0) u = rng();
    while (v === 0) v = rng();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  // Sample q1 from a band straddling the wall, close to the gap's height,
  // rather than the whole canvas. A real bridge test samples anywhere in
  // C-space, but on a small demo canvas the wall obstacles (and the handful
  // of rows near the one gap that could ever produce a successful bridge)
  // occupy a tiny fraction of the total area -- scoping the seed region to
  // "near the passage" (still uniform within that band) keeps runs short
  // enough to actually show the occasional accepted bridge the test is about,
  // instead of hundreds of attempts with no visible payoff.
  function nearWall(world, rng) {
    const xBand = 20, yBand = 46;
    const xMin = Math.max(world.margin * 0.4, world.wallX - xBand);
    const xMax = Math.min(world.width - world.margin * 0.4, world.wallX + xBand);
    const yMin = Math.max(world.margin * 0.4, world.gapCenterY - yBand);
    const yMax = Math.min(world.height - world.margin * 0.4, world.gapCenterY + yBand);
    return {
      x: xMin + rng() * (xMax - xMin),
      y: yMin + rng() * (yMax - yMin),
    };
  }

  function computeBridge(world, rng, attempts, std) {
    const events = [];
    const milestones = [];
    for (let a = 0; a < attempts; a++) {
      const q1 = nearWall(world, rng);
      const c1 = world.isFree(q1.x, q1.y);
      if (c1) {
        events.push({ stage: "q1-free", q1, accepted: false, milestoneCount: milestones.length });
        continue;
      }
      const q2 = { x: q1.x + gauss(rng) * std, y: q1.y + gauss(rng) * std };
      const c2 = world.isFree(q2.x, q2.y);
      if (c2) {
        events.push({ stage: "q2-free", q1, q2, accepted: false, milestoneCount: milestones.length });
        continue;
      }
      const mid = { x: (q1.x + q2.x) / 2, y: (q1.y + q2.y) / 2 };
      const midFree = world.isFree(mid.x, mid.y);
      if (midFree) milestones.push(mid);
      events.push({ stage: "midpoint", q1, q2, mid, accepted: midFree, milestoneCount: milestones.length });
    }
    return { milestones, events };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });
    if (idx < 0) { world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start"); world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal"); return; }
    const ev = data.events[idx];

    ctx.save();
    ctx.fillStyle = "#6a4fb0";
    for (let k = 0; k < ev.milestoneCount; k++) {
      const p = data.milestones[k];
      ctx.beginPath(); ctx.arc(p.x, p.y, 3, 0, Math.PI * 2); ctx.fill();
    }
    ctx.restore();

    if (ev.stage === "q1-free") {
      ctx.save();
      ctx.fillStyle = "#2b6cb0";
      ctx.beginPath(); ctx.arc(ev.q1.x, ev.q1.y, 4, 0, Math.PI * 2); ctx.fill();
      ctx.restore();
    } else {
      ctx.save();
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = "#c23b3b";
      ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.moveTo(ev.q1.x, ev.q1.y); ctx.lineTo(ev.q2.x, ev.q2.y); ctx.stroke();
      ctx.restore();
      [ev.q1, ev.q2].forEach((p) => {
        ctx.save(); ctx.fillStyle = "#c23b3b";
        ctx.beginPath(); ctx.arc(p.x, p.y, 4, 0, Math.PI * 2); ctx.fill();
        ctx.restore();
      });
      if (ev.stage === "midpoint") {
        ctx.save();
        ctx.fillStyle = ev.accepted ? "#2f8f5b" : "#8892a0";
        ctx.beginPath(); ctx.arc(ev.mid.x, ev.mid.y, ev.accepted ? 5 : 3.5, 0, Math.PI * 2); ctx.fill();
        if (ev.accepted) { ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.2; ctx.stroke(); }
        ctx.restore();
      }
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height }) {
    const world = makeNarrowPassageWorld(rng, { width, height, wallThickness: 30 });
    const ATTEMPTS = 260, STD = 18;
    const data = computeBridge(world, rng, ATTEMPTS, STD);
    let idx = -1;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, data.events.length - 1);
        const done = idx >= data.events.length - 1;
        const ev = data.events[idx];
        let note;
        if (ev.stage === "q1-free") note = "q1 landed in free space — the bridge test only starts from a colliding seed, discarded.";
        else if (ev.stage === "q2-free") note = "q1 collided but q2 (Gaussian offset) landed free — need both colliding, discarded.";
        else note = ev.accepted
          ? `Both q1 and q2 collide, but their midpoint is free — accepted as bridge milestone ${ev.milestoneCount}.`
          : "Both q1 and q2 collide, but the midpoint is still inside an obstacle — no bridge here, discarded.";
        return { done, note: note + (done ? ` ${ev.milestoneCount} bridge milestones kept out of ${data.events.length} attempts.` : "") };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.bridgetest = {
    title: "Bridge Test",
    badge: "narrow-passage sampling",
    subtitle: "Sample two colliding configurations; keep their midpoint as a milestone only if the midpoint itself is collision-free.",
    width: 560, height: 360,
    legend: [
      { color: "#6a4fb0", label: "milestone" },
      { color: "#c23b3b", label: "colliding sample" },
      { color: "#2f8f5b", label: "accepted bridge midpoint" },
      { color: "#8892a0", label: "rejected midpoint" },
    ],
    makeSim,
    pythonCode: `
def bridge_test(is_free, sample_anywhere, std=26):
    q1 = sample_anywhere()
    if is_free(q1):
        return None                         # only start from a colliding seed
    q2 = q1 + gaussian_offset(std)
    if is_free(q2):
        return None                         # need both endpoints in collision
    mid = (q1 + q2) / 2
    return mid if is_free(mid) else None    # midpoint bridges the gap, or doesn't
`,
  };
})();
