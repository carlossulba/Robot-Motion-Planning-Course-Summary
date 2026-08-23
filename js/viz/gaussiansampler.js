/* Gaussian sampler: sample q1 uniformly, sample q2 at a Gaussian-distributed
   offset from q1, and keep the pair's free point as a milestone only if
   exactly one of q1, q2 is in collision. That rejection rule biases accepted
   milestones toward obstacle boundaries -- including the two boundaries that
   frame a narrow passage -- instead of spreading them uniformly like plain
   PRM sampling. */
(function () {
  "use strict";
  const { makeNarrowPassageWorld } = window.RMP;

  function gauss(rng) {
    let u = 0, v = 0;
    while (u === 0) u = rng();
    while (v === 0) v = rng();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  function anywhere(world, rng) {
    return {
      x: world.margin * 0.4 + rng() * (world.width - world.margin * 0.8),
      y: world.margin * 0.4 + rng() * (world.height - world.margin * 0.8),
    };
  }

  function computeGaussian(world, rng, attempts, std) {
    const events = [];
    const milestones = [];
    for (let a = 0; a < attempts; a++) {
      const q1 = anywhere(world, rng);
      const q2 = { x: q1.x + gauss(rng) * std, y: q1.y + gauss(rng) * std };
      const c1 = world.isFree(q1.x, q1.y);
      const c2 = world.isFree(q2.x, q2.y);
      const accepted = c1 !== c2;
      if (accepted) milestones.push(c1 ? q1 : q2);
      events.push({ q1, q2, c1, c2, accepted, milestoneCount: milestones.length });
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

    ctx.save();
    ctx.setLineDash(ev.accepted ? [] : [3, 3]);
    ctx.strokeStyle = ev.accepted ? "#2f8f5b" : "#8892a0";
    ctx.lineWidth = ev.accepted ? 2 : 1.2;
    ctx.beginPath(); ctx.moveTo(ev.q1.x, ev.q1.y); ctx.lineTo(ev.q2.x, ev.q2.y); ctx.stroke();
    ctx.restore();

    [[ev.q1, ev.c1], [ev.q2, ev.c2]].forEach(([p, free]) => {
      ctx.save();
      ctx.fillStyle = free ? "#2b6cb0" : "#c23b3b";
      ctx.beginPath(); ctx.arc(p.x, p.y, 4, 0, Math.PI * 2); ctx.fill();
      ctx.restore();
    });

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height, world: sharedWorld }) {
    const world = sharedWorld || makeNarrowPassageWorld(rng, { width, height });
    const ATTEMPTS = 140, STD = 22;
    const data = computeGaussian(world, rng, ATTEMPTS, STD);
    let idx = -1;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, data.events.length - 1);
        const done = idx >= data.events.length - 1;
        const ev = data.events[idx];
        const note = ev.accepted
          ? `Accepted: exactly one of q1/q2 was in collision — kept the free one as milestone ${ev.milestoneCount} (near an obstacle boundary).`
          : `Rejected: q1 and q2 were ${ev.c1 === ev.c2 && ev.c1 ? "both free" : "both in collision"} — no boundary information gained.`;
        return { done, note: note + (done ? ` ${ev.milestoneCount} milestones kept out of ${data.events.length} attempts.` : "") };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.gaussiansampler = {
    title: "Gaussian Sampler",
    badge: "narrow-passage sampling",
    subtitle: "Sample q1 uniformly, q2 at a Gaussian offset from q1; keep the free one as a milestone only if exactly one of the pair is in collision.",
    width: 560, height: 360,
    legend: [
      { color: "#6a4fb0", label: "milestone" },
      { color: "#2b6cb0", label: "free sample" },
      { color: "#c23b3b", label: "colliding sample" },
      { color: "#2f8f5b", label: "accepted pair" },
    ],
    makeSim,
    pythonCode: `
def gaussian_sample(is_free, sample_anywhere, std=22):
    q1 = sample_anywhere()
    q2 = q1 + gaussian_offset(std)          # same point, small random displacement
    c1, c2 = is_free(q1), is_free(q2)
    if c1 == c2:
        return None                         # both free or both blocked: no signal
    return q1 if c1 else q2                 # keep the free half of the pair
`,
  };
})();
