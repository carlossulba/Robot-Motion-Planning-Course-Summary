/* GVD-retraction sampling: sample a raw configuration anywhere, then push
   ("retract") it toward the Generalized Voronoi Diagram -- the max-clearance
   skeleton of the obstacle world, reusing the same computeGvd() the GVD
   roadmap demo uses. Because the skeleton runs straight through the middle
   of any narrow passage by construction, retracted samples pile up exactly
   where plain uniform sampling struggles most. */
(function () {
  "use strict";
  const { makeNarrowPassageWorld, computeGvd } = window.RMP;

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }

  function anywhere(world, rng) {
    return {
      x: world.margin * 0.4 + rng() * (world.width - world.margin * 0.8),
      y: world.margin * 0.4 + rng() * (world.height - world.margin * 0.8),
    };
  }

  function nearestCell(cells, p) {
    let best = null, bestD = Infinity;
    for (const c of cells) {
      const d = dist(c, p);
      if (d < bestD) { bestD = d; best = c; }
    }
    return best;
  }

  function computeRetraction(world, rng, nSamples, substeps) {
    const gvd = computeGvd(world);
    const cells = gvd.gvdCells;
    const frames = [];
    const settledTargets = [];
    for (let a = 0; a < nSamples; a++) {
      if (!cells.length) break;
      const raw = anywhere(world, rng);
      const target = nearestCell(cells, raw);
      settledTargets.push(target);
      for (let s = 0; s <= substeps; s++) {
        frames.push({ attemptIdx: a, raw, target, t: s / substeps });
      }
    }
    return { gvdCells: cells, frames, settledTargets };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });

    ctx.save();
    ctx.fillStyle = "rgba(183,83,44,0.35)";
    for (const c of data.gvdCells) { ctx.beginPath(); ctx.arc(c.x, c.y, 1.4, 0, Math.PI * 2); ctx.fill(); }
    ctx.restore();

    if (idx >= 0 && data.frames.length) {
      const f = data.frames[Math.min(idx, data.frames.length - 1)];
      const settledCount = f.t >= 1 ? f.attemptIdx + 1 : f.attemptIdx;

      ctx.save();
      ctx.fillStyle = "#b7532c";
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 1;
      for (let k = 0; k < settledCount; k++) {
        const t = data.settledTargets[k];
        ctx.beginPath(); ctx.arc(t.x, t.y, 4, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      }
      ctx.restore();

      const cx = f.raw.x + (f.target.x - f.raw.x) * f.t;
      const cy = f.raw.y + (f.target.y - f.raw.y) * f.t;
      ctx.save();
      ctx.strokeStyle = "rgba(111,168,220,0.7)";
      ctx.setLineDash([2, 3]);
      ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.moveTo(f.raw.x, f.raw.y); ctx.lineTo(cx, cy); ctx.stroke();
      ctx.restore();

      ctx.save();
      ctx.fillStyle = "#8892a0";
      ctx.beginPath(); ctx.arc(f.raw.x, f.raw.y, 3, 0, Math.PI * 2); ctx.fill();
      ctx.restore();

      ctx.save();
      ctx.fillStyle = f.t >= 1 ? "#b7532c" : "#2b6cb0";
      ctx.beginPath(); ctx.arc(cx, cy, 4.5, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 1.2; ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height, world: sharedWorld }) {
    const world = sharedWorld || makeNarrowPassageWorld(rng, { width, height });
    const NSAMPLES = 24, SUBSTEPS = 6;
    const data = computeRetraction(world, rng, NSAMPLES, SUBSTEPS);
    let idx = -1;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        if (!data.frames.length) return { done: true, note: "No GVD skeleton could be extracted for this layout — try Generate new." };
        idx = Math.min(idx + 1, data.frames.length - 1);
        const done = idx >= data.frames.length - 1;
        const f = data.frames[idx];
        const note = f.t >= 1
          ? `Sample ${f.attemptIdx + 1} reached the skeleton — retained as a max-clearance milestone.`
          : `Retracting sample ${f.attemptIdx + 1} toward its nearest point on the Voronoi skeleton (${Math.round(f.t * 100)}% of the way there).`;
        return { done, note };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.gvdretraction = {
    title: "GVD-Retraction Sampling",
    badge: "narrow-passage sampling",
    subtitle: "Sample a raw configuration, then retract it toward the nearest point of the Generalized Voronoi Diagram — the max-clearance skeleton that runs through every passage.",
    width: 560, height: 360,
    legend: [
      { color: "rgba(183,83,44,0.6)", label: "GVD skeleton" },
      { color: "#8892a0", label: "raw sample" },
      { color: "#2b6cb0", label: "retracting sample" },
      { color: "#b7532c", label: "settled milestone" },
    ],
    makeSim,
    pythonCode: `
def gvd_retraction_sample(sample_anywhere, gvd_skeleton):
    raw = sample_anywhere()
    target = nearest_point(gvd_skeleton, raw)   # same skeleton the GVD roadmap uses
    return slide(raw, target)                   # animate raw -> target, keep target as milestone
`,
  };
})();
