/* Visibility-based sampling: only keep a candidate milestone if it usefully
   expands what the roadmap can currently see. A coarse grid of estimator
   points stands in for "free space"; each candidate's visibility set is the
   subset of estimator points it has straight-line visibility to (the same
   segment-collision check PRM uses for its local planner). A candidate is
   rejected if most of what it sees is already visible from some existing
   milestone -- so, unlike plain uniform PRM, the roadmap ends up with far
   fewer, more strategically placed milestones, several of them planted right
   at the mouth of the narrow passage where new visibility is cheap to gain. */
(function () {
  "use strict";
  const { makeNarrowPassageWorld } = window.RMP;

  function dist(a, b) { return Math.hypot(a.x - b.x, a.y - b.y); }
  function segmentFree(world, a, b, step) {
    step = step || 5;
    const d = dist(a, b), n = Math.max(1, Math.ceil(d / step));
    for (let i = 0; i <= n; i++) {
      const t = i / n;
      if (!world.isFree(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) return false;
    }
    return true;
  }

  function buildEstimatorGrid(world, spacing) {
    const pts = [];
    for (let y = world.margin * 0.6; y < world.height - world.margin * 0.6; y += spacing) {
      for (let x = world.margin * 0.6; x < world.width - world.margin * 0.6; x += spacing) {
        if (world.isFree(x, y)) pts.push({ x, y });
      }
    }
    return pts;
  }

  function computeVisibilitySampling(world, rng, attempts, spacing, coverageThresh) {
    const est = buildEstimatorGrid(world, spacing);
    const covered = new Uint8Array(est.length);
    const milestones = [];
    const edges = [];
    const events = [];

    for (let a = 0; a < attempts; a++) {
      const q = world.randFreePoint(6);
      const visible = [];
      for (let i = 0; i < est.length; i++) if (segmentFree(world, q, est[i])) visible.push(i);

      if (!visible.length) {
        events.push({ q, accepted: false, reason: "no-visibility", coveredSnapshot: covered.slice(), edgeCount: edges.length, milestoneCount: milestones.length });
        continue;
      }
      const alreadyCovered = visible.reduce((s, i) => s + (covered[i] ? 1 : 0), 0);
      const fraction = alreadyCovered / visible.length;
      const accept = milestones.length === 0 || fraction < coverageThresh;

      if (accept) {
        visible.forEach((i) => { covered[i] = 1; });
        const newIdx = milestones.length;
        milestones.push(q);
        for (let j = 0; j < newIdx; j++) {
          if (segmentFree(world, q, milestones[j])) edges.push([newIdx, j]);
        }
      }
      events.push({
        q, accepted: accept, reason: accept ? "accepted" : "redundant", fraction,
        coveredSnapshot: covered.slice(), edgeCount: edges.length, milestoneCount: milestones.length,
      });
    }
    return { est, milestones, edges, events };
  }

  function draw(ctx, world, data, idx) {
    world.draw(ctx, { alpha: 0.9 });
    const ev = idx >= 0 ? data.events[idx] : null;
    const coverage = ev ? ev.coveredSnapshot : new Uint8Array(data.est.length);
    const edgeCount = ev ? ev.edgeCount : 0;
    const milestoneCount = ev ? ev.milestoneCount : 0;

    ctx.save();
    for (let i = 0; i < data.est.length; i++) {
      const p = data.est[i];
      ctx.fillStyle = coverage[i] ? "rgba(43,108,176,0.28)" : "rgba(136,146,160,0.22)";
      ctx.beginPath(); ctx.arc(p.x, p.y, 1.6, 0, Math.PI * 2); ctx.fill();
    }
    ctx.restore();

    ctx.save();
    ctx.strokeStyle = "rgba(111,168,220,0.6)";
    ctx.lineWidth = 1.1;
    for (let k = 0; k < edgeCount; k++) {
      const [i, j] = data.edges[k];
      const a = data.milestones[i], b = data.milestones[j];
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
    ctx.restore();

    ctx.save();
    ctx.fillStyle = "#6a4fb0";
    for (let k = 0; k < milestoneCount; k++) {
      const p = data.milestones[k];
      ctx.beginPath(); ctx.arc(p.x, p.y, 4, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = "#fff"; ctx.lineWidth = 1; ctx.stroke();
    }
    ctx.restore();

    if (ev) {
      ctx.save();
      ctx.strokeStyle = ev.accepted ? "#2f8f5b" : "#c23b3b";
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(ev.q.x, ev.q.y, 6, 0, Math.PI * 2); ctx.stroke();
      ctx.restore();
    }

    world.drawMarker(ctx, world.start.x, world.start.y, "#2b6cb0", "start");
    world.drawMarker(ctx, world.goal.x, world.goal.y, "#2f8f5b", "goal");
  }

  function makeSim({ rng, width, height }) {
    const world = makeNarrowPassageWorld(rng, { width, height });
    const ATTEMPTS = 90, SPACING = 20, COVERAGE_THRESH = 0.55;
    const data = computeVisibilitySampling(world, rng, ATTEMPTS, SPACING, COVERAGE_THRESH);
    let idx = -1;
    return {
      draw(ctx) { draw(ctx, world, data, idx); },
      step() {
        idx = Math.min(idx + 1, data.events.length - 1);
        const done = idx >= data.events.length - 1;
        const ev = data.events[idx];
        let note;
        if (ev.reason === "no-visibility") note = "Candidate sees no estimator points at all (fully boxed in) — discarded.";
        else if (ev.reason === "accepted") note = `Candidate's visibility set was mostly new (only ${Math.round((ev.fraction || 0) * 100)}% already covered) — kept as milestone ${ev.milestoneCount}.`;
        else note = `Candidate sees ${Math.round(ev.fraction * 100)}% already-covered ground — redundant with the existing roadmap, discarded.`;
        return { done, note: note + (done ? ` Final roadmap: ${ev.milestoneCount} milestones from ${data.events.length} candidates — far fewer than uniform PRM would keep.` : "") };
      },
    };
  }

  window.RMP = window.RMP || {};
  window.RMP.vizDefs = window.RMP.vizDefs || {};
  window.RMP.vizDefs.visibilitysampling = {
    title: "Visibility-Based Sampling",
    badge: "narrow-passage sampling",
    subtitle: "Keep a candidate milestone only if it sees a useful amount of free space not already visible from the existing roadmap (estimated on a coarse grid).",
    width: 560, height: 360,
    legend: [
      { color: "#6a4fb0", label: "milestone" },
      { color: "rgba(111,168,220,0.8)", label: "roadmap edge" },
      { color: "rgba(43,108,176,0.6)", label: "covered estimator point" },
      { color: "#2f8f5b", label: "accepted candidate" },
      { color: "#c23b3b", label: "rejected (redundant) candidate" },
    ],
    makeSim,
    pythonCode: `
def visibility_sample(is_free, sample_free, estimator_grid, covered, thresh=0.55):
    q = sample_free()
    visible = [p for p in estimator_grid if segment_free(q, p, is_free)]
    if not visible:
        return None
    already = sum(1 for p in visible if p in covered)
    if already / len(visible) >= thresh:
        return None                          # mostly redundant with the roadmap
    covered.update(visible)
    return q                                 # usefully expands total visibility
`,
  };
})();
