/* ===========================================================
   world.js — shared random-world generator + collision queries
   used by (nearly) every visualization: Bug algorithms, Brushfire,
   Wave-Front, potential fields, visibility graph, Voronoi/GVD,
   cell decompositions, PRM, RRT...
   =========================================================== */

(function (global) {
  "use strict";

  function pointInPoly(x, y, pts) {
    let inside = false;
    for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      const xi = pts[i][0], yi = pts[i][1];
      const xj = pts[j][0], yj = pts[j][1];
      const intersect = ((yi > y) !== (yj > y)) &&
        (x < ((xj - xi) * (y - yi)) / (yj - yi) + xi);
      if (intersect) inside = !inside;
    }
    return inside;
  }

  // distance from point to segment
  function distToSeg(px, py, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay;
    const len2 = dx * dx + dy * dy;
    let t = len2 === 0 ? 0 : ((px - ax) * dx + (py - ay) * dy) / len2;
    t = Math.max(0, Math.min(1, t));
    const cx = ax + t * dx, cy = ay + t * dy;
    return Math.hypot(px - cx, py - cy);
  }

  function distToPoly(x, y, pts) {
    let d = Infinity;
    for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      d = Math.min(d, distToSeg(x, y, pts[i][0], pts[i][1], pts[j][0], pts[j][1]));
    }
    return d;
  }

  // proper segment-segment intersection (orientation test), endpoints count as touching
  function segmentsIntersect(ax, ay, bx, by, cx, cy, dx, dy) {
    function orient(px, py, qx, qy, rx, ry) {
      const v = (qx - px) * (ry - py) - (qy - py) * (rx - px);
      return v > 1e-9 ? 1 : v < -1e-9 ? -1 : 0;
    }
    function onSeg(px, py, qx, qy, rx, ry) {
      return Math.min(px, rx) - 1e-6 <= qx && qx <= Math.max(px, rx) + 1e-6 &&
        Math.min(py, ry) - 1e-6 <= qy && qy <= Math.max(py, ry) + 1e-6;
    }
    const o1 = orient(ax, ay, bx, by, cx, cy);
    const o2 = orient(ax, ay, bx, by, dx, dy);
    const o3 = orient(cx, cy, dx, dy, ax, ay);
    const o4 = orient(cx, cy, dx, dy, bx, by);
    if (o1 !== o2 && o3 !== o4) return true;
    if (o1 === 0 && onSeg(ax, ay, cx, cy, bx, by)) return true;
    if (o2 === 0 && onSeg(ax, ay, dx, dy, bx, by)) return true;
    if (o3 === 0 && onSeg(cx, cy, ax, ay, dx, dy)) return true;
    if (o4 === 0 && onSeg(cx, cy, bx, by, dx, dy)) return true;
    return false;
  }

  function segmentIntersectsPoly(ax, ay, bx, by, pts) {
    if (pointInPoly(ax, ay, pts) || pointInPoly(bx, by, pts)) return true;
    for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      if (segmentsIntersect(ax, ay, bx, by, pts[i][0], pts[i][1], pts[j][0], pts[j][1])) return true;
    }
    return false;
  }

  // ---- obstacle footprint / overlap helpers (shared by makeWorld & makeBugWorld) ----
  function footprintOf(o) {
    if (o.type === "circle") return { cx: o.cx, cy: o.cy, r: o.r };
    let r = 0;
    for (const [x, y] of o.pts) r = Math.max(r, Math.hypot(x - o.cx, y - o.cy));
    return { cx: o.cx, cy: o.cy, r };
  }

  function makeOverlapChecker(footprints, gap) {
    return function overlapsExisting(fp) {
      for (const f of footprints) {
        if (Math.hypot(fp.cx - f.cx, fp.cy - f.cy) < fp.r + f.r + gap) return true;
      }
      return false;
    };
  }

  function makePointInObstacle(obstacles) {
    return function pointInObstacle(x, y, pad) {
      pad = pad || 0;
      for (const o of obstacles) {
        if (o.type === "circle") {
          if (Math.hypot(x - o.cx, y - o.cy) <= o.r + pad) return true;
        } else {
          if (pointInPoly(x, y, o.pts)) return true;
          if (pad > 0 && distToPoly(x, y, o.pts) <= pad) return true;
        }
      }
      return false;
    };
  }

  function makeDistToNearestObstacle(obstacles) {
    return function distToNearestObstacle(x, y) {
      let d = Infinity;
      for (const o of obstacles) {
        if (o.type === "circle") d = Math.min(d, Math.hypot(x - o.cx, y - o.cy) - o.r);
        else d = Math.min(d, pointInPoly(x, y, o.pts) ? 0 : distToPoly(x, y, o.pts));
      }
      return Math.max(0, d);
    };
  }

  function makeNearestObstacle(obstacles) {
    return function nearestObstacle(x, y) {
      let best = null, bestD = Infinity;
      for (const o of obstacles) {
        const d = o.type === "circle"
          ? Math.abs(Math.hypot(x - o.cx, y - o.cy) - o.r)
          : distToPoly(x, y, o.pts);
        if (d < bestD) { bestD = d; best = o; }
      }
      return best;
    };
  }

  function makeDraw(obstacles) {
    return function draw(ctx, opts) {
      opts = opts || {};
      ctx.save();
      ctx.fillStyle = opts.fill || "#93a3b8";
      ctx.strokeStyle = opts.stroke || "#5b6b80";
      ctx.lineWidth = 1.4;
      ctx.globalAlpha = opts.alpha === undefined ? 0.85 : opts.alpha;
      for (const o of obstacles) {
        ctx.beginPath();
        if (o.type === "circle") {
          ctx.arc(o.cx, o.cy, o.r, 0, Math.PI * 2);
        } else {
          ctx.moveTo(o.pts[0][0], o.pts[0][1]);
          for (let k = 1; k < o.pts.length; k++) ctx.lineTo(o.pts[k][0], o.pts[k][1]);
          ctx.closePath();
        }
        ctx.fill();
        ctx.stroke();
      }
      ctx.restore();
    };
  }

  function drawMarker(ctx, x, y, color, label) {
    ctx.save();
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(x, y, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 1.5;
    ctx.stroke();
    if (label) {
      ctx.fillStyle = color;
      ctx.font = "bold 11px sans-serif";
      ctx.fillText(label, x + 9, y - 8);
    }
    ctx.restore();
  }

  function inBoundsFactory(width, height) {
    return function inBounds(x, y) { return x >= 2 && y >= 2 && x <= width - 2 && y <= height - 2; };
  }

  function buildGrid(width, height, cellSize, inBounds, pointInObstacle) {
    const cols = Math.ceil(width / cellSize);
    const rows = Math.ceil(height / cellSize);
    const grid = new Uint8Array(cols * rows);
    for (let j = 0; j < rows; j++) {
      for (let i = 0; i < cols; i++) {
        const x = (i + 0.5) * cellSize, y = (j + 0.5) * cellSize;
        grid[j * cols + i] = (!inBounds(x, y) || pointInObstacle(x, y)) ? 1 : 0;
      }
    }
    // Force the full outer ring of grid cells to be blocked, symmetrically on
    // all four edges. The pixel-based `inBounds` check above (a fixed 2px
    // margin) only reliably blocks the far edges -- ceil()'d grid dimensions
    // mean column cols-1 / row rows-1 always overflow past width-2/height-2 --
    // while column 0 / row 0 (cell centers at cellSize/2, comfortably inside
    // the margin for any realistic cellSize) never do. That asymmetry left
    // Brushfire/GVD seeding "fire" from the bottom/right boundary but not the
    // top/left, so this ring is set explicitly instead of relying on the
    // pixel margin lining up with the cell grid.
    for (let i = 0; i < cols; i++) { grid[i] = 1; grid[(rows - 1) * cols + i] = 1; }
    for (let j = 0; j < rows; j++) { grid[j * cols] = 1; grid[j * cols + (cols - 1)] = 1; }
    return { cols, rows, grid };
  }

  function finishWorld(base) {
    const { width, height, margin, obstacles, start, goal, cellSize } = base;
    const inBounds = inBoundsFactory(width, height);
    const pointInObstacle = makePointInObstacle(obstacles);
    const distToNearestObstacle = makeDistToNearestObstacle(obstacles);
    const nearestObstacle = makeNearestObstacle(obstacles);
    const draw = makeDraw(obstacles);
    const { cols, rows, grid } = buildGrid(width, height, cellSize, inBounds, pointInObstacle);

    function randFreePoint(pad) {
      for (let tries = 0; tries < 400; tries++) {
        const x = margin * 0.6 + Math.random() * (width - margin * 1.2);
        const y = margin * 0.6 + Math.random() * (height - margin * 1.2);
        if (inBounds(x, y) && !pointInObstacle(x, y, pad || 12)) return { x, y };
      }
      return { x: width / 2, y: height / 2 };
    }

    return {
      width, height, margin, obstacles, start, goal,
      cellSize, cols, rows, grid,
      isFree: (x, y, pad) => inBounds(x, y) && !pointInObstacle(x, y, pad),
      inBounds,
      distToNearestObstacle,
      nearestObstacle,
      cellOf: (x, y) => ({ i: Math.max(0, Math.min(cols - 1, Math.floor(x / cellSize))), j: Math.max(0, Math.min(rows - 1, Math.floor(y / cellSize))) }),
      isFreeCell: (i, j) => i >= 0 && j >= 0 && i < cols && j < rows && grid[j * cols + i] === 0,
      draw, drawMarker,
      randFreePoint,
    };
  }

  function makeWorld(rng, opts) {
    opts = opts || {};
    const width = opts.width, height = opts.height;
    const margin = opts.margin || 34;
    const nObstacles = opts.nObstacles || (3 + Math.floor(rng() * 3));
    const obstacles = [];
    const footprints = [];
    const GAP = opts.gap === undefined ? 16 : opts.gap;
    const overlapsExisting = makeOverlapChecker(footprints, GAP);

    for (let i = 0; i < nObstacles; i++) {
      let placed = false;
      for (let tries = 0; tries < 50 && !placed; tries++) {
        const cx = margin + rng() * (width - 2 * margin);
        const cy = margin + rng() * (height - 2 * margin);
        let candidate;
        if (!opts.polygonsOnly && rng() < 0.4) {
          const r = 16 + rng() * (opts.maxR || 26);
          candidate = { type: "circle", cx, cy, r };
        } else {
          const sides = 3 + Math.floor(rng() * 4);
          const rad = 20 + rng() * (opts.maxR || 28);
          const rot = rng() * Math.PI * 2;
          const pts = [];
          for (let k = 0; k < sides; k++) {
            const a = rot + (k / sides) * Math.PI * 2 + (rng() - 0.5) * 0.35;
            const rr = rad * (0.72 + rng() * 0.5);
            pts.push([cx + Math.cos(a) * rr, cy + Math.sin(a) * rr]);
          }
          candidate = { type: "poly", pts, cx, cy };
        }
        const fp = footprintOf(candidate);
        if (fp.cx - fp.r < 2 || fp.cy - fp.r < 2 || fp.cx + fp.r > width - 2 || fp.cy + fp.r > height - 2) continue;
        if (overlapsExisting(fp)) continue;
        obstacles.push(candidate);
        footprints.push(fp);
        placed = true;
      }
      // if 50 tries all failed (crowded canvas), just skip this obstacle rather than overlap one
    }

    const pointInObstacle = makePointInObstacle(obstacles);
    function inBounds(x, y) { return x >= 2 && y >= 2 && x <= width - 2 && y <= height - 2; }
    function randFreePoint(pad) {
      for (let tries = 0; tries < 400; tries++) {
        const x = margin * 0.6 + rng() * (width - margin * 1.2);
        const y = margin * 0.6 + rng() * (height - margin * 1.2);
        if (inBounds(x, y) && !pointInObstacle(x, y, pad || 12)) return { x, y };
      }
      return { x: width / 2, y: height / 2 };
    }

    let start = randFreePoint();
    let goal = randFreePoint();
    let tries = 0;
    while (Math.hypot(start.x - goal.x, start.y - goal.y) < Math.min(width, height) * 0.55 && tries < 200) {
      goal = randFreePoint();
      tries++;
    }

    const cellSize = opts.cellSize || 6;
    const world = finishWorld({ width, height, margin, obstacles, start, goal, cellSize });
    world.randFreePoint = randFreePoint;
    return world;
  }

  // ---- obstacle-shape generators used by makeBugWorld ----
  // Each returns {type:"poly", pts:[[x,y],...], cx, cy}. Points at radially
  // increasing angles never self-intersect, so "simple"/"complex" are safe by
  // construction; "closed"/"open" trace an outer curve then an inner
  // curve back, the standard (also self-intersection-safe) thick-band recipe.
  const SHAPE_GENERATORS = {
    simple(rng, cx, cy, rad) {
      const sides = 3 + Math.floor(rng() * 4);
      const rot = rng() * Math.PI * 2;
      const pts = [];
      for (let k = 0; k < sides; k++) {
        const a = rot + (k / sides) * Math.PI * 2 + (rng() - 0.5) * 0.35;
        const rr = rad * (0.72 + rng() * 0.5);
        pts.push([cx + Math.cos(a) * rr, cy + Math.sin(a) * rr]);
      }
      return { type: "poly", pts, cx, cy };
    },
    complex(rng, cx, cy, rad) {
      const sides = 10 + Math.floor(rng() * 5);
      const rot = rng() * Math.PI * 2;
      const pts = [];
      for (let k = 0; k < sides; k++) {
        const a = rot + (k / sides) * Math.PI * 2 + (rng() - 0.5) * 0.15;
        const deep = k % 2 === 0;
        const rr = rad * (deep ? 0.35 + rng() * 0.25 : 0.85 + rng() * 0.35);
        pts.push([cx + Math.cos(a) * rr, cy + Math.sin(a) * rr]);
      }
      return { type: "poly", pts, cx, cy };
    },
    closed(rng, cx, cy, rad) {
      // near-complete ring (horseshoe with a narrow gap) -- classic near-trap shape
      const outerR = rad, innerR = rad * (0.4 + rng() * 0.15);
      const gap = (20 + rng() * 25) * Math.PI / 180;
      const sweep = Math.PI * 2 - gap;
      const rot = rng() * Math.PI * 2;
      const a0 = rot + gap / 2;
      const n = 18;
      const pts = [];
      for (let k = 0; k <= n; k++) {
        const a = a0 + (k / n) * sweep;
        pts.push([cx + Math.cos(a) * outerR, cy + Math.sin(a) * outerR]);
      }
      for (let k = n; k >= 0; k--) {
        const a = a0 + (k / n) * sweep;
        pts.push([cx + Math.cos(a) * innerR, cy + Math.sin(a) * innerR]);
      }
      return { type: "poly", pts, cx, cy };
    },
    open(rng, cx, cy, rad) {
      // wide hook/bracket -- same ring recipe, much wider gap, ends far apart
      const outerR = rad, innerR = rad * (0.45 + rng() * 0.15);
      const sweep = (90 + rng() * 60) * Math.PI / 180;
      const rot = rng() * Math.PI * 2;
      const n = 14;
      const pts = [];
      for (let k = 0; k <= n; k++) {
        const a = rot + (k / n) * sweep;
        pts.push([cx + Math.cos(a) * outerR, cy + Math.sin(a) * outerR]);
      }
      for (let k = n; k >= 0; k--) {
        const a = rot + (k / n) * sweep;
        pts.push([cx + Math.cos(a) * innerR, cy + Math.sin(a) * innerR]);
      }
      return { type: "poly", pts, cx, cy };
    },
  };
  const SHAPE_ORDER = ["closed", "open", "simple", "complex"];

  function shuffled(rng, arr) {
    const a = arr.slice();
    for (let i = a.length - 1; i > 0; i--) {
      const j = Math.floor(rng() * (i + 1));
      [a[i], a[j]] = [a[j], a[i]];
    }
    return a;
  }

  // Bug-family world: same interface as makeWorld, but (a) always places one
  // obstacle so it genuinely blocks the straight start->goal line (guaranteeing
  // motion-to-goal actually triggers boundary-following), and (b) cycles the
  // remaining obstacles through four distinct shape families (closed, open,
  // simple, complex) instead of just circles/near-convex blobs.
  function makeBugWorld(rng, opts) {
    opts = opts || {};
    const width = opts.width, height = opts.height;
    const margin = opts.margin || 34;
    const nObstacles = opts.nObstacles || (3 + Math.floor(rng() * 3));
    const maxR = opts.maxR || 34;
    const GAP = opts.gap === undefined ? 16 : opts.gap;

    function inBounds(x, y) { return x >= margin * 0.5 && y >= margin * 0.5 && x <= width - margin * 0.5 && y <= height - margin * 0.5; }

    let start = { x: margin + rng() * (width - 2 * margin), y: margin + rng() * (height - 2 * margin) };
    let goal = { x: margin + rng() * (width - 2 * margin), y: margin + rng() * (height - 2 * margin) };
    let sgTries = 0;
    while (Math.hypot(start.x - goal.x, start.y - goal.y) < Math.min(width, height) * 0.55 && sgTries < 200) {
      goal = { x: margin + rng() * (width - 2 * margin), y: margin + rng() * (height - 2 * margin) };
      sgTries++;
    }

    const typeQueue = [];
    const bag = shuffled(rng, SHAPE_ORDER);
    for (let i = 0; i < nObstacles; i++) typeQueue.push(bag[i % bag.length]);

    const obstacles = [];
    const footprints = [];
    const overlapsExisting = makeOverlapChecker(footprints, GAP);
    function keepsClearOfEndpoints(fp) {
      const pad = 16;
      return Math.hypot(fp.cx - start.x, fp.cy - start.y) > fp.r + pad &&
        Math.hypot(fp.cx - goal.x, fp.cy - goal.y) > fp.r + pad;
    }
    function inCanvas(fp) {
      return fp.cx - fp.r > 2 && fp.cy - fp.r > 2 && fp.cx + fp.r < width - 2 && fp.cy + fp.r < height - 2;
    }

    // (a) place the blocking obstacle astride the start-goal segment
    let blocked = false;
    for (let tries = 0; tries < 150 && !blocked; tries++) {
      const t = 0.3 + rng() * 0.4;
      const jitter = (rng() - 0.5) * 26;
      const dx = goal.x - start.x, dy = goal.y - start.y;
      const len = Math.hypot(dx, dy) || 1;
      const nx = -dy / len, ny = dx / len;
      const bx = start.x + dx * t + nx * jitter;
      const by = start.y + dy * t + ny * jitter;
      const rad = 22 + rng() * maxR;
      const candidate = SHAPE_GENERATORS[typeQueue[0]](rng, bx, by, rad);
      const fp = footprintOf(candidate);
      if (!inCanvas(fp)) continue;
      if (!segmentIntersectsPoly(start.x, start.y, goal.x, goal.y, candidate.pts)) continue;
      if (pointInPoly(start.x, start.y, candidate.pts) || pointInPoly(goal.x, goal.y, candidate.pts)) continue;
      obstacles.push(candidate);
      footprints.push(fp);
      blocked = true;
    }
    if (!blocked) {
      // safety net: a simple polygon centered exactly on the segment always blocks it
      const t = 0.5, dx = goal.x - start.x, dy = goal.y - start.y;
      const bx = start.x + dx * t, by = start.y + dy * t;
      const candidate = SHAPE_GENERATORS.simple(rng, bx, by, 26);
      obstacles.push(candidate);
      footprints.push(footprintOf(candidate));
    }

    // (b) scatter the remaining obstacles, cycling through the shape queue
    for (let i = 1; i < nObstacles; i++) {
      let placed = false;
      for (let tries = 0; tries < 50 && !placed; tries++) {
        const cx = margin + rng() * (width - 2 * margin);
        const cy = margin + rng() * (height - 2 * margin);
        const rad = 20 + rng() * maxR;
        const candidate = SHAPE_GENERATORS[typeQueue[i]](rng, cx, cy, rad);
        const fp = footprintOf(candidate);
        if (!inCanvas(fp)) continue;
        if (overlapsExisting(fp)) continue;
        if (!keepsClearOfEndpoints(fp)) continue;
        obstacles.push(candidate);
        footprints.push(fp);
        placed = true;
      }
    }

    const cellSize = opts.cellSize || 6;
    return finishWorld({ width, height, margin, obstacles, start, goal, cellSize });
  }

  // ---- narrow-passage world (shared by the "other sampling strategies" demos) ----
  // Two wall segments (a top block and a bottom block) span most of the
  // canvas height at some x position, leaving a single narrow horizontal gap
  // between them; start sits in the free room to the left, goal in the free
  // room to the right, so any start->goal path has to thread the gap. A
  // handful of small scattered obstacles are added on both sides purely for
  // visual richness -- they never block the only route through the gap.
  function rectPts(x0, y0, x1, y1) {
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
  }

  function makeNarrowPassageWorld(rng, opts) {
    opts = opts || {};
    const width = opts.width, height = opts.height;
    const margin = opts.margin || 30;
    const wallThickness = opts.wallThickness || 24;
    const gapSize = opts.gapSize === undefined ? (16 + rng() * 8) : opts.gapSize;
    const wallX = width * (0.42 + rng() * 0.16);
    const gapCenterY = height * (0.38 + rng() * 0.24);
    const topTop = margin * 0.5;
    const topBottom = gapCenterY - gapSize / 2;
    const botTop = gapCenterY + gapSize / 2;
    const botBottom = height - margin * 0.5;

    const obstacles = [];
    if (topBottom > topTop + 4) {
      obstacles.push({
        type: "poly", cx: wallX, cy: (topTop + topBottom) / 2,
        pts: rectPts(wallX - wallThickness / 2, topTop, wallX + wallThickness / 2, topBottom),
      });
    }
    if (botBottom > botTop + 4) {
      obstacles.push({
        type: "poly", cx: wallX, cy: (botTop + botBottom) / 2,
        pts: rectPts(wallX - wallThickness / 2, botTop, wallX + wallThickness / 2, botBottom),
      });
    }

    // a little clutter on each side, well clear of the wall and the gap mouth
    const footprints = obstacles.map(footprintOf);
    const overlapsExisting = makeOverlapChecker(footprints, 14);
    const nClutter = opts.nClutter === undefined ? 2 : opts.nClutter;
    for (let side = 0; side < 2; side++) {
      for (let i = 0; i < nClutter; i++) {
        for (let tries = 0; tries < 30; tries++) {
          const cx = side === 0
            ? margin * 1.4 + rng() * (wallX - wallThickness / 2 - margin * 2)
            : wallX + wallThickness / 2 + margin * 0.6 + rng() * (width - margin * 1.4 - (wallX + wallThickness / 2));
          const cy = margin * 1.2 + rng() * (height - margin * 2.4);
          const rad = 12 + rng() * 16;
          const candidate = SHAPE_GENERATORS.simple(rng, cx, cy, rad);
          const fp = footprintOf(candidate);
          if (Math.abs(cy - gapCenterY) < gapSize / 2 + rad + 20 && Math.abs(cx - wallX) < wallThickness / 2 + rad + 40) continue;
          if (overlapsExisting(fp)) continue;
          obstacles.push(candidate);
          footprints.push(fp);
          break;
        }
      }
    }

    const roomMarginX = margin * 1.3;
    const start = {
      x: roomMarginX + rng() * Math.max(10, (wallX - wallThickness / 2 - roomMarginX * 1.4)),
      y: gapCenterY + (rng() - 0.5) * height * 0.4,
    };
    const goal = {
      x: wallX + wallThickness / 2 + roomMarginX * 0.6 + rng() * Math.max(10, (width - roomMarginX * 1.3 - (wallX + wallThickness / 2))),
      y: gapCenterY + (rng() - 0.5) * height * 0.4,
    };

    const cellSize = opts.cellSize || 6;
    const world = finishWorld({ width, height, margin, obstacles, start, goal, cellSize });
    world.gapCenterY = gapCenterY;
    world.wallX = wallX;
    return world;
  }

  // ---- boundary tracing (shared by Bug1 / Bug2 / Tangent Bug) ----
  // Returns an ordered array of {x,y} points offset `eps` outward from the
  // obstacle boundary, starting as close as possible to `near`, walking a
  // full loop (or until arcBudget is exhausted) in one consistent winding
  // direction, sampled roughly every `step` px of arc length.
  function traceObstacleBoundary(obstacle, near, step, eps, arcBudget) {
    step = step || 4; eps = eps === undefined ? 3 : eps; arcBudget = arcBudget || Infinity;
    const pts = [];
    if (obstacle.type === "circle") {
      const R = obstacle.r + eps;
      const a0 = Math.atan2(near.y - obstacle.cy, near.x - obstacle.cx);
      const dA = step / R;
      const maxA = Math.min(Math.PI * 2, arcBudget / R);
      for (let a = 0; a <= maxA + 1e-9; a += dA) {
        const ang = a0 + a;
        pts.push({ x: obstacle.cx + Math.cos(ang) * R, y: obstacle.cy + Math.sin(ang) * R, param: a });
      }
      return pts;
    }
    // polygon: offset each vertex outward along the true local normal (average
    // of its two adjacent edge normals) -- centroid-direction offsetting is
    // only correct for convex/star-shaped polygons and pushes points the wrong
    // way on concave shapes (horseshoes, deep notches), so this must be geometric.
    const raw = obstacle.pts;
    const n = raw.length;
    let signedArea = 0;
    for (let i = 0; i < n; i++) {
      const [x1, y1] = raw[i], [x2, y2] = raw[(i + 1) % n];
      signedArea += x1 * y2 - x2 * y1;
    }
    const outward = signedArea < 0 ? 1 : -1; // canvas y-down: CW (area<0) => outward = left-normal
    const off = raw.map(([x, y], i) => {
      const [px, py] = raw[(i - 1 + n) % n];
      const [nx, ny] = raw[(i + 1) % n];
      const e1x = x - px, e1y = y - py;
      const e2x = nx - x, e2y = ny - y;
      const len1 = Math.hypot(e1x, e1y) || 1, len2 = Math.hypot(e2x, e2y) || 1;
      // left-hand normal of each edge (rotate direction by -90deg), then average
      const n1x = -(e1y / len1) * outward, n1y = (e1x / len1) * outward;
      const n2x = -(e2y / len2) * outward, n2y = (e2x / len2) * outward;
      let sx = n1x + n2x, sy = n1y + n2y;
      const slen = Math.hypot(sx, sy) || 1;
      sx /= slen; sy /= slen;
      return [x + sx * eps, y + sy * eps];
    });
    const edgeLen = [];
    let perim = 0;
    for (let i = 0; i < n; i++) {
      const a = off[i], b = off[(i + 1) % n];
      const L = Math.hypot(b[0] - a[0], b[1] - a[1]);
      edgeLen.push(L); perim += L;
    }
    // find nearest point on the offset boundary to `near` -> starting edge index + local s
    let bestEdge = 0, bestT = 0, bestD = Infinity, bestS = 0;
    let acc = 0;
    for (let i = 0; i < n; i++) {
      const a = off[i], b = off[(i + 1) % n];
      const d2 = distToSeg(near.x, near.y, a[0], a[1], b[0], b[1]);
      if (d2 < bestD) {
        bestD = d2; bestEdge = i;
        const dx = b[0] - a[0], dy = b[1] - a[1];
        const len2 = dx * dx + dy * dy || 1;
        bestT = Math.max(0, Math.min(1, ((near.x - a[0]) * dx + (near.y - a[1]) * dy) / len2));
        bestS = acc + bestT * edgeLen[i];
      }
      acc += edgeLen[i];
    }
    const maxArc = Math.min(perim, arcBudget);
    for (let s = 0; s <= maxArc + 1e-9; s += step) {
      const sAbs = (bestS + s) % perim;
      // locate edge for sAbs
      let walked = 0, ei = 0;
      for (ei = 0; ei < n; ei++) {
        if (walked + edgeLen[ei] >= sAbs || ei === n - 1) break;
        walked += edgeLen[ei];
      }
      const t = edgeLen[ei] > 0 ? (sAbs - walked) / edgeLen[ei] : 0;
      const a = off[ei], b = off[(ei + 1) % n];
      pts.push({ x: a[0] + (b[0] - a[0]) * t, y: a[1] + (b[1] - a[1]) * t, param: s });
    }
    return pts;
  }

  global.RMP = global.RMP || {};
  global.RMP.makeWorld = makeWorld;
  global.RMP.makeBugWorld = makeBugWorld;
  global.RMP.makeNarrowPassageWorld = makeNarrowPassageWorld;
  // Exposed so page-local world generators (e.g. potentialfield.js's deliberate
  // horseshoe-trap layout) can build a fully-featured world object (isFree,
  // draw, distToNearestObstacle, ...) from a hand-placed obstacle/start/goal
  // set without duplicating this bookkeeping.
  global.RMP.finishWorld = finishWorld;
  global.RMP.geom = { pointInPoly, distToSeg, distToPoly, traceObstacleBoundary, segmentIntersectsPoly };
})(window);
