/* ===========================================================
   viz-core.js — shared engine for interactive algorithm demos.

   Every demo module (bug1.js, prm.js, ...) calls:

     RMP.mountViz(containerEl, {
       title, badge, subtitle,
       width, height,               // canvas size in CSS px
       legend: [{color, label}],
       params: [{key,label,type:'range',min,max,step,value}],
       pythonCode: `...`,
       makeSim({rng, params, width, height}) {
         return {
           draw(ctx) { ... },              // render current state
           step() { ... ; return {done, note}; } // advance one atomic step
         };
       }
     });

   The core owns: canvas + DPI scaling, Step/Play/Pause/Reset/
   Generate-new buttons, a speed slider, a status line, a
   collapsible Python listing, and a seeded RNG so "Generate new"
   is reproducible-but-varied while Step/Play never re-randomize.
   =========================================================== */

(function (global) {
  "use strict";

  function mulberry32(seed) {
    let a = seed >>> 0;
    return function () {
      a |= 0; a = (a + 0x6D2B79F5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function el(tag, cls, html) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (html !== undefined) e.innerHTML = html;
    return e;
  }

  function mountViz(container, def) {
    const width = def.width || 560;
    const height = def.height || 360;
    const dpr = Math.max(1, Math.min(2, window.devicePixelRatio || 1));

    const root = el("div", "viz");

    // ---- head ----
    const head = el("div", "viz-head");
    head.appendChild(el("h4", null, def.title));
    if (def.badge) head.appendChild(el("span", "badge book", def.badge));
    root.appendChild(head);
    if (def.subtitle) {
      const sub = el("div", null, def.subtitle);
      sub.style.cssText = "padding:0 1.2rem;color:var(--text-dim);font-size:0.86rem;";
      root.appendChild(sub);
    }

    // ---- stage ----
    const stage = el("div", "viz-stage");
    const canvas = document.createElement("canvas");
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    canvas.style.width = width + "px";
    canvas.style.height = height + "px";
    const ctx = canvas.getContext("2d");
    ctx.scale(dpr, dpr);
    stage.appendChild(canvas);
    root.appendChild(stage);

    // ---- legend ----
    if (def.legend && def.legend.length) {
      const legend = el("div", "viz-legend");
      def.legend.forEach((item) => {
        const span = el("span", null, `<i style="background:${item.color}"></i>${item.label}`);
        legend.appendChild(span);
      });
      root.appendChild(legend);
    }

    // ---- params ----
    const paramValues = {};
    let paramsBar = null;
    if (def.params && def.params.length) {
      paramsBar = el("div", "viz-params");
      def.params.forEach((p) => {
        paramValues[p.key] = p.value;
        const label = el("label");
        const valSpan = el("span", null, String(p.value));
        label.appendChild(document.createTextNode(p.label + " "));
        const input = document.createElement("input");
        input.type = "range";
        input.min = p.min; input.max = p.max; input.step = p.step || 1;
        input.value = p.value;
        input.addEventListener("input", () => {
          paramValues[p.key] = parseFloat(input.value);
          valSpan.textContent = input.value;
          rebuild(currentSeed);
        });
        label.appendChild(input);
        label.appendChild(valSpan);
        paramsBar.appendChild(label);
      });
      root.appendChild(paramsBar);
    }

    // ---- controls ----
    const controls = el("div", "viz-controls");
    const btnStep = el("button", "viz-btn", "⏭ Step");
    const btnPlay = el("button", "viz-btn primary", "▶ Play");
    const btnReset = el("button", "viz-btn", "⟲ Reset");
    const btnGen = el("button", "viz-btn", "🎲 Generate new");
    controls.appendChild(btnStep);
    controls.appendChild(btnPlay);
    controls.appendChild(btnReset);
    controls.appendChild(btnGen);
    controls.appendChild(el("span", "viz-spacer"));
    const speedWrap = el("div", "viz-speed");
    speedWrap.appendChild(document.createTextNode("speed"));
    const speedInput = document.createElement("input");
    speedInput.type = "range"; speedInput.min = 1; speedInput.max = 10; speedInput.value = 5;
    speedWrap.appendChild(speedInput);
    controls.appendChild(speedWrap);
    root.appendChild(controls);

    // ---- status ----
    const status = el("div", "viz-status", "Press <strong>Step</strong> or <strong>Play</strong> to begin.");
    root.appendChild(status);

    // ---- python code ----
    if (def.pythonCode) {
      const details = el("details", "viz-code-toggle");
      const summary = el("summary", null, "Show Python implementation");
      details.appendChild(summary);
      const pre = el("pre");
      const code = el("code", "language-python");
      code.textContent = def.pythonCode.trim();
      pre.appendChild(code);
      details.appendChild(pre);
      root.appendChild(details);
    }

    container.appendChild(root);

    // ---- simulation lifecycle ----
    let sim = null;
    let playing = false;
    let timer = null;
    let currentSeed = (Math.random() * 1e9) | 0;
    let stepCount = 0;
    let done = false;

    function rng() { return mulberry32(currentSeed); }

    function rebuild(seed) {
      currentSeed = seed;
      stepCount = 0;
      done = false;
      playing = false;
      clearInterval(timer);
      btnPlay.textContent = "▶ Play";
      sim = def.makeSim({ rng: rng(), params: paramValues, width, height });
      redraw();
      setStatus("Press <strong>Step</strong> or <strong>Play</strong> to begin.");
      setButtons();
    }

    function redraw() {
      ctx.clearRect(0, 0, width, height);
      sim.draw(ctx);
    }

    function setStatus(html) { status.innerHTML = html; }

    function setButtons() {
      btnStep.disabled = done;
      btnPlay.disabled = done;
    }

    function doStep() {
      if (done || !sim) return;
      const res = sim.step() || {};
      stepCount++;
      redraw();
      const prefix = `<strong>Step ${stepCount}.</strong> `;
      setStatus(prefix + (res.note || ""));
      if (res.done) {
        done = true;
        playing = false;
        clearInterval(timer);
        btnPlay.textContent = "▶ Play";
        setStatus(prefix + (res.note ? res.note + " " : "") + "<em>Finished.</em>");
      }
      setButtons();
    }

    btnStep.addEventListener("click", doStep);

    btnPlay.addEventListener("click", () => {
      if (done) return;
      playing = !playing;
      btnPlay.textContent = playing ? "⏸ Pause" : "▶ Play";
      if (playing) {
        const ms = 620 - parseInt(speedInput.value, 10) * 55;
        timer = setInterval(() => {
          doStep();
          if (done) { clearInterval(timer); }
        }, Math.max(30, ms));
      } else {
        clearInterval(timer);
      }
    });

    speedInput.addEventListener("input", () => {
      if (playing) {
        clearInterval(timer);
        const ms = 620 - parseInt(speedInput.value, 10) * 55;
        timer = setInterval(() => {
          doStep();
          if (done) { clearInterval(timer); }
        }, Math.max(30, ms));
      }
    });

    btnReset.addEventListener("click", () => rebuild(currentSeed));
    btnGen.addEventListener("click", () => rebuild((Math.random() * 1e9) | 0));

    rebuild(currentSeed);

    return { rebuild, canvas, ctx };
  }

  global.RMP = global.RMP || {};
  global.RMP.mountViz = mountViz;
  global.RMP.mulberry32 = mulberry32;
})(window);
