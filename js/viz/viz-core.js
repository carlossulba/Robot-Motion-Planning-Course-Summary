/* ===========================================================
   viz-core.js — shared engine for interactive algorithm demos.

   Every demo module (bug1.js, prm.js, ...) calls:

     RMP.mountViz(containerEl, {
       title, badge, subtitle,
       width, height,               // canvas size in CSS px
       legend: [{color, label}],
       params: [{key,label,type:'range'|'checkbox'|'select',min,max,step,value,
                  scale:'log' (optional, range only -- huge multiplicative
                  spans without a huge linear slider), format (optional,
                  range only -- (value)=>string for the readout),
                  options:[{value,label}] (select only)}],
       pseudocode: ["line one", {text:"nested line", indent:1}, ...],
       sidePanel: {title} (optional) -- like pseudocode's panel, but its body
                  is set from sim.sidePanelHTML (a string) on every (re)build,
                  for content that depends on live param values instead of a
                  fixed line list (e.g. a formula readout).
       world: sharedWorldObject,    // optional: use this pre-built world
                                     // instead of generating one internally
                                     // (see world.js makeWorld/makeBugWorld).
                                     // When supplied, this panel's own
                                     // "Generate new" button is hidden --
                                     // the page that owns the shared
                                     // instance drives it instead via the
                                     // returned handle's setWorld(world).
       pythonCode: `...`,
       makeSim({rng, params, width, height, world}) {
         return {
           draw(ctx) { ... },              // render current state
           step() { ... ; return {done, note, line}; } // advance one atomic
                                            // step; `line` (optional) is the
                                            // 0-based index into `pseudocode`
                                            // currently executing, highlighted
                                            // in the pseudocode panel.
         };
       }
     });

   mountViz returns { rebuild, canvas, ctx, setWorld } -- a page that owns a
   shared world instance across several panels calls setWorld(newWorld) on
   every panel's handle to re-seed and re-render them all in lockstep.

   The core owns: canvas + DPI scaling, an optional pseudocode side panel
   with live line-highlighting, Step/Play/Pause/Reset/Generate-new buttons,
   a speed slider, a status line, a collapsible Python listing, and a seeded
   RNG so "Generate new" is reproducible-but-varied while Step/Play never
   re-randomize.
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

    // ---- stage + pseudocode panel ----
    // If def.pseudocode is supplied, the canvas and a pseudocode listing sit
    // side by side (flex-wrap), so the panel drops below the canvas on
    // narrow viewports for free.
    const body = el("div", "viz-body");
    const stage = el("div", "viz-stage");
    const canvas = document.createElement("canvas");
    canvas.width = width * dpr;
    canvas.height = height * dpr;
    canvas.style.width = width + "px";
    canvas.style.height = height + "px";
    const ctx = canvas.getContext("2d");
    ctx.scale(dpr, dpr);
    stage.appendChild(canvas);
    body.appendChild(stage);

    let pcListEl = null;
    if (def.pseudocode && def.pseudocode.length) {
      const pcPanel = el("div", "viz-pseudocode");
      pcPanel.appendChild(el("div", "viz-pseudocode-title", "Pseudocode"));
      pcListEl = el("ol", "viz-pseudocode-list");
      def.pseudocode.forEach((line) => {
        const text = typeof line === "string" ? line : line.text;
        const indent = typeof line === "object" && line.indent ? line.indent : 0;
        const li = el("li", "pc-line", null);
        li.textContent = text;
        if (indent) li.style.paddingLeft = (1.6 + indent * 1.1) + "em";
        pcListEl.appendChild(li);
      });
      pcPanel.appendChild(pcListEl);
      body.appendChild(pcPanel);
    }

    // Optional side panel with content that changes as params change (e.g. a
    // formula readout) rather than def.pseudocode's static line list -- opt-in
    // via def.sidePanel = {title}. Reuses the pseudocode panel's visual style.
    // makeSim's returned sim object may set `sidePanelHTML` (a string); it's
    // pushed into this panel every time the sim is (re)built, i.e. on every
    // param change, Reset, and Generate new.
    let sidePanelBodyEl = null;
    if (def.sidePanel) {
      const panel = el("div", "viz-pseudocode viz-formula-panel");
      panel.style.fontFamily = "var(--font-sans)";
      panel.appendChild(el("div", "viz-pseudocode-title", def.sidePanel.title || "Formula"));
      sidePanelBodyEl = el("div", "viz-formula-body");
      // This panel reuses .viz-pseudocode's always-dark code-block background
      // (var(--code-bg) is intentionally the same dark shade in both themes),
      // so its text must use the code-block foreground tokens, not the
      // page-theme-dependent var(--text)/var(--text-dim) -- those flip to a
      // dark color in light mode and would go dark-on-dark here.
      sidePanelBodyEl.style.cssText = "font-size:0.86rem;line-height:1.85;color:var(--code-text);";
      panel.appendChild(sidePanelBodyEl);
      body.appendChild(panel);
    }
    root.appendChild(body);

    function highlightLine(lineIdx) {
      if (!pcListEl) return;
      const items = pcListEl.children;
      for (let i = 0; i < items.length; i++) {
        items[i].classList.toggle("active", i === lineIdx);
      }
    }

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
        if (p.type === "checkbox") {
          label.appendChild(document.createTextNode(p.label + " "));
          const input = document.createElement("input");
          input.type = "checkbox";
          input.checked = !!p.value;
          input.addEventListener("change", () => {
            paramValues[p.key] = input.checked;
            rebuild(currentSeed);
          });
          label.appendChild(input);
        } else if (p.type === "select") {
          label.appendChild(document.createTextNode(p.label + " "));
          const select = document.createElement("select");
          select.className = "viz-select";
          select.style.cssText = "background:var(--bg-inset);color:var(--text);border:1px solid var(--border);border-radius:4px;padding:0.15rem 0.3rem;font:inherit;";
          (p.options || []).forEach((opt) => {
            const o = document.createElement("option");
            o.value = opt.value;
            o.textContent = opt.label;
            if (opt.value === p.value) o.selected = true;
            select.appendChild(o);
          });
          select.addEventListener("change", () => {
            paramValues[p.key] = select.value;
            rebuild(currentSeed);
          });
          label.appendChild(select);
        } else {
          // Opt-in log scale (p.scale === "log"): the underlying <input> travels
          // linearly over log10(value) so a huge multiplicative range (needed
          // when a parameter's real-world effect compresses non-linearly, e.g.
          // a repulsive gain whose influence shrinks with the cube root of its
          // value) still fits a normal-feeling drag, while the label always
          // shows the true value the sim receives, not the raw slider position.
          const isLog = p.scale === "log";
          const format = p.format || ((v) => (Math.abs(v) >= 1000 ? Math.round(v).toLocaleString() : (Math.round(v * 100) / 100).toString()));
          const valSpan = el("span", null, format(p.value));
          label.appendChild(document.createTextNode(p.label + " "));
          const input = document.createElement("input");
          input.type = "range";
          if (isLog) {
            input.min = Math.log10(p.min); input.max = Math.log10(p.max);
            input.step = p.step || (Math.log10(p.max) - Math.log10(p.min)) / 200;
            input.value = Math.log10(p.value);
          } else {
            input.min = p.min; input.max = p.max; input.step = p.step || 1;
            input.value = p.value;
          }
          input.addEventListener("input", () => {
            const raw = parseFloat(input.value);
            const val = isLog ? Math.pow(10, raw) : raw;
            paramValues[p.key] = val;
            valSpan.textContent = format(val);
            rebuild(currentSeed);
          });
          label.appendChild(input);
          label.appendChild(valSpan);
        }
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
    // In shared-world mode (def.world supplied), the page that mounted this
    // panel owns the single "Generate new" control for the whole page --
    // this panel's own button would generate a different instance than its
    // siblings, breaking the side-by-side comparison, so it's hidden.
    const sharedWorldMode = !!def.world;
    if (!sharedWorldMode) controls.appendChild(btnGen);
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
    let currentWorld = def.world || null;
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
      sim = def.makeSim({ rng: rng(), params: paramValues, width, height, world: currentWorld });
      if (sidePanelBodyEl) sidePanelBodyEl.innerHTML = sim.sidePanelHTML || "";
      highlightLine(-1);
      redraw();
      setStatus("Press <strong>Step</strong> or <strong>Play</strong> to begin.");
      setButtons();
    }

    // Called by a page-level "Generate new" control on shared-world pages:
    // swaps in a freshly generated world (same instance for every panel on
    // the page) and re-runs this panel's algorithm against it from scratch.
    function setWorld(newWorld) {
      currentWorld = newWorld;
      rebuild(currentSeed);
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
      highlightLine(typeof res.line === "number" ? res.line : -1);
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

    return { rebuild, canvas, ctx, setWorld };
  }

  global.RMP = global.RMP || {};
  global.RMP.mountViz = mountViz;
  global.RMP.mulberry32 = mulberry32;
})(window);
