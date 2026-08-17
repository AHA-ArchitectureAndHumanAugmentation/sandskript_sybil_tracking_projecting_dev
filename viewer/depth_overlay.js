/* Participant-Mode popup logic (loaded by depth_view.html, the ⧉ Participant
   Mode window opened from the Depth viewport in Developer Mode).

   Required ids: #stage-wrap #stage #feed #overlay
                 #interval #interval-val #textsize #textsize-val
                 #auto-toggle #trigger #maxtime #status-chip #status-msg #countdown

   What it does:
   - keeps a stage fitted to the window in the CROP's aspect ratio, drawing the
     server's depth-number labels ([[u, v, mm], ...], coords relative to the
     Developer-Mode crop — the same region the /depth/cropped feed shows; the
     crop size arrives with each depth_labels message) over the live feed;
   - Region-interval slider → `depth_overlay_params`; Text-size is client-side;
   - Auto toggle → `set_automation` — ON arms the automated pipeline and locks
     the manual Capture/Generate/Run buttons in Developer Mode;
   - Trigger box (mm) → `set_trigger` (empty = none) — the distance that arms
     the trigger; both are server-side state shared by every open window;
   - Max drawing time box (min) → `set_max_draw_time` (empty = no limit) — how
     long one participant may draw before the drawing is refused;
   - shows the automation status (Auto Off/Auto On/Alerted/Sensing/Generating
     Paths/Actuating) big in the stage's top-right corner, from `state`, and
     the drawing-time countdown in the top-left corner. Both numbers come from
     the server — the countdown is never run client-side, so what the
     participant reads is the clock that actually judges the drawing. */

(function () {
  let srcW = 640, srcH = 480;   // size of the cropped region (updates with labels)
  const stage   = document.getElementById("stage");
  const canvas  = document.getElementById("overlay");
  const ctx     = canvas.getContext("2d");
  let labels = [];
  // True when the numbers (and the trigger box) are HEIGHTS ABOVE THE SAND,
  // which is what a reference frame buys: on a tilted camera the raw distance
  // from the camera varies across the box by more than a hand's clearance, so
  // no single absolute cutoff can separate the two. Set from the server.
  let labelsRelative = false;

  /* ── Layout: keep a stage in the crop's aspect that fits the window ─────── */
  function layout() {
    const wrap = document.getElementById("stage-wrap");
    const aw = wrap.clientWidth, ah = wrap.clientHeight;
    let w = aw, h = aw * (srcH / srcW);
    if (h > ah) { h = ah; w = ah * (srcW / srcH); }
    stage.style.width = w + "px";
    stage.style.height = h + "px";
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    draw();
  }
  window.addEventListener("resize", layout);

  /* ── Depth-number drawing ───────────────────────────────────────────────── */
  function draw() {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    if (!labels.length) return;
    const sx = w / srcW, sy = h / srcH;
    const px = parseFloat(document.getElementById("textsize").value) * dpr;
    ctx.font = `${px}px "SF Mono", "Fira Code", monospace`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.lineWidth = Math.max(2, px / 6);
    ctx.strokeStyle = "rgba(0,0,0,0.85)";
    ctx.fillStyle = "#ffffff";
    for (const [u, v, mm] of labels) {
      // Height above the sand is a SIGNED number and 0 is the surface itself,
      // so the sign carries the meaning: +90 = a hand, -6 = a raked groove.
      // An absolute distance from the camera is never signed, so no "+" there.
      const n = Math.round(mm);
      const t = labelsRelative && n > 0 ? "+" + n : String(n);
      const x = u * sx, y = v * sy;
      ctx.strokeText(t, x, y);   // dark outline keeps numbers readable on any colour
      ctx.fillText(t, x, y);
    }
  }

  /* ── Sliders ────────────────────────────────────────────────────────────── */
  const intervalEl = document.getElementById("interval");
  const textEl     = document.getElementById("textsize");

  let sendTimer = null;
  intervalEl.addEventListener("input", () => {
    document.getElementById("interval-val").textContent = intervalEl.value + " mm";
    if (sendTimer) clearTimeout(sendTimer);
    sendTimer = setTimeout(sendParams, 150);
  });
  textEl.addEventListener("input", () => {
    document.getElementById("textsize-val").textContent = textEl.value + " px";
    draw();                       // text size is client-side only
  });

  function sendParams() {
    send({ type: "depth_overlay_params",
           params: { interval_mm: parseFloat(intervalEl.value) } });
  }

  /* ── Trigger box (Participant-Mode automation threshold) ────────────────── */
  const triggerEl = document.getElementById("trigger");
  let trigTimer = null;
  triggerEl.addEventListener("input", () => {
    if (trigTimer) clearTimeout(trigTimer);
    trigTimer = setTimeout(() => {
      const v = parseFloat(triggerEl.value);
      send({ type: "set_trigger",
             params: { threshold_mm: Number.isFinite(v) && v > 0 ? v : null } });
    }, 400);
  });

  /* The trigger box and the overlay numbers measure the same quantity, and
     which quantity it is depends on whether a reference frame is set. Say so
     in both places rather than leaving the operator to infer it from the
     magnitudes — the two modes want values an order of magnitude apart. */
  const trigLabelEl = document.querySelector('label[for="trigger"]');
  const noteEl      = document.getElementById("note");
  function setMeasurementMode(relative) {
    if (relative === labelsRelative && trigLabelEl.dataset.set) return;
    labelsRelative = !!relative;
    trigLabelEl.dataset.set = "1";
    trigLabelEl.textContent = labelsRelative ? "Trigger above sand" : "Trigger below";
    trigLabelEl.title = labelsRelative
      ? "Fires when something rises more than this many mm ABOVE the sand surface (the reference frame). Unaffected by camera tilt — the same number means the same thing everywhere in the box."
      : "Fires when something comes closer to the camera than this many mm. No reference frame is set, so this is a raw distance: on a tilted camera the sand's own depth range may leave no value that works. Press Set Reference in Developer Mode to switch to height above the sand.";
    noteEl.textContent = (labelsRelative
      ? "numbers = mm above sand (0 = surface, − = groove)"
      : "numbers = mm from camera") + " · Auto ON + trigger = automated runs";
    draw();
  }

  function syncTrigger(mm) {
    // Adopt the server's threshold (set here or in another window) — but never
    // fight the user while they are typing in this box.
    if (document.activeElement === triggerEl) return;
    const cur = parseFloat(triggerEl.value);
    if (mm == null && triggerEl.value !== "") triggerEl.value = "";
    else if (mm != null && cur !== mm) triggerEl.value = mm;
  }

  /* ── Max drawing time box + countdown ───────────────────────────────────── */
  const maxTimeEl = document.getElementById("maxtime");
  const cdEl      = document.getElementById("countdown");
  let maxTimer = null;
  maxTimeEl.addEventListener("input", () => {
    if (maxTimer) clearTimeout(maxTimer);
    maxTimer = setTimeout(() => {
      const v = parseFloat(maxTimeEl.value);
      send({ type: "set_max_draw_time",
             params: { minutes: Number.isFinite(v) && v > 0 ? v : null } });
    }, 400);
  });

  function syncMaxTime(min) {
    if (document.activeElement === maxTimeEl) return;   // never fight the typist
    const cur = parseFloat(maxTimeEl.value);
    if (min == null && maxTimeEl.value !== "") maxTimeEl.value = "";
    else if (min != null && cur !== min) maxTimeEl.value = min;
  }

  function mmss(seconds) {
    const t = Math.max(0, Math.round(seconds));
    return Math.floor(t / 60) + ":" + String(t % 60).padStart(2, "0");
  }

  function updateCountdown(p) {
    // No limit set, or automation off → no clock to show.
    if (p.max_draw_min == null || p.remaining_s == null || !p.auto) {
      cdEl.className = "hidden";
      return;
    }
    const counting = p.status === "Alerted";     // only ticks while a hand is in
    const warn = counting && p.remaining_s <= (p.warn_s || 10);
    cdEl.textContent = mmss(p.remaining_s);
    cdEl.className = warn ? "warn" : (counting ? "counting" : "");
  }

  /* ── Auto toggle ────────────────────────────────────────────────────────── */
  const autoEl = document.getElementById("auto-toggle");
  let autoOn = false;
  autoEl.addEventListener("click", () => {
    send({ type: "set_automation", params: { on: !autoOn } });
    // The button reflects the server's answer (next state tick), not the click.
  });

  function syncAuto(on) {
    autoOn = !!on;
    autoEl.textContent = autoOn ? "Auto: ON" : "Auto: OFF";
    autoEl.classList.toggle("on", autoOn);
  }

  /* ── Status chip (top-right, big) ───────────────────────────────────────── */
  const chipEl = document.getElementById("status-chip");
  const msgEl  = document.getElementById("status-msg");
  const CHIP_CLASS = {
    "Auto Off": "off", "Auto On": "watching", "Alerted": "alerted",
    "Sensing": "sensing", "Generating Paths": "generating", "Actuating": "actuating",
    // Sticky verdict from the profanity guard — nothing was saved or run.
    "Invalid": "invalid",
  };

  function updateParticipant(p) {
    if (!p) return;
    chipEl.textContent = p.status || "Auto Off";
    chipEl.className = "chip-" + (CHIP_CLASS[p.status] || "off");
    msgEl.textContent = p.message || "";
    syncAuto(p.auto);
    syncTrigger(p.trigger_mm);
    syncMaxTime(p.max_draw_min);
    updateCountdown(p);
  }

  /* ── WebSocket: register as an overlay client; receive labels + state ───── */
  let ws = null;
  function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
  }
  function connectWS() {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    ws = new WebSocket(`${proto}//${location.host}/ws`);
    ws.onopen = () => {
      send({ type: "depth_overlay_hello" });
      sendParams();               // push the current interval on (re)connect
    };
    ws.onclose = () => setTimeout(connectWS, 2000);
    ws.onmessage = (ev) => {
      const data = JSON.parse(ev.data);
      if (data.type === "depth_labels") {
        labels = data.labels || [];
        setMeasurementMode(data.relative);
        // The labels' (and the feed's) crop size — re-fit the stage when the
        // user adjusts the crop in Developer Mode.
        const s = data.size;
        if (s && s[0] > 0 && s[1] > 0 && (s[0] !== srcW || s[1] !== srcH)) {
          srcW = s[0]; srcH = s[1];
          layout();               // layout() calls draw()
        } else {
          draw();
        }
      } else if (data.type === "state" || data.type === "init") {
        updateParticipant(data.participant);
        // Arrives at 20 Hz and from the very first message, so the box is
        // labelled correctly before any depth labels have been computed (they
        // only run while this popup is connected, and only ~4 Hz).
        if (typeof data.reference_set === "boolean")
          setMeasurementMode(data.reference_set);
      }
    };
  }

  connectWS();
  layout();
})();
