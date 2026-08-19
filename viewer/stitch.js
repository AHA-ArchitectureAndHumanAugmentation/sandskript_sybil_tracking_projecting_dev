/* Multi-Cam Vision prototype UI (contained from viewer.js).

   Two stacked areas, separated by a drag bar:

   · TOP — the result. The combined canvas, look-only. The selected camera's
     footprint is outlined faintly so you can tell which picture is which;
     nothing up here is draggable.
   · BOTTOM — the workbench, one panel per camera. Everything is done here:
       green corner handles 1-4  → shape where that camera lands on the canvas
                                   (the keystone fix for a tilted mount)
       drag inside the green     → move that camera across the canvas
       blue edge bars            → trim the picture
     plus Turn (⟲ ⟳) and Move (◀ ▶) in the sidebar, which are how the row is
     made to match the physical rig before any fine dragging.

   The green outline in a panel is that camera's canvas quad drawn at the
   panel's own scale, so an unskewed camera shows a plain rectangle sitting on
   its crop rectangle and a keystoned one visibly leans. Corner order is
   TL, TR, BL, BR = handles 1-4, matching viewer/projection.html. */
"use strict";

const $ = (id) => document.getElementById(id);
const SVG_NS = "http://www.w3.org/2000/svg";
const STRIP_KEY = "stitch.stripHeight";
const EDGE_PX = 9;             // thickness of a blue trim bar, in panel pixels

let ws = null;
let cams = [];            // placements, index-aligned with the connected cameras
let info = null;          // latest state.info (camera list, quads, canvas size)
let selected = 0;
let selHandle = -1;       // which corner the arrow keys nudge
let colour = false;
let dragging = false;     // freeze redraws so a gesture is not yanked away
let holdUntil = 0;        // ignore server echoes right after a local edit
let stripBuilt = -1;

function send(type, params) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type, params: params || {} }));
  }
}

function debounce(fn, ms) {
  let t = null;
  return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); };
}

function clamp(v, lo, hi) { return Math.min(Math.max(v, lo), hi); }

/* ── outgoing edits ──
   Applied locally first so dragging feels immediate, then coalesced per camera
   and sent. The server's echo is ignored for a moment afterwards: drags are
   RELATIVE, so a stale echo would not just flicker the outline, it would make
   the next delta start from the wrong corner. */
const pending = new Map();
const flushEdits = debounce(() => {
  for (const [index, changes] of pending) send("set_camera", { index, ...changes });
  pending.clear();
  holdUntil = Date.now() + 700;      // measured from the send, not the keystroke
}, 90);

function editCamera(index, changes) {
  const cam = cams[index];
  if (!cam) return;
  Object.assign(cam, changes);
  holdUntil = Date.now() + 700;
  pending.set(index, { ...(pending.get(index) || {}), ...changes });
  flushEdits();
}

/* Buttons the server owns outright — no local echo to protect. These move the
   quads wholesale (a turn, a swap, a reset), so the panels re-fit around
   whatever comes back rather than keeping a mapping built for the old one. */
function command(type, params) {
  holdUntil = 0;
  panelMaps.clear();
  send(type, { index: selected, ...(params || {}) });
}

/* ── camera list ── */
function cameraLabel(index) {
  const c = ((info && info.cameras) || []).find((x) => x.index === index);
  return { name: `Cam ${index + 1}`, serial: (c && c.serial) || "" };
}

function selectCamera(index) {
  if (!cams[index] || index === selected) return;
  selected = index;
  selHandle = -1;
  renderSidebar();
  redraw();
}

function renderSidebar() {
  const host = $("camera-list");
  host.innerHTML = "";
  cams.forEach((cam, i) => {
    const { name, serial } = cameraLabel(i);
    const on = cam.enabled !== false;
    const row = document.createElement("div");
    row.className = "cam-row" + (i === selected ? " sel" : "") + (on ? "" : " off");
    row.title = serial;
    const label = document.createElement("span");
    label.className = "name";
    label.textContent = name;
    const eye = document.createElement("button");
    eye.className = "eye";
    eye.textContent = on ? "👁" : "🚫";
    eye.title = on ? "Leave this camera out of the canvas" : "Bring this camera back";
    eye.addEventListener("click", (ev) => {
      ev.stopPropagation();
      editCamera(i, { enabled: !on });
      renderSidebar();
    });
    row.append(label, eye);
    row.addEventListener("click", () => selectCamera(i));
    host.appendChild(row);
  });
  const cam = cams[selected];
  const h = cam ? Math.round(cam.height_mm || 0) : 0;
  $("height-readout").textContent = `${h > 0 ? "+" : ""}${h} mm`;
  for (const id of ["btn-ccw", "btn-cw", "btn-lower", "btn-raise",
                    "btn-reset-corners", "btn-reset-cam"]) {
    $(id).disabled = !cam;
  }
  // Swapping places needs someone to swap with.
  for (const id of ["btn-left", "btn-right"]) $(id).disabled = cams.length < 2;
}

$("btn-ccw").addEventListener("click", () => command("rotate_camera", { steps: -1 }));
$("btn-cw").addEventListener("click", () => command("rotate_camera", { steps: 1 }));
$("btn-left").addEventListener("click", () => command("move_camera", { steps: -1 }));
$("btn-right").addEventListener("click", () => command("move_camera", { steps: 1 }));
$("btn-lower").addEventListener("click", () => command("nudge_height", { steps: -1 }));
$("btn-raise").addEventListener("click", () => command("nudge_height", { steps: 1 }));
$("btn-reset-corners").addEventListener("click", () =>
  command("reset_camera", { corners_only: true }));
$("btn-reset-cam").addEventListener("click", () => command("reset_camera"));
$("btn-save").addEventListener("click", () => send("save_calib"));
/* Discard is the one control that throws away work, so it arms on the first
   click and fires on the second — no modal (nothing else in this tool uses
   one, and a blocked dialog would leave the button dead). */
let revertArmed = null;
$("btn-revert").addEventListener("click", () => {
  const btn = $("btn-revert");
  if (revertArmed) {
    clearTimeout(revertArmed);
    revertArmed = null;
    btn.textContent = "Discard changes";
    // command() (not send()) so the panels re-fit onto the corners that come
    // back — a revert moves every quad, exactly like a reset does.
    command("revert_calib");
    return;
  }
  btn.textContent = "Click again to discard";
  revertArmed = setTimeout(() => {
    revertArmed = null;
    btn.textContent = "Discard changes";
  }, 4000);
});
$("btn-colour").addEventListener("click", () => {
  colour = !colour;
  applyColour();
  send("set_colour", { on: colour });
});

function applyColour() {
  $("btn-colour").textContent = colour ? "Showing colour" : "Showing depth";
  $("result-lbl").textContent = colour ? "Result — combined colour" : "Result — combined view";
}

/* ── the drag bar between result and workbench ── */
function setStripHeight(px) {
  const stage = document.querySelector(".stage").clientHeight;
  const h = clamp(px, 90, Math.max(120, stage - 120));
  $("strip").style.height = `${h}px`;
  try { localStorage.setItem(STRIP_KEY, String(Math.round(h))); } catch (_) {}
  redraw();
}

$("splitter").addEventListener("pointerdown", (ev) => {
  ev.preventDefault();
  $("splitter").setPointerCapture(ev.pointerId);
  $("splitter").classList.add("on");
  const start = ev.clientY;
  const from = $("strip").clientHeight;
  const move = (e) => setStripHeight(from - (e.clientY - start));
  const up = (e) => {
    try { $("splitter").releasePointerCapture(e.pointerId); } catch (_) {}
    $("splitter").classList.remove("on");
    $("splitter").removeEventListener("pointermove", move);
    $("splitter").removeEventListener("pointerup", up);
    $("splitter").removeEventListener("pointercancel", up);
  };
  $("splitter").addEventListener("pointermove", move);
  $("splitter").addEventListener("pointerup", up);
  $("splitter").addEventListener("pointercancel", up);
});

(function restoreStripHeight() {
  let saved = null;
  try { saved = localStorage.getItem(STRIP_KEY); } catch (_) {}
  if (saved) $("strip").style.height = `${clamp(parseInt(saved, 10) || 300, 90, 2000)}px`;
})();

/* ── overlay geometry ──
   The <img> is object-fit: contain, so the picture occupies a letterboxed box
   inside its panel. Working in that box (with the image's own pixel size as
   the coordinate system) keeps one uniform scale in both axes. */
function contentBox(img) {
  const cw = img.clientWidth, ch = img.clientHeight;
  const nw = img.naturalWidth, nh = img.naturalHeight;
  if (!nw || !nh || !cw || !ch) return null;
  const s = Math.min(cw / nw, ch / nh);
  return { left: img.offsetLeft + (cw - nw * s) / 2,
           top: img.offsetTop + (ch - nh * s) / 2,
           width: nw * s, height: nh * s, nw, nh, scale: s };
}

function fitSvg(img, svg) {
  const b = contentBox(img);
  if (!b) { svg.style.display = "none"; return null; }
  svg.style.display = "";
  svg.style.left = `${b.left}px`;
  svg.style.top = `${b.top}px`;
  svg.setAttribute("width", b.width);
  svg.setAttribute("height", b.height);
  svg.setAttribute("viewBox", `0 0 ${b.nw} ${b.nh}`);
  return b;
}

function svgEl(tag, attrs) {
  const el = document.createElementNS(SVG_NS, tag);
  for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
  return el;
}

/* Pointer drag reporting deltas in IMAGE pixels. */
function onDrag(el, scale, handler) {
  el.addEventListener("pointerdown", (ev) => {
    ev.preventDefault();
    ev.stopPropagation();
    el.setPointerCapture(ev.pointerId);
    dragging = true;
    let last = { x: ev.clientX, y: ev.clientY };
    const move = (e) => {
      handler((e.clientX - last.x) / scale, (e.clientY - last.y) / scale);
      last = { x: e.clientX, y: e.clientY };
    };
    const up = (e) => {
      dragging = false;
      try { el.releasePointerCapture(e.pointerId); } catch (_) {}
      el.removeEventListener("pointermove", move);
      el.removeEventListener("pointerup", up);
      el.removeEventListener("pointercancel", up);
      redraw();
    };
    el.addEventListener("pointermove", move);
    el.addEventListener("pointerup", up);
    el.addEventListener("pointercancel", up);
  });
}

/* ── top: the result, look-only ── */
function drawResult() {
  const img = $("result-img");
  const svg = $("result-marks");
  const box = fitSvg(img, svg);
  svg.innerHTML = "";
  if (!box || !info || !info.cameras) return;
  const cam = info.cameras.find((c) => c.index === selected);
  if (!cam || !cam.quad_px) return;
  const q = cam.quad_px;
  // Walked around the perimeter: TL, TR, BR, BL.
  svg.appendChild(svgEl("polygon", {
    points: [q[0], q[1], q[3], q[2]].map((p) => p.join(",")).join(" ") }));
}

/* ── bottom: the workbench ── */
const handles = [0, 1, 2, 3].map((k) => {
  const el = document.createElement("div");
  el.className = "handle";
  el.textContent = String(k + 1);
  el.style.display = "none";
  document.body.appendChild(el);
  return el;
});

function buildStrip(count) {
  if (stripBuilt === count) return;
  stripBuilt = count;
  panelMaps.clear();          // panel indices are about to mean something else
  const host = $("strip");
  host.innerHTML = "";
  for (let i = 0; i < count; i++) {
    const vp = document.createElement("div");
    vp.className = "vp";
    vp.dataset.index = String(i);
    const img = document.createElement("img");
    img.alt = "";
    img.dataset.src = `/cam/${i}`;
    const svg = document.createElementNS(SVG_NS, "svg");
    const lbl = document.createElement("span");
    lbl.className = "lbl";
    vp.append(img, svg, lbl);
    vp.addEventListener("pointerdown", () => selectCamera(i));
    host.appendChild(vp);
  }
  if (ws && ws.readyState === WebSocket.OPEN) claimStreams();
}

function normalizeCrop(c) {
  let [x, y, w, h] = c.map((v) => (Number.isFinite(v) ? v : 0));
  x = clamp(x, 0, 0.9); y = clamp(y, 0, 0.9);
  w = clamp(w, 0.1, 1 - x); h = clamp(h, 0.1, 1 - y);
  return [x, y, w, h];
}

/* ── panel ⇄ canvas mapping ──
   A quad lives in canvas millimetres; a panel draws it in image pixels. The
   mapping is CACHED per camera and deliberately does NOT depend on the quad:
   deriving the scale and centre from the corners each repaint is what made
   dragging one handle rescale and re-centre the other three. It is re-fitted
   only when something OTHER than the corners changes — a turn, a trim, a
   resize, a reset — and never mid-gesture. At each fit an unskewed quad lands
   exactly on the crop rectangle, so the cropped region normally sits inside
   the four handles, and pulling one handle out is visibly just that. */
const panelMaps = new Map();      // camera index → {pxPerMm, ox, oy, key}

/* Deliberately NOT keyed on the crop: trimming does not move the sand (the
   server re-cuts the quad through the old pin), so the outline should sit
   still while the blue rectangle shrinks. The re-fit comes a moment later,
   from `applyCalib`, once the re-cut corners have actually arrived. */
function mapKey(cam, box) {
  return `${cam.rot_deg}|${box.nw}x${box.nh}`;
}

function fitMap(cam, box) {
  const q = cam.quad_mm;
  if (!q || q.length !== 4) return null;
  const xs = q.map((p) => p[0]), ys = q.map((p) => p[1]);
  const qw = Math.max(...xs) - Math.min(...xs);
  const qh = Math.max(...ys) - Math.min(...ys);
  const [cx, cy, cw, ch] = cam.crop || [0, 0, 1, 1];
  if (qw < 1e-6 || qh < 1e-6 || cw < 1e-6 || ch < 1e-6) return null;
  // ONE scale for both axes, so a quad is never drawn distorted.
  const pxPerMm = Math.min((cw * box.nw) / qw, (ch * box.nh) / qh);
  const mx = (Math.min(...xs) + Math.max(...xs)) / 2;
  const my = (Math.min(...ys) + Math.max(...ys)) / 2;
  return { pxPerMm,
           ox: (cx + cw / 2) * box.nw - mx * pxPerMm,
           oy: (cy + ch / 2) * box.nh - my * pxPerMm };
}

function panelPts(cam, m) {
  const q = cam.quad_mm;
  if (!m || !q || q.length !== 4) return null;
  return q.map((p) => [m.ox + p[0] * m.pxPerMm, m.oy + p[1] * m.pxPerMm]);
}

/* Sliding a camera far enough across the canvas would walk its outline off
   the panel and leave nothing to grab. */
function driftedOut(cam, m, box) {
  const pts = panelPts(cam, m);
  if (!pts) return false;
  const cx = (pts[0][0] + pts[1][0] + pts[2][0] + pts[3][0]) / 4;
  const cy = (pts[0][1] + pts[1][1] + pts[2][1] + pts[3][1]) / 4;
  return cx < 0 || cy < 0 || cx > box.nw || cy > box.nh;
}

function panelMap(index, cam, box) {
  let m = panelMaps.get(index);
  if (dragging && m) return m;
  if (m && m.key === mapKey(cam, box) && !driftedOut(cam, m, box)) return m;
  m = fitMap(cam, box);
  if (!m) return null;
  m.key = mapKey(cam, box);
  panelMaps.set(index, m);
  return m;
}

/* Corners move in canvas millimetres; the drag arrives in panel pixels. */
function shiftCorners(which, dxPx, dyPx, pxPerMm) {
  const cam = cams[selected];
  if (!cam || !cam.quad_mm || cam.quad_mm.length !== 4 || !pxPerMm) return;
  const dx = dxPx / pxPerMm, dy = dyPx / pxPerMm;
  editCamera(selected, {
    quad_mm: cam.quad_mm.map((p, k) =>
      (which < 0 || which === k) ? [p[0] + dx, p[1] + dy] : [p[0], p[1]]),
  });
}

let nudgeScale = 0;     // panel px per mm for the selected camera (arrow keys)

function drawPanel(vp) {
  const index = Number(vp.dataset.index);
  const img = vp.querySelector("img");
  const svg = vp.querySelector("svg");
  const box = fitSvg(img, svg);
  const cam = cams[index];
  const sel = index === selected;
  const { name, serial } = cameraLabel(index);
  vp.classList.toggle("sel", sel);
  vp.classList.toggle("off", !!cam && cam.enabled === false);
  vp.querySelector(".lbl").textContent =
    name + (serial ? ` · ${serial}` : "") +
    (cam && cam.enabled === false ? " — off" : "");
  svg.innerHTML = "";
  if (!box || !cam) return;

  const cropRect = () => {
    const [x, y, w, h] = cam.crop || [0, 0, 1, 1];
    return { x: x * box.nw, y: y * box.nh, w: w * box.nw, h: h * box.nh };
  };
  const outline = svgEl("polygon", { class: "shape" });
  const crop = svgEl("rect", { class: "crop" });
  svg.append(outline, crop);
  const edges = ["h", "h", "v", "v"].map((axis) =>
    svg.appendChild(svgEl("rect", { class: `edge ${axis}` })));
  if (!sel) {
    for (const e of edges) e.style.display = "none";
  }

  // Repainted in place rather than rebuilt: replacing the DOM mid-gesture
  // would drop the pointer capture the drag is riding on.
  const repaint = () => {
    const r = cropRect();
    crop.setAttribute("x", r.x); crop.setAttribute("y", r.y);
    crop.setAttribute("width", r.w); crop.setAttribute("height", r.h);
    const pts = panelPts(cam, panelMap(index, cam, box));
    if (pts) {
      outline.setAttribute("points",
        [pts[0], pts[1], pts[3], pts[2]].map((p) => p.join(",")).join(" "));
    }
    const t = EDGE_PX / box.scale;
    const geo = [
      { x: r.x, y: r.y - t / 2, width: r.w, height: t },                 // top
      { x: r.x, y: r.y + r.h - t / 2, width: r.w, height: t },           // bottom
      { x: r.x - t / 2, y: r.y, width: t, height: r.h },                 // left
      { x: r.x + r.w - t / 2, y: r.y, width: t, height: r.h },           // right
    ];
    edges.forEach((e, k) => {
      for (const [key, v] of Object.entries(geo[k])) e.setAttribute(key, v);
    });
    if (sel && pts) placeHandles(vp, box, pts);
  };
  repaint();

  if (!sel) return;
  const m0 = panelMap(index, cam, box);
  nudgeScale = m0 ? m0.pxPerMm : 0;

  // Drag inside the green outline: slide this camera across the canvas.
  onDrag(outline, box.scale, (dx, dy) => {
    const m = panelMap(index, cam, box);
    shiftCorners(-1, dx, dy, m && m.pxPerMm);
    repaint();
  });

  // Blue edge bars trim the picture, one edge at a time.
  const trim = [
    (c, dx, dy) => [c[0], c[1] + dy, c[2], c[3] - dy],     // top
    (c, dx, dy) => [c[0], c[1], c[2], c[3] + dy],          // bottom
    (c, dx, dy) => [c[0] + dx, c[1], c[2] - dx, c[3]],     // left
    (c, dx, dy) => [c[0], c[1], c[2] + dx, c[3]],          // right
  ];
  edges.forEach((edge, k) => onDrag(edge, box.scale, (dx, dy) => {
    const c = cam.crop || [0, 0, 1, 1];
    editCamera(index, { crop: normalizeCrop(trim[k](c, dx / box.nw, dy / box.nh)) });
    repaint();
  }));

  // Corner handles: shape where this camera lands.
  handles.forEach((h, k) => {
    h.style.display = "";
    h.classList.toggle("sel", k === selHandle);
    if (h.dataset.wired === "1") return;
    h.dataset.wired = "1";
    h.addEventListener("pointerdown", () => {
      selHandle = k;
      handles.forEach((x, j) => x.classList.toggle("sel", j === k));
    });
    onDrag(h, 1, (dx, dy) => {
      const panel = $("strip").querySelector(".vp.sel");
      if (!panel) return;
      const b = contentBox(panel.querySelector("img"));
      const m = b && panelMap(selected, cams[selected], b);
      if (!m) return;
      shiftCorners(k, dx / b.scale, dy / b.scale, m.pxPerMm);
      drawPanel(panel);
    });
  });
}

/* Handles live on <body> so they are never clipped by a short panel; position
   them from the panel's box in page coordinates. */
function placeHandles(vp, box, pts) {
  const r = vp.getBoundingClientRect();
  handles.forEach((h, k) => {
    h.style.left = `${r.left + box.left + pts[k][0] * box.scale}px`;
    h.style.top = `${r.top + box.top + pts[k][1] * box.scale}px`;
  });
}

window.addEventListener("keydown", (ev) => {
  if (selHandle < 0 || !cams[selected] || !nudgeScale) return;
  const step = ev.shiftKey ? 10 : 1;
  let dx = 0, dy = 0;
  if (ev.key === "ArrowLeft") dx = -step;
  else if (ev.key === "ArrowRight") dx = step;
  else if (ev.key === "ArrowUp") dy = -step;
  else if (ev.key === "ArrowDown") dy = step;
  else return;
  ev.preventDefault();
  shiftCorners(selHandle, dx, dy, nudgeScale);
  redraw();
});

function redraw() {
  if (dragging) return;
  drawResult();
  const panels = [...$("strip").children];
  if (!panels.length || !cams.length) handles.forEach((h) => { h.style.display = "none"; });
  for (const vp of panels) drawPanel(vp);
}

// Otherwise redrawn from each `state` message (~4 Hz), which is also when a
// stream's first frame gives its <img> a natural size.
window.addEventListener("resize", redraw);
window.addEventListener("scroll", redraw, true);

/* ── streams ── */
function claimStreams() {
  // Claim MJPEG connections only once the WS is up (Chrome's per-host cap);
  // spread them across the localhost / 127.0.0.1 buckets.
  const alt = location.hostname === "127.0.0.1" ? "localhost" : "127.0.0.1";
  const imgs = [$("result-img"), ...$("strip").querySelectorAll("img")];
  imgs.forEach((img, i) => {
    if (img.src) return;
    const host = i < 2 ? location.host : `${alt}:${location.port}`;
    img.src = `http://${host}${img.dataset.src}`;
  });
}

/* ── incoming ── */
function quadsMatch(a, b) {
  if (!Array.isArray(a) || !Array.isArray(b) || a.length !== b.length) return false;
  return a.every((p, k) => Math.abs(p[0] - b[k][0]) < 1e-3 &&
                           Math.abs(p[1] - b[k][1]) < 1e-3);
}

function applyCalib(c) {
  if (!c || !Array.isArray(c.cams)) return;
  if (dragging || Date.now() < holdUntil) return;   // a local edit is in flight
  // Corners the SERVER moved — a trim re-cut, a saved layout, a reset — leave
  // the panel fitted to geometry that no longer exists, so drop the mapping and
  // let it re-fit onto the crop rectangle. A plain corner drag echoes back
  // identical, which is what keeps this from re-fitting under a live gesture.
  c.cams.forEach((p, i) => {
    if (cams[i] && !quadsMatch(cams[i].quad_mm, p.quad_mm)) panelMaps.delete(i);
  });
  cams = c.cams.map((p) => ({ ...p }));
  if (selected >= cams.length) selected = 0;
  renderSidebar();
}

function showMsg(text, isError) {
  const m = $("msg");
  m.textContent = text;
  m.classList.toggle("err", !!isError);
}

/* Saved vs. adjusted. The main app builds its combined view from the FILE, not
   from this window, so "you have changes" and "the app is still on the old
   layout" are the same statement — say it once, here. */
let calibFile = "stitch_calibration.json";
function showSaved(dirty) {
  $("btn-save").classList.toggle("dirty", !!dirty);
  $("btn-revert").disabled = !dirty;
  if (!dirty && revertArmed) {          // nothing left to discard — disarm
    clearTimeout(revertArmed);
    revertArmed = null;
    $("btn-revert").textContent = "Discard changes";
  }
  const el = $("saved-state");
  el.className = dirty ? "dirty" : "saved";
  el.textContent = dirty
    ? `Adjusted — not saved. The app still uses the layout in ${calibFile}.`
    : "Saved — this is what the app uses";
}

function handle(data) {
  if (data.type === "init") {
    colour = !!data.colour;
    applyColour();
    if (data.calib_file) calibFile = data.calib_file;
    applyCalib(data.calib);
    showSaved(data.dirty);
  } else if (data.type === "state") {
    applyCalib(data.calib);
    showSaved(data.dirty);
    info = data.info;
    if (info && info.count !== undefined) buildStrip(info.count);
    $("info").textContent = !info
      ? "waiting for frames…"
      : `${info.count} camera${info.count === 1 ? "" : "s"} · ` +
        `${info.size[0]}×${info.size[1]} px · ${info.mm_per_px} mm/px · ` +
        `overlap ${info.overlap_pct}%`;
    showSeams(info && info.seams);
    $("note").textContent = data.note || (info && info.synthetic ? "SYNTHETIC scene" : "");
    redraw();
  } else if (data.type === "save_result") {
    showMsg(data.message, !data.success);
  }
}

/* ── Seam agreement ────────────────────────────────────────────────────────
   Where two cameras see the same sand, stitch AVERAGES their depths — so a
   per-camera bias does not show as one step but as a plateau with a step at
   each edge of the overlap. A raked groove is ~1.5 mm deep, so even a few mm
   here is several times the signal, and detection will paint phantom grooves
   along the seam. The verdict is the useful part: "height" means one number is
   wrong and the Height ▼▲ buttons can cancel it; "tilt" means the gap changes
   across the seam, so no single offset will do. */
function showSeams(seams) {
  const el = $("seams");
  if (!el) return;
  if (!seams || !seams.length) {
    el.textContent = "";
    el.className = "";
    return;
  }
  const worst = seams.reduce((a, s) =>
    Math.abs(s.mean_mm) > Math.abs(a.mean_mm) ? s : a, seams[0]);
  el.className = worst.verdict === "aligned" ? "seam-ok"
               : worst.verdict === "height" ? "seam-height" : "seam-tilt";
  el.textContent = seams.map((s) =>
    `cam ${s.a + 1}↔${s.b + 1}: ${s.mean_mm >= 0 ? "+" : ""}${s.mean_mm.toFixed(1)} mm` +
    ` (±${s.std_mm.toFixed(1)}) — ${s.hint}`).join("\n");
}

/* ── connection ── */
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  // Watchdog: if the handshake hangs (e.g. queued behind other requests),
  // abort and retry instead of sitting in CONNECTING forever.
  const watchdog = setTimeout(() => {
    if (ws.readyState !== WebSocket.OPEN) ws.close();
  }, 4000);
  ws.onopen = () => {
    clearTimeout(watchdog);
    claimStreams();
  };
  ws.onmessage = (ev) => { try { handle(JSON.parse(ev.data)); } catch (_) {} };
  ws.onclose = () => {
    clearTimeout(watchdog);
    $("info").textContent = "disconnected — retrying…";
    setTimeout(connect, 1500);
  };
}
renderSidebar();
applyColour();
connect();
