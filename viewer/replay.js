/* Saved-toolpath replay UI (contained from viewer.js). Brand-neutral: all
   robot specifics live behind the server's ReplayBackend. */
"use strict";

const $ = (id) => document.getElementById(id);

let ws = null;
let selectedName = null;   // list item the user clicked
let loadedName = null;     // toolpath confirmed loaded by the server
let connected = false;
let executing = false;

function send(type, params) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type, params: params || {} }));
  }
}

/* ── robot ── */
$("btn-connect").addEventListener("click", () => {
  if (connected) {
    send("disconnect");
  } else {
    showRobotMsg("Connecting…", false);
    send("connect", { ip: $("robot-ip").value.trim() });
  }
});

/* ── toolpath list ── */
function renderList(items) {
  const ul = $("path-list");
  ul.textContent = "";
  for (const it of items) {
    const li = document.createElement("li");
    li.dataset.name = it.name;
    const label = document.createElement("span");
    label.textContent = it.name;
    const fmts = document.createElement("span");
    fmts.className = "fmts";
    // Each format is its own badge: click one to load that file explicitly
    // (clicking the row loads the default — json when present).
    const sources = [it.has_json ? "json" : null, it.has_script ? "script" : null]
      .filter(Boolean);
    sources.forEach((src, i) => {
      if (i) fmts.append(" · ");
      const b = document.createElement("a");
      b.textContent = src;
      b.title = `Load path.${src === "json" ? "json" : "script"}`;
      b.addEventListener("click", (ev) => {
        ev.stopPropagation();
        selectedName = it.name;
        highlightSelected();
        send("select", { name: it.name, source: src });
      });
      fmts.appendChild(b);
    });
    li.append(label, fmts);
    li.addEventListener("click", () => {
      selectedName = it.name;
      highlightSelected();
      send("select", { name: it.name });
    });
    ul.appendChild(li);
  }
  highlightSelected();
  if (!items.length) {
    const li = document.createElement("li");
    li.textContent = "No saved toolpaths yet — use Save Path in the main app.";
    li.style.cursor = "default";
    li.style.color = "#6d727c";
    ul.appendChild(li);
  }
}

function highlightSelected() {
  for (const li of $("path-list").children) {
    li.classList.toggle("sel", li.dataset.name === selectedName);
  }
}

$("btn-refresh").addEventListener("click", () => send("refresh"));

/* ── selected toolpath ── */
function fmt(v, digits) {
  return Number.isFinite(v) ? v.toFixed(digits) : "?";
}

function applyToolpath(tp) {
  if (!tp) return;
  if (!tp.success) { showMsg(tp.message, true); return; }
  loadedName = tp.name;
  selectedName = tp.name;
  highlightSelected();

  const img = $("preview");
  if (tp.has_preview) {
    img.src = `/preview/${encodeURIComponent(tp.name)}`;
    img.hidden = false;
    $("preview-empty").hidden = true;
  } else {
    img.hidden = true;
    $("preview-empty").hidden = false;
    $("preview-empty").textContent = "No preview image in this bundle.";
  }

  const m = tp.meta || {};
  const parts = [
    `${tp.name}  ·  loaded from ${tp.source}`,
    `${tp.stroke_count} strokes / ${tp.point_count} waypoints`,
  ];
  if (m.mode) parts.push(`mode ${m.mode}`);
  if (m.surface_name) parts.push(`surface ${m.surface_name}`);
  if (Number.isFinite(m.offset_mm)) parts.push(`saved offset ${fmt(m.offset_mm, 1)} mm`);
  $("details").textContent = parts.join("   ·   ");

  // Prefill the run controls from the file's own metadata.
  if (Number.isFinite(m.speed_pct)) $("run-speed").value = Math.round(m.speed_pct);
  if (Number.isFinite(m.safety_mm)) $("run-safety").value = Math.round(m.safety_mm);
  if (Number.isFinite(m.blend_mm)) $("run-blend").value = m.blend_mm;
  refreshSpeedLabel();
  showMsg("", false);
  updateButtons();
}

/* ── run ── */
function refreshSpeedLabel() {
  document.querySelector('output[for="run-speed"]').textContent =
    `${$("run-speed").value}%`;
}
$("run-speed").addEventListener("input", refreshSpeedLabel);

$("btn-run").addEventListener("click", () => {
  send("run", {
    speed_pct: parseFloat($("run-speed").value),
    safety_mm: parseFloat($("run-safety").value),
    blend_mm: parseFloat($("run-blend").value),
  });
});
$("btn-cancel").addEventListener("click", () => send("cancel"));

function updateButtons() {
  $("btn-run").disabled = !(connected && loadedName && !executing);
  $("btn-cancel").disabled = !executing;
  $("btn-connect").textContent = connected ? "Disconnect" : "Connect";
}

/* ── messages ── */
function showMsg(text, isError) {
  const m = $("msg");
  m.textContent = text;
  m.classList.toggle("err", !!isError);
}
function showRobotMsg(text, isError) {
  const m = $("robot-msg");
  m.textContent = text;
  m.classList.toggle("err", !!isError);
}

/* ── incoming ── */
function handle(data) {
  if (data.type === "init") {
    $("backend").textContent = data.backend || "";
    if (data.last_ip && !$("robot-ip").value) $("robot-ip").value = data.last_ip;
    renderList(data.toolpaths || []);
    if (data.toolpath) applyToolpath(data.toolpath);
  } else if (data.type === "toolpaths") {
    renderList(data.toolpaths || []);
  } else if (data.type === "toolpath") {
    applyToolpath(data);
  } else if (data.type === "connection_result") {
    showRobotMsg(data.message, !data.success);
  } else if (data.type === "run_result") {
    showMsg(data.message, !data.success);
  } else if (data.type === "state") {
    connected = !!data.connected;
    executing = !!data.executing;
    const chip = $("conn-chip");
    chip.textContent = connected ? "robot connected" : "robot disconnected";
    chip.classList.toggle("on", connected);
    $("phase").textContent = data.phase || "idle";
    $("progress").style.width =
      `${Math.round((data.progress || 0) * 100)}%`;
    if (data.exec_error) showMsg(data.exec_error, true);
    updateButtons();
  }
}

/* ── connection ── */
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  const watchdog = setTimeout(() => {
    if (ws.readyState !== WebSocket.OPEN) ws.close();
  }, 4000);
  ws.onopen = () => clearTimeout(watchdog);
  ws.onmessage = (ev) => { try { handle(JSON.parse(ev.data)); } catch (_) {} };
  ws.onclose = () => {
    clearTimeout(watchdog);
    $("phase").textContent = "reconnecting…";
    setTimeout(connect, 1500);
  };
}
connect();
