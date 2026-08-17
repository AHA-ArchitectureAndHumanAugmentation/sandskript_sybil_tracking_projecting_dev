/* Scheduler UI (contained from viewer.js).

   One spreadsheet of every saved toolpath bundle under paths/:

     #               a 1-based index, oldest first — row 1 is the earliest
     Path executed   the bundle folder, with the files it holds underneath
     Date and time   when that bundle was written
     Mask            the groove mask that path was traced from

   Read-only. The server pushes a fresh scan over the WebSocket whenever paths/
   changes, so a run finishing in the main app shows up here without a reload. */
"use strict";

const $ = (id) => document.getElementById(id);

// The files save_bundle writes. Listed in a fixed order so every row's file
// line reads the same way, and a bundle missing one is obvious at a glance
// rather than being spotted by its absence.
const EXPECTED = ["path.json", "path.script", "preview.png", "mask.png",
                  "skeleton.png"];

let ws = null;

function setStatus(text, isError) {
  const el = $("status");
  el.textContent = text;
  el.classList.toggle("err", !!isError);
}

function fileLine(files) {
  const have = new Set(files);
  const line = document.createElement("div");
  line.className = "files";
  for (const name of EXPECTED) {
    const s = document.createElement("span");
    s.textContent = have.has(name) ? name : `${name} —`;
    if (!have.has(name)) s.className = "missing";
    line.appendChild(s);
  }
  // Anything the exporter does not write, but that is sitting in the folder.
  for (const name of files) {
    if (EXPECTED.includes(name)) continue;
    const s = document.createElement("span");
    s.textContent = name;
    line.appendChild(s);
  }
  return line;
}

/* The groove mask that produced this path. Loaded lazily — a long ledger is a
   lot of 640×480 PNGs, and only the rows on screen are worth fetching. Bundles
   saved before mask.png existed simply have none. */
function maskCell(r) {
  const td = document.createElement("td");
  td.className = "col-mask";
  if (!r.has_mask) {
    const none = document.createElement("span");
    none.className = "none";
    none.textContent = "—";
    none.title = "this bundle was saved before mask images were written";
    td.appendChild(none);
    return td;
  }
  const href = `/mask/${encodeURIComponent(r.name)}`;
  const a = document.createElement("a");
  a.href = href;
  a.target = "_blank";
  a.rel = "noopener";
  a.title = "open full size";
  const img = document.createElement("img");
  img.src = href;
  img.loading = "lazy";
  img.alt = `groove mask for ${r.name}`;
  a.appendChild(img);
  td.appendChild(a);
  return td;
}

function render(payload) {
  const rows = payload.rows || [];
  const body = $("rows");
  body.innerHTML = "";
  $("empty").hidden = rows.length > 0;
  $("count").textContent =
    `${rows.length} path${rows.length === 1 ? "" : "s"}`;
  $("base").textContent = payload.base || "";

  for (const r of rows) {
    const tr = document.createElement("tr");

    const idx = document.createElement("td");
    idx.className = "col-index";
    idx.textContent = r.index;

    const path = document.createElement("td");
    const name = document.createElement("div");
    name.className = "name";
    name.textContent = r.name;
    path.append(name, fileLine(r.files || []));

    const when = document.createElement("td");
    when.className = "col-when";
    when.textContent = r.executed_at || "—";
    // Only a folder-name timestamp is exact; the rest were worked out.
    if (r.executed_at && r.time_source !== "folder name") {
      when.classList.add("guess");
      when.title = `time taken from the ${r.time_source}`;
    }

    tr.append(idx, path, when, maskCell(r));
    body.appendChild(tr);
  }
}

$("btn-refresh").addEventListener("click", () => {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "refresh" }));
    setStatus("refreshing…");
  }
});

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  const watchdog = setTimeout(() => {
    if (ws.readyState !== WebSocket.OPEN) ws.close();
  }, 4000);
  ws.onopen = () => {
    clearTimeout(watchdog);
    setStatus("watching paths folder");
  };
  ws.onmessage = (ev) => {
    try {
      const d = JSON.parse(ev.data);
      if (d.type === "init" || d.type === "schedule") {
        render(d);
        setStatus("watching paths folder");
      }
    } catch (_) {}
  };
  ws.onclose = () => {
    clearTimeout(watchdog);
    setStatus("disconnected — retrying…", true);
    setTimeout(connect, 1500);
  };
}

connect();
