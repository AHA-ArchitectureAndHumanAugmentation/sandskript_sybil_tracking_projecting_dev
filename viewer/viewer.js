/* ── Three.js scene setup ───────────────────────────────────────────────── */
const canvas   = document.getElementById("threejs-canvas");
const panel    = document.getElementById("panel-3d");

// preserveDrawingBuffer lets us grab the canvas as a PNG (for Save) at any time.
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x0e0e0e);

const scene  = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.001, 100);
camera.position.set(0.5, -0.9, 1.0);
camera.lookAt(0.5, 0, 0.15);

const controls = new THREE.OrbitControls(camera, canvas);
controls.target.set(0.5, 0, 0.15);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const dlight = new THREE.DirectionalLight(0xffffff, 0.9);
dlight.position.set(0.5, -1, 2);
scene.add(dlight);
scene.add(new THREE.AxesHelper(0.1));

/* ── Workspace visualization ────────────────────────────────────────────── */
let wsGroup = null;
let lastWorkspaceJson = null;

function buildWorkspaceViz(ws) {
  if (wsGroup) { scene.remove(wsGroup); wsGroup = null; }
  if (!ws) return;

  wsGroup = new THREE.Group();

  const o  = new THREE.Vector3(...ws.origin);
  const xa = new THREE.Vector3(...ws.x_axis);
  const ya = new THREE.Vector3(...ws.y_axis);
  const xe = ws.x_extent;
  const ye = ws.y_extent;

  const p00 = o.clone();
  const p10 = o.clone().addScaledVector(xa, xe);
  const p11 = o.clone().addScaledVector(xa, xe).addScaledVector(ya, ye);
  const p01 = o.clone().addScaledVector(ya, ye);

  const borderPts = [p00, p10, p11, p01, p00];
  const borderGeo = new THREE.BufferGeometry().setFromPoints(borderPts);
  wsGroup.add(new THREE.Line(borderGeo, new THREE.LineBasicMaterial({ color: 0x444444 })));

  const gridMat = new THREE.LineBasicMaterial({ color: 0x2a2a2a });
  const DIVS = 5;
  for (let i = 1; i < DIVS; i++) {
    const t = i / DIVS;
    const geoX = new THREE.BufferGeometry().setFromPoints([
      o.clone().addScaledVector(xa, xe * t),
      o.clone().addScaledVector(xa, xe * t).addScaledVector(ya, ye),
    ]);
    const geoY = new THREE.BufferGeometry().setFromPoints([
      o.clone().addScaledVector(ya, ye * t),
      o.clone().addScaledVector(xa, xe).addScaledVector(ya, ye * t),
    ]);
    wsGroup.add(new THREE.Line(geoX, gridMat));
    wsGroup.add(new THREE.Line(geoY, gridMat));
  }

  wsGroup.add(makeArrow(o, xa, xe * 0.15, 0xff3333));
  wsGroup.add(makeArrow(o, ya, ye * 0.15, 0x33cc55));

  scene.add(wsGroup);

  const za     = new THREE.Vector3(...ws.z_axis);
  const center = o.clone().addScaledVector(xa, xe / 2).addScaledVector(ya, ye / 2);
  const diag   = Math.sqrt(xe * xe + ye * ye);
  controls.target.copy(center);
  camera.position.copy(center).addScaledVector(za, diag * 1.5);
  camera.up.copy(ya);
  camera.lookAt(center);
  controls.update();
}

function makeArrow(origin, dir, length, color) {
  const tip = origin.clone().addScaledVector(dir, length);
  const geo = new THREE.BufferGeometry().setFromPoints([origin, tip]);
  return new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
}

/* ── Target surface visualization ───────────────────────────────────────── */
/* The mesh is sent once in its LOCAL frame; placing it (pose sliders) only
   updates this group's transform, so dragging sliders is instant. */
let surfaceGroup = null;

function buildSurfaceViz(mesh) {
  if (surfaceGroup) { scene.remove(surfaceGroup); surfaceGroup = null; }
  if (!mesh || !mesh.vertices || !mesh.vertices.length) return;

  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.Float32BufferAttribute(mesh.vertices, 3));
  geo.setIndex(mesh.faces);
  geo.computeVertexNormals();

  const mat = new THREE.MeshStandardMaterial({
    color: 0x9a7b4f,            // sand
    roughness: 0.95,
    metalness: 0.0,
    transparent: true,
    opacity: 0.85,
    side: THREE.DoubleSide,
  });
  surfaceGroup = new THREE.Group();
  surfaceGroup.add(new THREE.Mesh(geo, mat));
  const wire = new THREE.LineSegments(
    new THREE.WireframeGeometry(geo),
    new THREE.LineBasicMaterial({ color: 0x5a4a33, transparent: true, opacity: 0.25 })
  );
  surfaceGroup.add(wire);
  buildCornerMarkers(mesh.corners, geo);   // numbered touch-off corners (hidden)
  scene.add(surfaceGroup);
  applySurfacePose(readSurfacePose());
}

/* ── Corner markers for Register Corner → TCP ───────────────────────────── */
/* Numbered spheres at the mesh's touch-off corners (LOCAL frame, from the
   server's mesh payload). Children of surfaceGroup so they follow the pose
   sliders automatically; visible only while the registration popup is open. */
let cornerGroup = null;
let cornerMeshes = [];        // sphere per corner, for selection highlighting
let surfaceCorners = [];      // local-frame [x,y,z] — indices match the server
let selectedCorner = -1;
let hoveredCorner = -1;       // hover in the list OR over a marker in the preview

function buildCornerMarkers(corners, geo) {
  surfaceCorners = corners || [];
  cornerGroup = new THREE.Group();
  cornerGroup.visible = false;
  cornerMeshes = [];
  geo.computeBoundingSphere();
  const r = Math.max((geo.boundingSphere ? geo.boundingSphere.radius : 0.2) * 0.02, 0.005);
  surfaceCorners.forEach((c, i) => {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(r, 12, 8),
      new THREE.MeshBasicMaterial({ color: 0xffb020, depthTest: false }));
    m.position.set(c[0], c[1], c[2]);
    m.renderOrder = 5;
    cornerGroup.add(m);
    cornerMeshes.push(m);
    const label = makeTextSprite(String(i + 1));
    label.position.set(c[0], c[1], c[2]);
    label.center.set(0.5, -0.35);          // number floats just above the dot
    label.scale.setScalar(r * 7);
    label.renderOrder = 6;
    cornerGroup.add(label);
  });
  surfaceGroup.add(cornerGroup);
}

function makeTextSprite(text) {
  const cv = document.createElement("canvas");
  cv.width = cv.height = 64;
  const c = cv.getContext("2d");
  c.font = "bold 44px monospace";
  c.textAlign = "center";
  c.textBaseline = "middle";
  c.lineWidth = 8;
  c.strokeStyle = "#000000";
  c.strokeText(text, 32, 34);
  c.fillStyle = "#ffffff";
  c.fillText(text, 32, 34);
  const tex = new THREE.CanvasTexture(cv);
  return new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
}

function updateCornerHighlight() {
  // Selected = green; hovered (list row or preview marker) = cyan + enlarged,
  // so the user always sees WHICH corner they are about to pick.
  cornerMeshes.forEach((m, i) => {
    const sel = i === selectedCorner, hov = i === hoveredCorner;
    m.material.color.setHex(sel ? 0x00e5a0 : hov ? 0x66d9ff : 0xffb020);
    m.scale.setScalar(hov || sel ? 1.7 : 1.0);
  });
}

function applySurfacePose(p) {
  if (!surfaceGroup) return;
  const d = Math.PI / 180;
  // 'ZYX' here matches scipy's extrinsic 'xyz' euler on the server.
  surfaceGroup.setRotationFromEuler(new THREE.Euler(p.rx * d, p.ry * d, p.rz * d, "ZYX"));
  surfaceGroup.position.set(p.tx, p.ty, p.tz);
}

/* ── Path preview (set after Capture) ──────────────────────────────────── */
/* Two layers:
   - skelGroup: the detected skeleton projected onto the surface — a dense
     WHITE line lying exactly ON the surface (zero offset). Pure geometry.
   - pathGroup: the actual movep toolpath — waypoints at the chosen Spacing,
     lifted by the exec-bar Offset along each waypoint's tool axis, corners
     rounded by the movep blend radius, with waypoint dots, amber safety
     (retract) points and gray travel/approach moves. Rebuilt client-side
     whenever Offset or Safety change; Spacing re-generates on the server. */
let pathGroup  = null;
let skelGroup  = null;
let orderGroup = null;
let lastStrokes  = null;   // 6-DOF waypoint strokes from the server
let lastSkeleton = null;   // dense [x,y,z] skeleton polylines
let execViz    = { blend_m: 0.0005, reach_m: 1.30, min_reach_m: 0.18 };
let pathMode   = "path";   // "path" | "order"
let orderSize  = 1.0;      // multiplier for order numbers + start/end dots

/* Outward surface normal at a waypoint = -(R @ [0,0,1]) — mirrors
   path_executor._offset_pose / path_export._tool_axis on the server. */
function toolNormal(pose) {
  const rv  = new THREE.Vector3(pose[3], pose[4], pose[5]);
  const ang = rv.length();
  const q   = new THREE.Quaternion();
  if (ang > 1e-9) q.setFromAxisAngle(rv.divideScalar(ang), ang);
  return new THREE.Vector3(0, 0, 1).applyQuaternion(q).negate();
}

function offsetPoint(pose, dist) {
  const n = toolNormal(pose);
  return new THREE.Vector3(pose[0] + dist * n.x,
                           pose[1] + dist * n.y,
                           pose[2] + dist * n.z);
}

/* Same envelope test as reach.py, but applied to the OFFSET waypoint
   positions, so colors reflect where the TCP will actually be at run time. */
function outOfReach(v) {
  return v.length() > execViz.reach_m ||
         Math.hypot(v.x, v.y) < execViz.min_reach_m;
}

function readExecOffsetM() {
  return (parseFloat(document.getElementById("exec-offset").value) || 0) / 1000;
}
function readExecSafetyM() {
  return (parseFloat(document.getElementById("exec-safety").value) || 50) / 1000;
}
function readBlendMm() {
  const v = parseFloat(document.getElementById("exec-blend").value);
  return Number.isFinite(v) ? v : 0.5;   // 0 is a valid value — no || fallback
}

/* Entry point: store the capture payload and (re)build all preview layers. */
function setPathData(strokes, skeleton, viz) {
  lastStrokes  = strokes && strokes.length ? strokes : null;
  lastSkeleton = skeleton && skeleton.length ? skeleton : null;
  if (viz && Object.keys(viz).length) execViz = Object.assign({}, execViz, viz);
  buildSkeletonViz();
  rebuildToolpathViz();
  buildOrderViz(lastStrokes);
  applyPathMode();
}

function buildSkeletonViz() {
  if (skelGroup) { scene.remove(skelGroup); skelGroup = null; }
  if (!lastSkeleton) return;
  skelGroup = new THREE.Group();
  const mat = new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.9 });
  lastSkeleton.forEach(s => {
    if (s.length < 2) return;
    const geo = new THREE.BufferGeometry().setFromPoints(
      s.map(p => new THREE.Vector3(p[0], p[1], p[2])));
    skelGroup.add(new THREE.Line(geo, mat));
  });
  scene.add(skelGroup);
}

/* Corner-blended colored segments for one stroke: interior corners are
   trimmed by the movep blend radius and rounded with a small quadratic arc —
   the shape the controller actually drives. Pushes [a,b] segment pairs. */
function pushBlendedStroke(pts, flags, blend, okPts, badPts) {
  const n = pts.length;
  if (n < 2) return;
  const starts = pts.slice(0, n - 1);
  const ends   = pts.slice(1);
  for (let j = 1; j < n - 1; j++) {
    const dIn  = pts[j].clone().sub(pts[j - 1]);
    const dOut = pts[j + 1].clone().sub(pts[j]);
    const r = Math.min(blend, dIn.length() / 2, dOut.length() / 2);
    if (r <= 1e-6) continue;
    const a = pts[j].clone().addScaledVector(dIn.normalize(), -r);
    const b = pts[j].clone().addScaledVector(dOut.normalize(), r);
    ends[j - 1] = a;
    starts[j]   = b;
    const bucket = flags[j] ? badPts : okPts;   // corner takes the waypoint's flag
    let prev = a;
    for (let k = 1; k <= 4; k++) {              // quadratic bezier a → corner → b
      const t = k / 4, u = 1 - t;
      const q = new THREE.Vector3(
        u * u * a.x + 2 * u * t * pts[j].x + t * t * b.x,
        u * u * a.y + 2 * u * t * pts[j].y + t * t * b.y,
        u * u * a.z + 2 * u * t * pts[j].z + t * t * b.z);
      bucket.push(prev, q);
      prev = q;
    }
  }
  for (let i = 0; i < n - 1; i++) {
    const bucket = (flags[i] || flags[i + 1]) ? badPts : okPts;
    bucket.push(starts[i], ends[i]);
  }
}

function makeDots(positions, color, size) {
  const geo = new THREE.BufferGeometry().setFromPoints(positions);
  return new THREE.Points(geo, new THREE.PointsMaterial({ color, size, sizeAttenuation: true }));
}

function rebuildToolpathViz() {
  if (pathGroup) { scene.remove(pathGroup); pathGroup = null; }
  if (!lastStrokes) return;

  const off    = readExecOffsetM();
  const safety = readExecSafetyM();
  const blend  = readBlendMm() / 1000;   // the Radius slider drives the preview

  pathGroup = new THREE.Group();
  const okPts = [], badPts = [];           // colored draw segments
  const wpOk = [], wpBad = [];             // waypoint dots
  const safetyPts = [], travelPts = [];    // retract markers + pen-up moves

  let prevSafety = null;
  lastStrokes.forEach(stroke => {
    if (!stroke.length) return;
    const pts   = stroke.map(p => offsetPoint(p, off));
    const flags = pts.map(v => (outOfReach(v) ? 1 : 0));

    // Safety (retract) points off the stroke start/end along the tool axis —
    // where the executor's approach/lift movel targets actually are.
    const sStart = offsetPoint(stroke[0], off + safety);
    const sEnd   = offsetPoint(stroke[stroke.length - 1], off + safety);
    safetyPts.push(sStart, sEnd);

    if (prevSafety) travelPts.push(prevSafety, sStart);   // pen-up travel
    travelPts.push(sStart, pts[0]);                       // approach descend
    pushBlendedStroke(pts, flags, blend, okPts, badPts);
    travelPts.push(pts[pts.length - 1], sEnd);            // lift after stroke
    prevSafety = sEnd;

    pts.forEach((v, i) => (flags[i] ? wpBad : wpOk).push(v));
  });

  if (travelPts.length)
    pathGroup.add(new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(travelPts),
      new THREE.LineBasicMaterial({ color: 0x444466 })));
  if (okPts.length)
    pathGroup.add(new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(okPts),
      new THREE.LineBasicMaterial({ color: 0x00e5a0 })));
  if (badPts.length)
    pathGroup.add(new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(badPts),
      new THREE.LineBasicMaterial({ color: 0xff4444 })));
  if (wpOk.length)  pathGroup.add(makeDots(wpOk,  0x00e5a0, 0.004));
  if (wpBad.length) pathGroup.add(makeDots(wpBad, 0xff4444, 0.004));
  if (safetyPts.length) pathGroup.add(makeDots(safetyPts, 0xffb84d, 0.006));

  scene.add(pathGroup);
}

/* Order overlay: a numbered label per stroke + green start / red end markers,
   so the drawing order and each stroke's direction are visible. */
function buildOrderViz(strokes) {
  if (orderGroup) { scene.remove(orderGroup); orderGroup = null; }
  if (!strokes || strokes.length === 0) return;

  orderGroup = new THREE.Group();
  const startMat = new THREE.MeshBasicMaterial({ color: 0x00e5a0 }); // green — start
  const endMat   = new THREE.MeshBasicMaterial({ color: 0xff4444 }); // red   — end
  const dot = new THREE.SphereGeometry(0.005 * orderSize, 12, 12);

  strokes.forEach((stroke, i) => {
    if (stroke.length === 0) return;
    const s = stroke[0];
    const e = stroke[stroke.length - 1];
    const m = stroke[Math.floor(stroke.length / 2)];   // midpoint along the stroke

    const startDot = new THREE.Mesh(dot, startMat);
    startDot.position.set(s[0], s[1], s[2]);
    orderGroup.add(startDot);

    const endDot = new THREE.Mesh(dot, endMat);
    endDot.position.set(e[0], e[1], e[2]);
    orderGroup.add(endDot);

    // Number the stroke, placed just above its midpoint.
    const label = makeLabelSprite(String(i + 1), 0.03 * orderSize);
    label.position.set(m[0], m[1], m[2] + 0.012 * orderSize);
    orderGroup.add(label);
  });

  scene.add(orderGroup);
}

function makeLabelSprite(text, scale) {
  const size = 128;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext("2d");
  ctx.fillStyle = "rgba(10, 12, 18, 0.78)";
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, size / 2 - 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "#00e5a0";
  ctx.lineWidth = 6;
  ctx.stroke();
  ctx.fillStyle = "#ffffff";
  ctx.font = "bold 70px sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, size / 2, size / 2 + 4);

  const tex = new THREE.CanvasTexture(canvas);
  const mat = new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true });
  const sp = new THREE.Sprite(mat);
  sp.scale.set(scale, scale, 1);
  return sp;
}

function applyPathMode() {
  if (orderGroup) orderGroup.visible = (pathMode === "order");
  const legend = document.getElementById("path-legend");
  if (legend) legend.classList.toggle("hidden", pathMode !== "order");
}

/* ── End-effector sphere ────────────────────────────────────────────────── */
const eeMat    = new THREE.MeshStandardMaterial({ color: 0x4488ff, emissive: 0x223366, roughness: 0.4 });
const eeSphere = new THREE.Mesh(new THREE.SphereGeometry(0.007, 20, 20), eeMat);
scene.add(eeSphere);

/* ── Resize handling ────────────────────────────────────────────────────── */
function resizeRenderer() {
  const w = panel.clientWidth;
  const h = panel.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resizeRenderer).observe(panel);
resizeRenderer();

/* ── Render loop ────────────────────────────────────────────────────────── */
(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

/* ── WebSocket ──────────────────────────────────────────────────────────── */
let ws = null;

function connectWS() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${protocol}//${location.host}/ws`);

  ws.onopen  = () => setHeaderStatus("ws", true, "WS connected");
  ws.onclose = () => { setHeaderStatus("ws", false, "WS disconnected"); setTimeout(connectWS, 2000); };
  ws.onerror = () => setHeaderStatus("ws", false, "WS error");

  ws.onmessage = (ev) => {
    const data = JSON.parse(ev.data);

    if (data.type === "init") {
      handleInit(data);
    } else if (data.type === "state") {
      updateScene(data);
      updateFooter(data);
    } else if (data.type === "connection_result") {
      setHeaderStatus("robot", data.success, data.message);
      // After a successful connect, prompt for the drawing target unless a
      // surface is already loaded (there is no 3-point calibration anymore).
      if (data.success && !surfaceGroup) showOverlay(true);
    } else if (data.type === "still") {
      handleStill(data);
    } else if (data.type === "preview") {
      handlePreview(data);
    } else if (data.type === "capture_result") {
      handleCaptureResult(data);
    } else if (data.type === "reference_status") {
      handleReferenceStatus(data);
    } else if (data.type === "surface_status") {
      handleSurfaceStatus(data);
    } else if (data.type === "save_result") {
      handleSaveResult(data);
    } else if (data.type === "register_result") {
      handleRegisterResult(data);
    } else if (data.type === "execution_update") {
      handleExecutionUpdate(data);
    }
  };
}

connectWS();

/* ── Init ───────────────────────────────────────────────────────────────── */
function handleInit(data) {
  if (data.last_ip) {
    document.getElementById("ip-input").value = data.last_ip;
  }
  if (data.workspace) {
    applyWorkspace(data.workspace);
  }
  if (data.surface && data.surface.loaded) {
    handleSurfaceStatus(data.surface);
  }
  restoreSessionSettings(data);
}

/* Restore this window's controls from the server's current session settings
   (Participant Mode shares them). Without this, a reopened Developer window
   shows the DEFAULTS — and the next slider touch would send all of them,
   silently resetting the tuned values on the server. */
function restoreSessionSettings(data) {
  const d = data.detect || {};
  const adj = d.adjustments || {};
  adjRows().forEach((row) => {
    const v = adj[row.dataset.key];
    if (v !== undefined && v !== null && !Number.isNaN(parseFloat(v))) {
      row.querySelector("input").value = v;
      updateAdjVal(row);
    }
  });
  if (adj.detect) document.getElementById("detect-mode").value = adj.detect;
  const c = d.crop;
  if (c && [c.x, c.y, c.w, c.h].every(Number.isFinite)) {
    crop = { x: c.x, y: c.y, w: c.w, h: c.h };
    renderCrop();
  }
  if (Number.isFinite(d.spacing_mm)) {
    document.getElementById("exec-spacing").value = d.spacing_mm;
    document.getElementById("exec-spacing-val").textContent =
      document.getElementById("exec-spacing").value + " mm";
  }
  if (Number.isFinite(d.join_mm)) document.getElementById("exec-join").value = d.join_mm;
  const e = data.exec || {};
  if (Number.isFinite(e.speed_pct)) {
    document.getElementById("exec-speed").value = e.speed_pct;
    document.getElementById("exec-speed-val").textContent =
      document.getElementById("exec-speed").value + "%";
  }
  if (Number.isFinite(e.offset_mm)) document.getElementById("exec-offset").value = e.offset_mm;
  if (Number.isFinite(e.safety_mm)) document.getElementById("exec-safety").value = e.safety_mm;
  if (Number.isFinite(e.blend_mm)) {
    document.getElementById("exec-blend").value = e.blend_mm;
    document.getElementById("exec-blend-val").textContent =
      document.getElementById("exec-blend").value + " mm";
  }
}

function applyWorkspace(ws) {
  const wsJson = JSON.stringify(ws);
  if (wsJson !== lastWorkspaceJson) {
    lastWorkspaceJson = wsJson;
    buildWorkspaceViz(ws);
  }
}

/* ── Capture result ─────────────────────────────────────────────────────── */
function handleCaptureResult(data) {
  if (!data.success) {
    setHeaderStatus("robot", false, "Capture failed: " + (data.error || "unknown error"));
    return;
  }

  document.getElementById("val-strokes").textContent = data.stroke_count;

  if (data.strokes && data.strokes.length > 0) {
    setPathData(data.strokes, data.skeleton, data.exec_viz);
    setButtonsForPhase("captured");
    if (data.reach_out > 0) {
      setHeaderStatus("robot", false,
        `⚠ ${data.reach_out} waypoints outside the arm's estimated reach — shown in RED. ` +
        `Move the surface closer / crop smaller before running.`);
    } else {
      setHeaderStatus("robot", true,
        `Path ready: ${data.stroke_count} strokes, ${data.point_count} points — all within estimated reach`);
    }
  } else {
    setPathData(null, null, null);
    setButtonsForPhase("editing");
    setHeaderStatus("robot", false,
      "No grooves found — lower Groove depth / adjust detection and try again.");
  }
}

/* ── Execution update ───────────────────────────────────────────────────── */
function handleExecutionUpdate(data) {
  setButtonsForPhase(data.phase);
  setProgress(data.progress);
}

/* ── Scene update (50ms state broadcast) ───────────────────────────────── */
function updateScene(data) {
  if (data.workspace !== undefined) {
    const wsJson = JSON.stringify(data.workspace);
    if (wsJson !== lastWorkspaceJson) {
      lastWorkspaceJson = wsJson;
      buildWorkspaceViz(data.workspace);
    }
  }

  if (data.ee && data.ee.length >= 3) {
    eeSphere.position.set(data.ee[0], data.ee[1], data.ee[2]);
  }

  // Live TCP marker: bright blue + enlarged while the path is executing, so
  // the dot visibly travels along the strokes during actuation (RTDE, 20 Hz).
  eeMat.color.setHex(data.executing ? 0x2e9fff : 0x4488ff);
  eeMat.emissive.setHex(data.executing ? 0x1a5acc : 0x223366);
  const s = data.executing ? 1.5 : 1.0;
  eeSphere.scale.set(s, s, s);
}

/* ── Footer update ──────────────────────────────────────────────────────── */
let robotConnected = false;
let autoLocked = false;   // Participant automation ON → manual pipeline locked

function updateFooter(data) {
  if (typeof data.robot_connected === "boolean") robotConnected = data.robot_connected;
  if (typeof data.freedrive === "boolean") updateRegisterFreedrive(data.freedrive);
  if (data.participant) autoLocked = !!data.participant.auto;

  document.getElementById("val-phase").textContent   = data.phase || "idle";
  document.getElementById("val-strokes").textContent = data.stroke_count ?? 0;

  // Surface executor failures (unreachable pose, robot not in Remote, …) so
  // the reason is visible at the rig, not just in the terminal.
  if (data.phase === "error" && data.exec_error) {
    setHeaderStatus("robot", false, "Run failed: " + data.exec_error);
  }

  setButtonsForPhase(data.phase);
  syncEditUI(data.phase);
  setProgress(data.progress || 0);
}

function setButtonsForPhase(phase) {
  const cap      = document.getElementById("btn-capture-image");
  const gen      = document.getElementById("btn-generate");
  const retake   = document.getElementById("btn-retake");
  const btnRun   = document.getElementById("btn-run");
  const btnCancel = document.getElementById("btn-cancel");
  const progWrap = document.getElementById("progress-bar-wrap");

  const inEdit     = phase === "editing" || phase === "captured" || phase === "done";
  const executing  = phase === "executing";

  cap.classList.toggle("hidden", inEdit || executing);
  cap.disabled = phase !== "previewing";

  retake.classList.toggle("hidden", !(inEdit && !executing));
  gen.classList.toggle("hidden", !(inEdit && !executing));

  btnRun.disabled = !((phase === "captured" || phase === "done") && !executing);
  btnCancel.classList.toggle("hidden", !executing);
  progWrap.classList.toggle("hidden", !executing);

  // Participant automation owns the pipeline while Auto is ON: grey out the
  // manual buttons (the server refuses them too). Cancel stays live as the
  // emergency stop. Reapplied every state tick, so toggling Auto off in the
  // popup re-enables them within ~50 ms.
  if (autoLocked) {
    cap.disabled = true;
    retake.disabled = true;
    gen.disabled = true;
    btnRun.disabled = true;
  } else {
    retake.disabled = false;
    gen.disabled = false;
  }
}

function setProgress(value) {
  document.getElementById("progress-bar").style.width = (value * 100).toFixed(1) + "%";
}

/* ── Setup overlay ──────────────────────────────────────────────────────── */
const overlay = document.getElementById("ws-overlay");

function showOverlay(visible) {
  overlay.classList.toggle("hidden", !visible);
}

/* ── Header status ──────────────────────────────────────────────────────── */
function setHeaderStatus(type, ok, message) {
  const dot   = document.getElementById("status-dot");
  const label = document.getElementById("header-status-label");
  const msg   = document.getElementById("connection-msg");

  if (type === "robot") {
    dot.className   = ok ? "connected" : "error";
    label.textContent = ok ? "Connected" : "Disconnected";
  }
  msg.textContent = message;
}

function sendWS(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}

/* ── Connection buttons ─────────────────────────────────────────────────── */
document.getElementById("btn-connect").addEventListener("click", () => {
  const ip = document.getElementById("ip-input").value.trim();
  if (!ip || !ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ type: "connect", ip }));
  setHeaderStatus("robot", false, `Connecting to ${ip}…`);
});

document.getElementById("btn-disconnect").addEventListener("click", () => {
  sendWS({ type: "disconnect" });
  showOverlay(false);
  stillLoaded = false;
  setPathData(null, null, null);
  showLive();
});

document.getElementById("ip-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter") document.getElementById("btn-connect").click();
});

/* ── Test mode: simulated workspace (no robot) ─────────────────────────── */
document.getElementById("btn-simulate").addEventListener("click", () => {
  sendWS({ type: "simulate_workspace" });
  showOverlay(false);
  setHeaderStatus("robot", false, "Test mode: simulated workspace (no robot). Press Capture.");
});

/* ── Capture image / Edit / Generate / Retake ──────────────────────────── */
/* Four fixed viewports: Depth | RGB (top) and Skeleton | Mask (bottom). Each has
   a live MJPEG image and a captured/preview image; we show one set at a time. */
const vpDepth      = document.getElementById("vp-depth");
const cropBox      = document.getElementById("crop-box");
const editPanel    = document.getElementById("edit-panel");
const feedDepth    = document.getElementById("feed-depth");
const feedRgb      = document.getElementById("feed-rgb");
const feedSkeleton = document.getElementById("feed-skeleton");
const feedMask     = document.getElementById("feed-mask");
const stillDepth   = document.getElementById("still-depth");
const stillRgb     = document.getElementById("still-rgb");
const prevSkeleton = document.getElementById("prev-skeleton");
const prevMask     = document.getElementById("prev-mask");

const liveFeeds = [feedDepth, feedRgb, feedSkeleton, feedMask];
const stillImgs = [stillDepth, stillRgb, prevSkeleton, prevMask];

let stillLoaded = false;
let crop        = { x: 0, y: 0, w: 1, h: 1 };   // normalized to displayed image

/* True while the captured stills are showing (vs. the live feeds). */
function editingActive() {
  return !stillDepth.classList.contains("hidden");
}

/* The Depth image currently visible — the crop overlay's drag math uses it. */
function activeDepthImg() {
  return editingActive() ? stillDepth : feedDepth;
}

/* Show/hide the live feeds vs. the captured stills. Idempotent — safe to call
   every state broadcast. The Detect Grooves panel and crop overlay show during
   live preview too, so all four views can be tuned before capturing. */
function showLive(showPanel) {
  liveFeeds.forEach(f => f.classList.remove("hidden"));
  stillImgs.forEach(s => s.classList.add("hidden"));
  editPanel.classList.toggle("hidden", !showPanel);
  cropBox.classList.toggle("hidden", !showPanel);
  renderCrop();
}

function showEditing(phase) {
  const executing = phase === "executing";
  liveFeeds.forEach(f => f.classList.add("hidden"));
  stillImgs.forEach(s => s.classList.remove("hidden"));
  editPanel.classList.toggle("hidden", executing);
  cropBox.classList.toggle("hidden", executing);
  renderCrop();
}

function syncEditUI(phase) {
  if (!stillLoaded || phase === "previewing" || phase === "idle") {
    showLive(phase === "previewing");   // show the Detect panel + crop while previewing live
    return;
  }
  showEditing(phase);
}

/* ── Projection window (projector output) ──────────────────────────────── */
/* Opens /projection in its own window: drag it onto the projector display and
   press F11. The full-frame mask is only composed while this window is open,
   so there is zero projection overhead otherwise. */
/* Chrome allows only ~6 concurrent HTTP/1.1 connections per host, and MJPEG
   streams hold theirs open forever (this app tab alone uses 4). Opening the
   projection windows on 127.0.0.1 instead of localhost gives them their own
   pool, so their mask stream and corner requests never starve. */
function projOrigin() {
  return location.hostname === "localhost"
    ? location.protocol + "//127.0.0.1:" + location.port
    : "";
}

function openProjWindow(btn, url, name, feature, message) {
  let win = window.open(projOrigin() + url, name, feature);
  if (!win) {
    setHeaderStatus("robot", false, "Pop-up blocked — allow pop-ups for this site.");
    return null;
  }
  btn.classList.add("active");
  const watch = setInterval(() => {
    if (!win || win.closed) {
      clearInterval(watch);
      btn.classList.remove("active");
    }
  }, 1000);
  setHeaderStatus("robot", true, message);
  return win;
}

let projWin = null;
let projCalWin = null;
const btnProject = document.getElementById("btn-project");
const btnProjectCal = document.getElementById("btn-project-cal");

btnProject.addEventListener("click", () => {
  if (projWin && !projWin.closed) { projWin.focus(); return; }
  projWin = openProjWindow(btnProject, "/projection", "sandProjection",
    "width=1920,height=1080",
    "Projection window opened — move it to the projector display and press F11 " +
    "(output resolution follows the display: set the projector to 4K in Windows for 4K).");
});

btnProjectCal.addEventListener("click", () => {
  if (projCalWin && !projCalWin.closed) { projCalWin.focus(); return; }
  projCalWin = openProjWindow(btnProjectCal, "/projection?cal", "sandProjectionCal",
    "width=1100,height=760",
    "Calibration window opened — keep it on this screen, drag handles 1–4 while " +
    "watching the sand; the projector output follows live.");
});

/* Depth-numbers reference window (Depth viewport). Opened on the 127.0.0.1
   origin like the projection windows: it holds its own MJPEG stream, so it
   must not compete for this tab's 6-connection pool. */
let depthNumWin = null;
const btnDepthNumbers = document.getElementById("btn-depth-numbers");

btnDepthNumbers.addEventListener("click", () => {
  if (depthNumWin && !depthNumWin.closed) { depthNumWin.focus(); return; }
  depthNumWin = openProjWindow(btnDepthNumbers, "/depths", "depthNumbers",
    "width=920,height=760",
    "Participant Mode opened — switch Auto ON + set a trigger distance to automate the pipeline.");
});

/* ── Swap the left/right position of a row's two viewports ──────────────── */
document.getElementById("swap-depth").addEventListener("click", () => {
  document.getElementById("row-depth").classList.toggle("swapped");
  renderCrop();
});
document.getElementById("swap-grooves").addEventListener("click", () => {
  document.getElementById("row-grooves").classList.toggle("swapped");
});

/* ── Capture / Retake / Generate ───────────────────────────────────────── */
document.getElementById("btn-capture-image").addEventListener("click", () => {
  sendWS({ type: "capture_image" });
  setHeaderStatus("robot", true, "Capturing still…");
});

document.getElementById("btn-retake").addEventListener("click", () => {
  stillLoaded = false;
  setPathData(null, null, null);
  showLive(true);
  sendWS({ type: "retake" });
  requestAdjust(true);                  // keep the live groove feed in sync with the panel
  setHeaderStatus("robot", true, "Back to live feed.");
});

document.getElementById("btn-generate").addEventListener("click", () => {
  sendWS({ type: "generate_path", params: buildGenerateParams() });
  setHeaderStatus("robot", true, "Generating tool path…");
});

/* Path generation params = detection params + crop + waypoint spacing (mm)
   + the endpoint-join Distance Threshold (mm, 0 = off). */
function readSpacing() {
  return parseFloat(document.getElementById("exec-spacing").value) || 10;
}

function readJoinMm() {
  const v = parseFloat(document.getElementById("exec-join").value);
  return Number.isFinite(v) ? Math.min(Math.max(v, 0), 200) : 0;
}

function buildGenerateParams() {
  return { ...buildParams(), spacing_mm: readSpacing(), join_mm: readJoinMm() };
}

/* Spacing lives in the Path Preview bar. Update the label live; re-generate on
   release (not every drag tick) so the path rebuilds at the new mm spacing. */
const spacingSlider = document.getElementById("exec-spacing");
spacingSlider.addEventListener("input", (e) => {
  document.getElementById("exec-spacing-val").textContent = e.target.value + " mm";
});
spacingSlider.addEventListener("change", () => {
  if (stillLoaded && lastStrokes) {
    sendWS({ type: "generate_path", params: buildGenerateParams() });
    setHeaderStatus("robot", true, "Re-generating tool path at " + readSpacing() + " mm spacing…");
  }
});

/* Distance Threshold changes which strokes are connected, so — like Spacing,
   and unlike Offset/Safety/Radius — the path must be rebuilt server-side. */
const joinBox = document.getElementById("exec-join");
joinBox.addEventListener("change", () => {
  if (stillLoaded && lastStrokes) {
    const mm = readJoinMm();
    sendWS({ type: "generate_path", params: buildGenerateParams() });
    setHeaderStatus("robot", true, mm > 0
      ? "Re-generating tool path, joining ends within " + mm + " mm…"
      : "Re-generating tool path with stroke joining off…");
  }
});

/* ── Incoming still + preview ──────────────────────────────────────────── */
function handleStill(data) {
  if (!data.depth && !data.rgb) return;
  stillLoaded = true;
  prevSkeleton.src = "";
  prevMask.src = "";
  stillRgb.src = data.rgb || "";
  // Keep the crop the user drew on the live view — it carries into capture.

  // onload re-lays-out the crop box; harmless if it re-fires on later updates.
  stillDepth.onload = renderCrop;
  stillDepth.src = data.depth || "";
  showEditing("editing");
  requestAdjust(true);                  // …then fetch the cropped grooves + mask
  setHeaderStatus("robot", true, "Crop and adjust, then Generate Path.");
}

function handlePreview(data) {
  // Bottom row: cropped skeleton + thick mask, shown side by side.
  if (data.grooves) prevSkeleton.src = data.grooves;
  if (data.mask)    prevMask.src     = data.mask;
  // RGB view shows only the cropped region (Depth stays full so it's croppable).
  if (data.rgb)     stillRgb.src     = data.rgb;
  // The colorized depth can change with the view-range sliders; keep it current.
  if (data.depth && stillLoaded) stillDepth.src = data.depth;
}

const pathToggle = document.getElementById("path-toggle");
pathToggle.querySelectorAll(".seg").forEach(btn => {
  btn.addEventListener("click", () => {
    pathMode = btn.dataset.path;
    pathToggle.querySelectorAll(".seg").forEach(b =>
      b.classList.toggle("active", b === btn));
    applyPathMode();
  });
});

document.getElementById("order-size").addEventListener("input", (e) => {
  orderSize = parseFloat(e.target.value) || 1.0;
  buildOrderViz(lastStrokes);   // rebuild numbers + dots at the new size
  applyPathMode();
});

/* ── Adjustment sliders ────────────────────────────────────────────────── */
function adjRows() { return document.querySelectorAll(".adj-row[data-key]"); }

function initAdjustControls() {
  adjRows().forEach(row => {
    const input = row.querySelector("input");   // range slider or number box
    input.min   = row.dataset.min;
    input.max   = row.dataset.max;
    input.step  = row.dataset.step;
    input.value = row.dataset.default;
    updateAdjVal(row);
    input.addEventListener("input", () => { updateAdjVal(row); requestAdjust(); });
  });
  document.getElementById("detect-mode").addEventListener("change", () => requestAdjust(true));
}

function updateAdjVal(row) {
  const span = row.querySelector(".adj-val");
  if (!span) return;                            // number boxes show their own value
  const input = row.querySelector("input");
  const step  = parseFloat(row.dataset.step);
  span.textContent = step < 1 ? parseFloat(input.value).toFixed(2) : input.value;
}

function readAdjustments() {
  const adj = {};
  adjRows().forEach(row => {
    const v = parseFloat(row.querySelector("input").value);
    adj[row.dataset.key] = Number.isFinite(v) ? v : 0;   // empty box = off
  });
  adj.detect = document.getElementById("detect-mode").value;
  return adj;
}

function cropForSend() {
  const c = crop;
  if (c.w <= 0.001 || c.h <= 0.001) return { x: 0, y: 0, w: 1, h: 1 };
  return { x: c.x, y: c.y, w: c.w, h: c.h };
}

function buildParams() {
  return { crop: cropForSend(), adjustments: readAdjustments() };
}

let adjustTimer = null;
function requestAdjust(immediate) {
  if (adjustTimer) clearTimeout(adjustTimer);
  // Before capture → push params to the live feed; after capture → re-process
  // the frozen still (cropped). Same params either way.
  const fire = () => sendWS({
    type: stillLoaded ? "preview_adjust" : "set_groove_params",
    params: buildParams(),
  });
  if (immediate) fire();
  else adjustTimer = setTimeout(fire, 120);
}

document.getElementById("btn-reset-adjust").addEventListener("click", () => {
  adjRows().forEach(row => {
    const input = row.querySelector("input");
    input.value = row.dataset.default;
    updateAdjVal(row);
  });
  document.getElementById("detect-mode").value = "valley";
  requestAdjust(true);
});

/* ── Save / Load detection-parameter presets ───────────────────────────────
   Save POSTs the current slider values (readAdjustments) to the server, which
   writes them to presets/<date_time>.json. Load lists that folder in a popup
   and applies the chosen file back onto the sliders. */
document.getElementById("btn-save-adjust").addEventListener("click", async () => {
  try {
    const res = await fetch("/presets", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ params: readAdjustments() }),
    });
    const data = await res.json();
    if (data.ok) setHeaderStatus("robot", true, "✓ Parameters saved to presets/" + data.name);
    else setHeaderStatus("robot", false, "Save failed: " + (data.error || "unknown error"));
  } catch (err) {
    setHeaderStatus("robot", false, "Save failed (server unreachable).");
  }
});

const presetOverlay = document.getElementById("preset-overlay");
const presetList    = document.getElementById("preset-list");

document.getElementById("btn-load-adjust").addEventListener("click", openPresetPopup);
document.getElementById("btn-preset-close").addEventListener("click", closePresetPopup);
presetOverlay.addEventListener("click", (e) => {
  if (e.target === presetOverlay) closePresetPopup();   // click backdrop to dismiss
});

function closePresetPopup() { presetOverlay.classList.add("hidden"); }

async function openPresetPopup() {
  presetList.innerHTML = "<li class='preset-empty'>Loading…</li>";
  presetOverlay.classList.remove("hidden");
  try {
    const res = await fetch("/presets");
    const items = (await res.json()).presets || [];
    if (!items.length) {
      presetList.innerHTML = "<li class='preset-empty'>No saved parameters yet — use Save first.</li>";
      return;
    }
    presetList.innerHTML = "";
    items.forEach((it) => {
      const li = document.createElement("li");
      li.className = "preset-item";
      const name = document.createElement("span");
      name.className = "preset-name";
      name.textContent = it.name;
      const time = document.createElement("span");
      time.className = "preset-time";
      time.textContent = it.saved || "";
      li.append(name, time);
      li.addEventListener("click", () => applyPreset(it.name));
      presetList.appendChild(li);
    });
  } catch (err) {
    presetList.innerHTML = "<li class='preset-empty'>Could not list saved files.</li>";
  }
}

async function applyPreset(name) {
  try {
    const res = await fetch("/presets/" + encodeURIComponent(name));
    const data = await res.json();
    if (!data.ok) { setHeaderStatus("robot", false, "Load failed: " + (data.error || "?")); return; }
    const p = data.params || {};
    adjRows().forEach((row) => {
      const key = row.dataset.key;
      const v = p[key];
      if (v !== undefined && v !== null && !Number.isNaN(parseFloat(v))) {
        row.querySelector("input").value = v;
        updateAdjVal(row);
      }
    });
    if (p.detect) document.getElementById("detect-mode").value = p.detect;
    requestAdjust(true);
    closePresetPopup();
    setHeaderStatus("robot", true, "✓ Parameters loaded from presets/" + name);
  } catch (err) {
    setHeaderStatus("robot", false, "Load failed (server unreachable).");
  }
}

document.getElementById("btn-crop-reset").addEventListener("click", () => {
  crop = { x: 0, y: 0, w: 1, h: 1 };
  renderCrop();
  requestAdjust(true);
});

/* ── Reference (background) subtraction ─────────────────────────────────── */
document.getElementById("btn-set-reference").addEventListener("click", () => {
  sendWS({ type: "set_reference" });
  setHeaderStatus("robot", true, "Capturing reference (empty sand)…");
});

document.getElementById("btn-clear-reference").addEventListener("click", () => {
  sendWS({ type: "clear_reference" });
});

function handleReferenceStatus(data) {
  const btn = document.getElementById("btn-set-reference");
  btn.classList.toggle("active", data.active);
  btn.textContent = data.active ? "Reference set ✓ (re-capture)" : "Set Reference (empty sand)";
  setHeaderStatus("robot", true, data.message || "");
  // Re-run detection so the change is reflected immediately (live or captured).
  requestAdjust(true);
}

/* ── Register Corner → TCP (touch-off surface placement) ──────────────────
   Optional popup: pick a numbered mesh corner (markers appear in the Path
   Preview), freedrive the tool tip onto the physical corner, confirm — the
   server recomputes the surface pose (1-point: translation only) and the
   sliders/preview update via the normal surface_status broadcast. Closing
   the popup without confirming changes nothing. */
const regOverlay = document.getElementById("register-overlay");
const regList    = document.getElementById("register-list");
const btnRegFreedrive = document.getElementById("btn-register-freedrive");
const btnRegConfirm   = document.getElementById("btn-register-confirm");
let regFreedrive = false;   // mirrors shared_state.freedrive (state broadcast)

document.getElementById("btn-register").addEventListener("click", openRegisterPopup);
document.getElementById("btn-register-close").addEventListener("click", closeRegisterPopup);

function regStatus(msg) { document.getElementById("register-status").textContent = msg; }

function openRegisterPopup() {
  if (!surfaceCorners.length) {
    setHeaderStatus("robot", false, "Load a surface first — registration places the loaded mesh.");
    return;
  }
  selectedCorner = -1;
  hoveredCorner = -1;
  buildRegisterList();
  updateCornerHighlight();
  btnRegConfirm.disabled = true;
  if (cornerGroup) cornerGroup.visible = true;
  regOverlay.classList.remove("hidden");
  regStatus(robotConnected
    ? `Pick a corner: click a numbered marker in the Path Preview, or a row below.`
    : "Robot NOT connected — you can look at the corners, but freedrive/confirm need a connection.");
}

function closeRegisterPopup() {
  regOverlay.classList.add("hidden");
  if (cornerGroup) cornerGroup.visible = false;
  hoveredCorner = -1;
  canvas.style.cursor = "";
  // Never leave the arm limp behind a closed popup.
  if (regFreedrive) sendWS({ type: "register_freedrive", params: { on: false } });
}

function selectCorner(i) {
  selectedCorner = i;
  [...regList.children].forEach((el, j) => el.classList.toggle("sel", j === i));
  updateCornerHighlight();
  btnRegConfirm.disabled = !robotConnected;
  regStatus(`Corner ${i + 1} selected (green in the preview). Freedrive the tool tip onto it, then Confirm.`);
}

function setHoveredCorner(i) {
  if (i === hoveredCorner) return;
  hoveredCorner = i;
  updateCornerHighlight();
  // Mirror the preview hover into the list, so both views always agree.
  [...regList.children].forEach((el, j) => el.classList.toggle("hov", j === i));
}

function buildRegisterList() {
  regList.innerHTML = "";
  surfaceCorners.forEach((c, i) => {
    const li = document.createElement("li");
    li.className = "preset-item";
    const name = document.createElement("span");
    name.className = "preset-name";
    name.textContent = `Corner ${i + 1}`;
    const pos = document.createElement("span");
    pos.className = "preset-time";
    pos.textContent = `local ${c.map(v => v.toFixed(3)).join(", ")} m`;
    li.append(name, pos);
    li.addEventListener("click", () => selectCorner(i));
    // Hovering a row highlights that marker in the Path Preview (cyan, larger).
    li.addEventListener("mouseenter", () => setHoveredCorner(i));
    li.addEventListener("mouseleave", () => setHoveredCorner(-1));
    regList.appendChild(li);
  });
}

/* ── Pick corners directly in the Path Preview ────────────────────────────
   While the register dialog is open, hovering near a marker highlights it
   (and its list row); a click — not an orbit drag — selects it. Picking is
   screen-space: nearest projected marker within a pixel radius, which stays
   easy to hit regardless of zoom. */
const REG_PICK_PX = 22;

function pickCorner(e) {
  if (!cornerGroup || !cornerGroup.visible) return -1;
  const rect = canvas.getBoundingClientRect();
  const px = e.clientX - rect.left, py = e.clientY - rect.top;
  const v = new THREE.Vector3();
  let best = -1, bestD = REG_PICK_PX;
  cornerMeshes.forEach((m, i) => {
    m.getWorldPosition(v);
    v.project(camera);
    if (v.z > 1) return;                       // behind the camera
    const sx = (v.x * 0.5 + 0.5) * rect.width;
    const sy = (-v.y * 0.5 + 0.5) * rect.height;
    const d = Math.hypot(sx - px, sy - py);
    if (d < bestD) { bestD = d; best = i; }
  });
  return best;
}

let regPtrDown = null;
canvas.addEventListener("pointermove", (e) => {
  if (!cornerGroup || !cornerGroup.visible) return;
  const h = pickCorner(e);
  setHoveredCorner(h);
  canvas.style.cursor = h >= 0 ? "pointer" : "";
});
canvas.addEventListener("pointerdown", (e) => {
  regPtrDown = [e.clientX, e.clientY];
});
canvas.addEventListener("pointerup", (e) => {
  if (!cornerGroup || !cornerGroup.visible || !regPtrDown) { regPtrDown = null; return; }
  const moved = Math.hypot(e.clientX - regPtrDown[0], e.clientY - regPtrDown[1]);
  regPtrDown = null;
  if (moved > 5) return;                       // that was an orbit/pan drag
  const i = pickCorner(e);
  if (i >= 0) selectCorner(i);
});

btnRegFreedrive.addEventListener("click", () => {
  if (!robotConnected) { regStatus("Robot not connected."); return; }
  sendWS({ type: "register_freedrive", params: { on: !regFreedrive } });
});

btnRegConfirm.addEventListener("click", () => {
  if (selectedCorner < 0 || !robotConnected) return;
  btnRegConfirm.disabled = true;
  regStatus("Registering — reading TCP…");
  sendWS({ type: "register_corner", params: { corner_index: selectedCorner } });
});

function handleRegisterResult(data) {
  if (data.success) {
    closeRegisterPopup();
    setHeaderStatus("robot", true, "✓ " + (data.message || "Corner registered."));
  } else {
    btnRegConfirm.disabled = selectedCorner < 0 || !robotConnected;
    regStatus("Failed: " + (data.error || "unknown error"));
  }
}

/* Keep the freedrive toggle button in sync with the robot (state broadcast). */
function updateRegisterFreedrive(on) {
  regFreedrive = !!on;
  btnRegFreedrive.textContent = regFreedrive ? "Freedrive ON — click to stop" : "Start Freedrive";
  btnRegFreedrive.classList.toggle("active", regFreedrive);
}

/* ── Target surface (3D projection) ─────────────────────────────────────── */
function srfRows() { return document.querySelectorAll(".adj-row[data-skey]"); }

function initSurfaceControls() {
  srfRows().forEach(row => {
    const input = row.querySelector("input[type=range]");
    input.min   = row.dataset.min;
    input.max   = row.dataset.max;
    input.step  = row.dataset.step;
    input.value = row.dataset.default;
    updateAdjVal(row);
    input.addEventListener("input", () => {
      updateAdjVal(row);
      applySurfacePose(readSurfacePose());   // instant client-side placement
      sendSurfacePose();                     // debounced push to the server
    });
  });

  document.getElementById("btn-surface-load").addEventListener("click", () =>
    document.getElementById("surface-file").click());
  // Same file input, reachable from the workspace-setup overlay: loading a
  // surface replaces the flat-workspace calibration entirely.
  document.getElementById("btn-ws-load-surface").addEventListener("click", () =>
    document.getElementById("surface-file").click());
  document.getElementById("surface-file").addEventListener("change", uploadSurface);
  document.getElementById("btn-surface-clear").addEventListener("click", () => {
    sendWS({ type: "clear_surface" });
  });
}

function readSurfacePose() {
  const p = { tx: 0.4, ty: 0, tz: 0, rx: 0, ry: 0, rz: 0, offset_mm: 0 };
  srfRows().forEach(row => {
    p[row.dataset.skey] = parseFloat(row.querySelector("input[type=range]").value);
  });
  return p;
}

let surfacePoseTimer = null;
function sendSurfacePose() {
  if (surfacePoseTimer) clearTimeout(surfacePoseTimer);
  surfacePoseTimer = setTimeout(() => {
    const p = readSurfacePose();
    sendWS({ type: "set_surface_pose",
             params: { pose: p, offset_mm: p.offset_mm } });
  }, 150);
}

/* Uploads add to the scene, so several files can be picked at once. They go up
   ONE AT A TIME on purpose: each upload reads the current scene and writes back
   the combined one, so parallel posts could drop a part. */
async function uploadSurface(e) {
  const files = Array.from(e.target.files || []);
  if (!files.length) return;
  for (let i = 0; i < files.length; i++) {
    const file = files[i];
    const of = files.length > 1 ? ` (${i + 1}/${files.length})` : "";
    setHeaderStatus("robot", true, `Uploading ${file.name}…${of}`);
    const form = new FormData();
    form.append("file", file);
    try {
      const res = await fetch("/surface/upload", { method: "POST", body: form });
      const out = await res.json();
      if (!out.ok) throw new Error(out.error || "upload failed");
      // The server broadcasts surface_status (with the combined mesh) to all clients.
    } catch (err) {
      setHeaderStatus("robot", false, `Surface upload failed (${file.name}): ` + err.message);
      break;
    }
  }
  e.target.value = "";   // allow re-selecting the same file
}

function handleSurfaceStatus(data) {
  const status = document.getElementById("surface-status");
  if (data.loaded) {
    if (data.mesh) buildSurfaceViz(data.mesh);
    if (data.pose) setSurfaceSliders(data.pose, data.offset_mm);
    const i = data.info || {};
    const size = i.bbox ? `${i.bbox.size[0]}×${i.bbox.size[1]}×${i.bbox.size[2]} m` : "";
    const what = (i.count > 1) ? `${i.count} surfaces combined` : (i.name || "surface");
    status.textContent = `Loaded: ${what} — ${i.faces || "?"} faces, ${size}`;
    renderSurfaceList(i.parts || []);
    showOverlay(false);   // surface replaces the P0/Px/Py calibration
  } else {
    if (surfaceGroup) { scene.remove(surfaceGroup); surfaceGroup = null; }
    cornerGroup = null;             // children of surfaceGroup — gone with it
    cornerMeshes = [];
    surfaceCorners = [];
    selectedCorner = -1;
    hoveredCorner = -1;
    canvas.style.cursor = "";
    status.textContent = "No surface loaded.";
    renderSurfaceList([]);
  }
  if (data.message) setHeaderStatus("robot", true, data.message);
  applySurfacePose(readSurfacePose());
}

/* One row per loaded surface. The ✕ removes only that part (remove_surface);
   the indices come from the server, so they always match its parts list. */
function renderSurfaceList(parts) {
  const list = document.getElementById("surface-list");
  list.innerHTML = "";
  if (parts.length < 2) return;    // a single surface is already named above
  parts.forEach(p => {
    const li = document.createElement("li");
    li.className = "preset-item";
    const name = document.createElement("span");
    name.className = "preset-name";
    name.textContent = p.name;
    const right = document.createElement("span");
    right.className = "preset-time";
    right.textContent = `${p.faces} faces`;
    const del = document.createElement("button");
    del.className = "surface-del";
    del.textContent = "✕";
    del.title = `Remove ${p.name}`;
    del.addEventListener("click", () =>
      sendWS({ type: "remove_surface", params: { index: p.index } }));
    li.appendChild(name);
    li.appendChild(right);
    li.appendChild(del);
    list.appendChild(li);
  });
}

function setSurfaceSliders(pose, offsetMm) {
  srfRows().forEach(row => {
    const key = row.dataset.skey;
    const val = key === "offset_mm" ? offsetMm : pose[key];
    if (val === undefined || val === null) return;
    const input = row.querySelector("input[type=range]");
    input.value = val;
    updateAdjVal(row);
  });
}

/* ── Crop overlay (drag to draw / move / resize) ───────────────────────── */
/* The overlay lives on the Depth viewport and tracks its active image (live
   feed or captured still), so a region can be cropped before *or* after capture. */
function imgContentRect() {
  const cr = vpDepth.getBoundingClientRect();
  const ir = activeDepthImg().getBoundingClientRect();
  return { ox: ir.left - cr.left, oy: ir.top - cr.top, w: ir.width, h: ir.height };
}

function renderCrop() {
  if (cropBox.classList.contains("hidden")) return;
  const r = imgContentRect();
  if (r.w === 0 || r.h === 0) return;
  cropBox.style.left   = (r.ox + crop.x * r.w) + "px";
  cropBox.style.top    = (r.oy + crop.y * r.h) + "px";
  cropBox.style.width  = (crop.w * r.w) + "px";
  cropBox.style.height = (crop.h * r.h) + "px";
}

function clamp01(v) { return Math.min(Math.max(v, 0), 1); }

function ptNorm(e) {
  const ir = activeDepthImg().getBoundingClientRect();
  return {
    x: clamp01((e.clientX - ir.left) / ir.width),
    y: clamp01((e.clientY - ir.top)  / ir.height),
  };
}

let dragMode = null, dragStart = null, cropStart = null;

vpDepth.addEventListener("mousedown", (e) => {
  // No cropping when the overlay is hidden (idle).
  if (cropBox.classList.contains("hidden")) return;
  const handle = e.target.closest(".crop-handle");
  const p = ptNorm(e);
  if (handle) {
    dragMode = handle.dataset.h;
  } else if (p.x >= crop.x && p.x <= crop.x + crop.w &&
             p.y >= crop.y && p.y <= crop.y + crop.h) {
    dragMode = "move";
  } else {
    dragMode = "new";
    crop = { x: p.x, y: p.y, w: 0, h: 0 };
  }
  dragStart = p;
  cropStart = { ...crop };
  e.preventDefault();
  window.addEventListener("mousemove", onCropMove);
  window.addEventListener("mouseup", onCropUp);
});

function onCropMove(e) {
  const p = ptNorm(e);
  if (dragMode === "new") {
    crop.x = Math.min(dragStart.x, p.x);
    crop.y = Math.min(dragStart.y, p.y);
    crop.w = Math.abs(p.x - dragStart.x);
    crop.h = Math.abs(p.y - dragStart.y);
  } else if (dragMode === "move") {
    crop.x = clamp01(Math.min(cropStart.x + (p.x - dragStart.x), 1 - cropStart.w));
    crop.y = clamp01(Math.min(cropStart.y + (p.y - dragStart.y), 1 - cropStart.h));
  } else {
    let x0 = cropStart.x, y0 = cropStart.y;
    let x1 = cropStart.x + cropStart.w, y1 = cropStart.y + cropStart.h;
    if (dragMode.includes("w")) x0 = p.x;
    if (dragMode.includes("e")) x1 = p.x;
    if (dragMode.includes("n")) y0 = p.y;
    if (dragMode.includes("s")) y1 = p.y;
    crop.x = Math.min(x0, x1); crop.w = Math.abs(x1 - x0);
    crop.y = Math.min(y0, y1); crop.h = Math.abs(y1 - y0);
  }
  cropBox.classList.remove("hidden");
  renderCrop();
}

function onCropUp() {
  window.removeEventListener("mousemove", onCropMove);
  window.removeEventListener("mouseup", onCropUp);
  if (crop.w < 0.01 || crop.h < 0.01) crop = { x: 0, y: 0, w: 1, h: 1 };
  renderCrop();
  requestAdjust(true);
}

new ResizeObserver(renderCrop).observe(vpDepth);

/* ── Column splitter: drag to resize the 4-view block vs. path preview ──── */
(function () {
  const splitter = document.getElementById("col-splitter");
  const mainEl = document.querySelector("main");
  splitter.addEventListener("mousedown", (e) => {
    e.preventDefault();
    splitter.classList.add("dragging");
    document.body.style.userSelect = "none";
    const onMove = (ev) => {
      const rect = mainEl.getBoundingClientRect();
      const min = 240, max = rect.width - 320;              // keep both sides usable
      const w = Math.max(min, Math.min(ev.clientX - rect.left, max));
      mainEl.style.setProperty("--cam-w", w + "px");
      // 3D canvas + crop overlay re-fit automatically via their ResizeObservers.
    };
    const onUp = () => {
      splitter.classList.remove("dragging");
      document.body.style.userSelect = "";
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
    };
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
  });
})();
initAdjustControls();
initSurfaceControls();

/* ── Pop-out path preview ───────────────────────────────────────────────── */
/* Moves the whole #panel-3d (canvas + toggles + legend) into its own browser
   window for a bigger view. Same JS context, so the scene keeps updating. */
let popupWin = null;
const panelHome = panel.parentElement;   // <main>, where the panel lives normally
// Hold the node reference: once the panel (and this button with it) moves into
// the popup document, document.getElementById in the MAIN window returns null.
const btnPopout = document.getElementById("btn-popout");

btnPopout.addEventListener("click", () => {
  if (popupWin && !popupWin.closed) { popupWin.close(); popIn(); return; }
  popupWin = window.open("", "pathPreview", "width=1000,height=760");
  if (!popupWin) {
    setHeaderStatus("robot", false, "Pop-up blocked — allow pop-ups for this site.");
    return;
  }
  const d = popupWin.document;
  d.title = "Path Preview";
  const link = d.createElement("link");
  link.rel = "stylesheet";
  link.href = location.origin + "/static/style.css";
  d.head.appendChild(link);
  d.body.style.cssText = "margin:0;background:#0e0e0e;overflow:hidden;";
  d.body.appendChild(panel);
  panel.style.cssText = "position:absolute;inset:0;";
  btnPopout.textContent = "⇱ Pop in";
  popupWin.addEventListener("resize", resizeRenderer);
  popupWin.addEventListener("pagehide", popIn);   // fires when the popup closes
  setTimeout(resizeRenderer, 50);
});

function popIn() {
  if (panel.ownerDocument === document) return;   // already home
  panelHome.appendChild(panel);
  panel.style.cssText = "";                       // back to the CSS grid placement
  btnPopout.textContent = "⧉ Pop out";
  popupWin = null;
  setTimeout(resizeRenderer, 50);
}

window.addEventListener("beforeunload", () => {
  if (popupWin && !popupWin.closed) popupWin.close();
});

/* ── Run / Cancel buttons ──────────────────────────────────────────────── */
document.getElementById("btn-run").addEventListener("click", () => {
  sendWS({ type: "run", params: {
    speed_pct: parseFloat(document.getElementById("exec-speed").value) || 5,
    offset_mm: parseFloat(document.getElementById("exec-offset").value) || 0,
    safety_mm: parseFloat(document.getElementById("exec-safety").value) || 50,
    blend_mm: readBlendMm(),
  }});
  setButtonsForPhase("executing");
  setProgress(0);
  document.getElementById("val-phase").textContent = "executing";
});

document.getElementById("exec-speed").addEventListener("input", (e) => {
  document.getElementById("exec-speed-val").textContent = e.target.value + "%";
});

/* Offset / Safety / Radius change the run-time toolpath geometry (lift along
   the tool axis, retract points, corner rounding), so the preview rebuilds
   client-side on every edit. */
let execVizTimer = null;
["exec-offset", "exec-safety", "exec-blend"].forEach((id) => {
  document.getElementById(id).addEventListener("input", () => {
    if (execVizTimer) clearTimeout(execVizTimer);
    execVizTimer = setTimeout(rebuildToolpathViz, 100);
  });
});

document.getElementById("exec-blend").addEventListener("input", (e) => {
  document.getElementById("exec-blend-val").textContent = e.target.value + " mm";
});

/* Keep the server's session copy of the exec-bar values fresh so Participant
   Mode uses what this window shows — no Run/Save needed to 'commit' them —
   and a reopened window restores them (init.exec / init.detect). */
let execSyncTimer = null;
function syncExecParams() {
  if (execSyncTimer) clearTimeout(execSyncTimer);
  execSyncTimer = setTimeout(() => sendWS({ type: "set_exec_params", params: {
    speed_pct: parseFloat(document.getElementById("exec-speed").value) || 5,
    offset_mm: parseFloat(document.getElementById("exec-offset").value) || 0,
    safety_mm: parseFloat(document.getElementById("exec-safety").value) || 50,
    blend_mm: readBlendMm(),
    spacing_mm: readSpacing(),
    join_mm: readJoinMm(),
  }}), 300);
}
["exec-speed", "exec-offset", "exec-safety", "exec-blend"].forEach((id) =>
  document.getElementById(id).addEventListener("input", syncExecParams));
spacingSlider.addEventListener("change", syncExecParams);
joinBox.addEventListener("change", syncExecParams);

/* ── Save toolpath (URScript + JSON + preview image) ────────────────────── */
document.getElementById("btn-save-path").addEventListener("click", () => {
  // Capture the 3D preview as a PNG. Render first so the buffer is current.
  renderer.render(scene, camera);
  let image = null;
  try { image = renderer.domElement.toDataURL("image/png"); } catch (e) {}
  sendWS({ type: "save_path", params: {
    speed_pct: parseFloat(document.getElementById("exec-speed").value) || 5,
    offset_mm: parseFloat(document.getElementById("exec-offset").value) || 0,
    safety_mm: parseFloat(document.getElementById("exec-safety").value) || 50,
    blend_mm: readBlendMm(),
    image,
  }});
  setHeaderStatus("robot", true, "Saving toolpath…");
});

function handleSaveResult(data) {
  if (data.success) {
    setHeaderStatus("robot", true, "✓ Toolpath saved to " + data.folder);
  } else {
    setHeaderStatus("robot", false, "Save failed: " + (data.error || "unknown error"));
  }
}

document.getElementById("btn-cancel").addEventListener("click", () => {
  sendWS({ type: "cancel" });
  setHeaderStatus("robot", true, "Cancelling…");
});
