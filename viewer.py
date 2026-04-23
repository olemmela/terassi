#!/usr/bin/env python3
"""Geometrian 3D-katseluohjelma ja -editori.

Käyttö:   python3 viewer.py
Avaa:     http://localhost:5001
"""
import copy, json, os
from flask import Flask, jsonify, request, Response
import geometry_loader
import jsonschema

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_GEO_DIR    = os.path.join(_SCRIPT_DIR, "geometry")
_SCHEMA_PATH = os.path.join(_GEO_DIR, "schema.json")

app = Flask(__name__)
_schema = None


def _get_schema():
    global _schema
    if _schema is None:
        with open(_SCHEMA_PATH, encoding="utf-8") as f:
            _schema = json.load(f)
    return _schema


def _expand_patterns(geo):
    """Laajentaa pattern-jäsenet yksittäisiksi rakenneosiksi visualisointia varten."""
    for group in ("beams", "rafters", "purlins"):
        lst = geo.get("members", {}).get(group, [])
        expanded = []
        for m in lst:
            pat = m.get("pattern")
            if not pat:
                expanded.append(m)
                continue
            off  = pat["offset"]
            tmpl = pat.get("id_template", m["id"] + ".{i}")
            for i in range(pat["count"]):
                nm = copy.deepcopy(m)
                nm.pop("pattern", None)
                nm["id"] = tmpl.replace("{i}", str(i))
                for k in ("axis_start", "axis_end"):
                    if k in nm:
                        nm[k]["x"] += off["x"] * i
                        nm[k]["y"] += off["y"] * i
                        nm[k]["z"] += off["z"] * i
                expanded.append(nm)
        geo["members"][group] = expanded
    return geo


def _resolve_for_viewer(name):
    geo = geometry_loader.load(name + ".json")
    geo.pop("_points_by_id", None)
    _expand_patterns(geo)
    return geo


def _resolve_preview(name, data):
    tmp_name = f"_viewer_tmp_{name}"
    tmp_path = os.path.join(_GEO_DIR, tmp_name + ".json")
    try:
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        geo = geometry_loader.load(tmp_name + ".json")
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
    geo.pop("_points_by_id", None)
    _expand_patterns(geo)
    return geo


# ── Routes ────────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return Response(_HTML, mimetype="text/html")


@app.route("/api/geometry/<name>")
def get_geometry(name):
    try:
        return jsonify(_resolve_for_viewer(name))
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/geometry/<name>/raw")
def get_geometry_raw(name):
    path = os.path.join(_GEO_DIR, name + ".json")
    if not os.path.exists(path):
        return jsonify({"error": "Tiedostoa ei löydy"}), 404
    with open(path, encoding="utf-8") as f:
        return Response(f.read(), mimetype="application/json")


@app.route("/api/preview/<name>", methods=["POST"])
def preview_geometry(name):
    try:
        data = request.get_json(force=True)
        jsonschema.validate(data, _get_schema())
        return jsonify(_resolve_preview(name, data))
    except jsonschema.ValidationError as e:
        return jsonify({"error": f"Schema-virhe: {e.message}"}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/geometry/<name>", methods=["PUT"])
def put_geometry(name):
    path = os.path.join(_GEO_DIR, name + ".json")
    if not os.path.exists(path):
        return jsonify({"error": "Tiedostoa ei löydy"}), 404
    try:
        raw  = request.get_data(as_text=True)
        data = json.loads(raw)
        jsonschema.validate(data, _get_schema())
        _resolve_preview(name, data)       # varmistaa geometry_loader toimii
        with open(path, "w", encoding="utf-8") as f:
            f.write(raw)
            if not raw.endswith("\n"):
                f.write("\n")
        return jsonify({"ok": True})
    except json.JSONDecodeError as e:
        return jsonify({"error": f"JSON-virhe: {e}"}), 400
    except jsonschema.ValidationError as e:
        return jsonify({"error": f"Schema-virhe: {e.message}"}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/schema")
def get_schema():
    with open(_SCHEMA_PATH, encoding="utf-8") as f:
        return Response(f.read(), mimetype="application/json")


@app.route("/api/geometries")
def list_geometries():
    result = []
    for fn in sorted(os.listdir(_GEO_DIR)):
        if not fn.endswith(".json") or fn == "schema.json" or fn.startswith("_"):
            continue
        name = fn[:-5]
        path = os.path.join(_GEO_DIR, fn)
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            label = data.get("project", {}).get("name", name)
        except Exception:
            label = name
        result.append({"name": name, "label": label})
    return jsonify(result)


# ── HTML / JS / CSS ───────────────────────────────────────────────────────────

_HTML = """<!DOCTYPE html>
<html lang="fi">
<head>
<meta charset="utf-8">
<title>Geometrian katseluohjelma</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: system-ui, sans-serif; background: #1e1e1e; color: #ccc;
         display: flex; flex-direction: column; height: 100vh; overflow: hidden; }

  #toolbar {
    display: flex; align-items: center; gap: 10px; padding: 8px 12px;
    background: #2d2d2d; border-bottom: 1px solid #444; flex-shrink: 0;
  }
  .btn-tab { padding: 4px 16px; border: 1px solid #555; background: #3a3a3a;
    color: #ccc; cursor: pointer; border-radius: 4px; font-size: 13px; }
  .btn-tab.active  { background: #0066cc; border-color: #0088ff; color: #fff; }
  .btn-tab.editing { box-shadow: inset 0 -3px 0 0 #fff; font-weight: bold; }
  .btn-save { padding: 4px 16px; border: 1px solid #555; background: #3a3a3a;
    color: #ccc; cursor: pointer; border-radius: 4px; font-size: 13px;
    margin-left: auto; transition: background .2s; }
  .btn-save:hover  { background: #0055aa; }
  .btn-save.saved  { background: #226622; border-color: #44aa44; }
  .btn-save.error  { background: #662222; border-color: #aa4444; }

  #status { font-size: 12px; color: #aaa; max-width: 500px;
    white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  #status.ok  { color: #66cc66; }
  #status.err { color: #ff6666; }

  #main { display: flex; flex: 1; overflow: hidden; }

  #editor-panel { width: 420px; min-width: 180px; display: flex;
    flex-direction: column; flex-shrink: 0; }
  #editor { flex: 1; resize: none; background: #1e1e1e; color: #d4d4d4;
    border: none; border-right: 1px solid #444; outline: none; padding: 10px;
    font-family: 'Consolas', 'Monaco', monospace; font-size: 12px;
    line-height: 1.5; tab-size: 2; overflow: auto; }

  #resizer { width: 4px; background: #444; cursor: col-resize; flex-shrink: 0; }
  #resizer:hover, #resizer.dragging { background: #0066cc; }

  #viewport { flex: 1; position: relative; overflow: hidden; }
  canvas { display: block; }

  #tooltip { position: absolute; pointer-events: none;
    background: rgba(0,0,0,.8); color: #fff; padding: 4px 9px;
    border-radius: 4px; font-size: 12px; display: none; }

  #legend { position: absolute; bottom: 10px; right: 10px;
    background: rgba(0,0,0,.65); padding: 8px 12px; border-radius: 6px;
    font-size: 11px; line-height: 1.8; }
  .li { display: flex; align-items: center; gap: 7px; }
  .ld { width: 12px; height: 12px; border-radius: 2px; flex-shrink: 0; }
</style>
</head>
<body>
<div id="toolbar">
  <span id="status">Ladataan...</span>
  <button class="btn-save" id="btn-save">&#128190; Tallenna</button>
</div>
<div id="main">
  <div id="editor-panel">
    <textarea id="editor" spellcheck="false"></textarea>
  </div>
  <div id="resizer"></div>
  <div id="viewport">
    <div id="tooltip"></div>
    <div id="legend">
      <div class="li"><div class="ld" style="background:#888"></div>Pilari</div>
      <div class="li"><div class="ld" style="background:#2266cc"></div>Palkki</div>
      <div class="li"><div class="ld" style="background:#22aa44"></div>Kattotuoli</div>
      <div class="li"><div class="ld" style="background:#dd8800"></div>Orsi</div>
      <div class="li"><div class="ld" style="background:rgba(0,160,160,.6)"></div>Lasipinta</div>
      <div class="li"><div class="ld" style="background:rgba(60,140,60,.6)"></div>Kattopinta</div>
      <div class="li"><div class="ld" style="background:rgba(150,130,100,.3)"></div>Viitepinta</div>
      <div class="li"><div class="ld" style="background:#ff4444;border-radius:50%"></div>Tuki (supported_on)</div>
      <div class="li"><div class="ld" style="background:#ff8800;border-radius:50%"></div>Seinäkiinnitys</div>
      <div class="li"><div class="ld" style="background:#ffcc00;border-radius:50%"></div>Pilarijalkalevy</div>
      <div class="li"><div class="ld" style="background:#aa44ff;border-radius:50%"></div>Sivutuki (lateral)</div>
    </div>
  </div>
</div>

<script type="importmap">
{
  "imports": {
    "three": "https://cdn.jsdelivr.net/npm/three@0.161.0/build/three.module.js",
    "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.161.0/examples/jsm/"
  }
}
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ── Renderer & scene ──────────────────────────────────────────────────────────
const vp = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x1a1a2e);
vp.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, 1, 1, 200000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.06;

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const dl = new THREE.DirectionalLight(0xffffff, 0.6);
dl.position.set(-5000, 8000, 5000);
scene.add(dl);

const grid = new THREE.GridHelper(16000, 32, 0x333333, 0x282828);
scene.add(grid);
scene.add(new THREE.AxesHelper(600));

function resize() {
  const w = vp.clientWidth, h = vp.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
resize();
new ResizeObserver(resize).observe(vp);

(function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();

// ── Coord conversion: geometry (X right, Y out, Z up) → Three.js (Y up)
// pt = (x, z, y): X oikealle, Z (korkeus) → Y ylös, Y (ulospäin) → Z syvyys.
// Vasenkätinen systeemi: DoubleSide-materiaali korjaa face-culling-ongelmat.
const pt = p => new THREE.Vector3(p.x, p.z, p.y);

// ── Colors ────────────────────────────────────────────────────────────────────
const MCOL = { columns: 0x888888, beams: 0x2266cc, rafters: 0x22aa44, purlins: 0xdd8800 };
const SCOL = {
  roof_covering: 0x3c8c3c, solar_panel_array: 0x1155cc,
  side_glazing: 0x00aaaa, triangle_glazing: 0x00bbbb, gable_glazing: 0x00cccc,
  opening: 0xddcc00, boarding: 0x7a6040, purlin_layer: 0x888866,
  building_wall: 0x998877, building_roof: 0x887766,
};
const CCOL = {
  supported_on:  0xff4444,
  wall_bolted:   0xff8800,
  column_base:   0xffcc00,
  lateral_brace: 0xaa44ff,
};

// ── Scene groups (yksi per ladattu geometria) ─────────────────────────────────
const geoGroups = new Map(); // name → THREE.Group
let pickable = [];

function clearGeoFor(name) {
  const g = geoGroups.get(name);
  if (!g) return;
  scene.remove(g);
  g.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
  geoGroups.delete(name);
  pickable = pickable.filter(o => o.userData._geoName !== name);
}

// ── Member box helper ─────────────────────────────────────────────────────────
function memberQuaternion(start, end, strongAxis) {
  // Returns quaternion to align BoxGeometry's X axis with (start→end),
  // with Y dimension (h_mm) pointing as vertically as possible.
  const dir = end.clone().sub(start).normalize();
  const worldUp = new THREE.Vector3(0, 1, 0); // Three.js Y = geometry Z (up)
  const isVertical = Math.abs(dir.dot(worldUp)) > 0.95;
  // Choose reference up: if beam is nearly vertical (column), use X as ref
  const refUp = isVertical ? new THREE.Vector3(1, 0, 0) : worldUp;
  const zAx = new THREE.Vector3().crossVectors(dir, refUp).normalize();
  if (zAx.lengthSq() < 1e-8) return new THREE.Quaternion();
  const yAx = new THREE.Vector3().crossVectors(zAx, dir).normalize();
  return new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().makeBasis(dir, yAx, zAx)
  );
}

function addMemberMesh(g, pk, start3, end3, profile, strongAxis, color, userData) {
  const len  = start3.distanceTo(end3);
  if (len < 1) return;

  let hMm = profile?.h_mm, bMm = profile?.b_mm, cnt = profile?.count ?? 1;

  if (!hMm || !bMm) {
    // TBD profile: draw as line
    const lineGeo = new THREE.BufferGeometry().setFromPoints([start3, end3]);
    const line = new THREE.Line(lineGeo, new THREE.LineDashedMaterial({ color, dashSize: 60, gapSize: 40 }));
    line.computeLineDistances();
    line.userData = userData;
    g.add(line); pk.push(line);
    return;
  }

  // strong_axis='vertical' → h_mm is the vertical dimension
  // strong_axis='horizontal' → h_mm is horizontal, b_mm is vertical
  const [boxH, boxB] = strongAxis === 'horizontal'
    ? [bMm, hMm]
    : [hMm, bMm * cnt];

  const q   = memberQuaternion(start3, end3, strongAxis);
  const mid = start3.clone().add(end3).multiplyScalar(0.5);

  const boxGeo = new THREE.BoxGeometry(len, boxH, boxB);
  const mat = new THREE.MeshLambertMaterial({ color, transparent: true, opacity: 0.85, side: THREE.DoubleSide });
  const mesh   = new THREE.Mesh(boxGeo, mat);
  mesh.position.copy(mid);
  mesh.quaternion.copy(q);
  mesh.userData = userData;
  g.add(mesh); pk.push(mesh);

  // Edges for clarity
  const edgeMat = new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.4 });
  const edges   = new THREE.LineSegments(new THREE.EdgesGeometry(boxGeo), edgeMat);
  edges.position.copy(mid);
  edges.quaternion.copy(q);
  g.add(edges);
}

// ── Members ───────────────────────────────────────────────────────────────────
function addMembers(g, pk, geo) {
  for (const [grpName, lst] of Object.entries(geo.members ?? {})) {
    const col = MCOL[grpName] ?? 0xffffff;
    for (const m of lst ?? []) {
      const a = grpName === 'columns' ? m.base       : m.axis_start;
      const b = grpName === 'columns' ? m.top        : m.axis_end;
      if (!a || !b) continue;
      addMemberMesh(g, pk, pt(a), pt(b), m.profile, m.strong_axis, col,
        { id: m.id, kind: grpName, _geoName: g.userData.geoName });
    }
  }
}

// ── Surface meshes ────────────────────────────────────────────────────────────
function addSurfaces(g, pk, list, opacity) {
  const geoName = g.userData.geoName;
  for (const s of list ?? []) {
    const poly = s.polygon;
    if (!poly || poly.length < 3) continue;
    const col = SCOL[s.type] ?? 0xaaaaaa;

    // Fan triangulation (works for convex polygons)
    const pos = [];
    for (let i = 1; i < poly.length - 1; i++) {
      [poly[0], poly[i], poly[i+1]].forEach(p => { const v = pt(p); pos.push(v.x, v.y, v.z); });
    }
    const mg = new THREE.BufferGeometry();
    mg.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    mg.computeVertexNormals();
    const mesh = new THREE.Mesh(mg, new THREE.MeshBasicMaterial(
      { color: col, transparent: true, opacity, side: THREE.DoubleSide }
    ));
    mesh.userData = { id: s.id, kind: s.type ?? 'surface', _geoName: geoName };
    g.add(mesh); pk.push(mesh);

    // Outline
    const outPts = [...poly.map(pt), pt(poly[0])];
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(outPts),
      new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: Math.min(opacity * 2, 1) })
    );
    line.userData = { id: s.id, kind: s.type ?? 'surface', _geoName: geoName };
    g.add(line);
  }
}

// ── Connection markers ────────────────────────────────────────────────────────
function addConnections(g, pk, connections) {
  const sphereGeo = new THREE.SphereGeometry(80, 12, 8);
  for (const con of connections ?? []) {
    if (!con.at) continue;
    const col = CCOL[con.type] ?? 0xffffff;
    const mesh = new THREE.Mesh(
      sphereGeo,
      new THREE.MeshBasicMaterial({ color: col })
    );
    mesh.position.copy(pt(con.at));
    mesh.userData = { id: con.id, kind: con.type ?? 'connection', _geoName: g.userData.geoName };
    g.add(mesh);
    pk.push(mesh);
  }
}

// ── Fit camera ────────────────────────────────────────────────────────────────
function fitCamera() {
  const box = new THREE.Box3();
  geoGroups.forEach(g => g.traverse(o => { if (o.geometry) box.expandByObject(o); }));
  if (box.isEmpty()) return;
  const c = new THREE.Vector3(); box.getCenter(c);
  const s = new THREE.Vector3(); box.getSize(s);
  const d = Math.max(s.x, s.y, s.z) * 1.6;
  camera.position.set(c.x + d * 0.3, c.y + d * 0.5, c.z + d);
  camera.lookAt(c);
  controls.target.copy(c);
  controls.update();
}

// ── Render resolved geometry into a named group ───────────────────────────────
function renderGeoFor(name, resolved) {
  clearGeoFor(name);
  const g = new THREE.Group();
  g.userData.geoName = name;
  scene.add(g);
  geoGroups.set(name, g);
  addMembers(g, pickable, resolved);
  addSurfaces(g, pickable, resolved.surfaces, 0.30);
  addSurfaces(g, pickable, resolved.reference_surfaces, 0.12);
  addConnections(g, pickable, resolved.connections);
  fitCamera();
}

// ── Tooltip via raycasting ────────────────────────────────────────────────────
const tooltip  = document.getElementById('tooltip');
const raycaster = new THREE.Raycaster();
raycaster.params.Line.threshold = 25;
const mouse = new THREE.Vector2();

renderer.domElement.addEventListener('mousemove', e => {
  const r = renderer.domElement.getBoundingClientRect();
  mouse.x =  ((e.clientX - r.left) / r.width)  * 2 - 1;
  mouse.y = -((e.clientY - r.top)  / r.height) * 2 + 1;
  raycaster.setFromCamera(mouse, camera);
  const hits = raycaster.intersectObjects(pickable, false);
  if (hits.length) {
    const { id, kind } = hits[0].object.userData;
    tooltip.style.display = 'block';
    tooltip.style.left = (e.clientX - r.left + 14) + 'px';
    tooltip.style.top  = (e.clientY - r.top  - 28) + 'px';
    tooltip.textContent = id + (kind ? '  —  ' + kind : '');
  } else {
    tooltip.style.display = 'none';
  }
});

// ── State ─────────────────────────────────────────────────────────────────────
const activeNames = new Set();  // geometriat jotka näytetään sceneissä
let editingName = null;         // mikä on editorissa auki
const editor   = document.getElementById('editor');
const statusEl = document.getElementById('status');
let debounce = null;

function setStatus(msg, cls) { statusEl.textContent = msg; statusEl.className = cls ?? ''; }

function updateTabStyles() {
  for (const el of document.querySelectorAll('.btn-tab[data-geo]')) {
    const n = el.dataset.geo;
    el.classList.toggle('active',  activeNames.has(n));
    el.classList.toggle('editing', n === editingName);
  }
}

// ── Load & render ─────────────────────────────────────────────────────────────
async function loadGeometry(name) {
  editingName = name;
  activeNames.add(name);
  updateTabStyles();
  setStatus('Ladataan…', '');
  try {
    const [rawRes, resolvedRes] = await Promise.all([
      fetch('/api/geometry/' + name + '/raw'),
      fetch('/api/geometry/' + name),
    ]);
    editor.value = await rawRes.text();
    const data = await resolvedRes.json();
    if (data.error) throw new Error(data.error);
    renderGeoFor(name, data);
    setStatus('OK', 'ok');
  } catch(e) { setStatus('Virhe: ' + e.message, 'err'); }
}

// ── Tab toggle: useampi geometria yhtä aikaa näkyviin ─────────────────────────
async function toggleGeometry(name) {
  if (!activeNames.has(name)) {
    // Lisää sceneen ja aseta editoriin
    await loadGeometry(name);
  } else if (editingName !== name) {
    // Näkyy jo, vaihda vain editoitava
    editingName = name;
    updateTabStyles();
    setStatus('Ladataan…', '');
    try {
      const raw = await fetch('/api/geometry/' + name + '/raw');
      editor.value = await raw.text();
      setStatus('OK', 'ok');
    } catch(e) { setStatus('Virhe: ' + e.message, 'err'); }
  } else {
    // Klikattu uudelleen: piilota scenestä
    activeNames.delete(name);
    clearGeoFor(name);
    fitCamera();
    editingName = [...activeNames][0] ?? null;
    if (editingName) {
      try {
        const raw = await fetch('/api/geometry/' + editingName + '/raw');
        editor.value = await raw.text();
      } catch(_) {}
    } else {
      editor.value = '';
    }
    updateTabStyles();
  }
}

// ── Live preview ──────────────────────────────────────────────────────────────
editor.addEventListener('input', () => {
  clearTimeout(debounce);
  debounce = setTimeout(async () => {
    if (!editingName) return;
    let parsed;
    try { parsed = JSON.parse(editor.value); }
    catch(e) { setStatus('JSON-virhe: ' + e.message, 'err'); return; }
    try {
      const res  = await fetch('/api/preview/' + editingName, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(parsed),
      });
      const data = await res.json();
      if (data.error) { setStatus(data.error, 'err'); return; }
      renderGeoFor(editingName, data);
      setStatus('Esikatselu ✓', 'ok');
    } catch(e) { setStatus('Esikatselu virhe: ' + e.message, 'err'); }
  }, 800);
});

// ── Save ──────────────────────────────────────────────────────────────────────
async function saveGeometry() {
  if (!editingName) return;
  let parsed;
  try { parsed = JSON.parse(editor.value); }
  catch(e) { setStatus('JSON-virhe: ' + e.message, 'err'); return; }
  const btn = document.getElementById('btn-save');
  btn.textContent = '⏳ Tallennetaan…';
  try {
    const res  = await fetch('/api/geometry/' + editingName, {
      method: 'PUT', headers: { 'Content-Type': 'application/json' },
      body: editor.value,
    });
    const data = await res.json();
    if (data.error) {
      setStatus(data.error, 'err');
      btn.textContent = '💾 Tallenna'; btn.className = 'btn-save error';
      setTimeout(() => { btn.className = 'btn-save'; }, 3000);
    } else {
      setStatus('Tallennettu', 'ok');
      btn.textContent = '✓ Tallennettu'; btn.className = 'btn-save saved';
      setTimeout(() => { btn.textContent = '💾 Tallenna'; btn.className = 'btn-save'; }, 2000);
    }
  } catch(e) {
    setStatus('Tallennus epäonnistui: ' + e.message, 'err');
    btn.textContent = '💾 Tallenna'; btn.className = 'btn-save error';
    setTimeout(() => { btn.className = 'btn-save'; }, 3000);
  }
}

// ── Resizer ───────────────────────────────────────────────────────────────────
const resizerEl  = document.getElementById('resizer');
const editorPanel = document.getElementById('editor-panel');
const mainEl     = document.getElementById('main');
let dragging = false;
resizerEl.addEventListener('mousedown', e => { dragging = true; resizerEl.classList.add('dragging'); e.preventDefault(); });
window.addEventListener('mousemove', e => {
  if (!dragging) return;
  const x = e.clientX - mainEl.getBoundingClientRect().left;
  editorPanel.style.width = Math.max(180, Math.min(x, mainEl.clientWidth - 250)) + 'px';
});
window.addEventListener('mouseup', () => { dragging = false; resizerEl.classList.remove('dragging'); });

// ── Boot: load geometry list and wire up buttons ──────────────────────────────
(async () => {
  const toolbar  = document.getElementById('toolbar');
  const statusEl = document.getElementById('status');
  const saveBtn  = document.getElementById('btn-save');
  let   first    = null;

  try {
    const list = await (await fetch('/api/geometries')).json();
    for (const { name, label } of list) {
      const btn = document.createElement('button');
      btn.className   = 'btn-tab';
      btn.id          = 'tab-' + name;
      btn.dataset.geo = name;
      btn.textContent = label;
      btn.addEventListener('click', () => toggleGeometry(name));
      toolbar.insertBefore(btn, statusEl);
      if (!first) first = name;
    }
  } catch(e) {
    setStatus('Geometrioiden lataus epäonnistui: ' + e.message, 'err');
  }

  saveBtn.addEventListener('click', saveGeometry);
  if (first) loadGeometry(first);
})();
</script>
</body>
</html>"""


if __name__ == "__main__":
    import webbrowser, threading
    print("Geometrian katseluohjelma käynnistyy → http://localhost:5001")
    threading.Timer(0.8, lambda: webbrowser.open("http://localhost:5001")).start()
    app.run(port=5001, debug=False)
