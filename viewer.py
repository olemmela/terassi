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


def _collect_pattern_specs(geo):
    specs = {}
    for group in ("beams", "rafters", "purlins"):
        for m in geo.get("members", {}).get(group, []):
            pat = m.get("pattern")
            if not pat:
                continue
            id_template = pat.get("id_template", m["id"] + ".{i}")
            specs[m["id"]] = {
                "count": int(pat["count"]),
                "offset": dict(pat["offset"]),
                "id_template": id_template,
                "first_instance_id": id_template.replace("{i}", "0"),
            }
    return specs


def _should_expand_connection_member(con, pattern_member, spec):
    # If the pattern root ID is also the first real instance ID (e.g. "kattotuoli.0"),
    # an explicit connection may legitimately target only that single instance.
    # In that ambiguous case, only auto-expand when the connection ID itself stays generic.
    if pattern_member == spec["first_instance_id"] and pattern_member in con.get("id", ""):
        return False
    return True


def _expand_connection_patterns(geo, pattern_specs):
    """Laajentaa pattern-jäseniin viittaavat liitokset yksittäisiksi instansseiksi."""
    expanded = []
    for con in geo.get("connections", []):
        pattern_member = next(
            (
                mid for mid in con.get("members", [])
                if mid in pattern_specs and _should_expand_connection_member(con, mid, pattern_specs[mid])
            ),
            None,
        )
        if not pattern_member:
            expanded.append(con)
            continue

        spec = pattern_specs[pattern_member]
        off = spec["offset"]
        tmpl = spec["id_template"]
        for i in range(spec["count"]):
            nc = copy.deepcopy(con)
            nc["id"] = f"{con['id']}.{i}"
            nc["members"] = [
                tmpl.replace("{i}", str(i)) if mid == pattern_member else mid
                for mid in con["members"]
            ]
            if "at" in nc:
                nc["at"]["x"] += off["x"] * i
                nc["at"]["y"] += off["y"] * i
                nc["at"]["z"] += off["z"] * i
            expanded.append(nc)
    geo["connections"] = expanded
    return geo


def _resolve_for_viewer(name):
    geo = geometry_loader.load(name + ".json")
    geo.pop("_points_by_id", None)
    _expand_connection_patterns(geo, _collect_pattern_specs(geo))
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
    _expand_connection_patterns(geo, _collect_pattern_specs(geo))
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
  .btn-toggle { padding: 4px 16px; border: 1px solid #555; background: #3a3a3a;
    color: #ccc; cursor: pointer; border-radius: 4px; font-size: 13px;
    transition: background .2s, border-color .2s; }
  #btn-connections { margin-left: auto; }
  .btn-toggle:hover  { background: #4a4a4a; }
  .btn-toggle.active { background: #2b5d8a; border-color: #6ab0ff; color: #fff; }
  .btn-save { padding: 4px 16px; border: 1px solid #555; background: #3a3a3a;
    color: #ccc; cursor: pointer; border-radius: 4px; font-size: 13px;
    transition: background .2s; }
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

  #viewport { flex: 1; position: relative; overflow: hidden; outline: none; }
  #viewport:focus { box-shadow: inset 0 0 0 2px rgba(0,136,255,.55); }
  canvas { display: block; }

  #tooltip { position: absolute; pointer-events: none;
    background: rgba(0,0,0,.8); color: #fff; padding: 6px 9px;
    border-radius: 4px; font-size: 12px; display: none; white-space: pre-line;
    line-height: 1.35; max-width: 420px; }

  #legend { position: absolute; bottom: 10px; right: 10px;
    background: rgba(0,0,0,.65); padding: 8px 12px; border-radius: 6px;
    font-size: 11px; line-height: 1.8; }
  .li { display: flex; align-items: center; gap: 7px; }
  .ld { width: 12px; height: 12px; border-radius: 2px; flex-shrink: 0; }
  .ld-dot { border-radius: 50%; background: currentColor; }
  .ld-shift {
    width: 16px; height: 2px; background: currentColor; border-radius: 0; position: relative;
  }
  .ld-shift::after {
    content: ''; position: absolute; right: -1px; top: -4px; width: 6px; height: 10px;
    border: 1px solid currentColor; border-radius: 1px;
  }
</style>
</head>
<body>
<div id="toolbar">
  <span id="status">Ladataan...</span>
  <button class="btn-toggle" id="btn-connections" aria-pressed="false">&#9675; Liitospisteet</button>
  <button class="btn-toggle" id="btn-analysis" aria-pressed="false">&#9675; Analyysi-overlayt</button>
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
      <div class="li"><div class="ld" style="background:#66ccff;border-radius:50%"></div>Toistuva tuki (pattern)</div>
      <div class="li"><div class="ld" style="background:#66ffee;border-radius:50%"></div>Jatkuva liitos</div>
      <div class="li"><div class="ld" style="background:#ff66cc;border-radius:50%"></div>Loveus / leikkaus</div>
      <div class="li"><div class="ld" style="background:#ff8800;border-radius:50%"></div>Seinäkiinnitys</div>
      <div class="li"><div class="ld" style="background:#ffcc00;border-radius:50%"></div>Pilarijalkalevy</div>
      <div class="li"><div class="ld" style="background:#aa44ff;border-radius:50%"></div>Sivutuki (lateral)</div>
      <div class="li"><div class="ld ld-dot" style="color:#c8ccd4"></div>Analyysipiste: nivel</div>
      <div class="li"><div class="ld ld-dot" style="color:#ffb347"></div>Analyysipiste: puolijäykkä</div>
      <div class="li"><div class="ld ld-dot" style="color:#66ddff"></div>Analyysipiste: jäykkä</div>
      <div class="li"><div class="ld ld-shift" style="color:#99ff33"></div>Tukilinjan siirtymä (viiva + levy)</div>
      <div class="li"><div class="ld" style="background:#ffee66"></div>Hover kattopintaan → member_refs-korostus</div>
      <div class="li"><div class="ld" style="background:#666"></div>Klikkaa 3D-näkymää → nuolilla liike, Ctrl+nuoli kääntö</div>
    </div>
  </div>
</div>

<script type="importmap">
{
  "imports": {
    "three": "https://cdn.jsdelivr.net/npm/three@0.161.0/build/three.module.js",
    "three/addons/": "https://cdn.jsdelivr.net/npm/three@0.161.0/examples/jsm/",
    "three-mesh-bvh": "https://cdn.jsdelivr.net/npm/three-mesh-bvh@0.7.8/src/index.js",
    "three-bvh-csg": "https://cdn.jsdelivr.net/npm/three-bvh-csg@0.0.16/src/index.js"
  }
}
</script>
<script type="module">
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { mergeVertices } from 'three/addons/utils/BufferGeometryUtils.js';

// CSG boolean operations – loaded lazily so viewer works even if CDN is down
let _csgMod = null;
let _csgTried = false;
async function getCSG() {
  if (_csgTried) return _csgMod;
  _csgTried = true;
  try {
    _csgMod = await import('three-bvh-csg');
  } catch (e) {
    console.warn('CSG library not available, notch overlays used as fallback:', e);
  }
  return _csgMod;
}

// ── Renderer & scene ──────────────────────────────────────────────────────────
const vp = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x1a1a2e);
vp.appendChild(renderer.domElement);
vp.tabIndex = 0;
vp.setAttribute('aria-label', '3D-näkymä');

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, 1, 1, 200000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.06;
vp.addEventListener('pointerdown', () => vp.focus({ preventScroll: true }));

const KEY_MOVE_STEP_MM = 120;
const KEY_ROTATE_STEP_RAD = THREE.MathUtils.degToRad(3);
const WORLD_UP = new THREE.Vector3(0, 1, 0);

function translateViewLocal(forwardMm, strafeMm = 0) {
  const forward = new THREE.Vector3();
  camera.getWorldDirection(forward);
  if (forward.lengthSq() < 1e-12) return;
  forward.normalize();

  const right = new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion).normalize();
  const delta = forward.multiplyScalar(forwardMm).add(right.multiplyScalar(strafeMm));
  camera.position.add(delta);
  controls.target.add(delta);
  controls.update();
}

function rotateViewInPlace(yawRad = 0, pitchRad = 0) {
  const offset = controls.target.clone().sub(camera.position);
  if (offset.lengthSq() < 1e-12) return;

  if (Math.abs(yawRad) > 1e-12) {
    offset.applyAxisAngle(WORLD_UP, yawRad);
  }
  if (Math.abs(pitchRad) > 1e-12) {
    const right = new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion).normalize();
    offset.applyAxisAngle(right, pitchRad);
  }

  controls.target.copy(camera.position.clone().add(offset));
  controls.update();
}

vp.addEventListener('keydown', e => {
  const moveStep = e.shiftKey ? KEY_MOVE_STEP_MM * 5 : KEY_MOVE_STEP_MM;
  const rotateStep = e.shiftKey ? KEY_ROTATE_STEP_RAD * 3 : KEY_ROTATE_STEP_RAD;

  if (e.ctrlKey || e.metaKey) {
    switch (e.key) {
      case 'ArrowLeft':
        rotateViewInPlace(+rotateStep, 0);
        break;
      case 'ArrowRight':
        rotateViewInPlace(-rotateStep, 0);
        break;
      case 'ArrowUp':
        rotateViewInPlace(0, +rotateStep);
        break;
      case 'ArrowDown':
        rotateViewInPlace(0, -rotateStep);
        break;
      default:
        return;
    }
  } else {
    switch (e.key) {
      case 'ArrowLeft':
        translateViewLocal(0, -moveStep);
        break;
      case 'ArrowRight':
        translateViewLocal(0, +moveStep);
        break;
      case 'ArrowUp':
        translateViewLocal(+moveStep, 0);
        break;
      case 'ArrowDown':
        translateViewLocal(-moveStep, 0);
        break;
      default:
        return;
    }
  }
  e.preventDefault();
});

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
  building_wall: 0x998877, building_roof: 0x887766, floor: 0x777755,
};
const CCOL = {
  supported_on:  0xff4444,
  supported_by_pattern: 0x66ccff,
  continuous: 0x66ffee,
  notched_over:  0xff66cc,
  wall_bolted:   0xff8800,
  column_base:   0xffcc00,
  lateral_brace: 0xaa44ff,
};
const ANALYSIS_SUPPORT_MODEL_COLORS = {
  pinned: 0xc8ccd4,
  semi_rigid: 0xffb347,
  rigid: 0x66ddff,
};
const SUPPORT_MODEL_LABELS = {
  pinned: 'nivel',
  semi_rigid: 'puolijäykkä',
  rigid: 'jäykkä',
};
const REFERENCE_LABELS = {
  axis_start: 'axis_start-pää',
  axis_end: 'axis_end-pää',
  member_end: 'jäsenen ulkopää',
  support_centerline: 'tukikeskilinja',
  support_inner_edge: 'tuen sisäreuna',
  support_outer_edge: 'tuen ulkoreuna',
};
const HIGHLIGHT_MEMBER_COLOR = 0xffee66;
const SUPPORT_LINE_SHIFT_COLOR = 0x99ff33;

// ── Scene groups (yksi per ladattu geometria) ─────────────────────────────────
const geoGroups = new Map(); // name → THREE.Group
const memberVisuals = new Map(); // geo::memberId → Array<THREE.Object3D>
let pickable = [];
let showConnectionMarkers = false;
let showAnalysisOverlays = false;
let highlightedMemberKeys = new Set();

function memberVisualKey(geoName, memberId) {
  return `${geoName}::${memberId}`;
}

function registerMemberVisual(geoName, memberId, object) {
  if (!geoName || !memberId || !object) return;
  const key = memberVisualKey(geoName, memberId);
  if (!memberVisuals.has(key)) memberVisuals.set(key, []);
  if (object.material?.color && object.userData._baseColor == null) {
    object.userData._baseColor = object.material.color.getHex();
  }
  if (object.material?.emissive && object.userData._baseEmissive == null) {
    object.userData._baseEmissive = object.material.emissive.getHex();
  }
  if (object.material?.opacity != null && object.userData._baseOpacity == null) {
    object.userData._baseOpacity = object.material.opacity;
  }
  memberVisuals.get(key).push(object);
}

function updateObjectHighlight(object, active) {
  const mat = object.material;
  if (!mat) return;
  if (active) {
    if (mat.color) mat.color.setHex(HIGHLIGHT_MEMBER_COLOR);
    if (mat.emissive) mat.emissive.setHex(0x665500);
    if (mat.opacity != null) mat.opacity = Math.max(object.userData._baseOpacity ?? 1, 0.95);
    return;
  }
  if (mat.color && object.userData._baseColor != null) mat.color.setHex(object.userData._baseColor);
  if (mat.emissive && object.userData._baseEmissive != null) mat.emissive.setHex(object.userData._baseEmissive);
  if (mat.opacity != null && object.userData._baseOpacity != null) mat.opacity = object.userData._baseOpacity;
}

function setHighlightedMemberKeys(nextKeys) {
  for (const key of highlightedMemberKeys) {
    if (nextKeys.has(key)) continue;
    for (const object of memberVisuals.get(key) ?? []) updateObjectHighlight(object, false);
  }
  for (const key of nextKeys) {
    if (highlightedMemberKeys.has(key)) continue;
    for (const object of memberVisuals.get(key) ?? []) updateObjectHighlight(object, true);
  }
  highlightedMemberKeys = nextKeys;
}

function clearHighlightedMembers() {
  setHighlightedMemberKeys(new Set());
}

function setHighlightedMembersForSurface(geoName, memberRefs) {
  if (!showAnalysisOverlays || !geoName || !Array.isArray(memberRefs) || memberRefs.length === 0) {
    clearHighlightedMembers();
    return;
  }
  const keys = new Set(memberRefs.map(memberId => memberVisualKey(geoName, memberId)));
  setHighlightedMemberKeys(keys);
}

function clearGeoMemberVisuals(geoName) {
  for (const key of [...memberVisuals.keys()]) {
    if (key.startsWith(`${geoName}::`)) memberVisuals.delete(key);
  }
}

function createAnalysisMarker(color) {
  return new THREE.Mesh(
    new THREE.SphereGeometry(28, 12, 10),
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 1.0,
      depthWrite: false,
      depthTest: false,
    })
  );
}

function summarizeMemberRefs(memberRefs, limit = 4) {
  if (!Array.isArray(memberRefs) || memberRefs.length === 0) return 'ei kohdejäseniä';
  if (memberRefs.length <= limit) return memberRefs.join(', ');
  return `${memberRefs.slice(0, limit).join(', ')}, +${memberRefs.length - limit} muuta`;
}

function rotationSpringSummary(analysis) {
  const spring = analysis?.rotation_spring;
  if (!spring) return null;
  if (spring.model === 'explicit' && spring.k_theta_Nmm_per_rad != null) {
    return `kθ ${(spring.k_theta_Nmm_per_rad / 1e6).toFixed(1)} kNm/rad`;
  }
  if (spring.model === 'ec5_fasteners') {
    return `EC5-jousi d${spring.fastener_d_mm ?? '?'} n${spring.fastener_count ?? '?'}`;
  }
  return spring.model ?? null;
}

function connectionTooltipLines(con) {
  const lines = [`${con.id}  —  ${con.type ?? 'connection'}`];
  const analysis = con.analysis;
  if (analysis) {
    const supportModel = analysis.support_model ?? 'pinned';
    lines.push(`tuki: ${SUPPORT_MODEL_LABELS[supportModel] ?? supportModel}`);
    if (analysis.support_line_ref) {
      lines.push(`tukilinja: ${REFERENCE_LABELS[analysis.support_line_ref] ?? analysis.support_line_ref}`);
    }
    if (analysis.label) {
      lines.push(
        analysis.fastener_label
          ? `liitos: ${analysis.label}, ${analysis.fastener_label}`
          : `liitos: ${analysis.label}`
      );
    }
    const springSummary = rotationSpringSummary(analysis);
    if (springSummary) lines.push(`rotaatio: ${springSummary}`);
  }
  const cuts = connectionCuts(con);
  if (cuts.length) lines.push(`cuts: ${cuts.map(cut => cut.kind).join(', ')}`);
  return lines;
}

function loadTransferRuleSummary(rule) {
  const model = rule.model === 'partial_uniform'
    ? `osaviivakuorma ${rule.length_mm} mm`
    : rule.model === 'uniform'
      ? 'viivakuorma'
      : 'pistekuorma';
  const reference = REFERENCE_LABELS[rule.reference] ?? rule.reference ?? 'jäsen';
  const offset = rule.offset_mm ?? 0;
  return `${model}, ${offset} mm ${reference} → ${rule.member_refs?.length ?? 0} jäsentä (${summarizeMemberRefs(rule.member_refs)})`;
}

function surfaceTooltipLines(surfaceObj) {
  const lines = [`${surfaceObj.id}  —  ${surfaceObj.type ?? 'surface'}`];
  const rules = surfaceObj.load_transfer?.to_members;
  if (Array.isArray(rules) && rules.length) {
    for (const rule of rules) lines.push(`load_transfer: ${loadTransferRuleSummary(rule)}`);
  }
  return lines;
}

function clearGeoFor(name) {
  clearHighlightedMembers();
  clearGeoMemberVisuals(name);
  const g = geoGroups.get(name);
  if (!g) return;
  scene.remove(g);
  g.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
  geoGroups.delete(name);
  pickable = pickable.filter(o => o.userData._geoName !== name);
}

function setConnectionMarkerVisibility(show) {
  showConnectionMarkers = show;
  const btn = document.getElementById('btn-connections');
  if (btn) {
    btn.classList.toggle('active', show);
    btn.setAttribute('aria-pressed', String(show));
    btn.textContent = show ? '● Liitospisteet' : '○ Liitospisteet';
  }
  for (const g of geoGroups.values()) {
    g.traverse(o => {
      if (o.userData?._isConnectionMarker) o.visible = show;
      if (o.userData?._isCutMember) o.visible = !show;
      if (o.userData?._isUncutMember) o.visible = show;
      if (o.userData?._isNotchOverlay) o.visible = show;
    });
  }
}

function toggleConnectionMarkers() {
  setConnectionMarkerVisibility(!showConnectionMarkers);
}

function setAnalysisOverlayVisibility(show) {
  showAnalysisOverlays = show;
  const btn = document.getElementById('btn-analysis');
  if (btn) {
    btn.classList.toggle('active', show);
    btn.setAttribute('aria-pressed', String(show));
    btn.textContent = show ? '● Analyysi-overlayt' : '○ Analyysi-overlayt';
  }
  if (!show) clearHighlightedMembers();
  for (const g of geoGroups.values()) {
    g.traverse(o => {
      if (o.userData?._isAnalysisOverlay) o.visible = show;
    });
  }
}

function toggleAnalysisOverlays() {
  setAnalysisOverlayVisibility(!showAnalysisOverlays);
}

// ── Member box helper ─────────────────────────────────────────────────────────
function memberFrame(start, end, strongAxis) {
  const dirVec = end.clone().sub(start);
  if (dirVec.lengthSq() < 1e-12) return null;
  const dir = dirVec.normalize();
  const worldUp = new THREE.Vector3(0, 1, 0); // Three.js Y = geometry Z (up)
  const isVertical = Math.abs(dir.dot(worldUp)) > 0.95;
  // Choose reference up: if beam is nearly vertical (column), use X as ref
  const refUp = isVertical ? new THREE.Vector3(1, 0, 0) : worldUp;
  const zAx = new THREE.Vector3().crossVectors(dir, refUp).normalize();
  if (zAx.lengthSq() < 1e-8) return null;
  const yAx = new THREE.Vector3().crossVectors(zAx, dir).normalize();
  return {
    dir,
    yAx,
    zAx,
    quaternion: new THREE.Quaternion().setFromRotationMatrix(
      new THREE.Matrix4().makeBasis(dir, yAx, zAx)
    ),
  };
}

function memberQuaternion(start, end, strongAxis) {
  return memberFrame(start, end, strongAxis)?.quaternion ?? new THREE.Quaternion();
}

function cutLocalFrame(memberInfo, memberEnd) {
  const frame = memberFrame(memberInfo.start3, memberInfo.end3, memberInfo.strongAxis);
  if (!frame) return null;
  const xAx = memberEnd === 'axis_end'
    ? frame.dir.clone().negate()
    : frame.dir.clone();
  const yAx = frame.yAx.clone();
  let zAx = new THREE.Vector3().crossVectors(xAx, yAx).normalize();
  if (zAx.lengthSq() < 1e-8) zAx = frame.zAx.clone();
  return {
    xAx,
    yAx,
    zAx,
    quaternion: new THREE.Quaternion().setFromRotationMatrix(
      new THREE.Matrix4().makeBasis(xAx, yAx, zAx)
    ),
  };
}

function memberSectionDims(profile, strongAxis) {
  const hMm = profile?.h_mm, bMm = profile?.b_mm, cnt = profile?.count ?? 1;
  if (!hMm || !bMm) return [null, null];
  return strongAxis === 'horizontal'
    ? [bMm, hMm]
    : [hMm, bMm * cnt];
}

function memberEnds(grpName, m) {
  return grpName === 'columns'
    ? [m.base, m.top]
    : [m.axis_start, m.axis_end];
}

function buildMemberIndex(geo) {
  const idx = new Map();
  for (const [grpName, lst] of Object.entries(geo.members ?? {})) {
    for (const m of lst ?? []) {
      const [a, b] = memberEnds(grpName, m);
      if (!a || !b) continue;
      idx.set(m.id, {
        group: grpName,
        member: m,
        start3: pt(a),
        end3: pt(b),
        strongAxis: m.strong_axis,
        profile: m.profile,
      });
    }
  }
  return idx;
}

function connectionSupportLinePoint(con, memberIndex, supportLineRef) {
  if (!con.at || supportLineRef === 'support_centerline') return pt(con.at);
  const supportId = con.members?.[1];
  const supportInfo = supportId ? memberIndex.get(supportId) : null;
  if (!supportInfo) return null;

  const halfWidthMm = ((supportInfo.profile?.b_mm ?? 0) * (supportInfo.profile?.count ?? 1)) / 2;
  if (!halfWidthMm) return null;

  const anchor = { ...con.at };
  if (supportId === 'kattotuoli.vasen' || supportId === 'kattotuoli.oikea') {
    const outerSign = supportId === 'kattotuoli.vasen' ? -1 : 1;
    const sign = supportLineRef === 'support_outer_edge' ? outerSign : -outerSign;
    anchor.x += sign * halfWidthMm;
    return pt(anchor);
  }
  if (supportId.startsWith('beam.')) {
    const sign = supportLineRef === 'support_outer_edge' ? 1 : -1;
    anchor.y += sign * halfWidthMm;
    return pt(anchor);
  }
  return null;
}

function supportLinePlateAxes(supportInfo, shiftDir) {
  let plateU = null;
  if (supportInfo) {
    const supportFrame = memberFrame(supportInfo.start3, supportInfo.end3, supportInfo.strongAxis);
    if (supportFrame) {
      plateU = supportFrame.dir.clone().projectOnPlane(shiftDir);
    }
  }
  if (!plateU || plateU.lengthSq() < 1e-8) {
    plateU = WORLD_UP.clone().projectOnPlane(shiftDir);
  }
  if (plateU.lengthSq() < 1e-8) {
    plateU = new THREE.Vector3(1, 0, 0).projectOnPlane(shiftDir);
  }
  if (plateU.lengthSq() < 1e-8) {
    plateU = new THREE.Vector3(0, 0, 1).projectOnPlane(shiftDir);
  }
  plateU.normalize();
  const plateV = new THREE.Vector3().crossVectors(shiftDir, plateU).normalize();
  return { plateU, plateV };
}

function addSupportLineShiftVisual(g, pk, anchor, supportPoint, supportInfo, baseUserData) {
  const shiftVec = supportPoint.clone().sub(anchor);
  const shiftLen = shiftVec.length();
  if (shiftLen <= 1e-6) return;

  const shiftDir = shiftVec.clone().normalize();
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(8, 8, shiftLen, 10),
    new THREE.MeshBasicMaterial({
      color: SUPPORT_LINE_SHIFT_COLOR,
      transparent: true,
      opacity: 0.95,
      depthWrite: false,
      depthTest: false,
    })
  );
  shaft.position.copy(anchor.clone().add(supportPoint).multiplyScalar(0.5));
  shaft.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), shiftDir);
  shaft.renderOrder = 29;
  shaft.visible = showAnalysisOverlays;
  shaft.userData = { ...baseUserData };
  g.add(shaft);
  pk.push(shaft);

  const { plateU, plateV } = supportLinePlateAxes(supportInfo, shiftDir);
  const plateGeo = new THREE.BoxGeometry(150, 90, 8);
  const plate = new THREE.Mesh(
    plateGeo,
    new THREE.MeshBasicMaterial({
      color: SUPPORT_LINE_SHIFT_COLOR,
      transparent: true,
      opacity: 0.32,
      depthWrite: false,
      depthTest: false,
    })
  );
  plate.position.copy(supportPoint);
  plate.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(plateU, plateV, shiftDir));
  plate.renderOrder = 29;
  plate.visible = showAnalysisOverlays;
  plate.userData = { ...baseUserData };
  g.add(plate);
  pk.push(plate);

  const plateEdges = new THREE.LineSegments(
    new THREE.EdgesGeometry(plateGeo),
    new THREE.LineBasicMaterial({
      color: SUPPORT_LINE_SHIFT_COLOR,
      transparent: true,
      opacity: 1.0,
      depthWrite: false,
      depthTest: false,
    })
  );
  plateEdges.position.copy(supportPoint);
  plateEdges.quaternion.copy(plate.quaternion);
  plateEdges.renderOrder = 30;
  plateEdges.visible = showAnalysisOverlays;
  plateEdges.userData = { ...baseUserData, _isAnalysisOverlay: true };
  g.add(plateEdges);
}

function projectPointToLine(point3, start3, end3) {
  const vec = end3.clone().sub(start3);
  const len = vec.length();
  if (len < 1e-9) return start3.clone();
  const dir = vec.clone().divideScalar(len);
  const t = point3.clone().sub(start3).dot(dir);
  return start3.clone().add(dir.multiplyScalar(t));
}

function projectedMemberHalfExtent(memberInfo, axisDir) {
  if (!memberInfo) return 0;
  const frame = memberFrame(memberInfo.start3, memberInfo.end3, memberInfo.strongAxis);
  const [boxH, boxB] = memberSectionDims(memberInfo.profile, memberInfo.strongAxis);
  if (!frame || !boxH || !boxB) return 0;
  const dir = axisDir.clone().normalize();
  const halfLen = memberInfo.start3.distanceTo(memberInfo.end3) / 2;
  return Math.abs(dir.dot(frame.dir)) * halfLen
    + Math.abs(dir.dot(frame.yAx)) * (boxH / 2)
    + Math.abs(dir.dot(frame.zAx)) * (boxB / 2);
}

function connectionCuts(con) {
  return Array.isArray(con.cuts) && con.cuts.length
    ? con.cuts
    : (con.notch ? [con.notch] : []);
}

function cutOffsetMm(cut) {
  return cut.offset_mm ?? cut.x_from_support_edge_mm ?? 0;
}

function resolveCutAnchor(cut, con, memberInfo, supportInfo) {
  const frame = cutLocalFrame(memberInfo, cut.member_end);
  if (!frame) return null;

  const inwardDir = frame.xAx.clone();
  const axisPoint = projectPointToLine(pt(con.at), memberInfo.start3, memberInfo.end3);
  const endPoint = cut.member_end === 'axis_end'
    ? memberInfo.end3.clone()
    : memberInfo.start3.clone();
  const ref = cut.reference ?? 'support_inner_edge';
  const supportHalf = projectedMemberHalfExtent(supportInfo, inwardDir);

  let anchor = axisPoint.clone();
  switch (ref) {
    case 'member_end':
      anchor = endPoint;
      break;
    case 'support_outer_edge':
      anchor = axisPoint.clone().add(inwardDir.clone().multiplyScalar(-supportHalf));
      break;
    case 'support_centerline':
      anchor = axisPoint;
      break;
    case 'support_inner_edge':
    default:
      anchor = axisPoint.clone().add(inwardDir.clone().multiplyScalar(supportHalf));
      break;
  }
  return anchor.add(inwardDir.multiplyScalar(cutOffsetMm(cut)));
}

function extrudeNotchPolygon(points, widthMm) {
  const shape = new THREE.Shape();
  shape.moveTo(points[0][0], points[0][1]);
  for (let i = 1; i < points.length; i++) shape.lineTo(points[i][0], points[i][1]);
  shape.closePath();
  const geo = new THREE.ExtrudeGeometry(shape, { depth: widthMm, bevelEnabled: false });
  geo.translate(0, 0, -widthMm / 2);
  return geo;
}

function birdsmouthGeometry(notch, boxH, boxB, memberInfo, supportInfo) {
  const frame = cutLocalFrame(memberInfo, notch.member_end);
  if (!frame) return null;

  let supportNormal = null;
  if (supportInfo) {
    const supportFrame = memberFrame(supportInfo.start3, supportInfo.end3, supportInfo.strongAxis);
    if (supportFrame) {
      supportNormal = supportFrame.yAx.clone();
      if (supportNormal.dot(frame.yAx) < 0) supportNormal.negate();
    }
  }
  if (!supportNormal) supportNormal = frame.yAx.clone();

  let plumb3 = supportNormal.clone().projectOnPlane(frame.zAx);
  if (plumb3.lengthSq() < 1e-8) plumb3 = frame.yAx.clone();
  let plumb2 = new THREE.Vector2(plumb3.dot(frame.xAx), plumb3.dot(frame.yAx));
  if (plumb2.lengthSq() < 1e-8) return null;
  plumb2.normalize();
  if (plumb2.y > 0 || (Math.abs(plumb2.y) < 1e-6 && plumb2.x > 0)) {
    plumb2.multiplyScalar(-1);
  }

  let seat2 = new THREE.Vector2(-plumb2.y, plumb2.x);
  if (seat2.lengthSq() < 1e-8) return null;
  seat2.normalize();
  if (seat2.x > 0 || (Math.abs(seat2.x) < 1e-6 && seat2.y < 0)) {
    seat2.multiplyScalar(-1);
  }

  const bottom = -boxH / 2;
  const top = boxH / 2;
  const heelY = bottom + notch.heel_depth_mm;
  const heel = [0, heelY];

  // Plumb cut: from heel along plumb direction to where it exits the member bottom
  const plumbT = (bottom - heelY) / plumb2.y;
  const plumbEnd = [plumbT * plumb2.x, bottom];

  // Seat cut: from heel along seat direction by seat_length_mm
  const seatEnd = [
    seat2.x * notch.seat_length_mm,
    heelY + seat2.y * notch.seat_length_mm,
  ];

  // Clip seatEnd to member boundaries (bottom/top)
  let clippedSeatEnd = seatEnd;
  if (seatEnd[1] < bottom || seatEnd[1] > top) {
    const boundY = seatEnd[1] < bottom ? bottom : top;
    const tClip = (boundY - heelY) / seat2.y;
    clippedSeatEnd = [seat2.x * tClip, boundY];
  }

  // Build polygon: rafter bottom edge connects plumbEnd to seatBottom (CCW winding)
  const seatBottom = [clippedSeatEnd[0], bottom];
  const needsBottom = Math.abs(clippedSeatEnd[1] - bottom) > 0.1;
  const poly = needsBottom
    ? [seatBottom, plumbEnd, heel, clippedSeatEnd]
    : [plumbEnd, heel, clippedSeatEnd];
  return extrudeNotchPolygon(poly, boxB);
}

function notchGeometry(notch, boxH, boxB, memberInfo, supportInfo) {
  const bottom = -boxH / 2;
  const top = boxH / 2;
  if (notch.kind === 'rect_notch') {
    const side = notch.side;
    return side === 'top'
      ? extrudeNotchPolygon([
          [0, top],
          [notch.length_mm, top],
          [notch.length_mm, top - notch.depth_mm],
          [0, top - notch.depth_mm],
        ], boxB)
      : extrudeNotchPolygon([
          [0, bottom],
          [notch.length_mm, bottom],
          [notch.length_mm, bottom + notch.depth_mm],
          [0, bottom + notch.depth_mm],
        ], boxB);
  }
  if (notch.kind === 'bevel_bottom_notch') {
    return extrudeNotchPolygon([
      [0, bottom],
      [notch.length_mm, bottom],
      [0, bottom + notch.depth_mm],
    ], boxB);
  }
  if (notch.kind === 'birdsmouth_notch') {
    return birdsmouthGeometry(notch, boxH, boxB, memberInfo, supportInfo);
  }
  if (notch.kind === 'end_bevel_cut') {
    return notch.cut_from === 'top'
      ? extrudeNotchPolygon([
          [0, top],
          [notch.length_mm, top],
          [0, bottom],
        ], boxB)
      : extrudeNotchPolygon([
          [0, bottom],
          [notch.length_mm, bottom],
          [0, top],
        ], boxB);
  }
  return null;
}

function addNotchVisual(g, pk, con, notch, memberInfo, supportInfo, cutIndex = 0) {
  if (!notch || !memberInfo) return;
  const [boxH, boxB] = memberSectionDims(memberInfo.profile, memberInfo.strongAxis);
  if (!boxH || !boxB) return;

  const notchGeo = notchGeometry(notch, boxH, boxB, memberInfo, supportInfo);
  if (!notchGeo) return;

  const localFrame = cutLocalFrame(memberInfo, notch.member_end);
  if (!localFrame) return;
  const anchorPoint = resolveCutAnchor(notch, con, memberInfo, supportInfo);
  if (!anchorPoint) return;

  const wrapper = new THREE.Group();
  wrapper.position.copy(anchorPoint);
  wrapper.quaternion.copy(localFrame.quaternion);
  const notchMemberId = con.members?.[0];
  wrapper.userData = {
    id: `${con.id}:${cutIndex}`,
    kind: `${con.type}:${notch.kind}`,
    _geoName: g.userData.geoName,
    _isNotchOverlay: true,
    _notchMemberId: notchMemberId,
  };

  const mat = new THREE.MeshBasicMaterial({
    color: 0xff66cc,
    transparent: true,
    opacity: 0.45,
    side: THREE.DoubleSide,
    depthWrite: false,
    depthTest: false,
  });
  const mesh = new THREE.Mesh(notchGeo, mat);
  mesh.renderOrder = 20;
  mesh.userData = { ...wrapper.userData };
  wrapper.add(mesh);
  pk.push(mesh);

  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(notchGeo),
    new THREE.LineBasicMaterial({
      color: 0x441133,
      transparent: true,
      opacity: 0.9,
      depthTest: false,
    })
  );
  edges.renderOrder = 21;
  edges.userData = { ...wrapper.userData };
  wrapper.add(edges);
  g.add(wrapper);
}

function addMemberMesh(g, pk, start3, end3, profile, strongAxis, color, userData) {
  const len  = start3.distanceTo(end3);
  if (len < 1) return;

  let hMm = profile?.h_mm, bMm = profile?.b_mm;

  if (!hMm || !bMm) {
    // TBD profile: draw as line
    const lineGeo = new THREE.BufferGeometry().setFromPoints([start3, end3]);
    const line = new THREE.Line(lineGeo, new THREE.LineDashedMaterial({ color, dashSize: 60, gapSize: 40 }));
    line.computeLineDistances();
    line.userData = { ...userData, _isMemberLine: true };
    g.add(line); pk.push(line);
    registerMemberVisual(g.userData.geoName, userData.id, line);
    return;
  }

  // strong_axis='vertical' → h_mm is the vertical dimension
  // strong_axis='horizontal' → h_mm is horizontal, b_mm is vertical
  const [boxH, boxB] = memberSectionDims(profile, strongAxis);

  const q   = memberQuaternion(start3, end3, strongAxis);
  const mid = start3.clone().add(end3).multiplyScalar(0.5);

  const boxGeo = new THREE.BoxGeometry(len, boxH, boxB);
  const mat = new THREE.MeshLambertMaterial({ color, transparent: true, opacity: 0.85, side: THREE.DoubleSide });
  const mesh   = new THREE.Mesh(boxGeo, mat);
  mesh.position.copy(mid);
  mesh.quaternion.copy(q);
  mesh.userData = { ...userData, _isMemberMesh: true };
  g.add(mesh); pk.push(mesh);
  registerMemberVisual(g.userData.geoName, userData.id, mesh);

  // Edges for clarity
  const edgeMat = new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.4 });
  const edges   = new THREE.LineSegments(new THREE.EdgesGeometry(boxGeo), edgeMat);
  edges.position.copy(mid);
  edges.quaternion.copy(q);
  edges.userData = { ...userData, _isMemberEdge: true };
  g.add(edges);
  registerMemberVisual(g.userData.geoName, userData.id, edges);
}

// ── Members ───────────────────────────────────────────────────────────────────
function addMembers(g, pk, geo) {
  for (const [grpName, lst] of Object.entries(geo.members ?? {})) {
    const col = MCOL[grpName] ?? 0xffffff;
    for (const m of lst ?? []) {
      const [a, b] = memberEnds(grpName, m);
      if (!a || !b) continue;
      addMemberMesh(g, pk, pt(a), pt(b), m.profile, m.strong_axis, col,
        { id: m.id, kind: grpName, _geoName: g.userData.geoName });
    }
  }
}

// ── Surface meshes ────────────────────────────────────────────────────────────
function signedArea2D(points2) {
  let area = 0;
  for (let i = 0; i < points2.length; i++) {
    const a = points2[i];
    const b = points2[(i + 1) % points2.length];
    area += a.x * b.y - b.x * a.y;
  }
  return 0.5 * area;
}

function surfaceFrame(poly) {
  const pts3 = poly.map(pt);
  const origin = pts3[0].clone();

  let xAxis = null;
  for (let i = 1; i < pts3.length; i++) {
    const candidate = pts3[i].clone().sub(origin);
    if (candidate.lengthSq() > 1e-9) {
      xAxis = candidate.normalize();
      break;
    }
  }
  if (!xAxis) return null;

  let normal = null;
  for (let i = 2; i < pts3.length; i++) {
    const vec = pts3[i].clone().sub(origin);
    const cand = new THREE.Vector3().crossVectors(xAxis, vec);
    if (cand.lengthSq() > 1e-9) {
      normal = cand.normalize();
      break;
    }
  }
  if (!normal) return null;

  const yAxis = new THREE.Vector3().crossVectors(normal, xAxis).normalize();
  return { origin, xAxis, yAxis, normal };
}

function buildSurfaceGeometry(poly, thicknessMm = 0) {
  const frame = surfaceFrame(poly);
  if (!frame) return null;

  let points2 = poly.map(p => {
    const rel = pt(p).sub(frame.origin);
    return new THREE.Vector2(rel.dot(frame.xAxis), rel.dot(frame.yAxis));
  });
  if (signedArea2D(points2) < 0) points2 = points2.reverse();

  const shape = new THREE.Shape(points2);
  const geom = thicknessMm > 0
    ? new THREE.ExtrudeGeometry(shape, { depth: thicknessMm, bevelEnabled: false })
    : new THREE.ShapeGeometry(shape);

  if (thicknessMm > 0) geom.translate(0, 0, -thicknessMm / 2);

  const basis = new THREE.Matrix4().makeBasis(frame.xAxis, frame.yAxis, frame.normal);
  basis.setPosition(frame.origin);
  geom.applyMatrix4(basis);
  geom.computeVertexNormals();
  return geom;
}

function addSurfaces(g, pk, list, opacity) {
  const geoName = g.userData.geoName;
  for (const s of list ?? []) {
    const poly = s.polygon;
    if (!poly || poly.length < 3) continue;
    const col = SCOL[s.type] ?? 0xaaaaaa;
    const thicknessMm = Math.max(0, s.thickness_mm ?? 0);
    const mg = buildSurfaceGeometry(poly, thicknessMm);
    if (!mg) continue;
    const mesh = new THREE.Mesh(mg, new THREE.MeshBasicMaterial(
      { color: col, transparent: true, opacity, side: THREE.DoubleSide }
    ));
    const memberRefs = [...new Set((s.load_transfer?.to_members ?? []).flatMap(rule => rule.member_refs ?? []))];
    mesh.userData = {
      id: s.id,
      kind: s.type ?? 'surface',
      _geoName: geoName,
      _loadTransferMemberRefs: memberRefs,
      tooltipLines: surfaceTooltipLines(s),
    };
    g.add(mesh); pk.push(mesh);

    if (thicknessMm > 0) {
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(mg),
        new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: Math.min(opacity * 2, 1) })
      );
      edges.userData = { id: s.id, kind: s.type ?? 'surface', _geoName: geoName };
      g.add(edges);
    } else {
      const outPts = [...poly.map(pt), pt(poly[0])];
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(outPts),
        new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: Math.min(opacity * 2, 1) })
      );
      line.userData = { id: s.id, kind: s.type ?? 'surface', _geoName: geoName };
      g.add(line);
    }
  }
}

// ── Connection markers ────────────────────────────────────────────────────────
function addConnectionAnalysisVisuals(g, pk, con, memberIndex, tooltipLines) {
  const analysis = con.analysis;
  if (!analysis || !con.at) return;

  const anchor = pt(con.at);
  const supportModel = analysis.support_model ?? 'pinned';
  const markerColor = ANALYSIS_SUPPORT_MODEL_COLORS[supportModel] ?? 0xffffff;
  const baseUserData = {
    id: con.id,
    kind: `${con.type ?? 'connection'} / analysis`,
    _geoName: g.userData.geoName,
    _isAnalysisOverlay: true,
    tooltipLines,
  };

  const marker = createAnalysisMarker(markerColor);
  marker.position.copy(anchor);
  marker.renderOrder = 30;
  marker.visible = showAnalysisOverlays;
  marker.userData = { ...baseUserData };
  g.add(marker);
  pk.push(marker);

  const supportLineRef = analysis.support_line_ref ?? 'support_centerline';
  const supportInfo = con.members?.[1] ? memberIndex.get(con.members[1]) : null;
  const supportPoint = connectionSupportLinePoint(con, memberIndex, supportLineRef);
  if (!supportPoint || supportPoint.distanceTo(anchor) <= 1e-6) return;
  addSupportLineShiftVisual(g, pk, anchor, supportPoint, supportInfo, baseUserData);
}

function addConnections(g, pk, connections, memberIndex) {
  const sphereGeo = new THREE.SphereGeometry(80, 12, 8);
  const notchCutsMap = new Map();
  for (const con of connections ?? []) {
    if (!con.at) continue;
    const col = CCOL[con.type] ?? 0xffffff;
    const tooltipLines = connectionTooltipLines(con);
    const mesh = new THREE.Mesh(
      sphereGeo,
      new THREE.MeshBasicMaterial({ color: col })
    );
    mesh.position.copy(pt(con.at));
    mesh.visible = showConnectionMarkers;
    mesh.userData = {
      id: con.id,
      kind: con.type ?? 'connection',
      _geoName: g.userData.geoName,
      _isConnectionMarker: true,
      tooltipLines,
    };
    g.add(mesh);
    pk.push(mesh);
    addConnectionAnalysisVisuals(g, pk, con, memberIndex, tooltipLines);
    if (con.type === 'notched_over') {
      const cuts = connectionCuts(con);
      const memberId = con.members?.[0];
      const memberInfo = memberIndex.get(memberId);
      const supportInfo = memberIndex.get(con.members?.[1]);
      for (let i = 0; i < cuts.length; i++) {
        addNotchVisual(g, pk, con, cuts[i], memberInfo, supportInfo, i);
        // Collect notch geometry data for CSG subtraction
        if (memberInfo && memberId) {
          const [boxH, boxB] = memberSectionDims(memberInfo.profile, memberInfo.strongAxis);
          if (boxH && boxB) {
            const notchGeo = notchGeometry(cuts[i], boxH, boxB, memberInfo, supportInfo);
            const localFrame = cutLocalFrame(memberInfo, cuts[i].member_end);
            const anchorPoint = resolveCutAnchor(cuts[i], con, memberInfo, supportInfo);
            if (notchGeo && localFrame && anchorPoint) {
              if (!notchCutsMap.has(memberId)) notchCutsMap.set(memberId, []);
              notchCutsMap.get(memberId).push({
                notchGeo,
                anchorPoint,
                cutQ: localFrame.quaternion,
              });
            }
          }
        }
      }
    }
  }
  return notchCutsMap;
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

function captureViewState() {
  return {
    position: camera.position.clone(),
    target: controls.target.clone(),
    zoom: camera.zoom,
  };
}

function restoreViewState(state) {
  if (!state) return false;
  camera.position.copy(state.position);
  controls.target.copy(state.target);
  camera.zoom = state.zoom;
  camera.updateProjectionMatrix();
  controls.update();
  return true;
}

// ── CSG boolean subtraction of notches from member meshes ─────────────────────
// ── Categorized edge classification for notched members ──────────────────────
// Returns { sharp, shallow, boundary } – each a BufferGeometry or null.
// sharp:    dihedral > 30° → real corners
// shallow:  5° < dihedral ≤ 30° → possible CSG tessellation artefacts
// boundary: edges with only one adjacent face (open-mesh)
//
// Uses position-based vertex matching (rounded to 0.5 mm) instead of index
// matching, because three-bvh-csg can leave numerically distinct vertices at
// geometrically identical positions that mergeVertices fails to unify.
function classifyMeshEdges(geometry) {
  const pos = geometry.attributes.position;
  if (!pos) return { sharp: null, shallow: null, boundary: null };
  const idx = geometry.index;
  const triCount = idx ? idx.count / 3 : pos.count / 3;

  // Round each coordinate to the nearest 0.5 mm to tolerate CSG precision noise
  const vKey = i => {
    const r = 2; // multiply → round → gives 0.5-unit grid
    return `${Math.round(pos.getX(i) * r)}|${Math.round(pos.getY(i) * r)}|${Math.round(pos.getZ(i) * r)}`;
  };
  const getI = (t, v) => idx ? idx.getX(t * 3 + v) : t * 3 + v;

  // edgeKey → { n1, n2, pa, pb }  (pa/pb are the actual 3-D positions for output)
  const edgeMap = new Map();

  for (let t = 0; t < triCount; t++) {
    const ai = getI(t, 0), bi = getI(t, 1), ci = getI(t, 2);
    const pa = new THREE.Vector3(pos.getX(ai), pos.getY(ai), pos.getZ(ai));
    const pb = new THREE.Vector3(pos.getX(bi), pos.getY(bi), pos.getZ(bi));
    const pc = new THREE.Vector3(pos.getX(ci), pos.getY(ci), pos.getZ(ci));
    const n  = new THREE.Vector3().crossVectors(pb.clone().sub(pa), pc.clone().sub(pa));
    if (n.lengthSq() < 1e-20) continue; // skip degenerate triangles
    n.normalize();

    for (const [ei, ej, p0, p1] of [
      [ai, bi, pa, pb],
      [bi, ci, pb, pc],
      [ci, ai, pc, pa],
    ]) {
      const ka = vKey(ei), kb = vKey(ej);
      const k  = ka < kb ? `${ka}/${kb}` : `${kb}/${ka}`;
      if (!edgeMap.has(k)) edgeMap.set(k, { n1: n, n2: null, pa: p0, pb: p1 });
      else edgeMap.get(k).n2 = n;
    }
  }

  const COS30 = Math.cos(THREE.MathUtils.degToRad(30));
  const COS5  = Math.cos(THREE.MathUtils.degToRad(5));
  const sharp = [], shallow = [], boundary = [];

  for (const { n1, n2, pa, pb } of edgeMap.values()) {
    const seg = [pa.x, pa.y, pa.z, pb.x, pb.y, pb.z];
    if (!n2)                     boundary.push(...seg);
    else if (n1.dot(n2) < COS30) sharp.push(...seg);
    else if (n1.dot(n2) < COS5)  shallow.push(...seg);
  }

  const makeGeo = pts => {
    if (!pts.length) return null;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
    return g;
  };
  return { sharp: makeGeo(sharp), shallow: makeGeo(shallow), boundary: makeGeo(boundary) };
}

// Returns true if the segment pa→pb is parallel to and close to any segment in
// expectedSegs (array of [Vector3, Vector3] pairs). Used to distinguish real
// silhouette edges from CSG tessellation boundary artefacts.
function nearExpectedEdge(pa, pb, expectedSegs, tol = 2.0) {
  const len = pa.distanceTo(pb);
  if (len < 1e-6) return false;
  const dir = pb.clone().sub(pa).divideScalar(len);
  const mid = pa.clone().add(pb).multiplyScalar(0.5);

  for (const [ea, eb] of expectedSegs) {
    const eLen = ea.distanceTo(eb);
    if (eLen < 1e-6) continue;
    const eDir = eb.clone().sub(ea).divideScalar(eLen);
    if (Math.abs(dir.dot(eDir)) < 0.98) continue; // not parallel (< ~11°)
    // Distance from midpoint to the infinite line through ea in direction eDir
    const proj = ea.clone().addScaledVector(eDir, mid.clone().sub(ea).dot(eDir));
    if (mid.distanceTo(proj) < tol) return true;
  }
  return false;
}

async function applyCSGCuts(g, pk, memberIndex, notchCutsMap) {
  if (!notchCutsMap || notchCutsMap.size === 0) return;
  const csg = await getCSG();
  if (!csg) { console.warn('CSG module not loaded, skipping cuts'); return; }
  const { Evaluator, Brush, SUBTRACTION } = csg;
  if (!Evaluator || !Brush || !SUBTRACTION) {
    console.warn('CSG exports missing:', { Evaluator: !!Evaluator, Brush: !!Brush, SUBTRACTION: !!SUBTRACTION });
    return;
  }
  const evaluator = new Evaluator();
  console.log('CSG: processing', notchCutsMap.size, 'members with notch cuts');

  for (const [memberId, cuts] of notchCutsMap) {
    let memberMesh = null, memberEdges = null;
    g.traverse(o => {
      if (o.userData?.id === memberId) {
        if (o.userData._isMemberMesh) memberMesh = o;
        if (o.userData._isMemberEdge) memberEdges = o;
      }
    });
    if (!memberMesh) { console.warn('CSG: member mesh not found for', memberId); continue; }
    console.log('CSG: cutting member', memberId, 'with', cuts.length, 'notch(es)');

    const memberWorld = new THREE.Matrix4().compose(
      memberMesh.position, memberMesh.quaternion, new THREE.Vector3(1, 1, 1)
    );
    const memberWorldInv = memberWorld.clone().invert();

    try {
      let currentBrush = new Brush(memberMesh.geometry.clone());
      const localNotchGeos = []; // saved in member local space for expected-edge computation

      for (const cut of cuts) {
        const notchGeoCopy = cut.notchGeo.clone();
        const notchWorld = new THREE.Matrix4().compose(
          cut.anchorPoint, cut.cutQ, new THREE.Vector3(1, 1, 1)
        );
        const relMatrix = memberWorldInv.clone().multiply(notchWorld);
        notchGeoCopy.applyMatrix4(relMatrix);
        localNotchGeos.push(notchGeoCopy.clone()); // save before CSG consumes it

        const notchBrush = new Brush(notchGeoCopy);
        notchBrush.updateMatrixWorld();
        currentBrush.updateMatrixWorld();
        currentBrush = evaluator.evaluate(currentBrush, notchBrush, SUBTRACTION);
      }

      const cutGeo = currentBrush.geometry;
      cutGeo.computeVertexNormals();
      const cutMat = memberMesh.material.clone();
      cutMat.flatShading = true;
      const cutMesh = new THREE.Mesh(cutGeo, cutMat);
      cutMesh.position.copy(memberMesh.position);
      cutMesh.quaternion.copy(memberMesh.quaternion);
      cutMesh.userData = { ...memberMesh.userData, _isCutMember: true };
      cutMesh.visible = !showConnectionMarkers;
      g.add(cutMesh);
      pk.push(cutMesh);
      registerMemberVisual(g.userData.geoName, memberId, cutMesh);

      // Build expected edge segments in member local space:
      //   original box edges + all notch geometry edges.
      // Boundary CSG edges that lie on these lines are real silhouette edges;
      // others are tessellation artefacts introduced by the CSG algorithm.
      const expectedSegs = [];
      const addEdgesFromGeo = geo => {
        const eg = new THREE.EdgesGeometry(geo);
        const p  = eg.attributes.position;
        for (let i = 0; i < p.count; i += 2) {
          expectedSegs.push([
            new THREE.Vector3(p.getX(i),   p.getY(i),   p.getZ(i)),
            new THREE.Vector3(p.getX(i+1), p.getY(i+1), p.getZ(i+1)),
          ]);
        }
      };
      addEdgesFromGeo(memberMesh.geometry); // original box (local frame)
      for (const ng of localNotchGeos) addEdgesFromGeo(ng);

      // Classify CSG edges and split boundary into real vs artefact
      const { sharp: sharpGeo, shallow: shallowGeo, boundary: boundaryGeo } =
        classifyMeshEdges(cutGeo);

      const realPts = [];
      const artifactPts = [];

      const collectGeo = (geo, dest) => {
        if (!geo) return;
        const p = geo.attributes.position;
        for (let i = 0; i < p.count; i++)
          dest.push(p.getX(i), p.getY(i), p.getZ(i));
      };
      collectGeo(sharpGeo, realPts);

      if (boundaryGeo) {
        const p = boundaryGeo.attributes.position;
        for (let i = 0; i < p.count; i += 2) {
          const pa = new THREE.Vector3(p.getX(i),   p.getY(i),   p.getZ(i));
          const pb = new THREE.Vector3(p.getX(i+1), p.getY(i+1), p.getZ(i+1));
          const dest = nearExpectedEdge(pa, pb, expectedSegs) ? realPts : artifactPts;
          dest.push(pa.x, pa.y, pa.z, pb.x, pb.y, pb.z);
        }
      }

      const makePtsGeo = pts => {
        if (!pts.length) return null;
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
        return geo;
      };

      const realGeo = makePtsGeo(realPts);
      if (realGeo) {
        const line = new THREE.LineSegments(
          realGeo, new THREE.LineBasicMaterial({ color: 0x000000 })
        );
        line.position.copy(memberMesh.position);
        line.quaternion.copy(memberMesh.quaternion);
        line.userData = { ...memberMesh.userData, _isCutMember: true };
        line.visible = !showConnectionMarkers;
        g.add(line);
        registerMemberVisual(g.userData.geoName, memberId, line);
      }

      // Hide original box edges (uncut shape) along with the uncut mesh
      memberMesh.userData._isUncutMember = true;
      memberMesh.visible = showConnectionMarkers;
      if (memberEdges) {
        memberEdges.userData._isUncutMember = true;
        memberEdges.visible = showConnectionMarkers;
      }

    } catch (e) {
      console.warn('CSG failed for member', memberId, e.message, e.stack);
    }
  }
}

// ── Render resolved geometry into a named group ───────────────────────────────
async function renderGeoFor(name, resolved, options = {}) {
  const preserveView = options.preserveView ?? (geoGroups.size > 0);
  const viewState = preserveView ? captureViewState() : null;
  clearGeoFor(name);
  const g = new THREE.Group();
  g.userData.geoName = name;
  scene.add(g);
  geoGroups.set(name, g);
  const memberIndex = buildMemberIndex(resolved);
  addMembers(g, pickable, resolved);
  addSurfaces(g, pickable, resolved.surfaces, 0.30);
  addSurfaces(g, pickable, resolved.reference_surfaces, 0.12);
  const notchCutsMap = addConnections(g, pickable, resolved.connections, memberIndex);
  await applyCSGCuts(g, pickable, memberIndex, notchCutsMap);
  setConnectionMarkerVisibility(showConnectionMarkers);
  setAnalysisOverlayVisibility(showAnalysisOverlays);
  if (!restoreViewState(viewState)) fitCamera();
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
    const userData = hits[0].object.userData ?? {};
    const { id, kind } = userData;
    tooltip.style.display = 'block';
    tooltip.style.left = (e.clientX - r.left + 14) + 'px';
    tooltip.style.top  = (e.clientY - r.top  - 28) + 'px';
    tooltip.textContent = Array.isArray(userData.tooltipLines)
      ? userData.tooltipLines.join('\\n')
      : id + (kind ? '  —  ' + kind : '');
    setHighlightedMembersForSurface(userData._geoName, userData._loadTransferMemberRefs);
  } else {
    tooltip.style.display = 'none';
    clearHighlightedMembers();
  }
});

renderer.domElement.addEventListener('mouseleave', () => {
  tooltip.style.display = 'none';
  clearHighlightedMembers();
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
    await renderGeoFor(name, data);
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
      await renderGeoFor(editingName, data);
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
  const connBtn  = document.getElementById('btn-connections');
  const analysisBtn = document.getElementById('btn-analysis');
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

  connBtn.addEventListener('click', toggleConnectionMarkers);
  analysisBtn.addEventListener('click', toggleAnalysisOverlays);
  setConnectionMarkerVisibility(showConnectionMarkers);
  setAnalysisOverlayVisibility(showAnalysisOverlays);
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
