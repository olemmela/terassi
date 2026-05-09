#!/usr/bin/env python3
"""Geometrian 3D-katseluohjelma ja -editori.

Käyttö:   python3 viewer.py
Avaa:     http://localhost:5001
"""
import copy, json, os
from flask import Flask, jsonify, request, Response, send_from_directory
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


@app.route("/assets/<path:filename>")
def get_asset(filename):
    root = os.path.abspath(_SCRIPT_DIR)
    path = os.path.abspath(os.path.join(root, filename))
    if not path.startswith(root + os.sep) or not os.path.isfile(path):
        return jsonify({"error": "Tiedostoa ei löydy"}), 404
    return send_from_directory(os.path.dirname(path), os.path.basename(path))


@app.route("/api/calibration-camera", methods=["PUT"])
def put_calibration_camera():
    root = os.path.abspath(_SCRIPT_DIR)
    try:
        data = request.get_json(force=True)
        rel_path = str(data.get("path", "")).strip().replace("\\", "/")
        if not rel_path or os.path.isabs(rel_path) or rel_path.endswith("/"):
            return jsonify({"error": "Virheellinen tiedostopolku"}), 400
        if not rel_path.lower().endswith(".json"):
            return jsonify({"error": "Kamera tallennetaan .json-tiedostoon"}), 400
        path = os.path.abspath(os.path.join(root, rel_path))
        if not path.startswith(root + os.sep):
            return jsonify({"error": "Tiedostopolku menee repojuuren ulkopuolelle"}), 400
        calibration = data.get("calibration")
        if not isinstance(calibration, dict) or not isinstance(calibration.get("camera"), dict):
            return jsonify({"error": "Kamera-JSON:sta puuttuu camera-objekti"}), 400
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(calibration, f, ensure_ascii=False, indent=2)
            f.write("\n")
        return jsonify({"ok": True, "path": rel_path})
    except json.JSONDecodeError as e:
        return jsonify({"error": f"JSON-virhe: {e}"}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 400


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

  #editor-panel { width: 420px; min-width: 260px; display: flex;
    flex-direction: column; flex-shrink: 0; }
  #editor { flex: 1; resize: none; background: #1e1e1e; color: #d4d4d4;
    border: none; border-right: 1px solid #444; outline: none; padding: 10px;
    font-family: 'Consolas', 'Monaco', monospace; font-size: 12px;
    line-height: 1.5; tab-size: 2; overflow: auto; }
  #editor[hidden] { display: none; }

  #resizer { width: 4px; background: #444; cursor: col-resize; flex-shrink: 0; }
  #resizer:hover, #resizer.dragging { background: #0066cc; }

  #viewport { flex: 1; position: relative; overflow: hidden; outline: none; background: #1a1a2e; }
  #viewport:focus { box-shadow: inset 0 0 0 2px rgba(0,136,255,.55); }
  canvas { display: block; }
  #viewport canvas { position: absolute; left: 0; top: 0; z-index: 1; }
  #calibration-bg {
    position: absolute; left: 0; top: 0; display: none; object-fit: contain;
    pointer-events: none; z-index: 0; background: #111; opacity: .72;
  }

  #tooltip { position: absolute; pointer-events: none;
    background: rgba(0,0,0,.8); color: #fff; padding: 6px 9px;
    border-radius: 4px; font-size: 12px; display: none; white-space: pre-line;
    line-height: 1.35; max-width: 420px; z-index: 4; }

  #legend { position: absolute; bottom: 10px; right: 10px;
    background: rgba(0,0,0,.65); padding: 8px 12px; border-radius: 6px;
    font-size: 11px; line-height: 1.8; z-index: 2; }
  #calibration-panel {
    flex: 1; min-height: 0; overflow: auto;
    background: #12161c; border: none; border-right: 1px solid #444;
    padding: 10px; font-size: 12px;
  }
  #calibration-panel[hidden] { display: none; }
  .calib-row { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; flex-wrap: wrap; }
  .calib-row label { display: flex; align-items: center; gap: 6px; color: #d7dde5; }
  .calib-row input[type="text"] {
    width: 250px; max-width: 100%; background: #14181e; color: #e8edf2;
    border: 1px solid #52616d; border-radius: 4px; padding: 4px 6px;
  }
  .calib-row input[type="range"] { width: 140px; }
  .calib-value { min-width: 40px; color: #e8edf2; font-variant-numeric: tabular-nums; }
  .calib-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 6px; margin: 8px 0; }
  .calib-grid label { display: flex; flex-direction: column; gap: 3px; color: #bfc7d1; }
  .calib-grid input {
    min-width: 0; background: #14181e; color: #e8edf2; border: 1px solid #52616d;
    border-radius: 4px; padding: 4px 5px; font: inherit;
  }
  .calib-btn {
    padding: 4px 9px; border: 1px solid #596774; background: #2f3943; color: #e2e8ef;
    cursor: pointer; border-radius: 4px; font-size: 12px;
  }
  .calib-btn:hover { background: #3c4854; }
  .calib-btn.active { background: #2b5d8a; border-color: #6ab0ff; color: #fff; }
  #viewport.calibration-pan canvas { cursor: grab; }
  #viewport.calibration-panning canvas { cursor: grabbing; }
  #calib-camera-json {
    width: 100%; height: 130px; resize: vertical; background: #101419; color: #d9e4ee;
    border: 1px solid #52616d; border-radius: 4px; padding: 7px;
    font-family: 'Consolas', 'Monaco', monospace; font-size: 11px; line-height: 1.35;
  }
  .calib-help { color: #aeb8c3; line-height: 1.35; margin-top: 6px; }
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
  <button class="btn-toggle" id="btn-calibration" aria-pressed="false">&#9675; Kuvakalibrointi</button>
  <button class="btn-save" id="btn-save">&#128190; Tallenna</button>
</div>
<div id="main">
  <div id="editor-panel">
    <textarea id="editor" spellcheck="false"></textarea>
  </div>
  <div id="resizer"></div>
  <div id="viewport">
    <img id="calibration-bg" alt="">
    <div id="tooltip"></div>
    <div id="calibration-panel" hidden>
      <div class="calib-row">
        <label>Taustakuva
          <input id="calib-image-path" type="text" value="kuvat/IMG_2837.jpeg">
        </label>
        <button class="calib-btn" id="calib-load-image">Lataa</button>
        <button class="calib-btn" id="calib-hide-image">Piilota kuva</button>
        <button class="calib-btn" id="calib-fit-model">Sovita malli</button>
      </div>
      <div class="calib-row">
        <label>kuvan läpinäkyvyys
          <input id="calib-image-opacity" type="range" min="0.15" max="1" step="0.05" value="0.72">
        </label>
        <label>taustakuvan zoom
          <input id="calib-image-zoom" type="range" min="1" max="3" step="0.25" value="1">
        </label>
        <span id="calib-image-zoom-value" class="calib-value">1×</span>
        <button class="calib-btn" id="calib-image-pan-mode" type="button">Siirrä zoom-kohtaa</button>
        <button class="calib-btn" id="calib-image-pan-reset" type="button">Keskitä</button>
        <span id="calib-image-pan-value" class="calib-value">x 0 px, y 0 px</span>
        <label>3D-rakenteiden näkyvyys
          <input id="calib-model-opacity" type="range" min="0.05" max="1" step="0.05" value="1">
        </label>
        <span id="calib-model-opacity-value">100 %</span>
        <span id="calib-render-size">ei kuvaa</span>
      </div>
      <div class="calib-grid">
        <label>kamera X<input id="calib-pos-x" type="number" step="10"></label>
        <label>kamera Y<input id="calib-pos-y" type="number" step="10"></label>
        <label>kamera Z<input id="calib-pos-z" type="number" step="10"></label>
        <label>FOV °<input id="calib-fov" type="number" step="0.1" min="1" max="120"></label>
        <label>target X<input id="calib-target-x" type="number" step="10"></label>
        <label>target Y<input id="calib-target-y" type="number" step="10"></label>
        <label>target Z<input id="calib-target-z" type="number" step="10"></label>
        <label>zoom<input id="calib-zoom" type="number" step="0.01" min="0.01"></label>
        <label>roll °<input id="calib-roll" type="number" step="0.1"></label>
      </div>
      <div class="calib-row">
        <button class="calib-btn" id="calib-copy-camera">Kopioi kameran JSON</button>
        <button class="calib-btn" id="calib-save-local">Tallenna selaimeen</button>
        <button class="calib-btn" id="calib-restore-local">Palauta selaimesta</button>
      </div>
      <div class="calib-row">
        <label>Kamera JSON
          <input id="calib-camera-json-path" type="text" value="kuvat/IMG_2837_viewer_camera.json">
        </label>
        <button class="calib-btn" id="calib-load-camera-json">Palauta polusta</button>
        <button class="calib-btn" id="calib-save-camera-json">Tallenna kamera JSON</button>
        <input id="calib-import-camera-json" type="file" accept="application/json,.json">
      </div>
      <textarea id="calib-camera-json" spellcheck="false"></textarea>
      <div class="calib-help">
        Säädä näkymää hiirellä, rullalla ja nuolinäppäimillä. Nuoli siirtää kameraa, Ctrl+nuoli kääntää,
        Alt+vasen/oikea kallistaa kameraa (roll) ja Shift kasvattaa askelta. 3D-rakenteiden näkyvyys helpottaa
        mallin sovittamista valokuvaa vasten. Taustakuvan zoom suurentaa kuvan ja 3D-overlayn yhdessä
        tarkempaa kohdistusta varten; **Siirrä zoom-kohtaa** -tilassa zoomattua näkymää voi raahata.
        Kameran voi palauttaa repojuuren JSON-polusta tai paikallisesta tiedostosta.
      </div>
    </div>
    <div id="legend">
      <div class="li"><div class="ld" style="background:#888"></div>Pilari</div>
      <div class="li"><div class="ld" style="background:#2266cc"></div>Palkki</div>
      <div class="li"><div class="ld" style="background:#22aa44"></div>Kattotuoli</div>
      <div class="li"><div class="ld" style="background:#dd8800"></div>Orsi</div>
      <div class="li"><div class="ld" style="background:rgba(0,160,160,.6)"></div>Lasipinta</div>
      <div class="li"><div class="ld" style="background:rgba(60,140,60,.6)"></div>Kattopinta</div>
      <div class="li"><div class="ld" style="background:#e8f2ff"></div>Aurinkopaneelien rajat</div>
      <div class="li"><div class="ld" style="background:#ffdd33"></div>Ovi-/ikkuna-aukko</div>
      <div class="li"><div class="ld" style="background:rgba(150,130,100,.3)"></div>Viitepinta</div>
      <div class="li"><div class="ld" style="background:#ff4444;border-radius:50%"></div>Tuki (supported_on)</div>
      <div class="li"><div class="ld" style="background:#66aaff;border-radius:50%"></div>Siirtolinkki (transfer_link)</div>
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
      <div class="li"><div class="ld" style="background:#ffee66"></div>Hover pintaan → member_refs/supported_by-korostus</div>
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
const calibrationBg = document.getElementById('calibration-bg');
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x1a1a2e, 0);
vp.insertBefore(renderer.domElement, calibrationBg.nextSibling);
vp.tabIndex = 0;
vp.setAttribute('aria-label', '3D-näkymä');

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, 1, 1, 200000);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.06;
controls.rotateSpeed = 0.25;
controls.panSpeed = 0.25;
controls.zoomSpeed = 0.55;
vp.addEventListener('pointerdown', () => vp.focus({ preventScroll: true }));

const KEY_MOVE_STEP_MM = 20;
const KEY_ROTATE_STEP_RAD = THREE.MathUtils.degToRad(0.5);
const WORLD_UP = new THREE.Vector3(0, 1, 0);
let cameraRollRad = 0;

function rolledCameraUp() {
  const viewDir = controls.target.clone().sub(camera.position);
  if (viewDir.lengthSq() < 1e-12) return WORLD_UP.clone();
  viewDir.normalize();

  let up = WORLD_UP.clone().projectOnPlane(viewDir);
  if (up.lengthSq() < 1e-10) {
    up = new THREE.Vector3(1, 0, 0).projectOnPlane(viewDir);
  }
  if (up.lengthSq() < 1e-10) return WORLD_UP.clone();
  up.normalize();
  if (Math.abs(cameraRollRad) > 1e-12) {
    up.applyAxisAngle(viewDir, cameraRollRad).normalize();
  }
  return up;
}

function applyCameraRoll() {
  camera.up.copy(rolledCameraUp());
  camera.lookAt(controls.target);
  camera.updateMatrixWorld(true);
}

function updateControlsWithRoll() {
  controls.update();
  applyCameraRoll();
}

function adjustCameraRoll(deltaRad) {
  cameraRollRad += deltaRad;
  updateControlsWithRoll();
  updateCalibrationPanelFromCamera();
}

function translateViewLocal(forwardMm, strafeMm = 0) {
  const forward = new THREE.Vector3();
  camera.getWorldDirection(forward);
  if (forward.lengthSq() < 1e-12) return;
  forward.normalize();

  const right = new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion).normalize();
  const delta = forward.multiplyScalar(forwardMm).add(right.multiplyScalar(strafeMm));
  camera.position.add(delta);
  controls.target.add(delta);
  updateControlsWithRoll();
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
  updateControlsWithRoll();
}

vp.addEventListener('keydown', e => {
  const moveStep = e.shiftKey ? KEY_MOVE_STEP_MM * 5 : KEY_MOVE_STEP_MM;
  const rotateStep = e.shiftKey ? KEY_ROTATE_STEP_RAD * 3 : KEY_ROTATE_STEP_RAD;

  if (e.altKey) {
    switch (e.key) {
      case 'ArrowLeft':
        adjustCameraRoll(+rotateStep);
        break;
      case 'ArrowRight':
        adjustCameraRoll(-rotateStep);
        break;
      default:
        return;
    }
  } else if (e.ctrlKey || e.metaKey) {
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

const calibrationImage = {
  enabled: false,
  path: 'kuvat/IMG_2837.jpeg',
  naturalWidth: 0,
  naturalHeight: 0,
  displayZoom: 1,
  displayPan: { x: 0, y: 0 },
  pendingDisplayPan: null,
  baseRenderRect: { left: 0, top: 0, width: 1, height: 1 },
  renderRect: { left: 0, top: 0, width: 1, height: 1 },
};

function containedImageRect(containerW, containerH, imageW, imageH) {
  if (!imageW || !imageH || !containerW || !containerH) {
    return { left: 0, top: 0, width: containerW, height: containerH };
  }
  const scale = Math.min(containerW / imageW, containerH / imageH);
  const width = imageW * scale;
  const height = imageH * scale;
  return {
    left: (containerW - width) / 2,
    top: (containerH - height) / 2,
    width,
    height,
  };
}

function clampImagePan(pan, baseRect = calibrationImage.baseRenderRect, zoom = calibrationImage.displayZoom) {
  const safeZoom = Math.max(1, Number(zoom) || 1);
  const maxX = Math.max(0, baseRect.width * (safeZoom - 1) / 2);
  const maxY = Math.max(0, baseRect.height * (safeZoom - 1) / 2);
  return {
    x: THREE.MathUtils.clamp(Number(pan?.x) || 0, -maxX, maxX),
    y: THREE.MathUtils.clamp(Number(pan?.y) || 0, -maxY, maxY),
  };
}

function zoomedImageRect(baseRect, zoom, pan) {
  const safeZoom = Math.max(1, Number(zoom) || 1);
  const safePan = clampImagePan(pan, baseRect, safeZoom);
  if (safeZoom === 1) return { ...baseRect };
  const centerX = baseRect.left + baseRect.width / 2;
  const centerY = baseRect.top + baseRect.height / 2;
  const width = baseRect.width * safeZoom;
  const height = baseRect.height * safeZoom;
  return {
    left: centerX - width / 2 + safePan.x,
    top: centerY - height / 2 + safePan.y,
    width,
    height,
  };
}

function currentRenderRect() {
  const w = Math.max(1, vp.clientWidth);
  const h = Math.max(1, vp.clientHeight);
  if (!calibrationImage.enabled) {
    calibrationImage.baseRenderRect = { left: 0, top: 0, width: w, height: h };
    return { left: 0, top: 0, width: w, height: h };
  }
  const baseRect = containedImageRect(w, h, calibrationImage.naturalWidth, calibrationImage.naturalHeight);
  calibrationImage.baseRenderRect = baseRect;
  calibrationImage.displayPan = clampImagePan(calibrationImage.displayPan, baseRect, calibrationImage.displayZoom);
  return zoomedImageRect(baseRect, calibrationImage.displayZoom, calibrationImage.displayPan);
}

function updateCalibrationReadout() {
  const el = document.getElementById('calib-render-size');
  if (!el) return;
  if (!calibrationImage.enabled) {
    el.textContent = 'ei kuvaa';
    return;
  }
  const baseRect = calibrationImage.baseRenderRect;
  el.textContent = `${Math.round(baseRect.width)}×${Math.round(baseRect.height)} px ` +
    `@ ${formatZoom(calibrationImage.displayZoom)} ` +
    `(kuva ${calibrationImage.naturalWidth}×${calibrationImage.naturalHeight})`;
  updateCalibrationPanReadout();
}

function resize() {
  const rect = currentRenderRect();
  calibrationImage.renderRect = rect;
  renderer.setSize(rect.width, rect.height, false);
  renderer.domElement.style.left = rect.left + 'px';
  renderer.domElement.style.top = rect.top + 'px';
  renderer.domElement.style.width = rect.width + 'px';
  renderer.domElement.style.height = rect.height + 'px';
  calibrationBg.style.left = rect.left + 'px';
  calibrationBg.style.top = rect.top + 'px';
  calibrationBg.style.width = rect.width + 'px';
  calibrationBg.style.height = rect.height + 'px';
  camera.aspect = rect.width / rect.height;
  camera.updateProjectionMatrix();
  updateCalibrationReadout();
}
resize();
new ResizeObserver(resize).observe(vp);

(function loop() { requestAnimationFrame(loop); updateControlsWithRoll(); renderer.render(scene, camera); })();

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
  building_wall: 0x998877, building_roof: 0x887766, floor: 0x777755, ground: 0x556b2f,
};
const FCOL = { pad_footing: 0x6f6f75 };
const CCOL = {
  supported_on:  0xff4444,
  supported_by_pattern: 0x66ccff,
  continuous: 0x66ffee,
  notched_over:  0xff66cc,
  transfer_link: 0x66aaff,
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
  support_centerline: 'tukikeskilinja',
  support_inner_edge: 'tuen sisäreuna',
  support_outer_edge: 'tuen ulkoreuna',
};
const REACTION_DISTRIBUTION_LABELS = {
  point: 'pistekuorma',
  uniform_over_supported_member_width: 'tasainen tuetun jäsenen leveydelle',
  uniform_over_width: 'tasainen annetulle leveydelle',
};
const HIGHLIGHT_MEMBER_COLOR = 0xffee66;
const SUPPORT_LINE_SHIFT_COLOR = 0x99ff33;
const CONNECTION_DETAIL_PLATE_COLOR = 0xb0b7c3;
const CONNECTION_DETAIL_POINT_COLOR = 0xffe066;
const CONNECTION_DETAIL_BOLT_COLOR = 0xf7c948;
const SOLAR_PANEL_GRID_COLOR = 0xe8f2ff;
const OPENING_EDGE_COLOR = 0xffdd33;

// ── Scene groups (yksi per ladattu geometria) ─────────────────────────────────
const geoGroups = new Map(); // name → THREE.Group
const memberVisuals = new Map(); // geo::memberId → Array<THREE.Object3D>
let pickable = [];
let showConnectionMarkers = false;
let showAnalysisOverlays = false;
let highlightedMemberKeys = new Set();
let calibrationModelOpacity = 1.0;

function clamp01(value) {
  if (!Number.isFinite(value)) return 1;
  return Math.max(0, Math.min(1, value));
}

function objectMaterials(object) {
  if (!object?.material) return [];
  return Array.isArray(object.material) ? object.material.filter(Boolean) : [object.material];
}

function ensureOpacityBase(mat) {
  if (!mat.userData) mat.userData = {};
  if (mat.userData._calibrationBaseOpacity == null) {
    mat.userData._calibrationBaseOpacity = mat.opacity ?? 1;
  }
  if (mat.userData._calibrationBaseDepthWrite == null) {
    mat.userData._calibrationBaseDepthWrite = mat.depthWrite;
  }
}

function applyCalibrationOpacityToMaterial(mat, baseOpacity = null) {
  if (!mat || mat.opacity == null) return;
  ensureOpacityBase(mat);
  const base = baseOpacity ?? mat.userData._calibrationBaseOpacity ?? 1;
  const opacity = Math.max(0.02, Math.min(1, base * calibrationModelOpacity));
  mat.opacity = opacity;
  mat.transparent = opacity < 1 || mat.transparent;
  if (mat.depthWrite != null) {
    mat.depthWrite = opacity >= 0.82 && mat.userData._calibrationBaseDepthWrite !== false;
  }
  mat.needsUpdate = true;
}

function applyCalibrationOpacityToObject(object, baseOpacity = null) {
  for (const mat of objectMaterials(object)) applyCalibrationOpacityToMaterial(mat, baseOpacity);
}

function applyCalibrationModelOpacity() {
  for (const g of geoGroups.values()) {
    g.traverse(object => applyCalibrationOpacityToObject(object));
  }
}

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
    if (mat.opacity != null) {
      applyCalibrationOpacityToObject(object, Math.max(object.userData._baseOpacity ?? 1, 0.95));
    }
    return;
  }
  if (mat.color && object.userData._baseColor != null) mat.color.setHex(object.userData._baseColor);
  if (mat.emissive && object.userData._baseEmissive != null) mat.emissive.setHex(object.userData._baseEmissive);
  if (mat.opacity != null && object.userData._baseOpacity != null) {
    applyCalibrationOpacityToObject(object, object.userData._baseOpacity);
  }
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
    if (analysis.reaction_distribution) {
      const dist = analysis.reaction_distribution;
      lines.push(`reaktio: ${REACTION_DISTRIBUTION_LABELS[dist.type] ?? dist.type}`);
      if (dist.width_ref) lines.push(`reaktion leveys: ${dist.width_ref}`);
      if (dist.width_mm) lines.push(`reaktion leveys: ${dist.width_mm} mm`);
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
  const transfer = con.transfer;
  if (con.type === 'transfer_link' && transfer) {
    if (transfer.description) lines.push(`selite: ${transfer.description}`);
    const plateParts = [];
    if ((transfer.outer_plate_thickness_mm ?? 0) > 0) plateParts.push(`ulko ${transfer.outer_plate_thickness_mm} mm`);
    if ((transfer.inner_plate_thickness_mm ?? 0) > 0) plateParts.push(`sisa ${transfer.inner_plate_thickness_mm} mm`);
    if (plateParts.length) lines.push(`kaistalevyt: ${plateParts.join(' + ')}`);
    lines.push(`levykoko: ${transfer.strip_width_mm} x ${transfer.plate_height_mm} mm`);
    lines.push(`pultitus: ${transfer.fastener_count_per_member} x M${transfer.fastener_d_mm} ${transfer.fastener_grade} / jasen`);
    if (Array.isArray(transfer.fasteners) && transfer.fasteners.length) {
      lines.push(`pulttipaikat: ${transfer.fasteners.length} kpl`);
    }
  }
  const detail = con.detail;
  if (detail?.kind === 'plate_bracket') {
    const hostMember = detail.host_member === 'support_member'
      ? 'tuessa'
      : 'kannatetussa jäsenessä';
    lines.push(`liitosdetaili: levykannake (${hostMember})`);
    lines.push(`latat: ${detail.plates?.length ?? 0} x ${detail.plate_width_mm}x${detail.plate_thickness_mm} mm`);
    lines.push(`näkyvä / upotus: ${detail.visible_length_mm} + ${detail.embedded_length_mm} mm`);
    if ((detail.plates?.length ?? 0) === 2) {
      const sortedOffsets = [...detail.plates]
        .map(plate => Number(plate.offset_across_center_mm ?? 0))
        .sort((a, b) => a - b);
      const derivedGapMm = sortedOffsets[1] - sortedOffsets[0] - detail.plate_thickness_mm;
      lines.push(`laskennallinen vapaa väli: ${derivedGapMm} mm`);
    }
    for (const plate of detail.plates ?? []) {
      lines.push(`  levy ${plate.id}: offset ${plate.offset_across_center_mm ?? 0} mm`);
    }
    for (const point of detail.points ?? []) {
      const diameter = point.diameter_mm != null ? `d${point.diameter_mm}` : point.kind;
      lines.push(
        `  piste ${point.id}: ${point.plate_id}, ${diameter}, ` +
        `${point.distance_from_visible_end_mm} mm näkyvästä päästä`
      );
    }
  }
  const cuts = connectionCuts(con);
  if (cuts.length) lines.push(`cuts: ${cuts.map(cut => cutLabel(cut)).join(', ')}`);
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
  if (Array.isArray(surfaceObj.openings) && surfaceObj.openings.length) {
    const summary = surfaceObj.openings
      .slice(0, 4)
      .map(opening => `${opening.id} (${opening.type ?? 'opening'})`)
      .join(', ');
    lines.push(`openings: ${summary}${surfaceObj.openings.length > 4 ? `, +${surfaceObj.openings.length - 4} muuta` : ''}`);
  }
  if (Array.isArray(surfaceObj.supported_by) && surfaceObj.supported_by.length) {
    lines.push(`supported_by: ${summarizeMemberRefs(surfaceObj.supported_by)}`);
  }
  const rules = surfaceObj.load_transfer?.to_members;
  if (Array.isArray(rules) && rules.length) {
    for (const rule of rules) lines.push(`load_transfer: ${loadTransferRuleSummary(rule)}`);
  }
  return lines;
}

function openingTooltipLines(opening, surfaceObj) {
  const type = opening.type ?? 'opening';
  return [
    `${opening.id}  —  ${type}`,
    `parent: ${surfaceObj.id}`,
    `u/v: ${opening.u0_mm} / ${opening.v0_mm} mm`,
    `size: ${opening.width_mm} × ${opening.height_mm} mm`,
  ];
}

function foundationTooltipLines(foundation) {
  const size = foundation.size_mm ?? {};
  const lines = [`${foundation.id}  —  ${foundation.type ?? 'foundation'}`];
  if (foundation.supports) lines.push(`pilari: ${foundation.supports}`);
  if (size.x && size.y && size.z) lines.push(`koko: ${size.x} x ${size.y} x ${size.z} mm`);
  if (foundation.material) lines.push(`materiaali: ${foundation.material}`);
  if (foundation.ground_ref) lines.push(`maanpinta: ${foundation.ground_ref}`);
  if (foundation.frost_insulated) lines.push('routaeristetty');
  if (foundation.soil_cover?.gamma_kNm3 != null) {
    lines.push(`maanpeite γ = ${foundation.soil_cover.gamma_kNm3} kN/m³`);
  }
  if (foundation.anchorage?.type) {
    lines.push(`ankkurointi: ${foundation.anchorage.description ?? foundation.anchorage.type}`);
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
function memberSectionRotationDeg(member) {
  return Number(member?.section_rotation_deg ?? 0);
}

function memberFrame(start, end, sectionRotationDeg = 0) {
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
  if (Math.abs(sectionRotationDeg) > 1e-9) {
    const roll = new THREE.Quaternion().setFromAxisAngle(dir, THREE.MathUtils.degToRad(sectionRotationDeg));
    yAx.applyQuaternion(roll).normalize();
    zAx.applyQuaternion(roll).normalize();
  }
  return {
    dir,
    yAx,
    zAx,
    quaternion: new THREE.Quaternion().setFromRotationMatrix(
      new THREE.Matrix4().makeBasis(dir, yAx, zAx)
    ),
  };
}

function memberQuaternion(start, end, sectionRotationDeg = 0) {
  return memberFrame(start, end, sectionRotationDeg)?.quaternion ?? new THREE.Quaternion();
}

function cutLocalFrame(memberInfo, memberEnd) {
  const frame = memberFrame(memberInfo.start3, memberInfo.end3, memberInfo.sectionRotationDeg);
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

function memberSectionDims(profile) {
  const hMm = profile?.h_mm, bMm = profile?.b_mm, cnt = profile?.count ?? 1;
  if (!hMm || !bMm) return [null, null];
  return [hMm, bMm * cnt];
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
        sectionRotationDeg: memberSectionRotationDeg(m),
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
    const supportFrame = memberFrame(supportInfo.start3, supportInfo.end3, supportInfo.sectionRotationDeg);
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
  const frame = memberFrame(memberInfo.start3, memberInfo.end3, memberInfo.sectionRotationDeg);
  const [boxH, boxB] = memberSectionDims(memberInfo.profile);
  if (!frame || !boxH || !boxB) return 0;
  const dir = axisDir.clone().normalize();
  const halfLen = memberInfo.start3.distanceTo(memberInfo.end3) / 2;
  return Math.abs(dir.dot(frame.dir)) * halfLen
    + Math.abs(dir.dot(frame.yAx)) * (boxH / 2)
    + Math.abs(dir.dot(frame.zAx)) * (boxB / 2);
}

function transferLinkVisualGeometry(con, memberIndex) {
  if (!con.at || con.type !== 'transfer_link' || !con.transfer) return null;
  const memberA = con.members?.[0] ? memberIndex.get(con.members[0]) : null;
  const memberB = con.members?.[1] ? memberIndex.get(con.members[1]) : null;
  if (!memberA || !memberB) return null;

  const frameA = memberFrame(memberA.start3, memberA.end3, memberA.sectionRotationDeg);
  const frameB = memberFrame(memberB.start3, memberB.end3, memberB.sectionRotationDeg);
  if (!frameA || !frameB) return null;

  const [, widthA] = memberSectionDims(memberA.profile);
  const [, widthB] = memberSectionDims(memberB.profile);
  const referenceMember = (widthA ?? 0) >= (widthB ?? 0) ? memberA : memberB;
  const referenceFrame = referenceMember === memberA ? frameA : frameB;
  const referenceWidthMm = Math.max(widthA ?? 0, widthB ?? 0);
  if (referenceWidthMm <= 1e-9) return null;

  const anchor = pt(con.at);
  return {
    anchor,
    axisDir: referenceFrame.dir.clone(),
    heightAxis: referenceFrame.yAx.clone(),
    normalAxis: referenceFrame.zAx.clone(),
    faceOffsetMm: referenceWidthMm / 2,
    referenceMemberId: referenceMember.member?.id ?? null,
  };
}

function addTransferLinkVisuals(g, pk, con, memberIndex, tooltipLines) {
  const geom = transferLinkVisualGeometry(con, memberIndex);
  if (!geom) return;
  const transfer = con.transfer;
  const plateSpecs = [
    { key: 'outer', thicknessMm: Number(transfer.outer_plate_thickness_mm ?? 0), sign: 1 },
    { key: 'inner', thicknessMm: Number(transfer.inner_plate_thickness_mm ?? 0), sign: -1 },
  ].filter(spec => spec.thicknessMm > 1e-9);
  if (!plateSpecs.length) return;

  const detailUserData = {
    id: con.id,
    kind: `${con.type ?? 'connection'} / detail`,
    _geoName: g.userData.geoName,
    _isAlwaysVisibleConnectionDetail: true,
    tooltipLines,
  };
  const markerUserData = {
    id: con.id,
    kind: `${con.type ?? 'connection'} / detail`,
    _geoName: g.userData.geoName,
    _isConnectionMarker: true,
    tooltipLines,
  };
  const plateQuat = new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().makeBasis(geom.axisDir, geom.heightAxis, geom.normalAxis)
  );
  for (const spec of plateSpecs) {
    const plateGeo = new THREE.BoxGeometry(
      Number(transfer.strip_width_mm ?? 0),
      Number(transfer.plate_height_mm ?? 0),
      spec.thicknessMm
    );
    const plateCenter = geom.anchor.clone().add(
      geom.normalAxis.clone().multiplyScalar(spec.sign * (geom.faceOffsetMm + spec.thicknessMm / 2))
    );

    const plateMesh = new THREE.Mesh(
      plateGeo,
      new THREE.MeshBasicMaterial({
        color: CONNECTION_DETAIL_PLATE_COLOR,
        transparent: true,
        opacity: 0.42,
        depthWrite: false,
      })
    );
    plateMesh.position.copy(plateCenter);
    plateMesh.quaternion.copy(plateQuat);
    plateMesh.visible = true;
    plateMesh.userData = { ...detailUserData, id: `${con.id}.${spec.key}` };
    g.add(plateMesh);
    pk.push(plateMesh);

    const plateEdges = new THREE.LineSegments(
      new THREE.EdgesGeometry(plateGeo),
      new THREE.LineBasicMaterial({
        color: CCOL.transfer_link ?? CONNECTION_DETAIL_PLATE_COLOR,
        transparent: true,
        opacity: 0.95,
        depthWrite: false,
      })
    );
    plateEdges.position.copy(plateCenter);
    plateEdges.quaternion.copy(plateQuat);
    plateEdges.visible = true;
    plateEdges.userData = { ...detailUserData, id: `${con.id}.${spec.key}.edges` };
    g.add(plateEdges);
    pk.push(plateEdges);
  }

  const fasteners = Array.isArray(transfer.fasteners) ? transfer.fasteners : [];
  if (!fasteners.length) return;
  const outerFaceMm = Number(transfer.outer_plate_thickness_mm ?? 0) > 1e-9
    ? geom.faceOffsetMm + Number(transfer.outer_plate_thickness_mm ?? 0) / 2
    : geom.faceOffsetMm;
  const innerFaceMm = Number(transfer.inner_plate_thickness_mm ?? 0) > 1e-9
    ? -(geom.faceOffsetMm + Number(transfer.inner_plate_thickness_mm ?? 0) / 2)
    : -geom.faceOffsetMm;
  const boltLengthMm = outerFaceMm - innerFaceMm;
  const boltCenterShiftMm = 0.5 * (outerFaceMm + innerFaceMm);
  const boltQuat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), geom.normalAxis);
  for (const fastener of fasteners) {
    const diameterMm = Number(fastener.diameter_mm ?? transfer.fastener_d_mm ?? 0);
    if (diameterMm <= 1e-9) continue;
    const boltCenter = geom.anchor.clone()
      .add(geom.axisDir.clone().multiplyScalar(Number(fastener.offset_along_strip_mm ?? 0)))
      .add(geom.heightAxis.clone().multiplyScalar(Number(fastener.offset_height_mm ?? 0)))
      .add(geom.normalAxis.clone().multiplyScalar(boltCenterShiftMm));
    const boltMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(diameterMm / 2, diameterMm / 2, boltLengthMm, 14),
      new THREE.MeshBasicMaterial({
        color: CONNECTION_DETAIL_BOLT_COLOR,
        transparent: true,
        opacity: 0.95,
        depthWrite: false,
      })
    );
    boltMesh.position.copy(boltCenter);
    boltMesh.quaternion.copy(boltQuat);
    boltMesh.visible = true;
    boltMesh.userData = { ...detailUserData, id: `${con.id}.fastener.${fastener.id}` };
    g.add(boltMesh);
    pk.push(boltMesh);
  }
}

function closestMemberEnd(memberInfo, point3) {
  const axisPoint = projectPointToLine(point3, memberInfo.start3, memberInfo.end3);
  return axisPoint.distanceToSquared(memberInfo.start3) <= axisPoint.distanceToSquared(memberInfo.end3)
    ? 'axis_start'
    : 'axis_end';
}

function plateBracketSupportGeometry(con, memberIndex) {
  if (!con.at) return null;
  const detail = con.detail;
  const hostIndex = detail?.host_member === 'support_member' ? 1 : 0;
  const otherIndex = hostIndex === 0 ? 1 : 0;
  const hostInfo = con.members?.[hostIndex] ? memberIndex.get(con.members[hostIndex]) : null;
  const otherInfo = con.members?.[otherIndex] ? memberIndex.get(con.members[otherIndex]) : null;
  if (!hostInfo || !otherInfo) return null;
  const anchor = pt(con.at);
  const hostFrame = memberFrame(hostInfo.start3, hostInfo.end3, hostInfo.sectionRotationDeg);
  const otherFrame = memberFrame(otherInfo.start3, otherInfo.end3, otherInfo.sectionRotationDeg);
  if (!hostFrame || !otherFrame) return null;

  const hostEnd = closestMemberEnd(hostInfo, anchor);
  const hostFace = hostEnd === 'axis_end'
    ? hostInfo.end3.clone()
    : hostInfo.start3.clone();
  const plateAxisDir = hostEnd === 'axis_end'
    ? hostFrame.dir.clone()
    : hostFrame.dir.clone().negate();

  let plateWidthAxis = otherFrame.dir.clone().projectOnPlane(plateAxisDir);
  if (plateWidthAxis.lengthSq() < 1e-8) {
    plateWidthAxis = otherFrame.yAx.clone().projectOnPlane(plateAxisDir);
  }
  if (plateWidthAxis.lengthSq() < 1e-8) {
    plateWidthAxis = new THREE.Vector3(1, 0, 0).projectOnPlane(plateAxisDir);
  }
  if (plateWidthAxis.lengthSq() < 1e-8) return null;
  plateWidthAxis.normalize();

  let plateThicknessAxis = new THREE.Vector3().crossVectors(plateAxisDir, plateWidthAxis);
  if (plateThicknessAxis.lengthSq() < 1e-8) {
    plateThicknessAxis = otherFrame.zAx.clone().projectOnPlane(plateAxisDir);
  }
  if (plateThicknessAxis.lengthSq() < 1e-8) return null;
  plateThicknessAxis.normalize();
  if (plateThicknessAxis.dot(otherFrame.zAx) < 0) {
    plateWidthAxis.negate();
    plateThicknessAxis.negate();
  }

  return {
    hostFace,
    plateAxisDir,
    plateWidthAxis,
    plateThicknessAxis,
  };
}

function plateBracketPlateCenter(detail, plate, supportGeom) {
  const axisShiftMm = (detail.visible_length_mm - detail.embedded_length_mm) / 2;
  return supportGeom.hostFace.clone()
    .add(supportGeom.plateAxisDir.clone().multiplyScalar(axisShiftMm))
    .add(supportGeom.plateThicknessAxis.clone().multiplyScalar(Number(plate.offset_across_center_mm ?? 0)));
}

function plateBracketPointPosition(detail, plate, point, supportGeom) {
  const alongMm = detail.visible_length_mm - (point.distance_from_visible_end_mm ?? 0);
  return supportGeom.hostFace.clone()
    .add(supportGeom.plateAxisDir.clone().multiplyScalar(alongMm))
    .add(supportGeom.plateThicknessAxis.clone().multiplyScalar(
      Number(plate.offset_across_center_mm ?? 0) + (point.offset_across_plate_mm ?? 0)
    ))
    .add(supportGeom.plateWidthAxis.clone().multiplyScalar(point.offset_vertical_mm ?? 0));
}

function addPlateBracketVisuals(g, pk, con, memberIndex, tooltipLines) {
  const detail = con.detail;
  if (detail?.kind !== 'plate_bracket' || !con.at) return;
  const supportGeom = plateBracketSupportGeometry(con, memberIndex);
  if (!supportGeom) return;

  const detailUserData = {
    id: con.id,
    kind: `${con.type ?? 'connection'} / detail`,
    _geoName: g.userData.geoName,
    _isAlwaysVisibleConnectionDetail: true,
    tooltipLines,
  };
  const plateQuat = new THREE.Quaternion().setFromRotationMatrix(
    new THREE.Matrix4().makeBasis(
      supportGeom.plateAxisDir,
      supportGeom.plateWidthAxis,
      supportGeom.plateThicknessAxis
    )
  );
  const plateLengthMm = detail.visible_length_mm + detail.embedded_length_mm;
  for (const plate of detail.plates ?? []) {
    const plateGeo = new THREE.BoxGeometry(
      plateLengthMm,
      detail.plate_width_mm,
      detail.plate_thickness_mm
    );
    const plateMesh = new THREE.Mesh(
      plateGeo,
      new THREE.MeshBasicMaterial({
        color: CONNECTION_DETAIL_PLATE_COLOR,
        transparent: true,
        opacity: 0.38,
        depthWrite: false,
      })
    );
    plateMesh.position.copy(plateBracketPlateCenter(detail, plate, supportGeom));
    plateMesh.quaternion.copy(plateQuat);
    plateMesh.visible = true;
    plateMesh.userData = { ...detailUserData };
    g.add(plateMesh);
    pk.push(plateMesh);

    const plateEdges = new THREE.LineSegments(
      new THREE.EdgesGeometry(plateGeo),
      new THREE.LineBasicMaterial({
        color: CONNECTION_DETAIL_PLATE_COLOR,
        transparent: true,
        opacity: 0.95,
        depthWrite: false,
      })
    );
    plateEdges.position.copy(plateMesh.position);
    plateEdges.quaternion.copy(plateQuat);
    plateEdges.visible = true;
    plateEdges.userData = { ...detailUserData };
    g.add(plateEdges);
    pk.push(plateEdges);
  }

  const plateById = new Map((detail.plates ?? []).map(plate => [plate.id, plate]));
  for (const point of detail.points ?? []) {
    const plate = plateById.get(point.plate_id);
    if (!plate) continue;
    const radiusMm = Math.max(10, (point.diameter_mm ?? 20) / 2);
    const pointMesh = new THREE.Mesh(
      new THREE.SphereGeometry(radiusMm, 12, 10),
      new THREE.MeshBasicMaterial({
        color: CONNECTION_DETAIL_POINT_COLOR,
        transparent: true,
        opacity: 0.95,
        depthWrite: false,
      })
    );
    pointMesh.position.copy(plateBracketPointPosition(detail, plate, point, supportGeom));
    pointMesh.visible = true;
    pointMesh.userData = { ...detailUserData };
    g.add(pointMesh);
    pk.push(pointMesh);
  }
}

function connectionCuts(con) {
  return Array.isArray(con.cuts) && con.cuts.length
    ? con.cuts
    : (con.notch ? [con.notch] : []);
}

function cutLabel(cut) {
  if (cut.kind === 'rect_notch' || cut.kind === 'bevel_notch') {
    return `${cut.kind}(${cut.side})`;
  }
  if (cut.kind === 'end_bevel_cut') {
    return `${cut.kind}(${cut.cut_from})`;
  }
  return cut.kind;
}

function cutOffsetMm(cut) {
  return cut.offset_mm ?? cut.x_from_support_edge_mm ?? 0;
}

function cutMemberEnd(cut, con, memberInfo) {
  const ref = cut.reference;
  if (ref === 'axis_start' || ref === 'axis_end') return ref;
  if (!memberInfo) return null;
  const axisPoint = projectPointToLine(pt(con.at), memberInfo.start3, memberInfo.end3);
  return axisPoint.distanceToSquared(memberInfo.start3) <= axisPoint.distanceToSquared(memberInfo.end3)
    ? 'axis_start'
    : 'axis_end';
}

function resolveCutAnchor(cut, con, memberInfo, supportInfo) {
  const memberEnd = cutMemberEnd(cut, con, memberInfo);
  const frame = cutLocalFrame(memberInfo, memberEnd);
  if (!frame) return null;

  const inwardDir = frame.xAx.clone();
  const axisPoint = projectPointToLine(pt(con.at), memberInfo.start3, memberInfo.end3);
  const endPoint = memberEnd === 'axis_end'
    ? memberInfo.end3.clone()
    : memberInfo.start3.clone();
  const ref = cut.reference;
  const supportHalf = projectedMemberHalfExtent(supportInfo, inwardDir);

  let anchor = axisPoint.clone();
  switch (ref) {
    case 'axis_start':
      anchor = memberInfo.start3.clone();
      break;
    case 'axis_end':
      anchor = memberInfo.end3.clone();
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

function birdsmouthGeometry(notch, boxH, boxB, memberInfo, supportInfo, con) {
  const frame = cutLocalFrame(memberInfo, cutMemberEnd(notch, con, memberInfo));
  if (!frame) return null;

  let supportNormal = null;
  if (supportInfo) {
    const supportFrame = memberFrame(supportInfo.start3, supportInfo.end3, supportInfo.sectionRotationDeg);
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

function notchGeometry(notch, boxH, boxB, memberInfo, supportInfo, con) {
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
  if (notch.kind === 'bevel_notch') {
    return notch.side === 'top'
      ? extrudeNotchPolygon([
          [0, top],
          [notch.length_mm, top],
          [0, top - notch.depth_mm],
        ], boxB)
      : extrudeNotchPolygon([
          [0, bottom],
          [notch.length_mm, bottom],
          [0, bottom + notch.depth_mm],
        ], boxB);
  }
  if (notch.kind === 'birdsmouth_notch') {
    return birdsmouthGeometry(notch, boxH, boxB, memberInfo, supportInfo, con);
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
  const [boxH, boxB] = memberSectionDims(memberInfo.profile);
  if (!boxH || !boxB) return;

  const notchGeo = notchGeometry(notch, boxH, boxB, memberInfo, supportInfo, con);
  if (!notchGeo) return;

  const localFrame = cutLocalFrame(memberInfo, cutMemberEnd(notch, con, memberInfo));
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

function addMemberMesh(g, pk, start3, end3, profile, sectionRotationDeg, color, userData) {
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

  // section_rotation_deg=0 → h_mm is vertical; positive values roll about axis_start→axis_end
  const [boxH, boxB] = memberSectionDims(profile);

  const q   = memberQuaternion(start3, end3, sectionRotationDeg);
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
      addMemberMesh(g, pk, pt(a), pt(b), m.profile, memberSectionRotationDeg(m), col,
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

function polygonToSurfacePoints2(poly, frame) {
  return poly.map(p => {
    const rel = pt(p).sub(frame.origin);
    return new THREE.Vector2(rel.dot(frame.xAxis), rel.dot(frame.yAxis));
  });
}

function buildSurfaceGeometry(poly, thicknessMm = 0, openingPolygons = []) {
  const frame = surfaceFrame(poly);
  if (!frame) return null;

  let points2 = polygonToSurfacePoints2(poly, frame);
  if (signedArea2D(points2) < 0) points2 = points2.reverse();

  const shape = new THREE.Shape(points2);
  for (const openingPoly of openingPolygons ?? []) {
    if (!Array.isArray(openingPoly) || openingPoly.length < 3) continue;
    let holePoints = polygonToSurfacePoints2(openingPoly, frame);
    if (signedArea2D(holePoints) > 0) holePoints = holePoints.reverse();
    shape.holes.push(new THREE.Path(holePoints));
  }

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

function openingPolygons(surfaceObj) {
  return (surfaceObj.openings ?? [])
    .map(opening => opening.polygon)
    .filter(poly => Array.isArray(poly) && poly.length >= 3);
}

function surfaceLocalBounds(poly, frame) {
  const coords = poly.map(p => {
    const rel = pt(p).sub(frame.origin);
    return {
      u: rel.dot(frame.xAxis),
      v: rel.dot(frame.yAxis),
    };
  });
  return {
    uMin: Math.min(...coords.map(p => p.u)),
    uMax: Math.max(...coords.map(p => p.u)),
    vMin: Math.min(...coords.map(p => p.v)),
    vMax: Math.max(...coords.map(p => p.v)),
  };
}

function surfacePoint(frame, u, v, normalOffsetMm = 0) {
  return frame.origin.clone()
    .add(frame.xAxis.clone().multiplyScalar(u))
    .add(frame.yAxis.clone().multiplyScalar(v))
    .add(frame.normal.clone().multiplyScalar(normalOffsetMm));
}

function addSolarPanelGrid(g, surfaceObj, poly, thicknessMm, opacity) {
  if (surfaceObj.type !== 'solar_panel_array') return;
  const count = surfaceObj.count;
  const nx = Number(count?.nx ?? 0);
  const ny = Number(count?.ny ?? 0);
  if (nx < 2 && ny < 2) return;

  const frame = surfaceFrame(poly);
  if (!frame) return;
  const bounds = surfaceLocalBounds(poly, frame);
  const points = [];
  const normalOffsetMm = thicknessMm / 2 + 4;

  for (let i = 1; i < nx; i++) {
    const u = bounds.uMin + (bounds.uMax - bounds.uMin) * i / nx;
    points.push(
      surfacePoint(frame, u, bounds.vMin, normalOffsetMm),
      surfacePoint(frame, u, bounds.vMax, normalOffsetMm),
    );
  }
  for (let j = 1; j < ny; j++) {
    const v = bounds.vMin + (bounds.vMax - bounds.vMin) * j / ny;
    points.push(
      surfacePoint(frame, bounds.uMin, v, normalOffsetMm),
      surfacePoint(frame, bounds.uMax, v, normalOffsetMm),
    );
  }
  if (!points.length) return;

  const grid = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({
      color: SOLAR_PANEL_GRID_COLOR,
      transparent: true,
      opacity: Math.min(opacity * 3, 0.95),
      depthWrite: false,
      depthTest: false,
    })
  );
  grid.userData = {
    id: `${surfaceObj.id}.panel_grid`,
    kind: 'solar_panel_grid',
    _geoName: g.userData.geoName,
  };
  g.add(grid);
}

function addOpeningOutlines(g, pk, surfaceObj, poly, thicknessMm, opacity) {
  if (!Array.isArray(surfaceObj.openings) || surfaceObj.openings.length === 0) return;
  const frame = surfaceFrame(poly);
  if (!frame) return;
  const normalOffsetMm = thicknessMm / 2 + 5;

  for (const opening of surfaceObj.openings) {
    const openingPoly = opening.polygon;
    if (!Array.isArray(openingPoly) || openingPoly.length < 3) continue;
    const points = openingPoly
      .map(p => pt(p).add(frame.normal.clone().multiplyScalar(normalOffsetMm)));
    points.push(points[0].clone());

    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({
        color: OPENING_EDGE_COLOR,
        transparent: true,
        opacity: Math.min(opacity * 4, 1),
        depthWrite: false,
        depthTest: false,
      })
    );
    line.renderOrder = 24;
    line.userData = {
      id: opening.id,
      kind: opening.type ?? 'opening',
      _geoName: g.userData.geoName,
      tooltipLines: openingTooltipLines(opening, surfaceObj),
    };
    g.add(line);
    pk.push(line);
  }
}

function expandViewerMemberRefs(memberRefs, memberIndex) {
  const expanded = [];
  for (const memberRef of memberRefs ?? []) {
    if (memberIndex?.has(memberRef)) {
      expanded.push(memberRef);
      continue;
    }
    const prefix = `${memberRef}.`;
    const matches = [...(memberIndex?.keys() ?? [])]
      .filter(memberId => {
        const suffix = memberId.slice(prefix.length);
        return memberId.startsWith(prefix) && suffix !== '' && Number.isInteger(Number(suffix));
      })
      .sort((a, b) => a.localeCompare(b, undefined, { numeric: true }));
    expanded.push(...matches.length ? matches : [memberRef]);
  }
  return [...new Set(expanded)];
}

function surfaceHighlightMemberRefs(surfaceObj, memberIndex) {
  const refs = [
    ...(surfaceObj.supported_by ?? []),
    ...(surfaceObj.load_transfer?.to_members ?? []).flatMap(rule => rule.member_refs ?? []),
  ];
  return expandViewerMemberRefs(refs, memberIndex);
}

function addSurfaces(g, pk, list, opacity, memberIndex) {
  const geoName = g.userData.geoName;
  for (const s of list ?? []) {
    const poly = s.polygon;
    if (!poly || poly.length < 3) continue;
    const col = SCOL[s.type] ?? 0xaaaaaa;
    const thicknessMm = Math.max(0, s.thickness_mm ?? 0);
    const mg = buildSurfaceGeometry(poly, thicknessMm, openingPolygons(s));
    if (!mg) continue;
    const mesh = new THREE.Mesh(mg, new THREE.MeshBasicMaterial(
      { color: col, transparent: true, opacity, side: THREE.DoubleSide }
    ));
    const memberRefs = surfaceHighlightMemberRefs(s, memberIndex);
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
    addSolarPanelGrid(g, s, poly, thicknessMm, opacity);
    addOpeningOutlines(g, pk, s, poly, thicknessMm, opacity);
  }
}

// ── Foundation meshes ─────────────────────────────────────────────────────────
function foundationPlacement(foundation, memberIndex) {
  const size = foundation.size_mm;
  if (!size?.x || !size?.y || !size?.z) return null;
  const supportInfo = foundation.supports ? memberIndex.get(foundation.supports) : null;
  const center = foundation.center ?? null;
  const base = supportInfo?.member?.base ?? null;
  const x = center?.x ?? base?.x;
  const y = center?.y ?? base?.y;
  const topZ = foundation.top_z ?? base?.z;
  if (x == null || y == null || topZ == null) return null;
  return {
    center: pt({ x, y, z: topZ - size.z / 2 }),
    size,
  };
}

function addFoundations(g, pk, foundations, memberIndex) {
  const geoName = g.userData.geoName;
  for (const foundation of foundations ?? []) {
    const placement = foundationPlacement(foundation, memberIndex);
    if (!placement) continue;
    const { center, size } = placement;
    const col = FCOL[foundation.type] ?? 0x777777;
    const boxGeo = new THREE.BoxGeometry(size.x, size.z, size.y);
    const mat = new THREE.MeshLambertMaterial({
      color: col,
      transparent: true,
      opacity: 0.42,
      side: THREE.DoubleSide,
    });
    const mesh = new THREE.Mesh(boxGeo, mat);
    mesh.position.copy(center);
    mesh.userData = {
      id: foundation.id,
      kind: foundation.type ?? 'foundation',
      _geoName: geoName,
      tooltipLines: foundationTooltipLines(foundation),
    };
    g.add(mesh);
    pk.push(mesh);

    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(boxGeo),
      new THREE.LineBasicMaterial({ color: 0x111111, transparent: true, opacity: 0.55 })
    );
    edges.position.copy(center);
    edges.userData = { ...mesh.userData };
    g.add(edges);
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
    const isTransferLinkDetail = con.type === 'transfer_link' && !!con.transfer;
    if (!isTransferLinkDetail) {
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
    }
    addTransferLinkVisuals(g, pk, con, memberIndex, tooltipLines);
    addPlateBracketVisuals(g, pk, con, memberIndex, tooltipLines);
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
          const [boxH, boxB] = memberSectionDims(memberInfo.profile);
          if (boxH && boxB) {
            const notchGeo = notchGeometry(cuts[i], boxH, boxB, memberInfo, supportInfo, con);
            const localFrame = cutLocalFrame(memberInfo, cutMemberEnd(cuts[i], con, memberInfo));
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
  updateControlsWithRoll();
}

function captureViewState() {
  return {
    position: camera.position.clone(),
    target: controls.target.clone(),
    fov: camera.fov,
    zoom: camera.zoom,
    rollRad: cameraRollRad,
  };
}

function restoreViewState(state) {
  if (!state) return false;
  camera.position.copy(state.position);
  controls.target.copy(state.target);
  if (state.fov != null) camera.fov = state.fov;
  camera.zoom = state.zoom;
  if (state.rollRad != null) cameraRollRad = state.rollRad;
  camera.updateProjectionMatrix();
  updateControlsWithRoll();
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
  addSurfaces(g, pickable, resolved.surfaces, 0.30, memberIndex);
  addSurfaces(g, pickable, resolved.reference_surfaces, 0.12, memberIndex);
  addFoundations(g, pickable, resolved.foundations, memberIndex);
  const notchCutsMap = addConnections(g, pickable, resolved.connections, memberIndex);
  await applyCSGCuts(g, pickable, memberIndex, notchCutsMap);
  setConnectionMarkerVisibility(showConnectionMarkers);
  setAnalysisOverlayVisibility(showAnalysisOverlays);
  applyCalibrationModelOpacity();
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
const editorPanel = document.getElementById('editor-panel');
const statusEl = document.getElementById('status');
let debounce = null;

function setStatus(msg, cls) { statusEl.textContent = msg; statusEl.className = cls ?? ''; }

const CALIBRATION_LOCAL_KEY = 'terassi.viewer.cameraCalibration';
const calibPanel = document.getElementById('calibration-panel');
const calibBtn = document.getElementById('btn-calibration');
const calibImagePath = document.getElementById('calib-image-path');
const calibImageOpacity = document.getElementById('calib-image-opacity');
const calibImageZoom = document.getElementById('calib-image-zoom');
const calibImageZoomValue = document.getElementById('calib-image-zoom-value');
const calibImagePanMode = document.getElementById('calib-image-pan-mode');
const calibImagePanReset = document.getElementById('calib-image-pan-reset');
const calibImagePanValue = document.getElementById('calib-image-pan-value');
const calibModelOpacity = document.getElementById('calib-model-opacity');
const calibModelOpacityValue = document.getElementById('calib-model-opacity-value');
const calibCameraJsonPath = document.getElementById('calib-camera-json-path');
const calibImportCameraJson = document.getElementById('calib-import-camera-json');
const calibCameraJson = document.getElementById('calib-camera-json');
editorPanel.appendChild(calibPanel);
const calibFields = {
  posX: document.getElementById('calib-pos-x'),
  posY: document.getElementById('calib-pos-y'),
  posZ: document.getElementById('calib-pos-z'),
  targetX: document.getElementById('calib-target-x'),
  targetY: document.getElementById('calib-target-y'),
  targetZ: document.getElementById('calib-target-z'),
  fov: document.getElementById('calib-fov'),
  zoom: document.getElementById('calib-zoom'),
  roll: document.getElementById('calib-roll'),
};
let syncingCalibrationInputs = false;
let calibrationPanMode = false;
let calibrationPanDrag = null;

function normalizeRepoAssetPath(path) {
  let normalized = path.trim();
  if (normalized.startsWith('./')) normalized = normalized.slice(2);
  while (normalized.startsWith('/')) normalized = normalized.slice(1);
  return normalized;
}

function assetUrlForImagePath(value) {
  const path = value.trim();
  if (/^(https?:|data:|blob:)/i.test(path)) return path;
  const normalized = normalizeRepoAssetPath(path);
  return '/assets/' + normalized.split('/').map(encodeURIComponent).join('/');
}

function cameraJsonPathForImagePath(value) {
  const path = value.trim();
  if (!path || /^(https?:|data:|blob:)/i.test(path)) return 'viewer_camera.json';
  const normalized = normalizeRepoAssetPath(path);
  const slash = normalized.lastIndexOf('/');
  const dir = slash >= 0 ? normalized.slice(0, slash + 1) : '';
  const file = slash >= 0 ? normalized.slice(slash + 1) : normalized;
  const dot = file.lastIndexOf('.');
  const stem = dot > 0 ? file.slice(0, dot) : file;
  return dir + stem + '_viewer_camera.json';
}

function updateCameraJsonPathDefault(force = false) {
  const nextPath = cameraJsonPathForImagePath(calibImagePath.value);
  if (force || !calibCameraJsonPath.value.trim()) {
    calibCameraJsonPath.value = nextPath;
  }
}

function threeToGeometry(v) {
  return { x: v.x, y: v.z, z: v.y };
}

function geometryToThree(x, y, z) {
  return new THREE.Vector3(x, z, y);
}

function vectorPayload(v) {
  return {
    x: Number(v.x.toFixed(6)),
    y: Number(v.y.toFixed(6)),
    z: Number(v.z.toFixed(6)),
  };
}

function numericFieldValue(field) {
  const value = Number(field.value);
  return Number.isFinite(value) ? value : null;
}

function formatZoom(value) {
  return (Math.round(Number(value) * 100) / 100) + '×';
}

function updateCalibrationPanReadout() {
  const el = document.getElementById('calib-image-pan-value');
  if (!el) return;
  const pan = calibrationImage.displayPan;
  el.textContent = `x ${Math.round(pan.x)} px, y ${Math.round(pan.y)} px`;
}

function setCalibrationImageZoom(value, refreshJson = true) {
  const zoom = THREE.MathUtils.clamp(Number(value) || 1, 1, 3);
  calibrationImage.displayZoom = zoom;
  calibImageZoom.value = String(zoom);
  calibImageZoomValue.textContent = formatZoom(zoom);
  resize();
  if (refreshJson) updateCalibrationPanelFromCamera();
}

function setCalibrationImagePan(x, y, refreshJson = true) {
  calibrationImage.displayPan = clampImagePan({ x, y });
  resize();
  updateCalibrationPanReadout();
  if (refreshJson) updateCalibrationPanelFromCamera();
}

function setCalibrationPanMode(enabled) {
  calibrationPanMode = Boolean(enabled);
  if (!calibrationPanMode && calibrationPanDrag) {
    calibrationPanDrag = null;
    controls.enabled = true;
    vp.classList.remove('calibration-panning');
  }
  calibImagePanMode.classList.toggle('active', calibrationPanMode);
  calibImagePanMode.textContent = calibrationPanMode ? 'Zoom-kohdan siirto päällä' : 'Siirrä zoom-kohtaa';
  vp.classList.toggle('calibration-pan', calibrationPanMode);
}

function setCalibrationPanelOpen(open) {
  calibPanel.hidden = !open;
  editor.hidden = open;
  if (!open) setCalibrationPanMode(false);
  calibBtn.classList.toggle('active', open);
  calibBtn.setAttribute('aria-pressed', String(open));
  calibBtn.textContent = open ? '● Kuvakalibrointi' : '○ Kuvakalibrointi';
  updateCameraJsonPathDefault(false);
  if (open && !calibrationImage.enabled && calibImagePath.value.trim()) {
    loadCalibrationImage(calibImagePath.value);
  }
  updateCalibrationPanelFromCamera();
}

function cameraCalibrationPayload() {
  applyCameraRoll();
  camera.updateMatrixWorld(true);
  const rect = calibrationImage.baseRenderRect;
  return {
    version: 1,
    coordinate_mapping: 'geometry {x,y,z} -> three.js {x,z,y}',
    image: {
      path: calibrationImage.path,
      natural_size_px: {
        width: calibrationImage.naturalWidth,
        height: calibrationImage.naturalHeight,
      },
      render_size_px: {
        width: Number(rect.width.toFixed(3)),
        height: Number(rect.height.toFixed(3)),
      },
      render_offset_px: {
        left: Number(rect.left.toFixed(3)),
        top: Number(rect.top.toFixed(3)),
      },
      display_mode: 'contain',
      display_zoom: Number(calibrationImage.displayZoom.toFixed(3)),
      display_pan_px: {
        x: Number(calibrationImage.displayPan.x.toFixed(3)),
        y: Number(calibrationImage.displayPan.y.toFixed(3)),
      },
    },
    camera: {
      type: 'PerspectiveCamera',
      fov_deg: Number(camera.fov.toFixed(6)),
      roll_deg: Number(THREE.MathUtils.radToDeg(cameraRollRad).toFixed(6)),
      roll_rad: Number(cameraRollRad.toFixed(12)),
      aspect: Number(camera.aspect.toFixed(9)),
      zoom: Number(camera.zoom.toFixed(9)),
      near: camera.near,
      far: camera.far,
      position_three: vectorPayload(camera.position),
      target_three: vectorPayload(controls.target),
      up_three: vectorPayload(camera.up),
      position_geometry: vectorPayload(threeToGeometry(camera.position)),
      target_geometry: vectorPayload(threeToGeometry(controls.target)),
      projection_matrix: camera.projectionMatrix.elements.map(v => Number(v.toFixed(12))),
      matrix_world: camera.matrixWorld.elements.map(v => Number(v.toFixed(12))),
      matrix_world_inverse: camera.matrixWorldInverse.elements.map(v => Number(v.toFixed(12))),
    },
  };
}

function updateCalibrationPanelFromCamera() {
  if (syncingCalibrationInputs) return;
  syncingCalibrationInputs = true;
  const pos = threeToGeometry(camera.position);
  const target = threeToGeometry(controls.target);
  calibFields.posX.value = pos.x.toFixed(1);
  calibFields.posY.value = pos.y.toFixed(1);
  calibFields.posZ.value = pos.z.toFixed(1);
  calibFields.targetX.value = target.x.toFixed(1);
  calibFields.targetY.value = target.y.toFixed(1);
  calibFields.targetZ.value = target.z.toFixed(1);
  calibFields.fov.value = camera.fov.toFixed(2);
  calibFields.zoom.value = camera.zoom.toFixed(3);
  calibFields.roll.value = THREE.MathUtils.radToDeg(cameraRollRad).toFixed(2);
  calibCameraJson.value = JSON.stringify(cameraCalibrationPayload(), null, 2);
  syncingCalibrationInputs = false;
}

function applyCalibrationInputsToCamera() {
  if (syncingCalibrationInputs) return;
  const values = Object.fromEntries(
    Object.entries(calibFields).map(([key, field]) => [key, numericFieldValue(field)])
  );
  if (Object.values(values).some(value => value == null)) return;
  camera.position.copy(geometryToThree(values.posX, values.posY, values.posZ));
  controls.target.copy(geometryToThree(values.targetX, values.targetY, values.targetZ));
  camera.fov = THREE.MathUtils.clamp(values.fov, 1, 120);
  camera.zoom = Math.max(values.zoom, 0.01);
  cameraRollRad = THREE.MathUtils.degToRad(values.roll);
  camera.updateProjectionMatrix();
  updateControlsWithRoll();
  updateCalibrationPanelFromCamera();
}

function loadCalibrationImage(pathValue) {
  const path = pathValue.trim();
  if (!path) {
    setStatus('Anna taustakuvan polku', 'err');
    return;
  }
  updateCameraJsonPathDefault(true);
  calibrationBg.onload = () => {
    calibrationImage.enabled = true;
    calibrationImage.path = path;
    calibrationImage.naturalWidth = calibrationBg.naturalWidth;
    calibrationImage.naturalHeight = calibrationBg.naturalHeight;
    if (calibrationImage.pendingDisplayPan) {
      calibrationImage.displayPan = { ...calibrationImage.pendingDisplayPan };
      calibrationImage.pendingDisplayPan = null;
    }
    calibrationBg.style.display = 'block';
    resize();
    updateCalibrationPanelFromCamera();
    setStatus('Taustakuva ladattu', 'ok');
  };
  calibrationBg.onerror = () => {
    calibrationImage.enabled = false;
    calibrationBg.style.display = 'none';
    resize();
    setStatus('Taustakuvan lataus epäonnistui: ' + path, 'err');
  };
  calibrationBg.src = assetUrlForImagePath(path);
}

function hideCalibrationImage() {
  calibrationImage.enabled = false;
  calibrationBg.style.display = 'none';
  resize();
  updateCalibrationPanelFromCamera();
}

async function copyCalibrationCameraJson() {
  updateCalibrationPanelFromCamera();
  try {
    await navigator.clipboard.writeText(calibCameraJson.value);
    setStatus('Kameran JSON kopioitu', 'ok');
  } catch (_) {
    calibCameraJson.focus();
    calibCameraJson.select();
    setStatus('Kopioi JSON valitusta tekstistä', '');
  }
}

function saveCalibrationToLocalStorage() {
  updateCalibrationPanelFromCamera();
  localStorage.setItem(CALIBRATION_LOCAL_KEY, calibCameraJson.value);
  setStatus('Kalibrointi tallennettu selaimeen', 'ok');
}

async function saveCalibrationToPath() {
  updateCalibrationPanelFromCamera();
  const path = calibCameraJsonPath.value.trim();
  if (!path) {
    setStatus('Anna kamera-JSON-polku', 'err');
    return;
  }
  let calibration;
  try {
    calibration = JSON.parse(calibCameraJson.value);
  } catch (e) {
    setStatus('Kamera-JSON virhe: ' + e.message, 'err');
    return;
  }
  const btn = document.getElementById('calib-save-camera-json');
  const originalText = btn.textContent;
  btn.textContent = 'Tallennetaan…';
  try {
    const res = await fetch('/api/calibration-camera', {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ path, calibration }),
    });
    const data = await res.json();
    if (data.error) throw new Error(data.error);
    setStatus('Kamera-JSON tallennettu: ' + data.path, 'ok');
    btn.textContent = 'Tallennettu';
    setTimeout(() => { btn.textContent = originalText; }, 1600);
  } catch (e) {
    setStatus('Kamera-JSON tallennus epäonnistui: ' + e.message, 'err');
    btn.textContent = originalText;
  }
}

function applyCalibrationPayload(data) {
  const cam = data.camera ?? {};
  const pos = cam.position_geometry;
  const target = cam.target_geometry;
  if (!pos || !target) throw new Error('kamera-arvot puuttuvat');
  camera.position.copy(geometryToThree(pos.x, pos.y, pos.z));
  controls.target.copy(geometryToThree(target.x, target.y, target.z));
  if (cam.fov_deg != null) camera.fov = cam.fov_deg;
  if (cam.zoom != null) camera.zoom = cam.zoom;
  if (cam.roll_rad != null) {
    cameraRollRad = Number(cam.roll_rad);
  } else if (cam.roll_deg != null) {
    cameraRollRad = THREE.MathUtils.degToRad(Number(cam.roll_deg));
  } else {
    cameraRollRad = 0;
  }
  camera.updateProjectionMatrix();
  updateControlsWithRoll();
  setCalibrationImageZoom(data.image?.display_zoom ?? 1, false);
  if (data.image?.display_pan_px) {
    const pan = data.image.display_pan_px;
    calibrationImage.pendingDisplayPan = { x: Number(pan.x) || 0, y: Number(pan.y) || 0 };
    calibrationImage.displayPan = { ...calibrationImage.pendingDisplayPan };
  } else {
    calibrationImage.pendingDisplayPan = null;
    calibrationImage.displayPan = { x: 0, y: 0 };
  }
  if (data.image?.path) {
    calibImagePath.value = data.image.path;
    loadCalibrationImage(data.image.path);
  } else if (calibrationImage.pendingDisplayPan) {
    calibrationImage.pendingDisplayPan = null;
    setCalibrationImagePan(calibrationImage.displayPan.x, calibrationImage.displayPan.y, false);
  }
  updateCalibrationPanelFromCamera();
}

function restoreCalibrationFromRaw(raw, sourceLabel) {
  try {
    applyCalibrationPayload(JSON.parse(raw));
    setStatus('Kalibrointi palautettu: ' + sourceLabel, 'ok');
  } catch (e) {
    setStatus('Kalibroinnin palautus epäonnistui: ' + e.message, 'err');
  }
}

function restoreCalibrationFromLocalStorage() {
  const raw = localStorage.getItem(CALIBRATION_LOCAL_KEY);
  if (!raw) {
    setStatus('Selaimessa ei ole tallennettua kalibrointia', 'err');
    return;
  }
  restoreCalibrationFromRaw(raw, 'selain');
}

async function restoreCalibrationFromPath() {
  const path = calibCameraJsonPath.value.trim();
  if (!path) {
    setStatus('Anna kamera-JSON-polku', 'err');
    return;
  }
  try {
    const res = await fetch(assetUrlForImagePath(path), { cache: 'no-cache' });
    if (!res.ok) throw new Error(path + ' (' + res.status + ')');
    restoreCalibrationFromRaw(await res.text(), path);
  } catch (e) {
    setStatus('Kamera-JSON:n lataus epäonnistui: ' + e.message, 'err');
  }
}

async function restoreCalibrationFromFile(file) {
  if (!file) return;
  try {
    restoreCalibrationFromRaw(await file.text(), file.name);
  } catch (e) {
    setStatus('Kamera-JSON-tiedoston luku epäonnistui: ' + e.message, 'err');
  } finally {
    calibImportCameraJson.value = '';
  }
}

function canDragCalibrationZoom() {
  return (
    calibrationPanMode
    && !calibPanel.hidden
    && calibrationImage.enabled
    && calibrationImage.displayZoom > 1
  );
}

function startCalibrationZoomDrag(e) {
  if (!canDragCalibrationZoom() || e.button !== 0) return;
  e.preventDefault();
  e.stopPropagation();
  e.stopImmediatePropagation?.();
  vp.focus({ preventScroll: true });
  calibrationPanDrag = {
    pointerId: e.pointerId,
    startX: e.clientX,
    startY: e.clientY,
    startPan: { ...calibrationImage.displayPan },
  };
  controls.enabled = false;
  vp.classList.add('calibration-panning');
  try { vp.setPointerCapture(e.pointerId); } catch (_) {}
}

function moveCalibrationZoomDrag(e) {
  if (!calibrationPanDrag || e.pointerId !== calibrationPanDrag.pointerId) return;
  e.preventDefault();
  e.stopPropagation();
  e.stopImmediatePropagation?.();
  const dx = e.clientX - calibrationPanDrag.startX;
  const dy = e.clientY - calibrationPanDrag.startY;
  setCalibrationImagePan(
    calibrationPanDrag.startPan.x + dx,
    calibrationPanDrag.startPan.y + dy,
    false,
  );
}

function stopCalibrationZoomDrag(e) {
  if (!calibrationPanDrag || e.pointerId !== calibrationPanDrag.pointerId) return;
  e.preventDefault();
  e.stopPropagation();
  e.stopImmediatePropagation?.();
  calibrationPanDrag = null;
  controls.enabled = true;
  vp.classList.remove('calibration-panning');
  try { vp.releasePointerCapture(e.pointerId); } catch (_) {}
  updateCalibrationPanelFromCamera();
}

function wireCalibrationControls() {
  calibBtn.addEventListener('click', () => setCalibrationPanelOpen(calibPanel.hidden));
  document.getElementById('calib-load-image').addEventListener('click', () => loadCalibrationImage(calibImagePath.value));
  document.getElementById('calib-hide-image').addEventListener('click', hideCalibrationImage);
  document.getElementById('calib-fit-model').addEventListener('click', () => {
    fitCamera();
    updateCalibrationPanelFromCamera();
  });
  document.getElementById('calib-copy-camera').addEventListener('click', copyCalibrationCameraJson);
  document.getElementById('calib-save-local').addEventListener('click', saveCalibrationToLocalStorage);
  document.getElementById('calib-restore-local').addEventListener('click', restoreCalibrationFromLocalStorage);
  document.getElementById('calib-load-camera-json').addEventListener('click', restoreCalibrationFromPath);
  document.getElementById('calib-save-camera-json').addEventListener('click', saveCalibrationToPath);
  calibCameraJsonPath.addEventListener('keydown', e => {
    if (e.key === 'Enter') restoreCalibrationFromPath();
  });
  calibImportCameraJson.addEventListener('change', e => {
    restoreCalibrationFromFile(e.target.files?.[0]);
  });
  calibImageOpacity.addEventListener('input', () => {
    calibrationBg.style.opacity = calibImageOpacity.value;
  });
  calibImageZoom.addEventListener('input', () => {
    setCalibrationImageZoom(calibImageZoom.value);
  });
  calibImagePanMode.addEventListener('click', () => setCalibrationPanMode(!calibrationPanMode));
  calibImagePanReset.addEventListener('click', () => setCalibrationImagePan(0, 0));
  vp.addEventListener('pointerdown', startCalibrationZoomDrag, true);
  vp.addEventListener('pointermove', moveCalibrationZoomDrag, true);
  vp.addEventListener('pointerup', stopCalibrationZoomDrag, true);
  vp.addEventListener('pointercancel', stopCalibrationZoomDrag, true);
  calibModelOpacity.addEventListener('input', () => {
    calibrationModelOpacity = clamp01(Number(calibModelOpacity.value));
    calibModelOpacityValue.textContent = Math.round(calibrationModelOpacity * 100) + ' %';
    applyCalibrationModelOpacity();
  });
  calibImagePath.addEventListener('keydown', e => {
    if (e.key === 'Enter') loadCalibrationImage(calibImagePath.value);
  });
  calibImagePath.addEventListener('input', () => updateCameraJsonPathDefault(true));
  Object.values(calibFields).forEach(field => {
    field.addEventListener('change', applyCalibrationInputsToCamera);
  });
  controls.addEventListener('change', () => {
    applyCameraRoll();
    updateCalibrationPanelFromCamera();
  });
  updateCalibrationPanelFromCamera();
}

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
  wireCalibrationControls();
  saveBtn.addEventListener('click', saveGeometry);
  const params = new URLSearchParams(window.location.search);
  const photoParam = params.get('photo');
  if (photoParam) {
    calibImagePath.value = photoParam;
    setCalibrationPanelOpen(true);
  }
  const cameraParam = params.get('camera');
  if (cameraParam) {
    calibCameraJsonPath.value = cameraParam;
    setCalibrationPanelOpen(true);
    await restoreCalibrationFromPath();
  }
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
