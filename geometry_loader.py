"""
geometry_loader.py – pieni apumoduuli geometria-JSON:ien lataukseen.

Python-laskelmat (kuormituslaskenta.py ja terassilasitus_rakenne_vaihtoehdot.py)
lukevat primitiiviset geometriavakiot (leveydet, korkeudet, jaot,
profiilimitat) suoraan geometry/*.json -tiedostoista. Johdettu laskenta
(kaltevuudet, tributary-alueet, kuormat) pysyy Python-puolella.

Skeema: geometry/schema.json (JSON Schema 2020-12).
"""

import json
import math
import os
import copy

_GEO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "geometry")

_BASE_VEC = {
    "+x": (1, 0, 0), "-x": (-1, 0, 0),
    "+y": (0, 1, 0), "-y": (0, -1, 0),
    "+z": (0, 0, 1), "-z": (0, 0, -1),
}


def load(name):
    """Lataa geometria-JSON ja resolvoi profiiliviittaukset + pisteref.t + pintojen polygonit."""
    path = os.path.join(_GEO_DIR, name)
    with open(path, "r", encoding="utf-8") as f:
        geo = json.load(f)
    _resolve_profiles(geo)
    _resolve_named_points(geo)
    _resolve_member_refs(geo)
    _resolve_surface_refs(geo)
    _resolve_surface_member_patterns(geo)
    _resolve_polygons(geo)
    return geo


def _resolve_profiles(geo):
    """Korvaa member['profile']-merkkijonoviittaukset top-level 'profiles'-listan objekteilla."""
    by_id = {p["id"]: p for p in geo.get("profiles", [])}
    for group, lst in geo.get("members", {}).items():
        for m in lst:
            ref = m.get("profile")
            if isinstance(ref, str):
                if ref not in by_id:
                    raise KeyError(f"tuntematon profiiliviittaus '{ref}' kohteessa {group}/{m.get('id')}")
                m["profile"] = by_id[ref]


def _resolve_named_points(geo):
    """Resolvoi top-level 'points' -listan: korvaa ref-muotoiset absoluuttisilla (x,y,z).

    Piste voi olla:
      - {"id": ..., "x": ..., "y": ..., "z": ...}           absoluuttinen
      - {"id": ..., "ref": "...", "dx": ?, "dy": ?, "dz": ?} offset toisesta pisteestä

    Topologinen resolvointi, cycle detection. Tallentaa abs-koordinaatit takaisin
    point-objektiin (x/y/z) ja tekee lookup-taulun geo['_points_by_id'] (= id → dict).
    """
    raw = list(geo.get("points", []))
    by_id = {p["id"]: p for p in raw}
    resolved = {}
    visiting = set()

    def resolve(pid):
        if pid in resolved:
            return resolved[pid]
        if pid in visiting:
            raise ValueError(f"sykli nimettyjen pisteiden viittauksissa: {pid}")
        if pid not in by_id:
            raise KeyError(f"tuntematon point-id: '{pid}'")
        visiting.add(pid)
        p = by_id[pid]
        if "ref" in p:
            bx, by_, bz = resolve(p["ref"])
            x = bx + p.get("dx", 0)
            y = by_ + p.get("dy", 0)
            z = bz + p.get("dz", 0)
        else:
            x, y, z = p["x"], p["y"], p["z"]
        visiting.discard(pid)
        resolved[pid] = (x, y, z)
        return resolved[pid]

    for pid in list(by_id.keys()):
        resolve(pid)

    for p in raw:
        x, y, z = resolved[p["id"]]
        for k in ("ref", "dx", "dy", "dz"):
            p.pop(k, None)
        p["x"], p["y"], p["z"] = x, y, z

    geo["_points_by_id"] = {p["id"]: p for p in raw}


def _apply_ref_offset(pt, ref_xyz):
    """Tulkitsee pisteen x/y/z offsetteina ref_xyz:stä ja korvaa abs-koordinaateilla."""
    rx, ry, rz = ref_xyz
    pt["x"] = rx + pt.get("x", 0)
    pt["y"] = ry + pt.get("y", 0)
    pt["z"] = rz + pt.get("z", 0)


def _resolve_member_refs(geo):
    """Jos jäsenellä on 'ref': point-id, tulkitsee sen koordinaatit offseteina
    ja korvaa absoluuttisilla. Poistaa 'ref'-kentän (absoluuttinen jää jäljelle)."""
    points = geo.get("_points_by_id", {})
    for group, lst in geo.get("members", {}).items():
        for m in lst:
            rid = m.pop("ref", None)
            if rid is None:
                continue
            if rid not in points:
                raise KeyError(f"jäsen {group}/{m.get('id')} viittaa tuntemattomaan pisteeseen '{rid}'")
            p = points[rid]
            ref_xyz = (p["x"], p["y"], p["z"])
            if group == "columns":
                for k in ("base", "top"):
                    if k in m:
                        _apply_ref_offset(m[k], ref_xyz)
            else:
                for k in ("axis_start", "axis_end"):
                    if k in m:
                        _apply_ref_offset(m[k], ref_xyz)
                pat = m.get("pattern")
                if pat and "offset" in pat:
                    # pattern.offset on aina delta (sama tulkinta), ei muuta tarvita
                    pass


def _resolve_surface_refs(geo):
    """Jos pinnalla (surface / reference_surface) on 'ref': point-id, tulkitsee
    placement.anchor-koordinaatit offseteina ja korvaa absoluuttisilla."""
    points = geo.get("_points_by_id", {})
    for group_key in ("reference_surfaces", "surfaces"):
        for s in geo.get(group_key, []):
            rid = s.pop("ref", None)
            if rid is None:
                continue
            if rid not in points:
                raise KeyError(f"pinta {group_key}/{s.get('id')} viittaa tuntemattomaan pisteeseen '{rid}'")
            p = points[rid]
            anchor = s.get("placement", {}).get("anchor")
            if anchor is not None:
                _apply_ref_offset(anchor, (p["x"], p["y"], p["z"]))


def _pattern_ref_alias(id_template):
    """Palauttaa lyhyen alias-juuren id_template-muodosta kuten 'kattotuoli.{i}'."""
    if "{i}" not in id_template:
        return None
    before, after = id_template.split("{i}", 1)
    if after:
        return None
    alias = before.rstrip(".-_")
    return alias or None


def _pattern_member_specs(geo):
    specs = {}
    member_ids = {
        member_obj["id"]
        for group in ("beams", "rafters", "purlins")
        for member_obj in geo.get("members", {}).get(group, [])
    }
    alias_candidates = {}
    for group in ("beams", "rafters", "purlins"):
        for member_obj in geo.get("members", {}).get(group, []):
            pattern = member_obj.get("pattern")
            if not pattern:
                continue
            id_template = pattern.get("id_template", member_obj["id"] + ".{i}")
            spec = {
                "count": int(pattern["count"]),
                "offset": dict(pattern["offset"]),
                "id_template": id_template,
                "first_instance_id": id_template.replace("{i}", "0"),
            }
            specs[member_obj["id"]] = spec

            alias = _pattern_ref_alias(id_template)
            if alias is not None and alias not in member_ids:
                alias_candidates.setdefault(alias, []).append(spec)

    for alias, candidates in alias_candidates.items():
        if len(candidates) == 1:
            specs[alias] = candidates[0]
    return specs


def _expand_member_ref_list(member_refs, pattern_specs):
    expanded = []
    for member_id in member_refs:
        spec = pattern_specs.get(member_id)
        # Avoid ambiguous roots like "kattotuoli.0", which may intentionally
        # refer only to the first instance rather than the whole pattern.
        if spec is not None and member_id != spec["first_instance_id"]:
            expanded.extend(spec["id_template"].replace("{i}", str(i)) for i in range(spec["count"]))
        else:
            expanded.append(member_id)

    deduped = []
    seen = set()
    for member_id in expanded:
        if member_id in seen:
            continue
        deduped.append(member_id)
        seen.add(member_id)
    return deduped


def _resolve_surface_member_patterns(geo):
    """Laajentaa pintojen jäsenviittauksissa pattern-juuret yksittäisiksi ID:iksi."""
    pattern_specs = _pattern_member_specs(geo)
    if not pattern_specs:
        return

    for surface_obj in geo.get("surfaces", []):
        if isinstance(surface_obj.get("supported_by"), list):
            surface_obj["supported_by"] = _expand_member_ref_list(surface_obj["supported_by"], pattern_specs)

        load_transfer = surface_obj.get("load_transfer")
        if not isinstance(load_transfer, dict):
            continue
        for rule in load_transfer.get("to_members", []):
            if isinstance(rule.get("member_refs"), list):
                rule["member_refs"] = _expand_member_ref_list(rule["member_refs"], pattern_specs)


def _axis_vec(axis):
    """Palauttaa (dx, dy, dz)-siirtymävektorin akselin kuvauksesta.

    Kolme syötemuotoa:
      1) {"dir": "+x", "length_mm": 7938}                        – akselin suuntainen
      2) {"dir": "+x", "horizontal_mm": 7200, "rise_mm": 1500}   – vaaka + pystynousu
      3) {"dir": "+y", "slope_deg": 7.2, "slope_sense": "-z",
          "slope_length_mm": 3980}                               – slope-parametrinen
    """
    base = _BASE_VEC[axis["dir"]]
    if "slope_deg" in axis:
        theta = math.radians(axis["slope_deg"])
        L = axis["slope_length_mm"]
        horiz = L * math.cos(theta)
        vert = L * math.sin(theta)
        sense = _BASE_VEC[axis["slope_sense"]]
        return (base[0] * horiz + sense[0] * vert,
                base[1] * horiz + sense[1] * vert,
                base[2] * horiz + sense[2] * vert)
    if "horizontal_mm" in axis:
        H = axis["horizontal_mm"]
        R = axis.get("rise_mm", 0.0)
        return (base[0] * H, base[1] * H, base[2] * H + R)
    L = axis["length_mm"]
    return (base[0] * L, base[1] * L, base[2] * L)


def _axis_length_mm(axis):
    """Akselin kokonaispituus (skaalan normalisointiin polygon-pisteille)."""
    dx, dy, dz = _axis_vec(axis)
    return math.hypot(math.hypot(dx, dy), dz)


def _compute_polygon(placement, local_shape):
    anchor = placement["anchor"]
    ax, ay, az = anchor["x"], anchor["y"], anchor["z"]
    u_vec = _axis_vec(placement["u"])
    v_vec = _axis_vec(placement["v"])
    u_len = _axis_length_mm(placement["u"])
    v_len = _axis_length_mm(placement["v"])

    shape_type = local_shape["type"]
    if shape_type == "rectangle":
        corners_frac = [(0, 0), (1, 0), (1, 1), (0, 1)]
    elif shape_type == "polygon":
        verts = local_shape["vertices_uv"]
        corners_frac = [(u / u_len if u_len else 0.0, v / v_len if v_len else 0.0) for u, v in verts]
    else:
        raise ValueError(f"tuntematon local_shape.type: {shape_type!r}")

    polygon = []
    for fu, fv in corners_frac:
        x = ax + fu * u_vec[0] + fv * v_vec[0]
        y = ay + fu * u_vec[1] + fv * v_vec[1]
        z = az + fu * u_vec[2] + fv * v_vec[2]
        polygon.append({"x": int(round(x)), "y": int(round(y)), "z": int(round(z))})
    return polygon


def _resolve_polygons(geo):
    """Laskee jokaiselle pinnalle 3D-polygonin placement+local_shape -kuvauksesta
    ja asettaa sen 'polygon'-kenttään. Python-laskelmat lukevat 'polygon'-kenttää."""
    for ref in geo.get("reference_surfaces", []):
        if "placement" in ref:
            ref["polygon"] = _compute_polygon(ref["placement"], ref["local_shape"])
    for s in geo.get("surfaces", []):
        if "placement" in s:
            s["polygon"] = _compute_polygon(s["placement"], s["local_shape"])


def member(geo, group, mid):
    """Etsi kantava rakenneosa ID:llä ryhmästä (columns/beams/rafters/purlins)."""
    for m in geo["members"].get(group, []):
        if m["id"] == mid:
            return m
    raise KeyError(f"{group}/{mid}")


def surface(geo, sid):
    """Etsi pinta ID:llä."""
    for s in geo["surfaces"]:
        if s["id"] == sid:
            return s
    raise KeyError(sid)


def reference(geo, rid):
    """Etsi viitepinta ID:llä."""
    for r in geo.get("reference_surfaces", []):
        if r["id"] == rid:
            return r
    raise KeyError(rid)


def expanded_members(geo, group):
    """Palauttaa ryhmän jäsenet pattern-laajennettuna ilman geo-olion muokkausta."""
    expanded = []
    for member_obj in geo.get("members", {}).get(group, []):
        pattern = member_obj.get("pattern")
        if not pattern:
            expanded.append(member_obj)
            continue

        offset = pattern["offset"]
        id_template = pattern.get("id_template", member_obj["id"] + ".{i}")
        for i in range(int(pattern["count"])):
            clone = copy.deepcopy(member_obj)
            clone.pop("pattern", None)
            clone["id"] = id_template.replace("{i}", str(i))
            for key in ("axis_start", "axis_end"):
                if key in clone:
                    clone[key]["x"] += offset["x"] * i
                    clone[key]["y"] += offset["y"] * i
                    clone[key]["z"] += offset["z"] * i
            expanded.append(clone)
    return expanded


def _connection_pattern_specs(geo):
    return _pattern_member_specs(geo)


def _should_expand_connection_member(connection_obj, pattern_member, spec):
    # If the pattern root is also the first expanded ID (e.g. kattotuoli.0),
    # an explicit connection may intentionally target only that one instance.
    if pattern_member == spec["first_instance_id"] and pattern_member in connection_obj.get("id", ""):
        return False
    return True


def expanded_connections(geo):
    """Palauttaa connections-listan pattern-jäsenet yksittäisiksi instansseiksi laajennettuna."""
    specs = _connection_pattern_specs(geo)
    expanded = []
    for connection_obj in geo.get("connections", []):
        pattern_member = next(
            (
                member_id
                for member_id in connection_obj.get("members", [])
                if member_id in specs and _should_expand_connection_member(connection_obj, member_id, specs[member_id])
            ),
            None,
        )
        if pattern_member is None:
            expanded.append(copy.deepcopy(connection_obj))
            continue

        spec = specs[pattern_member]
        offset = spec["offset"]
        id_template = spec["id_template"]
        for i in range(spec["count"]):
            clone = copy.deepcopy(connection_obj)
            clone["id"] = f"{connection_obj['id']}.{i}"
            clone["members"] = [
                id_template.replace("{i}", str(i)) if member_id == pattern_member else member_id
                for member_id in connection_obj["members"]
            ]
            if "at" in clone:
                clone["at"]["x"] += offset["x"] * i
                clone["at"]["y"] += offset["y"] * i
                clone["at"]["z"] += offset["z"] * i
            expanded.append(clone)
    return expanded


def profile_b(m):
    """Profiilin kokonaisleveys ottaen huomioon rinnakkaisten palkkien lukumäärän."""
    p = m["profile"]
    return float(p["b_mm"]) * int(p.get("count", 1))


def profile_h(m):
    return float(m["profile"]["h_mm"])
