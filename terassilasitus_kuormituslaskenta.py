"""
LASITETUN TERASSIN LOPULLINEN PUURATKAISU – KUORMITUSLASKENTA – ETELÄSUOMI
============================================================================
Standardit: EN 1990, EN 1991-1-1, EN 1991-1-3, EN 1991-1-4, EN 1995-1-1

Geometria luetaan tiedostosta geometry/terassi_puu.json:
  - uusi puuratkaisu: orret 98×48, nurkkaorret 98×48, kattotuolit 198×48,
    ulkopalkki LP225×140, sisäpalkki LP315×140
  - lineaarijäsenten poikkileikkauksen kierto luetaan section_rotation_deg-kentästä
  - aurinkopaneelien reunakaistat siirtyvät reunakattotuoleille vain orsien kautta
  - ulkokulmien nurkkaorret sidotaan uloimpaan orteen ja kantavat ulkopalkin ulkoreunalla
  - liitosten tuki- ja rotaatiomallit luetaan connections.analysis-metadatasta
  - paneelikuorman piste/viivamalli luetaan surfaces[*].load_transfer-metadatasta
  - kinostuma talon seinää vasten johdetaan geometriasta muuttuvalla h(x)-korkeudella
  - paneelien kinostumakestävyys tarkistetaan 5.40 kN/m² etupuolen rajaa vasten
  - lovi- ja nettoh-tarkistukset luetaan geometry/terassi_puu.json:n cuts-kentistä
"""

import math

from beam_analysis import (
    combine_uniform_loads,
    intervals_to_uniform_loads,
    load_stats,
    refine_nodes_mm,
    solve_member_response,
    total_uniform_load_kN,
    uniform_loads_for_nodes,
)
from foundation_checks import foundation_checks_from_envelope, foundation_report_lines
from geometry_loader import expanded_connections, expanded_members, load, member, surface, reference, profile_b, profile_h
from portaikko_loads import existing_column_extra_loads_by_case as portaikko_existing_column_extra_loads_by_case
from structural_geometry import member_axis_length_mm, project_point_to_member_s_mm
from terrace_column_loads import (
    GAMMA_CONCRETE_KNM3,
    COLUMN_CASE_FACTORS,
    calculate_katos_total_column_loads,
    column_self_weight_kN,
    envelope_column_totals,
)
from timber_member_checks import (
    combined_section_h,
    governing_moment,
    member_rect_props,
    member_section_rotation_deg,
    sample_net_section_utilization,
)


# ── yleisapuja ───────────────────────────────────────────────────────────────

def tributary_ranges_mm(positions_mm, edge_start_mm, edge_end_mm):
    ranges = []
    for i, pos_mm in enumerate(positions_mm):
        left_mm = edge_start_mm if i == 0 else 0.5 * (positions_mm[i - 1] + pos_mm)
        right_mm = edge_end_mm if i == len(positions_mm) - 1 else 0.5 * (pos_mm + positions_mm[i + 1])
        ranges.append((left_mm, right_mm))
    return ranges


def tributary_widths_m(positions_mm, edge_start_mm, edge_end_mm):
    return [(b_mm - a_mm) / 1000.0 for a_mm, b_mm in tributary_ranges_mm(positions_mm, edge_start_mm, edge_end_mm)]


def local_interval_to_global(support_coord_mm, offset_mm, length_mm, local_positive_sign):
    a_mm = support_coord_mm + local_positive_sign * offset_mm
    b_mm = support_coord_mm + local_positive_sign * (offset_mm + length_mm)
    return (min(a_mm, b_mm), max(a_mm, b_mm))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def format_ok(ok):
    return "OK ✓" if ok else "YLITTYY ✗"


def format_birdsmouth_h3_status(ok):
    if ok:
        return "OK ohjesaannolla"
    return "ylittaa ohjesaannon -> vaatii erillisen detaljimitoituksen"


SIDE_MEMBER_LABEL = {"left": "vasen", "right": "oikea"}
SIDE_ORDER = {"left": 0, "right": 1}
INTERIOR_RAFTER_PREFIX = "kattotuoli."


def side_purlin_root_id(side):
    return f"orsi.{SIDE_MEMBER_LABEL[side]}"


def side_purlin_prefix(side):
    return side_purlin_root_id(side) + "."


def edge_rafter_id(side):
    return f"kattotuoli.{SIDE_MEMBER_LABEL[side]}"


def member_side_from_id(member_id):
    parts = member_id.split(".")
    if len(parts) < 2:
        return None
    if parts[1] == SIDE_MEMBER_LABEL["left"]:
        return "left"
    if parts[1] == SIDE_MEMBER_LABEL["right"]:
        return "right"
    return None


# ── geometria ───────────────────────────────────────────────────────────────

GEO = load("terassi_puu.json")
FOUNDATION_GEO = load("katos.json")
CONNECTIONS = {conn["id"]: conn for conn in GEO["connections"]}
CONNECTION_INSTANCES = expanded_connections(GEO)
MEMBERS_BY_ID = {}
for group_name in GEO["members"]:
    for member_obj in expanded_members(GEO, group_name):
        MEMBERS_BY_ID[member_obj["id"]] = member_obj


def connections_by_members(member_a_id, member_b_id, connection_objs=None):
    if connection_objs is None:
        connection_objs = GEO["connections"]
    wanted = {member_a_id, member_b_id}
    return [
        conn
        for conn in connection_objs
        if set(conn.get("members", [])) == wanted
    ]


def connection_by_members(member_a_id, member_b_id, predicate=None, description="connection", connection_objs=None):
    matches = connections_by_members(member_a_id, member_b_id, connection_objs=connection_objs)
    if predicate is not None:
        matches = [conn for conn in matches if predicate(conn)]
    if not matches:
        raise KeyError(f"{description} not found for members: {member_a_id}, {member_b_id}")
    if len(matches) == 1:
        return matches[0]
    analysis_matches = [conn for conn in matches if conn.get("analysis")]
    if len(analysis_matches) == 1:
        return analysis_matches[0]
    return matches[0]


def optional_connection_by_members(member_a_id, member_b_id, predicate=None, connection_objs=None):
    try:
        return connection_by_members(member_a_id, member_b_id, predicate=predicate, connection_objs=connection_objs)
    except KeyError:
        return None


def support_connection(member_id, support_member_id, support_line_ref, connection_objs=None):
    return connection_by_members(
        member_id,
        support_member_id,
        predicate=lambda conn: conn.get("analysis", {}).get("support_line_ref") == support_line_ref,
        description=f"{support_line_ref} support connection",
        connection_objs=connection_objs,
    )


def member_by_id_any(member_id):
    return MEMBERS_BY_ID[member_id]


def first_connection_matching(member_id, predicate):
    for conn in GEO["connections"]:
        members = conn.get("members", [])
        if member_id in members and predicate(conn, members):
            return conn
    return None


def connection_cut(connection, kind=None):
    if connection is None:
        return None
    if isinstance(connection, str):
        connection_obj = CONNECTIONS.get(connection)
        if connection_obj is None:
            return None
    else:
        connection_obj = connection
    cuts = connection_obj.get("cuts", [])
    if kind is None:
        return cuts[0] if cuts else None
    for cut in cuts:
        if cut.get("kind") == kind:
            return cut
    return None


def rect_notch_info(connection_id):
    cut = connection_cut(connection_id, "rect_notch")
    if cut is None:
        return {"active": False, "depth_mm": 0.0, "length_mm": 0.0, "offset_mm": 0.0}
    return {
        "active": True,
        "depth_mm": float(cut["depth_mm"]),
        "length_mm": float(cut["length_mm"]),
        "offset_mm": float(cut.get("offset_mm", 0.0)),
    }


def bevel_notch_info(connection_id):
    cut = connection_cut(connection_id, "bevel_notch")
    if cut is None:
        return {
            "active": False,
            "depth_mm": 0.0,
            "length_mm": 0.0,
            "offset_mm": 0.0,
            "reference": None,
            "side": None,
        }
    return {
        "active": True,
        "depth_mm": float(cut["depth_mm"]),
        "length_mm": float(cut["length_mm"]),
        "offset_mm": float(cut.get("offset_mm", 0.0)),
        "reference": cut["reference"],
        "side": cut["side"],
    }

roof = surface(GEO, "surf.solar_panels")
roof_poly = roof["polygon"]
roof_x0_mm = min(p["x"] for p in roof_poly)
roof_x1_mm = max(p["x"] for p in roof_poly)
roof_y0_mm = min(p["y"] for p in roof_poly)
roof_y1_mm = max(p["y"] for p in roof_poly)
roof_width_mm = roof_x1_mm - roof_x0_mm
roof_depth_mm = roof_y1_mm - roof_y0_mm
roof_area_m2 = roof_width_mm * roof_depth_mm / 1.0e6
roof_depth_m = roof_depth_mm / 1000.0
roof_slope_deg = math.degrees(math.atan2(
    max(p["z"] for p in roof_poly if p["y"] == roof_y0_mm) - max(p["z"] for p in roof_poly if p["y"] == roof_y1_mm),
    roof_depth_mm,
))
roof_slope_rad = math.radians(roof_slope_deg)
roof_slope_length_mm = roof_depth_mm / max(math.cos(roof_slope_rad), 1.0e-9)
roof_inner_pts = sorted([p for p in roof_poly if p["y"] == roof_y0_mm], key=lambda p: p["x"])
roof_inner_x0_mm = roof_inner_pts[0]["x"]
roof_inner_x1_mm = roof_inner_pts[-1]["x"]
roof_inner_z0_mm = roof_inner_pts[0]["z"]
roof_inner_z1_mm = roof_inner_pts[-1]["z"]


def optional_surface(sid):
    try:
        return surface(GEO, sid)
    except KeyError:
        return None


def surface_bounds(surface_obj):
    poly = surface_obj["polygon"]
    return {
        "x0_mm": min(p["x"] for p in poly),
        "x1_mm": max(p["x"] for p in poly),
        "y0_mm": min(p["y"] for p in poly),
        "y1_mm": max(p["y"] for p in poly),
        "z0_mm": min(p["z"] for p in poly),
        "z1_mm": max(p["z"] for p in poly),
    }


def rectangle_projected_area_m2(bounds):
    return (bounds["x1_mm"] - bounds["x0_mm"]) * (bounds["y1_mm"] - bounds["y0_mm"]) / 1.0e6


def surface_polygon_area_xz_m2(surface_obj):
    poly = surface_obj["polygon"]
    area2_mm2 = 0.0
    for p0, p1 in zip(poly, poly[1:] + poly[:1]):
        area2_mm2 += p0["x"] * p1["z"] - p1["x"] * p0["z"]
    return abs(area2_mm2) / 2.0e6


def surface_slope_span_y_mm(surface_obj):
    poly = surface_obj["polygon"]
    y0 = min(p["y"] for p in poly)
    y1 = max(p["y"] for p in poly)
    inner = sorted([p for p in poly if p["y"] == y0], key=lambda p: p["x"])[0]
    outer = sorted([p for p in poly if p["y"] == y1], key=lambda p: p["x"])[0]
    return math.hypot(outer["y"] - inner["y"], outer["z"] - inner["z"])


def surface_slope_span_to_y_mm(surface_obj, y_mm):
    poly = surface_obj["polygon"]
    y0 = min(p["y"] for p in poly)
    y1 = max(p["y"] for p in poly)
    if abs(y1 - y0) <= 1e-9:
        return 0.0
    ratio = clamp((y_mm - y0) / (y1 - y0), 0.0, 1.0)
    return ratio * surface_slope_span_y_mm(surface_obj)


infill_glass = optional_surface("surf.wall_infill_glass")
infill_glass_summary = None
if infill_glass is not None:
    infill_bounds = surface_bounds(infill_glass)
    infill_width_mm = infill_bounds["x1_mm"] - infill_bounds["x0_mm"]
    infill_depth_mm = infill_bounds["y1_mm"] - infill_bounds["y0_mm"]
    infill_slope_span_mm = surface_slope_span_y_mm(infill_glass)
    infill_area_m2 = rectangle_projected_area_m2(infill_bounds)
    infill_slope_area_m2 = infill_width_mm * infill_slope_span_mm / 1.0e6
else:
    infill_bounds = None
    infill_width_mm = 0.0
    infill_depth_mm = 0.0
    infill_slope_span_mm = 0.0
    infill_area_m2 = 0.0
    infill_slope_area_m2 = 0.0

gable_glazing = optional_surface("surf.triangle_glazing.gable")
gable_glazing_summary = None
if gable_glazing is not None:
    gable_bounds = surface_bounds(gable_glazing)
    gable_poly = gable_glazing["polygon"]
    gable_area_m2 = surface_polygon_area_xz_m2(gable_glazing)
    bottom_z_mm = min(p["z"] for p in gable_poly)
    bottom_pts = sorted([p for p in gable_poly if abs(p["z"] - bottom_z_mm) <= 1e-6], key=lambda p: p["x"])
    gable_bottom_x0_mm = float(bottom_pts[0]["x"])
    gable_bottom_x1_mm = float(bottom_pts[-1]["x"])
    gable_base_length_m = (gable_bottom_x1_mm - gable_bottom_x0_mm) / 1000.0
else:
    gable_bounds = None
    gable_area_m2 = 0.0
    gable_bottom_x0_mm = 0.0
    gable_bottom_x1_mm = 0.0
    gable_base_length_m = 0.0

panel_joint_y_mm = 0.5 * (roof_y0_mm + roof_y1_mm)
panel_frame_edge_offset_mm = 15.0
DEFAULT_ROOF_LOAD_TRANSFER_RULE = {
    "model": "point",
    "reference": "axis_end",
    "offset_mm": -panel_frame_edge_offset_mm,
}


def roof_load_transfer_rule(member_id):
    load_transfer = roof.get("load_transfer", {})
    rules = load_transfer.get("to_members", [])
    for rule in rules:
        if member_id in rule.get("member_refs", []):
            merged = dict(DEFAULT_ROOF_LOAD_TRANSFER_RULE)
            merged.update(rule)
            return merged
    if rules:
        raise KeyError(f"surf.solar_panels.load_transfer missing rule for {member_id}")
    return dict(DEFAULT_ROOF_LOAD_TRANSFER_RULE)


def load_transfer_tributary_width_mm(member_id, fallback_width_mm):
    rule = roof_load_transfer_rule(member_id)
    return float(rule.get("tributary_width_mm", fallback_width_mm))


LOAD_TRANSFER_REFERENCE_LABELS = {
    "axis_start": "axis_start-päästä",
    "axis_end": "axis_end-päästä",
    "support_centerline": "tukikeskilinjasta",
    "support_inner_edge": "tuen sisäreunasta",
    "support_outer_edge": "tuen ulkoreunasta",
}


def strip_member_refs_from_rule(rule):
    return {key: value for key, value in rule.items() if key != "member_refs"}


def format_load_transfer_rule(member_ids):
    rules = [roof_load_transfer_rule(member_id) for member_id in member_ids]
    if not rules:
        return "ei kuormansiirtosääntöä"
    base_rule = strip_member_refs_from_rule(rules[0])
    if any(strip_member_refs_from_rule(rule) != base_rule for rule in rules[1:]):
        return "jäsenkohtaiset member_refs-säännöt; omapaino viivakuormana"
    rule = rules[0]
    if rule["model"] == "uniform":
        return "kuorma viivakuormana koko matkalle; omapaino viivakuormana"
    ref_label = LOAD_TRANSFER_REFERENCE_LABELS[rule["reference"]]
    offset_mm = float(rule.get("offset_mm", 0.0))
    if rule["model"] == "point":
        width_text = f", b_trib {float(rule['tributary_width_mm']):.0f} mm" if "tributary_width_mm" in rule else ""
        return f"kuorma pisteenä {offset_mm:+.0f} mm {ref_label}{width_text}; omapaino viivakuormana"
    if rule["model"] == "partial_uniform":
        return (
            f"kuorma osaviivakuormana {offset_mm:+.0f} mm {ref_label}, "
            f"pituus {float(rule['length_mm']):.0f} mm; omapaino viivakuormana"
        )
    return f"kuormamalli {rule['model']}; omapaino viivakuormana"


def unique_bevel_notch_specs(notch_infos):
    seen = set()
    for info in notch_infos:
        if not info or not info.get("active"):
            continue
        spec = (info.get("side"), float(info["depth_mm"]), float(info["length_mm"]))
        if spec in seen:
            continue
        seen.add(spec)
    return sorted(seen)


def format_bevel_notch_specs(notch_infos):
    specs = unique_bevel_notch_specs(notch_infos)
    if not specs:
        return "ei bevel-lovea"
    sides = {side for side, _, _ in specs}
    if len(sides) == 1:
        side = specs[0][0]
        return f"{side} " + " / ".join(f"{depth_mm:.0f} × {length_mm:.0f} mm" for _, depth_mm, length_mm in specs)
    return " / ".join(f"{side} {depth_mm:.0f} × {length_mm:.0f} mm" for side, depth_mm, length_mm in specs)


def format_labeled_bevel_notch_specs(notch_info_by_label):
    label_text = {"left": "vasen", "right": "oikea"}
    active_items = [
        (label_text.get(label, label), info)
        for label, info in notch_info_by_label.items()
        if info and info.get("active")
    ]
    if not active_items:
        return "ei bevel-lovea"
    sides = {info.get("side") for _, info in active_items}
    if len(sides) == 1:
        return f"bevel_notch {next(iter(sides))} " + " | ".join(
            f"{label}: {float(info['depth_mm']):.0f} × {float(info['length_mm']):.0f} mm"
            for label, info in active_items
        )
    return "bevel_notch " + " | ".join(
        f"{label}/{info['side']}: {float(info['depth_mm']):.0f} × {float(info['length_mm']):.0f} mm"
        for label, info in active_items
    )


def bevel_notch_label(info):
    side = info.get("side")
    return f"{info['reference']}_bevel_{side}" if side else f"{info['reference']}_bevel"

house_roof = reference(GEO, "ref.house.roof")
house_roof_poly = house_roof["polygon"]
house_roof_y0 = min(p["y"] for p in house_roof_poly)
roof_eave_pts = sorted([p for p in house_roof_poly if p["y"] == house_roof_y0], key=lambda p: p["x"])
house_roof_x0_mm = roof_eave_pts[0]["x"]
house_roof_x1_mm = roof_eave_pts[-1]["x"]
house_roof_z0_mm = roof_eave_pts[0]["z"]
house_roof_z1_mm = roof_eave_pts[-1]["z"]


def house_roof_z_at_x(x_mm):
    if abs(house_roof_x1_mm - house_roof_x0_mm) < 1e-9:
        return house_roof_z0_mm
    t = (x_mm - house_roof_x0_mm) / (house_roof_x1_mm - house_roof_x0_mm)
    return house_roof_z0_mm + t * (house_roof_z1_mm - house_roof_z0_mm)


def roof_inner_z_at_x(x_mm):
    if abs(roof_inner_x1_mm - roof_inner_x0_mm) < 1e-9:
        return roof_inner_z0_mm
    t = (x_mm - roof_inner_x0_mm) / (roof_inner_x1_mm - roof_inner_x0_mm)
    return roof_inner_z0_mm + t * (roof_inner_z1_mm - roof_inner_z0_mm)


def local_wall_height_m_at_x(x_mm):
    return max(0.0, (house_roof_z_at_x(x_mm) - roof_inner_z_at_x(x_mm)) / 1000.0)


def member_axis_delta(member_obj):
    return (
        float(member_obj["axis_end"]["x"]) - float(member_obj["axis_start"]["x"]),
        float(member_obj["axis_end"]["y"]) - float(member_obj["axis_start"]["y"]),
    )


def member_axis_vector_3d(member_obj):
    return (
        float(member_obj["axis_end"]["x"]) - float(member_obj["axis_start"]["x"]),
        float(member_obj["axis_end"]["y"]) - float(member_obj["axis_start"]["y"]),
        float(member_obj["axis_end"]["z"]) - float(member_obj["axis_start"]["z"]),
    )


def connection_other_member_id(connection_obj, member_id):
    return next((other for other in connection_obj.get("members", []) if other != member_id), None)


def connection_support_point(connection_obj, member_id, support_line_ref=None):
    point_xyz = dict(connection_obj["at"])
    support_member_id = connection_other_member_id(connection_obj, member_id)
    if support_member_id is None:
        return point_xyz

    if support_line_ref is None:
        support_line_ref = connection_obj.get("analysis", {}).get("support_line_ref", "support_centerline")
    if support_line_ref == "support_centerline":
        return point_xyz

    support_member_obj = member_by_id_any(support_member_id)
    support_half_b_mm = profile_b(support_member_obj) / 2.0

    if support_member_id in {edge_rafter_id("left"), edge_rafter_id("right")}:
        outer_sign = -1.0 if support_member_id == edge_rafter_id("left") else 1.0
        sign = outer_sign if support_line_ref == "support_outer_edge" else -outer_sign
        point_xyz["x"] = float(point_xyz["x"]) + sign * support_half_b_mm
        return point_xyz

    if support_member_id.startswith(INTERIOR_RAFTER_PREFIX):
        if support_line_ref != "support_centerline":
            raise ValueError(f"Unsupported support line ref {support_line_ref} for {support_member_id}")
        return point_xyz

    if support_member_id.startswith("beam."):
        sign = 1.0 if support_line_ref == "support_outer_edge" else -1.0
        point_xyz["y"] = float(point_xyz["y"]) + sign * support_half_b_mm
        return point_xyz

    raise ValueError(f"Unsupported support member for support line ref: {support_member_id}")


rafters_all = sorted(expanded_members(GEO, "rafters"), key=lambda item: (float(item["axis_start"]["x"]), item["id"]))
interior_rafters = [m for m in rafters_all if m["id"].startswith(INTERIOR_RAFTER_PREFIX) and m["id"].split(".")[-1].isdigit()]
edge_rafters = {
    "left": member(GEO, "rafters", edge_rafter_id("left")),
    "right": member(GEO, "rafters", edge_rafter_id("right")),
}

purlins_all = sorted(expanded_members(GEO, "purlins"), key=lambda item: item["id"])

infill_support_purlin = None
infill_support_purlin_support_rows = []
infill_support_purlin_support_xs_mm = []
infill_support_purlin_y_mm = 0.0
infill_support_span_mm = infill_slope_span_mm
infill_outer_overhang_mm = 0.0
if infill_glass is not None:
    infill_supported_ids = set(infill_glass.get("supported_by", []))
    infill_support_purlins = [m for m in purlins_all if m["id"] in infill_supported_ids]
    if len(infill_support_purlins) != 1:
        raise ValueError(
            "surf.wall_infill_glass must list exactly one purlin support in supported_by "
            f"(found {[m['id'] for m in infill_support_purlins]})"
        )
    infill_support_purlin = infill_support_purlins[0]
    infill_support_purlin_y_mm = float(infill_support_purlin["axis_start"]["y"])
    infill_support_span_mm = surface_slope_span_to_y_mm(infill_glass, infill_support_purlin_y_mm)
    infill_outer_overhang_mm = max(0.0, infill_slope_span_mm - infill_support_span_mm)
    for support_id in infill_glass.get("supported_by", []):
        if support_id not in MEMBERS_BY_ID:
            continue
        if support_id in {edge_rafter_id("left"), edge_rafter_id("right")} or (
            support_id.startswith(INTERIOR_RAFTER_PREFIX) and support_id.split(".")[-1].isdigit()
        ):
            support_member = member_by_id_any(support_id)
            infill_support_purlin_support_rows.append({
                "member_id": support_id,
                "x_mm": float(support_member["axis_start"]["x"]),
            })
    infill_support_purlin_support_rows = sorted(infill_support_purlin_support_rows, key=lambda row: row["x_mm"])
    if len(infill_support_purlin_support_rows) < 2:
        raise ValueError("surf.wall_infill_glass support purlin needs at least two rafter supports in supported_by")
    infill_support_purlin_support_xs_mm = [row["x_mm"] for row in infill_support_purlin_support_rows]


def is_horizontal_side_purlin(member_obj):
    if not member_obj["id"].startswith(side_purlin_prefix("left")) and not member_obj["id"].startswith(side_purlin_prefix("right")):
        return False
    suffix = member_obj["id"].split(".")[-1]
    if not suffix.isdigit():
        return False
    return abs(float(member_obj["axis_end"]["y"]) - float(member_obj["axis_start"]["y"])) <= 1e-6


main_purlins = sorted(
    [
        m
        for m in purlins_all
        if is_horizontal_side_purlin(m)
    ],
    key=lambda item: (SIDE_ORDER[member_side_from_id(item["id"])], float(item["axis_start"]["y"]), item["id"]),
)
corner_purlins_by_side = {
    side: [
        m
        for m in purlins_all
        if m["id"].startswith(side_purlin_prefix(side))
        if not is_horizontal_side_purlin(m)
    ]
    for side in ("left", "right")
}
corner_purlins_by_side = {side: members for side, members in corner_purlins_by_side.items() if members}
corner_purlins = [member_obj for side in ("left", "right") for member_obj in corner_purlins_by_side.get(side, [])]
left_purlins = sorted([m for m in main_purlins if member_side_from_id(m["id"]) == "left"], key=lambda item: float(item["axis_start"]["y"]))
right_purlins = sorted([m for m in main_purlins if member_side_from_id(m["id"]) == "right"], key=lambda item: float(item["axis_start"]["y"]))
purlins_by_side = {"left": left_purlins, "right": right_purlins}
horizontal_edge_purlin_ids = [member_obj["id"] for member_obj in left_purlins + right_purlins]
slanted_edge_purlin_ids = [member_obj["id"] for member_obj in corner_purlins]

outer_beam = member(GEO, "beams", "beam.outer")
inner_beam = member(GEO, "beams", "beam.inner.new")
existing_beam = member(GEO, "beams", "beam.existing.kp360x2")

outer_supports_x_mm = sorted(float(member(GEO, "columns", cid)["base"]["x"]) for cid in ("col.outer.x0", "col.outer.x3600", "col.outer.x7200"))
inner_supports_x_mm = sorted(float(member(GEO, "columns", cid)["base"]["x"]) for cid in ("col.existing.inner.x125", "col.existing.inner.x7075"))

interior_rafter_xs_mm = [float(m["axis_start"]["x"]) for m in interior_rafters]
left_strip_geom_width_mm = interior_rafter_xs_mm[0] - roof_x0_mm
right_strip_geom_width_mm = roof_x1_mm - interior_rafter_xs_mm[-1]
left_strip_load_width_mm = min(
    left_strip_geom_width_mm,
    load_transfer_tributary_width_mm(left_purlins[0]["id"], left_strip_geom_width_mm),
)
right_strip_load_width_mm = min(
    right_strip_geom_width_mm,
    load_transfer_tributary_width_mm(right_purlins[0]["id"], right_strip_geom_width_mm),
)
interior_direct_ranges_mm = tributary_ranges_mm(
    interior_rafter_xs_mm,
    roof_x0_mm + left_strip_load_width_mm,
    roof_x1_mm - right_strip_load_width_mm,
)
interior_direct_widths_m = [(b_mm - a_mm) / 1000.0 for a_mm, b_mm in interior_direct_ranges_mm]
left_strip_width_m = left_strip_load_width_mm / 1000.0
right_strip_width_m = right_strip_load_width_mm / 1000.0

purlin_y_positions_mm = {
    side: [float(m["axis_start"]["y"]) for m in group]
    for side, group in purlins_by_side.items()
}


def member_y_at_s_mm(member_obj, s_mm):
    length_mm = max(member_axis_length_mm(member_obj), 1.0e-9)
    t = clamp(s_mm / length_mm, 0.0, 1.0)
    return float(member_obj["axis_start"]["y"]) + t * (float(member_obj["axis_end"]["y"]) - float(member_obj["axis_start"]["y"]))


def load_transfer_y_position_mm(member_obj):
    rule = roof_load_transfer_rule(member_obj["id"])
    length_mm = member_axis_length_mm(member_obj)
    if rule.get("reference") == "axis_start":
        reference_s_mm = 0.0
    else:
        reference_s_mm = length_mm
    s_mm = clamp(reference_s_mm + float(rule.get("offset_mm", 0.0)), 0.0, length_mm)
    return member_y_at_s_mm(member_obj, s_mm)


purlin_y_boundary_positions_mm = {}
purlin_trib_ranges_mm = {}
purlin_trib_heights_m = {}
corner_purlin_y_ranges_mm = {}
corner_purlin_y_ranges_by_id = {}
corner_purlin_y_depths_m = {}
for side, group in purlins_by_side.items():
    entries = [
        ("horizontal", member_obj["id"], float(member_obj["axis_start"]["y"]))
        for member_obj in group
    ] + [
        ("corner", member_obj["id"], load_transfer_y_position_mm(member_obj))
        for member_obj in corner_purlins_by_side.get(side, [])
    ]
    entries = sorted(entries, key=lambda item: (item[2], item[0], item[1]))
    all_ranges_mm = tributary_ranges_mm([pos_mm for _, _, pos_mm in entries], roof_y0_mm, roof_y1_mm)
    range_by_entry = {
        (kind, member_id): range_mm
        for (kind, member_id, _), range_mm in zip(entries, all_ranges_mm)
    }
    purlin_y_boundary_positions_mm[side] = [pos_mm for kind, _, pos_mm in entries if kind == "horizontal"]
    purlin_trib_ranges_mm[side] = [
        range_by_entry[("horizontal", member_obj["id"])]
        for member_obj in group
    ]
    purlin_trib_heights_m[side] = [(b_mm - a_mm) / 1000.0 for a_mm, b_mm in purlin_trib_ranges_mm[side]]
    corner_purlin_y_ranges_mm[side] = [
        range_by_entry[("corner", member_obj["id"])]
        for member_obj in corner_purlins_by_side.get(side, [])
    ]
    for member_obj, range_mm in zip(corner_purlins_by_side.get(side, []), corner_purlin_y_ranges_mm[side]):
        corner_purlin_y_ranges_by_id[member_obj["id"]] = range_mm
    corner_purlin_y_depths_m[side] = [(b_mm - a_mm) / 1000.0 for a_mm, b_mm in corner_purlin_y_ranges_mm[side]]

rafter_b_mm = profile_b(interior_rafters[0])
rafter_h_mm = profile_h(interior_rafters[0])
purlin_b_mm = profile_b(left_purlins[0])
purlin_h_mm = profile_h(left_purlins[0])
infill_support_purlin_b_mm = profile_b(infill_support_purlin) if infill_support_purlin is not None else 0.0
infill_support_purlin_h_mm = profile_h(infill_support_purlin) if infill_support_purlin is not None else 0.0
outer_beam_b_mm = profile_b(outer_beam)
outer_beam_h_mm = profile_h(outer_beam)
inner_beam_b_mm = profile_b(inner_beam)
inner_beam_h_mm = profile_h(inner_beam)
existing_beam_b_mm = profile_b(existing_beam)
existing_beam_h_mm = profile_h(existing_beam)

rafter_axis_step = 1.0
left_purlin_axis_step = -1.0
right_purlin_axis_step = 1.0

interior_inner_support_y_mm = float(connection_support_point(CONNECTIONS["con.kattotuoli.on.inner_beam"], "kattotuoli.0")["y"])
interior_outer_support_y_mm = float(connection_support_point(CONNECTIONS["con.kattotuoli.on.outer_beam"], "kattotuoli.0")["y"])
edge_inner_support_y_mm = float(connection_support_point(CONNECTIONS["con.kattotuoli.vasen.on.inner_beam"], edge_rafter_id("left"))["y"])
edge_outer_support_y_mm = float(connection_support_point(CONNECTIONS["con.kattotuoli.vasen.on.outer_beam"], edge_rafter_id("left"))["y"])
rafter_analysis_start_y_mm = min(interior_inner_support_y_mm, min(float(m["axis_start"]["y"]) for m in rafters_all))
drift_obstacle_y_mm = float(GEO.get("_points_by_id", {}).get("pt.beam.inner.new.axis_start", {}).get("y", roof_y0_mm))
drift_depth_m = max(0.0, (roof_y1_mm - drift_obstacle_y_mm) / 1000.0)
rafter_analysis_end_y_mm = max(float(m["axis_end"]["y"]) for m in rafters_all)

outer_beam_y_half_mm = outer_beam_b_mm / 2.0
birdsmouth_cut = connection_cut("con.kattotuoli.on.outer_beam", "birdsmouth_notch")
birdsmouth_anchor_y_mm = interior_outer_support_y_mm - outer_beam_y_half_mm
birdsmouth_heel_depth_mm = float(birdsmouth_cut["heel_depth_mm"])
birdsmouth_seat_length_mm = float(birdsmouth_cut["seat_length_mm"])
birdsmouth_seat_proj_mm = birdsmouth_seat_length_mm * math.cos(roof_slope_rad)
birdsmouth_start_depth_mm = max(0.0, birdsmouth_heel_depth_mm - birdsmouth_seat_length_mm * math.sin(roof_slope_rad))
birdsmouth_zone_mm = (birdsmouth_anchor_y_mm - birdsmouth_seat_proj_mm, birdsmouth_anchor_y_mm)

edge_top_notch_info = {
    side: rect_notch_info(optional_connection_by_members(
        edge_rafter_id(side),
        side_purlin_root_id(side),
        predicate=lambda conn: connection_cut(conn, "rect_notch") is not None,
    ))
    for side in ("left", "right")
}
edge_top_notch_ref = next((item for item in edge_top_notch_info.values() if item["active"]), None)
edge_rect_depth_mm = edge_top_notch_ref["depth_mm"] if edge_top_notch_ref else 0.0
edge_rect_length_mm = edge_top_notch_ref["length_mm"] if edge_top_notch_ref else 0.0

interior_edge_rafter_ids = {
    "left": interior_rafters[0]["id"],
    "right": interior_rafters[-1]["id"],
}


def purlin_edge_support_notch_info(connection_obj):
    rect_info = rect_notch_info(connection_obj)
    if rect_info["active"]:
        rect_info["kind"] = "rect_notch"
        return rect_info
    bevel_info = bevel_notch_info(connection_obj)
    if bevel_info["active"]:
        bevel_info["kind"] = "bevel_notch"
        return bevel_info
    rect_info["kind"] = None
    return rect_info


purlin_inner_support_connections_by_id = {}
purlin_edge_support_connections_by_id = {}
purlin_inner_support_points_by_id = {}
purlin_edge_support_center_points_by_id = {}
purlin_edge_support_line_points_by_id = {}
purlin_inner_notch_info_by_id = {}
purlin_edge_notch_info_by_id = {}

for side, group in purlins_by_side.items():
    for member_obj in group:
        member_id = member_obj["id"]
        inner_conn = support_connection(
            member_id,
            interior_edge_rafter_ids[side],
            "support_centerline",
            connection_objs=CONNECTION_INSTANCES,
        )
        edge_conn = support_connection(
            member_id,
            edge_rafter_id(side),
            "support_outer_edge",
            connection_objs=CONNECTION_INSTANCES,
        )
        purlin_inner_support_connections_by_id[member_id] = inner_conn
        purlin_edge_support_connections_by_id[member_id] = edge_conn
        purlin_inner_support_points_by_id[member_id] = connection_support_point(inner_conn, member_id)
        purlin_edge_support_center_points_by_id[member_id] = connection_support_point(edge_conn, member_id, "support_centerline")
        purlin_edge_support_line_points_by_id[member_id] = connection_support_point(edge_conn, member_id, "support_outer_edge")
        purlin_inner_notch_info_by_id[member_id] = bevel_notch_info(inner_conn)
        purlin_edge_notch_info_by_id[member_id] = purlin_edge_support_notch_info(edge_conn)

purlin_inner_support_connections = {
    side: purlin_inner_support_connections_by_id[group[0]["id"]]
    for side, group in purlins_by_side.items()
}
purlin_edge_support_connections = {
    side: purlin_edge_support_connections_by_id[group[0]["id"]]
    for side, group in purlins_by_side.items()
}
purlin_inner_notch_info = {
    side: purlin_inner_notch_info_by_id[group[0]["id"]]
    for side, group in purlins_by_side.items()
}
purlin_edge_notch_info = {
    side: purlin_edge_notch_info_by_id[group[0]["id"]]
    for side, group in purlins_by_side.items()
}
edge_purlin_support_ys_mm = {
    side: [float(purlin_edge_support_line_points_by_id[member_obj["id"]]["y"]) for member_obj in group]
    for side, group in purlins_by_side.items()
}
interior_purlin_support_ys_mm = {
    side: [float(purlin_inner_support_points_by_id[member_obj["id"]]["y"]) for member_obj in group]
    for side, group in purlins_by_side.items()
}
purlin_edge_notch_ref = next((item for item in purlin_edge_notch_info_by_id.values() if item["active"]), None)
purlin_edge_notch_depth_mm = purlin_edge_notch_ref["depth_mm"] if purlin_edge_notch_ref else 0.0
purlin_edge_notch_length_mm = purlin_edge_notch_ref["length_mm"] if purlin_edge_notch_ref else 0.0
left_purlin_inner_support_conn = purlin_inner_support_connections["left"]
right_purlin_inner_support_conn = purlin_inner_support_connections["right"]
left_purlin_edge_support_conn = purlin_edge_support_connections["left"]
right_purlin_edge_support_conn = purlin_edge_support_connections["right"]
left_purlin_support_x_mm = float(purlin_inner_support_points_by_id[left_purlins[0]["id"]]["x"])
right_purlin_support_x_mm = float(purlin_inner_support_points_by_id[right_purlins[0]["id"]]["x"])
left_purlin_edge_support_center_x_mm = float(purlin_edge_support_center_points_by_id[left_purlins[0]["id"]]["x"])
right_purlin_edge_support_center_x_mm = float(purlin_edge_support_center_points_by_id[right_purlins[0]["id"]]["x"])
corner_purlin_inner_support_connections = {}
corner_purlin_inner_support_points = {}
corner_purlin_inner_support_member_ids = {}
corner_purlin_outer_support_connections = {}
corner_purlin_outer_support_points = {}
corner_purlin_outer_support_member_ids = {}
corner_purlin_outer_notch_info = {}
corner_purlin_trib_width_m = {}
side_corner_strip_outer_x_mm = {"left": roof_x0_mm, "right": roof_x1_mm}
side_corner_strip_width_mm = {
    "left": min(
        left_strip_geom_width_mm,
        load_transfer_tributary_width_mm(corner_purlins_by_side.get("left", left_purlins)[0]["id"], left_strip_load_width_mm)
        if corner_purlins_by_side.get("left") else left_strip_load_width_mm,
    ),
    "right": min(
        right_strip_geom_width_mm,
        load_transfer_tributary_width_mm(corner_purlins_by_side.get("right", right_purlins)[0]["id"], right_strip_load_width_mm)
        if corner_purlins_by_side.get("right") else right_strip_load_width_mm,
    ),
}
side_corner_strip_inner_x_mm = {
    "left": roof_x0_mm + side_corner_strip_width_mm["left"],
    "right": roof_x1_mm - side_corner_strip_width_mm["right"],
}
inactive_bevel_notch_info = {
    "active": False,
    "depth_mm": 0.0,
    "length_mm": 0.0,
    "offset_mm": 0.0,
    "reference": None,
    "side": None,
}

for side, members in corner_purlins_by_side.items():
    for member_obj in members:
        member_id = member_obj["id"]
        inner_conn = first_connection_matching(
            member_id,
            lambda _conn, conn_members: any(other.startswith(INTERIOR_RAFTER_PREFIX) and other.split(".")[-1].isdigit() for other in conn_members if other != member_id),
        )
        corner_purlin_inner_support_connections[member_id] = inner_conn
        corner_purlin_inner_support_points[member_id] = connection_support_point(inner_conn, member_id) if inner_conn is not None else dict(member_obj["axis_start"])
        corner_purlin_inner_support_member_ids[member_id] = None if inner_conn is None else next(
            (other for other in inner_conn.get("members", []) if other != member_id),
            None,
        )

        outer_conn = first_connection_matching(
            member_id,
            lambda _conn, conn_members, side=side: any(
                other in {"beam.outer", edge_rafters[side]["id"]}
                for other in conn_members
                if other != member_id
            ),
        )
        if outer_conn is None:
            raise ValueError(f"Outer support connection not found for slanted purlin {member_id}")
        outer_support_member_id = next((other for other in outer_conn.get("members", []) if other != member_id), None)
        corner_purlin_outer_support_connections[member_id] = outer_conn
        corner_purlin_outer_support_member_ids[member_id] = outer_support_member_id
        corner_purlin_outer_support_points[member_id] = connection_support_point(outer_conn, member_id)
        corner_purlin_outer_notch_info[member_id] = bevel_notch_info(outer_conn["id"])

for side, members in corner_purlins_by_side.items():
    ordered = sorted(
        members,
        key=load_transfer_y_position_mm,
    )
    corner_purlins_by_side[side] = ordered
    for member_obj in ordered:
        a_mm, b_mm = corner_purlin_y_ranges_by_id.get(member_obj["id"], (roof_y1_mm, roof_y1_mm))
        member_length_mm = max(member_axis_length_mm(member_obj), 1.0e-9)
        corner_area_mm2 = side_corner_strip_width_mm[side] * max(0.0, b_mm - a_mm)
        corner_purlin_trib_width_m[member_obj["id"]] = corner_area_mm2 / member_length_mm / 1000.0

outer_glazing_surfaces = [surface(GEO, "surf.side_glazing.outer.vasen"), surface(GEO, "surf.side_glazing.outer.oikea")]
outer_glazing_intervals = []
outer_glazing_height_m = 0.0
for glazing in outer_glazing_surfaces:
    poly = glazing["polygon"]
    x0_mm = min(p["x"] for p in poly)
    x1_mm = max(p["x"] for p in poly)
    z0_mm = min(p["z"] for p in poly)
    z1_mm = max(p["z"] for p in poly)
    outer_glazing_intervals.append((x0_mm, x1_mm))
    outer_glazing_height_m = max(outer_glazing_height_m, (z1_mm - z0_mm) / 1000.0)

all_z_mm = [
    p["z"]
    for s in GEO.get("surfaces", [])
    for p in s.get("polygon", [])
    if "z" in p
] + [
    m[k]["z"]
    for group in GEO["members"].values()
    for m in group
    for k in ("axis_start", "axis_end", "base", "top")
    if k in m and isinstance(m[k], dict) and "z" in m[k]
]
z_ref_m = math.ceil(max(all_z_mm) / 500.0) * 0.5


# ── kuormat ja materiaalit ──────────────────────────────────────────────────

panel_count = roof["count"]
panel_material = roof.get("material", "aurinkopaneeli")
panel_unit_width_mm = float(panel_count["unit_size_mm"]["x"])
panel_unit_slope_length_mm = float(panel_count["unit_size_mm"]["y"])
panel_unit_thickness_mm = float(panel_count["unit_size_mm"].get("z", roof.get("thickness_mm", 0.0)))
panel_mass_kg = float(panel_count["unit_mass_kg"])
panel_count_x = int(panel_count["nx"])
panel_count_y = int(panel_count["ny"])
panel_count_total = int(panel_count["nx"]) * int(panel_count["ny"])
panel_total_mass_kg = panel_count_total * panel_mass_kg
panel_field_slope_width_mm = panel_count_x * panel_unit_width_mm
panel_field_slope_length_mm = panel_count_y * panel_unit_slope_length_mm
panel_row_projected_depth_mm = roof_depth_mm / max(panel_count_y, 1)
panel_front_snow_cap_kNm2 = 5.40
panels_total_kN = panel_total_mass_kg * 9.81 / 1000.0
gk_panels = panels_total_kN / roof_area_m2
gk_fixings = 0.05
gk_roofing = gk_panels + gk_fixings
if infill_glass is not None:
    infill_glass_thickness_mm = float(infill_glass["thickness_mm"])
    infill_glass_density_kg_m3 = float(infill_glass.get("density_kg_m3", 2500.0))
    infill_glass_mass_kg = infill_slope_area_m2 * (infill_glass_thickness_mm / 1000.0) * infill_glass_density_kg_m3
    infill_glass_total_kN = infill_glass_mass_kg * 9.81 / 1000.0
    gk_infill_glass = infill_glass_total_kN / infill_area_m2
else:
    infill_glass_thickness_mm = 0.0
    infill_glass_density_kg_m3 = 0.0
    infill_glass_mass_kg = 0.0
    infill_glass_total_kN = 0.0
    gk_infill_glass = 0.0
if gable_glazing is not None:
    gable_glass_thickness_mm = float(gable_glazing.get("thickness_mm", 8.0))
    gable_glass_density_kg_m3 = float(gable_glazing.get("density_kg_m3", 2500.0))
    gable_glass_mass_kg = gable_area_m2 * (gable_glass_thickness_mm / 1000.0) * gable_glass_density_kg_m3
    gable_glass_total_kN = gable_glass_mass_kg * 9.81 / 1000.0
    gable_glass_line_self_kNm = gable_glass_total_kN / max(gable_base_length_m, 1.0e-9)
else:
    gable_glass_thickness_mm = 0.0
    gable_glass_density_kg_m3 = 0.0
    gable_glass_mass_kg = 0.0
    gable_glass_total_kN = 0.0
    gable_glass_line_self_kNm = 0.0

sk = 2.0
mu1 = 0.8
s_roof = mu1 * sk

vb0 = 21.0
rho_air = 1.25
z0 = 0.05
kr = 0.19
cr_z = kr * math.log(max(z_ref_m, 2.0) / z0)
Iv_z = 1.0 / math.log(max(z_ref_m, 2.0) / z0)
vm_z = cr_z * vb0
qp_z = (1.0 + 7.0 * Iv_z) * 0.5 * rho_air * vm_z**2 / 1000.0

cp_net_down = 0.80
cp_net_up = -0.60
w_down = cp_net_down * qp_z
w_up = cp_net_up * qp_z
cpe_H_t0 = -0.54
cpe_H_t180 = -0.92
cpi_unfav_up = +0.20
cp_net_up_closed = min(cpe_H_t0, cpe_H_t180) - cpi_unfav_up
w_up_closed = cp_net_up_closed * qp_z

cp_wall_net = 1.0
q_outer_wind_h_char = qp_z * cp_wall_net * (outer_glazing_height_m / 2.0)
if gable_glazing is not None:
    gable_wind_share_per_beam = 0.50
    gable_wind_char_total_kN = qp_z * cp_wall_net * gable_area_m2
    gable_wind_char_to_inner_kN = gable_wind_share_per_beam * gable_wind_char_total_kN
    gable_wind_char_to_existing_kN = gable_wind_share_per_beam * gable_wind_char_total_kN
    gable_wind_line_inner_char_kNm = gable_wind_char_to_inner_kN / max(gable_base_length_m, 1.0e-9)
    gable_wind_line_existing_char_kNm = gable_wind_char_to_existing_kN / max(gable_base_length_m, 1.0e-9)
    gable_glazing_summary = {
        "material": gable_glazing.get("material", "lasi"),
        "thickness_mm": gable_glass_thickness_mm,
        "area_m2": gable_area_m2,
        "mass_kg": gable_glass_mass_kg,
        "total_kN": gable_glass_total_kN,
        "self_line_kNm": gable_glass_line_self_kNm,
        "x0_mm": gable_bottom_x0_mm,
        "x1_mm": gable_bottom_x1_mm,
        "base_length_m": gable_base_length_m,
        "wind_char_total_kN": gable_wind_char_total_kN,
        "wind_share_per_beam": gable_wind_share_per_beam,
        "wind_line_inner_char_kNm": gable_wind_line_inner_char_kNm,
        "wind_line_existing_char_kNm": gable_wind_line_existing_char_kNm,
    }

rho_c24 = 420.0
gamma_c24 = rho_c24 * 9.81 / 1000.0
gamma_gl30c = 5.0
rho_lvl = 480.0
gamma_lvl = rho_lvl * 9.81 / 1000.0

rafter_self_kNm = (rafter_b_mm / 1000.0) * (rafter_h_mm / 1000.0) * gamma_c24 / math.cos(roof_slope_rad)
purlin_self_kNm = (purlin_b_mm / 1000.0) * (purlin_h_mm / 1000.0) * gamma_c24
infill_support_purlin_self_kNm = (infill_support_purlin_b_mm / 1000.0) * (infill_support_purlin_h_mm / 1000.0) * gamma_c24
outer_beam_self_kNm = (outer_beam_b_mm / 1000.0) * (outer_beam_h_mm / 1000.0) * gamma_gl30c
inner_beam_self_kNm = (inner_beam_b_mm / 1000.0) * (inner_beam_h_mm / 1000.0) * gamma_gl30c
existing_beam_self_kNm = (existing_beam_b_mm / 1000.0) * (existing_beam_h_mm / 1000.0) * gamma_lvl

E_c24 = 11000.0
E_gl30c = 13000.0
E_lvl = 13800.0
kmod_c24 = 0.8
gammaM_c24 = 1.3
fm_d_c24 = kmod_c24 * 24.0 / gammaM_c24
fv_d_c24 = kmod_c24 * 4.0 / gammaM_c24
kmod_gl30c = 0.8
fm_d_gl30c = kmod_gl30c * 30.0 / 1.25
fv_d_gl30c = kmod_gl30c * 3.5 / 1.25
kmod_lvl = 0.8
gammaM_lvl = 1.2
fm_d_lvl = kmod_lvl * 44.0 / gammaM_lvl
fv_d_lvl = kmod_lvl * 4.5 / gammaM_lvl

gammaG = 1.35
gammaQ = 1.50
psi0_W = 0.6
psi0_snow = 0.7

def ec5_rotational_spring_k_Nmm_per_rad(fastener_d_mm, fastener_count, effective_height_mm):
    kser_per_fastener_N_per_mm = rho_c24**1.5 * fastener_d_mm / 23.0
    return fastener_count * kser_per_fastener_N_per_mm * effective_height_mm**2 / 12.0


DEFAULT_INNER_HANGER_ANALYSIS = {
    "support_model": "semi_rigid",
    "support_line_ref": "support_centerline",
    "label": "Palkkikenkä N 48x136",
    "fastener_label": "5.0x40 ankkuriruuvi",
    "rotation_spring": {
        "model": "ec5_fasteners",
        "fastener_d_mm": 5.0,
        "fastener_count": 24,
        "effective_height_mm": 136.0,
        "rho_kg_m3": rho_c24,
    },
}
DEFAULT_PINNED_CENTER_ANALYSIS = {"support_model": "pinned", "support_line_ref": "support_centerline"}
DEFAULT_PINNED_OUTER_EDGE_ANALYSIS = {"support_model": "pinned", "support_line_ref": "support_outer_edge"}

DEFAULT_CONNECTION_ANALYSIS_BY_ID = {
    "con.kattotuoli.on.inner_beam": DEFAULT_INNER_HANGER_ANALYSIS,
    "con.kattotuoli.on.outer_beam": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.kattotuoli.vasen.on.inner_beam": DEFAULT_INNER_HANGER_ANALYSIS,
    "con.kattotuoli.oikea.on.inner_beam": DEFAULT_INNER_HANGER_ANALYSIS,
    "con.kattotuoli.vasen.on.outer_beam": DEFAULT_INNER_HANGER_ANALYSIS,
    "con.kattotuoli.oikea.on.outer_beam": DEFAULT_INNER_HANGER_ANALYSIS,
    "con.orsi.vasen.on.kattotuoli.0": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.oikea.on.kattotuoli.5": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.vasen.on.kattotuoli.vasen": DEFAULT_PINNED_OUTER_EDGE_ANALYSIS,
    "con.orsi.oikea.on.kattotuoli.oikea": DEFAULT_PINNED_OUTER_EDGE_ANALYSIS,
    "con.orsi.vasen.67.on.kattotuoli.0": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.vasen.67.on.kattotuoli.vasen": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.vasen.45.on.kattotuoli.0": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.vasen.45.on.outer_beam": DEFAULT_PINNED_OUTER_EDGE_ANALYSIS,
    "con.orsi.vasen.22.on.kattotuoli.0": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.vasen.22.on.outer_beam": DEFAULT_PINNED_OUTER_EDGE_ANALYSIS,
    "con.orsi.oikea.67.on.kattotuoli.5": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.oikea.67.on.kattotuoli.oikea": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.oikea.45.on.kattotuoli.5": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.oikea.45.on.outer_beam": DEFAULT_PINNED_OUTER_EDGE_ANALYSIS,
    "con.orsi.oikea.22.on.kattotuoli.5": DEFAULT_PINNED_CENTER_ANALYSIS,
    "con.orsi.oikea.22.on.outer_beam": DEFAULT_PINNED_OUTER_EDGE_ANALYSIS,
}
RIGID_SUPPORT_ROT_K_NMM_PER_RAD = 1.0e15


def merge_analysis_dict(base, override):
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            nested = dict(merged[key])
            nested.update(value)
            merged[key] = nested
        else:
            merged[key] = value
    return merged


def connection_analysis_info(connection_id):
    default_info = DEFAULT_CONNECTION_ANALYSIS_BY_ID.get(connection_id, DEFAULT_PINNED_CENTER_ANALYSIS)
    analysis = CONNECTIONS[connection_id].get("analysis", {})
    merged = merge_analysis_dict(default_info, analysis)
    merged.setdefault("support_model", "pinned")
    merged.setdefault("support_line_ref", "support_centerline")
    return merged


def rotation_spring_k_from_info(spring_info):
    if spring_info is None:
        return None
    if spring_info["model"] == "explicit":
        return float(spring_info["k_theta_Nmm_per_rad"])
    if spring_info["model"] == "ec5_fasteners":
        rho_kg_m3 = float(spring_info["rho_kg_m3"])
        fastener_d_mm = float(spring_info["fastener_d_mm"])
        fastener_count = int(spring_info["fastener_count"])
        effective_height_mm = float(spring_info["effective_height_mm"])
        kser_per_fastener_N_per_mm = rho_kg_m3**1.5 * fastener_d_mm / 23.0
        return fastener_count * kser_per_fastener_N_per_mm * effective_height_mm**2 / 12.0
    raise ValueError(f"Unsupported rotation spring model: {spring_info['model']}")


def connection_rotational_spring_k_Nmm_per_rad(connection_id):
    analysis = connection_analysis_info(connection_id)
    if analysis["support_model"] == "pinned":
        return None
    if analysis["support_model"] == "rigid":
        return RIGID_SUPPORT_ROT_K_NMM_PER_RAD
    return rotation_spring_k_from_info(analysis["rotation_spring"])


inner_hanger_analysis = connection_analysis_info("con.kattotuoli.on.inner_beam")
inner_hanger_name = inner_hanger_analysis.get("label", "liitos")
inner_hanger_fastener = inner_hanger_analysis.get("fastener_label", "")
inner_hanger_rot_k_Nmm_per_rad = connection_rotational_spring_k_Nmm_per_rad("con.kattotuoli.on.inner_beam")

edge_rafter_support_analysis_by_side = {
    "left": connection_analysis_info("con.kattotuoli.vasen.on.inner_beam"),
    "right": connection_analysis_info("con.kattotuoli.oikea.on.inner_beam"),
}
edge_rafter_support_rot_k_by_side = {
    side: connection_rotational_spring_k_Nmm_per_rad(f"con.kattotuoli.{SIDE_MEMBER_LABEL[side]}.on.inner_beam")
    for side in ("left", "right")
}


def format_connection_behavior(analysis, rot_k_Nmm_per_rad):
    parts = []
    label = analysis.get("label")
    fastener_label = analysis.get("fastener_label")
    if label:
        parts.append(label)
    if fastener_label:
        parts.append(fastener_label)
    if analysis["support_model"] == "pinned":
        parts.append("nivel")
    elif analysis["support_model"] == "rigid":
        parts.append("jäykkä")
    elif rot_k_Nmm_per_rad is not None:
        parts.append(f"kθ ≈ {rot_k_Nmm_per_rad/1.0e6:.1f} kNm/rad")
    return ", ".join(parts) if parts else analysis["support_model"]

roof_area_uls_A = gammaG * gk_roofing + gammaQ * s_roof + gammaQ * psi0_W * w_down
roof_area_uls_B = gammaG * gk_roofing + gammaQ * psi0_snow * s_roof
roof_area_sls = gk_roofing + s_roof
roof_area_uplift = 0.9 * gk_roofing - gammaQ * abs(w_up_closed)


def snow_drift_params(h_m, b_panel_m, sk_, gamma_s=2.0, mu1_=0.8):
    if h_m <= 1e-9:
        s1_ = mu1_ * sk_
        return 0.0, mu1_, 0.0, s1_, s1_
    ls = min(5.0 * h_m, b_panel_m, 15.0)
    ls = max(ls, 0.5 * h_m)
    mu2_h_ = gamma_s * h_m / sk_
    mu2_ = min(max(mu2_h_, mu1_), 2.0)
    s1_ = mu1_ * sk_
    s_dr_ = mu2_ * sk_
    return ls, mu2_, mu2_h_, s1_, s_dr_


def drift_snow_kNm2(x_mm, y_mm):
    h_local_m = local_wall_height_m_at_x(x_mm)
    if h_local_m <= 1e-9:
        return s_roof
    ls_m, _, _, _, s_peak_kNm2 = snow_drift_params(h_local_m, drift_depth_m, sk, mu1_=mu1)
    distance_m = max(0.0, (y_mm - drift_obstacle_y_mm) / 1000.0)
    s_drift_local = s_peak_kNm2 * max(0.0, 1.0 - distance_m / ls_m)
    return max(s_roof, s_drift_local)


def drift_snow_kNm2_from_height(h_m, y_offset_mm):
    if h_m <= 1e-9:
        return s_roof
    ls_m, _, _, _, s_peak_kNm2 = snow_drift_params(h_m, drift_depth_m, sk, mu1_=mu1)
    distance_m = max(0.0, y_offset_mm / 1000.0)
    s_drift_local = s_peak_kNm2 * max(0.0, 1.0 - distance_m / ls_m)
    return max(s_roof, s_drift_local)


def wall_height_limit_for_panel_capacity(capacity_kNm2, y_offset_mm):
    if gammaQ * s_roof > capacity_kNm2 + 1e-9:
        return None
    low_h_m = 0.0
    high_h_m = max(0.5, critical_drift["h_m"] if "critical_drift" in globals() else 0.5)
    while gammaQ * drift_snow_kNm2_from_height(high_h_m, y_offset_mm) <= capacity_kNm2 and high_h_m < 20.0:
        high_h_m *= 2.0
    for _ in range(80):
        mid_h_m = 0.5 * (low_h_m + high_h_m)
        if gammaQ * drift_snow_kNm2_from_height(mid_h_m, y_offset_mm) <= capacity_kNm2:
            low_h_m = mid_h_m
        else:
            high_h_m = mid_h_m
    return low_h_m


def constant_roof_area_kNm2_at(value_kNm2):
    return lambda _x_mm, _y_mm, value_kNm2=value_kNm2: value_kNm2


CASE_DEFS = {
    "ULS A": {"roof_area_kNm2_at": constant_roof_area_kNm2_at(roof_area_uls_A), "gamma_self": gammaG},
    "ULS B": {"roof_area_kNm2_at": constant_roof_area_kNm2_at(roof_area_uls_B), "gamma_self": gammaG},
    "ULS DRIFT": {"roof_area_kNm2_at": lambda x_mm, y_mm: gammaG * gk_roofing + gammaQ * drift_snow_kNm2(x_mm, y_mm), "gamma_self": gammaG},
    "SLS": {"roof_area_kNm2_at": constant_roof_area_kNm2_at(roof_area_sls), "gamma_self": 1.0},
    "SLS DRIFT": {"roof_area_kNm2_at": lambda x_mm, y_mm: gk_roofing + drift_snow_kNm2(x_mm, y_mm), "gamma_self": 1.0},
    "UPLIFT": {"roof_area_kNm2_at": constant_roof_area_kNm2_at(roof_area_uplift), "gamma_self": 0.9},
}
ULS_CASE_KEYS = ("ULS A", "ULS B", "ULS DRIFT")
SLS_CASE_KEYS = ("SLS", "SLS DRIFT")

DRIFT_SUMMARY = []
for x_mm in sorted({roof_x0_mm, roof_x1_mm, *[float(m["axis_start"]["x"]) for m in rafters_all]}):
    h_local_m = local_wall_height_m_at_x(x_mm)
    ls_m, mu2, mu2_h, _, s_peak_kNm2 = snow_drift_params(h_local_m, roof_depth_m, sk, mu1_=mu1)
    DRIFT_SUMMARY.append({
        "x_mm": x_mm,
        "h_m": h_local_m,
        "ls_m": ls_m,
        "mu2": mu2,
        "mu2_h": mu2_h,
        "s_peak_kNm2": s_peak_kNm2,
        "s_inner_rafter_kNm2": drift_snow_kNm2(x_mm, interior_inner_support_y_mm),
        "s_inner_edge_rafter_kNm2": drift_snow_kNm2(x_mm, edge_inner_support_y_mm),
    })
critical_drift = max(DRIFT_SUMMARY, key=lambda item: (item["s_peak_kNm2"], item["h_m"], item["x_mm"]))


def glass_strip_response(q_kNm2, span_mm, thickness_mm, E_Nmm2=70000.0):
    q_N_per_mm = q_kNm2
    W_mm3_per_m = 1000.0 * thickness_mm**2 / 6.0
    I_mm4_per_m = 1000.0 * thickness_mm**3 / 12.0
    M_Nmm_per_m = q_N_per_mm * span_mm**2 / 8.0
    delta_mm = 5.0 * q_N_per_mm * span_mm**4 / (384.0 * E_Nmm2 * I_mm4_per_m)
    return {
        "M_kNm_per_m": M_Nmm_per_m / 1.0e6,
        "sigma_Nmm2": M_Nmm_per_m / W_mm3_per_m,
        "delta_mm": delta_mm,
    }


def glass_strip_response_with_outer_support(q_kNm2, support_span_mm, total_span_mm, thickness_mm, E_Nmm2=70000.0):
    q_N_per_mm = abs(q_kNm2)
    W_mm3_per_m = 1000.0 * thickness_mm**2 / 6.0
    I_mm4_per_m = 1000.0 * thickness_mm**3 / 12.0
    support_span_mm = max(support_span_mm, 1.0e-9)
    total_span_mm = max(total_span_mm, support_span_mm)
    overhang_mm = max(0.0, total_span_mm - support_span_mm)
    if overhang_mm <= 1e-9:
        return glass_strip_response(q_N_per_mm, support_span_mm, thickness_mm, E_Nmm2)
    outer_reaction_N_per_mm = q_N_per_mm * total_span_mm**2 / (2.0 * support_span_mm)
    inner_reaction_N_per_mm = q_N_per_mm * total_span_mm - outer_reaction_N_per_mm
    max_pos_x_mm = clamp(inner_reaction_N_per_mm / max(q_N_per_mm, 1.0e-9), 0.0, support_span_mm)
    M_pos_Nmm_per_m = inner_reaction_N_per_mm * max_pos_x_mm - q_N_per_mm * max_pos_x_mm**2 / 2.0
    M_neg_Nmm_per_m = q_N_per_mm * overhang_mm**2 / 2.0
    M_Nmm_per_m = max(M_pos_Nmm_per_m, M_neg_Nmm_per_m)
    delta_mm = 5.0 * q_N_per_mm * support_span_mm**4 / (384.0 * E_Nmm2 * I_mm4_per_m)
    return {
        "M_kNm_per_m": M_Nmm_per_m / 1.0e6,
        "sigma_Nmm2": M_Nmm_per_m / W_mm3_per_m,
        "delta_mm": delta_mm,
    }


def glass_strip_support_reactions_kN_per_m(q_kNm2, support_span_mm, total_span_mm):
    support_span_m = max(support_span_mm / 1000.0, 1.0e-9)
    total_span_m = max(total_span_mm / 1000.0, support_span_m)
    if total_span_m <= support_span_m + 1.0e-9:
        outer_reaction = q_kNm2 * total_span_m / 2.0
    else:
        outer_reaction = q_kNm2 * total_span_m**2 / (2.0 * support_span_m)
    inner_reaction = q_kNm2 * total_span_m - outer_reaction
    return inner_reaction, outer_reaction


if infill_glass is not None:
    infill_glass_design_strength_Nmm2 = 10.0
    infill_glass_deflection_limit_mm = infill_support_span_mm / 200.0
    infill_check_xs_mm = sorted({
        infill_bounds["x0_mm"],
        infill_bounds["x1_mm"],
        *[float(m["axis_start"]["x"]) for m in rafters_all],
    })
    infill_depth_rows = [
        (infill_bounds["y0_mm"], "seinareuna"),
        (0.5 * (infill_bounds["y0_mm"] + infill_bounds["y1_mm"]), "kaistan keskella"),
        (infill_bounds["y1_mm"], "ulkoreuna"),
    ]
    infill_snow_rows = []
    for y_mm, label in infill_depth_rows:
        row = max(
            (
                {
                    "label": label,
                    "x_mm": x_mm,
                    "y_mm": y_mm,
                    "s_char_kNm2": drift_snow_kNm2(x_mm, y_mm),
                }
                for x_mm in infill_check_xs_mm
            ),
            key=lambda item: (item["s_char_kNm2"], item["x_mm"]),
        )
        row["uls_snow_kNm2"] = gammaQ * row["s_char_kNm2"]
        infill_snow_rows.append(row)
    critical_infill_snow = max(infill_snow_rows, key=lambda item: (item["s_char_kNm2"], item["x_mm"]))
    infill_uls_A_kNm2 = gammaG * gk_infill_glass + gammaQ * s_roof + gammaQ * psi0_W * w_down
    infill_uls_drift_kNm2 = gammaG * gk_infill_glass + gammaQ * critical_infill_snow["s_char_kNm2"]
    infill_uls_down_kNm2 = max(infill_uls_A_kNm2, infill_uls_drift_kNm2)
    infill_sls_drift_kNm2 = gk_infill_glass + critical_infill_snow["s_char_kNm2"]
    infill_uplift_kNm2 = 0.9 * gk_infill_glass - gammaQ * abs(w_up_closed)
    infill_uls_response = glass_strip_response_with_outer_support(
        infill_uls_down_kNm2,
        infill_support_span_mm,
        infill_slope_span_mm,
        infill_glass_thickness_mm,
    )
    infill_sls_response = glass_strip_response_with_outer_support(
        infill_sls_drift_kNm2,
        infill_support_span_mm,
        infill_slope_span_mm,
        infill_glass_thickness_mm,
    )
    infill_uplift_response = glass_strip_response_with_outer_support(
        abs(infill_uplift_kNm2),
        infill_support_span_mm,
        infill_slope_span_mm,
        infill_glass_thickness_mm,
    )
    infill_inner_reaction_kN_per_m, infill_outer_reaction_kN_per_m = glass_strip_support_reactions_kN_per_m(
        infill_uls_down_kNm2,
        infill_support_span_mm,
        infill_slope_span_mm,
    )
    infill_glass_summary = {
        "material": infill_glass.get("material", "lasi"),
        "thickness_mm": infill_glass_thickness_mm,
        "density_kg_m3": infill_glass_density_kg_m3,
        "width_mm": infill_width_mm,
        "depth_mm": infill_depth_mm,
        "span_mm": infill_support_span_mm,
        "total_span_mm": infill_slope_span_mm,
        "outer_overhang_mm": infill_outer_overhang_mm,
        "outer_support_member_id": infill_support_purlin["id"],
        "outer_support_y_mm": infill_support_purlin_y_mm,
        "area_m2": infill_area_m2,
        "slope_area_m2": infill_slope_area_m2,
        "mass_kg": infill_glass_mass_kg,
        "total_kN": infill_glass_total_kN,
        "gk_kNm2": gk_infill_glass,
        "snow_rows": infill_snow_rows,
        "critical_snow": critical_infill_snow,
        "uls_A_kNm2": infill_uls_A_kNm2,
        "uls_drift_kNm2": infill_uls_drift_kNm2,
        "uls_down_kNm2": infill_uls_down_kNm2,
        "sls_drift_kNm2": infill_sls_drift_kNm2,
        "uplift_kNm2": infill_uplift_kNm2,
        "uplift_sigma_Nmm2": infill_uplift_response["sigma_Nmm2"],
        "inner_support_reaction_kN_per_m": infill_inner_reaction_kN_per_m,
        "outer_support_reaction_kN_per_m": infill_outer_reaction_kN_per_m,
        "M_kNm_per_m": infill_uls_response["M_kNm_per_m"],
        "sigma_Nmm2": infill_uls_response["sigma_Nmm2"],
        "sigma_limit_Nmm2": infill_glass_design_strength_Nmm2,
        "eta_sigma_pct": 100.0 * infill_uls_response["sigma_Nmm2"] / infill_glass_design_strength_Nmm2,
        "delta_mm": infill_sls_response["delta_mm"],
        "delta_limit_mm": infill_glass_deflection_limit_mm,
        "eta_delta_pct": 100.0 * infill_sls_response["delta_mm"] / infill_glass_deflection_limit_mm,
    }


def infill_area_load_kNm2_for_case(case_key):
    if infill_glass_summary is None:
        return 0.0
    if case_key == "ULS A":
        return infill_glass_summary["uls_A_kNm2"]
    if case_key == "ULS B":
        return gammaG * gk_infill_glass + gammaQ * psi0_snow * s_roof
    if case_key == "ULS DRIFT":
        return infill_glass_summary["uls_drift_kNm2"]
    if case_key == "SLS":
        return gk_infill_glass + s_roof
    if case_key == "SLS DRIFT":
        return infill_glass_summary["sls_drift_kNm2"]
    if case_key == "UPLIFT":
        return infill_glass_summary["uplift_kNm2"]
    raise KeyError(f"Unsupported infill load case: {case_key}")


def infill_support_reactions_for_case(case_key):
    q_kNm2 = infill_area_load_kNm2_for_case(case_key)
    inner_reaction, outer_reaction = glass_strip_support_reactions_kN_per_m(
        q_kNm2,
        infill_support_span_mm,
        infill_slope_span_mm,
    )
    return {
        "q_kNm2": q_kNm2,
        "inner_reaction_kN_per_m": inner_reaction,
        "outer_reaction_kN_per_m": outer_reaction,
    }


def dedupe_panel_checkpoints(points):
    deduped = []
    seen = set()
    for dy_mm, label in points:
        dy_key = round(float(dy_mm), 6)
        if dy_key in seen:
            continue
        seen.add(dy_key)
        deduped.append((float(dy_mm), label))
    return deduped


def panel_depth_checkpoints(projected_depth_mm, row_count):
    points = [(0.0, "sisäreuna")]
    if projected_depth_mm > 1e-9:
        points.append((min(200.0, projected_depth_mm), "200 mm sisäreunasta"))
    row_depth_mm = projected_depth_mm / max(row_count, 1)
    for row_index in range(row_count):
        row_no = row_index + 1
        points.append(((row_index + 0.5) * row_depth_mm, f"{row_no}. rivin puoliväli"))
        edge_label = "ulkoreuna" if row_no == row_count else "rivisauma"
        points.append((row_no * row_depth_mm, edge_label))
    return dedupe_panel_checkpoints(points)


PANEL_COLUMN_SUMMARY = []
for panel_col_index in range(panel_count_x):
    x_center_mm = roof_x0_mm + (panel_col_index + 0.5) * roof_width_mm / max(panel_count_x, 1)
    h_local_m = local_wall_height_m_at_x(x_center_mm)
    ls_m, mu2, mu2_h, s_base_kNm2, s_peak_kNm2 = snow_drift_params(h_local_m, drift_depth_m, sk, mu1_=mu1)
    uls_inner_kNm2 = gammaQ * drift_snow_kNm2(x_center_mm, roof_y0_mm)
    PANEL_COLUMN_SUMMARY.append({
        "index": panel_col_index + 1,
        "x_center_mm": x_center_mm,
        "h_m": h_local_m,
        "ls_m": ls_m,
        "mu2": mu2,
        "mu2_h": mu2_h,
        "s_base_kNm2": s_base_kNm2,
        "s_peak_kNm2": s_peak_kNm2,
        "uls_inner_kNm2": uls_inner_kNm2,
        "eta_inner_pct": 100.0 * uls_inner_kNm2 / panel_front_snow_cap_kNm2,
    })
critical_panel_column = max(PANEL_COLUMN_SUMMARY, key=lambda item: (item["uls_inner_kNm2"], item["x_center_mm"]))
critical_panel_check_rows = []
for dy_mm, label in panel_depth_checkpoints(roof_depth_mm, panel_count_y):
    y_mm = roof_y0_mm + dy_mm
    s_char_kNm2 = drift_snow_kNm2(critical_panel_column["x_center_mm"], y_mm)
    uls_kNm2 = gammaQ * s_char_kNm2
    critical_panel_check_rows.append({
        "label": label,
        "y_mm": y_mm,
        "s_char_kNm2": s_char_kNm2,
        "uls_kNm2": uls_kNm2,
        "eta_pct": 100.0 * uls_kNm2 / panel_front_snow_cap_kNm2,
        "ok": uls_kNm2 <= panel_front_snow_cap_kNm2 + 1e-9,
    })
critical_panel_check = max(critical_panel_check_rows, key=lambda item: (item["uls_kNm2"], -item["y_mm"]))
critical_panel_ok = all(item["ok"] for item in critical_panel_check_rows)
panel_char_limit_kNm2 = panel_front_snow_cap_kNm2 / gammaQ
critical_panel_y_offset_mm = critical_panel_check["y_mm"] - drift_obstacle_y_mm
panel_height_limit_m = wall_height_limit_for_panel_capacity(panel_front_snow_cap_kNm2, critical_panel_y_offset_mm)
panel_height_margin_mm = None if panel_height_limit_m is None else 1000.0 * (critical_panel_column["h_m"] - panel_height_limit_m)
panel_char_margin_kNm2 = critical_panel_check["s_char_kNm2"] - panel_char_limit_kNm2
panel_uls_margin_kNm2 = critical_panel_check["uls_kNm2"] - panel_front_snow_cap_kNm2


# ── jäsenanalyysit ───────────────────────────────────────────────────────────

def make_end_referenced_bevel_notch_depth_fn(info, end_coord_mm, inward_positive_sign):
    if not info["active"]:
        def depth_fn(_coord_mm):
            return 0.0
        return (end_coord_mm, end_coord_mm), depth_fn, False

    if info["reference"] not in {"axis_start", "axis_end"}:
        raise ValueError(f"Unsupported bevel notch reference: {info['reference']}")

    offset_mm = info["offset_mm"]
    length_mm = info["length_mm"]
    zone = local_interval_to_global(end_coord_mm, offset_mm, length_mm, inward_positive_sign)

    def depth_fn(coord_mm):
        local_mm = inward_positive_sign * (coord_mm - end_coord_mm)
        if local_mm < offset_mm - 1e-9 or local_mm > offset_mm + length_mm + 1e-9:
            return 0.0
        if length_mm <= 1e-9:
            return info["depth_mm"]
        return info["depth_mm"] * (1.0 - (local_mm - offset_mm) / length_mm)

    return zone, depth_fn, True


def make_purlin_notch_depth_fn(member_obj, side):
    info = purlin_inner_notch_info_by_id[member_obj["id"]]
    if not info["active"]:
        def depth_fn(_x_mm):
            return 0.0
        edge_x_mm = float(member_obj["axis_start"]["x"])
        return (edge_x_mm, edge_x_mm), depth_fn, False

    start_x_mm = float(member_obj["axis_start"]["x"])
    end_x_mm = float(member_obj["axis_end"]["x"])
    member_axis_positive_sign = 1.0 if end_x_mm >= start_x_mm else -1.0
    if info["reference"] not in {"axis_start", "axis_end"}:
        raise ValueError(f"Unsupported bevel notch reference: {info['reference']}")
    notch_end_x_mm = start_x_mm if info["reference"] == "axis_start" else end_x_mm
    inward_positive_sign = member_axis_positive_sign if info["reference"] == "axis_start" else -member_axis_positive_sign
    return make_end_referenced_bevel_notch_depth_fn(info, notch_end_x_mm, inward_positive_sign)


def make_purlin_edge_notch_depth_fn(member_obj, side):
    member_id = member_obj["id"]
    info = purlin_edge_notch_info_by_id[member_id]
    support_x_mm = float(purlin_edge_support_center_points_by_id[member_id]["x"])
    if not info["active"]:
        def depth_fn(_x_mm):
            return 0.0
        return (support_x_mm, support_x_mm), depth_fn, False

    if info.get("kind") == "bevel_notch":
        start_x_mm = float(member_obj["axis_start"]["x"])
        end_x_mm = float(member_obj["axis_end"]["x"])
        member_axis_positive_sign = 1.0 if end_x_mm >= start_x_mm else -1.0
        edge_support_line_x_mm = float(purlin_edge_support_line_points_by_id[member_id]["x"])
        reference_positions_mm = {
            "axis_start": start_x_mm,
            "axis_end": end_x_mm,
            "support_centerline": support_x_mm,
            "support_outer_edge": edge_support_line_x_mm,
            "support_inner_edge": 2.0 * support_x_mm - edge_support_line_x_mm,
        }
        if info["reference"] not in reference_positions_mm:
            raise ValueError(f"Unsupported bevel notch reference: {info['reference']}")
        notch_end_x_mm = reference_positions_mm[info["reference"]]
        inward_positive_sign = member_axis_positive_sign if info["reference"] == "axis_start" else -member_axis_positive_sign
        return make_end_referenced_bevel_notch_depth_fn(info, notch_end_x_mm, inward_positive_sign)

    if info.get("kind") not in {None, "rect_notch"}:
        raise ValueError(f"Unsupported purlin edge notch kind: {info.get('kind')}")

    zone = tuple(sorted((support_x_mm + info["offset_mm"], support_x_mm + info["offset_mm"] + info["length_mm"])))

    def depth_fn(x_mm):
        if not zone[0] - 1e-9 <= x_mm <= zone[1] + 1e-9:
            return 0.0
        return info["depth_mm"]

    return zone, depth_fn, True


def make_slanted_support_notch_depth_fn(info, member_length_mm):
    if not info["active"]:
        def depth_fn(_s_mm):
            return 0.0
        return (0.0, 0.0), depth_fn, False

    if info["reference"] not in {"axis_start", "axis_end"}:
        raise ValueError(f"Unsupported bevel notch reference: {info['reference']}")
    notch_end_s_mm = 0.0 if info["reference"] == "axis_start" else member_length_mm
    inward_positive_sign = 1.0 if info["reference"] == "axis_start" else -1.0
    return make_end_referenced_bevel_notch_depth_fn(info, notch_end_s_mm, inward_positive_sign)


def make_birdsmouth_depth_fn():
    start_mm, end_mm = birdsmouth_zone_mm
    def depth_fn(y_mm):
        if not start_mm - 1e-9 <= y_mm <= end_mm + 1e-9:
            return 0.0
        return birdsmouth_start_depth_mm + (birdsmouth_heel_depth_mm - birdsmouth_start_depth_mm) * (y_mm - start_mm) / (end_mm - start_mm)
    return depth_fn


BIRDSMOUTH_DEPTH_FN = make_birdsmouth_depth_fn()


def make_edge_rect_depth_fn(side):
    info = edge_top_notch_info[side]
    if not info["active"]:
        def depth_fn(_y_mm):
            return 0.0
        return [], depth_fn, False

    zones_mm = [
        local_interval_to_global(y_mm, info["offset_mm"], info["length_mm"], local_positive_sign=-1.0)
        for y_mm in edge_purlin_support_ys_mm[side]
    ]

    def depth_fn(y_mm):
        for start_mm, end_mm in zones_mm:
            if start_mm - 1e-9 <= y_mm <= end_mm + 1e-9:
                return info["depth_mm"]
        return 0.0

    return zones_mm, depth_fn, True


RAfter_PROPS = member_rect_props(rafter_b_mm, rafter_h_mm, member_section_rotation_deg(interior_rafters[0]))
PURLIN_PROPS = member_rect_props(purlin_b_mm, purlin_h_mm, member_section_rotation_deg(left_purlins[0]))
INFILL_SUPPORT_PURLIN_PROPS = (
    member_rect_props(infill_support_purlin_b_mm, infill_support_purlin_h_mm, member_section_rotation_deg(infill_support_purlin))
    if infill_support_purlin is not None
    else None
)
OUTER_BEAM_PROPS = member_rect_props(outer_beam_b_mm, outer_beam_h_mm, member_section_rotation_deg(outer_beam))
INNER_BEAM_PROPS = member_rect_props(inner_beam_b_mm, inner_beam_h_mm, member_section_rotation_deg(inner_beam))
EXISTING_BEAM_PROPS = member_rect_props(existing_beam_b_mm, existing_beam_h_mm, member_section_rotation_deg(existing_beam))

RAfter_MRd_kNm = fm_d_c24 * RAfter_PROPS["W_mm3"] / 1.0e6
RAfter_VRd_kN = fv_d_c24 * RAfter_PROPS["A_mm2"] / 1.5e3
PURLIN_MRd_kNm = fm_d_c24 * PURLIN_PROPS["W_mm3"] / 1.0e6
PURLIN_VRd_kN = fv_d_c24 * PURLIN_PROPS["A_mm2"] / 1.5e3
INFILL_SUPPORT_PURLIN_MRd_kNm = (
    fm_d_c24 * INFILL_SUPPORT_PURLIN_PROPS["W_mm3"] / 1.0e6
    if INFILL_SUPPORT_PURLIN_PROPS is not None
    else 0.0
)
INFILL_SUPPORT_PURLIN_VRd_kN = (
    fv_d_c24 * INFILL_SUPPORT_PURLIN_PROPS["A_mm2"] / 1.5e3
    if INFILL_SUPPORT_PURLIN_PROPS is not None
    else 0.0
)
OUTER_BEAM_MRd_y_kNm = fm_d_gl30c * OUTER_BEAM_PROPS["W_mm3"] / 1.0e6
OUTER_BEAM_VRd_kN = fv_d_gl30c * OUTER_BEAM_PROPS["A_mm2"] / 1.5e3
OUTER_BEAM_MRd_z_kNm = fm_d_gl30c * OUTER_BEAM_PROPS["W_horizontal_mm3"] / 1.0e6
INNER_BEAM_MRd_kNm = fm_d_gl30c * INNER_BEAM_PROPS["W_mm3"] / 1.0e6
INNER_BEAM_MRd_z_kNm = fm_d_gl30c * INNER_BEAM_PROPS["W_horizontal_mm3"] / 1.0e6
INNER_BEAM_VRd_kN = fv_d_gl30c * INNER_BEAM_PROPS["A_mm2"] / 1.5e3
EXISTING_BEAM_MRd_z_kNm = fm_d_lvl * EXISTING_BEAM_PROPS["W_horizontal_mm3"] / 1.0e6
EXISTING_BEAM_VRd_kN = fv_d_lvl * EXISTING_BEAM_PROPS["A_mm2"] / 1.5e3

edge_rafter_section_by_side = {}
for side, member_obj in edge_rafters.items():
    b_mm = profile_b(member_obj)
    h_mm = profile_h(member_obj)
    props = member_rect_props(b_mm, h_mm, member_section_rotation_deg(member_obj))
    edge_rafter_section_by_side[side] = {
        "profile": member_obj["profile"]["name"],
        "b_mm": b_mm,
        "h_mm": h_mm,
        "section_rotation_deg": props["section_rotation_deg"],
        "self_kNm": (b_mm / 1000.0) * (h_mm / 1000.0) * gamma_gl30c / math.cos(roof_slope_rad),
        "MRd_kNm": fm_d_gl30c * props["W_mm3"] / 1.0e6,
        "VRd_kN": fv_d_gl30c * props["A_mm2"] / 1.5e3,
    }

left_purlin_edge_support_line_x_mm = float(purlin_edge_support_line_points_by_id[left_purlins[0]["id"]]["x"])
right_purlin_edge_support_line_x_mm = float(purlin_edge_support_line_points_by_id[right_purlins[0]["id"]]["x"])


analysis_step_member_mm = 100.0
analysis_step_beam_mm = 150.0


def roof_area_uniform_loads(nodes_mm, fixed_coord_mm, roof_area_kNm2_at, trib_width_m, axis):
    loads = []
    for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:]):
        span_mid_mm = 0.5 * (a_mm + b_mm)
        if axis == "x":
            x_mm, y_mm = span_mid_mm, fixed_coord_mm
        else:
            x_mm, y_mm = fixed_coord_mm, span_mid_mm
        roof_area_kNm2 = roof_area_kNm2_at(x_mm, y_mm)
        q_line_kNm = roof_area_kNm2 * trib_width_m
        loads.append((a_mm, b_mm, q_line_kNm / 1000.0))
    return loads


def roof_area_uniform_loads_on_member(nodes_mm, member_obj, roof_area_kNm2_at, trib_width_m):
    start = member_obj["axis_start"]
    dx_mm, dy_mm, _ = member_axis_vector_3d(member_obj)
    length_mm = member_axis_length_mm(member_obj)
    loads = []
    for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:]):
        span_mid_mm = 0.5 * (a_mm + b_mm)
        t = 0.0 if length_mm <= 1e-9 else span_mid_mm / length_mm
        x_mm = float(start["x"]) + t * dx_mm
        y_mm = float(start["y"]) + t * dy_mm
        q_line_kNm = roof_area_kNm2_at(x_mm, y_mm) * trib_width_m
        loads.append((a_mm, b_mm, q_line_kNm / 1000.0))
    return loads


def member_uniform_loads(nodes_mm, fixed_coord_mm, roof_area_kNm2_at, trib_width_m, gamma_self, self_kNm, axis):
    roof_loads = roof_area_uniform_loads(nodes_mm, fixed_coord_mm, roof_area_kNm2_at, trib_width_m, axis)
    self_loads = uniform_loads_for_nodes(nodes_mm, gamma_self * self_kNm / 1000.0)
    loads = combine_uniform_loads(roof_loads, self_loads)
    return loads, load_stats(loads)


def load_application_interval_from_rule(load_rule, reference_positions_mm, local_axis_positive_sign, coord_min_mm, coord_max_mm):
    reference_mm = reference_positions_mm[load_rule["reference"]]
    start_mm = clamp(reference_mm + local_axis_positive_sign * float(load_rule.get("offset_mm", 0.0)), coord_min_mm, coord_max_mm)
    if load_rule["model"] != "partial_uniform":
        return start_mm, start_mm
    end_mm = clamp(start_mm + local_axis_positive_sign * float(load_rule["length_mm"]), coord_min_mm, coord_max_mm)
    return start_mm, end_mm


def roof_load_application_from_rule(roof_uniform_loads, load_rule, reference_positions_mm, local_axis_positive_sign, coord_min_mm, coord_max_mm):
    model = load_rule["model"]
    if model == "uniform":
        return [], roof_uniform_loads, "uniform"

    start_mm, end_mm = load_application_interval_from_rule(
        load_rule,
        reference_positions_mm,
        local_axis_positive_sign,
        coord_min_mm,
        coord_max_mm,
    )
    total_roof_load_kN = total_uniform_load_kN(roof_uniform_loads)

    if model == "point":
        return [(start_mm, total_roof_load_kN)], [], "point"

    if model == "partial_uniform":
        a_mm, b_mm = sorted((start_mm, end_mm))
        load_length_mm = max(1e-9, b_mm - a_mm)
        return [], [(a_mm, b_mm, total_roof_load_kN / load_length_mm)], "partial_uniform"

    raise ValueError(f"Unsupported load transfer model: {model}")


def analyse_purlin_case(side, index, trib_height_m, roof_area_kNm2_at, gamma_self):
    member_obj = left_purlins[index] if side == "left" else right_purlins[index]
    member_id = member_obj["id"]
    axis_start_x_mm = float(member_obj["axis_start"]["x"])
    axis_end_x_mm = float(member_obj["axis_end"]["x"])
    x0_mm = min(float(member_obj["axis_start"]["x"]), float(member_obj["axis_end"]["x"]))
    x1_mm = max(float(member_obj["axis_start"]["x"]), float(member_obj["axis_end"]["x"]))
    member_y_mm = float(member_obj["axis_start"]["y"])
    member_props = member_rect_props(purlin_b_mm, purlin_h_mm, member_section_rotation_deg(member_obj))
    member_MRd_kNm = fm_d_c24 * member_props["W_mm3"] / 1.0e6
    member_VRd_kN = fv_d_c24 * member_props["A_mm2"] / 1.5e3
    inner_support_point = purlin_inner_support_points_by_id[member_id]
    edge_support_center_point = purlin_edge_support_center_points_by_id[member_id]
    edge_support_line_point = purlin_edge_support_line_points_by_id[member_id]
    interior_support_x_mm = float(inner_support_point["x"])
    edge_support_x_mm = float(edge_support_line_point["x"])
    support_center_x_mm = float(edge_support_center_point["x"])
    if side == "left":
        supports = [edge_support_x_mm, interior_support_x_mm]
    else:
        supports = [interior_support_x_mm, edge_support_x_mm]

    inner_notch_zone_mm, inner_notch_depth_fn, inner_notch_active = make_purlin_notch_depth_fn(member_obj, side)
    edge_notch_zone_mm, edge_notch_depth_fn, edge_notch_active = make_purlin_edge_notch_depth_fn(member_obj, side)
    depth_functions = [inner_notch_depth_fn] if inner_notch_active else []
    if edge_notch_active:
        depth_functions.append(edge_notch_depth_fn)
    section_h_fn = combined_section_h(purlin_h_mm, depth_functions)

    outer_edge_x_mm = x0_mm if side == "left" else x1_mm
    roof_load_rule = roof_load_transfer_rule(member_obj["id"])
    roof_reference_positions_mm = {
        "axis_start": axis_start_x_mm,
        "axis_end": axis_end_x_mm,
        "support_centerline": support_center_x_mm,
        "support_outer_edge": edge_support_x_mm,
        "support_inner_edge": 2.0 * support_center_x_mm - edge_support_x_mm,
    }
    local_axis_positive_sign = 1.0 if axis_end_x_mm >= axis_start_x_mm else -1.0
    node_points = [x0_mm, x1_mm, *supports, *inner_notch_zone_mm]
    if edge_notch_active:
        node_points.extend(edge_notch_zone_mm)
    if roof_load_rule["model"] != "uniform":
        roof_load_start_mm, roof_load_end_mm = load_application_interval_from_rule(
            roof_load_rule,
            roof_reference_positions_mm,
            local_axis_positive_sign,
            x0_mm,
            x1_mm,
        )
        node_points.extend([roof_load_start_mm, roof_load_end_mm])
    nodes_mm = refine_nodes_mm(node_points, analysis_step_member_mm)
    member_axis_width_mm = max(1.0e-9, x1_mm - x0_mm)
    load_tributary_width_mm = min(
        member_axis_width_mm,
        float(roof_load_rule.get("tributary_width_mm", member_axis_width_mm)),
    )
    roof_load_width_factor = load_tributary_width_mm / member_axis_width_mm
    roof_uniform = roof_area_uniform_loads(nodes_mm, member_y_mm, roof_area_kNm2_at, trib_height_m * roof_load_width_factor, axis="x")
    self_uniform = uniform_loads_for_nodes(nodes_mm, gamma_self * purlin_self_kNm / 1000.0)
    q_line_stats = load_stats(combine_uniform_loads(roof_uniform, self_uniform))
    point_loads, roof_applied_uniform, panel_load_mode = roof_load_application_from_rule(
        roof_uniform,
        roof_load_rule,
        roof_reference_positions_mm,
        local_axis_positive_sign,
        x0_mm,
        x1_mm,
    )
    uniform = combine_uniform_loads(self_uniform, roof_applied_uniform)
    panel_point_load_kN = sum(load_kN for _, load_kN in point_loads)
    EI_by_segment = [
        E_c24 * member_rect_props(purlin_b_mm, section_h_fn(0.5 * (a_mm + b_mm)), member_props["section_rotation_deg"])["I_mm4"]
        for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:])
    ]

    response, internal, delta = solve_member_response(
        nodes_mm,
        supports,
        point_loads,
        uniform,
        EI_by_segment_Nmm2=EI_by_segment,
    )
    moment_gov = governing_moment(internal)
    inner_notch = None
    edge_notch = None
    notch_candidates = []
    if inner_notch_active:
        inner_notch = sample_net_section_utilization(
            response["elements"],
            member_obj=member_obj,
            section_h_mm_at_x=section_h_fn,
            fm_d_Nmm2=fm_d_c24,
            fv_d_Nmm2=fv_d_c24,
            x_start_mm=inner_notch_zone_mm[0],
            x_end_mm=inner_notch_zone_mm[1],
            step_mm=1.0,
        )
        notch_candidates.append({"label": bevel_notch_label(purlin_inner_notch_info_by_id[member_id]), **inner_notch})
    if edge_notch_active:
        edge_notch = sample_net_section_utilization(
            response["elements"],
            member_obj=member_obj,
            section_h_mm_at_x=section_h_fn,
            fm_d_Nmm2=fm_d_c24,
            fv_d_Nmm2=fv_d_c24,
            x_start_mm=edge_notch_zone_mm[0],
            x_end_mm=edge_notch_zone_mm[1],
            step_mm=1.0,
        )
        edge_label = "edge_rect" if purlin_edge_notch_info_by_id[member_id].get("kind") == "rect_notch" else bevel_notch_label(purlin_edge_notch_info_by_id[member_id])
        notch_candidates.append({"label": edge_label, **edge_notch})
    notch = max(notch_candidates, key=lambda item: item["eta_gov"]["value_pct"])

    return {
        "id": member_obj["id"],
        "trib_height_m": trib_height_m,
        "q_line_kNm": q_line_stats["avg_kNm"],
        "q_line_min_kNm": q_line_stats["min_kNm"],
        "q_line_max_kNm": q_line_stats["max_kNm"],
        "panel_load_mode": panel_load_mode,
        "panel_point_load_kN": panel_point_load_kN,
        "load_tributary_width_m": load_tributary_width_mm / 1000.0,
        "M_gov": moment_gov,
        "V_abs": internal["V_abs"],
        "delta": delta,
        "delta_lim_mm": abs(interior_support_x_mm - edge_support_x_mm) / 300.0,
        "section_rotation_deg": member_props["section_rotation_deg"],
        "MRd_kNm": member_MRd_kNm,
        "VRd_kN": member_VRd_kN,
        "R_edge_kN": response["reactions_kN"][edge_support_x_mm],
        "R_inner_kN": response["reactions_kN"][interior_support_x_mm],
        "support_inner_x_mm": interior_support_x_mm,
        "support_inner_y_mm": float(inner_support_point["y"]),
        "support_edge_x_mm": edge_support_x_mm,
        "support_edge_y_mm": float(edge_support_line_point["y"]),
        "eta_M": moment_gov["value_kNm"] / member_MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / member_VRd_kN * 100.0,
        "notch": notch,
        "inner_notch": inner_notch,
        "edge_notch": edge_notch,
        "notch_zones_mm": {"inner": inner_notch_zone_mm, "edge": edge_notch_zone_mm if edge_notch_active else None},
        "h_net_min_mm": purlin_h_mm - max(purlin_inner_notch_info_by_id[member_id]["depth_mm"], purlin_edge_notch_info_by_id[member_id]["depth_mm"]),
    }


def analyse_corner_purlin_case(side, member_obj, trib_width_m, roof_area_kNm2_at, gamma_self):
    member_id = member_obj["id"]
    member_length_mm = member_axis_length_mm(member_obj)
    member_props = member_rect_props(purlin_b_mm, purlin_h_mm, member_section_rotation_deg(member_obj))
    member_MRd_kNm = fm_d_c24 * member_props["W_mm3"] / 1.0e6
    member_VRd_kN = fv_d_c24 * member_props["A_mm2"] / 1.5e3
    inner_conn = corner_purlin_inner_support_connections[member_id]
    outer_conn = corner_purlin_outer_support_connections[member_id]
    support_inner_point = corner_purlin_inner_support_points[member_id]
    support_outer_point = corner_purlin_outer_support_points[member_id]
    support_inner_s_mm = project_point_to_member_s_mm(member_obj, support_inner_point)
    support_outer_s_mm = project_point_to_member_s_mm(member_obj, support_outer_point)
    support_outer_center_s_mm = project_point_to_member_s_mm(
        member_obj,
        connection_support_point(outer_conn, member_id, "support_centerline"),
    )
    support_outer_outer_edge_s_mm = project_point_to_member_s_mm(
        member_obj,
        connection_support_point(outer_conn, member_id, "support_outer_edge"),
    )
    support_outer_inner_edge_s_mm = project_point_to_member_s_mm(
        member_obj,
        connection_support_point(outer_conn, member_id, "support_inner_edge"),
    )
    outer_free_end_s_mm = 0.0 if support_inner_s_mm >= member_length_mm - support_inner_s_mm else member_length_mm
    inner_notch_info = dict(inactive_bevel_notch_info) if inner_conn is None else bevel_notch_info(inner_conn["id"])
    inner_notch_zone_mm, inner_notch_depth_fn, inner_notch_active = make_slanted_support_notch_depth_fn(inner_notch_info, member_length_mm)
    outer_notch_zone_mm, outer_notch_depth_fn, outer_notch_active = make_slanted_support_notch_depth_fn(
        corner_purlin_outer_notch_info[member_id],
        member_length_mm,
    )
    depth_functions = []
    if inner_notch_active:
        depth_functions.append(inner_notch_depth_fn)
    if outer_notch_active:
        depth_functions.append(outer_notch_depth_fn)
    section_h_fn = combined_section_h(purlin_h_mm, depth_functions)
    roof_load_rule = roof_load_transfer_rule(member_id)
    roof_reference_positions_mm = {
        "axis_start": 0.0,
        "axis_end": member_length_mm,
        "support_centerline": support_outer_center_s_mm,
        "support_outer_edge": support_outer_outer_edge_s_mm,
        "support_inner_edge": support_outer_inner_edge_s_mm,
    }
    local_axis_positive_sign = 1.0
    node_points = [0.0, member_length_mm, support_inner_s_mm, support_outer_s_mm]
    if inner_notch_active:
        node_points.extend(inner_notch_zone_mm)
    if outer_notch_active:
        node_points.extend(outer_notch_zone_mm)
    if roof_load_rule["model"] != "uniform":
        roof_load_start_s_mm, roof_load_end_s_mm = load_application_interval_from_rule(
            roof_load_rule,
            roof_reference_positions_mm,
            local_axis_positive_sign,
            0.0,
            member_length_mm,
        )
        node_points.extend([roof_load_start_s_mm, roof_load_end_s_mm])
    nodes_mm = refine_nodes_mm(node_points, analysis_step_member_mm)
    roof_uniform = roof_area_uniform_loads_on_member(nodes_mm, member_obj, roof_area_kNm2_at, trib_width_m)
    self_uniform = uniform_loads_for_nodes(nodes_mm, gamma_self * purlin_self_kNm / 1000.0)
    q_line_stats = load_stats(combine_uniform_loads(roof_uniform, self_uniform))
    point_loads, roof_applied_uniform, panel_load_mode = roof_load_application_from_rule(
        roof_uniform,
        roof_load_rule,
        roof_reference_positions_mm,
        local_axis_positive_sign,
        0.0,
        member_length_mm,
    )
    uniform = combine_uniform_loads(self_uniform, roof_applied_uniform)
    response, internal, delta = solve_member_response(
        nodes_mm,
        [support_inner_s_mm, support_outer_s_mm],
        point_loads,
        uniform,
        EI_by_segment_Nmm2=[
            E_c24 * member_rect_props(purlin_b_mm, section_h_fn(0.5 * (a_mm + b_mm)), member_props["section_rotation_deg"])["I_mm4"]
            for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:])
        ],
    )
    moment_gov = governing_moment(internal)
    reactions = response["reactions_kN"]
    inner_notch = None
    outer_notch = None
    notch_candidates = []
    if inner_notch_active:
        inner_notch = sample_net_section_utilization(
            response["elements"],
            member_obj=member_obj,
            section_h_mm_at_x=section_h_fn,
            fm_d_Nmm2=fm_d_c24,
            fv_d_Nmm2=fv_d_c24,
            x_start_mm=inner_notch_zone_mm[0],
            x_end_mm=inner_notch_zone_mm[1],
            step_mm=1.0,
        )
        notch_candidates.append({"label": bevel_notch_label(inner_notch_info), **inner_notch})
    if outer_notch_active:
        outer_notch = sample_net_section_utilization(
            response["elements"],
            member_obj=member_obj,
            section_h_mm_at_x=section_h_fn,
            fm_d_Nmm2=fm_d_c24,
            fv_d_Nmm2=fv_d_c24,
            x_start_mm=outer_notch_zone_mm[0],
            x_end_mm=outer_notch_zone_mm[1],
            step_mm=1.0,
        )
        notch_candidates.append({"label": bevel_notch_label(corner_purlin_outer_notch_info[member_id]), **outer_notch})
    notch = max(notch_candidates, key=lambda item: item["eta_gov"]["value_pct"]) if notch_candidates else None
    max_notch_depth_mm = max(
        ([inner_notch_info["depth_mm"]] if inner_notch_active else [])
        + ([corner_purlin_outer_notch_info[member_id]["depth_mm"]] if outer_notch_active else [])
        + [0.0]
    )
    return {
        "id": member_id,
        "side": side,
        "trib_width_m": trib_width_m,
        "q_line_kNm": q_line_stats["avg_kNm"],
        "q_line_min_kNm": q_line_stats["min_kNm"],
        "q_line_max_kNm": q_line_stats["max_kNm"],
        "panel_load_mode": panel_load_mode,
        "panel_point_load_kN": sum(load_kN for _, load_kN in point_loads),
        "M_gov": moment_gov,
        "V_abs": internal["V_abs"],
        "delta": delta,
        "delta_lim_mm": abs(outer_free_end_s_mm - support_outer_s_mm) / 300.0,
        "support_inner_s_mm": support_inner_s_mm,
        "support_outer_s_mm": support_outer_s_mm,
        "support_inner_y_mm": float(support_inner_point["y"]),
        "support_outer_y_mm": float(support_outer_point["y"]),
        "support_outer_x_mm": float(support_outer_point["x"]),
        "section_rotation_deg": member_props["section_rotation_deg"],
        "MRd_kNm": member_MRd_kNm,
        "VRd_kN": member_VRd_kN,
        "inner_support_member_id": corner_purlin_inner_support_member_ids[member_id],
        "outer_support_member_id": corner_purlin_outer_support_member_ids[member_id],
        "outer_support_label": corner_purlin_outer_support_member_ids[member_id],
        "R_inner_kN": reactions[support_inner_s_mm],
        "R_outer_kN": reactions[support_outer_s_mm],
        "R_outer_beam_kN": reactions[support_outer_s_mm],
        "eta_M": moment_gov["value_kNm"] / member_MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / member_VRd_kN * 100.0,
        "notch": notch,
        "inner_notch": inner_notch,
        "outer_notch": outer_notch,
        "notch_zones_mm": {
            "inner": inner_notch_zone_mm if inner_notch_active else None,
            "outer": outer_notch_zone_mm if outer_notch_active else None,
        },
        "h_net_min_mm": purlin_h_mm - max_notch_depth_mm,
    }


def analyse_rafter_case(member_obj, direct_width_m, roof_area_kNm2_at, gamma_self, point_loads_kN, edge=False):
    member_x_mm = float(member_obj["axis_start"]["x"])
    y_end_mm = float(member_obj["axis_end"]["y"])
    inner_support_y_mm = edge_inner_support_y_mm if edge else interior_inner_support_y_mm
    outer_support_y_mm = edge_outer_support_y_mm if edge else interior_outer_support_y_mm
    section_rotation = member_section_rotation_deg(member_obj)
    member_E_Nmm2 = E_gl30c if edge else E_c24
    member_fm_d_Nmm2 = fm_d_gl30c if edge else fm_d_c24
    member_fv_d_Nmm2 = fv_d_gl30c if edge else fv_d_c24
    if edge:
        side = "left" if member_obj["id"] == edge_rafter_id("left") else "right"
        edge_section = edge_rafter_section_by_side[side]
        member_b_mm = edge_section["b_mm"]
        member_h_mm = edge_section["h_mm"]
        member_MRd_kNm = edge_section["MRd_kNm"]
        member_VRd_kN = edge_section["VRd_kN"]
        member_self_kNm = edge_section["self_kNm"]
    else:
        member_b_mm = rafter_b_mm
        member_h_mm = rafter_h_mm
        member_MRd_kNm = RAfter_MRd_kNm
        member_VRd_kN = RAfter_VRd_kN
        member_self_kNm = rafter_self_kNm
    section_depth_functions = []
    birdsmouth_ranges = []
    top_notch_ranges = []
    top_notch_active = False
    if edge:
        top_notch_ranges, top_notch_depth_fn, top_notch_active = make_edge_rect_depth_fn(side)
        if top_notch_active:
            section_depth_functions.append(top_notch_depth_fn)
    else:
        section_depth_functions.append(BIRDSMOUTH_DEPTH_FN)
        birdsmouth_ranges = [birdsmouth_zone_mm]

    section_h_fn = combined_section_h(member_h_mm, section_depth_functions)
    point_positions_mm = [y_mm for y_mm, _ in point_loads_kN]
    nodes_mm = refine_nodes_mm(
        [rafter_analysis_start_y_mm, y_end_mm, inner_support_y_mm, outer_support_y_mm, *[v for rng in birdsmouth_ranges for v in rng], *point_positions_mm, *[v for rng in top_notch_ranges for v in rng]],
        analysis_step_member_mm,
    )
    uniform, q_line_stats = member_uniform_loads(nodes_mm, member_x_mm, roof_area_kNm2_at, direct_width_m, gamma_self, member_self_kNm, axis="y")
    EI_by_segment = [
        member_E_Nmm2 * member_rect_props(member_b_mm, section_h_fn(0.5 * (a_mm + b_mm)), section_rotation)["I_mm4"]
        for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:])
    ]
    if edge:
        rotational_springs = {}
        edge_rot_k = edge_rafter_support_rot_k_by_side[side]
        if edge_rot_k is not None:
            rotational_springs[inner_support_y_mm] = edge_rot_k
            rotational_springs[outer_support_y_mm] = edge_rot_k
    else:
        rotational_springs = {}
        if inner_hanger_rot_k_Nmm_per_rad is not None:
            rotational_springs[inner_support_y_mm] = inner_hanger_rot_k_Nmm_per_rad
    response, internal, delta = solve_member_response(
        nodes_mm,
        [inner_support_y_mm, outer_support_y_mm],
        point_loads_kN,
        uniform,
        EI_by_segment_Nmm2=EI_by_segment,
        rotational_springs_Nmm_per_rad=rotational_springs,
    )
    moment_gov = governing_moment(internal)

    birdsmouth = None
    if birdsmouth_ranges:
        birdsmouth = sample_net_section_utilization(
            response["elements"],
            member_obj=member_obj,
            section_h_mm_at_x=section_h_fn,
            fm_d_Nmm2=member_fm_d_Nmm2,
            fv_d_Nmm2=member_fv_d_Nmm2,
            x_start_mm=birdsmouth_zone_mm[0],
            x_end_mm=birdsmouth_zone_mm[1],
            step_mm=1.0,
        )
    top_notch_gov = None
    if top_notch_active:
        top_checks = [
            sample_net_section_utilization(
                response["elements"],
                member_obj=member_obj,
                section_h_mm_at_x=section_h_fn,
                fm_d_Nmm2=member_fm_d_Nmm2,
                fv_d_Nmm2=member_fv_d_Nmm2,
                x_start_mm=rng[0],
                x_end_mm=rng[1],
                step_mm=1.0,
            )
            for rng in top_notch_ranges
        ]
        top_notch_gov = max(top_checks, key=lambda item: item["eta_gov"]["value_pct"])

    notch_candidates = []
    if birdsmouth is not None:
        notch_candidates.append({"label": "birdsmouth", **birdsmouth})
    if top_notch_gov is not None:
        notch_candidates.append({"label": "rect_top", **top_notch_gov})
    governing_notch = max(notch_candidates, key=lambda item: item["eta_gov"]["value_pct"]) if notch_candidates else None

    return {
        "id": member_obj["id"],
        "direct_width_m": direct_width_m,
        "q_line_kNm": q_line_stats["avg_kNm"],
        "q_line_min_kNm": q_line_stats["min_kNm"],
        "q_line_max_kNm": q_line_stats["max_kNm"],
        "point_loads_kN": point_loads_kN,
        "profile": member_obj["profile"]["name"],
        "M_gov": moment_gov,
        "V_abs": internal["V_abs"],
        "delta": delta,
        "delta_lim_mm": abs(outer_support_y_mm - inner_support_y_mm) / 300.0,
        "section_rotation_deg": section_rotation,
        "R_inner_kN": response["reactions_kN"][inner_support_y_mm],
        "R_outer_kN": response["reactions_kN"][outer_support_y_mm],
        "MRd_kNm": member_MRd_kNm,
        "VRd_kN": member_VRd_kN,
        "g_self_d_kNm": gammaG * member_self_kNm,
        "eta_M": moment_gov["value_kNm"] / member_MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / member_VRd_kN * 100.0,
        "birdsmouth": birdsmouth,
        "top_notch": top_notch_gov,
        "governing_notch": governing_notch,
        "h_net_birdsmouth_min_mm": member_h_mm - birdsmouth_heel_depth_mm if birdsmouth is not None else member_h_mm,
        "birdsmouth_rule_ok": None if birdsmouth is None else birdsmouth_heel_depth_mm <= member_h_mm / 3.0,
        "section_h_fn": section_h_fn,
        "response": response,
    }


def analyse_beam_case(member_obj, support_xs_mm, point_loads_kN, gamma_self, E_Nmm2, section_I_mm4, MRd_kNm, VRd_kN, extra_uniform_intervals=None):
    x0_mm = float(member_obj["axis_start"]["x"])
    x1_mm = float(member_obj["axis_end"]["x"])
    if member_obj["id"] == "beam.outer":
        self_kNm = outer_beam_self_kNm
    else:
        self_kNm = inner_beam_self_kNm
    extra_uniform_intervals = [] if extra_uniform_intervals is None else list(extra_uniform_intervals)
    nodes_mm = refine_nodes_mm(
        [
            x0_mm,
            x1_mm,
            *support_xs_mm,
            *[x_mm for x_mm, _ in point_loads_kN],
            *[x_mm for interval in extra_uniform_intervals for x_mm in interval[:2]],
        ],
        analysis_step_beam_mm,
    )
    uniform = uniform_loads_for_nodes(nodes_mm, gamma_self * self_kNm / 1000.0)
    if extra_uniform_intervals:
        uniform = combine_uniform_loads(uniform, intervals_to_uniform_loads(nodes_mm, extra_uniform_intervals))
    response, internal, delta = solve_member_response(nodes_mm, support_xs_mm, point_loads_kN, uniform, EI_Nmm2=E_Nmm2 * section_I_mm4)
    moment_gov = governing_moment(internal)
    return {
        "reactions_kN": response["reactions_kN"],
        "M_pos": internal["M_pos"],
        "M_neg": internal["M_neg"],
        "M_gov": moment_gov,
        "V_abs": internal["V_abs"],
        "delta": delta,
        "eta_M": moment_gov["value_kNm"] / MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / VRd_kN * 100.0,
    }


def analyse_outer_beam_horizontal(q_line_kNm):
    x0_mm = float(outer_beam["axis_start"]["x"])
    x1_mm = float(outer_beam["axis_end"]["x"])
    interval_points_mm = [x0_mm, x1_mm, *outer_supports_x_mm]
    interval_loads = []
    for start_mm, end_mm in outer_glazing_intervals:
        interval_points_mm.extend([start_mm, end_mm])
        interval_loads.append((start_mm, end_mm, q_line_kNm / 1000.0))
    nodes_mm = refine_nodes_mm(interval_points_mm, analysis_step_beam_mm)
    uniform = intervals_to_uniform_loads(nodes_mm, interval_loads)
    response, internal, _ = solve_member_response(nodes_mm, outer_supports_x_mm, [], uniform, EI_Nmm2=E_gl30c * OUTER_BEAM_PROPS["I_horizontal_mm4"])
    moment_gov = governing_moment(internal)
    return {
        "M_pos": internal["M_pos"],
        "M_neg": internal["M_neg"],
        "M_gov": moment_gov,
        "reactions_kN": response["reactions_kN"],
        "eta_M": moment_gov["value_kNm"] / OUTER_BEAM_MRd_z_kNm * 100.0,
    }


def analyse_beam_horizontal_interval(member_obj, support_xs_mm, interval_x0_mm, interval_x1_mm, q_line_kNm, E_Nmm2, section_I_mm4, MRd_kNm, VRd_kN):
    x0_mm = float(member_obj["axis_start"]["x"])
    x1_mm = float(member_obj["axis_end"]["x"])
    nodes_mm = refine_nodes_mm([x0_mm, x1_mm, *support_xs_mm, interval_x0_mm, interval_x1_mm], analysis_step_beam_mm)
    uniform = intervals_to_uniform_loads(nodes_mm, [(interval_x0_mm, interval_x1_mm, q_line_kNm / 1000.0)])
    response, internal, delta = solve_member_response(nodes_mm, support_xs_mm, [], uniform, EI_Nmm2=E_Nmm2 * section_I_mm4)
    moment_gov = governing_moment(internal)
    return {
        "q_line_kNm": q_line_kNm,
        "reactions_kN": response["reactions_kN"],
        "M_pos": internal["M_pos"],
        "M_neg": internal["M_neg"],
        "M_gov": moment_gov,
        "V_abs": internal["V_abs"],
        "delta": delta,
        "eta_M": moment_gov["value_kNm"] / MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / VRd_kN * 100.0,
    }


def analyse_infill_support_purlin_case(case_key):
    if infill_glass_summary is None:
        return None
    case = CASE_DEFS[case_key]
    member_obj = infill_support_purlin
    member_props = INFILL_SUPPORT_PURLIN_PROPS
    loads = infill_support_reactions_for_case(case_key)
    x0_mm = float(member_obj["axis_start"]["x"])
    x1_mm = float(member_obj["axis_end"]["x"])
    load_x0_mm = max(min(x0_mm, x1_mm), infill_bounds["x0_mm"])
    load_x1_mm = min(max(x0_mm, x1_mm), infill_bounds["x1_mm"])
    nodes_mm = refine_nodes_mm(
        [x0_mm, x1_mm, load_x0_mm, load_x1_mm, *infill_support_purlin_support_xs_mm],
        analysis_step_beam_mm,
    )
    glass_uniform = intervals_to_uniform_loads(
        nodes_mm,
        [(load_x0_mm, load_x1_mm, loads["outer_reaction_kN_per_m"] / 1000.0)],
    )
    self_uniform = intervals_to_uniform_loads(
        nodes_mm,
        [(min(x0_mm, x1_mm), max(x0_mm, x1_mm), case["gamma_self"] * infill_support_purlin_self_kNm / 1000.0)],
    )
    uniform = combine_uniform_loads(glass_uniform, self_uniform)
    response, internal, delta = solve_member_response(
        nodes_mm,
        infill_support_purlin_support_xs_mm,
        [],
        uniform,
        EI_Nmm2=E_c24 * member_props["I_mm4"],
    )
    moment_gov = governing_moment(internal)
    support_rows = []
    for row in infill_support_purlin_support_rows:
        support_rows.append({
            **row,
            "R_kN": response["reactions_kN"][row["x_mm"]],
        })
    span_points = [x0_mm, *infill_support_purlin_support_xs_mm, x1_mm]
    max_span_mm = max(abs(b_mm - a_mm) for a_mm, b_mm in zip(span_points, span_points[1:]))
    return {
        "id": member_obj["id"],
        "profile": member_obj["profile"]["name"],
        "support_y_mm": infill_support_purlin_y_mm,
        "q_glass_line_kNm": loads["outer_reaction_kN_per_m"],
        "q_total_line_kNm": load_stats(uniform)["avg_kNm"],
        "support_rows": support_rows,
        "M_gov": moment_gov,
        "V_abs": internal["V_abs"],
        "delta": delta,
        "delta_lim_mm": max_span_mm / 300.0,
        "eta_M": moment_gov["value_kNm"] / INFILL_SUPPORT_PURLIN_MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / INFILL_SUPPORT_PURLIN_VRd_kN * 100.0,
    }


def analyse_case(case_key):
    case = CASE_DEFS[case_key]
    roof_area_kNm2_at = case["roof_area_kNm2_at"]
    gamma_self = case["gamma_self"]

    purlins = {"left": [], "right": []}
    for side, group in (("left", left_purlins), ("right", right_purlins)):
        for i, _ in enumerate(group):
            purlins[side].append(analyse_purlin_case(side, i, purlin_trib_heights_m[side][i], roof_area_kNm2_at, gamma_self))

    corner_purlin_results = {"left": [], "right": []}
    for side, members in corner_purlins_by_side.items():
        for member_obj in members:
            member_id = member_obj["id"]
            corner_purlin_results[side].append(
                analyse_corner_purlin_case(side, member_obj, corner_purlin_trib_width_m[member_id], roof_area_kNm2_at, gamma_self)
            )
    corner_purlin_results = {side: rows for side, rows in corner_purlin_results.items() if rows}
    infill_support_purlin_result = analyse_infill_support_purlin_case(case_key)

    left_inner_point_loads = [(item["support_inner_y_mm"], item["R_inner_kN"]) for item in purlins["left"]]
    right_inner_point_loads = [(item["support_inner_y_mm"], item["R_inner_kN"]) for item in purlins["right"]]
    left_edge_point_loads = [(item["support_edge_y_mm"], item["R_edge_kN"]) for item in purlins["left"]]
    right_edge_point_loads = [(item["support_edge_y_mm"], item["R_edge_kN"]) for item in purlins["right"]]
    interior_rafter_point_loads = {rafter_obj["id"]: [] for rafter_obj in interior_rafters}
    interior_rafter_point_loads[interior_rafters[0]["id"]].extend(left_inner_point_loads)
    interior_rafter_point_loads[interior_rafters[-1]["id"]].extend(right_inner_point_loads)
    edge_rafter_point_loads = {"left": left_edge_point_loads, "right": right_edge_point_loads}
    for side, rows in corner_purlin_results.items():
        for corner_result in rows:
            target_member_id = corner_result["inner_support_member_id"]
            if target_member_id in interior_rafter_point_loads:
                interior_rafter_point_loads[target_member_id].append((corner_result["support_inner_y_mm"], corner_result["R_inner_kN"]))
            elif target_member_id == edge_rafters["left"]["id"]:
                edge_rafter_point_loads["left"].append((corner_result["support_inner_y_mm"], corner_result["R_inner_kN"]))
            elif target_member_id == edge_rafters["right"]["id"]:
                edge_rafter_point_loads["right"].append((corner_result["support_inner_y_mm"], corner_result["R_inner_kN"]))
            else:
                raise ValueError(f"Unsupported corner purlin inner support target for {corner_result['id']}: {target_member_id}")

            outer_target_member_id = corner_result["outer_support_member_id"]
            if outer_target_member_id == "beam.outer":
                pass
            elif outer_target_member_id == edge_rafters["left"]["id"]:
                edge_rafter_point_loads["left"].append((corner_result["support_outer_y_mm"], corner_result["R_outer_kN"]))
            elif outer_target_member_id == edge_rafters["right"]["id"]:
                edge_rafter_point_loads["right"].append((corner_result["support_outer_y_mm"], corner_result["R_outer_kN"]))
            else:
                raise ValueError(f"Unsupported corner purlin outer support target for {corner_result['id']}: {outer_target_member_id}")

    if infill_support_purlin_result is not None:
        for row in infill_support_purlin_result["support_rows"]:
            target_member_id = row["member_id"]
            point_load = (infill_support_purlin_result["support_y_mm"], row["R_kN"])
            if target_member_id in interior_rafter_point_loads:
                interior_rafter_point_loads[target_member_id].append(point_load)
            elif target_member_id == edge_rafters["left"]["id"]:
                edge_rafter_point_loads["left"].append(point_load)
            elif target_member_id == edge_rafters["right"]["id"]:
                edge_rafter_point_loads["right"].append(point_load)
            else:
                raise ValueError(f"Unsupported infill support purlin target: {target_member_id}")

    interior_results = []
    for idx, rafter_obj in enumerate(interior_rafters):
        point_loads = list(interior_rafter_point_loads.get(rafter_obj["id"], []))
        interior_results.append(analyse_rafter_case(rafter_obj, interior_direct_widths_m[idx], roof_area_kNm2_at, gamma_self, point_loads, edge=False))

    edge_results = {
        "left": analyse_rafter_case(edge_rafters["left"], 0.0, roof_area_kNm2_at, gamma_self, edge_rafter_point_loads["left"], edge=True),
        "right": analyse_rafter_case(edge_rafters["right"], 0.0, roof_area_kNm2_at, gamma_self, edge_rafter_point_loads["right"], edge=True),
    }

    outer_beam_point_loads = []
    inner_beam_point_loads = []
    for rafter_obj, r in zip(interior_rafters, interior_results):
        x_mm = float(rafter_obj["axis_start"]["x"])
        outer_beam_point_loads.append((x_mm, r["R_outer_kN"]))
        inner_beam_point_loads.append((x_mm, r["R_inner_kN"]))
    for side, edge_obj in edge_rafters.items():
        x_mm = float(edge_obj["axis_start"]["x"])
        outer_beam_point_loads.append((x_mm, edge_results[side]["R_outer_kN"]))
        inner_beam_point_loads.append((x_mm, edge_results[side]["R_inner_kN"]))
    for side, rows in corner_purlin_results.items():
        for result in rows:
            if result["outer_support_member_id"] == "beam.outer":
                outer_beam_point_loads.append((result["support_outer_x_mm"], result["R_outer_kN"]))

    inner_beam_extra_uniform_intervals = []
    if infill_glass_summary is not None:
        infill_reactions = infill_support_reactions_for_case(case_key)
        inner_beam_extra_uniform_intervals.append((
            infill_bounds["x0_mm"],
            infill_bounds["x1_mm"],
            infill_reactions["inner_reaction_kN_per_m"] / 1000.0,
        ))
    if gable_glazing_summary is not None:
        inner_beam_extra_uniform_intervals.append((
            gable_glazing_summary["x0_mm"],
            gable_glazing_summary["x1_mm"],
            gamma_self * gable_glazing_summary["self_line_kNm"] / 1000.0,
        ))

    outer_beam_result = analyse_beam_case(
        outer_beam,
        outer_supports_x_mm,
        sorted(outer_beam_point_loads, key=lambda item: item[0]),
        gamma_self,
        E_gl30c,
        OUTER_BEAM_PROPS["I_vertical_mm4"],
        OUTER_BEAM_MRd_y_kNm,
        OUTER_BEAM_VRd_kN,
    )
    inner_beam_result = analyse_beam_case(
        inner_beam,
        inner_supports_x_mm,
        sorted(inner_beam_point_loads, key=lambda item: item[0]),
        gamma_self,
        E_gl30c,
        INNER_BEAM_PROPS["I_vertical_mm4"],
        INNER_BEAM_MRd_kNm,
        INNER_BEAM_VRd_kN,
        extra_uniform_intervals=inner_beam_extra_uniform_intervals,
    )

    return {
        "case": case,
        "purlins": purlins,
        "infill_support_purlin": infill_support_purlin_result,
        "corner_purlins": corner_purlin_results,
        "interior_rafters": interior_results,
        "edge_rafters": edge_results,
        "outer_beam_point_loads": sorted(outer_beam_point_loads, key=lambda item: item[0]),
        "inner_beam_point_loads": sorted(inner_beam_point_loads, key=lambda item: item[0]),
        "outer_beam": outer_beam_result,
        "inner_beam": inner_beam_result,
    }


RESULTS = {case_key: analyse_case(case_key) for case_key in (*ULS_CASE_KEYS, *SLS_CASE_KEYS, "UPLIFT")}
OUTER_BEAM_H = {
    "ULS A": analyse_outer_beam_horizontal(gammaQ * psi0_W * q_outer_wind_h_char),
    "ULS B": analyse_outer_beam_horizontal(gammaQ * q_outer_wind_h_char),
    "ULS DRIFT": analyse_outer_beam_horizontal(gammaQ * psi0_W * q_outer_wind_h_char),
}
GABLE_GLAZING_H = {}
if gable_glazing_summary is not None:
    for case_key, factor in {
        "ULS A": gammaQ * psi0_W,
        "ULS B": gammaQ,
        "ULS DRIFT": gammaQ * psi0_W,
        "SLS": 1.0,
        "SLS DRIFT": 1.0,
    }.items():
        GABLE_GLAZING_H[case_key] = {
            "inner_beam": analyse_beam_horizontal_interval(
                inner_beam,
                inner_supports_x_mm,
                gable_glazing_summary["x0_mm"],
                gable_glazing_summary["x1_mm"],
                factor * gable_glazing_summary["wind_line_inner_char_kNm"],
                E_gl30c,
                INNER_BEAM_PROPS["I_horizontal_mm4"],
                INNER_BEAM_MRd_z_kNm,
                INNER_BEAM_VRd_kN,
            ),
            "existing_beam": analyse_beam_horizontal_interval(
                existing_beam,
                inner_supports_x_mm,
                gable_glazing_summary["x0_mm"],
                gable_glazing_summary["x1_mm"],
                factor * gable_glazing_summary["wind_line_existing_char_kNm"],
                E_lvl,
                EXISTING_BEAM_PROPS["I_horizontal_mm4"],
                EXISTING_BEAM_MRd_z_kNm,
                EXISTING_BEAM_VRd_kN,
            ),
        }


# ── governing-yhteenvedot ───────────────────────────────────────────────────

def with_case(result, case_key):
    return {"case_key": case_key, **result}


def governing_notch_eta(result):
    if "notch" in result:
        notch = result["notch"]
        return 0.0 if notch is None else notch["eta_gov"]["value_pct"]
    notch = result.get("governing_notch")
    return 0.0 if notch is None else notch["eta_gov"]["value_pct"]


def member_eta_value(result):
    return max(result["eta_M"], result["eta_V"], governing_notch_eta(result))


purlin_design_results = {}
for side in ("left", "right"):
    purlin_design_results[side] = []
    for idx in range(len(RESULTS["ULS A"]["purlins"][side])):
        chosen = max((with_case(RESULTS[case_key]["purlins"][side][idx], case_key) for case_key in ULS_CASE_KEYS), key=member_eta_value)
        sls = max((with_case(RESULTS[case_key]["purlins"][side][idx], case_key) for case_key in SLS_CASE_KEYS), key=lambda item: abs(item["delta"]["value_mm"]))
        uplift = RESULTS["UPLIFT"]["purlins"][side][idx]
        chosen["eta_gov"] = member_eta_value(chosen)
        chosen["sls_delta_mm"] = abs(sls["delta"]["value_mm"])
        chosen["sls_case_key"] = sls["case_key"]
        chosen["uplift_edge_kN"] = uplift["R_edge_kN"]
        chosen["uplift_inner_kN"] = uplift["R_inner_kN"]
        purlin_design_results[side].append(chosen)

infill_support_purlin_design = None
if RESULTS["ULS A"]["infill_support_purlin"] is not None:
    infill_support_purlin_design = max(
        (with_case(RESULTS[case_key]["infill_support_purlin"], case_key) for case_key in ULS_CASE_KEYS),
        key=member_eta_value,
    )
    infill_support_purlin_sls = max(
        (with_case(RESULTS[case_key]["infill_support_purlin"], case_key) for case_key in SLS_CASE_KEYS),
        key=lambda item: abs(item["delta"]["value_mm"]),
    )
    infill_support_purlin_uplift = RESULTS["UPLIFT"]["infill_support_purlin"]
    infill_support_purlin_design["eta_gov"] = member_eta_value(infill_support_purlin_design)
    infill_support_purlin_design["sls_delta_mm"] = abs(infill_support_purlin_sls["delta"]["value_mm"])
    infill_support_purlin_design["sls_case_key"] = infill_support_purlin_sls["case_key"]
    infill_support_purlin_design["uplift_reaction_min_kN"] = min(
        row["R_kN"] for row in infill_support_purlin_uplift["support_rows"]
    )

corner_purlin_design_results = {}
for side in ("left", "right"):
    if side not in RESULTS["ULS A"]["corner_purlins"]:
        continue
    corner_purlin_design_results[side] = []
    for idx in range(len(RESULTS["ULS A"]["corner_purlins"][side])):
        chosen = max((with_case(RESULTS[case_key]["corner_purlins"][side][idx], case_key) for case_key in ULS_CASE_KEYS), key=member_eta_value)
        sls = max((with_case(RESULTS[case_key]["corner_purlins"][side][idx], case_key) for case_key in SLS_CASE_KEYS), key=lambda item: abs(item["delta"]["value_mm"]))
        uplift = RESULTS["UPLIFT"]["corner_purlins"][side][idx]
        chosen["eta_gov"] = member_eta_value(chosen)
        chosen["sls_delta_mm"] = abs(sls["delta"]["value_mm"])
        chosen["sls_case_key"] = sls["case_key"]
        chosen["uplift_inner_kN"] = uplift["R_inner_kN"]
        chosen["uplift_outer_kN"] = uplift["R_outer_kN"]
        corner_purlin_design_results[side].append(chosen)

interior_rafter_design_results = []
for idx in range(len(interior_rafters)):
    chosen = max((with_case(RESULTS[case_key]["interior_rafters"][idx], case_key) for case_key in ULS_CASE_KEYS), key=member_eta_value)
    sls = max((with_case(RESULTS[case_key]["interior_rafters"][idx], case_key) for case_key in SLS_CASE_KEYS), key=lambda item: abs(item["delta"]["value_mm"]))
    uplift = RESULTS["UPLIFT"]["interior_rafters"][idx]
    chosen["eta_gov"] = member_eta_value(chosen)
    chosen["sls_delta_mm"] = abs(sls["delta"]["value_mm"])
    chosen["sls_case_key"] = sls["case_key"]
    chosen["uplift_inner_kN"] = uplift["R_inner_kN"]
    chosen["uplift_outer_kN"] = uplift["R_outer_kN"]
    interior_rafter_design_results.append(chosen)

edge_rafter_design_results = {}
for side in ("left", "right"):
    chosen = max((with_case(RESULTS[case_key]["edge_rafters"][side], case_key) for case_key in ULS_CASE_KEYS), key=member_eta_value)
    sls = max((with_case(RESULTS[case_key]["edge_rafters"][side], case_key) for case_key in SLS_CASE_KEYS), key=lambda item: abs(item["delta"]["value_mm"]))
    uplift = RESULTS["UPLIFT"]["edge_rafters"][side]
    chosen["eta_gov"] = member_eta_value(chosen)
    chosen["sls_delta_mm"] = abs(sls["delta"]["value_mm"])
    chosen["sls_case_key"] = sls["case_key"]
    chosen["uplift_inner_kN"] = uplift["R_inner_kN"]
    chosen["uplift_outer_kN"] = uplift["R_outer_kN"]
    edge_rafter_design_results[side] = chosen

critical_purlin = max([*purlin_design_results["left"], *purlin_design_results["right"]], key=lambda item: item["eta_gov"])
critical_corner_purlin = max(
    (row for rows in corner_purlin_design_results.values() for row in rows),
    key=lambda item: item["eta_gov"],
) if corner_purlin_design_results else None
critical_interior_rafter = max(interior_rafter_design_results, key=lambda item: item["eta_gov"])
critical_edge_rafter = max(edge_rafter_design_results.values(), key=lambda item: item["eta_gov"])

outer_beam_eta = {
    case_key: max(RESULTS[case_key]["outer_beam"]["eta_M"], RESULTS[case_key]["outer_beam"]["eta_V"])
    for case_key in ULS_CASE_KEYS
}
outer_beam_interaction = {
    case_key: RESULTS[case_key]["outer_beam"]["M_gov"]["value_kNm"] / OUTER_BEAM_MRd_y_kNm * 100.0
    + 0.7 * OUTER_BEAM_H[case_key]["M_gov"]["value_kNm"] / OUTER_BEAM_MRd_z_kNm * 100.0
    for case_key in ULS_CASE_KEYS
}
outer_beam_governing_case = max(outer_beam_interaction, key=outer_beam_interaction.get)
inner_beam_governing_case = max(
    ULS_CASE_KEYS,
    key=lambda case_key: max(RESULTS[case_key]["inner_beam"]["eta_M"], RESULTS[case_key]["inner_beam"]["eta_V"]),
)
inner_beam_governing = RESULTS[inner_beam_governing_case]["inner_beam"]
if GABLE_GLAZING_H:
    gable_glazing_h_governing_case = max(
        ULS_CASE_KEYS,
        key=lambda case_key: max(
            GABLE_GLAZING_H[case_key]["inner_beam"]["eta_M"],
            GABLE_GLAZING_H[case_key]["inner_beam"]["eta_V"],
            GABLE_GLAZING_H[case_key]["existing_beam"]["eta_M"],
            GABLE_GLAZING_H[case_key]["existing_beam"]["eta_V"],
        ),
    )
    gable_glazing_h_governing = GABLE_GLAZING_H[gable_glazing_h_governing_case]
    gable_glazing_h_sls = GABLE_GLAZING_H["SLS"]
    gable_glazing_h_sls_drift = GABLE_GLAZING_H["SLS DRIFT"]
else:
    gable_glazing_h_governing_case = None
    gable_glazing_h_governing = None
    gable_glazing_h_sls = None
    gable_glazing_h_sls_drift = None

outer_beam_sls_normal_delta_mm = abs(RESULTS["SLS"]["outer_beam"]["delta"]["value_mm"])
outer_beam_sls_drift_delta_mm = abs(RESULTS["SLS DRIFT"]["outer_beam"]["delta"]["value_mm"])
outer_beam_sls_delta_mm = max(outer_beam_sls_normal_delta_mm, outer_beam_sls_drift_delta_mm)
inner_beam_sls_normal_delta_mm = abs(RESULTS["SLS"]["inner_beam"]["delta"]["value_mm"])
inner_beam_sls_drift_delta_mm = abs(RESULTS["SLS DRIFT"]["inner_beam"]["delta"]["value_mm"])
inner_beam_sls_delta_mm = max(inner_beam_sls_normal_delta_mm, inner_beam_sls_drift_delta_mm)
outer_beam_governing_point_load = max(RESULTS[outer_beam_governing_case]["outer_beam_point_loads"], key=lambda item: item[1])
outer_reaction_case = max(ULS_CASE_KEYS, key=lambda case_key: max(RESULTS[case_key]["outer_beam"]["reactions_kN"].values()))
inner_reaction_case = max(ULS_CASE_KEYS, key=lambda case_key: max(RESULTS[case_key]["inner_beam"]["reactions_kN"].values()))

outer_uplift = RESULTS["UPLIFT"]["outer_beam"]["reactions_kN"]
inner_uplift = RESULTS["UPLIFT"]["inner_beam"]["reactions_kN"]

column_case_groups = {
    case_key: ("UPLIFT" if case_key == "UPLIFT" else "SLS" if case_key in SLS_CASE_KEYS else "ULS")
    for case_key in (*ULS_CASE_KEYS, *SLS_CASE_KEYS, "UPLIFT")
}
inner_column_id_by_support_x = {
    inner_supports_x_mm[0]: "col.x125",
    inner_supports_x_mm[-1]: "col.x7075",
}
outer_ground_column_id_by_support_x = {
    outer_supports_x_mm[0]: "col.x125.outer.bottom",
    outer_supports_x_mm[1]: "col.x3600.outer.bottom",
    outer_supports_x_mm[2]: "col.x7075.outer.bottom",
}
outer_column_member_id_by_support_x = {
    float(member(GEO, "columns", "col.outer.x0")["base"]["x"]): "col.outer.x0",
    float(member(GEO, "columns", "col.outer.x3600")["base"]["x"]): "col.outer.x3600",
    float(member(GEO, "columns", "col.outer.x7200")["base"]["x"]): "col.outer.x7200",
}
outer_column_objs_by_support_x = {
    support_x_mm: member(GEO, "columns", column_id)
    for support_x_mm, column_id in outer_column_member_id_by_support_x.items()
}

additional_upper_column_loads_by_case = {
    case_key: {
        column_id: RESULTS[case_key]["inner_beam"]["reactions_kN"].get(support_x_mm, 0.0)
        for support_x_mm, column_id in inner_column_id_by_support_x.items()
    }
    for case_key in (*ULS_CASE_KEYS, *SLS_CASE_KEYS, "UPLIFT")
}
portaikko_extra_upper_column_loads_by_case = portaikko_existing_column_extra_loads_by_case(
    (*ULS_CASE_KEYS, *SLS_CASE_KEYS, "UPLIFT")
)
for case_key, column_loads in portaikko_extra_upper_column_loads_by_case.items():
    for column_id, load_kN in column_loads.items():
        additional_upper_column_loads_by_case[case_key][column_id] = (
            additional_upper_column_loads_by_case[case_key].get(column_id, 0.0) + load_kN
        )
portaikko_col_x7075_extra_sls = max(
    portaikko_extra_upper_column_loads_by_case[case_key]["col.x7075"]
    for case_key in SLS_CASE_KEYS
)
portaikko_col_x7075_extra_uls = max(
    portaikko_extra_upper_column_loads_by_case[case_key]["col.x7075"]
    for case_key in ULS_CASE_KEYS
)
portaikko_col_x7075_extra_uplift = portaikko_extra_upper_column_loads_by_case["UPLIFT"]["col.x7075"]
additional_ground_column_loads_by_case = {}
for case_key in (*ULS_CASE_KEYS, *SLS_CASE_KEYS, "UPLIFT"):
    permanent_factor = COLUMN_CASE_FACTORS[column_case_groups[case_key]]
    additional_ground_column_loads_by_case[case_key] = {}
    for support_x_mm, ground_column_id in outer_ground_column_id_by_support_x.items():
        column_obj = outer_column_objs_by_support_x[support_x_mm]
        additional_ground_column_loads_by_case[case_key][ground_column_id] = (
            RESULTS[case_key]["outer_beam"]["reactions_kN"].get(support_x_mm, 0.0)
            + column_self_weight_kN(column_obj, GAMMA_CONCRETE_KNM3, factor=permanent_factor)
        )

terrace_total_column_loads = calculate_katos_total_column_loads(
    case_groups=column_case_groups,
    extra_upper_column_loads_by_case=additional_upper_column_loads_by_case,
    extra_ground_column_loads_by_case=additional_ground_column_loads_by_case,
)
terrace_total_column_envelope = envelope_column_totals(
    terrace_total_column_loads["case_totals"],
    ULS_CASE_KEYS,
    SLS_CASE_KEYS,
)
foundation_checks = foundation_checks_from_envelope(FOUNDATION_GEO, terrace_total_column_envelope)

h_seina_left_m = local_wall_height_m_at_x(inner_supports_x_mm[0])
h_seina_right_m = local_wall_height_m_at_x(inner_supports_x_mm[-1])


# ── tulostus ─────────────────────────────────────────────────────────────────

W = 70
DW = "=" * W

print(DW)
print("  LASITETTU TERASSI – LOPULLINEN PUURATKAISU – KUORMITUSLASKENTA")
print("  EN 1990 / EN 1991-1-1/3/4 / EN 1995-1-1")
print(DW)

print("\n── GEOMETRIA ─────────────────────────────────────────────────────")
print(f"  Geometria                     geometry/terassi_puu.json")
print(f"  Paneelikenttä                 {roof_width_mm:.0f} × {roof_depth_mm:.0f} mm  ({roof_area_m2:.2f} m²)")
if infill_glass_summary is not None:
    print(
        f"  Seinän täytekaista            {infill_glass_summary['width_mm']:.0f} × {infill_glass_summary['depth_mm']:.0f} mm  "
        f"({infill_glass_summary['area_m2']:.2f} m²), {infill_glass_summary['thickness_mm']:.0f} mm lasi, "
        f"ulkoreuna -> {infill_glass_summary['outer_support_member_id']}"
    )
if gable_glazing_summary is not None:
    print(
        f"  Päätykolmiolasi               x = {gable_glazing_summary['x0_mm']:.0f}…{gable_glazing_summary['x1_mm']:.0f} mm, "
        f"A = {gable_glazing_summary['area_m2']:.2f} m², {gable_glazing_summary['thickness_mm']:.0f} mm lasi"
    )
print(f"  Kattokaltevuus y-suunnassa    {roof_slope_deg:.1f}°")
print(f"  Sisäkattotuolit               {len(interior_rafters)} kpl  @ {interior_rafter_xs_mm[1]-interior_rafter_xs_mm[0]:.0f} mm")
print(f"  Reunakattotuolit              2 kpl  @ x = {edge_rafters['left']['axis_start']['x']:.0f} / {edge_rafters['right']['axis_start']['x']:.0f} mm")
print(
    f"  Orret / purlins x-suunnassa   {len(left_purlins) + len(right_purlins)} kpl"
    f"  (vasen y = {', '.join(f'{y_mm:.0f}' for y_mm in purlin_y_positions_mm['left'])} mm;"
    f" oikea y = {', '.join(f'{y_mm:.0f}' for y_mm in purlin_y_positions_mm['right'])} mm)"
)
if infill_support_purlin is not None:
    print(
        f"  Täytekaistan tukiorsi         {infill_support_purlin['id']} "
        f"{infill_support_purlin['profile']['name']} @ y = {infill_support_purlin_y_mm:.0f} mm"
    )
if corner_purlins:
    print(f"  Nurkkaorret y-suunnassa       {len(corner_purlins)} kpl  @ x = " + ", ".join(f"{float(item['axis_start']['x']):.0f}" for item in corner_purlins) + " mm")
print(f"  Kattotuolien tuet sisa/ulko   y = {interior_inner_support_y_mm:.0f} / {interior_outer_support_y_mm:.0f} mm")
print(f"  Reunakattotuolien sisatuki    y = {edge_inner_support_y_mm:.0f} mm")
print(f"  Sisäpään liitos               {format_connection_behavior(inner_hanger_analysis, inner_hanger_rot_k_Nmm_per_rad)}")
print(
    "  Reunakattotuolien liitokset   "
    + " / ".join(
        f"{SIDE_MEMBER_LABEL[side]}: {format_connection_behavior(edge_rafter_support_analysis_by_side[side], edge_rafter_support_rot_k_by_side[side])}"
        for side in ("left", "right")
    )
)
print("  Orsien liitokset              nivelletyt tuet")
print(f"  Ulkopalkin tuet               x = " + " / ".join(f"{x_mm:.0f}" for x_mm in outer_supports_x_mm) + " mm")
print(f"  Sisäpalkin tuet               x = " + " / ".join(f"{x_mm:.0f}" for x_mm in inner_supports_x_mm) + " mm")

print("\n── KUORMAT ───────────────────────────────────────────────────────")
print(f"  Paneelit                      {panel_count['nx']}×{panel_count['ny']} = {panel_count_total} kpl, {panels_total_kN:.2f} kN")
print(f"  Pysyvä katekuorma             gk = {gk_roofing:.3f} kN/m²  (paneelit {gk_panels:.3f} + kiinnikkeet {gk_fixings:.2f})")
if infill_glass_summary is not None:
    print(
        f"  Täytekaistan lasi             {infill_glass_summary['mass_kg']:.0f} kg = "
        f"{infill_glass_summary['total_kN']:.2f} kN, gk = {infill_glass_summary['gk_kNm2']:.3f} kN/m²"
    )
if gable_glazing_summary is not None:
    print(
        f"  Päätykolmiolasi               {gable_glazing_summary['mass_kg']:.0f} kg = "
        f"{gable_glazing_summary['total_kN']:.2f} kN, omapaino alapalkille {gable_glazing_summary['self_line_kNm']:.3f} kN/m"
    )
print(f"  Lumi                          sk = {sk:.1f} kN/m², μ1 = {mu1:.1f}  →  s = {s_roof:.2f} kN/m²")
print(f"  Tuuli katolle                 qp(z={z_ref_m:.1f} m) = {qp_z:.3f} kN/m²")
print(f"    alas (auki)                 cp,net = {cp_net_down:+.2f}  →  w = {w_down:.3f} kN/m²")
print(f"    imu (kiinni)                cp,net = {cp_net_up_closed:+.2f}  →  w = {w_up_closed:.3f} kN/m²")
print(f"  ULS A kattokuorma             {roof_area_uls_A:.3f} kN/m²  (1.35G + 1.5S + 1.5·0.6W↓)")
print(f"  ULS B kattokuorma             {roof_area_uls_B:.3f} kN/m²  (1.35G + 1.5·0.7S)")
print(f"  ULS DRIFT kattokuorma         1.35G + 1.5·S_kin(x,y)")
print(f"  Uplift-kattokuorma            {roof_area_uplift:.3f} kN/m²  (0.9G + 1.5W↑)")
print(f"  Seinää vasten h(x)            {h_seina_left_m:.2f} … {h_seina_right_m:.2f} m")
print(f"  Kinostuman alku               y = {drift_obstacle_y_mm:.0f} mm, paneelikentän sisäreuna y = {roof_y0_mm:.0f} mm")
print(f"    hallitseva x                {critical_drift['x_mm']:.0f} mm, ls = {critical_drift['ls_m']:.2f} m, μ2 = {critical_drift['mu2']:.2f}")
print(f"    s_kin,max / s@y_in          {critical_drift['s_peak_kNm2']:.2f} / {critical_drift['s_inner_rafter_kNm2']:.2f} kN/m²")

print("\n── PANEELIT ──────────────────────────────────────────────────────")
print(f"  Tyyppi                        {panel_material}")
print(
    f"  Moduuli                       {panel_unit_width_mm:.0f} × {panel_unit_slope_length_mm:.0f} × "
    f"{panel_unit_thickness_mm:.0f} mm, {panel_mass_kg:.0f} kg/kpl"
)
print(
    f"  Moduulikenttä                 {panel_count_x} × {panel_count_y} = {panel_count_total} kpl, "
    f"lappeella {panel_field_slope_width_mm:.0f} × {panel_field_slope_length_mm:.0f} mm, "
    f"massa moduuleista {panel_count_total * panel_mass_kg:.0f} kg"
)
print(
    f"  Laskentapinta                 lappeella {roof_width_mm:.0f} × {roof_slope_length_mm:.0f} mm, "
    f"vaakaprojektio {roof_width_mm:.0f} × {roof_depth_mm:.0f} mm"
)
print(f"  Kokonaismassa                 {panel_total_mass_kg:.0f} kg = {panels_total_kN:.2f} kN")
print(
    f"  Kinostumatarkistus            etupuolen mekaaninen raja {panel_front_snow_cap_kNm2:.2f} kN/m², "
    f"ULS = 1.5·s_kin"
)
print(
    f"  Mitoittava paneelisarake      {critical_panel_column['index']}/{panel_count_x}, "
    f"x = {critical_panel_column['x_center_mm']:.0f} mm, h = {critical_panel_column['h_m']:.2f} m, "
    f"ls = {critical_panel_column['ls_m']:.2f} m, s_peak = {critical_panel_column['s_peak_kNm2']:.2f} kN/m²"
)
panel_status = "OK" if critical_panel_ok else "YLITTYY"
print(
    f"  Mitoittava piste              {critical_panel_check['label']}, y = {critical_panel_check['y_mm']:.0f} mm, "
    f"ULS = {critical_panel_check['uls_kNm2']:.2f}/{panel_front_snow_cap_kNm2:.2f} kN/m², "
    f"η = {critical_panel_check['eta_pct']:.1f}%  -> {panel_status}"
)
if panel_height_limit_m is None:
    print(
        f"  Rajan alitus edellyttää       ei mahdollinen pelkällä kinostumageometrian tarkennuksella "
        f"(1.5·μ1·sk = {gammaQ * s_roof:.2f} kN/m² > {panel_front_snow_cap_kNm2:.2f} kN/m²)"
    )
else:
    print(
        f"  Rajan alitus edellyttää       s_kin <= {panel_char_limit_kNm2:.2f} kN/m², "
        f"h <= {panel_height_limit_m:.2f} m @ {critical_panel_check['label']}"
    )
    print(
        f"  Nykyinen ylitys / varmuus     ΔULS = {panel_uls_margin_kNm2:+.2f} kN/m², "
        f"Δs_kin = {panel_char_margin_kNm2:+.2f} kN/m², Δh = {panel_height_margin_mm:+.0f} mm"
    )
print()
print(f"  Tarkistuspisteet              kriittinen sarake {critical_panel_column['index']}/{panel_count_x}")
print(f"  {'Sijainti':<24} {'y [mm]':>7} {'s_kin':>7} {'ULS':>7} {'η':>7} {'Tila':>8}")
print(f"  {'-'*24} {'-'*7} {'-'*7} {'-'*7} {'-'*7} {'-'*8}")
for row in critical_panel_check_rows:
    print(
        f"  {row['label']:<24} {row['y_mm']:>7.0f} {row['s_char_kNm2']:>7.2f} "
        f"{row['uls_kNm2']:>7.2f} {row['eta_pct']:>6.1f}% "
        f"{('OK' if row['ok'] else 'YLITYS'):>8}"
    )

if infill_glass_summary is not None:
    infill_status = "OK" if infill_glass_summary["eta_sigma_pct"] <= 100.0 and infill_glass_summary["eta_delta_pct"] <= 100.0 else "YLITTYY"
    print("\n── SEINÄN TÄYTEKAISTA – 8 mm LASI ────────────────────────────────")
    print(f"  Materiaali                    {infill_glass_summary['material']}")
    print(
        f"  Kantosuunta                    seinältä {infill_glass_summary['outer_support_member_id']}:lle, "
        f"L = {infill_glass_summary['span_mm']:.0f} mm"
        + (
            f" (+ ulkoreunan ylitys {infill_glass_summary['outer_overhang_mm']:.0f} mm)"
            if infill_glass_summary["outer_overhang_mm"] > 1.0
            else ""
        )
    )
    print(
        f"  Pinta-ala / massa              {infill_glass_summary['area_m2']:.2f} m² / "
        f"{infill_glass_summary['mass_kg']:.0f} kg"
    )
    print(
        f"  Mitoittava lumi                {infill_glass_summary['critical_snow']['label']}, "
        f"x = {infill_glass_summary['critical_snow']['x_mm']:.0f} mm, "
        f"y = {infill_glass_summary['critical_snow']['y_mm']:.0f} mm, "
        f"s_kin = {infill_glass_summary['critical_snow']['s_char_kNm2']:.2f} kN/m²"
    )
    print(
        f"  ULS alas                       A = {infill_glass_summary['uls_A_kNm2']:.2f}, "
        f"DRIFT = {infill_glass_summary['uls_drift_kNm2']:.2f} -> "
        f"{infill_glass_summary['uls_down_kNm2']:.2f} kN/m²"
    )
    print(
        f"  Uplift                         {infill_glass_summary['uplift_kNm2']:.2f} kN/m², "
        f"σ = {infill_glass_summary['uplift_sigma_Nmm2']:.1f} N/mm²"
    )
    print(
        f"  Taivutus                       M = {infill_glass_summary['M_kNm_per_m']:.3f} kNm/m, "
        f"σ = {infill_glass_summary['sigma_Nmm2']:.1f}/{infill_glass_summary['sigma_limit_Nmm2']:.1f} N/mm², "
        f"η = {infill_glass_summary['eta_sigma_pct']:.1f}%"
    )
    print(
        f"  Taipuma SLS DRIFT              δ = {infill_glass_summary['delta_mm']:.2f}/"
        f"{infill_glass_summary['delta_limit_mm']:.2f} mm, η = {infill_glass_summary['eta_delta_pct']:.1f}%"
    )
    print(
        f"  Tukireaktiot ULS               beam.inner.new {infill_glass_summary['inner_support_reaction_kN_per_m']:.2f} kN/m, "
        f"{infill_glass_summary['outer_support_member_id']} {infill_glass_summary['outer_support_reaction_kN_per_m']:.2f} kN/m  -> {infill_status}"
    )
    if infill_support_purlin_design is not None:
        max_support_reaction = max(abs(row["R_kN"]) for row in infill_support_purlin_design["support_rows"])
        print(
            f"  Tukiorren tarkistus            {infill_support_purlin_design['id']} {infill_support_purlin_design['profile']}, "
            f"{infill_support_purlin_design['case_key']}: Md = {infill_support_purlin_design['M_gov']['value_kNm']:.2f} kNm, "
            f"η_M/η_V = {infill_support_purlin_design['eta_M']:.1f}%/{infill_support_purlin_design['eta_V']:.1f}%, "
            f"δ = {infill_support_purlin_design['sls_delta_mm']:.2f}/{infill_support_purlin_design['delta_lim_mm']:.1f} mm"
        )
        print(
            f"  Tukiorren kuormareitti         {len(infill_support_purlin_design['support_rows'])} kattotuolitukea, "
            f"max |R| = {max_support_reaction:.2f} kN"
        )

if gable_glazing_summary is not None:
    inner_h = gable_glazing_h_governing["inner_beam"]
    existing_h = gable_glazing_h_governing["existing_beam"]
    inner_sls_delta = max(
        abs(gable_glazing_h_sls["inner_beam"]["delta"]["value_mm"]),
        abs(gable_glazing_h_sls_drift["inner_beam"]["delta"]["value_mm"]),
    )
    existing_sls_delta = max(
        abs(gable_glazing_h_sls["existing_beam"]["delta"]["value_mm"]),
        abs(gable_glazing_h_sls_drift["existing_beam"]["delta"]["value_mm"]),
    )
    inner_span_limit_mm = (inner_supports_x_mm[-1] - inner_supports_x_mm[0]) / 300.0
    existing_span_limit_mm = (inner_supports_x_mm[-1] - inner_supports_x_mm[0]) / 300.0
    print("\n── PÄÄTYKOLMIOLASI – KUORMANSIIRTO ──────────────────────────────")
    print(
        f"  Omapaino                      {gable_glazing_summary['total_kN']:.2f} kN -> "
        f"beam.inner.new viivakuormana {gable_glazing_summary['self_line_kNm']:.3f} kN/m"
    )
    print(
        f"  Tuulijako                     50% alapalkille + 50% yläreunan 2×KP360×51-palkille "
        f"(pystyreunaa ei hyvitetä)"
    )
    print(
        f"  Vaakatuet                     beam.inner.new x = {inner_supports_x_mm[0]:.0f} / {inner_supports_x_mm[-1]:.0f} mm; "
        f"2×KP360×51 x = {inner_supports_x_mm[0]:.0f} / {inner_supports_x_mm[-1]:.0f} mm"
    )
    print(
        f"  Tuulikuorma yhteensä          {gable_glazing_summary['wind_char_total_kN']:.2f} kN, "
        f"q_line,char = {gable_glazing_summary['wind_line_inner_char_kNm']:.3f} kN/m per palkki"
    )
    print(f"  Hallitseva vaakakuorma        {gable_glazing_h_governing_case}")
    print(
        f"  beam.inner.new vaakataivutus  Md = {inner_h['M_gov']['value_kNm']:.2f} kNm, "
        f"Vd = {abs(inner_h['V_abs']['value_kN']):.2f} kN, "
        f"η_M/η_V = {inner_h['eta_M']:.1f}%/{inner_h['eta_V']:.1f}%, "
        f"δ = {inner_sls_delta:.2f}/{inner_span_limit_mm:.1f} mm"
    )
    print(
        f"  beam.existing.kp360x2         Md = {existing_h['M_gov']['value_kNm']:.2f} kNm, "
        f"Vd = {abs(existing_h['V_abs']['value_kN']):.2f} kN, "
        f"η_M/η_V = {existing_h['eta_M']:.1f}%/{existing_h['eta_V']:.1f}%, "
        f"δ = {existing_sls_delta:.2f}/{existing_span_limit_mm:.1f} mm"
    )

print("\n── ORRET X-SUUNNASSA 98×48 C24 ──────────────────────────────────")
print(f"  Sivukaista vasen / oikea      {left_strip_width_m:.3f} / {right_strip_width_m:.3f} m")
print(
    f"  Tributäärikorkeudet           vasen: {' / '.join(f'{h_m:.3f}' for h_m in purlin_trib_heights_m['left'])} m"
    f" | oikea: {' / '.join(f'{h_m:.3f}' for h_m in purlin_trib_heights_m['right'])} m"
)
print(f"  Poikkileikkauksen kierto      vasen: {member_section_rotation_deg(left_purlins[0]):+.1f}° | oikea: {member_section_rotation_deg(right_purlins[0]):+.1f}°")
print(f"  Paneelikehikko                {format_load_transfer_rule(horizontal_edge_purlin_ids)}")
print(
    f"  MRd = {PURLIN_MRd_kNm:.2f} kNm,  VRd = {PURLIN_VRd_kN:.2f} kN,  "
    f"δ_lim vasen / oikea = {(left_purlin_support_x_mm-left_purlin_edge_support_line_x_mm)/300.0:.1f} / "
    f"{(right_purlin_edge_support_line_x_mm-right_purlin_support_x_mm)/300.0:.1f} mm"
)
print(f"  Paatybevel ulokepaassa        {format_labeled_bevel_notch_specs(purlin_inner_notch_info)}")
if purlin_edge_notch_ref is not None:
    if purlin_edge_notch_ref.get("kind") == "bevel_notch":
        edge_notch_desc = f"bevel_notch {purlin_edge_notch_ref.get('side')} {purlin_edge_notch_depth_mm:.0f} × {purlin_edge_notch_length_mm:.0f} mm"
    else:
        edge_notch_desc = f"rect_notch {purlin_edge_notch_depth_mm:.0f} × {purlin_edge_notch_length_mm:.0f} mm"
    print(f"  Lovi reunatuella              {edge_notch_desc}, h_net,min = {critical_purlin['h_net_min_mm']:.0f} mm")
else:
    print("  Lovi reunatuella              ei lovea")
print(
    f"  Tukilinjat vasen              x_sisä = {left_purlin_support_x_mm:.0f} mm, "
    f"x_reuna = {left_purlin_edge_support_line_x_mm:.0f} mm; "
    f"y_conn = {' / '.join(f'{y_mm:.0f}' for y_mm in edge_purlin_support_ys_mm['left'])} mm"
)
print(
    f"  Tukilinjat oikea              x_sisä = {right_purlin_support_x_mm:.0f} mm, "
    f"x_reuna = {right_purlin_edge_support_line_x_mm:.0f} mm; "
    f"y_conn = {' / '.join(f'{y_mm:.0f}' for y_mm in edge_purlin_support_ys_mm['right'])} mm"
)
print()
print(f"  {'ID':<15} {'b_trib':>7} {'q_avg':>7} {'R_edge':>8} {'R_inner':>8} {'Md':>7} {'η_M':>7} {'η_V':>7} {'η_lovi':>8} {'δ_sls':>9}")
print(f"  {'-'*15} {'-'*7} {'-'*7} {'-'*8} {'-'*8} {'-'*7} {'-'*7} {'-'*7} {'-'*8} {'-'*9}")
for side in ("left", "right"):
    for row in purlin_design_results[side]:
        print(
            f"  {row['id']:<15} {row['trib_height_m']:>7.3f} {row['q_line_kNm']:>7.3f} {row['R_edge_kN']:>8.2f} {row['R_inner_kN']:>8.2f}"
            f" {row['M_gov']['value_kNm']:>6.2f} {row['eta_M']:>6.1f}% {row['eta_V']:>6.1f}% {row['notch']['eta_gov']['value_pct']:>7.1f}%"
            f" {row['sls_delta_mm']:>5.2f}/{row['delta_lim_mm']:.1f}"
        )
print(f"  Kriittinen orsi               {critical_purlin['id']}  ({critical_purlin['case_key']}, η = {critical_purlin['eta_gov']:.1f}%)")
print(f"    q_avg / q_max               {critical_purlin['q_line_kNm']:.3f} / {critical_purlin['q_line_max_kNm']:.3f} kN/m")
print(f"    Reaktio reunakattotuolelle  {critical_purlin['R_edge_kN']:.2f} kN,  sisäkattotuolille {critical_purlin['R_inner_kN']:.2f} kN")
print(f"    Governing lovi              {critical_purlin['notch']['label']} / {critical_purlin['notch']['eta_gov']['mode']} = {critical_purlin['notch']['eta_gov']['value_pct']:.1f}%")

if corner_purlin_design_results:
    print("\n── NURKKAORRET Y-SUUNNASSA 98×48 C24 ────────────────────────────")
    print(f"  Kantotapa                     tuet geometrian mukaan toiseksi uloimpaan kattotuoliin ja ulompaan reunajaseneen")
    print(
        f"  Tributäärileveydet            "
        + " | ".join(
            f"{side}: {' / '.join('{:.3f}'.format(row['trib_width_m']) for row in corner_purlin_design_results[side])} m"
            for side in sorted(corner_purlin_design_results)
        )
    )
    print(
        "  Poikkileikkauksen kierto      "
        + " | ".join(
            f"{side}: {min(row['section_rotation_deg'] for row in corner_purlin_design_results[side]):+.1f} … {max(row['section_rotation_deg'] for row in corner_purlin_design_results[side]):+.1f}°"
            for side in sorted(corner_purlin_design_results)
        )
    )
    print(f"  Paneelikuorma                 {format_load_transfer_rule(slanted_edge_purlin_ids)}")
    slanted_notch_desc = []
    slanted_rafter_notches = [
        bevel_notch_info(conn["id"])
        for conn in corner_purlin_inner_support_connections.values()
        if conn is not None and bevel_notch_info(conn["id"])["active"]
    ]
    if not slanted_rafter_notches:
        slanted_rafter_notches = [
            info
            for member_id, info in corner_purlin_outer_notch_info.items()
            if info["active"] and corner_purlin_outer_support_member_ids[member_id].startswith(INTERIOR_RAFTER_PREFIX)
        ]
    if slanted_rafter_notches:
        slanted_notch_desc.append(
            f"rafter-tuet bevel_notch {format_bevel_notch_specs(slanted_rafter_notches)}"
        )
    slanted_beam_notches = [
        info
        for member_id, info in corner_purlin_outer_notch_info.items()
        if info["active"] and corner_purlin_outer_support_member_ids[member_id] == "beam.outer"
    ]
    if slanted_beam_notches:
        slanted_notch_desc.append(
            f"beam.outer bevel_notch {format_bevel_notch_specs(slanted_beam_notches)}"
        )
    if slanted_notch_desc:
        print(
        f"  Paatybevelit vinoissa orsissa "
            + "; ".join(slanted_notch_desc)
            + f", h_net,min = {min(row['h_net_min_mm'] for rows in corner_purlin_design_results.values() for row in rows):.0f} mm"
        )
    print()
    print(f"  {'ID':<20} {'b_trib':>7} {'q_avg':>7} {'R_in':>8} {'R_out':>8} {'Md':>7} {'η_M':>7} {'η_V':>7} {'η_lovi':>8} {'δ_sls':>9}")
    print(f"  {'-'*20} {'-'*7} {'-'*7} {'-'*8} {'-'*8} {'-'*7} {'-'*7} {'-'*7} {'-'*8} {'-'*9}")
    for side in ("left", "right"):
        for row in corner_purlin_design_results.get(side, []):
            print(
                f"  {row['id']:<20} {row['trib_width_m']:>7.3f} {row['q_line_kNm']:>7.3f} {row['R_inner_kN']:>8.2f} {row['R_outer_kN']:>8.2f}"
                f" {row['M_gov']['value_kNm']:>6.2f} {row['eta_M']:>6.1f}% {row['eta_V']:>6.1f}% {governing_notch_eta(row):>7.1f}% {row['sls_delta_mm']:>5.2f}/{row['delta_lim_mm']:.1f}"
            )
    print(f"  Kriittinen nurkkaorsi         {critical_corner_purlin['id']}  ({critical_corner_purlin['case_key']}, η = {critical_corner_purlin['eta_gov']:.1f}%)")
    print(f"    q_avg / q_max               {critical_corner_purlin['q_line_kNm']:.3f} / {critical_corner_purlin['q_line_max_kNm']:.3f} kN/m")
    print(f"    Reaktio sisatuelle          {critical_corner_purlin['R_inner_kN']:.2f} kN,  ulkotuella {critical_corner_purlin['R_outer_kN']:.2f} kN ({critical_corner_purlin['outer_support_label']})")
    if critical_corner_purlin['notch'] is not None:
        print(f"    Governing lovi              {critical_corner_purlin['notch']['label']} / {critical_corner_purlin['notch']['eta_gov']['mode']} = {critical_corner_purlin['notch']['eta_gov']['value_pct']:.1f}%")

print(f"\n── SISÄKATTOTUOLIT {rafter_h_mm:.0f}×{rafter_b_mm:.0f} C24 ───────────────────────────────────")
print(f"  Suora paneelikaista           " + " / ".join(f"{b_m:.3f}" for b_m in interior_direct_widths_m) + " m")
print(f"  MRd = {RAfter_MRd_kNm:.2f} kNm,  VRd = {RAfter_VRd_kN:.2f} kN,  δ_lim = {(interior_outer_support_y_mm-interior_inner_support_y_mm)/300.0:.1f} mm")
print(f"  Ulkotuki                      birdsmouth heel = {birdsmouth_heel_depth_mm:.0f} mm, seat = {birdsmouth_seat_length_mm:.0f} mm, h_net,min = {critical_interior_rafter['h_net_birdsmouth_min_mm']:.0f} mm")
print()
print(f"  {'ID':<10} {'b_dir':>7} {'purlin':>8} {'q_avg':>7} {'R_in':>8} {'R_out':>8} {'Md':>7} {'η_M':>7} {'η_V':>7} {'η_lovi':>8} {'δ_sls':>9}")
print(f"  {'-'*10} {'-'*7} {'-'*8} {'-'*7} {'-'*8} {'-'*8} {'-'*7} {'-'*7} {'-'*7} {'-'*8} {'-'*9}")
for row in interior_rafter_design_results:
    print(
        f"  {row['id']:<10} {row['direct_width_m']:>7.3f} {sum(load for _, load in row['point_loads_kN']):>8.2f} {row['q_line_kNm']:>7.3f}"
        f" {row['R_inner_kN']:>8.2f} {row['R_outer_kN']:>8.2f} {row['M_gov']['value_kNm']:>6.2f} {row['eta_M']:>6.1f}% {row['eta_V']:>6.1f}%"
        f" {governing_notch_eta(row):>7.1f}% {row['sls_delta_mm']:>5.2f}/{row['delta_lim_mm']:.1f}"
    )
print(f"  Kriittinen sisäkattotuoli     {critical_interior_rafter['id']}  ({critical_interior_rafter['case_key']}, η = {critical_interior_rafter['eta_gov']:.1f}%)")
print(f"    q_avg / q_max               {critical_interior_rafter['q_line_kNm']:.3f} / {critical_interior_rafter['q_line_max_kNm']:.3f} kN/m")
print(f"    Birdsmouth-nettotarkistus   {critical_interior_rafter['birdsmouth']['eta_gov']['mode']} = {critical_interior_rafter['birdsmouth']['eta_gov']['value_pct']:.1f}%"
      f" @ y = {critical_interior_rafter['birdsmouth']['eta_gov']['x_mm']:.0f} mm")
print(f"    Lovisyvyys h/3              {format_birdsmouth_h3_status(critical_interior_rafter['birdsmouth_rule_ok'])}")

print("\n── REUNAKATTOTUOLIT GL30c ───────────────────────────────────────")
print(f"  Reunakaista kuormansiirto     paneelikaista → orret → reunakattotuoli (ei suoraa hajakuormaa)")
print(
    f"  Profiilit vasen / oikea       {edge_rafter_section_by_side['left']['profile']} / {edge_rafter_section_by_side['right']['profile']}"
)
if edge_top_notch_ref is not None:
    print(
        f"  Yläpinnan lovet               rect_notch {edge_rect_depth_mm:.0f} × {edge_rect_length_mm:.0f} mm"
        f" @ vasen {', '.join(f'{y_mm:.0f}' for y_mm in edge_purlin_support_ys_mm['left'])} mm"
        f" / oikea {', '.join(f'{y_mm:.0f}' for y_mm in edge_purlin_support_ys_mm['right'])} mm"
    )
else:
    print("  Yläpinnan lovet               ei lovea")
print(
    "  MRd / VRd / g_self,d         "
    + " | ".join(
        f"{side}: {data['MRd_kNm']:.2f} kNm / {data['VRd_kN']:.2f} kN / {gammaG*data['self_kNm']:.3f} kN/m"
        for side, data in edge_rafter_section_by_side.items()
    )
)
print()
print(f"  {'ID':<12} {'P_orret':>8} {'g_self':>7} {'R_in':>8} {'R_out':>8} {'Md':>7} {'η_M':>7} {'η_V':>7} {'η_lovi':>8} {'δ_sls':>9}")
print(f"  {'-'*12} {'-'*8} {'-'*7} {'-'*8} {'-'*8} {'-'*7} {'-'*7} {'-'*7} {'-'*8} {'-'*9}")
for side, row in edge_rafter_design_results.items():
    print(
        f"  {row['id']:<12} {sum(load for _, load in row['point_loads_kN']):>8.2f} {row['g_self_d_kNm']:>7.3f} {row['R_inner_kN']:>8.2f} {row['R_outer_kN']:>8.2f}"
        f" {row['M_gov']['value_kNm']:>6.2f} {row['eta_M']:>6.1f}% {row['eta_V']:>6.1f}% {governing_notch_eta(row):>7.1f}%"
        f" {row['sls_delta_mm']:>5.2f}/{row['delta_lim_mm']:.1f}"
    )
print(f"  Kriittinen reunakattotuoli    {critical_edge_rafter['id']}  ({critical_edge_rafter['case_key']}, η = {critical_edge_rafter['eta_gov']:.1f}%)")
if critical_edge_rafter['top_notch'] is not None:
    print(f"    Yläreunan lovi              {critical_edge_rafter['top_notch']['eta_gov']['mode']} = {critical_edge_rafter['top_notch']['eta_gov']['value_pct']:.1f}%")
if critical_edge_rafter['birdsmouth'] is not None:
    print(f"    Birdsmouth                  {critical_edge_rafter['birdsmouth']['eta_gov']['mode']} = {critical_edge_rafter['birdsmouth']['eta_gov']['value_pct']:.1f}%")
    print(f"    Lovisyvyys h/3              {format_birdsmouth_h3_status(critical_edge_rafter['birdsmouth_rule_ok'])}")
if critical_edge_rafter['top_notch'] is None and critical_edge_rafter['birdsmouth'] is None:
    print("    Lovitarkistus               ei lovea")

print("\n── ULKOPALKKI LP225×140 GL30c ───────────────────────────────────")
print(f"  MRd,y = {OUTER_BEAM_MRd_y_kNm:.2f} kNm,  MRd,z = {OUTER_BEAM_MRd_z_kNm:.2f} kNm,  VRd = {OUTER_BEAM_VRd_kN:.2f} kN")
print(
    f"  Pystymomentti A / B / DRIFT   {RESULTS['ULS A']['outer_beam']['M_gov']['value_kNm']:.2f}"
    f" / {RESULTS['ULS B']['outer_beam']['M_gov']['value_kNm']:.2f} / {RESULTS['ULS DRIFT']['outer_beam']['M_gov']['value_kNm']:.2f} kNm"
)
print(
    f"  Vaakamomentti A / B / DRIFT   {OUTER_BEAM_H['ULS A']['M_gov']['value_kNm']:.2f}"
    f" / {OUTER_BEAM_H['ULS B']['M_gov']['value_kNm']:.2f} / {OUTER_BEAM_H['ULS DRIFT']['M_gov']['value_kNm']:.2f} kNm"
)
print(
    f"  Vuorovaikutus η               A: {outer_beam_interaction['ULS A']:.1f}%  /"
    f"  B: {outer_beam_interaction['ULS B']:.1f}%  /  DRIFT: {outer_beam_interaction['ULS DRIFT']:.1f}%"
    f"  →  {outer_beam_governing_case}"
)
print(
    f"  Taipuma SLS / DRIFT           {outer_beam_sls_normal_delta_mm:.2f} / {outer_beam_sls_drift_delta_mm:.2f}"
    f" / {(outer_supports_x_mm[1]-outer_supports_x_mm[0])/300.0:.1f} mm  {format_ok(outer_beam_sls_delta_mm <= (outer_supports_x_mm[1]-outer_supports_x_mm[0])/300.0)}"
)
print(
    f"  Pystykuormat kattotuoleilta   max {outer_beam_governing_point_load[1]:.2f} kN @ x = {outer_beam_governing_point_load[0]:.0f} mm"
    f" ({outer_beam_governing_case})"
)
print(
    f"  Tulos                         η_v = {max(outer_beam_eta.values()):.1f}%,  η_int = {max(outer_beam_interaction.values()):.1f}%"
    f"  {format_ok(max(outer_beam_eta.values()) <= 100.0 and max(outer_beam_interaction.values()) <= 100.0)}"
)

print("\n── SISÄPALKKI LP315×140 GL30c ────────────────────────────────────")
print(f"  MRd = {INNER_BEAM_MRd_kNm:.2f} kNm,  VRd = {INNER_BEAM_VRd_kN:.2f} kN")
print(f"  Hallitseva tapaus             {inner_beam_governing_case}")
print(f"  Md,max                        {inner_beam_governing['M_gov']['value_kNm']:.2f} kNm @ x = {inner_beam_governing['M_gov']['x_mm']:.0f} mm")
print(f"  Vd,max                        {abs(inner_beam_governing['V_abs']['value_kN']):.2f} kN")
print(f"  Lisäreaktio 2×KP360×51:lle    {inner_beam_governing['reactions_kN'][inner_supports_x_mm[0]]:.2f} / {inner_beam_governing['reactions_kN'][inner_supports_x_mm[1]]:.2f} kN")
print(
    f"  Taipuma SLS / DRIFT           {inner_beam_sls_normal_delta_mm:.2f} / {inner_beam_sls_drift_delta_mm:.2f}"
    f" / {(inner_supports_x_mm[-1]-inner_supports_x_mm[0])/300.0:.1f} mm  {format_ok(inner_beam_sls_delta_mm <= (inner_supports_x_mm[-1]-inner_supports_x_mm[0])/300.0)}"
)
print(f"  Tulos                         η_M = {inner_beam_governing['eta_M']:.1f}%,  η_V = {inner_beam_governing['eta_V']:.1f}%  {format_ok(inner_beam_governing['eta_M'] <= 100.0 and inner_beam_governing['eta_V'] <= 100.0)}")

print("\n── PILARIKUORMAT JA NOSTO ───────────────────────────────────────")
print(f"  Ulkopalkin reaktiot {outer_reaction_case} / UPLIFT [kN]:")
for x_mm in outer_supports_x_mm:
    print(f"    x = {x_mm:>4.0f} mm              {RESULTS[outer_reaction_case]['outer_beam']['reactions_kN'][x_mm]:>6.2f} / {outer_uplift[x_mm]:>6.2f}")
print(f"  Sisäpalkin reaktiot {inner_reaction_case} / UPLIFT [kN]:")
for x_mm in inner_supports_x_mm:
    print(f"    x = {x_mm:>4.0f} mm              {RESULTS[inner_reaction_case]['inner_beam']['reactions_kN'][x_mm]:>6.2f} / {inner_uplift[x_mm]:>6.2f}")
print(f"  Suurin ulkopilaripuristus         {max(RESULTS[outer_reaction_case]['outer_beam']['reactions_kN'].values()):.2f} kN")
print(f"  Suurin ulkopilarin nostotarve     {abs(min(outer_uplift.values())):.2f} kN")
print(f"  Suurin sisätuen nostotarve        {abs(min(inner_uplift.values())):.2f} kN")

print("\n── KOKONAISPILARIKUORMAT TERASSIN JÄLKEEN ───────────────────────")
print(f"  Olemassa oleva baseline        geometry/katos.json + uuden terassin lisäkuormat")
print(f"  Ontelolaatta saumattuna h=150  {terrace_total_column_loads['gk_hollow_slab_kNm2']:.2f} kN/m²")
print(f"  Pintavalu 60 mm                {terrace_total_column_loads['gk_floor_cast_kNm2']:.2f} kN/m²")
print(f"  Terassin hyötykuorma           {terrace_total_column_loads['qk_terrace_live_kNm2']:.2f} kN/m²")
if terrace_total_column_loads["outer_beam_count"] == 1:
    print(f"  Alapalkki 350×300              {terrace_total_column_loads['outer_beam_self_kNm']:.3f} kN/m")
else:
    print(
        f"  Alapalkit {terrace_total_column_loads['outer_beam_count']}×350×300            "
        f"{terrace_total_column_loads['outer_beam_total_self_kN'] / terrace_total_column_loads['outer_beam_count']:.2f}"
        f" kN / palkki  = {terrace_total_column_loads['outer_beam_self_kNm']:.3f} kN/m"
    )
print("  Kuormareitti                   sisäpilarit suoraan perustuksille, ontelolaatat seinästä alapalkille, alapalkki ulompiin pilareihin")
print("  Ontelolaatan päätyreaktio      jaetaan alapalkille laatan leveyden matkalle")
print(
    f"  Portaikon lisä col.x7075       SLS {portaikko_col_x7075_extra_sls:.2f} kN /"
    f" ULS {portaikko_col_x7075_extra_uls:.2f} kN / UPLIFT {portaikko_col_x7075_extra_uplift:.2f} kN"
)
print(f"  N_sls / N_uls / N_min          max(SLS,SLS DRIFT) / max(ULS A, ULS B, ULS DRIFT) / UPLIFT")
print()
print(f"  {'Pilari':<22} {'Ryhmä':<10} {'N_sls':>9} {'N_uls':>9} {'N_min':>9}  {'Tila'}")
print(f"  {'-'*22} {'-'*10} {'-'*9} {'-'*9} {'-'*9}  {'-'*18}")
for column_id in terrace_total_column_loads["column_output_order"]:
    row = terrace_total_column_envelope[column_id]
    status = f"Puristus {row['N_min']:.2f} kN  OK ✓" if row["N_min"] >= 0.0 else f"NOSTO {abs(row['N_min']):.2f} kN"
    print(
        f"  {terrace_total_column_loads['column_display'][column_id]:<22} "
        f"{terrace_total_column_loads['column_group_label'][column_id]:<10} "
        f"{row['N_sls']:>9.2f} {row['N_uls']:>9.2f} {row['N_min']:>9.2f}  {status}"
    )
print(f"  Suurin kokonaispuristus        {max(row['N_uls'] for row in terrace_total_column_envelope.values()):.2f} kN")
print(f"  Suurin kokonaisnostotarve      {max(0.0, max(-row['N_min'] for row in terrace_total_column_envelope.values())):.2f} kN")

print("\n── PERUSTUKSET ─────────────────────────────────────────")
for line in foundation_report_lines(foundation_checks):
    print(line)

print("\n── HUOMIOT ───────────────────────────────────────────────────────")
print("  * Sisäkattotuolit saavat suoran paneelikaistan tributäärialueensa mukaan; reunapaneelin")
print("    sisäpuolikas sisältyy uloimman sisäkattotuolin b_dir-arvoon.")
print("  * Reunapaneelin ulkopuolikas siirtyy geometry/terassi_puu.json:n")
print("    load_transfer.member_refs- ja tributary_width_mm-metadatan mukaisesti orsien kautta;")
print("    reunakattotuoleille ei anneta suoraa paneelikaistan hajakuormaa.")
print("  * Orsien oma paino mallinnetaan viivakuormana.")
print("  * Seinän täytekaistan ulkoreuna tukeutuu erilliselle tukiorrelle; sen")
print("    reaktiot siirretään kattotuoleille, ei aurinkopaneelikentälle.")
print("  * Päätykolmiolasin omapaino lisätään beam.inner.new-pystymalliin; tuulikuorma")
print("    jaetaan konservatiivisesti 50/50 alapalkille ja yläreunan 2×KP360×51-palkille.")
print("  * Ulkokulmien nurkkaorret mallinnetaan tuettuina toiseksi uloimpaan kattotuoliin ja")
print("    ulkopalkin ulkoreunalta kantavina ulokkeina; reunatuki voi siksi olla vetava.")
print("  * Liitosten tuki- ja rotaatiomallit luetaan geometry/terassi_puu.json:n")
print("    connections.analysis-metadatasta; nykygeometriassa orsien tuet ovat niveliä.")
print("  * Orsien ja kattotuolien lovi-/nettoh-tarkistukset luetaan suoraan geometry/terassi_puu.json:n")
print("    cuts-kentistä; terassivertailuskriptin hardkoodattuja loviarvoja ei käytetä.")
print("  * Kinostuma mallinnetaan jäsenkohtaisena muuttuvana s(x,y)-kuormana; taulukon q_avg")
print("    on roof-stripin ekvivalentin viivakuorman pituuspainotettu keskiarvo ennen kehikkosiirtoa.")
print("  * Birdsmouth-loven seat-pituus 570 mm tekee nettoh:n nousun lineaariseksi lovivyöhykkeellä;")
print("    tarkistus tehdään koko loven pituudella, ei vain yhdessä poikkileikkauksessa.")
print(f"  * Sisäkattotuolien sisäpään liitos luetaan metadataan: {format_connection_behavior(inner_hanger_analysis, inner_hanger_rot_k_Nmm_per_rad)}.")
print(
    "  * Reunakattotuolien liitosmallit luetaan metadataan: "
    + " / ".join(
        f"{SIDE_MEMBER_LABEL[side]}: {format_connection_behavior(edge_rafter_support_analysis_by_side[side], edge_rafter_support_rot_k_by_side[side])}"
        for side in ("left", "right")
    )
    + "."
)
print("  * Uplift käyttää suljetun lasituksen imutapausta (w_up_closed); kiinnitykset tulee mitoittaa")
print("    vähintään yllä raportoiduille nostoreaktioille.")
print(DW)
