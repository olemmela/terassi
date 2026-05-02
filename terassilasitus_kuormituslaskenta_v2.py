"""
LASITETUN TERASSIN PUU2-VAIHTOEHTO V2 – KUORMITUSLASKENTA – ETELÄSUOMI
====================================================================
Standardit: EN 1990, EN 1991-1-1, EN 1991-1-3, EN 1991-1-4, EN 1995-1-1

Geometria luetaan tiedostosta geometry/terassi_puu2.json:
  - uusi puu2-ratkaisu: orret 98×48, nurkkaorret 98×48, kattotuolit 198×48,
    ulkopalkki LP225×115, sisäpalkki LP315×115
  - lineaarijäsenten poikkileikkauksen kierto luetaan section_rotation_deg-kentästä
  - aurinkopaneelien reunakaistat siirtyvät reunakattotuoleille vain orsien kautta
  - ulkokulmien nurkkaorret sidotaan uloimpaan orteen ja kantavat ulkopalkin ulkoreunalla
  - liitosten tuki- ja rotaatiomallit luetaan connections.analysis-metadatasta
  - paneelikuorman piste/viivamalli luetaan surfaces[*].load_transfer-metadatasta
  - KP360-siirtovyöhyke luetaan transfer_link-liitospisteistä beam.inner.new ↔ beam.existing.kp360x2
  - kinostuma talon seinää vasten johdetaan geometriasta muuttuvalla h(x)-korkeudella
  - paneelien kinostumakestävyys tarkistetaan 5.40 kN/m² etupuolen rajaa vasten
  - lovi- ja nettoh-tarkistukset luetaan geometry/terassi_puu2.json:n cuts-kentistä
"""

import math

from beam_analysis import (
    combine_uniform_loads,
    intervals_to_uniform_loads,
    load_stats,
    refine_nodes_mm,
    sample_internal_forces,
    sample_max_deflection_mm,
    segment_key,
    solve_linear_system,
    solve_member_response,
    total_uniform_load_kN,
    uniform_loads_for_nodes,
)
from existing_beam_checks import (
    check_existing_kp360_combined,
    check_existing_lp225_x125_combined,
    katos_existing_context,
)
from foundation_checks import foundation_checks_from_envelope, foundation_report_lines
from geometry_loader import expanded_members, load, member, surface, reference, profile_b, profile_h
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
    sample_min_section_height_mm,
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

GEOMETRY_NAME = "terassi_puu2.json"
GEOMETRY_PATH_LABEL = f"geometry/{GEOMETRY_NAME}"

GEO = load(GEOMETRY_NAME)
FOUNDATION_GEO = load("katos.json")
KATOS_EXISTING_CTX = katos_existing_context()
CONNECTIONS = {conn["id"]: conn for conn in GEO["connections"]}
MEMBERS_BY_ID = {}
for group_name in GEO["members"]:
    for member_obj in expanded_members(GEO, group_name):
        MEMBERS_BY_ID[member_obj["id"]] = member_obj


def connection_by_members(member_a_id, member_b_id):
    wanted = {member_a_id, member_b_id}
    for conn in GEO["connections"]:
        if set(conn.get("members", [])) == wanted:
            return conn
    raise KeyError(f"Connection not found for members: {member_a_id}, {member_b_id}")


def member_by_id_any(member_id):
    return MEMBERS_BY_ID[member_id]


def first_connection_matching(member_id, predicate):
    for conn in GEO["connections"]:
        members = conn.get("members", [])
        if member_id in members and predicate(conn, members):
            return conn
    return None


def transfer_link_connections(member_a_id, member_b_id):
    wanted = {member_a_id, member_b_id}
    return sorted(
        [
            conn
            for conn in GEO["connections"]
            if conn.get("type") == "transfer_link" and set(conn.get("members", [])) == wanted
        ],
        key=lambda conn: float(conn["at"]["x"]),
    )


def member_column_connections(member_id, column_prefix=None):
    conns = []
    for conn in GEO["connections"]:
        members = conn.get("members", [])
        if member_id not in members:
            continue
        other = next((other_id for other_id in members if other_id != member_id), None)
        if other is None or not other.startswith("col."):
            continue
        if column_prefix is not None and not other.startswith(column_prefix):
            continue
        conns.append(conn)
    return sorted(
        conns,
        key=lambda conn: float(member_by_id_any(connection_other_member_id(conn, member_id))["base"]["x"]),
    )


def connection_cut(connection_id, kind=None):
    cuts = CONNECTIONS[connection_id].get("cuts", [])
    if kind is None:
        return cuts[0] if cuts else None
    for cut in cuts:
        if cut.get("kind") == kind:
            return cut
    return None


def rect_notch_info(connection_id):
    cut = connection_cut(connection_id, "rect_notch")
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
roof_inner_pts = sorted([p for p in roof_poly if p["y"] == roof_y0_mm], key=lambda p: p["x"])
roof_inner_x0_mm = roof_inner_pts[0]["x"]
roof_inner_x1_mm = roof_inner_pts[-1]["x"]
roof_inner_z0_mm = roof_inner_pts[0]["z"]
roof_inner_z1_mm = roof_inner_pts[-1]["z"]
panel_joint_y_mm = 0.5 * (roof_y0_mm + roof_y1_mm)
panel_frame_edge_offset_mm = 15.0
DEFAULT_ROOF_LOAD_TRANSFER_RULE = {
    "model": "point",
    "reference": "axis_end",
    "offset_mm": -panel_frame_edge_offset_mm,
}


def roof_load_transfer_rule(member_id):
    load_transfer = roof.get("load_transfer", {})
    for rule in load_transfer.get("to_members", []):
        if member_id in rule.get("member_refs", []):
            return merge_analysis_dict(DEFAULT_ROOF_LOAD_TRANSFER_RULE, rule)
    return dict(DEFAULT_ROOF_LOAD_TRANSFER_RULE)


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
        return f"kuorma pisteenä {offset_mm:+.0f} mm {ref_label}; omapaino viivakuormana"
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


def edge_rafter_inner_support_connection(side):
    rafter_id = edge_rafter_id(side)
    purlin_root = side_purlin_root_id(side)
    connection_obj = first_connection_matching(
        rafter_id,
        lambda conn, members: (
            next((other for other in members if other != rafter_id), None) not in {None, "beam.outer"}
            and not next((other for other in members if other != rafter_id), "").startswith(purlin_root)
        ),
    )
    if connection_obj is None:
        raise KeyError(f"Inner support connection not found for {rafter_id}")
    return connection_obj


def edge_rafter_outer_support_connection(side):
    return connection_by_members(edge_rafter_id(side), "beam.outer")


rafters_all = sorted(expanded_members(GEO, "rafters"), key=lambda item: (float(item["axis_start"]["x"]), item["id"]))
interior_rafters = [m for m in rafters_all if m["id"].startswith(INTERIOR_RAFTER_PREFIX) and m["id"].split(".")[-1].isdigit()]
edge_rafters = {
    "left": member(GEO, "rafters", edge_rafter_id("left")),
    "right": member(GEO, "rafters", edge_rafter_id("right")),
}

purlins_all = sorted(expanded_members(GEO, "purlins"), key=lambda item: item["id"])


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
existing_support_beam = member(GEO, "beams", "beam.lp225.x125")
existing_beam = member(GEO, "beams", "beam.existing.kp360x2")
inner_beam_transfer_links = transfer_link_connections("beam.inner.new", "beam.existing.kp360x2")
inner_column_ids = sorted(
    [
        column_obj["id"]
        for column_obj in expanded_members(GEO, "columns")
        if column_obj["id"].startswith("col.existing.inner.")
    ],
    key=lambda cid: float(member(GEO, "columns", cid)["base"]["x"]),
)
inner_supports_x_mm = [
    float(member(GEO, "columns", cid)["base"]["x"])
    for cid in inner_column_ids
]
inner_beam_support_connections = member_column_connections("beam.inner.new", "col.existing.inner.")
if not inner_beam_support_connections:
    raise ValueError(
        f"Expected at least 1 existing inner column support connection for {inner_beam['id']}, got 0"
    )
inner_beam_direct_support_rows = sorted(
    [
        {
            "support_x_mm": float(conn["at"]["x"]),
            "column_x_mm": float(member_by_id_any(next((other_id for other_id in conn.get("members", []) if other_id != "beam.inner.new"), None))["base"]["x"]),
            "connection_id": conn["id"],
        }
        for conn in inner_beam_support_connections
    ],
    key=lambda row: row["support_x_mm"],
)
inner_beam_direct_supports_x_mm = [row["support_x_mm"] for row in inner_beam_direct_support_rows]
inner_beam_support_column_x_by_support_x = {
    row["support_x_mm"]: row["column_x_mm"]
    for row in inner_beam_direct_support_rows
}
existing_support_beam_column_y_mm = KATOS_EXISTING_CTX["lp225_x125"]["support_right_y_mm"]

outer_supports_x_mm = sorted(float(member(GEO, "columns", cid)["base"]["x"]) for cid in ("col.outer.x0", "col.outer.x3600", "col.outer.x7200"))

inner_beam_fit_notch_connection = next(
    (
        conn
        for conn in GEO["connections"]
        if conn.get("type") == "notched_over"
        and set(conn.get("members", [])) == {"beam.inner.new", "beam.existing.kp360x2"}
    ),
    None,
)
inner_beam_fit_rect_info = (
    {
        "active": False,
        "depth_mm": 0.0,
        "length_mm": 0.0,
        "offset_mm": 0.0,
        "reference": None,
        "side": None,
    }
    if inner_beam_fit_notch_connection is None
    else rect_notch_info(inner_beam_fit_notch_connection["id"])
)
inner_beam_fit_bevel_info = (
    {
        "active": False,
        "depth_mm": 0.0,
        "length_mm": 0.0,
        "offset_mm": 0.0,
        "reference": None,
        "side": None,
    }
    if inner_beam_fit_notch_connection is None
    else bevel_notch_info(inner_beam_fit_notch_connection["id"])
)


def make_inner_beam_fit_notch_depth_functions():
    start_x_mm = float(inner_beam["axis_start"]["x"])
    end_x_mm = float(inner_beam["axis_end"]["x"])
    member_axis_positive_sign = 1.0 if end_x_mm >= start_x_mm else -1.0
    depth_functions = []
    zones_mm = []
    for info, builder in (
        (inner_beam_fit_rect_info, make_end_referenced_rect_notch_depth_fn),
        (inner_beam_fit_bevel_info, make_end_referenced_bevel_notch_depth_fn),
    ):
        if not info["active"]:
            continue
        if info["reference"] not in {"axis_start", "axis_end"}:
            raise ValueError(f"Unsupported inner beam notch reference: {info['reference']}")
        notch_end_x_mm = start_x_mm if info["reference"] == "axis_start" else end_x_mm
        inward_positive_sign = member_axis_positive_sign if info["reference"] == "axis_start" else -member_axis_positive_sign
        zone_mm, depth_fn, active = builder(info, notch_end_x_mm, inward_positive_sign)
        if active:
            zones_mm.append(zone_mm)
            depth_functions.append(depth_fn)
    return zones_mm, depth_functions

interior_rafter_xs_mm = [float(m["axis_start"]["x"]) for m in interior_rafters]
interior_direct_ranges_mm = tributary_ranges_mm(interior_rafter_xs_mm, interior_rafter_xs_mm[0], interior_rafter_xs_mm[-1])
interior_direct_widths_m = [(b_mm - a_mm) / 1000.0 for a_mm, b_mm in interior_direct_ranges_mm]
left_strip_width_m = (interior_rafter_xs_mm[0] - roof_x0_mm) / 1000.0
right_strip_width_m = (roof_x1_mm - interior_rafter_xs_mm[-1]) / 1000.0

purlin_y_positions_mm = {
    side: [float(m["axis_start"]["y"]) for m in group]
    for side, group in purlins_by_side.items()
}
purlin_y_boundary_positions_mm = {}
purlin_trib_ranges_mm = {}
purlin_trib_heights_m = {}
for side, group in purlins_by_side.items():
    positions_mm = [float(m["axis_start"]["y"]) for m in group]
    corner_boundary_candidates_mm = [
        min(float(m["axis_start"]["y"]), float(m["axis_end"]["y"]))
        for m in corner_purlins_by_side.get(side, [])
    ]
    boundary_positions_mm = positions_mm[:]
    if corner_boundary_candidates_mm:
        first_corner_y_mm = min(corner_boundary_candidates_mm)
        if all(abs(first_corner_y_mm - y_mm) > 1e-6 for y_mm in boundary_positions_mm):
            boundary_positions_mm.append(first_corner_y_mm)
    boundary_positions_mm = sorted(boundary_positions_mm)
    purlin_y_boundary_positions_mm[side] = boundary_positions_mm
    all_ranges_mm = tributary_ranges_mm(boundary_positions_mm, roof_y0_mm, roof_y1_mm)
    purlin_trib_ranges_mm[side] = all_ranges_mm[: len(positions_mm)]
    purlin_trib_heights_m[side] = [(b_mm - a_mm) / 1000.0 for a_mm, b_mm in purlin_trib_ranges_mm[side]]

rafter_b_mm = profile_b(interior_rafters[0])
rafter_h_mm = profile_h(interior_rafters[0])
purlin_b_mm = profile_b(left_purlins[0])
purlin_h_mm = profile_h(left_purlins[0])
outer_beam_b_mm = profile_b(outer_beam)
outer_beam_h_mm = profile_h(outer_beam)
inner_beam_b_mm = profile_b(inner_beam)
inner_beam_h_mm = profile_h(inner_beam)
existing_beam_b_mm = profile_b(existing_beam)
existing_beam_h_mm = profile_h(existing_beam)

rafter_axis_step = 1.0
left_purlin_axis_step = -1.0
right_purlin_axis_step = 1.0

edge_rafter_inner_support_connection_by_side = {
    side: edge_rafter_inner_support_connection(side)
    for side in ("left", "right")
}
edge_rafter_outer_support_connection_by_side = {
    side: edge_rafter_outer_support_connection(side)
    for side in ("left", "right")
}
edge_rafter_inner_support_member_by_side = {
    side: connection_other_member_id(edge_rafter_inner_support_connection_by_side[side], edge_rafter_id(side))
    for side in ("left", "right")
}

interior_inner_support_y_mm = float(connection_support_point(CONNECTIONS["con.kattotuoli.on.inner_beam"], "kattotuoli.0")["y"])
interior_outer_support_y_mm = float(connection_support_point(CONNECTIONS["con.kattotuoli.on.outer_beam"], "kattotuoli.0")["y"])
edge_inner_support_y_mm_by_side = {
    side: float(connection_support_point(edge_rafter_inner_support_connection_by_side[side], edge_rafter_id(side))["y"])
    for side in ("left", "right")
}
edge_outer_support_y_mm_by_side = {
    side: float(connection_support_point(edge_rafter_outer_support_connection_by_side[side], edge_rafter_id(side))["y"])
    for side in ("left", "right")
}
rafter_analysis_start_y_mm = min(interior_inner_support_y_mm, min(float(m["axis_start"]["y"]) for m in rafters_all))
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
    "left": rect_notch_info("con.kattotuoli.vasen.on.orsi.vasen"),
    "right": rect_notch_info("con.kattotuoli.oikea.on.orsi.oikea"),
}
edge_top_notch_ref = next((item for item in edge_top_notch_info.values() if item["active"]), None)
edge_rect_depth_mm = edge_top_notch_ref["depth_mm"] if edge_top_notch_ref else 0.0
edge_rect_length_mm = edge_top_notch_ref["length_mm"] if edge_top_notch_ref else 0.0
edge_purlin_support_ys_mm = {side: positions_mm[:] for side, positions_mm in purlin_y_positions_mm.items()}
interior_purlin_support_ys_mm = {side: positions_mm[:] for side, positions_mm in purlin_y_positions_mm.items()}

purlin_inner_notch_info = {
    "left": bevel_notch_info("con.orsi.vasen.on.kattotuoli.0"),
    "right": bevel_notch_info("con.orsi.oikea.on.kattotuoli.5"),
}
purlin_edge_notch_info = {
    "left": rect_notch_info("con.orsi.vasen.on.kattotuoli.vasen"),
    "right": rect_notch_info("con.orsi.oikea.on.kattotuoli.oikea"),
}
purlin_edge_notch_ref = next((item for item in purlin_edge_notch_info.values() if item["active"]), None)
purlin_edge_notch_depth_mm = purlin_edge_notch_ref["depth_mm"] if purlin_edge_notch_ref else 0.0
purlin_edge_notch_length_mm = purlin_edge_notch_ref["length_mm"] if purlin_edge_notch_ref else 0.0
left_purlin_inner_support_conn = CONNECTIONS["con.orsi.vasen.on.kattotuoli.0"]
right_purlin_inner_support_conn = CONNECTIONS["con.orsi.oikea.on.kattotuoli.5"]
left_purlin_edge_support_conn = CONNECTIONS["con.orsi.vasen.on.kattotuoli.vasen"]
right_purlin_edge_support_conn = CONNECTIONS["con.orsi.oikea.on.kattotuoli.oikea"]
left_purlin_support_x_mm = float(connection_support_point(left_purlin_inner_support_conn, side_purlin_root_id("left"))["x"])
right_purlin_support_x_mm = float(connection_support_point(right_purlin_inner_support_conn, side_purlin_root_id("right"))["x"])
left_purlin_edge_support_center_x_mm = float(connection_support_point(left_purlin_edge_support_conn, side_purlin_root_id("left"), "support_centerline")["x"])
right_purlin_edge_support_center_x_mm = float(connection_support_point(right_purlin_edge_support_conn, side_purlin_root_id("right"), "support_centerline")["x"])
corner_purlin_inner_support_connections = {}
corner_purlin_inner_support_points = {}
corner_purlin_inner_support_member_ids = {}
corner_purlin_outer_support_connections = {}
corner_purlin_outer_support_points = {}
corner_purlin_outer_support_member_ids = {}
corner_purlin_outer_notch_info = {}
corner_purlin_trib_width_m = {}
side_corner_strip_outer_x_mm = {"left": roof_x0_mm, "right": roof_x1_mm}
side_corner_strip_inner_x_mm = {"left": interior_rafter_xs_mm[0], "right": interior_rafter_xs_mm[-1]}
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
    outer_x_mm = side_corner_strip_outer_x_mm[side]
    inner_x_mm = side_corner_strip_inner_x_mm[side]
    strip_width_mm = abs(inner_x_mm - outer_x_mm)
    ordered = sorted(
        members,
        key=lambda member_obj: abs(corner_purlin_outer_support_points[member_obj["id"]]["x"] - outer_x_mm),
    )
    corner_purlins_by_side[side] = ordered
    support_offsets_mm = [abs(corner_purlin_outer_support_points[member_obj["id"]]["x"] - outer_x_mm) for member_obj in ordered]
    for member_obj, (a_mm, b_mm) in zip(ordered, tributary_ranges_mm(support_offsets_mm, 0.0, strip_width_mm)):
        corner_purlin_trib_width_m[member_obj["id"]] = (b_mm - a_mm) / 1000.0

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
panels_total_kN = panel_count_total * panel_mass_kg * 9.81 / 1000.0
gk_panels = panels_total_kN / roof_area_m2
gk_fixings = 0.05
gk_roofing = gk_panels + gk_fixings

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

rho_c24 = 420.0
gamma_c24 = rho_c24 * 9.81 / 1000.0
rho_lvl = 480.0
gamma_gl30c = 5.0

rafter_self_kNm = (rafter_b_mm / 1000.0) * (rafter_h_mm / 1000.0) * gamma_c24 / math.cos(roof_slope_rad)
purlin_self_kNm = (purlin_b_mm / 1000.0) * (purlin_h_mm / 1000.0) * gamma_c24
outer_beam_self_kNm = (outer_beam_b_mm / 1000.0) * (outer_beam_h_mm / 1000.0) * gamma_gl30c
inner_beam_self_kNm = (inner_beam_b_mm / 1000.0) * (inner_beam_h_mm / 1000.0) * gamma_gl30c

E_c24 = 11000.0
E_gl30c = 13000.0
E_lvl = 13800.0
kmod_c24 = 0.8
gammaM_c24 = 1.3
fm_d_c24 = kmod_c24 * 24.0 / gammaM_c24
fv_d_c24 = kmod_c24 * 4.0 / gammaM_c24
kmod_gl30c = 0.8
gammaM_gl30c = 1.25
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
    side: connection_analysis_info(edge_rafter_inner_support_connection_by_side[side]["id"])
    for side in ("left", "right")
}
edge_rafter_support_rot_k_by_side = {
    side: connection_rotational_spring_k_Nmm_per_rad(edge_rafter_inner_support_connection_by_side[side]["id"])
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


def material_density_kg_m3(member_obj):
    material = member_obj["profile"].get("material", "")
    if "Kerto-S LVL" in material:
        return rho_lvl
    if "Liimapuu GL30c" in material:
        return 500.0
    if "Mitallistettu sahatavara" in material:
        return rho_c24
    raise ValueError(f"Unsupported material for density lookup: {material}")


def material_gammaM(member_obj):
    material = member_obj["profile"].get("material", "")
    if "Kerto-S LVL" in material:
        return gammaM_lvl
    if "Liimapuu GL30c" in material:
        return gammaM_gl30c
    if "Mitallistettu sahatavara" in material:
        return gammaM_c24
    raise ValueError(f"Unsupported material for gammaM lookup: {material}")


def fastener_stress_area_mm2(d_mm):
    stress_area_by_d = {10: 58.0, 12: 84.3, 16: 157.0}
    d_key = int(round(float(d_mm)))
    if d_key not in stress_area_by_d:
        raise ValueError(f"Unsupported fastener diameter for stress area lookup: M{d_key}")
    return stress_area_by_d[d_key]


def fastener_fu_Nmm2(grade):
    if grade in {"4.6", "5.8"}:
        return float(grade.split(".")[0]) * 100.0
    if grade == "8.8":
        return 800.0
    if grade == "10.9":
        return 1000.0
    raise ValueError(f"Unsupported fastener grade: {grade}")


def transfer_link_face_count(transfer_info):
    return max(
        1,
        int(float(transfer_info["outer_plate_thickness_mm"]) > 1e-9)
        + int(float(transfer_info["inner_plate_thickness_mm"]) > 1e-9),
    )


def fastener_embedment_fh_d_Nmm2(member_obj, d_mm):
    rho_kg_m3 = material_density_kg_m3(member_obj)
    gamma_m = material_gammaM(member_obj)
    fh_k = 0.082 * (1.0 - 0.01 * float(d_mm)) * rho_kg_m3
    return 0.8 * fh_k / gamma_m


def transfer_link_member_group_stiffness_N_per_mm(connection_obj, member_obj):
    transfer_info = connection_obj["transfer"]
    face_count = transfer_link_face_count(transfer_info)
    d_mm = float(transfer_info["fastener_d_mm"])
    n_fasteners = int(transfer_info["fastener_count_per_member"])
    kser_per_fastener_N_per_mm = material_density_kg_m3(member_obj) ** 1.5 * d_mm / 23.0
    return face_count * n_fasteners * kser_per_fastener_N_per_mm


def transfer_link_spring_k_N_per_mm(connection_obj):
    member_a = member_by_id_any(connection_obj["members"][0])
    member_b = member_by_id_any(connection_obj["members"][1])
    k_a = transfer_link_member_group_stiffness_N_per_mm(connection_obj, member_a)
    k_b = transfer_link_member_group_stiffness_N_per_mm(connection_obj, member_b)
    if k_a <= 1e-9 or k_b <= 1e-9:
        return 0.0
    return (k_a * k_b) / (k_a + k_b)


def transfer_link_member_capacity_kN(connection_obj, member_obj):
    transfer_info = connection_obj["transfer"]
    face_count = transfer_link_face_count(transfer_info)
    d_mm = float(transfer_info["fastener_d_mm"])
    grade = transfer_info["fastener_grade"]
    n_fasteners = int(transfer_info["fastener_count_per_member"])
    bolt_steel_kN = 0.6 * fastener_fu_Nmm2(grade) * fastener_stress_area_mm2(d_mm) / 1.25 / 1000.0
    timber_embed_kN = fastener_embedment_fh_d_Nmm2(member_obj, d_mm) * d_mm * profile_b(member_obj) / 1000.0
    return face_count * n_fasteners * min(bolt_steel_kN, timber_embed_kN)


def transfer_link_plate_shear_capacity_kN(connection_obj):
    transfer_info = connection_obj["transfer"]
    t_total_mm = float(transfer_info["outer_plate_thickness_mm"]) + float(transfer_info["inner_plate_thickness_mm"])
    strip_width_mm = float(transfer_info["strip_width_mm"])
    if t_total_mm <= 1e-9:
        return 0.0
    fy_steel_Nmm2 = 355.0
    return fy_steel_Nmm2 / math.sqrt(3.0) * t_total_mm * strip_width_mm / 1000.0

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
    ls_m, _, _, _, s_peak_kNm2 = snow_drift_params(h_local_m, roof_depth_m, sk, mu1_=mu1)
    distance_m = max(0.0, (y_mm - roof_y0_mm) / 1000.0)
    s_drift_local = s_peak_kNm2 * max(0.0, 1.0 - distance_m / ls_m)
    return max(s_roof, s_drift_local)


def drift_snow_kNm2_from_height(h_m, y_offset_mm):
    if h_m <= 1e-9:
        return s_roof
    ls_m, _, _, _, s_peak_kNm2 = snow_drift_params(h_m, roof_depth_m, sk, mu1_=mu1)
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


def existing_structure_case_key(case_key):
    if case_key == "UPLIFT":
        return "UPLIFT"
    if case_key in SLS_CASE_KEYS:
        return "SLS"
    return "ULS"

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
        "s_inner_edge_rafter_kNm2": max(
            drift_snow_kNm2(x_mm, edge_inner_support_y_mm_by_side["left"]),
            drift_snow_kNm2(x_mm, edge_inner_support_y_mm_by_side["right"]),
        ),
    })
critical_drift = max(DRIFT_SUMMARY, key=lambda item: (item["s_peak_kNm2"], item["h_m"], item["x_mm"]))


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
    ls_m, mu2, mu2_h, s_base_kNm2, s_peak_kNm2 = snow_drift_params(h_local_m, roof_depth_m, sk, mu1_=mu1)
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
critical_panel_y_offset_mm = critical_panel_check["y_mm"] - roof_y0_mm
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


def make_end_referenced_rect_notch_depth_fn(info, end_coord_mm, inward_positive_sign):
    if not info["active"]:
        def depth_fn(_coord_mm):
            return 0.0
        return (end_coord_mm, end_coord_mm), depth_fn, False

    if info["reference"] not in {"axis_start", "axis_end"}:
        raise ValueError(f"Unsupported rect notch reference: {info['reference']}")

    offset_mm = info["offset_mm"]
    length_mm = info["length_mm"]
    zone = local_interval_to_global(end_coord_mm, offset_mm, length_mm, inward_positive_sign)

    def depth_fn(coord_mm):
        local_mm = inward_positive_sign * (coord_mm - end_coord_mm)
        if local_mm < offset_mm - 1e-9 or local_mm > offset_mm + length_mm + 1e-9:
            return 0.0
        return info["depth_mm"]

    return zone, depth_fn, True


def make_purlin_notch_depth_fn(member_obj, side):
    info = purlin_inner_notch_info[side]
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


def make_purlin_edge_notch_depth_fn(side):
    info = purlin_edge_notch_info[side]
    support_x_mm = left_purlin_edge_support_center_x_mm if side == "left" else right_purlin_edge_support_center_x_mm
    if not info["active"]:
        def depth_fn(_x_mm):
            return 0.0
        return (support_x_mm, support_x_mm), depth_fn, False

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


inner_beam_fit_notch_zones_mm, inner_beam_fit_notch_depth_functions = make_inner_beam_fit_notch_depth_functions()


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
OUTER_BEAM_PROPS = member_rect_props(outer_beam_b_mm, outer_beam_h_mm, member_section_rotation_deg(outer_beam))
INNER_BEAM_PROPS = member_rect_props(inner_beam_b_mm, inner_beam_h_mm, member_section_rotation_deg(inner_beam))
EXISTING_BEAM_PROPS = member_rect_props(existing_beam_b_mm, existing_beam_h_mm, member_section_rotation_deg(existing_beam))

RAfter_MRd_kNm = fm_d_c24 * RAfter_PROPS["W_mm3"] / 1.0e6
RAfter_VRd_kN = fv_d_c24 * RAfter_PROPS["A_mm2"] / 1.5e3
PURLIN_MRd_kNm = fm_d_c24 * PURLIN_PROPS["W_mm3"] / 1.0e6
PURLIN_VRd_kN = fv_d_c24 * PURLIN_PROPS["A_mm2"] / 1.5e3
OUTER_BEAM_MRd_y_kNm = fm_d_gl30c * OUTER_BEAM_PROPS["W_mm3"] / 1.0e6
OUTER_BEAM_VRd_kN = fv_d_gl30c * OUTER_BEAM_PROPS["A_mm2"] / 1.5e3
OUTER_BEAM_MRd_z_kNm = fm_d_gl30c * OUTER_BEAM_PROPS["W_horizontal_mm3"] / 1.0e6
INNER_BEAM_MRd_kNm = fm_d_gl30c * INNER_BEAM_PROPS["W_mm3"] / 1.0e6
INNER_BEAM_VRd_kN = fv_d_gl30c * INNER_BEAM_PROPS["A_mm2"] / 1.5e3
EXISTING_BEAM_MRd_kNm = fm_d_lvl * EXISTING_BEAM_PROPS["W_mm3"] / 1.0e6
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

left_purlin_edge_support_line_x_mm = float(connection_support_point(left_purlin_edge_support_conn, side_purlin_root_id("left"), "support_outer_edge")["x"])
right_purlin_edge_support_line_x_mm = float(connection_support_point(right_purlin_edge_support_conn, side_purlin_root_id("right"), "support_outer_edge")["x"])


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
    axis_start_x_mm = float(member_obj["axis_start"]["x"])
    axis_end_x_mm = float(member_obj["axis_end"]["x"])
    x0_mm = min(float(member_obj["axis_start"]["x"]), float(member_obj["axis_end"]["x"]))
    x1_mm = max(float(member_obj["axis_start"]["x"]), float(member_obj["axis_end"]["x"]))
    member_y_mm = float(member_obj["axis_start"]["y"])
    member_props = member_rect_props(purlin_b_mm, purlin_h_mm, member_section_rotation_deg(member_obj))
    member_MRd_kNm = fm_d_c24 * member_props["W_mm3"] / 1.0e6
    member_VRd_kN = fv_d_c24 * member_props["A_mm2"] / 1.5e3
    if side == "left":
        edge_support_x_mm = left_purlin_edge_support_line_x_mm
        supports = [edge_support_x_mm, left_purlin_support_x_mm]
    else:
        edge_support_x_mm = right_purlin_edge_support_line_x_mm
        supports = [right_purlin_support_x_mm, edge_support_x_mm]

    inner_notch_zone_mm, inner_notch_depth_fn, inner_notch_active = make_purlin_notch_depth_fn(member_obj, side)
    edge_notch_zone_mm, edge_notch_depth_fn, edge_notch_active = make_purlin_edge_notch_depth_fn(side)
    depth_functions = [inner_notch_depth_fn] if inner_notch_active else []
    if edge_notch_active:
        depth_functions.append(edge_notch_depth_fn)
    section_h_fn = combined_section_h(purlin_h_mm, depth_functions)

    outer_edge_x_mm = x0_mm if side == "left" else x1_mm
    support_center_x_mm = left_purlin_edge_support_center_x_mm if side == "left" else right_purlin_edge_support_center_x_mm
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
    roof_uniform = roof_area_uniform_loads(nodes_mm, member_y_mm, roof_area_kNm2_at, trib_height_m, axis="x")
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
        notch_candidates.append({"label": bevel_notch_label(purlin_inner_notch_info[side]), **inner_notch})
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
        notch_candidates.append({"label": "edge_rect", **edge_notch})
    notch = max(notch_candidates, key=lambda item: item["eta_gov"]["value_pct"])

    interior_support_x_mm = left_purlin_support_x_mm if side == "left" else right_purlin_support_x_mm
    return {
        "id": member_obj["id"],
        "trib_height_m": trib_height_m,
        "q_line_kNm": q_line_stats["avg_kNm"],
        "q_line_min_kNm": q_line_stats["min_kNm"],
        "q_line_max_kNm": q_line_stats["max_kNm"],
        "panel_load_mode": panel_load_mode,
        "panel_point_load_kN": panel_point_load_kN,
        "M_gov": moment_gov,
        "V_abs": internal["V_abs"],
        "delta": delta,
        "delta_lim_mm": abs(interior_support_x_mm - edge_support_x_mm) / 300.0,
        "section_rotation_deg": member_props["section_rotation_deg"],
        "MRd_kNm": member_MRd_kNm,
        "VRd_kN": member_VRd_kN,
        "R_edge_kN": response["reactions_kN"][edge_support_x_mm],
        "R_inner_kN": response["reactions_kN"][interior_support_x_mm],
        "eta_M": moment_gov["value_kNm"] / member_MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / member_VRd_kN * 100.0,
        "notch": notch,
        "inner_notch": inner_notch,
        "edge_notch": edge_notch,
        "notch_zones_mm": {"inner": inner_notch_zone_mm, "edge": edge_notch_zone_mm if edge_notch_active else None},
        "h_net_min_mm": purlin_h_mm - max(purlin_inner_notch_info[side]["depth_mm"], purlin_edge_notch_info[side]["depth_mm"]),
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
    side = None
    if edge:
        side = "left" if member_obj["id"] == edge_rafter_id("left") else "right"
        inner_support_y_mm = edge_inner_support_y_mm_by_side[side]
        outer_support_y_mm = edge_outer_support_y_mm_by_side[side]
    else:
        inner_support_y_mm = interior_inner_support_y_mm
        outer_support_y_mm = interior_outer_support_y_mm
    section_rotation = member_section_rotation_deg(member_obj)
    member_E_Nmm2 = E_gl30c if edge else E_c24
    member_fm_d_Nmm2 = fm_d_gl30c if edge else fm_d_c24
    member_fv_d_Nmm2 = fv_d_gl30c if edge else fv_d_c24
    if edge:
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


def analyse_beam_case(member_obj, support_xs_mm, point_loads_kN, gamma_self, E_Nmm2, section_I_mm4, MRd_kNm, VRd_kN, EI_by_segment_Nmm2=None, nodes_mm_override=None):
    x0_mm = float(member_obj["axis_start"]["x"])
    x1_mm = float(member_obj["axis_end"]["x"])
    if member_obj["id"] == "beam.outer":
        self_kNm = outer_beam_self_kNm
    else:
        self_kNm = inner_beam_self_kNm
    nodes_mm = (
        list(nodes_mm_override)
        if nodes_mm_override is not None
        else refine_nodes_mm([x0_mm, x1_mm, *support_xs_mm, *[x_mm for x_mm, _ in point_loads_kN]], analysis_step_beam_mm)
    )
    uniform = uniform_loads_for_nodes(nodes_mm, gamma_self * self_kNm / 1000.0)
    response, internal, delta = solve_member_response(
        nodes_mm,
        support_xs_mm,
        point_loads_kN,
        uniform,
        EI_Nmm2=E_Nmm2 * section_I_mm4,
        EI_by_segment_Nmm2=EI_by_segment_Nmm2,
    )
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
        "response": response,
    }


def solve_coupled_beam_responses(beam_specs, connector_springs):
    prepared_specs = []
    connector_xs_by_beam = {}
    for connector in connector_springs:
        connector_xs_by_beam.setdefault(connector["beam_a_id"], []).append(float(connector["x_mm"]))
        connector_xs_by_beam.setdefault(connector["beam_b_id"], []).append(float(connector["x_mm"]))

    dof_offset = 0
    for beam_spec in beam_specs:
        connector_xs_mm = connector_xs_by_beam.get(beam_spec["id"], [])
        nodes = sorted(
            {float(x_mm) for x_mm in beam_spec["nodes_mm"]}
            | {float(x_mm) for x_mm in beam_spec["supports_mm"]}
            | {float(x_mm) for x_mm, _ in beam_spec["point_loads_kN"]}
            | {float(a_mm) for a_mm, _, _ in beam_spec["uniform_loads_kN_per_mm"]}
            | {float(b_mm) for _, b_mm, _ in beam_spec["uniform_loads_kN_per_mm"]}
            | {float(x_mm) for x_mm in connector_xs_mm}
        )
        prepared_specs.append({
            **beam_spec,
            "nodes": nodes,
            "node_index": {x_mm: i for i, x_mm in enumerate(nodes)},
            "dof_offset": dof_offset,
            "element_data": [],
        })
        dof_offset += 2 * len(nodes)

    spec_by_id = {spec["id"]: spec for spec in prepared_specs}
    n_dof = dof_offset
    K = [[0.0] * n_dof for _ in range(n_dof)]
    F = [0.0] * n_dof
    fixed = set()

    for spec in prepared_specs:
        nodes = spec["nodes"]
        node_index = spec["node_index"]
        dof_base = spec["dof_offset"]
        q_by_segment = {
            segment_key(a_mm, b_mm): float(q_kN_per_mm)
            for a_mm, b_mm, q_kN_per_mm in spec["uniform_loads_kN_per_mm"]
        }

        for x_mm, p_kN in spec["point_loads_kN"]:
            F[dof_base + 2 * node_index[float(x_mm)]] -= p_kN * 1000.0

        for elem_i in range(len(nodes) - 1):
            x0_mm = nodes[elem_i]
            x1_mm = nodes[elem_i + 1]
            L_mm = x1_mm - x0_mm
            EI_by_segment_Nmm2 = spec.get("EI_by_segment_Nmm2")
            EI_elem_Nmm2 = float(spec["EI_Nmm2"]) if EI_by_segment_Nmm2 is None else float(EI_by_segment_Nmm2[elem_i])
            fac = EI_elem_Nmm2 / (L_mm**3)
            k = [
                [12.0 * fac, 6.0 * L_mm * fac, -12.0 * fac, 6.0 * L_mm * fac],
                [6.0 * L_mm * fac, 4.0 * L_mm * L_mm * fac, -6.0 * L_mm * fac, 2.0 * L_mm * L_mm * fac],
                [-12.0 * fac, -6.0 * L_mm * fac, 12.0 * fac, -6.0 * L_mm * fac],
                [6.0 * L_mm * fac, 2.0 * L_mm * L_mm * fac, -6.0 * L_mm * fac, 4.0 * L_mm * L_mm * fac],
            ]
            dofs = [
                dof_base + 2 * elem_i,
                dof_base + 2 * elem_i + 1,
                dof_base + 2 * (elem_i + 1),
                dof_base + 2 * (elem_i + 1) + 1,
            ]
            for a_i, I in enumerate(dofs):
                for b_i, J in enumerate(dofs):
                    K[I][J] += k[a_i][b_i]

            q_kN_per_mm = q_by_segment.get(segment_key(x0_mm, x1_mm), 0.0)
            if abs(q_kN_per_mm) > 0.0:
                w_N_per_mm = q_kN_per_mm * 1000.0
                fe = [
                    -w_N_per_mm * L_mm / 2.0,
                    -w_N_per_mm * L_mm * L_mm / 12.0,
                    -w_N_per_mm * L_mm / 2.0,
                    w_N_per_mm * L_mm * L_mm / 12.0,
                ]
                for a_i, I in enumerate(dofs):
                    F[I] += fe[a_i]
            else:
                fe = [0.0, 0.0, 0.0, 0.0]

            spec["element_data"].append({
                "x0_mm": x0_mm,
                "x1_mm": x1_mm,
                "dofs": dofs,
                "k": k,
                "fe": fe,
                "q_kN_per_mm": q_kN_per_mm,
            })

        for x_mm, k_theta in spec.get("rotational_springs_Nmm_per_rad", {}).items():
            x_key = float(x_mm)
            if x_key not in node_index:
                continue
            rot_dof = dof_base + 2 * node_index[x_key] + 1
            K[rot_dof][rot_dof] += float(k_theta)

        fixed.update(dof_base + 2 * node_index[float(x_mm)] for x_mm in spec["supports_mm"])

    prepared_connectors = []
    for connector in connector_springs:
        spec_a = spec_by_id[connector["beam_a_id"]]
        spec_b = spec_by_id[connector["beam_b_id"]]
        x_mm = float(connector["x_mm"])
        dof_a = spec_a["dof_offset"] + 2 * spec_a["node_index"][x_mm]
        dof_b = spec_b["dof_offset"] + 2 * spec_b["node_index"][x_mm]
        k_link = float(connector["k_N_per_mm"])
        K[dof_a][dof_a] += k_link
        K[dof_b][dof_b] += k_link
        K[dof_a][dof_b] -= k_link
        K[dof_b][dof_a] -= k_link
        prepared_connectors.append({**connector, "dof_a": dof_a, "dof_b": dof_b})

    fixed = sorted(fixed)
    free = [i for i in range(n_dof) if i not in fixed]
    Kff = [[K[i][j] for j in free] for i in free]
    Ff = [F[i] for i in free]
    uf = solve_linear_system(Kff, Ff)

    u = [0.0] * n_dof
    for dof_i, value in zip(free, uf):
        u[dof_i] = value

    Ku = [sum(K[i][j] * u[j] for j in range(n_dof)) for i in range(n_dof)]
    R = [Ku[i] - F[i] for i in range(n_dof)]

    responses = {}
    for spec in prepared_specs:
        node_index = spec["node_index"]
        dof_base = spec["dof_offset"]
        elements = []
        for elem in spec["element_data"]:
            u_elem = [u[dof] for dof in elem["dofs"]]
            end_forces = [
                sum(elem["k"][row_i][col_i] * u_elem[col_i] for col_i in range(4)) - elem["fe"][row_i]
                for row_i in range(4)
            ]
            elements.append({
                "x0_mm": elem["x0_mm"],
                "x1_mm": elem["x1_mm"],
                "q_kN_per_mm": elem["q_kN_per_mm"],
                "end_forces": end_forces,
            })
        responses[spec["id"]] = {
            "nodes_mm": spec["nodes"],
            "reactions_kN": {
                float(x_mm): R[dof_base + 2 * node_index[float(x_mm)]] / 1000.0
                for x_mm in spec["supports_mm"]
            },
            "disp_mm": {x_mm: u[dof_base + 2 * node_index[x_mm]] for x_mm in spec["nodes"]},
            "rot_rad": {x_mm: u[dof_base + 2 * node_index[x_mm] + 1] for x_mm in spec["nodes"]},
            "elements": elements,
        }

    connector_forces = []
    for connector in prepared_connectors:
        u_a_mm = u[connector["dof_a"]]
        u_b_mm = u[connector["dof_b"]]
        connector_forces.append({
            **connector,
            "u_a_mm": u_a_mm,
            "u_b_mm": u_b_mm,
            "relative_disp_mm": u_a_mm - u_b_mm,
            "force_on_a_kN": connector["k_N_per_mm"] * (u_b_mm - u_a_mm) / 1000.0,
            "force_on_b_kN": connector["k_N_per_mm"] * (u_a_mm - u_b_mm) / 1000.0,
        })

    return responses, connector_forces


def build_beam_result_from_response(response, MRd_kNm, VRd_kN):
    internal = sample_internal_forces(response["elements"])
    delta = sample_max_deflection_mm(response["nodes_mm"], response["disp_mm"], response["rot_rad"], step_mm=2.0)
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


def analyse_transfer_zone_case(point_loads_kN, gamma_self, compression_only_inner_supports=False):
    inner_beam_section_h_fn = combined_section_h(inner_beam_h_mm, inner_beam_fit_notch_depth_functions)
    if not inner_beam_transfer_links:
        direct_support_x0_mm = min(inner_beam_direct_supports_x_mm)
        direct_support_x1_mm = max(inner_beam_direct_supports_x_mm)
        if len(inner_beam_direct_supports_x_mm) == 1:
            direct_support_x0_mm = float(inner_beam["axis_start"]["x"])
        inner_nodes_no_links_mm = refine_nodes_mm(
            [
                float(inner_beam["axis_start"]["x"]),
                float(inner_beam["axis_end"]["x"]),
                *inner_beam_direct_supports_x_mm,
                *[x_mm for x_mm, _ in point_loads_kN],
                *[x_mm for zone_mm in inner_beam_fit_notch_zones_mm for x_mm in zone_mm],
            ],
            analysis_step_beam_mm,
        )
        inner_result_no_links = analyse_beam_case(
            inner_beam,
            inner_beam_direct_supports_x_mm,
            point_loads_kN,
            gamma_self,
            E_gl30c,
            INNER_BEAM_PROPS["I_vertical_mm4"],
            INNER_BEAM_MRd_kNm,
            INNER_BEAM_VRd_kN,
            EI_by_segment_Nmm2=[
                E_gl30c * member_rect_props(
                    inner_beam_b_mm,
                    inner_beam_section_h_fn(0.5 * (a_mm + b_mm)),
                    member_section_rotation_deg(inner_beam),
                )["I_mm4"]
                for a_mm, b_mm in zip(inner_nodes_no_links_mm, inner_nodes_no_links_mm[1:])
            ],
            nodes_mm_override=inner_nodes_no_links_mm,
        )
        inner_notch_result = None
        inner_notch_min_h = None
        if inner_beam_fit_notch_zones_mm:
            inner_notch_checks = [
                sample_net_section_utilization(
                    inner_result_no_links["response"]["elements"],
                    member_obj=inner_beam,
                    section_h_mm_at_x=inner_beam_section_h_fn,
                    fm_d_Nmm2=fm_d_gl30c,
                    fv_d_Nmm2=fv_d_gl30c,
                    x_start_mm=zone_mm[0],
                    x_end_mm=zone_mm[1],
                    step_mm=1.0,
                )
                for zone_mm in inner_beam_fit_notch_zones_mm
            ]
            inner_notch_result = max(inner_notch_checks, key=lambda item: item["eta_gov"]["value_pct"])
            inner_notch_min_h = min(
                (
                    sample_min_section_height_mm(inner_beam_section_h_fn, zone_mm[0], zone_mm[1], step_mm=1.0)
                    for zone_mm in inner_beam_fit_notch_zones_mm
                ),
                key=lambda item: item["h_mm"],
            )
        return {
            "inner_beam": inner_result_no_links,
            "existing_beam": None,
            "transfer_rows": [],
            "equivalent_support_x_mm": direct_support_x0_mm,
            "effective_span_mm": direct_support_x1_mm - direct_support_x0_mm,
            "total_transferred_kN": 0.0,
            "active_inner_supports_mm": list(inner_beam_direct_supports_x_mm),
            "inner_beam_notch": inner_notch_result,
            "inner_beam_notch_h_min": inner_notch_min_h,
        }

    inner_x0_mm = float(inner_beam["axis_start"]["x"])
    inner_x1_mm = float(inner_beam["axis_end"]["x"])
    existing_x0_mm = float(existing_beam["axis_start"]["x"])
    existing_x1_mm = float(existing_beam["axis_end"]["x"])
    transfer_xs_mm = [float(conn["at"]["x"]) for conn in inner_beam_transfer_links]
    inner_nodes_mm = refine_nodes_mm(
        [
            inner_x0_mm,
            inner_x1_mm,
            *inner_beam_direct_supports_x_mm,
            *transfer_xs_mm,
            *[x_mm for x_mm, _ in point_loads_kN],
            *[x_mm for zone_mm in inner_beam_fit_notch_zones_mm for x_mm in zone_mm],
        ],
        analysis_step_beam_mm,
    )
    existing_nodes_mm = refine_nodes_mm(
        [existing_x0_mm, existing_x1_mm, *inner_supports_x_mm, *transfer_xs_mm],
        analysis_step_beam_mm,
    )
    connector_springs = [
        {
            "id": conn["id"],
            "beam_a_id": inner_beam["id"],
            "beam_b_id": existing_beam["id"],
            "x_mm": float(conn["at"]["x"]),
            "k_N_per_mm": transfer_link_spring_k_N_per_mm(conn),
        }
        for conn in inner_beam_transfer_links
    ]
    def solve_with_active_inner_supports(active_inner_supports_mm):
        responses, connector_forces_local = solve_coupled_beam_responses(
            [
                {
                    "id": inner_beam["id"],
                    "nodes_mm": inner_nodes_mm,
                    "supports_mm": active_inner_supports_mm,
                    "point_loads_kN": point_loads_kN,
                    "uniform_loads_kN_per_mm": uniform_loads_for_nodes(inner_nodes_mm, gamma_self * inner_beam_self_kNm / 1000.0),
                    "EI_Nmm2": E_gl30c * INNER_BEAM_PROPS["I_vertical_mm4"],
                    "EI_by_segment_Nmm2": [
                        E_gl30c * member_rect_props(
                            inner_beam_b_mm,
                            inner_beam_section_h_fn(0.5 * (a_mm + b_mm)),
                            member_section_rotation_deg(inner_beam),
                        )["I_mm4"]
                        for a_mm, b_mm in zip(inner_nodes_mm, inner_nodes_mm[1:])
                    ],
                    "rotational_springs_Nmm_per_rad": {},
                },
                {
                    "id": existing_beam["id"],
                    "nodes_mm": existing_nodes_mm,
                    "supports_mm": inner_supports_x_mm,
                    "point_loads_kN": [],
                    "uniform_loads_kN_per_mm": [],
                    "EI_Nmm2": E_lvl * EXISTING_BEAM_PROPS["I_vertical_mm4"],
                    "rotational_springs_Nmm_per_rad": {},
                },
            ],
            connector_springs,
        )
        inner_result_local = build_beam_result_from_response(responses[inner_beam["id"]], INNER_BEAM_MRd_kNm, INNER_BEAM_VRd_kN)
        inner_result_local["reactions_kN"] = {
            x_mm: inner_result_local["reactions_kN"].get(x_mm, 0.0)
            for x_mm in inner_beam_direct_supports_x_mm
        }
        existing_result_local = build_beam_result_from_response(responses[existing_beam["id"]], EXISTING_BEAM_MRd_kNm, EXISTING_BEAM_VRd_kN)
        existing_result_local["reactions_kN"] = {
            x_mm: existing_result_local["reactions_kN"].get(x_mm, 0.0)
            for x_mm in inner_supports_x_mm
        }
        return responses, connector_forces_local, inner_result_local, existing_result_local

    active_inner_supports_mm = list(inner_beam_direct_supports_x_mm)
    responses, connector_forces, inner_result, existing_result = solve_with_active_inner_supports(active_inner_supports_mm)
    if compression_only_inner_supports:
        while True:
            negative_supports = [
                x_mm
                for x_mm in active_inner_supports_mm
                if inner_result["reactions_kN"].get(x_mm, 0.0) < -1e-6
            ]
            if not negative_supports or len(active_inner_supports_mm) <= 1:
                break
            active_inner_supports_mm = [x_mm for x_mm in active_inner_supports_mm if x_mm not in negative_supports]
            responses, connector_forces, inner_result, existing_result = solve_with_active_inner_supports(active_inner_supports_mm)

    connector_force_by_id = {row["id"]: row for row in connector_forces}

    transfer_rows = []
    for conn in inner_beam_transfer_links:
        transfer_info = conn["transfer"]
        force_row = connector_force_by_id[conn["id"]]
        source_capacity_kN = transfer_link_member_capacity_kN(conn, inner_beam)
        target_capacity_kN = transfer_link_member_capacity_kN(conn, existing_beam)
        group_capacity_kN = min(source_capacity_kN, target_capacity_kN)
        cap_per_fastener_kN = group_capacity_kN / max(1, int(transfer_info["fastener_count_per_member"]))
        transfer_force_kN = force_row["force_on_a_kN"]
        abs_transfer_force_kN = abs(transfer_force_kN)
        h_net_inner_mm = inner_beam_section_h_fn(float(conn["at"]["x"]))
        plate_height_mm = float(transfer_info["plate_height_mm"])
        plate_height_per_member_mm = plate_height_mm / 2.0
        transfer_rows.append({
            "id": conn["id"],
            "x_mm": float(conn["at"]["x"]),
            "description": transfer_info.get("description", conn["id"]),
            "strip_width_mm": float(transfer_info["strip_width_mm"]),
            "plate_height_mm": plate_height_mm,
            "plate_height_per_member_mm": plate_height_per_member_mm,
            "outer_plate_thickness_mm": float(transfer_info["outer_plate_thickness_mm"]),
            "inner_plate_thickness_mm": float(transfer_info["inner_plate_thickness_mm"]),
            "fastener_d_mm": float(transfer_info["fastener_d_mm"]),
            "fastener_grade": transfer_info["fastener_grade"],
            "fastener_count_per_member": int(transfer_info["fastener_count_per_member"]),
            "face_count": transfer_link_face_count(transfer_info),
            "k_N_per_mm": transfer_link_spring_k_N_per_mm(conn),
            "relative_disp_mm": force_row["relative_disp_mm"],
            "force_kN": transfer_force_kN,
            "force_abs_kN": abs_transfer_force_kN,
            "capacity_inner_kN": source_capacity_kN,
            "capacity_existing_kN": target_capacity_kN,
            "capacity_governing_kN": group_capacity_kN,
            "plate_capacity_kN": transfer_link_plate_shear_capacity_kN(conn),
            "eta_fasteners_pct": abs_transfer_force_kN / group_capacity_kN * 100.0 if group_capacity_kN > 1e-9 else float("inf"),
            "eta_plate_pct": abs_transfer_force_kN / transfer_link_plate_shear_capacity_kN(conn) * 100.0 if transfer_link_plate_shear_capacity_kN(conn) > 1e-9 else float("inf"),
            "required_fastener_count_per_member": max(1, int(math.ceil(abs_transfer_force_kN / max(1e-9, cap_per_fastener_kN)))),
            "h_net_inner_mm": h_net_inner_mm,
            "plate_height_margin_mm": h_net_inner_mm - plate_height_per_member_mm,
            "fits_inner_net_height": h_net_inner_mm + 1e-9 >= plate_height_per_member_mm,
            "within_inner_beam_notch": any(zone_mm[0] - 1e-9 <= float(conn["at"]["x"]) <= zone_mm[1] + 1e-9 for zone_mm in inner_beam_fit_notch_zones_mm),
        })

    rightmost_direct_support_x_mm = max(active_inner_supports_mm) if active_inner_supports_mm else max(inner_beam_direct_supports_x_mm)
    left_cluster_forces = [
        (x_mm, max(0.0, inner_result["reactions_kN"].get(x_mm, 0.0)))
        for x_mm in active_inner_supports_mm
        if x_mm < rightmost_direct_support_x_mm - 1e-6
    ]
    left_cluster_forces.extend((row["x_mm"], row["force_kN"]) for row in transfer_rows if row["force_kN"] > 0.0)
    left_cluster_total_kN = sum(force_kN for _, force_kN in left_cluster_forces)
    if left_cluster_total_kN > 1e-9:
        equivalent_support_x_mm = sum(x_mm * force_kN for x_mm, force_kN in left_cluster_forces) / left_cluster_total_kN
    else:
        equivalent_support_x_mm = min([*transfer_xs_mm, *active_inner_supports_mm]) if transfer_xs_mm or active_inner_supports_mm else inner_x0_mm

    inner_notch_result = None
    inner_notch_min_h = None
    if inner_beam_fit_notch_zones_mm:
        inner_notch_checks = [
            sample_net_section_utilization(
                responses[inner_beam["id"]]["elements"],
                member_obj=inner_beam,
                section_h_mm_at_x=inner_beam_section_h_fn,
                fm_d_Nmm2=fm_d_gl30c,
                fv_d_Nmm2=fv_d_gl30c,
                x_start_mm=zone_mm[0],
                x_end_mm=zone_mm[1],
                step_mm=1.0,
            )
            for zone_mm in inner_beam_fit_notch_zones_mm
        ]
        inner_notch_result = max(inner_notch_checks, key=lambda item: item["eta_gov"]["value_pct"])
        inner_notch_min_h = min(
            (
                sample_min_section_height_mm(inner_beam_section_h_fn, zone_mm[0], zone_mm[1], step_mm=1.0)
                for zone_mm in inner_beam_fit_notch_zones_mm
            ),
            key=lambda item: item["h_mm"],
        )

    return {
        "inner_beam": inner_result,
        "existing_beam": existing_result,
        "transfer_rows": transfer_rows,
        "equivalent_support_x_mm": equivalent_support_x_mm,
        "effective_span_mm": rightmost_direct_support_x_mm - equivalent_support_x_mm,
        "total_transferred_kN": sum(max(0.0, row["force_kN"]) for row in transfer_rows),
        "active_inner_supports_mm": active_inner_supports_mm,
        "inner_beam_notch": inner_notch_result,
        "inner_beam_notch_h_min": inner_notch_min_h,
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


def analyse_case(case_key):
    case = CASE_DEFS[case_key]
    roof_area_kNm2_at = case["roof_area_kNm2_at"]
    gamma_self = case["gamma_self"]
    existing_case = existing_structure_case_key(case_key)

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

    left_inner_point_loads = list(zip(interior_purlin_support_ys_mm["left"], [item["R_inner_kN"] for item in purlins["left"]]))
    right_inner_point_loads = list(zip(interior_purlin_support_ys_mm["right"], [item["R_inner_kN"] for item in purlins["right"]]))
    left_edge_point_loads = list(zip(edge_purlin_support_ys_mm["left"], [item["R_edge_kN"] for item in purlins["left"]]))
    right_edge_point_loads = list(zip(edge_purlin_support_ys_mm["right"], [item["R_edge_kN"] for item in purlins["right"]]))
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
    existing_support_beam_point_loads = []
    inner_column_direct_loads_kN = {x_mm: 0.0 for x_mm in inner_supports_x_mm}
    for rafter_obj, r in zip(interior_rafters, interior_results):
        x_mm = float(rafter_obj["axis_start"]["x"])
        outer_beam_point_loads.append((x_mm, r["R_outer_kN"]))
        inner_beam_point_loads.append((x_mm, r["R_inner_kN"]))
    for side, edge_obj in edge_rafters.items():
        x_mm = float(edge_obj["axis_start"]["x"])
        outer_beam_point_loads.append((x_mm, edge_results[side]["R_outer_kN"]))
        target_member_id = edge_rafter_inner_support_member_by_side[side]
        if target_member_id == inner_beam["id"]:
            inner_beam_point_loads.append((x_mm, edge_results[side]["R_inner_kN"]))
        elif target_member_id == existing_support_beam["id"]:
            existing_support_beam_point_loads.append((edge_inner_support_y_mm_by_side[side], edge_results[side]["R_inner_kN"]))
        elif target_member_id.startswith("col."):
            col_x_mm = float(member(GEO, "columns", target_member_id)["base"]["x"])
            inner_column_direct_loads_kN[col_x_mm] = inner_column_direct_loads_kN.get(col_x_mm, 0.0) + edge_results[side]["R_inner_kN"]
        else:
            raise ValueError(f"Unsupported edge rafter inner support target for {edge_obj['id']}: {target_member_id}")
    for side, rows in corner_purlin_results.items():
        for result in rows:
            if result["outer_support_member_id"] == "beam.outer":
                outer_beam_point_loads.append((result["support_outer_x_mm"], result["R_outer_kN"]))

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
    transfer_zone_result = analyse_transfer_zone_case(
        sorted(inner_beam_point_loads, key=lambda item: item[0]),
        gamma_self,
        compression_only_inner_supports=(case_key != "UPLIFT"),
    )
    inner_beam_result = transfer_zone_result["inner_beam"]
    existing_beam_combined = check_existing_kp360_combined(
        [(row["x_mm"], row["force_kN"]) for row in transfer_zone_result["transfer_rows"]],
        load_case=existing_case,
        context=KATOS_EXISTING_CTX,
    )
    existing_support_beam_result = check_existing_lp225_x125_combined(
        sorted(existing_support_beam_point_loads, key=lambda item: item[0]),
        load_case=existing_case,
        context=KATOS_EXISTING_CTX,
    )
    existing_support_beam_right_base = check_existing_lp225_x125_combined(
        [],
        load_case=existing_case,
        context=KATOS_EXISTING_CTX,
    )

    return {
        "case": case,
        "purlins": purlins,
        "corner_purlins": corner_purlin_results,
        "interior_rafters": interior_results,
        "edge_rafters": edge_results,
        "outer_beam_point_loads": sorted(outer_beam_point_loads, key=lambda item: item[0]),
        "inner_beam_point_loads": sorted(inner_beam_point_loads, key=lambda item: item[0]),
        "existing_support_beam_point_loads": sorted(existing_support_beam_point_loads, key=lambda item: item[0]),
        "inner_column_direct_loads_kN": inner_column_direct_loads_kN,
        "outer_beam": outer_beam_result,
        "inner_beam": inner_beam_result,
        "existing_beam_transfer": transfer_zone_result["existing_beam"],
        "existing_beam": existing_beam_combined,
        "existing_support_beam": existing_support_beam_result,
        "existing_support_beam_right_base": existing_support_beam_right_base,
        "transfer_zone": transfer_zone_result,
    }


RESULTS = {case_key: analyse_case(case_key) for case_key in (*ULS_CASE_KEYS, *SLS_CASE_KEYS, "UPLIFT")}
OUTER_BEAM_H = {
    "ULS A": analyse_outer_beam_horizontal(gammaQ * psi0_W * q_outer_wind_h_char),
    "ULS B": analyse_outer_beam_horizontal(gammaQ * q_outer_wind_h_char),
    "ULS DRIFT": analyse_outer_beam_horizontal(gammaQ * psi0_W * q_outer_wind_h_char),
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
inner_beam_notch_governing_case = max(
    ULS_CASE_KEYS,
    key=lambda case_key: 0.0 if RESULTS[case_key]["transfer_zone"]["inner_beam_notch"] is None else RESULTS[case_key]["transfer_zone"]["inner_beam_notch"]["eta_gov"]["value_pct"],
)
inner_beam_notch_governing = RESULTS[inner_beam_notch_governing_case]["transfer_zone"]["inner_beam_notch"]
inner_beam_notch_h_min = RESULTS[inner_beam_notch_governing_case]["transfer_zone"]["inner_beam_notch_h_min"]
inner_beam_notch_eta_pct = 0.0 if inner_beam_notch_governing is None else inner_beam_notch_governing["eta_gov"]["value_pct"]
transfer_zone_governing_case = max(
    ULS_CASE_KEYS,
    key=lambda case_key: max(
        (row["eta_fasteners_pct"] for row in RESULTS[case_key]["transfer_zone"]["transfer_rows"]),
        default=0.0,
    ),
)
transfer_zone_governing = RESULTS[transfer_zone_governing_case]["transfer_zone"]
transfer_link_governing = max(
    transfer_zone_governing["transfer_rows"],
    key=lambda row: row["eta_fasteners_pct"],
) if transfer_zone_governing["transfer_rows"] else None
transfer_link_fit_governing = min(
    transfer_zone_governing["transfer_rows"],
    key=lambda row: row["plate_height_margin_mm"],
) if transfer_zone_governing["transfer_rows"] else None
existing_beam_governing_case = max(
    ULS_CASE_KEYS,
    key=lambda case_key: 0.0 if RESULTS[case_key]["existing_beam"] is None else max(
        RESULTS[case_key]["existing_beam"]["eta_M"],
        RESULTS[case_key]["existing_beam"]["eta_V"],
        RESULTS[case_key]["existing_beam"]["eta_LTB"],
    ),
)
existing_beam_governing = RESULTS[existing_beam_governing_case]["existing_beam"]
existing_support_beam_governing_case = max(
    ULS_CASE_KEYS,
    key=lambda case_key: max(
        RESULTS[case_key]["existing_support_beam"]["eta_M"],
        RESULTS[case_key]["existing_support_beam"]["eta_V"],
    ),
)
existing_support_beam_governing = RESULTS[existing_support_beam_governing_case]["existing_support_beam"]

outer_beam_sls_normal_delta_mm = abs(RESULTS["SLS"]["outer_beam"]["delta"]["value_mm"])
outer_beam_sls_drift_delta_mm = abs(RESULTS["SLS DRIFT"]["outer_beam"]["delta"]["value_mm"])
outer_beam_sls_delta_mm = max(outer_beam_sls_normal_delta_mm, outer_beam_sls_drift_delta_mm)
inner_beam_sls_normal_delta_mm = abs(RESULTS["SLS"]["inner_beam"]["delta"]["value_mm"])
inner_beam_sls_drift_delta_mm = abs(RESULTS["SLS DRIFT"]["inner_beam"]["delta"]["value_mm"])
inner_beam_sls_delta_mm = max(inner_beam_sls_normal_delta_mm, inner_beam_sls_drift_delta_mm)
existing_beam_sls_delta_mm = 0.0 if RESULTS["SLS"]["existing_beam"] is None else max(
    abs(RESULTS["SLS"]["existing_beam"]["delta"]["value_mm"]),
    abs(RESULTS["SLS DRIFT"]["existing_beam"]["delta"]["value_mm"]),
)
inner_beam_sls_governing_case = "SLS DRIFT" if inner_beam_sls_drift_delta_mm >= inner_beam_sls_normal_delta_mm else "SLS"
inner_beam_sls_effective_span_mm = RESULTS[inner_beam_sls_governing_case]["transfer_zone"]["effective_span_mm"]
inner_beam_sls_limit_mm = inner_beam_sls_effective_span_mm / 300.0
outer_beam_governing_point_load = max(RESULTS[outer_beam_governing_case]["outer_beam_point_loads"], key=lambda item: item[1])
outer_reaction_case = max(ULS_CASE_KEYS, key=lambda case_key: max(RESULTS[case_key]["outer_beam"]["reactions_kN"].values()))
inner_reaction_case = max(ULS_CASE_KEYS, key=lambda case_key: max(RESULTS[case_key]["inner_beam"]["reactions_kN"].values()))

outer_uplift = RESULTS["UPLIFT"]["outer_beam"]["reactions_kN"]
inner_uplift = RESULTS["UPLIFT"]["inner_beam"]["reactions_kN"]
existing_uplift = (
    {x_mm: 0.0 for x_mm in inner_supports_x_mm}
    if RESULTS["UPLIFT"]["existing_beam"] is None
    else RESULTS["UPLIFT"]["existing_beam"]["reactions_kN"]
)
existing_support_beam_uplift = RESULTS["UPLIFT"]["existing_support_beam"]["reactions_kN"]
existing_support_beam_right_base_uplift = RESULTS["UPLIFT"]["existing_support_beam_right_base"]["reactions_kN"]


def total_inner_support_loads_kN(case_key):
    loads = {x_mm: 0.0 for x_mm in inner_supports_x_mm}
    for support_x_mm, value_kN in RESULTS[case_key]["inner_beam"]["reactions_kN"].items():
        loads[inner_beam_support_column_x_by_support_x.get(support_x_mm, support_x_mm)] += value_kN
    existing_result = RESULTS[case_key]["existing_beam"]
    if existing_result is not None:
        for x_mm, value_kN in existing_result["reactions_kN"].items():
            loads[x_mm] += value_kN
    loads[inner_supports_x_mm[0]] += RESULTS[case_key]["existing_support_beam"]["reactions_kN"][existing_support_beam_column_y_mm]
    loads[inner_supports_x_mm[-1]] += RESULTS[case_key]["existing_support_beam_right_base"]["reactions_kN"][existing_support_beam_column_y_mm]
    for x_mm, value_kN in RESULTS[case_key]["inner_column_direct_loads_kN"].items():
        loads[x_mm] += value_kN
    return loads


inner_support_totals_by_case = {
    case_key: total_inner_support_loads_kN(case_key)
    for case_key in (*ULS_CASE_KEYS, *SLS_CASE_KEYS, "UPLIFT")
}
inner_total_reaction_case = max(
    ULS_CASE_KEYS,
    key=lambda case_key: max(inner_support_totals_by_case[case_key].values()),
)
inner_total_uplift = inner_support_totals_by_case["UPLIFT"]

baseline_total_column_loads = calculate_katos_total_column_loads()
column_case_groups = {
    case_key: existing_structure_case_key(case_key)
    for case_key in (*ULS_CASE_KEYS, *SLS_CASE_KEYS, "UPLIFT")
}
inner_column_id_by_support_x = {
    inner_supports_x_mm[0]: "col.x125",
    inner_supports_x_mm[-1]: "col.x7075",
}
additional_upper_column_loads_by_case = {
    case_key: {
        column_id: (
            inner_support_totals_by_case[case_key][support_x_mm]
            - baseline_total_column_loads["upper_column_support_loads"][column_case_groups[case_key]][column_id]
        )
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
print("  LASITETTU TERASSI – PUU2-VAIHTOEHTO – KUORMITUSLASKENTA")
print("  EN 1990 / EN 1991-1-1/3/4 / EN 1995-1-1")
print(DW)

print("\n── GEOMETRIA ─────────────────────────────────────────────────────")
print(f"  Geometria                     {GEOMETRY_PATH_LABEL}")
print(f"  Paneelikenttä                 {roof_width_mm:.0f} × {roof_depth_mm:.0f} mm  ({roof_area_m2:.2f} m²)")
print(f"  Kattokaltevuus y-suunnassa    {roof_slope_deg:.1f}°")
print(f"  Sisäkattotuolit               {len(interior_rafters)} kpl  @ {interior_rafter_xs_mm[1]-interior_rafter_xs_mm[0]:.0f} mm")
print(f"  Reunakattotuolit              2 kpl  @ x = {edge_rafters['left']['axis_start']['x']:.0f} / {edge_rafters['right']['axis_start']['x']:.0f} mm")
print(
    f"  Orret / purlins x-suunnassa   {len(left_purlins) + len(right_purlins)} kpl"
    f"  (vasen y = {', '.join(f'{y_mm:.0f}' for y_mm in purlin_y_positions_mm['left'])} mm;"
    f" oikea y = {', '.join(f'{y_mm:.0f}' for y_mm in purlin_y_positions_mm['right'])} mm)"
)
if corner_purlins:
    print(f"  Nurkkaorret y-suunnassa       {len(corner_purlins)} kpl  @ x = " + ", ".join(f"{float(item['axis_start']['x']):.0f}" for item in corner_purlins) + " mm")
print(f"  Kattotuolien tuet sisa/ulko   y = {interior_inner_support_y_mm:.0f} / {interior_outer_support_y_mm:.0f} mm")
print(
    f"  Reunakattotuolien sisatuet    vasen y = {edge_inner_support_y_mm_by_side['left']:.0f} mm"
    f" / oikea y = {edge_inner_support_y_mm_by_side['right']:.0f} mm"
)
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
print(f"  Sisäpalkin suorat tuet        x = " + " / ".join(f"{x_mm:.0f}" for x_mm in inner_beam_direct_supports_x_mm) + " mm")
if inner_beam_transfer_links:
    transfer_geom = inner_beam_transfer_links[0]["transfer"]
    print(
        f"  KP360-siirtolinkit            {len(inner_beam_transfer_links)} kpl @ x = "
        + " / ".join(f"{float(conn['at']['x']):.0f}" for conn in inner_beam_transfer_links)
        + " mm"
    )
    print(
        f"  Kaistalevy / pultitus         ulko {transfer_geom['outer_plate_thickness_mm']:.0f} mm + "
        f"sisa {transfer_geom['inner_plate_thickness_mm']:.0f} mm, {transfer_geom['strip_width_mm']:.0f} × "
        f"{transfer_geom['plate_height_mm']:.0f} mm, {transfer_geom['fastener_count_per_member']} × "
        f"M{transfer_geom['fastener_d_mm']:.0f} {transfer_geom['fastener_grade']} / jasen"
    )

print("\n── KUORMAT ───────────────────────────────────────────────────────")
print(f"  Paneelit                      {panel_count['nx']}×{panel_count['ny']} = {panel_count_total} kpl, {panels_total_kN:.2f} kN")
print(f"  Pysyvä katekuorma             gk = {gk_roofing:.3f} kN/m²  (paneelit {gk_panels:.3f} + kiinnikkeet {gk_fixings:.2f})")
print(f"  Lumi                          sk = {sk:.1f} kN/m², μ1 = {mu1:.1f}  →  s = {s_roof:.2f} kN/m²")
print(f"  Tuuli katolle                 qp(z={z_ref_m:.1f} m) = {qp_z:.3f} kN/m²")
print(f"    alas (auki)                 cp,net = {cp_net_down:+.2f}  →  w = {w_down:.3f} kN/m²")
print(f"    imu (kiinni)                cp,net = {cp_net_up_closed:+.2f}  →  w = {w_up_closed:.3f} kN/m²")
print(f"  ULS A kattokuorma             {roof_area_uls_A:.3f} kN/m²  (1.35G + 1.5S + 1.5·0.6W↓)")
print(f"  ULS B kattokuorma             {roof_area_uls_B:.3f} kN/m²  (1.35G + 1.5·0.7S)")
print(f"  ULS DRIFT kattokuorma         1.35G + 1.5·S_kin(x,y)")
print(f"  Uplift-kattokuorma            {roof_area_uplift:.3f} kN/m²  (0.9G + 1.5W↑)")
print(f"  Seinää vasten h(x)            {h_seina_left_m:.2f} … {h_seina_right_m:.2f} m")
print(f"    hallitseva x                {critical_drift['x_mm']:.0f} mm, ls = {critical_drift['ls_m']:.2f} m, μ2 = {critical_drift['mu2']:.2f}")
print(f"    s_kin,max / s@y_in          {critical_drift['s_peak_kNm2']:.2f} / {critical_drift['s_inner_rafter_kNm2']:.2f} kN/m²")

print("\n── PANEELIT ──────────────────────────────────────────────────────")
print(f"  Tyyppi                        {panel_material}")
print(
    f"  Moduuli                       {panel_unit_width_mm:.0f} × {panel_unit_slope_length_mm:.0f} × "
    f"{panel_unit_thickness_mm:.0f} mm, {panel_mass_kg:.0f} kg/kpl"
)
print(
    f"  Kenttä                        {panel_count_x} × {panel_count_y} = {panel_count_total} kpl, "
    f"lappeella {panel_field_slope_width_mm:.0f} × {panel_field_slope_length_mm:.0f} mm, "
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
    print(f"  Lovi reunatuella              rect_notch {purlin_edge_notch_depth_mm:.0f} × {purlin_edge_notch_length_mm:.0f} mm, h_net,min = {critical_purlin['h_net_min_mm']:.0f} mm")
else:
    print("  Lovi reunatuella              ei lovea")
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

print(f"\n── ULKOPALKKI {outer_beam['profile']['name']} ───────────────────────────────────")
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

print(f"\n── SISÄPALKKI {inner_beam['profile']['name']} ─────────────────────────────────────")
print(f"  MRd = {INNER_BEAM_MRd_kNm:.2f} kNm,  VRd = {INNER_BEAM_VRd_kN:.2f} kN")
print(f"  Hallitseva tapaus             {inner_beam_governing_case}")
print(f"  Md,max                        {inner_beam_governing['M_gov']['value_kNm']:.2f} kNm @ x = {inner_beam_governing['M_gov']['x_mm']:.0f} mm")
print(f"  Vd,max                        {abs(inner_beam_governing['V_abs']['value_kN']):.2f} kN")
print(
    "  Pilarireaktiot suorilla tuilla  "
    + " / ".join(
        f"{inner_beam_governing['reactions_kN'].get(x_mm, 0.0):.2f} kN @ x = {x_mm:.0f} mm"
        for x_mm in inner_beam_direct_supports_x_mm
    )
)
print(
    "  Aktiiviset omat tuet          x = "
    + " / ".join(f"{x_mm:.0f}" for x_mm in RESULTS[inner_beam_governing_case]["transfer_zone"]["active_inner_supports_mm"])
    + " mm"
)
print(
    f"  Ekvivalentti vasen tukipiste  x ≈ {RESULTS[inner_beam_governing_case]['transfer_zone']['equivalent_support_x_mm']:.0f} mm"
    f"  ->  L_eff ≈ {RESULTS[inner_beam_governing_case]['transfer_zone']['effective_span_mm']:.0f} mm"
)
print(
    f"  Siirtyvä voima {existing_beam['profile']['name']}:lle   "
    f"{RESULTS[inner_beam_governing_case]['transfer_zone']['total_transferred_kN']:.2f} kN"
)
print(
    f"  Taipuma SLS / DRIFT           {inner_beam_sls_normal_delta_mm:.2f} / {inner_beam_sls_drift_delta_mm:.2f}"
    f" / {inner_beam_sls_limit_mm:.1f} mm  {format_ok(inner_beam_sls_delta_mm <= inner_beam_sls_limit_mm)}"
)
if inner_beam_notch_governing is not None:
    print(
        f"  Sovituslovi beam.existing.kp360x2:lla  {format_bevel_notch_specs([inner_beam_fit_bevel_info])}, "
        f"h_net,min = {inner_beam_notch_h_min['h_mm']:.0f} mm @ x = {inner_beam_notch_h_min['x_mm']:.0f} mm"
    )
    print(
        f"  Loven nettoh-tarkistus        {inner_beam_notch_governing_case}, "
        f"{inner_beam_notch_governing['eta_gov']['mode']} = {inner_beam_notch_governing['eta_gov']['value_pct']:.1f}% "
        f"@ x = {inner_beam_notch_governing['eta_gov']['x_mm']:.0f} mm"
    )
print(
    f"  Tulos                         η_M = {inner_beam_governing['eta_M']:.1f}%,  "
    f"η_V = {inner_beam_governing['eta_V']:.1f}%,  η_lovi = {inner_beam_notch_eta_pct:.1f}%  "
    f"{format_ok(inner_beam_governing['eta_M'] <= 100.0 and inner_beam_governing['eta_V'] <= 100.0 and inner_beam_notch_eta_pct <= 100.0)}"
)

print(f"\n── SIIRTOVYÖHYKE {inner_beam['profile']['name']} -> {existing_beam['profile']['name']} ─────────────────")
if transfer_link_governing is None:
    print("  Siirtolinkit                  ei mallinnettu")
else:
    print(f"  Hallitseva tapaus             {transfer_zone_governing_case}")
    print(f"  Siirtyvä voima yhteensa       {transfer_zone_governing['total_transferred_kN']:.2f} kN")
    print(
        f"  Ekvivalentti vasen tukipiste  x = {transfer_zone_governing['equivalent_support_x_mm']:.0f} mm"
        f"  ->  L_eff = {transfer_zone_governing['effective_span_mm']:.0f} mm"
    )
    print(
        f"  Kriittinen kaistalevy         {transfer_link_governing['description']}, "
        f"Fd = {transfer_link_governing['force_abs_kN']:.2f} kN, "
        f"η_fast = {transfer_link_governing['eta_fasteners_pct']:.1f}%, "
        f"η_levy = {transfer_link_governing['eta_plate_pct']:.1f}%"
    )
    print(
        f"  Lovialueen pienin h_net/linkki {transfer_link_fit_governing['description']}, "
        f"h_net = {transfer_link_fit_governing['h_net_inner_mm']:.0f} mm, "
        f"levy/jäsen = {transfer_link_fit_governing['plate_height_per_member_mm']:.0f} mm "
        f"(levy yht. {transfer_link_fit_governing['plate_height_mm']:.0f} mm), "
        f"Δh = {transfer_link_fit_governing['plate_height_margin_mm']:+.0f} mm  "
        f"{format_ok(transfer_link_fit_governing['fits_inner_net_height'])}"
    )
    print()
    print(f"  {'ID':<25} {'x':>6} {'k':>8} {'Fd':>8} {'η_fast':>9} {'η_levy':>9} {'h_net':>7} {'levy/j':>7} {'Δh':>7} {'M12/tarve':>11}")
    print(f"  {'-'*25} {'-'*6} {'-'*8} {'-'*8} {'-'*9} {'-'*9} {'-'*7} {'-'*7} {'-'*7} {'-'*11}")
    for row in transfer_zone_governing['transfer_rows']:
        print(
            f"  {row['description']:<25} {row['x_mm']:>6.0f} {row['k_N_per_mm']/1000.0:>8.2f} {row['force_abs_kN']:>8.2f}"
            f" {row['eta_fasteners_pct']:>8.1f}% {row['eta_plate_pct']:>8.1f}%"
            f" {row['h_net_inner_mm']:>7.0f} {row['plate_height_per_member_mm']:>7.0f} {row['plate_height_margin_mm']:>+7.0f}"
            f" {row['fastener_count_per_member']:>4d}/{row['required_fastener_count_per_member']:>5d}"
        )

if existing_beam_governing is not None:
    print(f"\n── OLEMASSA OLEVA {existing_support_beam['profile']['name']} @ x125 ───────────────────────")
    print(f"  Hallitseva tapaus             {existing_support_beam_governing_case}")
    print(
        "  Pistekuormat y = "
        + " / ".join(
            f"{y_mm:.0f} mm: {p_kN:.2f} kN"
            for y_mm, p_kN in existing_support_beam_governing["point_loads_abs_mm"]
        )
    )
    print(
        f"  Tukireaktiot seinä / pilari   {existing_support_beam_governing['reactions_kN'][0.0]:.2f}"
        f" / {existing_support_beam_governing['reactions_kN'][existing_support_beam_column_y_mm]:.2f} kN"
    )
    print(
        f"  Md,max / Vd,max               {existing_support_beam_governing['M_gov']['value_kNm']:.2f} kNm @ y = "
        f"{existing_support_beam_governing['M_gov']['x_mm']:.0f} mm / "
        f"{abs(existing_support_beam_governing['V_abs']['value_kN']):.2f} kN"
    )
    print(
        f"  Tulos                         η_M = {existing_support_beam_governing['eta_M']:.1f}%,  "
        f"η_V = {existing_support_beam_governing['eta_V']:.1f}%  "
        f"{format_ok(existing_support_beam_governing['eta_M'] <= 100.0 and existing_support_beam_governing['eta_V'] <= 100.0)}"
    )

    print(f"\n── OLEMASSA OLEVA {existing_beam['profile']['name']} (vanha katos + puu2) ───────────────")
    print(f"  MRd = {EXISTING_BEAM_MRd_kNm:.2f} kNm,  VRd = {EXISTING_BEAM_VRd_kN:.2f} kN")
    print(f"  Hallitseva tapaus             {existing_beam_governing_case}")
    print(f"  Md,max                        {existing_beam_governing['M_gov']['value_kNm']:.2f} kNm @ x = {existing_beam_governing['M_gov']['x_mm']:.0f} mm")
    print(f"  Vd,max                        {abs(existing_beam_governing['V_abs']['value_kN']):.2f} kN")
    print(
        f"  Pilarireaktiot yhteensa       {existing_beam_governing['reactions_kN'][inner_supports_x_mm[0]]:.2f}"
        f" / {existing_beam_governing['reactions_kN'][inner_supports_x_mm[1]]:.2f} kN"
    )
    print(
        f"  Taipuma SLS / DRIFT           {existing_beam_sls_delta_mm:.2f} / {existing_beam_governing['delta_lim_mm']:.1f} mm  "
        f"{format_ok(existing_beam_sls_delta_mm <= existing_beam_governing['delta_lim_mm'])}"
    )
    print(
        f"  Tulos                         η_M = {existing_beam_governing['eta_M']:.1f}%,  "
        f"η_V = {existing_beam_governing['eta_V']:.1f}%,  η_LTB = {existing_beam_governing['eta_LTB']:.1f}%  "
        f"{format_ok(existing_beam_governing['eta_M'] <= 100.0 and existing_beam_governing['eta_V'] <= 100.0 and existing_beam_governing['eta_LTB'] <= 100.0)}"
    )

print("\n── PILARIKUORMAT JA NOSTO ───────────────────────────────────────")
print(f"  Ulkopalkin reaktiot {outer_reaction_case} / UPLIFT [kN]:")
for x_mm in outer_supports_x_mm:
    print(f"    x = {x_mm:>4.0f} mm              {RESULTS[outer_reaction_case]['outer_beam']['reactions_kN'][x_mm]:>6.2f} / {outer_uplift[x_mm]:>6.2f}")
print(f"  Sisäpalkin reaktiot {inner_reaction_case} / UPLIFT [kN]:")
for x_mm in inner_beam_direct_supports_x_mm:
    print(f"    x = {x_mm:>4.0f} mm              {RESULTS[inner_reaction_case]['inner_beam']['reactions_kN'][x_mm]:>6.2f} / {inner_uplift[x_mm]:>6.2f}")
if existing_beam_governing is not None:
    print(f"  KP360 yhteisreaktiot {existing_beam_governing_case} / UPLIFT [kN]:")
    for x_mm in inner_supports_x_mm:
        print(f"    x = {x_mm:>4.0f} mm              {RESULTS[existing_beam_governing_case]['existing_beam']['reactions_kN'][x_mm]:>6.2f} / {existing_uplift[x_mm]:>6.2f}")
    print(
        f"  LP225 x125 pilarireaktio {inner_total_reaction_case} / UPLIFT [kN]:"
        f" {RESULTS[inner_total_reaction_case]['existing_support_beam']['reactions_kN'][existing_support_beam_column_y_mm]:.2f}"
        f" / {existing_support_beam_uplift[existing_support_beam_column_y_mm]:.2f}"
    )
    print(
        f"  LP225 x7075 perusreaktio {inner_total_reaction_case} / UPLIFT [kN]:"
        f" {RESULTS[inner_total_reaction_case]['existing_support_beam_right_base']['reactions_kN'][existing_support_beam_column_y_mm]:.2f}"
        f" / {existing_support_beam_right_base_uplift[existing_support_beam_column_y_mm]:.2f}"
    )
print(f"  Sisatukien kokonaiskuormat {inner_total_reaction_case} / UPLIFT [kN]:")
for x_mm in inner_supports_x_mm:
    print(f"    x = {x_mm:>4.0f} mm              {inner_support_totals_by_case[inner_total_reaction_case][x_mm]:>6.2f} / {inner_total_uplift[x_mm]:>6.2f}")
print(f"  Suurin ulkopilaripuristus         {max(RESULTS[outer_reaction_case]['outer_beam']['reactions_kN'].values()):.2f} kN")
print(f"  Suurin ulkopilarin nostotarve     {abs(min(outer_uplift.values())):.2f} kN")
print(f"  Suurin sisatukipuristus           {max(inner_support_totals_by_case[inner_total_reaction_case].values()):.2f} kN")
print(f"  Suurin sisatuen nostotarve        {abs(min(inner_total_uplift.values())):.2f} kN")

print("\n── KOKONAISPILARIKUORMAT TERASSIN JALKEEN ───────────────────────")
print("  Olemassa oleva baseline        geometry/katos.json + puu2-variantin lisakuormat")
print(f"  Ontelolaatta saumattuna h=150  {terrace_total_column_loads['gk_hollow_slab_kNm2']:.2f} kN/m²")
print(f"  Pintavalu 60 mm                {terrace_total_column_loads['gk_floor_cast_kNm2']:.2f} kN/m²")
print(f"  Terassin hyötykuorma           {terrace_total_column_loads['qk_terrace_live_kNm2']:.2f} kN/m²")
if terrace_total_column_loads["outer_beam_count"] == 1:
    print(f"  Alapalkki 350x300              {terrace_total_column_loads['outer_beam_self_kNm']:.3f} kN/m")
else:
    print(
        f"  Alapalkit {terrace_total_column_loads['outer_beam_count']}x350x300            "
        f"{terrace_total_column_loads['outer_beam_total_self_kN'] / terrace_total_column_loads['outer_beam_count']:.2f}"
        f" kN / palkki  = {terrace_total_column_loads['outer_beam_self_kNm']:.3f} kN/m"
    )
print("  Kuormareitti                   sisapilarit suoraan perustuksille, ontelolaatat seinasta alapalkille, alapalkki ulompiin pilareihin")
print("  Ontelolaatan paatyreaktio      jaetaan alapalkille laatan leveyden matkalle")
print(
    f"  Portaikon lisa col.x7075       SLS {portaikko_col_x7075_extra_sls:.2f} kN /"
    f" ULS {portaikko_col_x7075_extra_uls:.2f} kN / UPLIFT {portaikko_col_x7075_extra_uplift:.2f} kN"
)
print("  N_sls / N_uls / N_min          max(SLS,SLS DRIFT) / max(ULS A, ULS B, ULS DRIFT) / UPLIFT")
print()
print(f"  {'Pilari':<22} {'Ryhma':<10} {'N_sls':>9} {'N_uls':>9} {'N_min':>9}  {'Tila'}")
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
print("  * Reunakattotuoleille ei anneta suoraa paneelikaistan hajakuormaa; kuorma siirtyy")
print(f"    {GEOMETRY_PATH_LABEL}:n load_transfer.member_refs-metadatan mukaisesti orsien kautta;")
print("    orren oma paino mallinnetaan viivakuormana.")
print("  * Ulkokulmien nurkkaorret mallinnetaan tuettuina toiseksi uloimpaan kattotuoliin ja")
print("    ulkopalkin ulkoreunalta kantavina ulokkeina; reunatuki voi siksi olla vetava.")
print(f"  * Liitosten tuki- ja rotaatiomallit luetaan {GEOMETRY_PATH_LABEL}:n")
print("    connections.analysis-metadatasta; nykygeometriassa orsien tuet ovat niveliä.")
print(f"  * Orsien ja kattotuolien lovi-/nettoh-tarkistukset luetaan suoraan {GEOMETRY_PATH_LABEL}:n")
print("    cuts-kentistä; terassivertailuskriptin hardkoodattuja loviarvoja ei käytetä.")
if inner_beam_transfer_links:
    print("  * KP360-siirtovyohyke mallinnetaan diskreetteina transfer_link-liitoksina;")
    print("    linkkien jousijakyys johdetaan M12-pulttien slip-moduluksesta ja kapasiteetti")
    print("    tarkistetaan boltin leikkauksella, puun reunapuristuksella seka levyn in-plane shearilla.")
    print("  * Sisapalkin vasen paa ei ole suoraan x125-pilarilla, vaan kuorma siirtyy")
    print("    KP360-siirtolinkkien kautta olemassa olevalle rungolle.")
if inner_beam_fit_notch_connection is not None:
    print("  * Sisapalkin sovituslovi luetaan erillisena notched_over-liitoksena beam.inner.new")
    print("    -> beam.existing.kp360x2; lovi vaikuttaa sisapalkin paikalliseen nettoh-tarkistukseen")
    print("    ja raportissa naytettavaan transfer_link-kohtaiseen h_net-marginaaliin.")
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
print("  * Vasen reunakattotuoli tukeutuu beam.lp225.x125:een ja sen kuorma tarkistetaan")
print("    yhdistettynä olemassa olevan katoksen KP450-paadykuormaan; oikea reunakattotuoli menee suoraan x7075-pilarille.")
print("  * Olemassa olevien KP360- ja LP225-palkkien tarkistus kayttaa samoja peruskuormia kuin kuormituslaskenta.py,")
print("    joten puu2-raportin combined-check vastaa vanhan katoksen laskentaa eika kasin kopioituja kuormia.")
print("  * Uplift käyttää suljetun lasituksen imutapausta (w_up_closed); kiinnitykset tulee mitoittaa")
print("    vähintään yllä raportoiduille nostoreaktioille.")
print(DW)
