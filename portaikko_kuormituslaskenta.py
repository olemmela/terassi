"""
PORTAIKON KATOKSEN KUORMITUSLASKENTA – ETELÄSUOMI
=================================================
Standardit: EN 1990, EN 1991-1-3, EN 1991-1-4, EN 1995-1-1

Geometria luetaan tiedostosta geometry/portaikko.json:
  - 7 kpl sahatavara-C24 kattotuoleja, profiili luetaan geometriasta
  - Katto viettää 12° y-suunnassa talon seinältä portaille
  - Kattotuolit ovat tuettu seinään (y=-1000) ja LP225×90-palkkiin (y=1675),
    jonka jälkeen on 225 mm puu-uloke
  - LP225×90 on jatkuva palkki neljällä pystytuella

Rajaukset:
  - Kaide/metallikaiteet eivät sisälly tähän ennen kuin niiden geometria on
    mallinnettu geometry/portaikko.json:iin
  - Liitostarvikkeet ja pilarien nurjahdus eivät sisälly tähän erilliseen
    mitoitukseen
  - Katteen pieni peltilippa rafter-tipin yli huomioidaan vain geometrian
    selityksenä, ei erillisenä puu-ulokkeena
"""

import math

from beam_analysis import (
    element_section_state,
    refine_nodes_mm,
    section_state_at_x_mm,
    solve_member_response,
    uniform_loads_for_nodes,
)
from foundation_checks import foundation_report_lines
from geometry_loader import expanded_members, load, member, surface, reference, profile_b, profile_h
from portaikko_loads import calculate_portaikko_foundation_loads
from timber_member_checks import governing_moment, sample_net_section_utilization_rect as sample_net_section_utilization


def tributary_widths_m(x_positions_mm, edge_start_mm, edge_end_mm):
    """Laskee tributäärileveydet x-suunnassa katon todellisista reunoista."""
    return [(right_mm - left_mm) / 1000.0 for left_mm, right_mm in tributary_ranges_mm(x_positions_mm, edge_start_mm, edge_end_mm)]


def tributary_ranges_mm(x_positions_mm, edge_start_mm, edge_end_mm):
    """Laskee tributäärialueiden x-rajat katon todellisista reunoista."""
    ranges = []
    for i, x_mm in enumerate(x_positions_mm):
        left_mm = edge_start_mm if i == 0 else 0.5 * (x_positions_mm[i - 1] + x_mm)
        right_mm = edge_end_mm if i == len(x_positions_mm) - 1 else 0.5 * (x_mm + x_positions_mm[i + 1])
        ranges.append((left_mm, right_mm))
    return ranges


def beam_state_inputs(reactions_kN, point_loads_down_kN, uniform_loads_down_kN_per_mm):
    """Valmistelee palkin tilanäytteistyksen kuormalistat kerran."""
    support_forces = [(float(xp_mm), float(r_kN)) for xp_mm, r_kN in reactions_kN.items()]
    point_loads = [(float(xp_mm), float(p_kN)) for xp_mm, p_kN in point_loads_down_kN]
    line_loads = [(float(a_mm), float(b_mm), float(q_kN_per_mm)) for a_mm, b_mm, q_kN_per_mm in uniform_loads_down_kN_per_mm]
    return support_forces, point_loads, line_loads


def beam_state_at_x_mm(x_mm, support_forces, point_loads, line_loads):
    """Laskee leikkausvoiman ja momentin yhdessä pisteessä."""

    V_kN = 0.0
    M_kNm = 0.0

    for xp_mm, force_kN in support_forces:
        if xp_mm <= x_mm + 1e-9:
            V_kN += force_kN
            M_kNm += force_kN * (x_mm - xp_mm) / 1000.0

    for xp_mm, load_kN in point_loads:
        if xp_mm <= x_mm + 1e-9:
            V_kN -= load_kN
            M_kNm -= load_kN * (x_mm - xp_mm) / 1000.0

    for a_mm, b_mm, q_kN_per_mm in line_loads:
        if x_mm > a_mm + 1e-9:
            l_mm = min(x_mm, b_mm) - a_mm
            if l_mm > 0.0:
                resultant_kN = -q_kN_per_mm * l_mm
                centroid_mm = a_mm + l_mm / 2.0
                V_kN += resultant_kN
                M_kNm += resultant_kN * (x_mm - centroid_mm) / 1000.0

    return {"V_kN": V_kN, "M_kNm": M_kNm}


def element_end_moment_kNm(elements, x_mm, side, tol=1e-9):
    """Palauttaa jäsenen sisäisen päätemomentin valitulta puolelta."""
    if side == "left":
        candidates = [elem for elem in elements if abs(float(elem["x1_mm"]) - x_mm) <= tol]
    elif side == "right":
        candidates = [elem for elem in elements if abs(float(elem["x0_mm"]) - x_mm) <= tol]
    else:
        raise ValueError(f"Tuntematon puoli: {side}")
    if not candidates:
        return section_state_at_x_mm(elements, x_mm)["M_kNm"]
    elem = candidates[0]
    return element_section_state(elem, x_mm)["M_kNm"]


def ok_mark(value, limit=100.0):
    return "OK ✓" if value <= limit else "YLITTYY ✗"


def governing_label(mode):
    return {
        "M": "taivutus",
        "V": "leikkaus",
        "lovi/M_netto": "lovi/taivutus",
        "lovi/V_netto": "lovi/leikkaus",
        "M_netto": "netto/taivutus",
        "V_netto": "netto/leikkaus",
    }.get(mode, mode)


def format_spacing_text(spacings_mm):
    """Muotoilee kattotuolijaon joko tasajaoksi tai todellisiksi väleiksi."""
    if not spacings_mm:
        return "-"
    first = spacings_mm[0]
    if all(abs(s_mm - first) < 1e-9 for s_mm in spacings_mm[1:]):
        return f"@ {first:.0f} mm"
    return "jaot " + " / ".join(f"{s_mm:.0f}" for s_mm in spacings_mm) + " mm"


def segment_uniform_loads(nodes_mm, q_kN_per_m_at_mm):
    """Rakentaa elementtikohtaiset viivakuormat jäsenakselin paikallisesta q(x):stä."""
    return [
        (a_mm, b_mm, q_kN_per_m_at_mm(0.5 * (a_mm + b_mm)) / 1000.0)
        for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:])
    ]


def bbox_contains_xy(x_mm, y_mm, polygon, tol=1e-9):
    xs = [p["x"] for p in polygon]
    ys = [p["y"] for p in polygon]
    return min(xs) - tol <= x_mm <= max(xs) + tol and min(ys) - tol <= y_mm <= max(ys) + tol


def plane_z_at_xy(polygon, x_mm, y_mm):
    """Laskee tasomaisen polygonin z-korkeuden annetussa (x, y)-pisteessä."""
    p0, p1, p2 = polygon[:3]
    ux = p1["x"] - p0["x"]
    uy = p1["y"] - p0["y"]
    uz = p1["z"] - p0["z"]
    vx = p2["x"] - p0["x"]
    vy = p2["y"] - p0["y"]
    vz = p2["z"] - p0["z"]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    if abs(nz) < 1e-12:
        return None
    d = -(nx * p0["x"] + ny * p0["y"] + nz * p0["z"])
    return -(nx * x_mm + ny * y_mm + d) / nz


def snow_drift_params(h_m, b_panel_m, sk_, gamma_s=2.0, mu1_=0.8):
    """EN 1991-1-3 §6.3: kinostuma korkeampaa estettä vasten."""
    s1_ = mu1_ * sk_
    if h_m <= 1e-9:
        return 0.0, mu1_, 0.0, s1_, s1_
    ls = min(5.0 * h_m, b_panel_m, 15.0)
    ls = max(ls, 0.5 * h_m)
    mu2_h_ = gamma_s * h_m / sk_
    mu2_ = min(max(mu2_h_, mu1_), 2.0)
    s_dr_ = mu2_ * sk_
    return ls, mu2_, mu2_h_, s1_, s_dr_


def analyse():
    geo = load("portaikko.json")

    roof = surface(geo, "surf.roof")
    roof_poly = roof["polygon"]
    roof_x0_mm = min(p["x"] for p in roof_poly)
    roof_x1_mm = max(p["x"] for p in roof_poly)
    roof_y0_mm = min(p["y"] for p in roof_poly)
    roof_y1_mm = max(p["y"] for p in roof_poly)
    roof_width_mm = roof_x1_mm - roof_x0_mm
    roof_depth_mm = roof_y1_mm - roof_y0_mm
    roof_depth_m = roof_depth_mm / 1000.0
    slope_deg = float(roof["placement"]["v"]["slope_deg"])
    slope_rad = math.radians(slope_deg)

    entrance_wall = reference(geo, "ref.house_wall_entrance")
    entrance_wall_poly = entrance_wall["polygon"]
    house_roof_ref = reference(geo, "ref.house.roof")
    house_roof_poly = house_roof_ref["polygon"]

    rafters = sorted(
        expanded_members(geo, "rafters"),
        key=lambda item: (float(item["axis_start"]["x"]), item["id"]),
    )
    if not rafters:
        raise ValueError("geometry/portaikko.json ei sisällä kattotuoleja")
    rafter_profile_keys = {
        (m["profile"]["name"], m["profile"].get("material"), profile_b(m), profile_h(m))
        for m in rafters
    }
    if len(rafter_profile_keys) != 1:
        raise ValueError("portaikko_kuormituslaskenta.py olettaa kaikkien kattotuolien olevan samaa profiilia")

    rafter = rafters[0]
    rafter_ids = [m["id"] for m in rafters]
    rafter_xs_mm = [float(m["axis_start"]["x"]) for m in rafters]
    tributary_widths = tributary_widths_m(rafter_xs_mm, roof_x0_mm, roof_x1_mm)
    rafter_tributary_ranges_mm = tributary_ranges_mm(rafter_xs_mm, roof_x0_mm, roof_x1_mm)
    rafter_spacings_mm = [b_mm - a_mm for a_mm, b_mm in zip(rafter_xs_mm, rafter_xs_mm[1:])]
    rafter_wall_y_mm = float(next(c["at"]["y"] for c in geo["connections"] if "ref.house_wall_entrance" in c["members"]))
    rafter_beam_y_mm = float(
        next(
            c["at"]["y"]
            for c in geo["connections"]
            if "beam.lp225" in c["members"] and any(member_id.startswith("rafter") for member_id in c["members"])
        )
    )
    rafter_tip_y_mm = max(float(m["axis_end"]["y"]) for m in rafters)
    rafter_main_span_mm = rafter_beam_y_mm - rafter_wall_y_mm
    rafter_overhang_mm = rafter_tip_y_mm - rafter_beam_y_mm
    roof_sheet_extra_mm = roof_y1_mm - rafter_tip_y_mm
    rafter_delta_limit_mm = rafter_main_span_mm / 300.0
    widest_rafter_id, widest_tributary_m = max(
        zip(rafter_ids, tributary_widths),
        key=lambda item: item[1],
    )

    beam = member(geo, "beams", "beam.lp225")
    beam_start_x_mm = float(beam["axis_start"]["x"])
    beam_end_x_mm = float(beam["axis_end"]["x"])
    beam_supports = sorted(
        (
            conn["members"][1],
            float(conn["at"]["x"]),
            float(conn["at"]["z"]),
        )
        for conn in geo["connections"]
        if conn["id"].startswith("con.beam.lp225.on.col")
    )
    beam_supports.sort(key=lambda item: item[1])
    beam_support_xs_mm = [x_mm for _, x_mm, _ in beam_supports]
    beam_spans_mm = [b_mm - a_mm for a_mm, b_mm in zip(beam_support_xs_mm, beam_support_xs_mm[1:])]
    beam_right_overhang_mm = beam_end_x_mm - beam_support_xs_mm[-1]
    beam_delta_limit_mm = max(beam_spans_mm) / 300.0

    rafter_b_mm = profile_b(rafter)
    rafter_h_mm = profile_h(rafter)
    beam_b_mm = profile_b(beam)
    beam_h_mm = profile_h(beam)

    # Kuormat ja materiaalit
    gk_roofing = 0.20
    rho_C24 = 420.0
    gamma_C24 = rho_C24 * 9.81 / 1000.0
    gamma_GL30c = 5.0
    rafter_self_kNm = (rafter_b_mm / 1000.0) * (rafter_h_mm / 1000.0) * gamma_C24 / math.cos(slope_rad)
    beam_self_kNm = (beam_b_mm / 1000.0) * (beam_h_mm / 1000.0) * gamma_GL30c

    sk = 2.0
    mu1 = 0.8
    Ce = 1.0
    Ct = 1.0
    s_roof = mu1 * Ce * Ct * sk

    Qk_huolto = 1.0
    qk_H = 0.4

    vb0 = 21.0
    rho_air = 1.25
    z0 = 0.05
    z_min = 2.0
    all_z_mm = [
        p["z"] for s in geo.get("surfaces", []) for p in s.get("polygon", []) if "z" in p
    ] + [
        m[k]["z"]
        for grp in geo["members"].values() for m in grp
        for k in ("axis_start", "axis_end", "base", "top")
        if k in m and isinstance(m[k], dict) and "z" in m[k]
    ]
    z_ref = math.ceil(max(all_z_mm) / 500.0) * 0.5
    kr = 0.19 * (z0 / 0.05) ** 0.07
    cr_z = kr * math.log(max(z_ref, z_min) / z0)
    Iv_z = 1.0 / math.log(max(z_ref, z_min) / z0)
    vm_z = cr_z * vb0
    qp_z = (1.0 + 7.0 * Iv_z) * 0.5 * rho_air * vm_z**2 / 1000.0

    # Rakennukseen kiinnittyvä avoin pulpettikatos: käytetään yksinkertaistettua
    # attached-roof mallia vapaan katoksen nettopaineiden sijaan.
    wind_model = "Rakennukseen kiinnittyvä avoin pulpettikatos"
    cp_net_down = 0.20
    cp_net_up = -1.20
    w_wind_down = cp_net_down * qp_z
    w_wind_up = cp_net_up * qp_z

    gammaG = 1.35
    gammaQ = 1.50
    psi0_W = 0.6
    psi0_snow = 0.7

    rafter_W_mm3 = rafter_b_mm * rafter_h_mm**2 / 6.0
    rafter_A_mm2 = rafter_b_mm * rafter_h_mm
    rafter_I_mm4 = rafter_b_mm * rafter_h_mm**3 / 12.0
    rafter_EI_Nmm2 = 11000.0 * rafter_I_mm4
    fm_d_C24 = 0.65 * 24.0 / 1.3
    fv_d_C24 = 0.65 * 4.0 / 1.3
    rafter_MRd = fm_d_C24 * rafter_W_mm3 / 1.0e6
    rafter_VRd = fv_d_C24 * rafter_A_mm2 / 1.5e3

    rafter_notch_connections = [
        conn
        for conn in geo["connections"]
        if conn.get("type") == "notched_over"
        and "beam.lp225" in conn.get("members", [])
        and any(member_id.startswith("rafter") for member_id in conn.get("members", []))
        and conn.get("cuts")
    ]
    if not rafter_notch_connections:
        raise ValueError("geometry/portaikko.json ei sisällä kattotuolien lovitusta LP225-palkilla")

    def cut_signature(cut):
        return tuple((key, cut.get(key)) for key in ("kind", "reference", "offset_mm", "depth_mm", "length_mm", "heel_depth_mm", "seat_length_mm", "side"))

    rafter_notch_cuts = [conn["cuts"][0] for conn in rafter_notch_connections]
    if len({cut_signature(cut) for cut in rafter_notch_cuts}) != 1:
        raise ValueError("portaikko_kuormituslaskenta.py olettaa kaikille kattotuoleille saman loven LP225-tuella")

    rafter_notch_cut = rafter_notch_cuts[0]
    if rafter_notch_cut.get("kind") != "bevel_notch":
        raise ValueError("portaikko_kuormituslaskenta.py tukee kattotuolille vain bevel_notch-loven")
    if rafter_notch_cut["reference"] != "axis_end":
        raise ValueError("portaikko_kuormituslaskenta.py olettaa loven reference-ankkurin olevan kattotuolin axis_end-päässä")
    rafter_notch_side = rafter_notch_cut.get("side")
    if rafter_notch_side not in {"bottom", "top"}:
        raise ValueError("portaikko_kuormituslaskenta.py odottaa bevel_notch-lovelle side-kentän")

    rafter_notch_depth_mm = float(rafter_notch_cut["depth_mm"])
    rafter_notch_length_mm = float(rafter_notch_cut["length_mm"])
    rafter_notch_offset_mm = float(rafter_notch_cut.get("offset_mm", 0.0))
    rafter_notch_start_y_mm = rafter_tip_y_mm - rafter_notch_offset_mm - rafter_notch_length_mm

    def rafter_notch_depth_at_y_mm(y_mm):
        distance_from_end_mm = rafter_tip_y_mm - y_mm
        if distance_from_end_mm < rafter_notch_offset_mm:
            return rafter_notch_depth_mm
        local_mm = distance_from_end_mm - rafter_notch_offset_mm
        if local_mm < 0.0 or local_mm > rafter_notch_length_mm:
            return 0.0
        return rafter_notch_depth_mm * max(0.0, 1.0 - local_mm / rafter_notch_length_mm)

    def rafter_section_h_mm_at_y(y_mm):
        return max(1e-9, rafter_h_mm - rafter_notch_depth_at_y_mm(y_mm))

    rafter_notch_h_min_mm = max(1e-9, rafter_h_mm - rafter_notch_depth_mm)
    rafter_notch_depth_at_beam_mm = rafter_notch_depth_at_y_mm(rafter_beam_y_mm)
    rafter_notch_h_at_beam_mm = rafter_section_h_mm_at_y(rafter_beam_y_mm)
    rafter_notch_depth_rule_ok = rafter_notch_depth_mm <= rafter_h_mm / 3.0
    rafter_support_model = "Seinäpää kiertymäjäykkä, LP225-pää nivelletty"

    beam_W_mm3 = beam_b_mm * beam_h_mm**2 / 6.0
    beam_A_mm2 = beam_b_mm * beam_h_mm
    beam_I_mm4 = beam_b_mm * beam_h_mm**3 / 12.0
    beam_EI_Nmm2 = 13000.0 * beam_I_mm4
    fm_d_GL30c = 0.65 * 30.0 / 1.25
    fv_d_GL30c = 0.65 * 3.5 / 1.25
    beam_MRd = fm_d_GL30c * beam_W_mm3 / 1.0e6
    beam_VRd = fv_d_GL30c * beam_A_mm2 / 1.5e3

    # Kinostuma rakennuksen katon / seinän aiheuttamana rafterikohtaisesti.
    entrance_x0_mm = min(p["x"] for p in entrance_wall_poly)
    entrance_x1_mm = max(p["x"] for p in entrance_wall_poly)
    entrance_y_mm = float(entrance_wall_poly[0]["y"])
    entrance_top_z_mm = max(p["z"] for p in entrance_wall_poly)
    glazed_corner_point = {
        "source": "ref.house.roof (x=7200, y=1800)",
        "x_mm": 7200.0,
        "y_mm": 1800.0,
    }
    if not bbox_contains_xy(glazed_corner_point["x_mm"], glazed_corner_point["y_mm"], roof_poly):
        raise ValueError("Kinostuman konservatiivinen kulmapiste ei osu surf.roof-pintaan")
    if not bbox_contains_xy(glazed_corner_point["x_mm"], glazed_corner_point["y_mm"], house_roof_poly):
        raise ValueError("Kinostuman konservatiivinen kulmapiste ei osu ref.house.roof-pintaan")
    glazed_corner_point["roof_z_mm"] = plane_z_at_xy(roof_poly, glazed_corner_point["x_mm"], glazed_corner_point["y_mm"])
    glazed_corner_point["obstacle_z_mm"] = plane_z_at_xy(house_roof_poly, glazed_corner_point["x_mm"], glazed_corner_point["y_mm"])
    glazed_corner_point["h_mm"] = max(0.0, glazed_corner_point["obstacle_z_mm"] - glazed_corner_point["roof_z_mm"])

    drift_by_rafter = {}
    for rid, x_mm, (trib_left_mm, trib_right_mm) in zip(rafter_ids, rafter_xs_mm, rafter_tributary_ranges_mm):
        roof_z_at_wall_mm = plane_z_at_xy(roof_poly, x_mm, rafter_wall_y_mm)
        obstacle_candidates = []
        if entrance_x0_mm <= x_mm <= entrance_x1_mm and abs(rafter_wall_y_mm - entrance_y_mm) < 1e-9:
            obstacle_candidates.append(
                {
                    "source": "ref.house_wall_entrance",
                    "reference_x_mm": x_mm,
                    "reference_y_mm": rafter_wall_y_mm,
                    "roof_z_mm": roof_z_at_wall_mm,
                    "obstacle_z_mm": entrance_top_z_mm,
                    "h_mm": max(0.0, entrance_top_z_mm - roof_z_at_wall_mm),
                }
            )
        if bbox_contains_xy(x_mm, rafter_wall_y_mm, house_roof_poly):
            house_roof_z_mm = plane_z_at_xy(house_roof_poly, x_mm, rafter_wall_y_mm)
            if house_roof_z_mm is not None:
                obstacle_candidates.append(
                    {
                        "source": "ref.house.roof",
                        "reference_x_mm": x_mm,
                        "reference_y_mm": rafter_wall_y_mm,
                        "roof_z_mm": roof_z_at_wall_mm,
                        "obstacle_z_mm": house_roof_z_mm,
                        "h_mm": max(0.0, house_roof_z_mm - roof_z_at_wall_mm),
                    }
                )
        if trib_left_mm - 1e-9 <= glazed_corner_point["x_mm"] <= trib_right_mm + 1e-9:
            obstacle_candidates.append(
                {
                    "source": glazed_corner_point["source"],
                    "reference_x_mm": glazed_corner_point["x_mm"],
                    "reference_y_mm": glazed_corner_point["y_mm"],
                    "roof_z_mm": glazed_corner_point["roof_z_mm"],
                    "obstacle_z_mm": glazed_corner_point["obstacle_z_mm"],
                    "h_mm": glazed_corner_point["h_mm"],
                }
            )
        if obstacle_candidates:
            critical_candidate = max(obstacle_candidates, key=lambda item: item["h_mm"])
        else:
            critical_candidate = {
                "source": "-",
                "reference_x_mm": x_mm,
                "reference_y_mm": rafter_wall_y_mm,
                "roof_z_mm": roof_z_at_wall_mm,
                "obstacle_z_mm": roof_z_at_wall_mm,
                "h_mm": 0.0,
            }
        h_mm = critical_candidate["h_mm"]
        ls_m, mu2, mu2_h, _, s_peak_kNm2 = snow_drift_params(h_mm / 1000.0, roof_depth_m, sk, mu1_=mu1)
        drift_by_rafter[rid] = {
            "id": rid,
            "x_mm": x_mm,
            "source": critical_candidate["source"],
            "reference_x_mm": critical_candidate["reference_x_mm"],
            "reference_y_mm": critical_candidate["reference_y_mm"],
            "roof_z_reference_mm": critical_candidate["roof_z_mm"],
            "obstacle_z_mm": critical_candidate["obstacle_z_mm"],
            "h_mm": h_mm,
            "ls_m": ls_m,
            "mu2": mu2,
            "mu2_h": mu2_h,
            "s_peak_kNm2": s_peak_kNm2,
        }

    critical_drift = max(drift_by_rafter.values(), key=lambda item: (item["s_peak_kNm2"], item["h_mm"]))

    def drift_snow_kNm2(rid, y_mm):
        info = drift_by_rafter[rid]
        if info["ls_m"] <= 1e-9:
            return s_roof
        distance_m = max(0.0, (y_mm - rafter_wall_y_mm) / 1000.0)
        s_drift_local = info["s_peak_kNm2"] * max(0.0, 1.0 - distance_m / info["ls_m"])
        return max(s_roof, s_drift_local)

    analysis_elem_mm = 200.0

    def solve_rafter_result(rid, x_mm, trib_w_m, roof_area_kNm2_at, gamma_g_self, point_loads_kN=None):
        point_loads_kN = [] if point_loads_kN is None else list(point_loads_kN)
        point_positions_mm = [y_mm for y_mm, _ in point_loads_kN]
        rafter_nodes = refine_nodes_mm(
            [rafter_wall_y_mm, rafter_notch_start_y_mm, rafter_beam_y_mm, rafter_tip_y_mm, *point_positions_mm],
            analysis_elem_mm,
        )
        rafter_uniform = segment_uniform_loads(
            rafter_nodes,
            lambda y_mm: roof_area_kNm2_at(rid, x_mm, y_mm) * trib_w_m + gamma_g_self * rafter_self_kNm,
        )
        rafter_EI_by_segment_Nmm2 = [
            11000.0 * rafter_b_mm * rafter_section_h_mm_at_y(0.5 * (a_mm + b_mm)) ** 3 / 12.0
            for a_mm, b_mm in zip(rafter_nodes, rafter_nodes[1:])
        ]
        response, internal, delta = solve_member_response(
            rafter_nodes,
            [rafter_wall_y_mm, rafter_beam_y_mm],
            point_loads_kN=point_loads_kN,
            uniform_loads_kN_per_mm=rafter_uniform,
            EI_by_segment_Nmm2=rafter_EI_by_segment_Nmm2,
            fixed_rotations_mm=[rafter_wall_y_mm],
        )
        reactions = response["reactions_kN"]
        support_moments = response["support_moments_kNm"]
        wall_end_moment_kNm = element_end_moment_kNm(response["elements"], rafter_wall_y_mm, side="right")
        beam_end_moment_kNm = element_end_moment_kNm(response["elements"], rafter_beam_y_mm, side="left")
        moment_gov = governing_moment(internal)
        max_shear_kN = abs(internal["V_abs"]["value_kN"])
        notch_check = sample_net_section_utilization(
            response["elements"],
            section_h_mm_at_x=rafter_section_h_mm_at_y,
            b_mm=rafter_b_mm,
            fm_d_Nmm2=fm_d_C24,
            fv_d_Nmm2=fv_d_C24,
            x_start_mm=rafter_notch_start_y_mm,
            x_end_mm=rafter_tip_y_mm,
            step_mm=1.0,
        )
        eta_M_pct = moment_gov["value_kNm"] / rafter_MRd * 100.0
        eta_V_pct = max_shear_kN / rafter_VRd * 100.0
        governing_check = max(
            [
                {"mode": "M", "value_pct": eta_M_pct},
                {"mode": "V", "value_pct": eta_V_pct},
                {"mode": "lovi/" + notch_check["eta_gov"]["mode"], "value_pct": notch_check["eta_gov"]["value_pct"]},
            ],
            key=lambda item: item["value_pct"],
        )
        q_at_beam_kNm = roof_area_kNm2_at(rid, x_mm, rafter_beam_y_mm) * trib_w_m + gamma_g_self * rafter_self_kNm
        return {
            "id": rid,
            "x_mm": x_mm,
            "trib_w_m": trib_w_m,
            "q_proj_kNm": q_at_beam_kNm,
            "R_wall_kN": reactions[rafter_wall_y_mm],
            "R_beam_kN": reactions[rafter_beam_y_mm],
            "M_pos": internal["M_pos"],
            "M_neg": internal["M_neg"],
            "M_gov": moment_gov,
            "V_abs": internal["V_abs"],
            "delta": delta,
            "point_loads_kN": point_loads_kN,
            "notch": notch_check,
            "support_moments_kNm": support_moments,
            "M_wall_kNm": wall_end_moment_kNm,
            "M_beam_kNm": beam_end_moment_kNm,
            "eta_M": eta_M_pct,
            "eta_V": eta_V_pct,
            "eta_notch": notch_check["eta_gov"]["value_pct"],
            "eta_gov": governing_check["value_pct"],
            "gov_mode": governing_check["mode"],
        }

    def solve_beam_result(point_loads_kN, gamma_g_self):
        beam_nodes = refine_nodes_mm(
            [beam_start_x_mm, beam_end_x_mm, *beam_support_xs_mm, *rafter_xs_mm],
            analysis_elem_mm,
        )
        beam_uniform = uniform_loads_for_nodes(beam_nodes, gamma_g_self * beam_self_kNm / 1000.0)
        beam_response, beam_internal, beam_delta = solve_member_response(
            beam_nodes,
            beam_support_xs_mm,
            point_loads_kN=point_loads_kN,
            uniform_loads_kN_per_mm=beam_uniform,
            EI_Nmm2=beam_EI_Nmm2,
        )
        beam_moment_gov = governing_moment(beam_internal)
        return {
            "reactions_kN": beam_response["reactions_kN"],
            "M_pos": beam_internal["M_pos"],
            "M_neg": beam_internal["M_neg"],
            "M_gov": beam_moment_gov,
            "V_abs": beam_internal["V_abs"],
            "delta": beam_delta,
            "eta_M_pos": beam_internal["M_pos"]["value_kNm"] / beam_MRd * 100.0,
            "eta_M_neg": (-beam_internal["M_neg"]["value_kNm"]) / beam_MRd * 100.0,
            "eta_M": beam_moment_gov["value_kNm"] / beam_MRd * 100.0,
            "eta_V": abs(beam_internal["V_abs"]["value_kN"]) / beam_VRd * 100.0,
        }

    def assemble_case(case_data, rafter_results, beam_point_loads_kN=None):
        if beam_point_loads_kN is None:
            beam_point_loads_kN = [(r["x_mm"], r["R_beam_kN"]) for r in rafter_results]
        beam_result = solve_beam_result(beam_point_loads_kN, case_data["gamma_g_self"])
        return {
            "case": case_data,
            "rafters": rafter_results,
            "beam_point_loads_kN": beam_point_loads_kN,
            "wall_total_kN": sum(r["R_wall_kN"] for r in rafter_results),
            "beam_total_kN": sum(p_kN for _, p_kN in beam_point_loads_kN),
            "beam": beam_result,
        }

    regular_cases = {
        "ULS A": {
            "title": "1.35G + 1.5S + 1.5·0.6W↓",
            "gamma_g_self": gammaG,
            "roof_area_kNm2_at": lambda _rid, _x_mm, _y_mm: gammaG * gk_roofing + gammaQ * s_roof + gammaQ * psi0_W * w_wind_down,
        },
        "ULS B": {
            "title": "1.35G + 1.5W↓ + 1.5·0.7S",
            "gamma_g_self": gammaG,
            "roof_area_kNm2_at": lambda _rid, _x_mm, _y_mm: gammaG * gk_roofing + gammaQ * w_wind_down + gammaQ * psi0_snow * s_roof,
        },
        "ULS DRIFT": {
            "title": "1.35G + 1.5S_kin + 1.5·0.6W↓",
            "gamma_g_self": gammaG,
            "roof_area_kNm2_at": lambda rid, _x_mm, y_mm: gammaG * gk_roofing + gammaQ * drift_snow_kNm2(rid, y_mm) + gammaQ * psi0_W * w_wind_down,
        },
        "SLS": {
            "title": "G + S",
            "gamma_g_self": 1.0,
            "roof_area_kNm2_at": lambda _rid, _x_mm, _y_mm: gk_roofing + s_roof,
        },
        "SLS DRIFT": {
            "title": "G + S_kin",
            "gamma_g_self": 1.0,
            "roof_area_kNm2_at": lambda rid, _x_mm, y_mm: gk_roofing + drift_snow_kNm2(rid, y_mm),
        },
        "UPLIFT": {
            "title": "0.9G + 1.5W↑",
            "gamma_g_self": 0.9,
            "roof_area_kNm2_at": lambda _rid, _x_mm, _y_mm: 0.9 * gk_roofing + 1.5 * w_wind_up,
        },
    }

    results = {}
    for case_key, case_data in regular_cases.items():
        rafter_results = [
            solve_rafter_result(rid, x_mm, trib_w_m, case_data["roof_area_kNm2_at"], case_data["gamma_g_self"])
            for rid, x_mm, trib_w_m in zip(rafter_ids, rafter_xs_mm, tributary_widths)
        ]
        results[case_key] = assemble_case(case_data, rafter_results)

    maintenance_case = {
        "title": "1.35G + 1.5qH + 1.5·0.7S + 1.5Qk",
        "gamma_g_self": gammaG,
        "roof_area_kNm2_at": lambda _rid, _x_mm, _y_mm: gammaG * gk_roofing + gammaQ * qk_H + gammaQ * psi0_snow * s_roof,
    }
    maintenance_base_results = [
        solve_rafter_result(rid, x_mm, trib_w_m, maintenance_case["roof_area_kNm2_at"], maintenance_case["gamma_g_self"])
        for rid, x_mm, trib_w_m in zip(rafter_ids, rafter_xs_mm, tributary_widths)
    ]
    maintenance_base_by_id = {r["id"]: r for r in maintenance_base_results}
    maintenance_candidate_y_mm = refine_nodes_mm([rafter_wall_y_mm, rafter_beam_y_mm, rafter_tip_y_mm], 50.0)
    maintenance_point_design_kN = gammaQ * Qk_huolto
    best_rafter_scenario = None
    best_beam_scenario = None

    for rid, x_mm, trib_w_m in zip(rafter_ids, rafter_xs_mm, tributary_widths):
        for y_point_mm in maintenance_candidate_y_mm:
            modified_rafter = solve_rafter_result(
                rid,
                x_mm,
                trib_w_m,
                maintenance_case["roof_area_kNm2_at"],
                maintenance_case["gamma_g_self"],
                point_loads_kN=[(y_point_mm, maintenance_point_design_kN)],
            )
            scenario_rafters = [
                modified_rafter if other_rid == rid else maintenance_base_by_id[other_rid]
                for other_rid in rafter_ids
            ]
            scenario_point = {
                "rafter_id": rid,
                "x_mm": x_mm,
                "y_mm": y_point_mm,
                "P_kN": maintenance_point_design_kN,
            }
            scenario_eta_rafter = max(r["eta_gov"] for r in scenario_rafters)
            if best_rafter_scenario is None or scenario_eta_rafter > best_rafter_scenario["max_eta_gov"]:
                best_rafter_scenario = {
                    "max_eta_gov": scenario_eta_rafter,
                    "point": scenario_point,
                    "rafters": [dict(r) for r in scenario_rafters],
                    "wall_total_kN": sum(r["R_wall_kN"] for r in scenario_rafters),
                }

            scenario_beam_point_loads = [(r["x_mm"], r["R_beam_kN"]) for r in scenario_rafters]
            scenario_beam = solve_beam_result(scenario_beam_point_loads, maintenance_case["gamma_g_self"])
            if best_beam_scenario is None or scenario_beam["eta_M"] > best_beam_scenario["beam"]["eta_M"]:
                best_beam_scenario = {
                    "point": scenario_point,
                    "beam_point_loads_kN": scenario_beam_point_loads,
                    "beam": scenario_beam,
                }

    results["ULS MAINT"] = {
        "case": maintenance_case,
        "rafters": best_rafter_scenario["rafters"],
        "beam_point_loads_kN": best_beam_scenario["beam_point_loads_kN"],
        "wall_total_kN": best_rafter_scenario["wall_total_kN"],
        "beam_total_kN": sum(p_kN for _, p_kN in best_beam_scenario["beam_point_loads_kN"]),
        "beam": best_beam_scenario["beam"],
        "maintenance_rafter_point": best_rafter_scenario["point"],
        "maintenance_beam_point": best_beam_scenario["point"],
    }

    down_case_keys = ("ULS A", "ULS B", "ULS DRIFT", "ULS MAINT")
    rafter_case_etas = {case_key: max(r["eta_gov"] for r in results[case_key]["rafters"]) for case_key in down_case_keys}
    beam_case_etas = {case_key: results[case_key]["beam"]["eta_M"] for case_key in down_case_keys}

    down_case_rafter = max(down_case_keys, key=lambda case_key: rafter_case_etas[case_key])
    down_case_beam = max(down_case_keys, key=lambda case_key: beam_case_etas[case_key])
    sls_case_rafter = "SLS DRIFT" if down_case_rafter == "ULS DRIFT" else "SLS"
    sls_case_beam = "SLS DRIFT" if down_case_beam == "ULS DRIFT" else "SLS"
    down_rafter_results = {r["id"]: r for r in results[down_case_rafter]["rafters"]}
    sls_rafter_results = {r["id"]: r for r in results[sls_case_rafter]["rafters"]}
    uplift_rafter_results = {r["id"]: r for r in results["UPLIFT"]["rafters"]}

    critical_rafter = max(results[down_case_rafter]["rafters"], key=lambda r: r["eta_gov"])
    beam_governing = results[down_case_beam]["beam"]
    max_column_down = max(results[down_case_beam]["beam"]["reactions_kN"].items(), key=lambda item: item[1])
    max_column_uplift = min(results["UPLIFT"]["beam"]["reactions_kN"].items(), key=lambda item: item[1])

    foundation = calculate_portaikko_foundation_loads(
        geo,
        results,
        beam_support_xs_mm,
        down_case_keys,
        gammaG,
        gammaQ,
        gamma_GL30c,
    )

    return {
        "geo": {
            "roof_width_mm": roof_width_mm,
            "roof_depth_mm": roof_depth_mm,
            "slope_deg": slope_deg,
            "rafter_spacings_mm": rafter_spacings_mm,
            "rafter_main_span_mm": rafter_main_span_mm,
            "rafter_overhang_mm": rafter_overhang_mm,
            "roof_sheet_extra_mm": roof_sheet_extra_mm,
            "rafter_wall_y_mm": rafter_wall_y_mm,
            "rafter_beam_y_mm": rafter_beam_y_mm,
            "rafter_tip_y_mm": rafter_tip_y_mm,
            "rafter_xs_mm": rafter_xs_mm,
            "tributary_widths": tributary_widths,
            "widest_rafter_id": widest_rafter_id,
            "widest_tributary_m": widest_tributary_m,
            "rafter_notch_start_y_mm": rafter_notch_start_y_mm,
            "beam_start_x_mm": beam_start_x_mm,
            "beam_end_x_mm": beam_end_x_mm,
            "beam_supports": beam_supports,
            "beam_spans_mm": beam_spans_mm,
            "beam_right_overhang_mm": beam_right_overhang_mm,
            "rafter_profile_name": rafter["profile"]["name"],
            "rafter_b_mm": rafter_b_mm,
            "rafter_h_mm": rafter_h_mm,
            "beam_b_mm": beam_b_mm,
            "beam_h_mm": beam_h_mm,
        },
        "loads": {
            "gk_roofing": gk_roofing,
            "rafter_self_kNm": rafter_self_kNm,
            "beam_self_kNm": beam_self_kNm,
            "sk": sk,
            "mu1": mu1,
            "Ce": Ce,
            "Ct": Ct,
            "s_roof": s_roof,
            "Qk_huolto": Qk_huolto,
            "qk_H": qk_H,
            "vb0": vb0,
            "z_ref": z_ref,
            "cr_z": cr_z,
            "Iv_z": Iv_z,
            "vm_z": vm_z,
            "qp_z": qp_z,
            "wind_model": wind_model,
            "cp_net_down": cp_net_down,
            "cp_net_up": cp_net_up,
            "w_wind_down": w_wind_down,
            "w_wind_up": w_wind_up,
        },
        "drift": {
            "critical_rafter_id": critical_drift["id"],
            "critical_source": critical_drift["source"],
            "reference_x_mm": critical_drift["reference_x_mm"],
            "reference_y_mm": critical_drift["reference_y_mm"],
            "h_mm": critical_drift["h_mm"],
            "ls_m": critical_drift["ls_m"],
            "mu2": critical_drift["mu2"],
            "mu2_h": critical_drift["mu2_h"],
            "s_peak_kNm2": critical_drift["s_peak_kNm2"],
            "s_at_beam_kNm2": drift_snow_kNm2(critical_drift["id"], rafter_beam_y_mm),
        },
        "sections": {
            "rafter_MRd": rafter_MRd,
            "rafter_VRd": rafter_VRd,
            "beam_MRd": beam_MRd,
            "beam_VRd": beam_VRd,
            "fm_d_C24": fm_d_C24,
            "fv_d_C24": fv_d_C24,
            "fm_d_GL30c": fm_d_GL30c,
            "fv_d_GL30c": fv_d_GL30c,
            "rafter_delta_limit_mm": rafter_delta_limit_mm,
            "beam_delta_limit_mm": beam_delta_limit_mm,
            "rafter_notch_depth_mm": rafter_notch_depth_mm,
            "rafter_notch_length_mm": rafter_notch_length_mm,
            "rafter_notch_side": rafter_notch_side,
            "rafter_notch_h_at_beam_mm": rafter_notch_h_at_beam_mm,
            "rafter_notch_h_min_mm": rafter_notch_h_min_mm,
            "rafter_notch_depth_at_beam_mm": rafter_notch_depth_at_beam_mm,
            "rafter_notch_depth_rule_ok": rafter_notch_depth_rule_ok,
            "rafter_support_model": rafter_support_model,
        },
        "cases": results,
        "down_case_keys": down_case_keys,
        "rafter_case_etas": rafter_case_etas,
        "beam_case_etas": beam_case_etas,
        "down_case_rafter": down_case_rafter,
        "down_case_beam": down_case_beam,
        "sls_case_rafter": sls_case_rafter,
        "sls_case_beam": sls_case_beam,
        "critical_rafter": critical_rafter,
        "max_column_down": max_column_down,
        "max_column_uplift": max_column_uplift,
        "down_rafter_results": down_rafter_results,
        "sls_rafter_results": sls_rafter_results,
        "uplift_rafter_results": uplift_rafter_results,
        "beam_governing": beam_governing,
        "foundation": foundation,
    }


def main():
    data = analyse()
    geo = data["geo"]
    loads = data["loads"]
    drift = data["drift"]
    sec = data["sections"]
    cases = data["cases"]
    foundation = data["foundation"]

    W = 64
    dw = "=" * W

    print(dw)
    print("  PORTAIKON KATOKSEN KUORMITUSLASKENTA – ETELÄSUOMI")
    print("  EN 1990 / EN 1991-1-3 / EN 1991-1-4 / EN 1995-1-1")
    print(dw)

    print("\n── GEOMETRIA ─────────────────────────────────────────────────")
    print(f"  Katon kaltevuus                 {geo['slope_deg']:.0f}°  y-suunnassa (seinältä portaille)")
    print(f"  Katon leveys x-suunnassa        {geo['roof_width_mm']:.0f} mm")
    print(f"  Katon vaakasyvyys y-suunnassa   {geo['roof_depth_mm']:.0f} mm")
    print(f"  Kattotuolit                     {len(geo['rafter_xs_mm'])} kpl {format_spacing_text(geo['rafter_spacings_mm'])}")
    print(f"  Tukilinja seinällä              y = {geo['rafter_wall_y_mm']:.0f} mm")
    print(f"  Tukilinja LP225×90-palkilla     y = {geo['rafter_beam_y_mm']:.0f} mm")
    print(f"  Kattotuolin jänneväli           {geo['rafter_main_span_mm']:.0f} mm  + puu-uloke {geo['rafter_overhang_mm']:.0f} mm")
    print(f"  Katteen lisäuloke               {geo['roof_sheet_extra_mm']:.0f} mm rafter-tipin yli")
    print(f"  LP225×90-palkki                 x = {geo['beam_start_x_mm']:.0f} ... {geo['beam_end_x_mm']:.0f} mm")
    print(f"  Palkin jännevälit               {' / '.join(f'{span:.0f}' for span in geo['beam_spans_mm'])} mm, oikea uloke {geo['beam_right_overhang_mm']:.0f} mm")
    print("  Tributäärileveydet x-suunnassa  " + " / ".join(f"{w*1000:.0f}" for w in geo["tributary_widths"]) + " mm")

    print("\n── KUORMAT ───────────────────────────────────────────────────")
    print(f"  Kate + alusrakenne gk           {loads['gk_roofing']:.2f} kN/m²")
    print(f"  Kattotuolin omapaino            {loads['rafter_self_kNm']:.3f} kN/m  (vaakaprojisoitu)")
    print(f"  LP225×90 omapaino               {loads['beam_self_kNm']:.3f} kN/m")
    print(f"  Lumikuorma sk                   {loads['sk']:.1f} kN/m²")
    print(f"  μ1 · Ce · Ct                    {loads['mu1']:.1f} · {loads['Ce']:.1f} · {loads['Ct']:.1f}")
    print(f"  Lumikuorma katolla s            {loads['s_roof']:.2f} kN/m²")
    print(f"  Huoltokuorma Qk / qk,H          {loads['Qk_huolto']:.1f} kN / {loads['qk_H']:.1f} kN/m²")
    print(f"  Tuulimalli                      {loads['wind_model']}")
    print(f"  qp({loads['z_ref']:.1f} m)                     {loads['qp_z']:.3f} kN/m²")
    print(f"  cp,net alas / ylös              {loads['cp_net_down']:.2f} / {loads['cp_net_up']:.2f}")
    print(f"  Tuuli alas / ylös               {loads['w_wind_down']:.3f} / {loads['w_wind_up']:.3f} kN/m²")
    print(
        f"  Kinostuma max                   {drift['critical_source']} @ x={drift['reference_x_mm']:.0f}, y={drift['reference_y_mm']:.0f}"
        f" → h = {drift['h_mm']/1000.0:.2f} m, ls = {drift['ls_m']:.2f} m, μ2 = {drift['mu2']:.2f}"
    )
    print(f"  s_kin,max / s_kin@palkki        {drift['s_peak_kNm2']:.2f} / {drift['s_at_beam_kNm2']:.2f} kN/m² @ {drift['critical_rafter_id']}")

    print("\n── HALLITSEVAT KUORMATAPAUKSET ──────────────────────────────")
    print(f"  Kattotuolit (alas)              {data['down_case_rafter']}: {cases[data['down_case_rafter']]['case']['title']}")
    print(f"  LP225×90-palkki (alas)          {data['down_case_beam']}: {cases[data['down_case_beam']]['case']['title']}")
    print("  η kattotuoli tapausvertailu     " + " / ".join(f"{k}:{data['rafter_case_etas'][k]:.1f}%" for k in data["down_case_keys"]))
    print("  η_M LP225 tapausvertailu        " + " / ".join(f"{k}:{data['beam_case_etas'][k]:.1f}%" for k in data["down_case_keys"]))
    maint_rafter = cases["ULS MAINT"]["maintenance_rafter_point"]
    maint_beam = cases["ULS MAINT"]["maintenance_beam_point"]
    print(f"  Huoltopiste kattotuoli          {maint_rafter['rafter_id']} @ y={maint_rafter['y_mm']:.0f} mm")
    print(f"  Huoltopiste LP225               {maint_beam['rafter_id']} @ y={maint_beam['y_mm']:.0f} mm")
    print(f"  Nosto                           UPLIFT: {cases['UPLIFT']['case']['title']}")

    print(f"\n── KATTOTUOLIT {geo['rafter_h_mm']:.0f}×{geo['rafter_b_mm']:.0f} ──────────")
    print(f"  MRd = {sec['rafter_MRd']:.2f} kNm,  VRd = {sec['rafter_VRd']:.2f} kN,  δ_lim = {sec['rafter_delta_limit_mm']:.1f} mm")
    print(f"  fm,d = {sec['fm_d_C24']:.2f} N/mm²,  fv,d = {sec['fv_d_C24']:.2f} N/mm²")
    print(f"  Tukimalli                       {sec['rafter_support_model']}")
    print(
        f"  Lovi LP225-tuella               bevel_notch {sec['rafter_notch_side']} {sec['rafter_notch_depth_mm']:.0f} × {sec['rafter_notch_length_mm']:.0f} mm,"
        f"  h_net@palkki = {sec['rafter_notch_h_at_beam_mm']:.1f} mm"
    )
    print(
        f"  Loven nettoh min                {sec['rafter_notch_h_min_mm']:.1f} mm,  lovisyvyys @ palkki {sec['rafter_notch_depth_at_beam_mm']:.1f} mm,"
        f"  nyrkkisääntö h/3: {'OK ✓' if sec['rafter_notch_depth_rule_ok'] else 'YLITTYY ✗'}"
    )
    print()
    print(f"  {'ID':<10} {'x [mm]':>7} {'b_trib':>7} {'qd':>7} {'R_seinä':>8} {'R_palkki':>9} {'Md':>7} {'η_M':>7} {'η_V':>7} {'η_lovi':>8} {'δ_sls':>9}")
    print(f"  {'-'*10} {'-'*7} {'-'*7} {'-'*7} {'-'*8} {'-'*9} {'-'*7} {'-'*7} {'-'*7} {'-'*8} {'-'*9}")
    for rid in data["down_rafter_results"]:
        r = data["down_rafter_results"][rid]
        r_sls = data["sls_rafter_results"][rid]
        print(
            f"  {rid:<10} {r['x_mm']:>7.0f} {r['trib_w_m']*1000:>7.0f} {r['q_proj_kNm']:>7.3f}"
            f" {r['R_wall_kN']:>8.2f} {r['R_beam_kN']:>9.2f} {r['M_gov']['value_kNm']:>7.2f}"
            f" {r['eta_M']:>6.1f}% {r['eta_V']:>6.1f}% {r['eta_notch']:>7.1f}% {abs(r_sls['delta']['value_mm']):>5.1f}/{sec['rafter_delta_limit_mm']:.1f}"
        )
    if data["down_case_rafter"] == "ULS MAINT":
        print("  Huom. qd-sarake kuvaa hajakuormaa; ULS MAINT sisältää lisäksi 1.5 kN pistekuorman yllä ilmoitettuun huoltopisteeseen.")

    crit = data["critical_rafter"]
    print()
    print(f"  Kriittinen kattotuoli           {crit['id']}  (b_trib = {crit['trib_w_m']*1000:.0f} mm)")
    print(f"    Md,max                        {crit['M_gov']['value_kNm']:.2f} kNm ({crit['M_gov']['sign']}) @ y={crit['M_gov']['x_mm']:.0f} mm")
    print(f"    Sisäinen M tuella seinä/LP225 {crit['M_wall_kNm']:.2f} / {crit['M_beam_kNm']:.2f} kNm")
    print(f"    Negatiivinen momentti tuella  {crit['M_neg']['value_kNm']:.2f} kNm @ y={crit['M_neg']['x_mm']:.0f} mm")
    print(f"    Vd,max                        {abs(crit['V_abs']['value_kN']):.2f} kN @ y={crit['V_abs']['x_mm']:.0f} mm")
    print(
        f"    Käyttöaste                    η_M = {crit['eta_M']:.1f}% ({ok_mark(crit['eta_M'])}),"
        f" η_V = {crit['eta_V']:.1f}%, η_lovi = {crit['eta_notch']:.1f}%"
    )
    print(
        f"    Lovivyöhykkeen max            {governing_label(crit['notch']['eta_gov']['mode'])} = {crit['notch']['eta_gov']['value_pct']:.1f}%"
        f" @ y={crit['notch']['eta_gov']['x_mm']:.0f} mm, h_net = {crit['notch']['eta_gov']['h_mm']:.1f} mm"
    )
    crit_sls = data["sls_rafter_results"][crit["id"]]
    print(f"    Taipuma {data['sls_case_rafter']:<20} {abs(crit_sls['delta']['value_mm']):.2f} / {sec['rafter_delta_limit_mm']:.2f} mm  {ok_mark(abs(crit_sls['delta']['value_mm']) / sec['rafter_delta_limit_mm'] * 100.0)}")

    beam = data["beam_governing"]
    print("\n── LP225×90 PALKKI, GL30c (jatkuva 4 tukea) ─────────────────")
    print(f"  MRd = {sec['beam_MRd']:.2f} kNm,  VRd = {sec['beam_VRd']:.2f} kN,  δ_lim = {sec['beam_delta_limit_mm']:.1f} mm")
    print(f"  fm,d = {sec['fm_d_GL30c']:.2f} N/mm²,  fv,d = {sec['fv_d_GL30c']:.2f} N/mm²")
    print("  Pistekuormat kattotuoleilta     " + ", ".join(f"{p_kN:.2f} kN @ x={x_mm:.0f}" for x_mm, p_kN in cases[data["down_case_beam"]]["beam_point_loads_kN"]))
    print(f"  Md,max                          {beam['M_gov']['value_kNm']:.2f} kNm ({beam['M_gov']['sign']}) @ x={beam['M_gov']['x_mm']:.0f} mm")
    print(f"  M+ max                          {beam['M_pos']['value_kNm']:.2f} kNm @ x={beam['M_pos']['x_mm']:.0f} mm")
    print(f"  M− max                          {beam['M_neg']['value_kNm']:.2f} kNm @ x={beam['M_neg']['x_mm']:.0f} mm")
    print(f"  Vd,max                          {abs(beam['V_abs']['value_kN']):.2f} kN @ x={beam['V_abs']['x_mm']:.0f} mm")
    print(f"  Käyttöaste                      η_M = {beam['eta_M']:.1f}%  ({ok_mark(beam['eta_M'])}), η_V = {beam['eta_V']:.1f}%")
    beam_sls = cases[data["sls_case_beam"]]["beam"]
    print(f"  Taipuma {data['sls_case_beam']:<20} {abs(beam_sls['delta']['value_mm']):.3f} / {sec['beam_delta_limit_mm']:.2f} mm  {ok_mark(abs(beam_sls['delta']['value_mm']) / sec['beam_delta_limit_mm'] * 100.0)}")

    print("\n── SEINÄ- JA PILARIKUORMAT ──────────────────────────────────")
    print(f"  Seinälinjan kokonaisreaktio ULS {cases[data['down_case_rafter']]['wall_total_kN']:.2f} kN")
    print(f"  Seinälinjan nosto ULS           {cases['UPLIFT']['wall_total_kN']:.2f} kN")
    print(f"  Max seinäreaktio / kattotuoli   {max(r['R_wall_kN'] for r in cases[data['down_case_rafter']]['rafters']):.2f} kN  ({data['down_case_rafter']})")
    print(f"  Max seinän nostoreaktio         {min(r['R_wall_kN'] for r in cases['UPLIFT']['rafters']):.2f} kN  (UPLIFT)")
    print()
    print(f"  {'Tuki':<28} {data['down_case_beam']:>10} {'UPLIFT':>10}")
    print(f"  {'-'*28} {'-'*10} {'-'*10}")
    uplift_reactions = cases["UPLIFT"]["beam"]["reactions_kN"]
    for col_id, x_mm, _ in geo["beam_supports"]:
        down_val = cases[data["down_case_beam"]]["beam"]["reactions_kN"][x_mm]
        uplift_val = uplift_reactions[x_mm]
        tag = "  ← max alas" if x_mm == data["max_column_down"][0] else ("  ← max nosto" if x_mm == data["max_column_uplift"][0] else "")
        display = f"{col_id} @ x={x_mm:.0f}"
        print(f"  {display:<28} {down_val:>9.2f} {uplift_val:>10.2f}{tag}")

    print("\n── PORTAIKON BETONIRAKENTEET JA PERUSTUSKUORMAT ─────────────")
    print(f"  Ontelolaatat                   gk = {foundation['gk_hollow_slab']:.2f} kN/m², qk = {foundation['qk_floor_live']:.2f} kN/m²")
    print("  Kuormareitti                   ontelolaatat / porraspalkki → vaakapalkki → betonipilarit")
    print("  P15-22 sivutuki                mallinnettu ref.house_wall_corner-liitoksena; ei hyvitetä 1D-perustuskuormissa")
    floor_uls = foundation["floor_results"]["ULS"]
    print()
    print(f"  {'Jäsen':<24} {'b/q':>10} {'R_seinä':>9} {'R_vaaka':>9} {'Md':>8} {'Vd':>8}")
    print(f"  {'-'*24} {'-'*10} {'-'*9} {'-'*9} {'-'*8} {'-'*8}")
    for slab in floor_uls["slabs"]:
        moment = max(abs(slab["M_pos"]["value_kNm"]), abs(slab["M_neg"]["value_kNm"]))
        print(
            f"  {slab['id']:<24} {slab['width_m']*1000:>6.0f} mm {slab['wall_reaction_kN']:>9.2f}"
            f" {slab['beam_reaction_kN']:>9.2f} {moment:>8.2f} {abs(slab['V_abs']['value_kN']):>8.2f}"
        )
    stair = floor_uls["stair_beam"]
    stair_moment = max(abs(stair["M_pos"]["value_kNm"]), abs(stair["M_neg"]["value_kNm"]))
    print(
        f"  {'beam.stairs':<24} {stair['q_line_kNm']:>6.3f} kN/m {stair['wall_reaction_kN']:>9.2f}"
        f" {stair['beam_reaction_kN']:>9.2f} {stair_moment:>8.2f} {abs(stair['V_abs']['value_kN']):>8.2f}"
    )
    print()
    print(f"  {'Vaakapalkin tuki':<24} {'SLS':>9} {'ULS':>9} {'UPLIFT':>9}")
    print(f"  {'-'*24} {'-'*9} {'-'*9} {'-'*9}")
    for col_id, _x_mm in foundation["concrete_beam_supports"]:
        print(
            f"  {col_id:<24}"
            f" {foundation['floor_results']['SLS']['concrete_beam']['reactions_by_column_id_kN'][col_id]:>9.2f}"
            f" {foundation['floor_results']['ULS']['concrete_beam']['reactions_by_column_id_kN'][col_id]:>9.2f}"
            f" {foundation['floor_results']['UPLIFT']['concrete_beam']['reactions_by_column_id_kN'][col_id]:>9.2f}"
        )

    print("\n── PERUSTUKSET ───────────────────────────────────────────────")
    print("  col.x7075                      sisältää katos.json-baselinen + portaikon lisäkuormat")
    print("  col.x9100                      sisältää vaakapalkin, LP225 col.lp1 -tuen ja pilarin omapainon")
    for line in foundation_report_lines(foundation["checks"]):
        print(line)

    print("\n── YHTEENVETO ────────────────────────────────────────────────")
    print(f"  Hallitseva rakenneosa           {crit['id']} → {data['down_case_rafter']} / {governing_label(crit['gov_mode'])} = {crit['eta_gov']:.1f}%")
    print(f"  LP225×90 suurin käyttöaste      {data['down_case_beam']} / {beam['eta_M']:.1f}% (momentti), {beam['eta_V']:.1f}% (leikkaus)")
    print(f"  Suurin pilaripuristus           {data['max_column_down'][1]:.2f} kN @ x={data['max_column_down'][0]:.0f} mm")
    print(f"  Suurin pilarinostotarve         {abs(data['max_column_uplift'][1]):.2f} kN @ x={data['max_column_uplift'][0]:.0f} mm")
    print(f"  Suurin perustuspaine            {max(row['q_uls_kPa'] for row in foundation['checks']):.0f} kPa")
    print(f"  Tulos                           {'Kattotuoli on lähes täynnä mutta OK ✓' if crit['eta_gov'] <= 100.0 else 'Kattotuoli ylittää kapasiteetin ✗'}")

    print("\n  HUOMIOT:")
    print("  * Kuormat on laskettu vaakaprojektiona; kaltevuus on kattotuolin jännesuunnassa,")
    print("    joten taivutusmomenttiin ei käytetä 1/cos-korjausta.")
    print("  * Tuuli mallinnetaan rakennukseen kiinnittyvänä avoimena pulpettikatoksena;")
    print("    vapaasti seisovan katoksen taulukkoa 7.7 ei käytetä tässä tiedostossa.")
    print(f"  * Kinostuma lasketaan rafterikohtaisesti. Suurin este on {drift['critical_source']} pisteessä")
    print(
        f"    x = {drift['reference_x_mm']:.0f} mm, y = {drift['reference_y_mm']:.0f} mm; tämä antaa"
        f" {drift['critical_rafter_id']}:lle h = {drift['h_mm']/1000.0:.2f} m ja s_kin,max = {drift['s_peak_kNm2']:.2f} kN/m²."
    )
    print("  * Kattotuolit laajennetaan geometriasta sekä erillisistä että pattern-jäsenistä,")
    print(f"    joten suurin tributäärileveys on {geo['widest_tributary_m']*1000:.0f} mm kattotuolilla {geo['widest_rafter_id']}.")
    print("  * Kattotuolien seinäpää mallinnetaan piirustusten perusteella kiertymäjäykkänä,")
    print("    mutta LP225-pää nivellettyenä; tukimomentti muodostuu siis vain seinäpäähän.")
    print(f"  * LP225-tuella oleva bevel_notch ({sec['rafter_notch_side']}) huomioidaan geometriasta lineaarisesti")
    print("    muuttuvana nettoh-/nettoW-/nettoA- ja EI-tarkistuksena koko loven pituudella.")
    print("  * Kaiteet/sivulasit jätetään pois tästä versiosta, kunnes niiden oikea")
    print("    geometria ja metallikaiteet on mallinnettu.")
    print(f"  * Kiinnitystarvikkeet, pilarien nurjahdus sekä katteen {geo['roof_sheet_extra_mm']:.0f} mm lisäuloke")
    print("    rafter-tipin yli eivät sisälly tähän mitoitukseen.")
    print(dw)


if __name__ == "__main__":
    main()
