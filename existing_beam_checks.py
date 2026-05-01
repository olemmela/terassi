import math
from functools import lru_cache

from geometry_loader import expanded_connections, expanded_members, load, member, surface, profile_b, profile_h


def aggregate_point_loads(point_loads):
    aggregated = {}
    for x_pos, p_kN in point_loads or []:
        key = round(float(x_pos), 9)
        aggregated[key] = aggregated.get(key, 0.0) + float(p_kN)
    return sorted(aggregated.items())


def format_point_loads(point_loads):
    point_loads = aggregate_point_loads(point_loads)
    if not point_loads:
        return "ei pistekuormia"
    use_mm = any(abs(x_pos) > 20.0 for x_pos, _ in point_loads)
    return ", ".join(
        f"{p_kN:.1f} kN @ x={(x_pos if use_mm else x_pos * 1000.0):.0f} mm"
        for x_pos, p_kN in point_loads
    )


def simple_span_point_reactions(span_m, point_loads_local_m):
    point_loads_local_m = aggregate_point_loads(point_loads_local_m)
    r_left = sum(p_kN * (span_m - x_m) / span_m for x_m, p_kN in point_loads_local_m)
    r_right = sum(p_kN * x_m / span_m for x_m, p_kN in point_loads_local_m)
    return r_left, r_right


def simple_span_combined_moment_max(span_m, udl_kNm, point_loads_local_m):
    point_loads_local_m = aggregate_point_loads(point_loads_local_m)
    point_load_map = dict(point_loads_local_m)
    r_left_points, _ = simple_span_point_reactions(span_m, point_loads_local_m)

    def moment_at(x_m):
        moment_udl = udl_kNm * x_m * (span_m - x_m) / 2.0
        moment_points = r_left_points * x_m
        for a_m, p_kN in point_loads_local_m:
            if a_m <= x_m + 1e-12:
                moment_points -= p_kN * (x_m - a_m)
        return moment_udl + moment_points

    candidates = {0.0, span_m}
    segment_points = [0.0] + [x_m for x_m, _ in point_loads_local_m] + [span_m]
    running_point_load = 0.0
    for x0_m, x1_m in zip(segment_points, segment_points[1:]):
        candidates.add(x0_m)
        candidates.add(x1_m)
        if abs(udl_kNm) > 1e-12:
            x_zero_m = (udl_kNm * span_m / 2.0 + r_left_points - running_point_load) / udl_kNm
            if x0_m - 1e-12 <= x_zero_m <= x1_m + 1e-12:
                candidates.add(max(x0_m, min(x1_m, x_zero_m)))
        if x1_m in point_load_map:
            running_point_load += point_load_map[x1_m]

    moment_candidates = [(x_m, moment_at(x_m)) for x_m in sorted(candidates)]
    return max(moment_candidates, key=lambda item: abs(item[1]))


def simple_span_max_shear(span_m, udl_kNm=0.0, point_loads_local_m=None):
    point_loads_local_m = aggregate_point_loads(point_loads_local_m)
    reaction_left = udl_kNm * span_m / 2.0
    reaction_right = reaction_left
    add_left, add_right = simple_span_point_reactions(span_m, point_loads_local_m)
    reaction_left += add_left
    reaction_right += add_right

    max_abs_shear_kN = reaction_left
    x_at_max_m = 0.0
    current_shear_kN = reaction_left
    previous_x_m = 0.0

    for load_x_m, p_kN in point_loads_local_m:
        shear_before_kN = current_shear_kN - udl_kNm * (load_x_m - previous_x_m)
        if abs(shear_before_kN) > abs(max_abs_shear_kN):
            max_abs_shear_kN = shear_before_kN
            x_at_max_m = load_x_m
        current_shear_kN = shear_before_kN - p_kN
        if abs(current_shear_kN) > abs(max_abs_shear_kN):
            max_abs_shear_kN = current_shear_kN
            x_at_max_m = load_x_m
        previous_x_m = load_x_m

    shear_before_right_support_kN = current_shear_kN - udl_kNm * (span_m - previous_x_m)
    if abs(shear_before_right_support_kN) > abs(max_abs_shear_kN):
        max_abs_shear_kN = shear_before_right_support_kN
        x_at_max_m = span_m
    if abs(-reaction_right) > abs(max_abs_shear_kN):
        max_abs_shear_kN = -reaction_right
        x_at_max_m = span_m
    return max_abs_shear_kN, x_at_max_m


def simple_span_max_deflection_mm(span_mm, EI_Nmm2, udl_kNm=0.0, point_loads_local_m=None, sample_count=2001):
    point_loads_local_m = aggregate_point_loads(point_loads_local_m)
    q_Nmm = udl_kNm
    point_loads_mm = [(x_m * 1000.0, p_kN * 1000.0) for x_m, p_kN in point_loads_local_m]
    max_delta_mm = 0.0
    x_at_max_mm = 0.0
    for i in range(sample_count):
        x_mm = span_mm * i / (sample_count - 1)
        delta_mm = q_Nmm * x_mm * (span_mm**3 - 2.0 * span_mm * x_mm**2 + x_mm**3) / (24.0 * EI_Nmm2)
        for a_mm, p_N in point_loads_mm:
            b_mm = span_mm - a_mm
            if x_mm <= a_mm + 1e-9:
                delta_mm += p_N * b_mm * x_mm * (span_mm**2 - b_mm**2 - x_mm**2) / (6.0 * span_mm * EI_Nmm2)
            else:
                delta_mm += p_N * a_mm * (span_mm - x_mm) * (
                    span_mm**2 - a_mm**2 - (span_mm - x_mm)**2
                ) / (6.0 * span_mm * EI_Nmm2)
        if abs(delta_mm) > abs(max_delta_mm):
            max_delta_mm = delta_mm
            x_at_max_mm = x_mm
    return max_delta_mm, x_at_max_mm


def point_beam_with_right_overhang_response(span_mm, beam_end_mm, point_loads_abs_mm):
    point_loads_abs_mm = aggregate_point_loads(point_loads_abs_mm)
    if beam_end_mm < span_mm - 1e-9:
        raise ValueError("Beam end cannot be before right support.")

    total_load_kN = sum(p_kN for _, p_kN in point_loads_abs_mm)
    reaction_right_kN = sum(p_kN * x_mm for x_mm, p_kN in point_loads_abs_mm) / span_mm
    reaction_left_kN = total_load_kN - reaction_right_kN

    def moment_at(x_mm):
        moment_kNmm = reaction_left_kN * x_mm
        if x_mm >= span_mm - 1e-9:
            moment_kNmm += reaction_right_kN * (x_mm - span_mm)
        for load_x_mm, p_kN in point_loads_abs_mm:
            if load_x_mm <= x_mm + 1e-9:
                moment_kNmm -= p_kN * (x_mm - load_x_mm)
        return moment_kNmm / 1000.0

    def shear_after_events(x_mm):
        shear_kN = reaction_left_kN
        if x_mm >= span_mm - 1e-9:
            shear_kN += reaction_right_kN
        for load_x_mm, p_kN in point_loads_abs_mm:
            if load_x_mm <= x_mm + 1e-9:
                shear_kN -= p_kN
        return shear_kN

    candidate_positions_mm = sorted({0.0, span_mm, beam_end_mm, *[x_mm for x_mm, _ in point_loads_abs_mm]})
    moment_candidates = [(x_mm, moment_at(x_mm)) for x_mm in candidate_positions_mm]
    moment_x_mm, moment_kNm = max(moment_candidates, key=lambda item: abs(item[1]))

    max_abs_shear_kN = reaction_left_kN
    shear_x_mm = 0.0
    if abs(reaction_right_kN) > abs(max_abs_shear_kN):
        max_abs_shear_kN = reaction_right_kN
        shear_x_mm = span_mm
    for x_mm in candidate_positions_mm:
        shear_kN = shear_after_events(x_mm)
        if abs(shear_kN) > abs(max_abs_shear_kN):
            max_abs_shear_kN = shear_kN
            shear_x_mm = x_mm

    return {
        "reactions_kN": {
            0.0: reaction_left_kN,
            float(span_mm): reaction_right_kN,
        },
        "M_gov": {"value_kNm": abs(moment_kNm), "x_mm": moment_x_mm, "raw_value_kNm": moment_kNm},
        "V_abs": {"value_kN": max_abs_shear_kN, "x_mm": shear_x_mm},
        "point_loads_abs_mm": point_loads_abs_mm,
    }


def symmetric_support_reaction(span_line_load_kNm, span_m, beam_overhang_m, roof_area_load_kNm2, tributary_width_m, roof_eave_m):
    return (
        span_line_load_kNm * (span_m / 2.0 + beam_overhang_m)
        + roof_area_load_kNm2 * tributary_width_m * roof_eave_m
    )


def tributary_ranges_mm(positions_mm, edge_start_mm, edge_end_mm):
    ranges = []
    for i, pos_mm in enumerate(positions_mm):
        left_mm = edge_start_mm if i == 0 else 0.5 * (positions_mm[i - 1] + pos_mm)
        right_mm = edge_end_mm if i == len(positions_mm) - 1 else 0.5 * (pos_mm + positions_mm[i + 1])
        ranges.append((left_mm, right_mm))
    return ranges


def uniform_line_member_support_reactions(member_start_mm, member_end_mm, support_left_mm, support_right_mm, line_load_kNm):
    member_start_mm = float(member_start_mm)
    member_end_mm = float(member_end_mm)
    support_left_mm = float(support_left_mm)
    support_right_mm = float(support_right_mm)
    load_start_mm = min(member_start_mm, member_end_mm)
    load_end_mm = max(member_start_mm, member_end_mm)
    if abs(support_right_mm - support_left_mm) <= 1e-9:
        raise ValueError("Support points for uniform member load cannot coincide.")
    if support_left_mm > support_right_mm:
        reaction_right_kN, reaction_left_kN = uniform_line_member_support_reactions(
            member_start_mm,
            member_end_mm,
            support_right_mm,
            support_left_mm,
            line_load_kNm,
        )
        return reaction_left_kN, reaction_right_kN

    total_load_kN = float(line_load_kNm) * (load_end_mm - load_start_mm) / 1000.0
    resultant_x_mm = 0.5 * (load_start_mm + load_end_mm)
    support_span_mm = support_right_mm - support_left_mm
    reaction_right_kN = total_load_kN * (resultant_x_mm - support_left_mm) / support_span_mm
    reaction_left_kN = total_load_kN - reaction_right_kN
    return reaction_left_kN, reaction_right_kN


def _main_katos_purlins(geo):
    purlins = []
    for member_obj in expanded_members(geo, "purlins"):
        parts = member_obj["id"].split(".")
        if len(parts) == 2 and parts[0] == "purlin" and parts[1].isdigit():
            purlins.append(member_obj)
    return sorted(purlins, key=lambda item: (float(item["axis_start"]["x"]), item["id"]))


def _diag_katos_purlins(geo):
    return sorted(
        [member_obj for member_obj in expanded_members(geo, "purlins") if member_obj["id"].startswith("diag.")],
        key=lambda item: (float(item["axis_start"]["x"]), item["id"]),
    )


def _kp450_side_katos_purlins(geo):
    return sorted(
        [member_obj for member_obj in expanded_members(geo, "purlins") if member_obj["id"].startswith("purlin.rayst.")],
        key=lambda item: (float(item["axis_start"]["y"]), float(item["axis_start"]["x"]), item["id"]),
    )


def roof_continuation_wind_model(qp_z_kNm2):
    """Wind envelope for a roof plane that continues from the main building roof."""
    # Sign convention: positive cp,net acts downward on the roof members.
    # The uplift envelope combines roof-edge suction on the top surface with
    # windward wall pressure acting on the underside of the eaves/overhang.
    # The downward envelope combines weak top pressure with wall-side suction
    # below the overhang; final project-specific cpe zones still need review.
    cpe_top_down = 0.2
    cpe_under_suction = -0.5
    cpe_top_uplift = -1.2
    cpe_under_windward = 0.8
    cp_net_down = cpe_top_down - cpe_under_suction
    cp_net_up = cpe_top_uplift - cpe_under_windward
    return {
        "model": "building_roof_overhang_continuation",
        "description": "Rakennuksen kattolappeen jatke / räystäsuloke, ei vapaasti seisova katos",
        "basis": "EN 1991-1-4 rakennuksen kattopinta + räystään alapinnan viereisen seinän paine",
        "cpe_top_down": cpe_top_down,
        "cpe_under_down": cpe_under_suction,
        "cp_net_down": cp_net_down,
        "cpe_top_uplift": cpe_top_uplift,
        "cpe_under_uplift": cpe_under_windward,
        "cp_net_up": cp_net_up,
        "w_wind_down_kNm2": cp_net_down * qp_z_kNm2,
        "w_wind_up_kNm2": cp_net_up * qp_z_kNm2,
    }


def _connection_by_members(connections, member_a_id, member_b_id):
    wanted = {member_a_id, member_b_id}
    for connection_obj in connections:
        if set(connection_obj.get("members", [])) == wanted:
            return connection_obj
    raise KeyError(f"Connection not found for members: {member_a_id}, {member_b_id}")


def member_axis_length_mm(member_obj):
    start = member_obj["axis_start"]
    end = member_obj["axis_end"]
    dx_mm = float(end["x"]) - float(start["x"])
    dy_mm = float(end["y"]) - float(start["y"])
    dz_mm = float(end["z"]) - float(start["z"])
    return math.sqrt(dx_mm**2 + dy_mm**2 + dz_mm**2)


def project_point_to_member_s_mm(member_obj, point):
    start = member_obj["axis_start"]
    end = member_obj["axis_end"]
    dx_mm = float(end["x"]) - float(start["x"])
    dy_mm = float(end["y"]) - float(start["y"])
    dz_mm = float(end["z"]) - float(start["z"])
    length_mm = math.sqrt(dx_mm**2 + dy_mm**2 + dz_mm**2)
    if length_mm <= 1e-9:
        return 0.0
    vx_mm = float(point["x"]) - float(start["x"])
    vy_mm = float(point["y"]) - float(start["y"])
    vz_mm = float(point["z"]) - float(start["z"])
    s_mm = (vx_mm * dx_mm + vy_mm * dy_mm + vz_mm * dz_mm) / length_mm
    return max(0.0, min(length_mm, s_mm))


@lru_cache(maxsize=1)
def katos_existing_context():
    geo = load("katos.json")
    connections = expanded_connections(geo)
    wall_poly = geo["reference_surfaces"][0]["polygon"]
    wall_width_mm = int(max(p["x"] for p in wall_poly) - min(p["x"] for p in wall_poly))
    pillar_size_mm = int(member(geo, "columns", "col.x125")["profile"]["b_mm"])
    roof = surface(geo, "surf.roof")
    roof_poly = roof["polygon"]
    roof_x0_mm = min(p["x"] for p in roof_poly)
    roof_x1_mm = max(p["x"] for p in roof_poly)
    roof_xspan_mm = max(p["x"] for p in roof_poly) - min(p["x"] for p in roof_poly)
    roof_zspan_mm = max(p["z"] for p in roof_poly) - min(p["z"] for p in roof_poly)
    slope_deg = math.degrees(math.atan(roof_zspan_mm / roof_xspan_mm))
    slope_rad = math.radians(slope_deg)
    roof_edge_y_mm = int(max(p["y"] for p in roof_poly))
    beam1_y_mm = int(member(geo, "beams", "beam.kp450.y900")["axis_start"]["y"])
    beam2_y_mm = int(member(geo, "beams", "beam.kp360x2")["axis_start"]["y"])
    col_xs_mm = [member(geo, "columns", cid)["base"]["x"] for cid in ("col.x125", "col.x7075")]
    support_left_x_mm = float(min(col_xs_mm))
    support_right_x_mm = float(max(col_xs_mm))
    span_mm = support_right_x_mm - support_left_x_mm
    span_m = span_mm / 1000.0

    kp450_mid = member(geo, "beams", "beam.kp450.y900")
    kp450_x0_mm = float(kp450_mid["axis_start"]["x"])
    kp450_x1_mm = float(kp450_mid["axis_end"]["x"])
    beam_overhang_m = (support_left_x_mm - kp450_x0_mm) / 1000.0
    roof_eave_m = (kp450_x0_mm - min(p["x"] for p in roof_poly)) / 1000.0

    kp450_wall = member(geo, "beams", "beam.kp450.wall")
    kp360 = member(geo, "beams", "beam.kp360x2")
    lp_x125 = member(geo, "beams", "beam.lp225.x125")
    main_purlins = _main_katos_purlins(geo)
    diag_purlins = _diag_katos_purlins(geo)
    kp450_side_purlins = _kp450_side_katos_purlins(geo)
    if not main_purlins:
        raise ValueError("Main katos purlins are missing from geometry.")
    loadbearing_purlins = [*main_purlins, *diag_purlins]

    b1_mm = profile_b(kp450_wall)
    h1_mm = profile_h(kp450_wall)
    b2_mm = profile_b(kp360)
    h2_mm = profile_h(kp360)
    b_lp_mm = profile_b(lp_x125)
    h_lp_mm = profile_h(lp_x125)
    purlin_b_mm = profile_b(main_purlins[0])
    purlin_h_mm = profile_h(main_purlins[0])

    trib1_start_mm = beam1_y_mm / 2.0
    purlin_rows = []
    for purlin_obj in loadbearing_purlins:
        inner_conn = _connection_by_members(connections, purlin_obj["id"], "beam.kp450.y900")
        outer_conn = _connection_by_members(connections, purlin_obj["id"], "beam.kp360x2")
        inner_point = dict(inner_conn["at"])
        outer_point = dict(outer_conn["at"])
        purlin_rows.append({
            "id": purlin_obj["id"],
            "kind": "diag" if purlin_obj["id"].startswith("diag.") else "main",
            "inner_connection_id": inner_conn["id"],
            "outer_connection_id": outer_conn["id"],
            "support_inner_x_mm": float(inner_point["x"]),
            "support_outer_x_mm": float(outer_point["x"]),
            "support_inner_y_mm": float(inner_point["y"]),
            "support_outer_y_mm": float(outer_point["y"]),
            "reaction_inner_x_mm": max(support_left_x_mm, min(support_right_x_mm, float(inner_point["x"]))),
            "reaction_outer_x_mm": max(support_left_x_mm, min(support_right_x_mm, float(outer_point["x"]))),
            "member_length_mm": member_axis_length_mm(purlin_obj),
            "support_inner_s_mm": project_point_to_member_s_mm(purlin_obj, inner_point),
            "support_outer_s_mm": project_point_to_member_s_mm(purlin_obj, outer_point),
        })
    purlin_rows.sort(key=lambda item: (item["support_inner_x_mm"], item["id"]))
    purlin_xs_mm = [row["support_inner_x_mm"] for row in purlin_rows]

    kp450_side_rows = []
    for purlin_obj in kp450_side_purlins:
        beam_id = "beam.kp450.wall" if purlin_obj["id"].endswith(".0") else "beam.kp450.y900"
        _connection_by_members(connections, purlin_obj["id"], beam_id)
        kp450_side_rows.append({
            "id": purlin_obj["id"],
            "beam_id": beam_id,
            "member_length_mm": member_axis_length_mm(purlin_obj),
        })

    purlin_inner_support_y_mm = purlin_rows[0]["support_inner_y_mm"]
    purlin_outer_support_y_mm = purlin_rows[0]["support_outer_y_mm"]
    for row in purlin_rows[1:]:
        if abs(row["support_inner_y_mm"] - purlin_inner_support_y_mm) > 1e-6:
            raise ValueError("Load-bearing purlin inner support y-coordinates must be consistent.")
        if abs(row["support_outer_y_mm"] - purlin_outer_support_y_mm) > 1e-6:
            raise ValueError("Load-bearing purlin outer support y-coordinates must be consistent.")

    trib1_end_mm = purlin_inner_support_y_mm
    trib2_start_mm = purlin_outer_support_y_mm
    trib2_end_mm = float(roof_edge_y_mm)
    trib_w1_m = max(0.0, (trib1_end_mm - trib1_start_mm) / 1000.0)
    trib_w2_m = 0.0
    purlin_trib_ranges_mm = tributary_ranges_mm(purlin_xs_mm, roof_x0_mm, roof_x1_mm)
    purlin_loaded_depth_m = max(0.0, (float(roof_edge_y_mm) - purlin_inner_support_y_mm) / 1000.0)

    gk_roofing = 0.20
    gamma_lvl = 480.0 * 9.81 / 1000.0
    gamma_c24 = 420.0 * 9.81 / 1000.0
    g_beam1_kNm = b1_mm / 1000.0 * h1_mm / 1000.0 * gamma_lvl
    g_beam2_kNm = b2_mm / 1000.0 * h2_mm / 1000.0 * gamma_lvl
    purlin_self_kNm = purlin_b_mm / 1000.0 * purlin_h_mm / 1000.0 * gamma_c24
    gk1_direct_kNm = gk_roofing * trib_w1_m + g_beam1_kNm
    gk2_direct_kNm = g_beam2_kNm

    sk = 2.0
    mu1 = 0.8
    s_roof = mu1 * sk
    qk_snow1_direct_kNm = s_roof * trib_w1_m
    qk_snow2_direct_kNm = 0.0

    vb0 = 21.0
    rho_air = 1.25
    z0 = 0.05
    z_min = 2.0
    all_z_mm = [
        p["z"] for s_obj in geo.get("surfaces", []) for p in s_obj.get("polygon", []) if "z" in p
    ] + [
        m[k]["z"]
        for grp in geo["members"].values() for m in grp
        for k in ("axis_start", "axis_end", "base", "top")
        if k in m and isinstance(m[k], dict) and "z" in m[k]
    ]
    z_ref_m = math.ceil(max(all_z_mm) / 500.0) * 0.5
    kr = 0.19 * (z0 / 0.05) ** 0.07
    cr_z = kr * math.log(max(z_ref_m, z_min) / z0)
    iv_z = 1.0 / math.log(max(z_ref_m, z_min) / z0)
    vm_z = cr_z * vb0
    qp_z = (1.0 + 7.0 * iv_z) * 0.5 * rho_air * vm_z**2 / 1000.0
    wind_model = roof_continuation_wind_model(qp_z)
    cp_net_down = wind_model["cp_net_down"]
    cp_net_up = wind_model["cp_net_up"]
    w_wind_down = wind_model["w_wind_down_kNm2"]
    w_wind_up = wind_model["w_wind_up_kNm2"]
    qk_wind_down1_direct_kNm = w_wind_down * trib_w1_m
    qk_wind_down2_direct_kNm = 0.0
    qk_wind_up1_direct_kNm = w_wind_up * trib_w1_m
    qk_wind_up2_direct_kNm = 0.0

    gammaG = 1.35
    gammaQ = 1.50
    psi0_W = 0.6
    qd1_direct_kNm = gammaG * gk1_direct_kNm + gammaQ * qk_snow1_direct_kNm + gammaQ * psi0_W * qk_wind_down1_direct_kNm
    qd2_direct_kNm = gammaG * gk2_direct_kNm + gammaQ * qk_snow2_direct_kNm + gammaQ * psi0_W * qk_wind_down2_direct_kNm
    qk_sls1_direct_kNm = gk1_direct_kNm + qk_snow1_direct_kNm
    qk_sls2_direct_kNm = gk2_direct_kNm + qk_snow2_direct_kNm
    qmin1_direct_kNm = 0.9 * gk1_direct_kNm + 1.5 * qk_wind_up1_direct_kNm
    qmin2_direct_kNm = 0.9 * gk2_direct_kNm + 1.5 * qk_wind_up2_direct_kNm

    q_roof_d_kNm2 = gammaG * gk_roofing + gammaQ * s_roof + gammaQ * psi0_W * w_wind_down
    q_roof_sls_kNm2 = gk_roofing + s_roof
    q_roof_min_kNm2 = 0.9 * gk_roofing + 1.5 * w_wind_up

    def purlin_point_loads(area_load_kNm2, self_factor):
        beam1_point_loads_abs_x_mm = []
        beam2_point_loads_abs_x_mm = []
        rows = []
        for row, (trib_start_x_mm, trib_end_x_mm) in zip(purlin_rows, purlin_trib_ranges_mm):
            tributary_width_m = (trib_end_x_mm - trib_start_x_mm) / 1000.0
            tributary_area_m2 = tributary_width_m * purlin_loaded_depth_m
            member_length_m = row["member_length_mm"] / 1000.0
            area_load_factor_m = tributary_area_m2 / member_length_m
            line_load_kNm = area_load_kNm2 * area_load_factor_m + self_factor * purlin_self_kNm
            reaction_inner_kN, reaction_outer_kN = uniform_line_member_support_reactions(
                0.0,
                row["member_length_mm"],
                row["support_inner_s_mm"],
                row["support_outer_s_mm"],
                line_load_kNm,
            )
            beam1_point_loads_abs_x_mm.append((row["reaction_inner_x_mm"], reaction_inner_kN))
            beam2_point_loads_abs_x_mm.append((row["reaction_outer_x_mm"], reaction_outer_kN))
            rows.append({
                **row,
                "tributary_x_start_mm": trib_start_x_mm,
                "tributary_x_end_mm": trib_end_x_mm,
                "tributary_width_m": tributary_width_m,
                "tributary_area_m2": tributary_area_m2,
                "area_load_factor_m": area_load_factor_m,
                "member_length_m": member_length_m,
                "line_load_kNm": line_load_kNm,
                "reaction_inner_kN": reaction_inner_kN,
                "reaction_outer_kN": reaction_outer_kN,
            })
        return (
            aggregate_point_loads(beam1_point_loads_abs_x_mm),
            aggregate_point_loads(beam2_point_loads_abs_x_mm),
            rows,
        )

    beam1_purlin_uls_abs_x_mm, beam2_purlin_uls_abs_x_mm, purlin_rows_uls = purlin_point_loads(q_roof_d_kNm2, gammaG)
    beam1_purlin_sls_abs_x_mm, beam2_purlin_sls_abs_x_mm, purlin_rows_sls = purlin_point_loads(q_roof_sls_kNm2, 1.0)
    beam1_purlin_uplift_abs_x_mm, beam2_purlin_uplift_abs_x_mm, purlin_rows_uplift = purlin_point_loads(q_roof_min_kNm2, 0.9)

    beam1_direct_reaction_uls_kN = symmetric_support_reaction(qd1_direct_kNm, span_m, beam_overhang_m, q_roof_d_kNm2, trib_w1_m, roof_eave_m)
    beam2_direct_reaction_uls_kN = symmetric_support_reaction(qd2_direct_kNm, span_m, beam_overhang_m, q_roof_d_kNm2, trib_w2_m, roof_eave_m)
    beam1_direct_reaction_sls_kN = symmetric_support_reaction(qk_sls1_direct_kNm, span_m, beam_overhang_m, q_roof_sls_kNm2, trib_w1_m, roof_eave_m)
    beam2_direct_reaction_sls_kN = symmetric_support_reaction(qk_sls2_direct_kNm, span_m, beam_overhang_m, q_roof_sls_kNm2, trib_w2_m, roof_eave_m)
    beam1_direct_reaction_uplift_kN = symmetric_support_reaction(qmin1_direct_kNm, span_m, beam_overhang_m, q_roof_min_kNm2, trib_w1_m, roof_eave_m)
    beam2_direct_reaction_uplift_kN = symmetric_support_reaction(qmin2_direct_kNm, span_m, beam_overhang_m, q_roof_min_kNm2, trib_w2_m, roof_eave_m)

    beam1_add_reactions_uls = simple_span_point_reactions(
        span_m,
        [((x_mm - support_left_x_mm) / 1000.0, p_kN) for x_mm, p_kN in beam1_purlin_uls_abs_x_mm],
    )
    beam1_add_reactions_sls = simple_span_point_reactions(
        span_m,
        [((x_mm - support_left_x_mm) / 1000.0, p_kN) for x_mm, p_kN in beam1_purlin_sls_abs_x_mm],
    )
    beam1_add_reactions_uplift = simple_span_point_reactions(
        span_m,
        [((x_mm - support_left_x_mm) / 1000.0, p_kN) for x_mm, p_kN in beam1_purlin_uplift_abs_x_mm],
    )
    beam2_add_reactions_uls = simple_span_point_reactions(
        span_m,
        [((x_mm - support_left_x_mm) / 1000.0, p_kN) for x_mm, p_kN in beam2_purlin_uls_abs_x_mm],
    )
    beam2_add_reactions_sls = simple_span_point_reactions(
        span_m,
        [((x_mm - support_left_x_mm) / 1000.0, p_kN) for x_mm, p_kN in beam2_purlin_sls_abs_x_mm],
    )
    beam2_add_reactions_uplift = simple_span_point_reactions(
        span_m,
        [((x_mm - support_left_x_mm) / 1000.0, p_kN) for x_mm, p_kN in beam2_purlin_uplift_abs_x_mm],
    )

    def equivalent_line_load_kNm(direct_line_load_kNm, point_loads_abs_x_mm):
        return direct_line_load_kNm + sum(p_kN for _, p_kN in point_loads_abs_x_mm) / span_m

    qd1_eq_kNm = equivalent_line_load_kNm(qd1_direct_kNm, beam1_purlin_uls_abs_x_mm)
    qd2_eq_kNm = equivalent_line_load_kNm(qd2_direct_kNm, beam2_purlin_uls_abs_x_mm)
    qk_sls1_eq_kNm = equivalent_line_load_kNm(qk_sls1_direct_kNm, beam1_purlin_sls_abs_x_mm)
    qk_sls2_eq_kNm = equivalent_line_load_kNm(qk_sls2_direct_kNm, beam2_purlin_sls_abs_x_mm)
    qmin1_eq_kNm = equivalent_line_load_kNm(qmin1_direct_kNm, beam1_purlin_uplift_abs_x_mm)
    qmin2_eq_kNm = equivalent_line_load_kNm(qmin2_direct_kNm, beam2_purlin_uplift_abs_x_mm)

    kmod_lvl = 0.65
    gammaM_lvl = 1.2
    fm_d_lvl = kmod_lvl * 44.0 / gammaM_lvl
    fv_d_lvl = kmod_lvl * 4.5 / gammaM_lvl
    W2_mm3 = b2_mm * h2_mm**2 / 6.0
    MRd2_kNm = fm_d_lvl * W2_mm3 / 1.0e6
    VRd2_kN = fv_d_lvl * (b2_mm * h2_mm) / 1.5e3
    E_005_lvl = 11600.0
    E_mean_lvl = 13800.0
    I2_mm4 = b2_mm * h2_mm**3 / 12.0
    EI2_Nmm2 = E_mean_lvl * I2_mm4
    ltb_L_ruode_mm = 0.9 * float(member(geo, "purlins", "purlin.50x100")["pattern"]["offset"]["x"])
    sigma_crit = 0.78 * b2_mm**2 * E_005_lvl / (h2_mm * ltb_L_ruode_mm)
    lam = math.sqrt(44.0 / sigma_crit)
    if lam <= 0.75:
        kcrit = 1.0
    elif lam <= 1.4:
        kcrit = 1.56 - 0.75 * lam
    else:
        kcrit = 1.0 / lam**2

    kmod_lp = 0.65
    gammaM_lp = 1.25
    fm_d_lp = kmod_lp * 30.0 / gammaM_lp
    fv_d_lp = kmod_lp * 3.5 / gammaM_lp
    W_lp_mm3 = b_lp_mm * h_lp_mm**2 / 6.0
    MRd_lp_kNm = fm_d_lp * W_lp_mm3 / 1.0e6
    VRd_lp_kN = fv_d_lp * (b_lp_mm * h_lp_mm) / 1.5e3

    lp_end_y_mm = float(lp_x125["axis_end"]["y"])

    return {
        "gammaG": gammaG,
        "gammaQ": gammaQ,
        "roof": {
            "x0_mm": roof_x0_mm,
            "x1_mm": roof_x1_mm,
            "edge_y_mm": float(roof_edge_y_mm),
            "gk_roofing_kNm2": gk_roofing,
            "snow_kNm2": s_roof,
            "wind_model": wind_model,
            "cp_net_down": cp_net_down,
            "cp_net_up": cp_net_up,
            "wind_down_kNm2": w_wind_down,
            "wind_up_kNm2": w_wind_up,
            "q_roof_uls_kNm2": q_roof_d_kNm2,
            "q_roof_sls_kNm2": q_roof_sls_kNm2,
            "q_roof_uplift_kNm2": q_roof_min_kNm2,
        },
        "purlins_main": {
            "count": len(purlin_rows),
            "count_main": len(main_purlins),
            "count_diag": len(diag_purlins),
            "support_inner_y_mm": purlin_inner_support_y_mm,
            "support_outer_y_mm": purlin_outer_support_y_mm,
            "loaded_depth_m": purlin_loaded_depth_m,
            "self_weight_kNm": purlin_self_kNm,
            "members": [
                {
                    **base_row,
                    "tributary_x_start_mm": uls_row["tributary_x_start_mm"],
                    "tributary_x_end_mm": uls_row["tributary_x_end_mm"],
                    "tributary_width_m": uls_row["tributary_width_m"],
                    "tributary_area_m2": uls_row["tributary_area_m2"],
                    "area_load_factor_m": uls_row["area_load_factor_m"],
                    "member_length_m": uls_row["member_length_m"],
                    "uls": {
                        "line_load_kNm": uls_row["line_load_kNm"],
                        "reaction_inner_kN": uls_row["reaction_inner_kN"],
                        "reaction_outer_kN": uls_row["reaction_outer_kN"],
                    },
                    "sls": {
                        "line_load_kNm": sls_row["line_load_kNm"],
                        "reaction_inner_kN": sls_row["reaction_inner_kN"],
                        "reaction_outer_kN": sls_row["reaction_outer_kN"],
                    },
                    "uplift": {
                        "line_load_kNm": uplift_row["line_load_kNm"],
                        "reaction_inner_kN": uplift_row["reaction_inner_kN"],
                        "reaction_outer_kN": uplift_row["reaction_outer_kN"],
                    },
                }
                for base_row, uls_row, sls_row, uplift_row in zip(purlin_rows, purlin_rows_uls, purlin_rows_sls, purlin_rows_uplift)
            ],
        },
        "purlins_kp450_side": {
            "count": len(kp450_side_rows),
            "members": kp450_side_rows,
        },
        "kp450_y900": {
            "support_left_x_mm": support_left_x_mm,
            "support_right_x_mm": support_right_x_mm,
            "span_mm": span_mm,
            "span_m": span_m,
            "direct_tributary_start_y_mm": trib1_start_mm,
            "direct_tributary_end_y_mm": trib1_end_mm,
            "direct_tributary_width_m": trib_w1_m,
            "gk_direct_kNm": gk1_direct_kNm,
            "q_snow_direct_kNm": qk_snow1_direct_kNm,
            "q_wind_down_direct_kNm": qk_wind_down1_direct_kNm,
            "q_wind_up_direct_kNm": qk_wind_up1_direct_kNm,
            "qd_uls_direct_kNm": qd1_direct_kNm,
            "q_sls_direct_kNm": qk_sls1_direct_kNm,
            "q_uplift_direct_kNm": qmin1_direct_kNm,
            "q_eq_uls_kNm": qd1_eq_kNm,
            "q_eq_sls_kNm": qk_sls1_eq_kNm,
            "q_eq_uplift_kNm": qmin1_eq_kNm,
            "direct_support_reaction_uls_kN": beam1_direct_reaction_uls_kN,
            "direct_support_reaction_sls_kN": beam1_direct_reaction_sls_kN,
            "direct_support_reaction_uplift_kN": beam1_direct_reaction_uplift_kN,
            "base_point_loads_uls_abs_x_mm": beam1_purlin_uls_abs_x_mm,
            "base_point_loads_sls_abs_x_mm": beam1_purlin_sls_abs_x_mm,
            "base_point_loads_uplift_abs_x_mm": beam1_purlin_uplift_abs_x_mm,
            "reactions_uls_kN": {
                support_left_x_mm: beam1_direct_reaction_uls_kN + beam1_add_reactions_uls[0],
                support_right_x_mm: beam1_direct_reaction_uls_kN + beam1_add_reactions_uls[1],
            },
            "reactions_sls_kN": {
                support_left_x_mm: beam1_direct_reaction_sls_kN + beam1_add_reactions_sls[0],
                support_right_x_mm: beam1_direct_reaction_sls_kN + beam1_add_reactions_sls[1],
            },
            "reactions_uplift_kN": {
                support_left_x_mm: beam1_direct_reaction_uplift_kN + beam1_add_reactions_uplift[0],
                support_right_x_mm: beam1_direct_reaction_uplift_kN + beam1_add_reactions_uplift[1],
            },
        },
        "kp360": {
            "support_left_x_mm": support_left_x_mm,
            "support_right_x_mm": support_right_x_mm,
            "span_mm": span_mm,
            "span_m": span_m,
            "direct_tributary_start_y_mm": trib2_start_mm,
            "direct_tributary_end_y_mm": trib2_end_mm,
            "direct_tributary_width_m": trib_w2_m,
            "gk_direct_kNm": gk2_direct_kNm,
            "q_snow_direct_kNm": qk_snow2_direct_kNm,
            "q_wind_down_direct_kNm": qk_wind_down2_direct_kNm,
            "q_wind_up_direct_kNm": qk_wind_up2_direct_kNm,
            "qd_uls_direct_kNm": qd2_direct_kNm,
            "q_sls_direct_kNm": qk_sls2_direct_kNm,
            "q_uplift_direct_kNm": qmin2_direct_kNm,
            "q_eq_uls_kNm": qd2_eq_kNm,
            "q_eq_sls_kNm": qk_sls2_eq_kNm,
            "q_eq_uplift_kNm": qmin2_eq_kNm,
            "direct_support_reaction_uls_kN": beam2_direct_reaction_uls_kN,
            "direct_support_reaction_sls_kN": beam2_direct_reaction_sls_kN,
            "direct_support_reaction_uplift_kN": beam2_direct_reaction_uplift_kN,
            "base_point_loads_uls_abs_x_mm": beam2_purlin_uls_abs_x_mm,
            "base_point_loads_sls_abs_x_mm": beam2_purlin_sls_abs_x_mm,
            "base_point_loads_uplift_abs_x_mm": beam2_purlin_uplift_abs_x_mm,
            "reactions_uls_kN": {
                support_left_x_mm: beam2_direct_reaction_uls_kN + beam2_add_reactions_uls[0],
                support_right_x_mm: beam2_direct_reaction_uls_kN + beam2_add_reactions_uls[1],
            },
            "reactions_sls_kN": {
                support_left_x_mm: beam2_direct_reaction_sls_kN + beam2_add_reactions_sls[0],
                support_right_x_mm: beam2_direct_reaction_sls_kN + beam2_add_reactions_sls[1],
            },
            "reactions_uplift_kN": {
                support_left_x_mm: beam2_direct_reaction_uplift_kN + beam2_add_reactions_uplift[0],
                support_right_x_mm: beam2_direct_reaction_uplift_kN + beam2_add_reactions_uplift[1],
            },
            "MRd_kNm": MRd2_kNm,
            "VRd_kN": VRd2_kN,
            "W_mm3": W2_mm3,
            "fm_d_Nmm2": fm_d_lvl,
            "ltb_kcrit": kcrit,
            "EI_Nmm2": EI2_Nmm2,
            "profile_name": kp360["profile"]["name"],
        },
        "lp225_x125": {
            "support_left_y_mm": 0.0,
            "support_right_y_mm": float(beam2_y_mm),
            "beam_end_y_mm": lp_end_y_mm,
            "base_point_y_mm": float(beam1_y_mm),
            "base_point_uls_kN": beam1_direct_reaction_uls_kN + beam1_add_reactions_uls[0],
            "base_point_sls_kN": beam1_direct_reaction_sls_kN + beam1_add_reactions_sls[0],
            "base_point_uplift_kN": beam1_direct_reaction_uplift_kN + beam1_add_reactions_uplift[0],
            "MRd_kNm": MRd_lp_kNm,
            "VRd_kN": VRd_lp_kN,
            "profile_name": lp_x125["profile"]["name"],
        },
    }


def _kp360_base_values(context, load_case):
    kp360 = context["kp360"]
    if load_case == "ULS":
        return kp360["qd_uls_direct_kNm"], kp360["direct_support_reaction_uls_kN"], kp360["base_point_loads_uls_abs_x_mm"]
    if load_case == "SLS":
        return kp360["q_sls_direct_kNm"], kp360["direct_support_reaction_sls_kN"], kp360["base_point_loads_sls_abs_x_mm"]
    if load_case == "UPLIFT":
        return kp360["q_uplift_direct_kNm"], kp360["direct_support_reaction_uplift_kN"], kp360["base_point_loads_uplift_abs_x_mm"]
    raise ValueError(f"Unsupported load case: {load_case}")


def _lp_base_point_load(context, load_case):
    lp = context["lp225_x125"]
    if load_case == "ULS":
        return lp["base_point_uls_kN"]
    if load_case == "SLS":
        return lp["base_point_sls_kN"]
    if load_case == "UPLIFT":
        return lp["base_point_uplift_kN"]
    raise ValueError(f"Unsupported load case: {load_case}")


def check_existing_kp360_combined(extra_point_loads_abs_x_mm=None, load_case="ULS", context=None):
    if context is None:
        context = katos_existing_context()
    kp360 = context["kp360"]
    line_load_kNm, direct_support_reaction_kN, base_point_loads_abs_x_mm = _kp360_base_values(context, load_case)
    extra_point_loads_abs_x_mm = aggregate_point_loads(extra_point_loads_abs_x_mm)
    point_loads_abs_x_mm = aggregate_point_loads([*base_point_loads_abs_x_mm, *extra_point_loads_abs_x_mm])
    point_loads_local_m = [
        ((x_mm - kp360["support_left_x_mm"]) / 1000.0, p_kN)
        for x_mm, p_kN in point_loads_abs_x_mm
    ]
    add_left_kN, add_right_kN = simple_span_point_reactions(kp360["span_m"], point_loads_local_m)
    moment_x_m, moment_kNm = simple_span_combined_moment_max(kp360["span_m"], line_load_kNm, point_loads_local_m)
    shear_kN, shear_x_m = simple_span_max_shear(kp360["span_m"], line_load_kNm, point_loads_local_m)
    delta_point_loads_abs_x_mm = aggregate_point_loads([*kp360["base_point_loads_sls_abs_x_mm"], *extra_point_loads_abs_x_mm])
    delta_point_loads_local_m = [
        ((x_mm - kp360["support_left_x_mm"]) / 1000.0, p_kN)
        for x_mm, p_kN in delta_point_loads_abs_x_mm
    ]
    delta_mm, delta_x_mm = simple_span_max_deflection_mm(
        kp360["span_mm"],
        kp360["EI_Nmm2"],
        udl_kNm=kp360["q_sls_direct_kNm"],
        point_loads_local_m=delta_point_loads_local_m,
    )
    sigma_md = abs(moment_kNm) * 1.0e6 / kp360["W_mm3"]
    return {
        "profile": kp360["profile_name"],
        "load_case": load_case,
        "reactions_kN": {
            kp360["support_left_x_mm"]: direct_support_reaction_kN + add_left_kN,
            kp360["support_right_x_mm"]: direct_support_reaction_kN + add_right_kN,
        },
        "point_loads_abs_mm": point_loads_abs_x_mm,
        "M_gov": {
            "value_kNm": abs(moment_kNm),
            "x_mm": kp360["support_left_x_mm"] + 1000.0 * moment_x_m,
            "raw_value_kNm": moment_kNm,
        },
        "V_abs": {
            "value_kN": shear_kN,
            "x_mm": kp360["support_left_x_mm"] + 1000.0 * shear_x_m,
        },
        "delta": {
            "value_mm": abs(delta_mm),
            "x_mm": kp360["support_left_x_mm"] + delta_x_mm,
        },
        "MRd_kNm": kp360["MRd_kNm"],
        "VRd_kN": kp360["VRd_kN"],
        "eta_M": abs(moment_kNm) / kp360["MRd_kNm"] * 100.0,
        "eta_V": abs(shear_kN) / kp360["VRd_kN"] * 100.0,
        "eta_LTB": sigma_md / (kp360["ltb_kcrit"] * kp360["fm_d_Nmm2"]) * 100.0,
        "delta_lim_mm": kp360["span_mm"] / 300.0,
    }


def check_existing_lp225_x125_combined(extra_point_loads_abs_y_mm=None, load_case="ULS", context=None):
    if context is None:
        context = katos_existing_context()
    lp = context["lp225_x125"]
    base_point_kN = _lp_base_point_load(context, load_case)
    extra_point_loads_abs_y_mm = aggregate_point_loads(extra_point_loads_abs_y_mm)
    point_loads_abs_mm = [(lp["base_point_y_mm"], base_point_kN), *extra_point_loads_abs_y_mm]
    response = point_beam_with_right_overhang_response(
        lp["support_right_y_mm"],
        lp["beam_end_y_mm"],
        point_loads_abs_mm,
    )
    moment_kNm = response["M_gov"]["value_kNm"]
    shear_kN = abs(response["V_abs"]["value_kN"])
    return {
        "profile": lp["profile_name"],
        "load_case": load_case,
        "reactions_kN": response["reactions_kN"],
        "point_loads_abs_mm": response["point_loads_abs_mm"],
        "M_gov": response["M_gov"],
        "V_abs": response["V_abs"],
        "MRd_kNm": lp["MRd_kNm"],
        "VRd_kN": lp["VRd_kN"],
        "eta_M": moment_kNm / lp["MRd_kNm"] * 100.0,
        "eta_V": shear_kN / lp["VRd_kN"] * 100.0,
    }
