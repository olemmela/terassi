"""
LASITETUN TERASSIN LOPULLINEN PUURATKAISU – KUORMITUSLASKENTA – ETELÄSUOMI
============================================================================
Standardit: EN 1990, EN 1991-1-1, EN 1991-1-3, EN 1991-1-4, EN 1995-1-1

Geometria luetaan tiedostosta geometry/terassi_puu.json:
  - uusi puuratkaisu: orret 98×48, nurkkaorret 98×48, kattotuolit 198×48,
    ulkopalkki LP225×140, sisäpalkki LP315×140
  - aurinkopaneelien reunakaistat siirtyvät reunakattotuoleille vain orsien kautta
  - ulkokulmien nurkkaorret sidotaan uloimpaan orteen ja kantavat ulkopalkin ulkoreunalla
  - orsien tuet mallinnetaan nivellettyinä; reunakattotuolien molemmat päät puolijäykkinä
  - sisäpään palkkikenkä N 48×136 mallinnetaan heikosti kiertymää jäykistävänä
    puolijäykkänä liitoksena (5.0×40 ankkuriruuvit, täyskiinnitys)
  - kinostuma talon seinää vasten johdetaan geometriasta muuttuvalla h(x)-korkeudella
  - lovi- ja nettoh-tarkistukset luetaan geometry/terassi_puu.json:n cuts-kentistä
"""

import math

from geometry_loader import expanded_members, load, member, surface, reference, profile_b, profile_h


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


def solve_linear_system(A, b):
    A = [row[:] for row in A]
    b = b[:]
    n = len(b)
    for i in range(n):
        pivot = max(range(i, n), key=lambda r: abs(A[r][i]))
        if abs(A[pivot][i]) < 1e-12:
            raise ValueError("Singular matrix in beam analysis")
        if pivot != i:
            A[i], A[pivot] = A[pivot], A[i]
            b[i], b[pivot] = b[pivot], b[i]
        pivot_val = A[i][i]
        for j in range(i, n):
            A[i][j] /= pivot_val
        b[i] /= pivot_val
        for r in range(n):
            if r == i:
                continue
            factor = A[r][i]
            if abs(factor) < 1e-18:
                continue
            for j in range(i, n):
                A[r][j] -= factor * A[i][j]
            b[r] -= factor * b[i]
    return b


def refine_nodes_mm(points_mm, max_len_mm):
    pts = sorted(set(float(x) for x in points_mm))
    refined = [pts[0]]
    for a_mm, b_mm in zip(pts, pts[1:]):
        n = max(1, int(math.ceil((b_mm - a_mm) / max_len_mm)))
        step_mm = (b_mm - a_mm) / n
        for i in range(1, n + 1):
            refined.append(b_mm if i == n else a_mm + i * step_mm)
    return refined


def segment_key(a_mm, b_mm):
    return (round(float(a_mm), 6), round(float(b_mm), 6))


def beam_solver(
    nodes_mm,
    supports_mm,
    point_loads_kN=None,
    uniform_loads_kN_per_mm=None,
    EI_Nmm2=1.0,
    EI_by_segment_Nmm2=None,
    rotational_springs_Nmm_per_rad=None,
):
    if point_loads_kN is None:
        point_loads_kN = []
    if uniform_loads_kN_per_mm is None:
        uniform_loads_kN_per_mm = []
    if rotational_springs_Nmm_per_rad is None:
        rotational_springs_Nmm_per_rad = {}

    nodes = sorted(
        {float(x_mm) for x_mm in nodes_mm}
        | {float(x_mm) for x_mm in supports_mm}
        | {float(x_mm) for x_mm, _ in point_loads_kN}
        | {float(a_mm) for a_mm, _, _ in uniform_loads_kN_per_mm}
        | {float(b_mm) for _, b_mm, _ in uniform_loads_kN_per_mm}
    )
    n_nodes = len(nodes)
    node_index = {x: i for i, x in enumerate(nodes)}
    n_dof = 2 * n_nodes
    K = [[0.0] * n_dof for _ in range(n_dof)]
    F = [0.0] * n_dof
    element_data = []

    for x_mm, p_kN in point_loads_kN:
        F[2 * node_index[float(x_mm)]] -= p_kN * 1000.0

    q_by_segment = {
        segment_key(a_mm, b_mm): float(q_kN_per_mm)
        for a_mm, b_mm, q_kN_per_mm in uniform_loads_kN_per_mm
    }

    for elem_i in range(n_nodes - 1):
        x0_mm = nodes[elem_i]
        x1_mm = nodes[elem_i + 1]
        L_mm = x1_mm - x0_mm
        EI_elem_Nmm2 = EI_Nmm2 if EI_by_segment_Nmm2 is None else EI_by_segment_Nmm2[elem_i]
        fac = EI_elem_Nmm2 / (L_mm**3)
        k = [
            [12.0 * fac, 6.0 * L_mm * fac, -12.0 * fac, 6.0 * L_mm * fac],
            [6.0 * L_mm * fac, 4.0 * L_mm * L_mm * fac, -6.0 * L_mm * fac, 2.0 * L_mm * L_mm * fac],
            [-12.0 * fac, -6.0 * L_mm * fac, 12.0 * fac, -6.0 * L_mm * fac],
            [6.0 * L_mm * fac, 2.0 * L_mm * L_mm * fac, -6.0 * L_mm * fac, 4.0 * L_mm * L_mm * fac],
        ]
        dofs = [2 * elem_i, 2 * elem_i + 1, 2 * (elem_i + 1), 2 * (elem_i + 1) + 1]
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

        element_data.append({
            "x0_mm": x0_mm,
            "x1_mm": x1_mm,
            "dofs": dofs,
            "k": k,
            "fe": fe,
            "q_kN_per_mm": q_kN_per_mm,
        })

    for x_mm, k_theta in rotational_springs_Nmm_per_rad.items():
        x_key = float(x_mm)
        if x_key not in node_index:
            continue
        rot_dof = 2 * node_index[x_key] + 1
        K[rot_dof][rot_dof] += float(k_theta)

    fixed = sorted({2 * node_index[float(x_mm)] for x_mm in supports_mm})
    free = [i for i in range(n_dof) if i not in fixed]
    Kff = [[K[i][j] for j in free] for i in free]
    Ff = [F[i] for i in free]
    uf = solve_linear_system(Kff, Ff)

    u = [0.0] * n_dof
    for dof_i, value in zip(free, uf):
        u[dof_i] = value

    Ku = [sum(K[i][j] * u[j] for j in range(n_dof)) for i in range(n_dof)]
    R = [Ku[i] - F[i] for i in range(n_dof)]

    elements = []
    for elem in element_data:
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

    return {
        "nodes_mm": nodes,
        "reactions_kN": {float(x_mm): R[2 * node_index[float(x_mm)]] / 1000.0 for x_mm in supports_mm},
        "disp_mm": {x_mm: u[2 * node_index[x_mm]] for x_mm in nodes},
        "rot_rad": {x_mm: u[2 * node_index[x_mm] + 1] for x_mm in nodes},
        "elements": elements,
    }


def uniform_loads_for_nodes(nodes_mm, q_kN_per_mm):
    return [(a_mm, b_mm, q_kN_per_mm) for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:])]


def combine_uniform_loads(*load_sets):
    combined = {}
    for load_set in load_sets:
        for a_mm, b_mm, q_kN_per_mm in load_set:
            key = segment_key(a_mm, b_mm)
            if key not in combined:
                combined[key] = [float(a_mm), float(b_mm), 0.0]
            combined[key][2] += float(q_kN_per_mm)
    return [(a_mm, b_mm, q_kN_per_mm) for a_mm, b_mm, q_kN_per_mm in combined.values()]


def load_stats(loads):
    total_q_len = 0.0
    total_len = 0.0
    q_min_kNm = None
    q_max_kNm = None
    for a_mm, b_mm, q_kN_per_mm in loads:
        q_kNm = q_kN_per_mm * 1000.0
        seg_len_mm = b_mm - a_mm
        total_q_len += q_kNm * seg_len_mm
        total_len += seg_len_mm
        q_min_kNm = q_kNm if q_min_kNm is None else min(q_min_kNm, q_kNm)
        q_max_kNm = q_kNm if q_max_kNm is None else max(q_max_kNm, q_kNm)
    return {
        "avg_kNm": total_q_len / total_len if total_len > 0.0 else 0.0,
        "min_kNm": 0.0 if q_min_kNm is None else q_min_kNm,
        "max_kNm": 0.0 if q_max_kNm is None else q_max_kNm,
    }


def total_uniform_load_kN(loads):
    return sum((b_mm - a_mm) * q_kN_per_mm for a_mm, b_mm, q_kN_per_mm in loads)


def intervals_to_uniform_loads(nodes_mm, intervals):
    loads = []
    for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:]):
        q_kN_per_mm = 0.0
        mid_mm = 0.5 * (a_mm + b_mm)
        for i0_mm, i1_mm, q_interval_kN_per_mm in intervals:
            if i0_mm - 1e-9 <= mid_mm <= i1_mm + 1e-9:
                q_kN_per_mm += q_interval_kN_per_mm
        loads.append((a_mm, b_mm, q_kN_per_mm))
    return loads


def element_section_state(elem, x_mm):
    x0_mm = float(elem["x0_mm"])
    x1_mm = float(elem["x1_mm"])
    xi_mm = max(0.0, min(float(x_mm), x1_mm) - x0_mm)
    q_N_per_mm = float(elem["q_kN_per_mm"]) * 1000.0
    V1_N, M1_Nmm = float(elem["end_forces"][0]), float(elem["end_forces"][1])
    return {
        "V_kN": (V1_N - q_N_per_mm * xi_mm) / 1000.0,
        "M_kNm": (-M1_Nmm + V1_N * xi_mm - 0.5 * q_N_per_mm * xi_mm**2) / 1.0e6,
    }


def section_state_at_x_mm(elements, x_mm, tol=1e-9):
    candidates = []
    for elem in elements:
        if float(elem["x0_mm"]) - tol <= x_mm <= float(elem["x1_mm"]) + tol:
            candidates.append(element_section_state(elem, x_mm))
    if not candidates:
        raise ValueError(f"x={x_mm} ei osu mihinkään elementtiin")
    moment = max(candidates, key=lambda item: abs(item["M_kNm"]))["M_kNm"]
    shear = max(candidates, key=lambda item: abs(item["V_kN"]))["V_kN"]
    return {"M_kNm": moment, "V_kN": shear}


def sample_internal_forces(elements):
    max_pos = (-1e18, None)
    max_neg = (1e18, None)
    max_shear = (0.0, None)

    def update_moment(value_kNm, x_mm):
        nonlocal max_pos, max_neg
        if value_kNm > max_pos[0]:
            max_pos = (value_kNm, x_mm)
        if value_kNm < max_neg[0]:
            max_neg = (value_kNm, x_mm)

    def update_shear(value_kN, x_mm):
        nonlocal max_shear
        if abs(value_kN) > abs(max_shear[0]):
            max_shear = (value_kN, x_mm)

    for elem in elements:
        x0_mm = float(elem["x0_mm"])
        x1_mm = float(elem["x1_mm"])
        L_mm = x1_mm - x0_mm
        q_N_per_mm = float(elem["q_kN_per_mm"]) * 1000.0
        V1_N = float(elem["end_forces"][0])

        left_state = element_section_state(elem, x0_mm)
        right_state = element_section_state(elem, x1_mm)
        update_moment(left_state["M_kNm"], x0_mm)
        update_moment(right_state["M_kNm"], x1_mm)
        update_shear(left_state["V_kN"], x0_mm)
        update_shear(right_state["V_kN"], x1_mm)

        if abs(q_N_per_mm) > 1e-18:
            xi_zero_mm = V1_N / q_N_per_mm
            if 0.0 < xi_zero_mm < L_mm:
                x_zero_mm = x0_mm + xi_zero_mm
                update_moment(element_section_state(elem, x_zero_mm)["M_kNm"], x_zero_mm)

    return {
        "M_pos": {"value_kNm": max_pos[0], "x_mm": max_pos[1]},
        "M_neg": {"value_kNm": max_neg[0], "x_mm": max_neg[1]},
        "V_abs": {"value_kN": max_shear[0], "x_mm": max_shear[1]},
    }


def sample_max_deflection_mm(nodes_mm, disp_mm, rot_rad, step_mm=2.0):
    max_abs = (0.0, None)
    for x0_mm, x1_mm in zip(nodes_mm, nodes_mm[1:]):
        L_mm = x1_mm - x0_mm
        v1 = disp_mm[x0_mm]
        t1 = rot_rad[x0_mm]
        v2 = disp_mm[x1_mm]
        t2 = rot_rad[x1_mm]
        xi_mm = 0.0
        while xi_mm <= L_mm + 1e-9:
            s = xi_mm / L_mm
            N1 = 1.0 - 3.0 * s**2 + 2.0 * s**3
            N2 = L_mm * (s - 2.0 * s**2 + s**3)
            N3 = 3.0 * s**2 - 2.0 * s**3
            N4 = L_mm * (-s**2 + s**3)
            v_mm = N1 * v1 + N2 * t1 + N3 * v2 + N4 * t2
            if abs(v_mm) > abs(max_abs[0]):
                max_abs = (v_mm, x0_mm + xi_mm)
            xi_mm += step_mm
    return {"value_mm": max_abs[0], "x_mm": max_abs[1]}


def sample_net_section_utilization(elements, section_h_mm_at_x, b_mm, fm_d_Nmm2, fv_d_Nmm2, x_start_mm, x_end_mm, step_mm=1.0):
    x_lo_mm = min(float(x_start_mm), float(x_end_mm))
    x_hi_mm = max(float(x_start_mm), float(x_end_mm))
    max_eta_M = {"value_pct": 0.0, "x_mm": None, "M_kNm": 0.0, "h_mm": None}
    max_eta_V = {"value_pct": 0.0, "x_mm": None, "V_kN": 0.0, "h_mm": None}

    x_mm = x_lo_mm
    while x_mm <= x_hi_mm + 1e-9:
        h_mm = max(1e-9, float(section_h_mm_at_x(x_mm)))
        state = section_state_at_x_mm(elements, x_mm)
        W_mm3 = b_mm * h_mm**2 / 6.0
        A_mm2 = b_mm * h_mm
        MRd_kNm = fm_d_Nmm2 * W_mm3 / 1.0e6
        VRd_kN = fv_d_Nmm2 * A_mm2 / 1.5e3
        eta_M_pct = abs(state["M_kNm"]) / MRd_kNm * 100.0
        eta_V_pct = abs(state["V_kN"]) / VRd_kN * 100.0

        if eta_M_pct > max_eta_M["value_pct"]:
            max_eta_M = {"value_pct": eta_M_pct, "x_mm": x_mm, "M_kNm": state["M_kNm"], "h_mm": h_mm}
        if eta_V_pct > max_eta_V["value_pct"]:
            max_eta_V = {"value_pct": eta_V_pct, "x_mm": x_mm, "V_kN": state["V_kN"], "h_mm": h_mm}
        x_mm += step_mm

    eta_gov = max(
        ({"mode": "M_netto", **max_eta_M}, {"mode": "V_netto", **max_eta_V}),
        key=lambda item: item["value_pct"],
    )
    return {"eta_M": max_eta_M, "eta_V": max_eta_V, "eta_gov": eta_gov}


def governing_moment(internal):
    if internal["M_pos"]["value_kNm"] >= -internal["M_neg"]["value_kNm"]:
        return {"sign": "+", **internal["M_pos"]}
    return {
        "sign": "−",
        "value_kNm": -internal["M_neg"]["value_kNm"],
        "x_mm": internal["M_neg"]["x_mm"],
        "raw_value_kNm": internal["M_neg"]["value_kNm"],
    }


def member_rect_props(b_mm, h_mm):
    return {
        "W_mm3": b_mm * h_mm**2 / 6.0,
        "A_mm2": b_mm * h_mm,
        "I_strong_mm4": b_mm * h_mm**3 / 12.0,
        "I_weak_mm4": h_mm * b_mm**3 / 12.0,
    }


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
CONNECTIONS = {conn["id"]: conn for conn in GEO["connections"]}


def connection_by_members(member_a_id, member_b_id):
    wanted = {member_a_id, member_b_id}
    for conn in GEO["connections"]:
        if set(conn.get("members", [])) == wanted:
            return conn
    raise KeyError(f"Connection not found for members: {member_a_id}, {member_b_id}")


def first_connection_matching(member_id, predicate):
    for conn in GEO["connections"]:
        members = conn.get("members", [])
        if member_id in members and predicate(conn, members):
            return conn
    return None


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
        return {"active": False, "depth_mm": 0.0, "length_mm": 0.0, "offset_mm": 0.0}
    return {
        "active": True,
        "depth_mm": float(cut["depth_mm"]),
        "length_mm": float(cut["length_mm"]),
        "offset_mm": float(cut.get("offset_mm", 0.0)),
    }


def bevel_notch_info(connection_id):
    cut = connection_cut(connection_id, "bevel_bottom_notch")
    if cut is None:
        return {
            "active": False,
            "depth_mm": 0.0,
            "length_mm": 0.0,
            "offset_mm": 0.0,
            "member_end": None,
            "reference": None,
        }
    return {
        "active": True,
        "depth_mm": float(cut["depth_mm"]),
        "length_mm": float(cut["length_mm"]),
        "offset_mm": float(cut.get("offset_mm", 0.0)),
        "member_end": cut["member_end"],
        "reference": cut.get("reference", "member_end"),
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


def member_axis_length_mm(member_obj):
    dx_mm, dy_mm, dz_mm = member_axis_vector_3d(member_obj)
    return math.sqrt(dx_mm**2 + dy_mm**2 + dz_mm**2)


def project_point_to_member_s_mm(member_obj, point_xyz):
    start = member_obj["axis_start"]
    dx_mm, dy_mm, dz_mm = member_axis_vector_3d(member_obj)
    length_sq = dx_mm**2 + dy_mm**2 + dz_mm**2
    if length_sq <= 1e-9:
        return 0.0
    px_mm = float(point_xyz["x"]) - float(start["x"])
    py_mm = float(point_xyz["y"]) - float(start["y"])
    pz_mm = float(point_xyz.get("z", start["z"])) - float(start["z"])
    t = clamp((px_mm * dx_mm + py_mm * dy_mm + pz_mm * dz_mm) / length_sq, 0.0, 1.0)
    return t * math.sqrt(length_sq)


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

outer_beam = member(GEO, "beams", "beam.outer")
inner_beam = member(GEO, "beams", "beam.inner.new")
existing_beam = member(GEO, "beams", "beam.existing.kp360x2")

outer_supports_x_mm = sorted(float(member(GEO, "columns", cid)["base"]["x"]) for cid in ("col.outer.x0", "col.outer.x3600", "col.outer.x7200"))
inner_supports_x_mm = sorted(float(member(GEO, "columns", cid)["base"]["x"]) for cid in ("col.existing.inner.x125", "col.existing.inner.x7075"))

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

interior_inner_support_y_mm = float(CONNECTIONS["con.kattotuoli.on.inner_beam"]["at"]["y"])
interior_outer_support_y_mm = float(CONNECTIONS["con.kattotuoli.on.outer_beam"]["at"]["y"])
edge_inner_support_y_mm = float(CONNECTIONS["con.kattotuoli.vasen.on.inner_beam"]["at"]["y"])
edge_outer_support_y_mm = float(CONNECTIONS["con.kattotuoli.vasen.on.outer_beam"]["at"]["y"])
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
purlin_inner_notch_ref = next((item for item in purlin_inner_notch_info.values() if item["active"]), None)
purlin_notch_depth_mm = purlin_inner_notch_ref["depth_mm"] if purlin_inner_notch_ref else 0.0
purlin_notch_length_mm = purlin_inner_notch_ref["length_mm"] if purlin_inner_notch_ref else 0.0
purlin_edge_notch_info = {
    "left": rect_notch_info("con.orsi.vasen.on.kattotuoli.vasen"),
    "right": rect_notch_info("con.orsi.oikea.on.kattotuoli.oikea"),
}
purlin_edge_notch_ref = next((item for item in purlin_edge_notch_info.values() if item["active"]), None)
purlin_edge_notch_depth_mm = purlin_edge_notch_ref["depth_mm"] if purlin_edge_notch_ref else 0.0
purlin_edge_notch_length_mm = purlin_edge_notch_ref["length_mm"] if purlin_edge_notch_ref else 0.0
left_purlin_support_x_mm = max(float(left_purlins[0]["axis_start"]["x"]), float(left_purlins[0]["axis_end"]["x"]))
right_purlin_support_x_mm = min(float(right_purlins[0]["axis_start"]["x"]), float(right_purlins[0]["axis_end"]["x"]))
left_purlin_edge_support_center_x_mm = float(edge_rafters["left"]["axis_start"]["x"])
right_purlin_edge_support_center_x_mm = float(edge_rafters["right"]["axis_start"]["x"])
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
    "member_end": None,
    "reference": None,
}

for side, members in corner_purlins_by_side.items():
    for member_obj in members:
        member_id = member_obj["id"]
        inner_conn = first_connection_matching(
            member_id,
            lambda _conn, conn_members: any(other.startswith(INTERIOR_RAFTER_PREFIX) and other.split(".")[-1].isdigit() for other in conn_members if other != member_id),
        )
        corner_purlin_inner_support_connections[member_id] = inner_conn
        corner_purlin_inner_support_points[member_id] = dict(inner_conn["at"]) if inner_conn is not None else dict(member_obj["axis_start"])
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
        if outer_support_member_id == "beam.outer":
            corner_purlin_outer_support_points[member_id] = {
                "x": float(outer_conn["at"]["x"]),
                "y": float(outer_conn["at"]["y"]) + outer_beam_b_mm / 2.0,
                "z": float(outer_conn["at"]["z"]),
            }
        else:
            corner_purlin_outer_support_points[member_id] = dict(outer_conn["at"])
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
panel_mass_kg = float(panel_count["unit_mass_kg"])
panel_count_total = int(panel_count["nx"]) * int(panel_count["ny"])
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
gamma_gl30c = 5.0

rafter_self_kNm = (rafter_b_mm / 1000.0) * (rafter_h_mm / 1000.0) * gamma_c24 / math.cos(roof_slope_rad)
purlin_self_kNm = (purlin_b_mm / 1000.0) * (purlin_h_mm / 1000.0) * gamma_c24
outer_beam_self_kNm = (outer_beam_b_mm / 1000.0) * (outer_beam_h_mm / 1000.0) * gamma_gl30c
inner_beam_self_kNm = (inner_beam_b_mm / 1000.0) * (inner_beam_h_mm / 1000.0) * gamma_gl30c

E_c24 = 11000.0
E_gl30c = 13000.0
kmod_c24 = 0.8
gammaM_c24 = 1.3
fm_d_c24 = kmod_c24 * 24.0 / gammaM_c24
fv_d_c24 = kmod_c24 * 4.0 / gammaM_c24
kmod_gl30c = 0.8
fm_d_gl30c = kmod_gl30c * 30.0 / 1.25
fv_d_gl30c = kmod_gl30c * 3.5 / 1.25

gammaG = 1.35
gammaQ = 1.50
psi0_W = 0.6
psi0_snow = 0.7

def ec5_rotational_spring_k_Nmm_per_rad(fastener_d_mm, fastener_count, effective_height_mm):
    kser_per_fastener_N_per_mm = rho_c24**1.5 * fastener_d_mm / 23.0
    return fastener_count * kser_per_fastener_N_per_mm * effective_height_mm**2 / 12.0


# Sisäpään puolijäykkä liitos: palkkikenkä N 48×136, 5.0×40 ankkuriruuvit, täyskiinnitys.
# Likimalli:
#   - EN 1995-1-1 taulukko 7.1: Kser = rho_m^1.5 * d / 23  (d <= 6 mm)
#   - 24 kpl Ø5-reikiä oletetaan täyskiinnitetyiksi
#   - ruuvit mallinnetaan korkeussuunnassa tasaisesti 136 mm matkalle
#     -> rotaatiojousi k_theta = Σ(Kser_i * y_i^2) ≈ n * Kser * h^2 / 12
inner_hanger_name = "Palkkikenkä N 48x136"
inner_hanger_fastener = "5.0x40 ankkuriruuvi"
inner_hanger_fastener_count = 24
inner_hanger_height_mm = 136.0
inner_hanger_rot_k_Nmm_per_rad = ec5_rotational_spring_k_Nmm_per_rad(5.0, inner_hanger_fastener_count, inner_hanger_height_mm)

# Reunimmaiset kattotuolit oletetaan N-kiinnikkeillä molemmista päistä.
edge_rafter_support_name = inner_hanger_name
edge_rafter_support_fastener = inner_hanger_fastener
edge_rafter_support_rot_k_Nmm_per_rad = inner_hanger_rot_k_Nmm_per_rad

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


# ── jäsenanalyysit ───────────────────────────────────────────────────────────

def combined_section_h(full_h_mm, depth_functions):
    def section_h(coord_mm):
        return max(1e-9, full_h_mm - sum(max(0.0, fn(coord_mm)) for fn in depth_functions))
    return section_h


def make_end_referenced_bevel_notch_depth_fn(info, end_coord_mm, inward_positive_sign):
    if not info["active"]:
        def depth_fn(_coord_mm):
            return 0.0
        return (end_coord_mm, end_coord_mm), depth_fn, False

    if info["reference"] != "member_end":
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
    info = purlin_inner_notch_info[side]
    if not info["active"]:
        def depth_fn(_x_mm):
            return 0.0
        edge_x_mm = float(member_obj["axis_start"]["x"])
        return (edge_x_mm, edge_x_mm), depth_fn, False

    start_x_mm = float(member_obj["axis_start"]["x"])
    end_x_mm = float(member_obj["axis_end"]["x"])
    member_axis_positive_sign = 1.0 if end_x_mm >= start_x_mm else -1.0
    notch_end_x_mm = start_x_mm if info["member_end"] == "axis_start" else end_x_mm
    inward_positive_sign = member_axis_positive_sign if info["member_end"] == "axis_start" else -member_axis_positive_sign
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

    notch_end_s_mm = 0.0 if info["member_end"] == "axis_start" else member_length_mm
    inward_positive_sign = 1.0 if info["member_end"] == "axis_start" else -1.0
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


RAfter_PROPS = member_rect_props(rafter_b_mm, rafter_h_mm)
PURLIN_PROPS = member_rect_props(purlin_b_mm, purlin_h_mm)
OUTER_BEAM_PROPS = member_rect_props(outer_beam_b_mm, outer_beam_h_mm)
INNER_BEAM_PROPS = member_rect_props(inner_beam_b_mm, inner_beam_h_mm)

RAfter_MRd_kNm = fm_d_c24 * RAfter_PROPS["W_mm3"] / 1.0e6
RAfter_VRd_kN = fv_d_c24 * RAfter_PROPS["A_mm2"] / 1.5e3
PURLIN_MRd_kNm = fm_d_c24 * PURLIN_PROPS["W_mm3"] / 1.0e6
PURLIN_VRd_kN = fv_d_c24 * PURLIN_PROPS["A_mm2"] / 1.5e3
OUTER_BEAM_MRd_y_kNm = fm_d_gl30c * OUTER_BEAM_PROPS["W_mm3"] / 1.0e6
OUTER_BEAM_VRd_kN = fv_d_gl30c * OUTER_BEAM_PROPS["A_mm2"] / 1.5e3
OUTER_BEAM_MRd_z_kNm = fm_d_gl30c * (outer_beam_h_mm * outer_beam_b_mm**2 / 6.0) / 1.0e6
INNER_BEAM_MRd_kNm = fm_d_gl30c * INNER_BEAM_PROPS["W_mm3"] / 1.0e6
INNER_BEAM_VRd_kN = fv_d_gl30c * INNER_BEAM_PROPS["A_mm2"] / 1.5e3

edge_rafter_section_by_side = {}
for side, member_obj in edge_rafters.items():
    b_mm = profile_b(member_obj)
    h_mm = profile_h(member_obj)
    props = member_rect_props(b_mm, h_mm)
    edge_rafter_section_by_side[side] = {
        "profile": member_obj["profile"]["name"],
        "b_mm": b_mm,
        "h_mm": h_mm,
        "self_kNm": (b_mm / 1000.0) * (h_mm / 1000.0) * gamma_gl30c / math.cos(roof_slope_rad),
        "MRd_kNm": fm_d_gl30c * props["W_mm3"] / 1.0e6,
        "VRd_kN": fv_d_gl30c * props["A_mm2"] / 1.5e3,
    }

left_purlin_edge_support_line_x_mm = left_purlin_edge_support_center_x_mm - edge_rafter_section_by_side["left"]["b_mm"] / 2.0
right_purlin_edge_support_line_x_mm = right_purlin_edge_support_center_x_mm + edge_rafter_section_by_side["right"]["b_mm"] / 2.0


analysis_step_member_mm = 100.0
analysis_step_beam_mm = 150.0


def solve_member_response(
    nodes_mm,
    supports_mm,
    point_loads_kN,
    uniform_loads_kN_per_mm,
    EI_Nmm2=None,
    EI_by_segment_Nmm2=None,
    rotational_springs_Nmm_per_rad=None,
):
    response = beam_solver(
        nodes_mm,
        supports_mm,
        point_loads_kN=point_loads_kN,
        uniform_loads_kN_per_mm=uniform_loads_kN_per_mm,
        EI_Nmm2=1.0 if EI_Nmm2 is None else EI_Nmm2,
        EI_by_segment_Nmm2=EI_by_segment_Nmm2,
        rotational_springs_Nmm_per_rad=rotational_springs_Nmm_per_rad,
    )
    internal = sample_internal_forces(response["elements"])
    delta = sample_max_deflection_mm(response["nodes_mm"], response["disp_mm"], response["rot_rad"], step_mm=2.0)
    return response, internal, delta


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


def analyse_purlin_case(side, index, trib_height_m, roof_area_kNm2_at, gamma_self):
    member_obj = left_purlins[index] if side == "left" else right_purlins[index]
    x0_mm = min(float(member_obj["axis_start"]["x"]), float(member_obj["axis_end"]["x"]))
    x1_mm = max(float(member_obj["axis_start"]["x"]), float(member_obj["axis_end"]["x"]))
    member_y_mm = float(member_obj["axis_start"]["y"])
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

    node_points = [x0_mm, x1_mm, *supports, *inner_notch_zone_mm]
    if edge_notch_active:
        node_points.extend(edge_notch_zone_mm)
    outer_edge_x_mm = x0_mm if side == "left" else x1_mm
    panel_load_x_mm = outer_edge_x_mm + panel_frame_edge_offset_mm if side == "left" else outer_edge_x_mm - panel_frame_edge_offset_mm
    node_points.append(panel_load_x_mm)
    nodes_mm = refine_nodes_mm(node_points, analysis_step_member_mm)
    roof_uniform = roof_area_uniform_loads(nodes_mm, member_y_mm, roof_area_kNm2_at, trib_height_m, axis="x")
    self_uniform = uniform_loads_for_nodes(nodes_mm, gamma_self * purlin_self_kNm / 1000.0)
    q_line_stats = load_stats(combine_uniform_loads(roof_uniform, self_uniform))
    uniform = self_uniform
    panel_point_load_kN = total_uniform_load_kN(roof_uniform)
    point_loads = [(panel_load_x_mm, panel_point_load_kN)]
    panel_load_mode = "outer_edge_point"
    EI_by_segment = [E_c24 * purlin_b_mm * section_h_fn(0.5 * (a_mm + b_mm)) ** 3 / 12.0 for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:])]

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
            section_h_mm_at_x=section_h_fn,
            b_mm=purlin_b_mm,
            fm_d_Nmm2=fm_d_c24,
            fv_d_Nmm2=fv_d_c24,
            x_start_mm=inner_notch_zone_mm[0],
            x_end_mm=inner_notch_zone_mm[1],
            step_mm=1.0,
        )
        notch_candidates.append({"label": f"{purlin_inner_notch_info[side]['member_end']}_bevel", **inner_notch})
    if edge_notch_active:
        edge_notch = sample_net_section_utilization(
            response["elements"],
            section_h_mm_at_x=section_h_fn,
            b_mm=purlin_b_mm,
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
        "R_edge_kN": response["reactions_kN"][edge_support_x_mm],
        "R_inner_kN": response["reactions_kN"][interior_support_x_mm],
        "eta_M": moment_gov["value_kNm"] / PURLIN_MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / PURLIN_VRd_kN * 100.0,
        "notch": notch,
        "inner_notch": inner_notch,
        "edge_notch": edge_notch,
        "notch_zones_mm": {"inner": inner_notch_zone_mm, "edge": edge_notch_zone_mm if edge_notch_active else None},
        "h_net_min_mm": purlin_h_mm - max(purlin_inner_notch_info[side]["depth_mm"], purlin_edge_notch_info[side]["depth_mm"]),
    }


def analyse_corner_purlin_case(side, member_obj, trib_width_m, roof_area_kNm2_at, gamma_self):
    member_id = member_obj["id"]
    member_length_mm = member_axis_length_mm(member_obj)
    inner_conn = corner_purlin_inner_support_connections[member_id]
    support_inner_point = corner_purlin_inner_support_points[member_id]
    support_outer_point = corner_purlin_outer_support_points[member_id]
    support_inner_s_mm = project_point_to_member_s_mm(member_obj, support_inner_point)
    support_outer_s_mm = project_point_to_member_s_mm(member_obj, support_outer_point)
    outer_end_s_mm = 0.0 if support_outer_s_mm <= member_length_mm - support_outer_s_mm else member_length_mm
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
    node_points = [0.0, member_length_mm, support_inner_s_mm, support_outer_s_mm]
    if inner_notch_active:
        node_points.extend(inner_notch_zone_mm)
    if outer_notch_active:
        node_points.extend(outer_notch_zone_mm)
    panel_load_s_mm = clamp(
        outer_end_s_mm + (panel_frame_edge_offset_mm if outer_end_s_mm <= 1e-9 else -panel_frame_edge_offset_mm),
        0.0,
        member_length_mm,
    )
    node_points.append(panel_load_s_mm)
    nodes_mm = refine_nodes_mm(node_points, analysis_step_member_mm)
    roof_uniform = roof_area_uniform_loads_on_member(nodes_mm, member_obj, roof_area_kNm2_at, trib_width_m)
    self_uniform = uniform_loads_for_nodes(nodes_mm, gamma_self * purlin_self_kNm / 1000.0)
    q_line_stats = load_stats(combine_uniform_loads(roof_uniform, self_uniform))
    point_loads = [(panel_load_s_mm, total_uniform_load_kN(roof_uniform))]
    uniform = self_uniform
    response, internal, delta = solve_member_response(
        nodes_mm,
        [support_inner_s_mm, support_outer_s_mm],
        point_loads,
        uniform,
        EI_by_segment_Nmm2=[E_c24 * purlin_b_mm * section_h_fn(0.5 * (a_mm + b_mm)) ** 3 / 12.0 for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:])],
    )
    moment_gov = governing_moment(internal)
    reactions = response["reactions_kN"]
    inner_notch = None
    outer_notch = None
    notch_candidates = []
    if inner_notch_active:
        inner_notch = sample_net_section_utilization(
            response["elements"],
            section_h_mm_at_x=section_h_fn,
            b_mm=purlin_b_mm,
            fm_d_Nmm2=fm_d_c24,
            fv_d_Nmm2=fv_d_c24,
            x_start_mm=inner_notch_zone_mm[0],
            x_end_mm=inner_notch_zone_mm[1],
            step_mm=1.0,
        )
        notch_candidates.append({"label": f"{inner_notch_info['member_end']}_bevel", **inner_notch})
    if outer_notch_active:
        outer_notch = sample_net_section_utilization(
            response["elements"],
            section_h_mm_at_x=section_h_fn,
            b_mm=purlin_b_mm,
            fm_d_Nmm2=fm_d_c24,
            fv_d_Nmm2=fv_d_c24,
            x_start_mm=outer_notch_zone_mm[0],
            x_end_mm=outer_notch_zone_mm[1],
            step_mm=1.0,
        )
        notch_candidates.append({"label": f"{corner_purlin_outer_notch_info[member_id]['member_end']}_bevel", **outer_notch})
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
        "panel_load_mode": "outer_edge_point",
        "panel_point_load_kN": point_loads[0][1],
        "M_gov": moment_gov,
        "V_abs": internal["V_abs"],
        "delta": delta,
        "delta_lim_mm": abs(outer_end_s_mm - support_outer_s_mm) / 300.0,
        "support_inner_s_mm": support_inner_s_mm,
        "support_outer_s_mm": support_outer_s_mm,
        "support_inner_y_mm": float(support_inner_point["y"]),
        "support_outer_y_mm": float(support_outer_point["y"]),
        "support_outer_x_mm": float(support_outer_point["x"]),
        "inner_support_member_id": corner_purlin_inner_support_member_ids[member_id],
        "outer_support_member_id": corner_purlin_outer_support_member_ids[member_id],
        "outer_support_label": corner_purlin_outer_support_member_ids[member_id],
        "R_inner_kN": reactions[support_inner_s_mm],
        "R_outer_kN": reactions[support_outer_s_mm],
        "R_outer_beam_kN": reactions[support_outer_s_mm],
        "eta_M": moment_gov["value_kNm"] / PURLIN_MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / PURLIN_VRd_kN * 100.0,
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
    EI_by_segment = [member_E_Nmm2 * member_b_mm * section_h_fn(0.5 * (a_mm + b_mm)) ** 3 / 12.0 for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:])]
    rotational_springs = {inner_support_y_mm: inner_hanger_rot_k_Nmm_per_rad}
    if edge:
        rotational_springs = {
            inner_support_y_mm: edge_rafter_support_rot_k_Nmm_per_rad,
            outer_support_y_mm: edge_rafter_support_rot_k_Nmm_per_rad,
        }
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
            section_h_mm_at_x=section_h_fn,
            b_mm=member_b_mm,
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
                section_h_mm_at_x=section_h_fn,
                b_mm=member_b_mm,
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


def analyse_beam_case(member_obj, support_xs_mm, point_loads_kN, gamma_self, E_Nmm2, section_I_mm4, MRd_kNm, VRd_kN):
    x0_mm = float(member_obj["axis_start"]["x"])
    x1_mm = float(member_obj["axis_end"]["x"])
    if member_obj["id"] == "beam.outer":
        self_kNm = outer_beam_self_kNm
    else:
        self_kNm = inner_beam_self_kNm
    nodes_mm = refine_nodes_mm([x0_mm, x1_mm, *support_xs_mm, *[x_mm for x_mm, _ in point_loads_kN]], analysis_step_beam_mm)
    uniform = uniform_loads_for_nodes(nodes_mm, gamma_self * self_kNm / 1000.0)
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
    response, internal, _ = solve_member_response(nodes_mm, outer_supports_x_mm, [], uniform, EI_Nmm2=E_gl30c * OUTER_BEAM_PROPS["I_weak_mm4"])
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

    outer_beam_result = analyse_beam_case(
        outer_beam,
        outer_supports_x_mm,
        sorted(outer_beam_point_loads, key=lambda item: item[0]),
        gamma_self,
        E_gl30c,
        OUTER_BEAM_PROPS["I_strong_mm4"],
        OUTER_BEAM_MRd_y_kNm,
        OUTER_BEAM_VRd_kN,
    )
    inner_beam_result = analyse_beam_case(
        inner_beam,
        inner_supports_x_mm,
        sorted(inner_beam_point_loads, key=lambda item: item[0]),
        gamma_self,
        E_gl30c,
        INNER_BEAM_PROPS["I_strong_mm4"],
        INNER_BEAM_MRd_kNm,
        INNER_BEAM_VRd_kN,
    )

    return {
        "case": case,
        "purlins": purlins,
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
print(f"  Reunakattotuolien sisatuki    y = {edge_inner_support_y_mm:.0f} mm")
print(f"  Sisäpään liitos               {inner_hanger_name}, {inner_hanger_fastener}, kθ ≈ {inner_hanger_rot_k_Nmm_per_rad/1.0e6:.1f} kNm/rad")
print(f"  Reunakattotuolien liitokset   {edge_rafter_support_name} molemmissa päissä, kθ ≈ {edge_rafter_support_rot_k_Nmm_per_rad/1.0e6:.1f} kNm/rad")
print("  Orsien liitokset              nivelletyt tuet")
print(f"  Ulkopalkin tuet               x = " + " / ".join(f"{x_mm:.0f}" for x_mm in outer_supports_x_mm) + " mm")
print(f"  Sisäpalkin tuet               x = " + " / ".join(f"{x_mm:.0f}" for x_mm in inner_supports_x_mm) + " mm")

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

print("\n── ORRET X-SUUNNASSA 98×48 C24 ──────────────────────────────────")
print(f"  Sivukaista vasen / oikea      {left_strip_width_m:.3f} / {right_strip_width_m:.3f} m")
print(
    f"  Tributäärikorkeudet           vasen: {' / '.join(f'{h_m:.3f}' for h_m in purlin_trib_heights_m['left'])} m"
    f" | oikea: {' / '.join(f'{h_m:.3f}' for h_m in purlin_trib_heights_m['right'])} m"
)
print(f"  Paneelikehikko                kuorma pisteena {panel_frame_edge_offset_mm:.0f} mm ulkoreunasta; omapaino viivakuormana")
print(f"  MRd = {PURLIN_MRd_kNm:.2f} kNm,  VRd = {PURLIN_VRd_kN:.2f} kN,  δ_lim = {(left_purlin_support_x_mm-left_purlin_edge_support_line_x_mm)/300.0:.1f} mm")
print(f"  Paatybevel ulokepaassa        bevel_bottom_notch {purlin_notch_depth_mm:.0f} × {purlin_notch_length_mm:.0f} mm")
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
    print(f"  Paneelikuorma                 paatypistekuormana {panel_frame_edge_offset_mm:.0f} mm ulkoreunasta; omapaino viivakuormana")
    slanted_notch_desc = []
    slanted_rafter_notch = next(
        (
            bevel_notch_info(conn["id"])
            for conn in corner_purlin_inner_support_connections.values()
            if conn is not None and bevel_notch_info(conn["id"])["active"]
        ),
        None,
    )
    if slanted_rafter_notch is None:
        slanted_rafter_notch = next(
            (
                info
                for member_id, info in corner_purlin_outer_notch_info.items()
                if info["active"] and corner_purlin_outer_support_member_ids[member_id].startswith(INTERIOR_RAFTER_PREFIX)
            ),
            None,
        )
    if slanted_rafter_notch is not None:
        slanted_notch_desc.append(
            f"rafter-tuet bevel_bottom_notch {slanted_rafter_notch['depth_mm']:.0f} × {slanted_rafter_notch['length_mm']:.0f} mm"
        )
    slanted_beam_notch = next(
        (
            info
            for member_id, info in corner_purlin_outer_notch_info.items()
            if info["active"] and corner_purlin_outer_support_member_ids[member_id] == "beam.outer"
        ),
        None,
    )
    if slanted_beam_notch is not None:
        slanted_notch_desc.append(
            f"beam.outer bevel_bottom_notch {slanted_beam_notch['depth_mm']:.0f} × {slanted_beam_notch['length_mm']:.0f} mm"
        )
    if slanted_notch_desc:
        print(
        f"  Paatybevelit vinoissa orsissa "
            + "; ".join(slanted_notch_desc)
            + f", h_net,min = {critical_corner_purlin['h_net_min_mm']:.0f} mm"
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

print("\n── HUOMIOT ───────────────────────────────────────────────────────")
print("  * Reunakattotuoleille ei anneta suoraa paneelikaistan hajakuormaa; kuorma siirtyy")
print(f"    geometry/terassi_puu.json:n mukaisten orsien kautta paatypistekuormina {panel_frame_edge_offset_mm:.0f} mm")
print("    ulkoreunasta; orren oma paino mallinnetaan viivakuormana.")
print("  * Ulkokulmien nurkkaorret mallinnetaan tuettuina toiseksi uloimpaan kattotuoliin ja")
print("    ulkopalkin ulkoreunalta kantavina ulokkeina; reunatuki voi siksi olla vetava.")
print("  * Orsien kaikki tuet mallinnetaan puolijaykkina EC5/Kser-likimallin rotaatiojousina")
print("    (oletus: 4 x 5.0x80 puuruuvi per tuki, ruuvit jakautuvat 98 mm korkeudelle).")
print("  * Orsien ja kattotuolien lovi-/nettoh-tarkistukset luetaan suoraan geometry/terassi_puu.json:n")
print("    cuts-kentistä; terassivertailuskriptin hardkoodattuja loviarvoja ei käytetä.")
print("  * Kinostuma mallinnetaan jäsenkohtaisena muuttuvana s(x,y)-kuormana; taulukon q_avg")
print("    on roof-stripin ekvivalentin viivakuorman pituuspainotettu keskiarvo ennen kehikkosiirtoa.")
print("  * Birdsmouth-loven seat-pituus 570 mm tekee nettoh:n nousun lineaariseksi lovivyöhykkeellä;")
print("    tarkistus tehdään koko loven pituudella, ei vain yhdessä poikkileikkauksessa.")
print("  * Sisäpään liitos mallinnetaan heikosti kiertymää jäykistävänä rotaatiojousena")
print("    (palkkikenkä N 48x136, 5.0x40 ankkuriruuvit, täyskiinnitys; EC5 Kser-likimalli).")
print("  * Reunarafterien molemmat päät mallinnetaan puolijaykkinä samoilla N-kiinnikejousilla.")
print("  * Uplift käyttää suljetun lasituksen imutapausta (w_up_closed); kiinnitykset tulee mitoittaa")
print("    vähintään yllä raportoiduille nostoreaktioille.")
print(DW)
