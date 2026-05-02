"""Shared beam-analysis helpers for the calculator scripts."""

import math


def solve_linear_system(A, b):
    """Solve a small dense linear system with Gaussian elimination."""
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
    """Split long member intervals for FE deflection calculations."""
    pts = sorted(set(float(x_mm) for x_mm in points_mm))
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
    fixed_rotations_mm=None,
    rotational_springs_Nmm_per_rad=None,
):
    """Euler-Bernoulli beam network with point and uniform element loads."""
    if point_loads_kN is None:
        point_loads_kN = []
    if uniform_loads_kN_per_mm is None:
        uniform_loads_kN_per_mm = []
    if fixed_rotations_mm is None:
        fixed_rotations_mm = []
    if rotational_springs_Nmm_per_rad is None:
        rotational_springs_Nmm_per_rad = {}

    nodes = sorted(
        {float(x_mm) for x_mm in nodes_mm}
        | {float(x_mm) for x_mm in supports_mm}
        | {float(x_mm) for x_mm in fixed_rotations_mm}
        | {float(x_mm) for x_mm, _ in point_loads_kN}
        | {float(a_mm) for a_mm, _, _ in uniform_loads_kN_per_mm}
        | {float(b_mm) for _, b_mm, _ in uniform_loads_kN_per_mm}
    )
    n_nodes = len(nodes)
    node_index = {x_mm: i for i, x_mm in enumerate(nodes)}
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

        element_data.append(
            {
                "x0_mm": x0_mm,
                "x1_mm": x1_mm,
                "dofs": dofs,
                "k": k,
                "fe": fe,
                "q_kN_per_mm": q_kN_per_mm,
            }
        )

    for x_mm, k_theta in rotational_springs_Nmm_per_rad.items():
        x_key = float(x_mm)
        if x_key not in node_index:
            continue
        rot_dof = 2 * node_index[x_key] + 1
        K[rot_dof][rot_dof] += float(k_theta)

    fixed = sorted(
        {2 * node_index[float(x_mm)] for x_mm in supports_mm}
        | {2 * node_index[float(x_mm)] + 1 for x_mm in fixed_rotations_mm}
    )
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
        elements.append(
            {
                "x0_mm": elem["x0_mm"],
                "x1_mm": elem["x1_mm"],
                "q_kN_per_mm": elem["q_kN_per_mm"],
                "end_forces": end_forces,
            }
        )

    return {
        "nodes_mm": nodes,
        "reactions_kN": {float(x_mm): R[2 * node_index[float(x_mm)]] / 1000.0 for x_mm in supports_mm},
        "disp_mm": {x_mm: u[2 * node_index[x_mm]] for x_mm in nodes},
        "rot_rad": {x_mm: u[2 * node_index[x_mm] + 1] for x_mm in nodes},
        "support_moments_kNm": {
            float(x_mm): R[2 * node_index[float(x_mm)] + 1] / 1.0e6 for x_mm in fixed_rotations_mm
        },
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


def solve_member_response(
    nodes_mm,
    supports_mm,
    point_loads_kN=None,
    uniform_loads_kN_per_mm=None,
    EI_Nmm2=None,
    EI_by_segment_Nmm2=None,
    fixed_rotations_mm=None,
    rotational_springs_Nmm_per_rad=None,
    deflection_step_mm=2.0,
):
    """Solve a beam member and return FE response, internal forces, and max deflection."""

    response = beam_solver(
        nodes_mm,
        supports_mm,
        point_loads_kN=[] if point_loads_kN is None else point_loads_kN,
        uniform_loads_kN_per_mm=[] if uniform_loads_kN_per_mm is None else uniform_loads_kN_per_mm,
        EI_Nmm2=1.0 if EI_Nmm2 is None else EI_Nmm2,
        EI_by_segment_Nmm2=EI_by_segment_Nmm2,
        fixed_rotations_mm=fixed_rotations_mm,
        rotational_springs_Nmm_per_rad=rotational_springs_Nmm_per_rad,
    )
    internal = sample_internal_forces(response["elements"])
    delta = sample_max_deflection_mm(
        response["nodes_mm"],
        response["disp_mm"],
        response["rot_rad"],
        step_mm=deflection_step_mm,
    )
    return response, internal, delta


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
