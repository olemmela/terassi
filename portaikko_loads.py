"""Shared load-path helpers for geometry/portaikko.json."""

from functools import lru_cache

from beam_analysis import (
    beam_solver,
    combine_uniform_loads,
    intervals_to_uniform_loads,
    sample_internal_forces,
    uniform_loads_for_nodes,
)
from foundation_checks import foundation_checks_from_envelope
from geometry_loader import member, profile_b
from structural_geometry import (
    column_self_weight_kN,
    connection_by_members,
    distributed_interval_on_member_axis_mm,
    member_axis_length_mm,
    member_point_at_s_mm,
    member_s_at_axis_value_mm,
    member_uniform_self_weight_kNm,
)
from terrace_column_loads import calculate_katos_total_column_loads


def existing_structure_case_key(case_key):
    if case_key == "UPLIFT":
        return "UPLIFT"
    if case_key.startswith("SLS"):
        return "SLS"
    return "ULS"


def _solve_y_member_reactions(member_obj, support_conns, uniform_kNm):
    support_s_mm = [
        member_s_at_axis_value_mm(member_obj, "y", conn["at"]["y"])
        for conn in support_conns
    ]
    nodes_mm = sorted(
        {
            0.0,
            member_axis_length_mm(member_obj),
            *support_s_mm,
        }
    )
    response = beam_solver(
        nodes_mm,
        support_s_mm,
        uniform_loads_kN_per_mm=uniform_loads_for_nodes(nodes_mm, uniform_kNm / 1000.0),
    )
    internal = sample_internal_forces(response["elements"])
    return response, internal, support_s_mm


def _add_distributed_reaction(loads, support_member_obj, source_id, center_x_mm, width_mm, reaction_kN):
    start_s_mm, end_s_mm = distributed_interval_on_member_axis_mm(support_member_obj, "x", center_x_mm, width_mm)
    interval_length_mm = end_s_mm - start_s_mm
    start_point = member_point_at_s_mm(support_member_obj, start_s_mm)
    end_point = member_point_at_s_mm(support_member_obj, end_s_mm)
    loads.append(
        {
            "source_id": source_id,
            "start_s_mm": start_s_mm,
            "end_s_mm": end_s_mm,
            "start_x_mm": start_point["x"],
            "end_x_mm": end_point["x"],
            "width_mm": interval_length_mm,
            "q_kN_per_mm": reaction_kN / interval_length_mm,
            "total_kN": reaction_kN,
        }
    )


def calculate_portaikko_foundation_loads(
    geo,
    roof_results,
    beam_support_xs_mm,
    down_case_keys,
    gammaG,
    gammaQ,
    gamma_GL30c,
    gamma_concrete=25.0,
    gk_hollow_slab=2.37,
    qk_floor_live=2.5,
):
    """Calculate entrance concrete load path and stair-foundation envelopes."""

    concrete_beam = member(geo, "beams", "beam.entrance")
    concrete_column_x9100 = member(geo, "columns", "col.x9100")
    timber_post_lp1 = member(geo, "columns", "col.lp1")
    concrete_beam_supports = sorted(
        (
            conn["members"][1],
            float(conn["at"]["x"]),
        )
        for conn in geo["connections"]
        if conn["id"].startswith("con.beam.entrance.on.col")
    )
    concrete_beam_support_xs_mm = [x_mm for _, x_mm in concrete_beam_supports]
    concrete_beam_support_by_column = {column_id: x_mm for column_id, x_mm in concrete_beam_supports}
    hollow_slab_ids = ["beam.hollow.entrance.0", "beam.hollow.entrance.1"]
    stair_beam_id = "beam.stairs"
    floor_case_defs = {
        "ULS": {"gamma_g": gammaG, "gamma_q": gammaQ},
        "SLS": {"gamma_g": 1.0, "gamma_q": 1.0},
        "UPLIFT": {"gamma_g": 0.9, "gamma_q": 0.0},
    }

    def solve_floor_group(group_key):
        factors = floor_case_defs[group_key]
        gamma_g = factors["gamma_g"]
        gamma_q = factors["gamma_q"]
        point_loads_on_concrete_beam = []
        distributed_loads_on_concrete_beam = []
        slab_rows = []

        for slab_id in hollow_slab_ids:
            slab = member(geo, "beams", slab_id)
            wall_conn = connection_by_members(geo, slab_id, "ref.house_wall_entrance")
            beam_conn = connection_by_members(geo, slab_id, "beam.entrance")
            q_line_kNm = (gamma_g * gk_hollow_slab + gamma_q * qk_floor_live) * profile_b(slab) / 1000.0
            response, internal, support_s_mm = _solve_y_member_reactions(
                slab,
                [wall_conn, beam_conn],
                q_line_kNm,
            )
            wall_reaction_kN = response["reactions_kN"][support_s_mm[0]]
            beam_reaction_kN = response["reactions_kN"][support_s_mm[1]]
            distribution = beam_conn.get("analysis", {}).get("reaction_distribution", {"type": "point"})
            if distribution.get("type") == "uniform_over_supported_member_width":
                _add_distributed_reaction(
                    distributed_loads_on_concrete_beam,
                    concrete_beam,
                    slab_id,
                    float(beam_conn["at"]["x"]),
                    profile_b(slab),
                    beam_reaction_kN,
                )
            else:
                point_loads_on_concrete_beam.append((float(beam_conn["at"]["x"]), beam_reaction_kN))
            slab_rows.append(
                {
                    "id": slab_id,
                    "width_m": profile_b(slab) / 1000.0,
                    "q_line_kNm": q_line_kNm,
                    "wall_reaction_kN": wall_reaction_kN,
                    "beam_reaction_kN": beam_reaction_kN,
                    "M_pos": internal["M_pos"],
                    "M_neg": internal["M_neg"],
                    "V_abs": internal["V_abs"],
                }
            )

        stair_beam = member(geo, "beams", stair_beam_id)
        stair_wall_conn = connection_by_members(geo, stair_beam_id, "ref.house_wall_entrance")
        stair_concrete_conn = connection_by_members(geo, stair_beam_id, "beam.entrance")
        stair_self_kNm = gamma_g * member_uniform_self_weight_kNm(stair_beam, gamma_concrete)
        stair_response, stair_internal, stair_support_s_mm = _solve_y_member_reactions(
            stair_beam,
            [stair_wall_conn, stair_concrete_conn],
            stair_self_kNm,
        )
        stair_wall_reaction_kN = stair_response["reactions_kN"][stair_support_s_mm[0]]
        stair_beam_reaction_kN = stair_response["reactions_kN"][stair_support_s_mm[1]]
        point_loads_on_concrete_beam.append((float(stair_concrete_conn["at"]["x"]), stair_beam_reaction_kN))

        point_loads_local = [
            (member_s_at_axis_value_mm(concrete_beam, "x", x_mm), load_kN)
            for x_mm, load_kN in point_loads_on_concrete_beam
        ]
        distributed_intervals = [
            (load["start_s_mm"], load["end_s_mm"], load["q_kN_per_mm"])
            for load in distributed_loads_on_concrete_beam
        ]
        concrete_beam_nodes = sorted(
            {
                0.0,
                member_axis_length_mm(concrete_beam),
                *[member_s_at_axis_value_mm(concrete_beam, "x", x_mm) for x_mm in concrete_beam_support_xs_mm],
                *[x_mm for x_mm, _ in point_loads_local],
                *[x_mm for interval in distributed_intervals for x_mm in interval[:2]],
            }
        )
        concrete_beam_self = gamma_g * member_uniform_self_weight_kNm(concrete_beam, gamma_concrete)
        concrete_beam_uniform = combine_uniform_loads(
            uniform_loads_for_nodes(concrete_beam_nodes, concrete_beam_self / 1000.0),
            intervals_to_uniform_loads(concrete_beam_nodes, distributed_intervals),
        )
        concrete_beam_support_s_mm = [
            member_s_at_axis_value_mm(concrete_beam, "x", x_mm)
            for x_mm in concrete_beam_support_xs_mm
        ]
        concrete_response = beam_solver(
            concrete_beam_nodes,
            concrete_beam_support_s_mm,
            point_loads_kN=point_loads_local,
            uniform_loads_kN_per_mm=concrete_beam_uniform,
        )
        concrete_internal = sample_internal_forces(concrete_response["elements"])
        concrete_reactions_by_column = {
            column_id: concrete_response["reactions_kN"][member_s_at_axis_value_mm(concrete_beam, "x", x_mm)]
            for column_id, x_mm in concrete_beam_support_by_column.items()
        }
        return {
            "key": group_key,
            "slabs": slab_rows,
            "stair_beam": {
                "q_line_kNm": stair_self_kNm,
                "wall_reaction_kN": stair_wall_reaction_kN,
                "beam_reaction_kN": stair_beam_reaction_kN,
                "M_pos": stair_internal["M_pos"],
                "M_neg": stair_internal["M_neg"],
                "V_abs": stair_internal["V_abs"],
            },
            "concrete_beam": {
                "self_kNm": concrete_beam_self,
                "point_loads_kN": point_loads_on_concrete_beam,
                "distributed_loads": distributed_loads_on_concrete_beam,
                "reactions_by_column_id_kN": concrete_reactions_by_column,
                "M_pos": concrete_internal["M_pos"],
                "M_neg": concrete_internal["M_neg"],
                "V_abs": concrete_internal["V_abs"],
            },
        }

    floor_results = {group_key: solve_floor_group(group_key) for group_key in floor_case_defs}
    katos_case_totals = calculate_katos_total_column_loads()["case_totals"]
    foundation_column_ids = [foundation["supports"] for foundation in geo.get("foundations", [])]
    foundation_case_keys = ("SLS", "SLS DRIFT", "ULS A", "ULS B", "ULS DRIFT", "ULS MAINT", "UPLIFT")

    existing_column_extra_loads_by_case = {}
    foundation_case_totals = {}
    for case_key in foundation_case_keys:
        floor_group = existing_structure_case_key(case_key)
        floor_row = floor_results[floor_group]
        gamma_g_foundation = floor_case_defs[floor_group]["gamma_g"]
        existing_column_extra_loads_by_case[case_key] = {
            "col.x7075": (
                roof_results[case_key]["beam"]["reactions_kN"][beam_support_xs_mm[0]]
                + floor_row["concrete_beam"]["reactions_by_column_id_kN"]["col.x7075"]
            )
        }
        row = {column_id: 0.0 for column_id in foundation_column_ids}
        if "col.x7075" in row:
            row["col.x7075"] += katos_case_totals[existing_structure_case_key(case_key)]["col.x7075"]
            row["col.x7075"] += existing_column_extra_loads_by_case[case_key]["col.x7075"]
        if "col.x9100" in row:
            row["col.x9100"] += roof_results[case_key]["beam"]["reactions_kN"][beam_support_xs_mm[1]]
            row["col.x9100"] += floor_row["concrete_beam"]["reactions_by_column_id_kN"]["col.x9100"]
            row["col.x9100"] += column_self_weight_kN(concrete_column_x9100, gamma_concrete, factor=gamma_g_foundation)
            row["col.x9100"] += column_self_weight_kN(timber_post_lp1, gamma_GL30c, factor=gamma_g_foundation)
        foundation_case_totals[case_key] = row

    foundation_envelope = {}
    for column_id in foundation_column_ids:
        sls_case = max(("SLS", "SLS DRIFT"), key=lambda key: foundation_case_totals[key][column_id])
        uls_case = max(down_case_keys, key=lambda key: foundation_case_totals[key][column_id])
        foundation_envelope[column_id] = {
            "N_sls": foundation_case_totals[sls_case][column_id],
            "sls_case_key": sls_case,
            "N_uls": foundation_case_totals[uls_case][column_id],
            "uls_case_key": uls_case,
            "N_min": foundation_case_totals["UPLIFT"][column_id],
            "uplift_case_key": "UPLIFT",
        }

    return {
        "gk_hollow_slab": gk_hollow_slab,
        "qk_floor_live": qk_floor_live,
        "concrete_gamma": gamma_concrete,
        "floor_results": floor_results,
        "case_totals": foundation_case_totals,
        "envelope": foundation_envelope,
        "checks": foundation_checks_from_envelope(geo, foundation_envelope),
        "existing_column_extra_loads_by_case": existing_column_extra_loads_by_case,
        "concrete_beam_supports": concrete_beam_supports,
        "hollow_edge_support_modeled": any(
            set(conn.get("members", [])) == {"beam.hollow.entrance.0", "ref.house_wall_corner"}
            for conn in geo.get("connections", [])
        ),
    }


@lru_cache(maxsize=1)
def _portaikko_analysis():
    from portaikko_kuormituslaskenta import analyse

    return analyse()


def existing_column_extra_loads_by_case(case_keys):
    """Return stair/entrance loads added to existing columns in geometry/katos.json."""

    data = _portaikko_analysis()
    extras = data["foundation"]["existing_column_extra_loads_by_case"]
    result = {}
    for case_key in case_keys:
        if case_key in extras:
            result[case_key] = dict(extras[case_key])
        elif case_key == "ULS":
            column_ids = sorted({column_id for key in data["down_case_keys"] for column_id in extras[key]})
            result[case_key] = {
                column_id: max(extras[key].get(column_id, 0.0) for key in data["down_case_keys"])
                for column_id in column_ids
            }
        else:
            raise KeyError(f"Unknown portaikko load case: {case_key}")
    return result
