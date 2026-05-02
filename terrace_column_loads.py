from beam_analysis import (
    beam_solver,
    combine_uniform_loads,
    intervals_to_uniform_loads,
    sample_internal_forces,
    uniform_loads_for_nodes,
)
from existing_beam_checks import (
    aggregate_point_loads,
    check_existing_lp225_x125_combined,
    katos_existing_context,
    point_beam_with_right_overhang_response,
    tributary_ranges_mm,
)
from geometry_loader import expanded_connections, load, member, surface, profile_b
from structural_geometry import (
    column_self_weight_kN,
    connection_by_members,
    distributed_reaction_interval_on_member_mm,
    member_axis_length_mm,
    member_point_at_s_mm,
    member_s_at_axis_value_mm,
    member_total_self_weight_kN,
    member_uniform_self_weight_kNm,
    project_point_to_member_s_mm,
)


GEO = load("katos.json")
CONNECTION_LIST = expanded_connections(GEO)
EXISTING_CTX = katos_existing_context()
FLOOR_CAST = surface(GEO, "surf.floor.cast")
FLOOR_CAST_POLY = FLOOR_CAST["polygon"]
FLOOR_X0_MM = min(p["x"] for p in FLOOR_CAST_POLY)
FLOOR_X1_MM = max(p["x"] for p in FLOOR_CAST_POLY)
FLOOR_Y0_MM = min(p["y"] for p in FLOOR_CAST_POLY)
FLOOR_Y1_MM = max(p["y"] for p in FLOOR_CAST_POLY)

GAMMA_CONCRETE_KNM3 = 25.0
GAMMA_G = EXISTING_CTX["gammaG"]
GAMMA_G_MIN = 0.9
GK_HOLLOW_SLAB_KNM2 = 2.37
GK_FLOOR_CAST_KNM2 = float(FLOOR_CAST.get("thickness_mm", 0.0)) / 1000.0 * GAMMA_CONCRETE_KNM3
QK_TERRACE_LIVE_KNM2 = 2.5
GK_HOLLOW_SLAB_ALLOW_KNM2 = 2.0
QK_HOLLOW_SLAB_ALLOW_KNM2 = 2.5
COLUMN_CASE_FACTORS = {"ULS": GAMMA_G, "SLS": 1.0, "UPLIFT": GAMMA_G_MIN}
FLOOR_LIVE_CASE_FACTORS = {"ULS": 1.5, "SLS": 1.0, "UPLIFT": 0.0}
COLUMN_OUTPUT_ORDER = [
    "col.x125",
    "col.x7075",
    "col.x125.outer.bottom",
    "col.x3600.outer.bottom",
    "col.x7075.outer.bottom",
]
COLUMN_DISPLAY = {
    "col.x125": "col.x125",
    "col.x7075": "col.x7075",
    "col.x125.outer.bottom": "col.x125.outer.bottom",
    "col.x3600.outer.bottom": "col.x3600.outer.bottom",
    "col.x7075.outer.bottom": "col.x7075.outer.bottom",
}
COLUMN_GROUP_LABEL = {
    "col.x125": "sisäpilari",
    "col.x7075": "sisäpilari",
    "col.x125.outer.bottom": "ulkopilari",
    "col.x3600.outer.bottom": "ulkopilari",
    "col.x7075.outer.bottom": "ulkopilari",
}


def reaction_distribution(connection_obj):
    return connection_obj.get("analysis", {}).get("reaction_distribution", {"type": "point"})


def reaction_distribution_width_mm(distribution, supported_member_obj):
    distribution_type = distribution.get("type", "point")
    if distribution_type == "point":
        return None
    if distribution_type == "uniform_over_supported_member_width":
        if distribution.get("width_ref") != "supported_member_profile_b":
            raise ValueError(f"Unsupported reaction distribution width_ref: {distribution.get('width_ref')}")
        return profile_b(supported_member_obj)
    if distribution_type == "uniform_over_width":
        return float(distribution["width_mm"])
    raise ValueError(f"Unsupported reaction distribution type: {distribution_type}")


def empty_column_loads():
    return {column_id: 0.0 for column_id in COLUMN_OUTPUT_ORDER}


def case_group_for(case_key):
    if case_key == "UPLIFT":
        return "UPLIFT"
    if case_key in {"SLS", "SLS DRIFT"}:
        return "SLS"
    return "ULS"


def calculate_katos_total_column_loads(
    case_groups=None,
    extra_upper_column_loads_by_case=None,
    extra_ground_column_loads_by_case=None,
):
    if case_groups is None:
        case_groups = {"ULS": "ULS", "SLS": "SLS", "UPLIFT": "UPLIFT"}
    if extra_upper_column_loads_by_case is None:
        extra_upper_column_loads_by_case = {}
    if extra_ground_column_loads_by_case is None:
        extra_ground_column_loads_by_case = {}

    left_inner_column = member(GEO, "columns", "col.x125")
    right_inner_column = member(GEO, "columns", "col.x7075")
    left_outer_bottom_column = member(GEO, "columns", "col.x125.outer.bottom")
    mid_outer_bottom_column = member(GEO, "columns", "col.x3600.outer.bottom")
    right_outer_bottom_column = member(GEO, "columns", "col.x7075.outer.bottom")

    roof_support_cases = {
        "ULS": {
            "kp450": {
                "left": EXISTING_CTX["kp450_y900"]["reactions_uls_kN"][EXISTING_CTX["kp450_y900"]["support_left_x_mm"]],
                "right": EXISTING_CTX["kp450_y900"]["reactions_uls_kN"][EXISTING_CTX["kp450_y900"]["support_right_x_mm"]],
            },
            "kp360": {
                "left": EXISTING_CTX["kp360"]["reactions_uls_kN"][EXISTING_CTX["kp360"]["support_left_x_mm"]],
                "right": EXISTING_CTX["kp360"]["reactions_uls_kN"][EXISTING_CTX["kp360"]["support_right_x_mm"]],
            },
        },
        "SLS": {
            "kp450": {
                "left": EXISTING_CTX["kp450_y900"]["reactions_sls_kN"][EXISTING_CTX["kp450_y900"]["support_left_x_mm"]],
                "right": EXISTING_CTX["kp450_y900"]["reactions_sls_kN"][EXISTING_CTX["kp450_y900"]["support_right_x_mm"]],
            },
            "kp360": {
                "left": EXISTING_CTX["kp360"]["reactions_sls_kN"][EXISTING_CTX["kp360"]["support_left_x_mm"]],
                "right": EXISTING_CTX["kp360"]["reactions_sls_kN"][EXISTING_CTX["kp360"]["support_right_x_mm"]],
            },
        },
        "UPLIFT": {
            "kp450": {
                "left": EXISTING_CTX["kp450_y900"]["reactions_uplift_kN"][EXISTING_CTX["kp450_y900"]["support_left_x_mm"]],
                "right": EXISTING_CTX["kp450_y900"]["reactions_uplift_kN"][EXISTING_CTX["kp450_y900"]["support_right_x_mm"]],
            },
            "kp360": {
                "left": EXISTING_CTX["kp360"]["reactions_uplift_kN"][EXISTING_CTX["kp360"]["support_left_x_mm"]],
                "right": EXISTING_CTX["kp360"]["reactions_uplift_kN"][EXISTING_CTX["kp360"]["support_right_x_mm"]],
            },
        },
    }

    lp_left_cases = {
        "ULS": check_existing_lp225_x125_combined(load_case="ULS", context=EXISTING_CTX),
        "SLS": check_existing_lp225_x125_combined(load_case="SLS", context=EXISTING_CTX),
        "UPLIFT": check_existing_lp225_x125_combined(load_case="UPLIFT", context=EXISTING_CTX),
    }
    lp_right_beam = member(GEO, "beams", "beam.lp225.x7075")
    lp_span_mm = EXISTING_CTX["lp225_x125"]["support_right_y_mm"]
    lp_point_y_mm = EXISTING_CTX["lp225_x125"]["base_point_y_mm"]
    lp_column_y_mm = EXISTING_CTX["lp225_x125"]["support_right_y_mm"]
    lp_right_beam_end_y_mm = float(lp_right_beam["axis_end"]["y"])

    def lp_right_response(point_load_kN):
        return point_beam_with_right_overhang_response(
            lp_span_mm,
            lp_right_beam_end_y_mm,
            [(lp_point_y_mm, point_load_kN)],
        )

    lp_right_cases = {
        case_group: lp_right_response(roof_support_cases[case_group]["kp450"]["right"])
        for case_group in ("ULS", "SLS", "UPLIFT")
    }

    hollow_beams = sorted(
        [beam_obj for beam_obj in GEO["members"]["beams"] if beam_obj["id"].startswith("beam.hollow.")],
        key=lambda item: (float(item["axis_start"]["x"]), item["id"]),
    )
    hollow_xs_mm = [float(beam_obj["axis_start"]["x"]) for beam_obj in hollow_beams]
    hollow_trib_ranges_mm = tributary_ranges_mm(hollow_xs_mm, FLOOR_X0_MM, FLOOR_X1_MM)
    outer_column_objs = {
        "col.x125.outer.bottom": left_outer_bottom_column,
        "col.x3600.outer.bottom": mid_outer_bottom_column,
        "col.x7075.outer.bottom": right_outer_bottom_column,
    }
    outer_column_base_x_mm = {
        column_id: float(column_obj["base"]["x"])
        for column_id, column_obj in outer_column_objs.items()
    }
    outer_beams = sorted(
        [beam_obj for beam_obj in GEO["members"]["beams"] if beam_obj["id"].startswith("beam.bottom.conreate")],
        key=lambda item: (float(item["axis_start"]["x"]), item["id"]),
    )
    if not outer_beams:
        raise ValueError("Outer concrete beam members are missing from geometry.")

    outer_beam_rows = []
    for outer_beam in outer_beams:
        support_rows = []
        for column_id in outer_column_objs:
            try:
                support_conn = connection_by_members(CONNECTION_LIST, outer_beam["id"], column_id)
            except KeyError:
                continue
            support_rows.append(
                {
                    "column_id": column_id,
                    "global_x_mm": outer_column_base_x_mm[column_id],
                    "local_s_mm": project_point_to_member_s_mm(outer_beam, support_conn["at"]),
                }
            )
        support_rows.sort(key=lambda row: row["local_s_mm"])
        if len(support_rows) < 2:
            raise ValueError(f"Outer beam {outer_beam['id']} must have at least two supports.")
        outer_beam_rows.append(
            {
                "id": outer_beam["id"],
                "obj": outer_beam,
                "length_mm": member_axis_length_mm(outer_beam),
                "self_total_kN": member_total_self_weight_kN(outer_beam, GAMMA_CONCRETE_KNM3),
                "self_kNm": member_uniform_self_weight_kNm(outer_beam, GAMMA_CONCRETE_KNM3),
                "support_rows": support_rows,
            }
        )

    outer_beam_self_kNm = outer_beam_rows[0]["self_kNm"]
    outer_beam_total_self_kN = sum(row["self_total_kN"] for row in outer_beam_rows)

    upper_column_support_loads = {}
    upper_column_totals = {}
    ground_column_totals = {}
    case_totals = {}
    hollow_case_results = {}
    outer_beam_case_results = {}
    for case_key, case_group in case_groups.items():
        permanent_factor = COLUMN_CASE_FACTORS[case_group]
        extra_upper_loads = extra_upper_column_loads_by_case.get(case_key, {})
        upper_column_support_loads[case_key] = {
            "col.x125": (
                roof_support_cases[case_group]["kp360"]["left"]
                + lp_left_cases[case_group]["reactions_kN"][lp_column_y_mm]
                + extra_upper_loads.get("col.x125", 0.0)
            ),
            "col.x7075": (
                roof_support_cases[case_group]["kp360"]["right"]
                + lp_right_cases[case_group]["reactions_kN"][lp_span_mm]
                + extra_upper_loads.get("col.x7075", 0.0)
            ),
        }
        upper_column_totals[case_key] = {
            "col.x125": upper_column_support_loads[case_key]["col.x125"]
            + column_self_weight_kN(left_inner_column, GAMMA_CONCRETE_KNM3, factor=permanent_factor),
            "col.x7075": upper_column_support_loads[case_key]["col.x7075"]
            + column_self_weight_kN(right_inner_column, GAMMA_CONCRETE_KNM3, factor=permanent_factor),
        }

        hollow_rows = []
        outer_loads_by_beam = {
            row["id"]: {"point_loads": [], "distributed_loads": []}
            for row in outer_beam_rows
        }

        for beam_obj, (trib_start_x_mm, trib_end_x_mm) in zip(hollow_beams, hollow_trib_ranges_mm):
            beam_id = beam_obj["id"]
            beam_length_mm = member_axis_length_mm(beam_obj)
            cast_trib_width_m = max(0.0, (trib_end_x_mm - trib_start_x_mm) / 1000.0)
            hollow_self_line_load_kNm = permanent_factor * GK_HOLLOW_SLAB_KNM2 * cast_trib_width_m
            cast_line_load_kNm = permanent_factor * GK_FLOOR_CAST_KNM2 * cast_trib_width_m
            live_line_load_kNm = FLOOR_LIVE_CASE_FACTORS[case_group] * QK_TERRACE_LIVE_KNM2 * cast_trib_width_m

            wall_support_s_mm = project_point_to_member_s_mm(beam_obj, connection_by_members(CONNECTION_LIST, beam_id, "ref.house_wall")["at"])
            outer_beam_row = None
            outer_support_conn = None
            for candidate in outer_beam_rows:
                try:
                    outer_support_conn = connection_by_members(CONNECTION_LIST, beam_id, candidate["id"])
                    outer_beam_row = candidate
                    break
                except KeyError:
                    continue
            if outer_beam_row is None or outer_support_conn is None:
                raise KeyError(f"Outer beam support not found for hollow slab {beam_id}")
            outer_support_s_mm = project_point_to_member_s_mm(beam_obj, outer_support_conn["at"])
            load_start_s_mm = member_s_at_axis_value_mm(beam_obj, "y", FLOOR_Y0_MM)
            load_end_s_mm = member_s_at_axis_value_mm(beam_obj, "y", FLOOR_Y1_MM)

            point_loads_kN = []
            support_s_mm = [wall_support_s_mm, outer_support_s_mm]

            node_points_mm = [0.0, beam_length_mm, wall_support_s_mm, outer_support_s_mm, load_start_s_mm, load_end_s_mm]
            node_points_mm.extend(x_mm for x_mm, _ in point_loads_kN)
            nodes_mm = sorted(set(node_points_mm))

            self_uniform = uniform_loads_for_nodes(nodes_mm, hollow_self_line_load_kNm / 1000.0)
            cast_uniform = intervals_to_uniform_loads(nodes_mm, [(load_start_s_mm, load_end_s_mm, cast_line_load_kNm / 1000.0)])
            live_uniform = intervals_to_uniform_loads(nodes_mm, [(load_start_s_mm, load_end_s_mm, live_line_load_kNm / 1000.0)])
            uniform = combine_uniform_loads(self_uniform, cast_uniform, live_uniform)

            response = beam_solver(
                nodes_mm,
                support_s_mm,
                point_loads_kN=point_loads_kN,
                uniform_loads_kN_per_mm=uniform,
            )
            reactions_kN = response["reactions_kN"]
            internal = sample_internal_forces(response["elements"])
            outer_reaction_kN = reactions_kN[outer_support_s_mm]
            outer_distribution = reaction_distribution(outer_support_conn)
            distribution_width_mm = reaction_distribution_width_mm(outer_distribution, beam_obj)
            outer_reaction_transfer = {
                "type": outer_distribution.get("type", "point"),
                "center_x_mm": float(outer_support_conn["at"]["x"]),
                "reaction_kN": outer_reaction_kN,
            }
            if distribution_width_mm is None:
                outer_loads_by_beam[outer_beam_row["id"]]["point_loads"].append(
                    (float(outer_support_conn["at"]["x"]), outer_reaction_kN)
                )
            else:
                interval_start_s_mm, interval_end_s_mm = distributed_reaction_interval_on_member_mm(
                    outer_beam_row["obj"],
                    outer_support_conn["at"],
                    distribution_width_mm,
                )
                interval_length_mm = interval_end_s_mm - interval_start_s_mm
                q_reaction_kN_per_mm = outer_reaction_kN / interval_length_mm
                start_point = member_point_at_s_mm(outer_beam_row["obj"], interval_start_s_mm)
                end_point = member_point_at_s_mm(outer_beam_row["obj"], interval_end_s_mm)
                distributed_load = {
                    "source_id": beam_id,
                    "start_s_mm": interval_start_s_mm,
                    "end_s_mm": interval_end_s_mm,
                    "start_x_mm": start_point["x"],
                    "end_x_mm": end_point["x"],
                    "width_mm": interval_length_mm,
                    "profile_width_mm": distribution_width_mm,
                    "q_kN_per_mm": q_reaction_kN_per_mm,
                    "total_kN": outer_reaction_kN,
                }
                outer_loads_by_beam[outer_beam_row["id"]]["distributed_loads"].append(distributed_load)
                outer_reaction_transfer.update(
                    {
                        "width_mm": interval_length_mm,
                        "profile_width_mm": distribution_width_mm,
                        "start_x_mm": start_point["x"],
                        "end_x_mm": end_point["x"],
                        "q_kNm": q_reaction_kN_per_mm * 1000.0,
                    }
                )

            hollow_rows.append(
                {
                    "id": beam_id,
                    "cast_tributary_width_m": cast_trib_width_m,
                    "hollow_self_line_load_kNm": hollow_self_line_load_kNm,
                    "cast_line_load_kNm": cast_line_load_kNm,
                    "live_line_load_kNm": live_line_load_kNm,
                    "uniform_total_line_load_kNm": hollow_self_line_load_kNm + cast_line_load_kNm + live_line_load_kNm,
                    "uniform_total_area_load_kNm2": (
                        (hollow_self_line_load_kNm + cast_line_load_kNm + live_line_load_kNm) / cast_trib_width_m
                        if cast_trib_width_m > 1e-9 else 0.0
                    ),
                    "point_loads_kN": point_loads_kN,
                    "wall_reaction_kN": reactions_kN[wall_support_s_mm],
                    "inner_reaction_kN": None,
                    "outer_reaction_kN": outer_reaction_kN,
                    "outer_reaction_transfer": outer_reaction_transfer,
                    "column_point_load_kN": 0.0,
                    "upper_column_id": None,
                    "M_pos": internal["M_pos"],
                    "M_neg": internal["M_neg"],
                    "V_abs": internal["V_abs"],
                }
            )

        outer_reactions_by_column_id_kN = {column_id: 0.0 for column_id in outer_column_objs}
        outer_reactions_by_global_x_kN = {
            base_x_mm: 0.0
            for base_x_mm in sorted(set(outer_column_base_x_mm.values()))
        }
        all_outer_point_loads_kN = []
        all_outer_distributed_loads = []
        outer_beam_case_beam_results = {}
        for outer_beam_row in outer_beam_rows:
            outer_beam_obj = outer_beam_row["obj"]
            outer_loads = outer_loads_by_beam[outer_beam_row["id"]]
            point_loads_global = aggregate_point_loads(outer_loads["point_loads"])
            all_outer_point_loads_kN.extend(point_loads_global)
            all_outer_distributed_loads.extend(outer_loads["distributed_loads"])
            point_loads_local = [
                (
                    project_point_to_member_s_mm(
                        outer_beam_obj,
                        {
                            "x": x_mm,
                            "y": float(outer_beam_obj["axis_start"]["y"]),
                            "z": float(outer_beam_obj["axis_start"]["z"]),
                        },
                    ),
                    load_kN,
                )
                for x_mm, load_kN in point_loads_global
            ]
            distributed_intervals_local = [
                (load["start_s_mm"], load["end_s_mm"], load["q_kN_per_mm"])
                for load in outer_loads["distributed_loads"]
            ]
            support_s_mm = [support_row["local_s_mm"] for support_row in outer_beam_row["support_rows"]]
            outer_beam_nodes_mm = sorted(
                {
                    0.0,
                    outer_beam_row["length_mm"],
                    *support_s_mm,
                    *[x_mm for x_mm, _ in point_loads_local],
                    *[x_mm for interval in distributed_intervals_local for x_mm in interval[:2]],
                }
            )
            outer_beam_uniform = combine_uniform_loads(
                uniform_loads_for_nodes(
                    outer_beam_nodes_mm,
                    permanent_factor * outer_beam_row["self_kNm"] / 1000.0,
                ),
                intervals_to_uniform_loads(outer_beam_nodes_mm, distributed_intervals_local),
            )
            outer_beam_response = beam_solver(
                outer_beam_nodes_mm,
                support_s_mm,
                point_loads_kN=point_loads_local,
                uniform_loads_kN_per_mm=outer_beam_uniform,
            )
            beam_reactions_by_global_x_kN = {}
            beam_reactions_by_column_id_kN = {}
            for support_row in outer_beam_row["support_rows"]:
                reaction_kN = outer_beam_response["reactions_kN"][support_row["local_s_mm"]]
                beam_reactions_by_column_id_kN[support_row["column_id"]] = reaction_kN
                beam_reactions_by_global_x_kN[support_row["global_x_mm"]] = (
                    beam_reactions_by_global_x_kN.get(support_row["global_x_mm"], 0.0) + reaction_kN
                )
                outer_reactions_by_column_id_kN[support_row["column_id"]] += reaction_kN
                outer_reactions_by_global_x_kN[support_row["global_x_mm"]] += reaction_kN

            outer_beam_case_beam_results[outer_beam_row["id"]] = {
                "point_loads_kN": point_loads_global,
                "distributed_loads": list(outer_loads["distributed_loads"]),
                "reactions_kN": beam_reactions_by_global_x_kN,
                "reactions_by_column_id_kN": beam_reactions_by_column_id_kN,
                "self_total_kN": permanent_factor * outer_beam_row["self_total_kN"],
                "self_kNm": outer_beam_row["self_kNm"],
            }

        outer_beam_case_results[case_key] = {
            "point_loads_kN": aggregate_point_loads(all_outer_point_loads_kN),
            "distributed_loads": all_outer_distributed_loads,
            "reactions_kN": outer_reactions_by_global_x_kN,
            "reactions_by_column_id_kN": outer_reactions_by_column_id_kN,
            "beam_results": outer_beam_case_beam_results,
        }
        hollow_case_results[case_key] = hollow_rows

        ground_column_totals[case_key] = {
            "col.x125": upper_column_totals[case_key]["col.x125"],
            "col.x7075": upper_column_totals[case_key]["col.x7075"],
            "col.x125.outer.bottom": outer_reactions_by_column_id_kN["col.x125.outer.bottom"] + column_self_weight_kN(left_outer_bottom_column, GAMMA_CONCRETE_KNM3, factor=permanent_factor),
            "col.x3600.outer.bottom": outer_reactions_by_column_id_kN["col.x3600.outer.bottom"] + column_self_weight_kN(mid_outer_bottom_column, GAMMA_CONCRETE_KNM3, factor=permanent_factor),
            "col.x7075.outer.bottom": outer_reactions_by_column_id_kN["col.x7075.outer.bottom"] + column_self_weight_kN(right_outer_bottom_column, GAMMA_CONCRETE_KNM3, factor=permanent_factor),
        }
        for column_id, extra_load_kN in extra_ground_column_loads_by_case.get(case_key, {}).items():
            ground_column_totals[case_key][column_id] = ground_column_totals[case_key].get(column_id, 0.0) + float(extra_load_kN)

        case_totals[case_key] = dict(ground_column_totals[case_key])

    return {
        "gamma_concrete_kNm3": GAMMA_CONCRETE_KNM3,
        "gk_hollow_slab_kNm2": GK_HOLLOW_SLAB_KNM2,
        "gk_hollow_slab_allow_kNm2": GK_HOLLOW_SLAB_ALLOW_KNM2,
        "gk_floor_cast_kNm2": GK_FLOOR_CAST_KNM2,
        "qk_terrace_live_kNm2": QK_TERRACE_LIVE_KNM2,
        "qk_hollow_slab_allow_kNm2": QK_HOLLOW_SLAB_ALLOW_KNM2,
        "outer_beam_self_kNm": outer_beam_self_kNm,
        "outer_beam_total_self_kN": outer_beam_total_self_kN,
        "outer_beam_count": len(outer_beam_rows),
        "hollow_beam_self_kNm": profile_b(hollow_beams[0]) / 1000.0 * GK_HOLLOW_SLAB_KNM2,
        "column_output_order": COLUMN_OUTPUT_ORDER,
        "column_display": COLUMN_DISPLAY,
        "column_group_label": COLUMN_GROUP_LABEL,
        "case_groups": dict(case_groups),
        "upper_column_support_loads": upper_column_support_loads,
        "upper_column_totals": upper_column_totals,
        "ground_column_totals": ground_column_totals,
        "case_totals": case_totals,
        "hollow_case_results": hollow_case_results,
        "outer_beam_case_results": outer_beam_case_results,
    }


def envelope_column_totals(case_totals, uls_case_keys, sls_case_keys, uplift_case_key="UPLIFT"):
    envelope = {}
    for column_id in COLUMN_OUTPUT_ORDER:
        sls_case = max(sls_case_keys, key=lambda case_key: case_totals[case_key][column_id])
        uls_case = max(uls_case_keys, key=lambda case_key: case_totals[case_key][column_id])
        envelope[column_id] = {
            "N_sls": case_totals[sls_case][column_id],
            "sls_case_key": sls_case,
            "N_uls": case_totals[uls_case][column_id],
            "uls_case_key": uls_case,
            "N_min": case_totals[uplift_case_key][column_id],
            "uplift_case_key": uplift_case_key,
        }
    return envelope
