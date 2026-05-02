"""Shared geometry and self-weight helpers for structural calculators."""

import math

from geometry_loader import profile_b, profile_h


def member_axis_vector_3d(member_obj):
    return (
        float(member_obj["axis_end"]["x"]) - float(member_obj["axis_start"]["x"]),
        float(member_obj["axis_end"]["y"]) - float(member_obj["axis_start"]["y"]),
        float(member_obj["axis_end"]["z"]) - float(member_obj["axis_start"]["z"]),
    )


def member_axis_length_mm(member_obj):
    dx_mm, dy_mm, dz_mm = member_axis_vector_3d(member_obj)
    return math.sqrt(dx_mm**2 + dy_mm**2 + dz_mm**2)


def column_length_mm(column_obj):
    base = column_obj["base"]
    top = column_obj["top"]
    dx_mm = float(top["x"]) - float(base["x"])
    dy_mm = float(top["y"]) - float(base["y"])
    dz_mm = float(top["z"]) - float(base["z"])
    return math.sqrt(dx_mm**2 + dy_mm**2 + dz_mm**2)


def member_total_self_weight_kN(member_obj, gamma_kNm3):
    if "mass_kg" in member_obj:
        return float(member_obj["mass_kg"]) * 9.81 / 1000.0
    return (
        profile_b(member_obj) / 1000.0
        * profile_h(member_obj) / 1000.0
        * (member_axis_length_mm(member_obj) / 1000.0)
        * gamma_kNm3
    )


def member_uniform_self_weight_kNm(member_obj, gamma_kNm3):
    member_length_m = member_axis_length_mm(member_obj) / 1000.0
    if member_length_m <= 1e-9:
        return 0.0
    return member_total_self_weight_kN(member_obj, gamma_kNm3) / member_length_m


def column_self_weight_kN(column_obj, gamma_kNm3, factor=1.0):
    area_m2 = profile_b(column_obj) / 1000.0 * profile_h(column_obj) / 1000.0
    return factor * area_m2 * (column_length_mm(column_obj) / 1000.0) * gamma_kNm3


def member_s_at_axis_value_mm(member_obj, axis, value_mm):
    start_value_mm = float(member_obj["axis_start"][axis])
    end_value_mm = float(member_obj["axis_end"][axis])
    delta_mm = end_value_mm - start_value_mm
    if abs(delta_mm) <= 1e-9:
        raise ValueError(f"Member {member_obj['id']} is not monotonic in {axis}.")
    t = (float(value_mm) - start_value_mm) / delta_mm
    return max(0.0, min(1.0, t)) * member_axis_length_mm(member_obj)


def member_point_at_s_mm(member_obj, s_mm):
    length_mm = member_axis_length_mm(member_obj)
    if length_mm <= 1e-9:
        return dict(member_obj["axis_start"])
    t = max(0.0, min(1.0, float(s_mm) / length_mm))
    start = member_obj["axis_start"]
    end = member_obj["axis_end"]
    return {
        axis: float(start[axis]) + (float(end[axis]) - float(start[axis])) * t
        for axis in ("x", "y", "z")
    }


def project_point_to_member_s_mm(member_obj, point_xyz):
    start = member_obj["axis_start"]
    dx_mm, dy_mm, dz_mm = member_axis_vector_3d(member_obj)
    length_sq = dx_mm**2 + dy_mm**2 + dz_mm**2
    if length_sq <= 1e-9:
        return 0.0
    px_mm = float(point_xyz["x"]) - float(start["x"])
    py_mm = float(point_xyz["y"]) - float(start["y"])
    pz_mm = float(point_xyz.get("z", start["z"])) - float(start["z"])
    length_mm = math.sqrt(length_sq)
    s_mm = (px_mm * dx_mm + py_mm * dy_mm + pz_mm * dz_mm) / length_mm
    return max(0.0, min(length_mm, s_mm))


def connection_by_members(geo_or_connections, member_a_id, member_b_id):
    connections = geo_or_connections.get("connections", []) if isinstance(geo_or_connections, dict) else geo_or_connections
    wanted = {member_a_id, member_b_id}
    for connection_obj in connections:
        if set(connection_obj.get("members", [])) == wanted:
            return connection_obj
    raise KeyError(f"Connection not found for members: {member_a_id}, {member_b_id}")


def distributed_interval_on_member_axis_mm(support_member_obj, axis, center_value_mm, width_mm):
    center_s_mm = member_s_at_axis_value_mm(support_member_obj, axis, center_value_mm)
    half_width_mm = float(width_mm) / 2.0
    start_s_mm = max(0.0, center_s_mm - half_width_mm)
    end_s_mm = min(member_axis_length_mm(support_member_obj), center_s_mm + half_width_mm)
    if end_s_mm - start_s_mm <= 1e-9:
        raise ValueError(f"Distributed reaction does not overlap {support_member_obj['id']}.")
    return start_s_mm, end_s_mm


def distributed_reaction_interval_on_member_mm(support_member_obj, center_point, width_mm):
    center_s_mm = project_point_to_member_s_mm(support_member_obj, center_point)
    half_width_mm = float(width_mm) / 2.0
    start_s_mm = max(0.0, center_s_mm - half_width_mm)
    end_s_mm = min(member_axis_length_mm(support_member_obj), center_s_mm + half_width_mm)
    if end_s_mm - start_s_mm <= 1e-9:
        raise ValueError(
            f"Distributed reaction width {width_mm:.0f} mm does not overlap support member {support_member_obj['id']}"
        )
    return start_s_mm, end_s_mm
