"""Shared timber member section and notch check helpers."""

import math

from beam_analysis import section_state_at_x_mm
from geometry_loader import profile_b


def member_section_rotation_deg(member_obj):
    return float(member_obj.get("section_rotation_deg", 0.0))


def member_rect_props(b_mm, h_mm, section_rotation_deg=0.0):
    theta = math.radians(float(section_rotation_deg))
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    I_strong_mm4 = b_mm * h_mm**3 / 12.0
    I_weak_mm4 = h_mm * b_mm**3 / 12.0
    I_vertical_mm4 = I_strong_mm4 * cos_theta**2 + I_weak_mm4 * sin_theta**2
    I_horizontal_mm4 = I_strong_mm4 * sin_theta**2 + I_weak_mm4 * cos_theta**2
    c_vertical_mm = max(1e-9, 0.5 * (abs(h_mm * cos_theta) + abs(b_mm * sin_theta)))
    c_horizontal_mm = max(1e-9, 0.5 * (abs(h_mm * sin_theta) + abs(b_mm * cos_theta)))
    return {
        "W_mm3": I_vertical_mm4 / c_vertical_mm,
        "W_vertical_mm3": I_vertical_mm4 / c_vertical_mm,
        "W_horizontal_mm3": I_horizontal_mm4 / c_horizontal_mm,
        "A_mm2": b_mm * h_mm,
        "I_mm4": I_vertical_mm4,
        "I_vertical_mm4": I_vertical_mm4,
        "I_horizontal_mm4": I_horizontal_mm4,
        "I_strong_mm4": I_strong_mm4,
        "I_weak_mm4": I_weak_mm4,
        "section_rotation_deg": float(section_rotation_deg),
    }


def _sample_net_section_utilization_impl(
    elements,
    section_h_mm_at_x,
    b_mm,
    section_rotation_deg,
    fm_d_Nmm2,
    fv_d_Nmm2,
    x_start_mm,
    x_end_mm,
    step_mm,
):
    x_lo_mm = min(float(x_start_mm), float(x_end_mm))
    x_hi_mm = max(float(x_start_mm), float(x_end_mm))
    max_eta_M = {"value_pct": 0.0, "x_mm": None, "M_kNm": 0.0, "h_mm": None}
    max_eta_V = {"value_pct": 0.0, "x_mm": None, "V_kN": 0.0, "h_mm": None}

    x_mm = x_lo_mm
    while x_mm <= x_hi_mm + 1e-9:
        h_mm = max(1e-9, float(section_h_mm_at_x(x_mm)))
        state = section_state_at_x_mm(elements, x_mm)
        props = member_rect_props(b_mm, h_mm, section_rotation_deg)
        MRd_kNm = fm_d_Nmm2 * props["W_mm3"] / 1.0e6
        VRd_kN = fv_d_Nmm2 * props["A_mm2"] / 1.5e3
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


def sample_net_section_utilization(
    elements,
    member_obj,
    section_h_mm_at_x,
    fm_d_Nmm2,
    fv_d_Nmm2,
    x_start_mm,
    x_end_mm,
    step_mm=1.0,
):
    return _sample_net_section_utilization_impl(
        elements,
        section_h_mm_at_x,
        profile_b(member_obj),
        member_section_rotation_deg(member_obj),
        fm_d_Nmm2,
        fv_d_Nmm2,
        x_start_mm,
        x_end_mm,
        step_mm,
    )


def sample_net_section_utilization_rect(
    elements,
    section_h_mm_at_x,
    b_mm,
    fm_d_Nmm2,
    fv_d_Nmm2,
    x_start_mm,
    x_end_mm,
    step_mm=1.0,
    section_rotation_deg=0.0,
):
    return _sample_net_section_utilization_impl(
        elements,
        section_h_mm_at_x,
        b_mm,
        section_rotation_deg,
        fm_d_Nmm2,
        fv_d_Nmm2,
        x_start_mm,
        x_end_mm,
        step_mm,
    )


def sample_min_section_height_mm(section_h_mm_at_x, x_start_mm, x_end_mm, step_mm=1.0):
    x_lo_mm = min(float(x_start_mm), float(x_end_mm))
    x_hi_mm = max(float(x_start_mm), float(x_end_mm))
    min_h_mm = float(section_h_mm_at_x(x_lo_mm))
    x_at_min_mm = x_lo_mm
    x_mm = x_lo_mm
    while x_mm <= x_hi_mm + 1e-9:
        h_mm = float(section_h_mm_at_x(x_mm))
        if h_mm < min_h_mm:
            min_h_mm = h_mm
            x_at_min_mm = x_mm
        x_mm += step_mm
    return {"h_mm": min_h_mm, "x_mm": x_at_min_mm}


def governing_moment(internal):
    if internal["M_pos"]["value_kNm"] >= -internal["M_neg"]["value_kNm"]:
        return {"sign": "+", **internal["M_pos"], "raw_value_kNm": internal["M_pos"]["value_kNm"]}
    return {
        "sign": "−",
        "value_kNm": -internal["M_neg"]["value_kNm"],
        "x_mm": internal["M_neg"]["x_mm"],
        "raw_value_kNm": internal["M_neg"]["value_kNm"],
    }


def combined_section_h(full_h_mm, depth_functions):
    def section_h(coord_mm):
        return max(1e-9, full_h_mm - sum(max(0.0, fn(coord_mm)) for fn in depth_functions))

    return section_h
