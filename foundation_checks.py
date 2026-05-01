from geometry_loader import member, reference, profile_b, profile_h


_BASE_VEC = {
    "+x": (1.0, 0.0, 0.0),
    "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0),
    "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0),
    "-z": (0.0, 0.0, -1.0),
}


def _axis_vector(axis):
    direction = _BASE_VEC[axis["dir"]]
    if "length_mm" in axis:
        length_mm = float(axis["length_mm"])
        return tuple(component * length_mm for component in direction)
    horizontal_mm = float(axis.get("horizontal_mm", 0.0))
    rise_mm = float(axis.get("rise_mm", 0.0))
    return (
        direction[0] * horizontal_mm,
        direction[1] * horizontal_mm,
        direction[2] * horizontal_mm + rise_mm,
    )


def surface_z_at(reference_surface, x_mm, y_mm):
    placement = reference_surface["placement"]
    anchor = placement["anchor"]
    u = _axis_vector(placement["u"])
    v = _axis_vector(placement["v"])
    dx = float(x_mm) - float(anchor["x"])
    dy = float(y_mm) - float(anchor["y"])
    det = u[0] * v[1] - u[1] * v[0]
    if abs(det) <= 1e-9:
        raise ValueError(f"Reference surface {reference_surface['id']} is not usable as an x/y ground plane.")
    a = (dx * v[1] - dy * v[0]) / det
    b = (u[0] * dy - u[1] * dx) / det
    return float(anchor["z"]) + a * u[2] + b * v[2]


def _range_text(min_value, max_value, unit="mm", decimals=0):
    if abs(max_value - min_value) <= 0.5 * 10 ** (-decimals):
        return f"{min_value:.{decimals}f} {unit}"
    return f"{min_value:.{decimals}f}...{max_value:.{decimals}f} {unit}"


def _foundation_column(geo, foundation):
    return member(geo, "columns", foundation["supports"])


def _foundation_center_mm(geo, foundation):
    if "center" in foundation:
        center = foundation["center"]
        return float(center["x"]), float(center["y"])
    column_obj = _foundation_column(geo, foundation)
    return float(column_obj["base"]["x"]), float(column_obj["base"]["y"])


def _foundation_top_z_mm(geo, foundation):
    if "top_z" in foundation:
        return float(foundation["top_z"])
    return float(_foundation_column(geo, foundation)["base"]["z"])


def _foundation_corners_mm(geo, foundation):
    center_x_mm, center_y_mm = _foundation_center_mm(geo, foundation)
    size = foundation["size_mm"]
    half_x_mm = 0.5 * float(size["x"])
    half_y_mm = 0.5 * float(size["y"])
    return [
        (center_x_mm - half_x_mm, center_y_mm - half_y_mm),
        (center_x_mm + half_x_mm, center_y_mm - half_y_mm),
        (center_x_mm - half_x_mm, center_y_mm + half_y_mm),
        (center_x_mm + half_x_mm, center_y_mm + half_y_mm),
    ]


def foundation_checks_from_envelope(geo, column_envelope, gamma_g=1.35, gamma_g_min=0.9):
    checks = []
    for foundation in geo.get("foundations", []):
        column_id = foundation["supports"]
        if column_id not in column_envelope:
            raise KeyError(f"Foundation {foundation['id']} supports {column_id}, but no column loads were provided.")

        column_obj = _foundation_column(geo, foundation)
        size = foundation["size_mm"]
        width_x_m = float(size["x"]) / 1000.0
        width_y_m = float(size["y"]) / 1000.0
        height_m = float(size["z"]) / 1000.0
        area_m2 = width_x_m * width_y_m
        gamma_concrete = float(foundation.get("gamma_kNm3", 24.0))
        footing_gk_kN = area_m2 * height_m * gamma_concrete

        top_z_mm = _foundation_top_z_mm(geo, foundation)
        bottom_z_mm = top_z_mm - float(size["z"])
        ground = reference(geo, foundation.get("ground_ref", "ref.ground"))
        center_x_mm, center_y_mm = _foundation_center_mm(geo, foundation)
        ground_center_z_mm = surface_z_at(ground, center_x_mm, center_y_mm)
        corner_ground_z_mm = [
            surface_z_at(ground, x_mm, y_mm)
            for x_mm, y_mm in _foundation_corners_mm(geo, foundation)
        ]
        top_cover_values_mm = [ground_z_mm - top_z_mm for ground_z_mm in corner_ground_z_mm]
        bottom_depth_values_mm = [ground_z_mm - bottom_z_mm for ground_z_mm in corner_ground_z_mm]
        top_cover_min_mm = min(top_cover_values_mm)
        top_cover_max_mm = max(top_cover_values_mm)
        bottom_depth_min_mm = min(bottom_depth_values_mm)
        bottom_depth_max_mm = max(bottom_depth_values_mm)

        soil_cover = foundation.get("soil_cover")
        soil_cover_gk_kN = 0.0
        soil_cover_in_uplift = False
        if soil_cover is not None:
            avg_cover_m = max(0.0, 0.5 * (top_cover_min_mm + top_cover_max_mm) / 1000.0)
            column_area_m2 = profile_b(column_obj) / 1000.0 * profile_h(column_obj) / 1000.0
            cover_area_m2 = max(0.0, area_m2 - column_area_m2)
            soil_cover_gk_kN = cover_area_m2 * avg_cover_m * float(soil_cover["gamma_kNm3"])
            soil_cover_in_uplift = bool(soil_cover.get("include_in_uplift", True))

        load_row = column_envelope[column_id]
        permanent_gk_kN = footing_gk_kN + soil_cover_gk_kN
        q_sls_kPa = (float(load_row["N_sls"]) + permanent_gk_kN) / area_m2
        q_uls_kPa = (float(load_row["N_uls"]) + gamma_g * permanent_gk_kN) / area_m2
        uplift_demand_kN = max(0.0, -float(load_row["N_min"]))
        uplift_resistance_gk_kN = footing_gk_kN + (soil_cover_gk_kN if soil_cover_in_uplift else 0.0)
        uplift_resistance_kN = gamma_g_min * uplift_resistance_gk_kN
        uplift_residual_kN = max(0.0, uplift_demand_kN - uplift_resistance_kN)

        soil_bearing = foundation.get("soil_bearing", {})
        q_design_kPa = soil_bearing.get("q_design_kPa")
        bearing_ok = None if q_design_kPa is None else q_uls_kPa <= float(q_design_kPa) + 1e-9

        checks.append({
            "id": foundation["id"],
            "column_id": column_id,
            "material": foundation.get("material", "reinforced_concrete"),
            "frost_insulated": bool(foundation.get("frost_insulated", False)),
            "anchorage_type": foundation.get("anchorage", {}).get("type", "unknown"),
            "ground_ref": foundation.get("ground_ref", "ref.ground"),
            "ground_center_z_mm": ground_center_z_mm,
            "top_z_mm": top_z_mm,
            "bottom_z_mm": bottom_z_mm,
            "size_x_mm": float(size["x"]),
            "size_y_mm": float(size["y"]),
            "size_z_mm": float(size["z"]),
            "area_m2": area_m2,
            "footing_gk_kN": footing_gk_kN,
            "soil_cover_gk_kN": soil_cover_gk_kN,
            "permanent_gk_kN": permanent_gk_kN,
            "top_cover_min_mm": top_cover_min_mm,
            "top_cover_max_mm": top_cover_max_mm,
            "bottom_depth_min_mm": bottom_depth_min_mm,
            "bottom_depth_max_mm": bottom_depth_max_mm,
            "N_sls": float(load_row["N_sls"]),
            "N_uls": float(load_row["N_uls"]),
            "N_min": float(load_row["N_min"]),
            "q_sls_kPa": q_sls_kPa,
            "q_uls_kPa": q_uls_kPa,
            "q_design_kPa": None if q_design_kPa is None else float(q_design_kPa),
            "bearing_ok": bearing_ok,
            "uplift_demand_kN": uplift_demand_kN,
            "uplift_resistance_kN": uplift_resistance_kN,
            "uplift_residual_kN": uplift_residual_kN,
        })
    return checks


def foundation_report_lines(checks):
    if not checks:
        return ["  Perustuksia ei ole mallinnettu geometriassa."]

    first = checks[0]
    same_size = all(
        (
            abs(row["size_x_mm"] - first["size_x_mm"]) <= 1e-9
            and abs(row["size_y_mm"] - first["size_y_mm"]) <= 1e-9
            and abs(row["size_z_mm"] - first["size_z_mm"]) <= 1e-9
        )
        for row in checks
    )
    q_design_values = {row["q_design_kPa"] for row in checks if row["q_design_kPa"] is not None}

    lines = []
    if same_size:
        lines.append(
            f"  Anturat                       {len(checks)} kpl, "
            f"{first['size_x_mm']:.0f}x{first['size_y_mm']:.0f}x{first['size_z_mm']:.0f} mm"
        )
    else:
        lines.append(f"  Anturat                       {len(checks)} kpl")
    lines.append(
        f"  Maanpinta                     {first['ground_ref']}; peitesyvyydet laskettu anturan kulmista"
    )
    lines.append(
        "  Pysyvä paino nostossa         antura + maanpeite, jos soil_cover.include_in_uplift = true"
    )
    if q_design_values:
        if len(q_design_values) == 1:
            lines.append(f"  Maapohjan q_Rd                {next(iter(q_design_values)):.0f} kPa")
        else:
            lines.append("  Maapohjan q_Rd                perustuksittain")
    else:
        lines.append("  Maapohjan q_Rd                ei annettu; vertaa alla oleviin q_uls-arvoihin")
    lines.append("")
    lines.append(
        f"  {'Perustus':<23} {'Pilari':<22} {'peite yläp.':>12} {'alap. syv.':>12}"
        f" {'Gk':>7} {'q_sls':>7} {'q_uls':>7} {'N_up':>7} {'R_up':>7} {'jäännös':>8}"
    )
    lines.append(
        f"  {'-'*23} {'-'*22} {'-'*12} {'-'*12}"
        f" {'-'*7} {'-'*7} {'-'*7} {'-'*7} {'-'*7} {'-'*8}"
    )
    for row in checks:
        lines.append(
            f"  {row['id']:<23} {row['column_id']:<22} "
            f"{_range_text(row['top_cover_min_mm'], row['top_cover_max_mm']):>12} "
            f"{_range_text(row['bottom_depth_min_mm'], row['bottom_depth_max_mm']):>12} "
            f"{row['permanent_gk_kN']:>7.2f} {row['q_sls_kPa']:>7.0f} {row['q_uls_kPa']:>7.0f} "
            f"{row['uplift_demand_kN']:>7.2f} {row['uplift_resistance_kN']:>7.2f} {row['uplift_residual_kN']:>8.2f}"
        )
    max_q = max(checks, key=lambda row: row["q_uls_kPa"])
    max_residual = max(checks, key=lambda row: row["uplift_residual_kN"])
    lines.append(
        f"  Suurin pohjapaine ULS         {max_q['q_uls_kPa']:.0f} kPa ({max_q['id']})"
    )
    lines.append(
        f"  Suurin jäljelle jäävä nosto   {max_residual['uplift_residual_kN']:.2f} kN ({max_residual['id']})"
    )
    lines.append("  Huom.                         vaakakuormat, liukuminen ja kaatuminen tarkistettava erikseen")
    return lines
