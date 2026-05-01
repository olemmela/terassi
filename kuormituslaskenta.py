"""
KATOKSEN KUORMITUSLASKENTA - ETELÄSUOMI
========================================
Standardi: EN 1990, EN 1991-1-3, EN 1991-1-4, EN 1995-1-1 (Eurokoodit)
Suomen kansalliset liitteet (FI NA)

Geometria:
  - Yksilappinen katos, kallostus 12° seinän suuntaisesti (vasen alas, oikea ylhäällä)
  - Palkit kulkevat seinän suuntaisesti (x-suunta = jännevälin suunta)
  - LP225x90 (liimapuu) kulkee seinältä pilarille (y-suunta, 1675mm),
    toimii KP450x51-palkin päätykannakkeena molemmissa päissä
  - KP450x51 sijaitsee 0mm seinästä, pultattu seinään 900mm välein M10
  - KP450x51 sijaitsee 900mm seinästä, tuettu LP225x90:n päälle
  - 2xKP360x51 sijaitsee 1675mm seinästä (tolppien päällä)
  - Katon reuna 2200mm seinästä
"""

import contextlib
import importlib.util
import io
import math
from pathlib import Path

from beam_analysis import (
    beam_solver,
    combine_uniform_loads,
    intervals_to_uniform_loads,
    refine_nodes_mm,
    sample_internal_forces,
    sample_max_deflection_mm,
    uniform_loads_for_nodes,
)
from existing_beam_checks import (
    aggregate_point_loads,
    check_existing_lp225_x125_combined,
    format_point_loads,
    katos_existing_context,
    simple_span_combined_moment_max,
    simple_span_max_deflection_mm,
    simple_span_max_shear,
    simple_span_point_reactions,
    uniform_line_member_support_reactions,
)
from geometry_loader import expanded_connections, expanded_members, load, member, surface, profile_b, profile_h
from terrace_column_loads import calculate_katos_total_column_loads
from timber_member_checks import (
    combined_section_h,
    governing_moment,
    member_rect_props,
    member_section_rotation_deg,
    sample_min_section_height_mm,
    sample_net_section_utilization,
)

# ============================================================
# GEOMETRIA  (luetaan geometry/katos.json:ista)
# ============================================================
_GEO = load("katos.json")
_CONNECTION_LIST = expanded_connections(_GEO)
_PURLIN_MEMBERS = {member_obj["id"]: member_obj for member_obj in expanded_members(_GEO, "purlins")}
_CONNECTIONS = {connection_obj["id"]: connection_obj for connection_obj in _CONNECTION_LIST}
_EXISTING_CTX = katos_existing_context()
_ROOF_CTX = _EXISTING_CTX["roof"]
_PURLIN_CTX = _EXISTING_CTX["purlins_main"]
_RAYST_CTX = _EXISTING_CTX["purlins_kp450_side"]
_BEAM1_CTX = _EXISTING_CTX["kp450_y900"]
_KP360_CTX = _EXISTING_CTX["kp360"]

_wall_poly = _GEO["reference_surfaces"][0]["polygon"]
wall_width  = int(max(p["x"] for p in _wall_poly) - min(p["x"] for p in _wall_poly))
pillar_size = int(member(_GEO, "columns", "col.x125")["profile"]["b_mm"])
clear_span  = wall_width - 2 * pillar_size  # = 6700 mm

_roof = surface(_GEO, "surf.roof")
_roof_poly = _roof["polygon"]
_roof_xspan = max(p["x"] for p in _roof_poly) - min(p["x"] for p in _roof_poly)
_roof_zspan = max(p["z"] for p in _roof_poly) - min(p["z"] for p in _roof_poly)
slope_deg       = math.degrees(math.atan(_roof_zspan / _roof_xspan))
slope_rad       = math.radians(slope_deg)
roof_edge_y     = int(max(p["y"] for p in _roof_poly))  # 2200 mm
roof_x0_mm      = min(p["x"] for p in _roof_poly)
roof_x1_mm      = max(p["x"] for p in _roof_poly)

beam1_y         = int(member(_GEO, "beams", "beam.kp450.y900")["axis_start"]["y"])  # 900 mm
beam2_y         = int(member(_GEO, "beams", "beam.kp360x2")["axis_start"]["y"])     # 1675 mm

# Olemassa olevat ruoteet 50×100 mm (KP450 → 2×KP360, y-suunta)
# Antavat sivutuen KP450×51 #2:lle heikossa akselissa.
_purlin = member(_GEO, "purlins", "purlin.50x100")
ruode_jako_mm   = float(_purlin["pattern"]["offset"]["x"])  # 900 mm
ruode_b         = float(_purlin["profile"]["b_mm"])         # 50 mm
ruode_h         = float(_purlin["profile"]["h_mm"])         # 100 mm

# Jänneväli (pilarin keskilinjalta keskilinjalle)
_col_xs = [member(_GEO, "columns", cid)["base"]["x"]
           for cid in ("col.x125", "col.x7075")]
L_mm  = float(max(_col_xs) - min(_col_xs))  # mm (c/c pilarikeskiöt)
L_m   = L_mm / 1000.0                        # m

# Palkin uloke tukien yli ja katon räystäs (x-suunta)
_beam_ref = member(_GEO, "beams", "beam.kp450.y900")
_bx0 = float(_beam_ref["axis_start"]["x"])
_bx1 = float(_beam_ref["axis_end"]["x"])
a_oh_left_mm  = float(min(_col_xs)) - _bx0                   # mm
a_oh_right_mm = _bx1 - float(max(_col_xs))                   # mm
eave_left_mm  = _bx0 - min(p["x"] for p in _roof_poly)       # mm
eave_right_mm = max(p["x"] for p in _roof_poly) - _bx1       # mm

# Kaltevuus on palkin JÄNTEEN SUUNNASSA (x-suunta), ei kohtisuora.
# Pystysuorilla kuormilla (q kN/m vaaka-alaa kohti) yksinkertaisesti
# tuetulle palkille statiikan perusyhtälöistä:
#   R = q × L_h / 2  (riippumatta α)  →  M_max = q × L_h² / 8
# Kaltevuuskorjausta momenttiin EI tarvita.
moment_factor = 1.0   # ei korjauskerrointa – kaltevuus jänteen suunnassa


def symmetric_support_reaction(span_line_load, span_m, beam_overhang_m,
                               roof_area_load, tributary_width_m, roof_eave_m):
    """Tukireaktio per tuki symmetriselle palkille, jossa on uloke ja räystäs."""
    return (
        span_line_load * (span_m / 2.0 + beam_overhang_m)
        + roof_area_load * tributary_width_m * roof_eave_m
    )


def load_script_module_quietly(script_name, module_name):
    """Lataa laskentaskriptin moduulina ilman konsolitulostetta."""
    script_path = Path(__file__).with_name(script_name)
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load script module: {script_name}")
    module = importlib.util.module_from_spec(spec)
    with contextlib.redirect_stdout(io.StringIO()):
        spec.loader.exec_module(module)
    return module


def connection_by_members(member_a_id, member_b_id):
    wanted = {member_a_id, member_b_id}
    for connection_obj in _CONNECTION_LIST:
        if set(connection_obj.get("members", [])) == wanted:
            return connection_obj
    raise KeyError(f"Connection not found for members: {member_a_id}, {member_b_id}")


def connection_cut(connection_id, kind):
    for cut in _CONNECTIONS[connection_id].get("cuts", []):
        if cut.get("kind") == kind:
            return cut
    return None


def bevel_notch_info(connection_id):
    cut = connection_cut(connection_id, "bevel_notch")
    if cut is None:
        return {
            "active": False,
            "depth_mm": 0.0,
            "length_mm": 0.0,
            "offset_mm": 0.0,
            "reference": None,
        }
    return {
        "active": True,
        "depth_mm": float(cut["depth_mm"]),
        "length_mm": float(cut["length_mm"]),
        "offset_mm": float(cut.get("offset_mm", 0.0)),
        "reference": cut["reference"],
    }


def make_end_referenced_bevel_notch_depth_fn(info, end_coord_mm, inward_positive_sign, coord_min_mm, coord_max_mm):
    if not info["active"]:
        def depth_fn(_coord_mm):
            return 0.0
        return (end_coord_mm, end_coord_mm), depth_fn, False

    offset_mm = info["offset_mm"]
    length_mm = info["length_mm"]
    zone_start_mm = max(coord_min_mm, min(coord_max_mm, end_coord_mm + inward_positive_sign * offset_mm))
    zone_end_mm = max(coord_min_mm, min(coord_max_mm, end_coord_mm + inward_positive_sign * (offset_mm + length_mm)))
    zone = tuple(sorted((zone_start_mm, zone_end_mm)))

    def depth_fn(coord_mm):
        local_mm = inward_positive_sign * (coord_mm - end_coord_mm)
        if local_mm < offset_mm - 1e-9 or local_mm > offset_mm + length_mm + 1e-9:
            return 0.0
        if length_mm <= 1e-9:
            return info["depth_mm"]
        return info["depth_mm"] * (1.0 - (local_mm - offset_mm) / length_mm)

    return zone, depth_fn, True


def make_support_referenced_bevel_notch_depth_fn(info, support_coord_mm, coord_min_mm, coord_max_mm):
    if not info["active"]:
        def depth_fn(_coord_mm):
            return 0.0
        return (support_coord_mm, support_coord_mm), depth_fn, False

    offset_mm = info["offset_mm"]
    length_mm = info["length_mm"]
    zone = (
        max(coord_min_mm, min(coord_max_mm, support_coord_mm + offset_mm)),
        max(coord_min_mm, min(coord_max_mm, support_coord_mm + offset_mm + length_mm)),
    )
    zone = tuple(sorted(zone))

    def depth_fn(coord_mm):
        local_mm = coord_mm - support_coord_mm
        if local_mm < offset_mm - 1e-9 or local_mm > offset_mm + length_mm + 1e-9:
            return 0.0
        if length_mm <= 1e-9:
            return info["depth_mm"]
        return info["depth_mm"] * (1.0 - (local_mm - offset_mm) / length_mm)

    return zone, depth_fn, True


def make_connection_bevel_notch_depth_fn(info, support_coord_mm, member_length_mm):
    if not info["active"]:
        def depth_fn(_coord_mm):
            return 0.0
        return (support_coord_mm, support_coord_mm), depth_fn, False
    if info["reference"] == "axis_start":
        return make_end_referenced_bevel_notch_depth_fn(info, 0.0, 1.0, 0.0, member_length_mm)
    if info["reference"] == "axis_end":
        return make_end_referenced_bevel_notch_depth_fn(info, member_length_mm, -1.0, 0.0, member_length_mm)
    if info["reference"] in {"support_centerline", "support_inner_edge", "support_outer_edge"}:
        return make_support_referenced_bevel_notch_depth_fn(info, support_coord_mm, 0.0, member_length_mm)
    raise ValueError(f"Unsupported bevel notch reference for purlin check: {info['reference']}")


# ============================================================
# PALKKIEN POIKKILEIKKAUKSET  (Kerto-S LVL)
# ============================================================
# KP450x51  -> leveys b=51 mm, korkeus h=450 mm  (yksi palkki)
b1 = profile_b(member(_GEO, "beams", "beam.kp450.wall"))
h1 = profile_h(member(_GEO, "beams", "beam.kp450.wall"))

# 2xKP360x51 -> kaksi rinnakkaista palkkia: b=2×51=102 mm, h=360 mm
b2 = profile_b(member(_GEO, "beams", "beam.kp360x2"))
h2 = profile_h(member(_GEO, "beams", "beam.kp360x2"))

# Jäykkyydet taipuma- ja pistekuormalaskentaa varten
E_mean = 13800.0   # N/mm² Kerto-S E-moduuli (E0,mean)
I1     = b1 * h1**3 / 12.0     # mm⁴
I2     = b2 * h2**3 / 12.0     # mm⁴
EI1    = E_mean * I1            # N·mm²
EI2    = E_mean * I2            # N·mm²

# ============================================================
# TRIBUTÄÄRIALUEET (y-suunta, kohtisuora palkin akselille)
# ============================================================
# Seinäliitos ottaa edelleen kuorman alueelta y=0 ... 450 mm suoraan.
# KP450×51 @ y=900 mm kantaa suoran kattokaistan y=450 ... purlin.tuki.
# 2×KP360×51 kuormittuu katon osalta vain orsireaktioiden kautta.
trib1_start_mm = _BEAM1_CTX["direct_tributary_start_y_mm"]
trib1_end_mm   = _BEAM1_CTX["direct_tributary_end_y_mm"]
trib_w1        = _BEAM1_CTX["direct_tributary_width_m"]

trib2_start_mm = _KP360_CTX["direct_tributary_start_y_mm"]
trib2_end_mm   = _KP360_CTX["direct_tributary_end_y_mm"]
trib_w2        = _KP360_CTX["direct_tributary_width_m"]

# ============================================================
# PYSYVÄT KUORMAT
# ============================================================
# Kate + ruoteet + muut pysyvät (arvo teräsprofiililevy + alusrakenne)
gk_roofing = 0.20   # kN/m²

# Kertopuun tiheys (Kerto-S): 480 kg/m³ → γ = 4.71 kN/m³
gamma_lvl = 480.0 * 9.81 / 1000.0   # kN/m³ ≈ 4.71

g_beam1 = b1 / 1000.0 * h1 / 1000.0 * gamma_lvl   # kN/m (KP450x51 omapaino)
g_beam2 = b2 / 1000.0 * h2 / 1000.0 * gamma_lvl   # kN/m (2xKP360x51 omapaino)
gamma_c24 = 420.0 * 9.81 / 1000.0
g_purlin = ruode_b / 1000.0 * ruode_h / 1000.0 * gamma_c24  # kN/m (50x100 C24)

# Pysyvä suora viivakuorma palkeille (kate + palkki).
# Orsien kautta siirtyvät kuormat lisätään erillisinä pistekuormina.
gk1 = _BEAM1_CTX["gk_direct_kNm"]   # kN/m
gk2 = _KP360_CTX["gk_direct_kNm"]   # kN/m

# ============================================================
# LUMIKUORMA  (EN 1991-1-3, FI NA)
# ============================================================
# Kohde: 04330 Lahela (Tuusula) – noin 30 km rannikosta pohjoiseen.
# FI NA / YM asetus 6/16: Tuusula kuuluu vyöhyke II (Etelä-Suomi sisämaa).
# sk = 2.0 kN/m²  (EN 1991-1-3 FI NA, vyöhyke II)
# Muutoshistoria: vanha RakMk B1 (ennen v. 2010) käytti sk ≈ 2.2–2.5 kN/m²;
# nykyinen eurokoodi FI NA on laskenut arvoa sisämaassa 2.0:aan.
# Vuoden 2005 laskennassa käytetty 2.2 kN/m² vastasi silloista RakMk B1-arvoa.
sk   = 2.0    # kN/m² – Tuusula, FI NA vyöhyke II (EN 1991-1-3 / YM asetus 6/16)
# Katon muotokerroin μ1:
#   0° ≤ α ≤ 30° → μ1 = 0.8 (monotoninen katto)
mu1  = 0.8
Ce   = 1.0    # altistumiskerroin (normaali)
Ct   = 1.0    # lämpökerroin (kylmä katto)
s_roof = mu1 * Ce * Ct * sk   # kN/m² vaakatasolle projisoituna

# Lumikuorman suora viivakuorma palkeille
qk_snow1 = _BEAM1_CTX["q_snow_direct_kNm"]   # kN/m
qk_snow2 = _KP360_CTX["q_snow_direct_kNm"]   # kN/m

# ============================================================
# TUULIKUORMA  (EN 1991-1-4, FI NA)
# ============================================================
vb0     = 21.0    # m/s - perusnopeus, Eteläsuomi vyöhyke I
rho_air = 1.25    # kg/m³
# Maastoluokka II (tavanomainen avoin alue):
z0     = 0.05     # m
z_min  = 2.0      # m
_all_z_mm = [
    p["z"] for s in _GEO.get("surfaces", []) for p in s.get("polygon", []) if "z" in p
] + [
    m[k]["z"]
    for grp in _GEO["members"].values() for m in grp
    for k in ("axis_start", "axis_end", "base", "top")
    if k in m and isinstance(m[k], dict) and "z" in m[k]
]
# Pyöristetään ylöspäin 0.5 m tarkkuudella (EN 1991-1-4 §4.3.2)
# Pyöristetään ylöspäin 0.5 m tarkkuudella (EN 1991-1-4 §4.3.2)
z_ref = math.ceil(max(_all_z_mm) / 500.0) * 0.5   # m – korkein kohta geometriasta
kr     = 0.19 * (z0 / 0.05) ** 0.07    # = 0.19

# Rosoisuuskerroin cr(z)
cr_z = kr * math.log(max(z_ref, z_min) / z0)
# Turbulenssi-intensiteetti Iv(z)
Iv_z = 1.0 / math.log(max(z_ref, z_min) / z0)
# Tuulen keskinopeus
vm_z = cr_z * vb0
# Huippupainekorkeus qp(z)  [kN/m²]
qp_z = (1.0 + 7.0 * Iv_z) * 0.5 * rho_air * vm_z**2 / 1000.0

# Katos on geometrian mukaan rakennuksen kattolappeen jatke:
# surf.roof jatkuu rakennuksen puolelta (y < 0) katoksen räystäälle asti.
# Siksi ei käytetä vapaasti seisovan katoksen nettokertoimia, vaan
# rakennuksen kattopinnan ja räystäsulokkeen alapinnan envelopea.
wind_model = _ROOF_CTX["wind_model"]
cp_net_down = wind_model["cp_net_down"]   # alaspäin
cp_net_up = wind_model["cp_net_up"]       # ylöspäin / imu

w_wind_down = wind_model["w_wind_down_kNm2"]   # kN/m²
w_wind_up = wind_model["w_wind_up_kNm2"]       # kN/m²

qk_wind_down1 = _BEAM1_CTX["q_wind_down_direct_kNm"]
qk_wind_down2 = _KP360_CTX["q_wind_down_direct_kNm"]
qk_wind_up1   = _BEAM1_CTX["q_wind_up_direct_kNm"]
qk_wind_up2   = _KP360_CTX["q_wind_up_direct_kNm"]

# ============================================================
# KUORMAYHDISTELMÄT  (EN 1990 kaava 6.10)
# ============================================================
gammaG = 1.35    # pysyvien kuormien mitoituskerroin
gammaQ = 1.50    # muuttuvien kuormien mitoituskerroin
psi0_W = 0.6     # tuulikuorman yhdistelmäarvokerroin (lumi hallitsee)

# ULS - suora viivakuorma (alaspäin). Orsien reaktiot lisätään pistekuormina.
qd1 = _BEAM1_CTX["qd_uls_direct_kNm"]
qd2 = _KP360_CTX["qd_uls_direct_kNm"]

def point_loads_local_m(point_loads_abs_x_mm):
    return [((x_mm - float(min(_col_xs))) / 1000.0, p_kN) for x_mm, p_kN in aggregate_point_loads(point_loads_abs_x_mm)]


def purlin_reaction_point_loads(area_load_kNm2, self_factor, target_beam_id):
    point_loads_abs_x_mm = []
    for row in _PURLIN_CTX["members"]:
        line_load_kNm = area_load_kNm2 * float(row["area_load_factor_m"]) + self_factor * g_purlin
        reaction_inner_kN, reaction_outer_kN = uniform_line_member_support_reactions(
            0.0,
            float(row["member_length_mm"]),
            float(row["support_inner_s_mm"]),
            float(row["support_outer_s_mm"]),
            line_load_kNm,
        )
        if target_beam_id == "beam.kp450.y900":
            point_loads_abs_x_mm.append((row["reaction_inner_x_mm"], reaction_inner_kN))
        elif target_beam_id == "beam.kp360x2":
            point_loads_abs_x_mm.append((row["reaction_outer_x_mm"], reaction_outer_kN))
        else:
            raise ValueError(f"Unsupported target beam for purlin reactions: {target_beam_id}")
    return aggregate_point_loads(point_loads_abs_x_mm)


def simple_span_beam_response(udl_kNm, point_loads_abs_x_mm, deflection_udl_kNm, EI_Nmm2, deflection_point_loads_abs_x_mm=None):
    point_loads_abs_x_mm = aggregate_point_loads(point_loads_abs_x_mm)
    point_loads_local = point_loads_local_m(point_loads_abs_x_mm)
    x_md_m, moment_kNm = simple_span_combined_moment_max(L_m, udl_kNm, point_loads_local)
    shear_kN, shear_x_m = simple_span_max_shear(L_m, udl_kNm, point_loads_local)
    delta_point_loads_abs_x_mm = point_loads_abs_x_mm if deflection_point_loads_abs_x_mm is None else aggregate_point_loads(deflection_point_loads_abs_x_mm)
    delta_point_loads_local = point_loads_local_m(delta_point_loads_abs_x_mm)
    delta_mm, delta_x_mm = simple_span_max_deflection_mm(
        L_mm,
        EI_Nmm2,
        udl_kNm=deflection_udl_kNm,
        point_loads_local_m=delta_point_loads_local,
    )
    return {
        "point_loads_abs_x_mm": point_loads_abs_x_mm,
        "point_loads_local_m": point_loads_local,
        "M_gov": {"value_kNm": abs(moment_kNm), "x_m": x_md_m, "raw_value_kNm": moment_kNm},
        "V_abs": {"value_kN": abs(shear_kN), "x_m": shear_x_m, "raw_value_kN": shear_kN},
        "delta": {"value_mm": abs(delta_mm), "x_mm": delta_x_mm, "raw_value_mm": delta_mm},
        "added_reactions_kN": simple_span_point_reactions(L_m, point_loads_local),
    }


# Tukireaktiot (ULS) – palkin uloke ja katon räystäs mukana suoralle kattokaistalle.
_a_oh_m  = a_oh_left_mm / 1000.0     # m (symmetrinen)
_eave_m  = eave_left_mm / 1000.0     # m (symmetrinen)
q_roof_d = _ROOF_CTX["q_roof_uls_kNm2"]

beam1_purlin_point_loads_uls_abs_x_mm = aggregate_point_loads(_BEAM1_CTX["base_point_loads_uls_abs_x_mm"])
beam1_purlin_point_loads_sls_abs_x_mm = aggregate_point_loads(_BEAM1_CTX["base_point_loads_sls_abs_x_mm"])
beam1_purlin_point_loads_uplift_abs_x_mm = aggregate_point_loads(_BEAM1_CTX["base_point_loads_uplift_abs_x_mm"])
beam2_purlin_point_loads_uls_abs_x_mm = aggregate_point_loads(_KP360_CTX["base_point_loads_uls_abs_x_mm"])
beam2_purlin_point_loads_sls_abs_x_mm = aggregate_point_loads(_KP360_CTX["base_point_loads_sls_abs_x_mm"])
beam2_purlin_point_loads_uplift_abs_x_mm = aggregate_point_loads(_KP360_CTX["base_point_loads_uplift_abs_x_mm"])

R1_left = _BEAM1_CTX["reactions_uls_kN"][float(min(_col_xs))]
R1_right = _BEAM1_CTX["reactions_uls_kN"][float(max(_col_xs))]
R2_left = _KP360_CTX["reactions_uls_kN"][float(min(_col_xs))]
R2_right = _KP360_CTX["reactions_uls_kN"][float(max(_col_xs))]
R1 = max(R1_left, R1_right)
R2 = max(R2_left, R2_right)

beam1_response_uls = simple_span_beam_response(qd1, beam1_purlin_point_loads_uls_abs_x_mm, _BEAM1_CTX["q_sls_direct_kNm"], EI1, beam1_purlin_point_loads_sls_abs_x_mm)
beam2_response_uls = simple_span_beam_response(qd2, beam2_purlin_point_loads_uls_abs_x_mm, _KP360_CTX["q_sls_direct_kNm"], EI2, beam2_purlin_point_loads_sls_abs_x_mm)
qd1_eq = _BEAM1_CTX["q_eq_uls_kNm"]
qd2_eq = _KP360_CTX["q_eq_uls_kNm"]
qk_sls1_eq = _BEAM1_CTX["q_eq_sls_kNm"]
qk_sls2_eq = _KP360_CTX["q_eq_sls_kNm"]
qmin1_eq = _BEAM1_CTX["q_eq_uplift_kNm"]
qmin2_eq = _KP360_CTX["q_eq_uplift_kNm"]

# Taivutusmomentti / leikkaus / taipuma combined-kuormituksesta
Md1 = beam1_response_uls["M_gov"]["value_kNm"]
Md2 = beam2_response_uls["M_gov"]["value_kNm"]

# ============================================================
# PALKIN KANTOKYKY  (EN 1995-1-1, Kerto-S LVL)
# ============================================================
fm_k    = 44.0    # N/mm² - taivutuslujuuden ominaisarvo, Kerto-S
kmod    = 0.65    # kerroin: lumi = keskipitkäaikainen kuorma, SC3 (ulkorakenne)
gammaM  = 1.2     # materiaalikerroin LVL (EN 1995)
fm_d    = kmod * fm_k / gammaM   # N/mm²

# Taivutusvastus (vahva akseli)
W1 = b1 * h1**2 / 6.0   # mm³
W2 = b2 * h2**2 / 6.0   # mm³

# Taivutusmomenttikantokyky
MRd1 = fm_d * W1 / 1.0e6   # kNm
MRd2 = fm_d * W2 / 1.0e6   # kNm

# Käyttöaste taivutuksessa
eta1 = Md1 / MRd1 * 100.0   # %
eta2 = Md2 / MRd2 * 100.0   # %

# ============================================================
# LATERAALINURJAHDUS (LTB)  – EN 1995-1-1 §6.3.3
# ============================================================
# Kaltevuus on palkin JÄNTEEN SUUNNASSA (x-suunta).
# Poikkileikkaus pysyy pystysuorana (h pysty, b vaaka) kaikissa kohdissa.
# Pystysuorat kuormat aiheuttavat AINOASTAAN vahvan akselin taivutuksen:
#   M_y = Md  (vahva akseli, sama kuin yksinkertaisessa yksiaksiaalitarkistuksessa)
#   M_z = 0   (ei heikon akselin momenttia pystysuorasta kuormasta)
# → EN 1995-1-1 §6.2.4 biaksiaali EI sovellu tässä geometriassa.
# → Kriittinen tarkistus korkealle h/b-suhteelle: §6.3.3 LTB.
#
# Yksinkertaistettu kriittinen jännitys suorakaidepoikkileikkaukselle:
#   σ_m,crit = 0.78 × b² × E_0,05 / (h × L_eff)
#
# L_eff – taulukko 6.1, UDL yksinkertaisesti tuettu, kuorma yläreunassa
#   (puristusvyöhyke ylhäällä) → L_eff = 0.9 × L_buckle
#
# Suhteellinen hoikkuus ja k_crit:
#   λ_rel,m = √(fm,k / σ_m,crit)
#   λ ≤ 0.75          → k_crit = 1.0
#   0.75 < λ ≤ 1.4    → k_crit = 1.56 − 0.75λ
#   λ > 1.4           → k_crit = 1/λ²
# Mitoitusehto: σ_m,d / (k_crit × fm,d) ≤ 1.0

E_005_lvl  = 11600.0  # N/mm² – Kerto-S E_0,05 (Metsä Wood)

def ltb_check(b_mm, h_mm, E005, fm_k_val, fm_d_val, L_eff_mm, sigma_md_val):
    """EN 1995-1-1 §6.3.3 LTB suorakaidepoikkileikkaukselle."""
    sigma_crit = 0.78 * b_mm**2 * E005 / (h_mm * L_eff_mm)
    lam = math.sqrt(fm_k_val / sigma_crit)
    if lam <= 0.75:
        kcrit = 1.0
    elif lam <= 1.4:
        kcrit = 1.56 - 0.75 * lam
    else:
        kcrit = 1.0 / lam**2
    eta = sigma_md_val / (kcrit * fm_d_val) * 100.0
    return sigma_crit, lam, kcrit, eta

# Taivutusjännitykset (ULS, vahva akseli, yksiaksiaalinen)
sigma_md1 = Md1 * 1.0e6 / W1   # N/mm²  KP450×51
sigma_md2 = Md2 * 1.0e6 / W2   # N/mm²  2×KP360×51

# L_eff: UDL yksinkertaisesti tuettu, kuorma yläreunassa → L_eff = 0.9 × L_buckle
ltb_L_full  = 0.9 * L_mm           # ilman sivutukea: 0.9 × 6700 = 6030 mm
ltb_L_ruode = 0.9 * ruode_jako_mm  # ruodesivutuella: 0.9 × 900  =  810 mm

# ── KP450×51  (h/b = 8.8) ────────────────────────────────────
# #1 (y=0mm, seinässä kiinni pultein): seinä estää LTB-liikkeen → ei kriittinen
# #2 (y=900mm, LP225×90:n päällä): tarkistetaan ilman ja kanssa sivutukea
sc1_ns, lam1_ns, kc1_ns, eta_ltb1_ns = ltb_check(b1, h1, E_005_lvl, fm_k, fm_d, ltb_L_full,  sigma_md1)
sc1_r,  lam1_r,  kc1_r,  eta_ltb1_r  = ltb_check(b1, h1, E_005_lvl, fm_k, fm_d, ltb_L_ruode, sigma_md1)

# ── 2×KP360×51  (h/b = 3.5) ─────────────────────────────────
sc2_ns, lam2_ns, kc2_ns, eta_ltb2_ns = ltb_check(b2, h2, E_005_lvl, fm_k, fm_d, ltb_L_full,  sigma_md2)
sc2_r,  lam2_r,  kc2_r,  eta_ltb2_r  = ltb_check(b2, h2, E_005_lvl, fm_k, fm_d, ltb_L_ruode, sigma_md2)

# Sivutukivoima per ruode (kiinnitystarkistusta varten, arvio)
# Voima aiheutuu katon kallistuman vuoksi syntyvästä komponentista:
q_z1_ltb   = qd1_eq * math.sin(slope_rad)           # kN/m (vaaka-komponentti)
F_sivutuki = q_z1_ltb * (ruode_jako_mm / 1000.0)    # kN per ruode (ULS)

# Suurin sallittu UDL ennen taivutuskapasiteetin ylittymistä (100 %)
qd_max1 = MRd1 * 8.0 / (L_m**2 * moment_factor)   # kN/m
qd_max2 = MRd2 * 8.0 / (L_m**2 * moment_factor)   # kN/m

# ============================================================
# LEIKKAUSVOIMA  (SLS-tarkistus jätetään pois; ULS leikkaus)
# ============================================================
fv_k  = 4.5     # N/mm² Kerto-S leikkauslujuus
fv_d  = kmod * fv_k / gammaM   # N/mm²
# Leikkauskestävyys (suorakulmainen poikkileikkaus)
VRd1 = fv_d * (b1 * h1) / 1.5e3   # kN  (3/2 kerroin parabooliselle jakaumalle)
VRd2 = fv_d * (b2 * h2) / 1.5e3   # kN
Vd1  = beam1_response_uls["V_abs"]["value_kN"]   # kN
Vd2  = beam2_response_uls["V_abs"]["value_kN"]   # kN
eta_V1 = Vd1 / VRd1 * 100.0
eta_V2 = Vd2 / VRd2 * 100.0

# ============================================================
# TAIPUMA  (SLS, ominaisyhdistelmä)
# ============================================================
E_mean = 13800.0   # N/mm² Kerto-S E-moduuli (E0,mean)
I1     = b1 * h1**3 / 12.0     # mm⁴
I2     = b2 * h2**3 / 12.0     # mm⁴
EI1    = E_mean * I1            # N·mm²
EI2    = E_mean * I2            # N·mm²

# SLS ominaiskuorma (pysyvä + lumi, γ=1.0) – suora viivakuorma.
qk_sls1 = _BEAM1_CTX["q_sls_direct_kNm"]   # kN/m
qk_sls2 = _KP360_CTX["q_sls_direct_kNm"]   # kN/m
L_mm_eff = L_mm   # käytetään nettoväliä

def deflection_mm(q_kNm, L_mm_, EI_Nmm2):
    """5qL⁴/384EI, q kN/m → N/mm"""
    q_Nmm = q_kNm   # kN/m = N/mm
    return 5.0 * q_Nmm * L_mm_**4 / (384.0 * EI_Nmm2)

delta1 = beam1_response_uls["delta"]["value_mm"]   # mm
delta2 = beam2_response_uls["delta"]["value_mm"]   # mm
delta_lim = L_mm_eff / 300.0   # mm  (L/300)

# ============================================================
# LP225x90  –  PÄÄTYKANNAKE (liimapuu, seinältä pilarille)
# ============================================================
# LP225x90 kulkee y-suunnassa seinältä (y=0) pilarille (y=1675mm).
# Se toimii yksinkertaisesti tuettuna palkkina, joka kantaa
# KP450x51:n päätytukireaktion pistekuormana y=900mm kohdalla.
#
# Lisäksi se kantaa omaa tributäärialuettaan: puolet
# KP450x51-välistä (lähimpään LP:hen) on tässä approksimoitu nollaksi,
# koska LP225x90 on vain päädyssä (ei varsinainen runkopalkki).
# Pistekuorma P = KP450x51 tukireaktio (toinen pää).

_lp_beam = member(_GEO, "beams", "beam.lp225.x125")
_lp_check = check_existing_lp225_x125_combined(context=_EXISTING_CTX)
b_lp = profile_b(_lp_beam)   # mm – LP225x90 leveys
h_lp = profile_h(_lp_beam)   # mm – LP225x90 korkeus
L_lp_mm       = _EXISTING_CTX["lp225_x125"]["support_right_y_mm"]  # mm – jänneväli: seinä → pilari
L_lp_m        = L_lp_mm / 1000.0
a_lp_mm       = _EXISTING_CTX["lp225_x125"]["base_point_y_mm"]  # mm – pistekuorman sijainti seinältä
a_lp_m        = a_lp_mm / 1000.0

# KP450x51 tukireaktio (sis. palkin uloke ja katon räystäs)
P_kp1 = _EXISTING_CTX["lp225_x125"]["base_point_uls_kN"]  # kN (yksi päätytuki)

# LP225x90 tukireaktiot pistekuormasta P
R_seinä_lp = _lp_check["reactions_kN"][0.0]       # kN
R_pilari_lp = _lp_check["reactions_kN"][L_lp_mm]  # kN

# Maksimitaivutusmomentti (pistekuorma a:n kohdalla)
Md_lp = _lp_check["M_gov"]["value_kNm"]             # kNm

# LP225x90 kantokyky (liimapuu GL30c)
fm_k_lp  = 30.0    # N/mm² GL30c
kmod_lp  = 0.65    # lumikuorma, SC3 (ulkorakenne)
gammaM_lp = 1.25   # materiaalikerroin liimapuulle
fm_d_lp  = kmod_lp * fm_k_lp / gammaM_lp   # N/mm²
W_lp     = b_lp * h_lp**2 / 6.0            # mm³
MRd_lp   = fm_d_lp * W_lp / 1.0e6         # kNm
eta_lp   = Md_lp / MRd_lp * 100.0         # %

# Leikkaus LP225x90
fv_k_lp  = 3.5     # N/mm² GL30c
fv_d_lp  = kmod_lp * fv_k_lp / gammaM_lp
VRd_lp   = fv_d_lp * (b_lp * h_lp) / 1.5e3
Vd_lp    = abs(_lp_check["V_abs"]["value_kN"])
eta_V_lp = Vd_lp / VRd_lp * 100.0

# ============================================================
# LISÄKAPASITEETTI – 2×KP360×51 uutta ulompaa katosta varten
# ============================================================
# Periaate:
#   Uusi ulompi palkki kulkee x-suunnassa (seinän suuntaisesti) samoin kuin
#   2×KP360×51. Yhdistämällä nämä poikittaisilla rimoilla/palkeilla y-suunnassa
#   voidaan 2×KP360×51 toimia välipisteenä, jolloin uuden palkin jänneväli
#   lyhenee: 6700mm → L_new = L / (n_tukia + 1).
#
#   Olemassa oleva UDL + lisäpistekuorma → superposiitio.
#   Molemmat ovat maksimissaan L/2-kohdassa → summaus suoraan.
#
# Käyttämätön kapasiteetti:
delta_MRd2 = MRd2 - Md2    # kNm

# ── Tapaus A: 1 välituki (x = L/2 = 3350mm) ──────────────
# ΔM = P * L/4  →  P = ΔM * 4 / L
P_A    = delta_MRd2 * 4.0 / L_m    # kN sallittu pistekuorma
span_A = L_mm / 2.0                 # mm uusi jänneväli

# ── Tapaus B: 2 välitukea (x = L/3 ja 2L/3) ─────────────
# 2 symmetristä pistekuormaa P: M_max(L/2) = P*L/3
# → P per tuki = ΔM * 3 / L   (molemmat kuormat yhtä suuret)
P_B_each  = delta_MRd2 * 3.0 / L_m # kN per tukipiste
P_B_total = 2.0 * P_B_each          # kN yhteensä
span_B    = L_mm / 3.0              # mm uusi jänneväli

# ── Uuden palkin mitoituskapasiteetti lyhennetyllä jännevälillä ──
# Käytetään samaa kertopuupalkkia 2×KP360×51 vertailukohtana.
# Uuden palkin sallittu UDL (yksinkertaisesti tuettu, MRd2 täysin vapaana):
#   tapaus A: q_max = 8*MRd2 / span_A²
#   tapaus B: q_max = 8*MRd2 / span_B²
q_new_A = MRd2 * 8.0 / (span_A / 1000.0)**2   # kN/m (jos sama 2×KP360×51)
q_new_B = MRd2 * 8.0 / (span_B / 1000.0)**2   # kN/m

# Vastaava kattopinta-alakuorma uudelle palkille (arvio trib.leveydellä 0.9m)
trib_new = 0.9   # m (arvio, tarkenna geometrian mukaan)
q_roof_new_A = q_new_A / trib_new
q_roof_new_B = q_new_B / trib_new

# ============================================================
# TÄYDENTÄVÄT KUORMATARKISTUKSET
# ============================================================

# ── 1) Huoltokuorma – EN 1991-1-1 Kategoria H katto ─────
# Qk = 1.0 kN (pistekuorma, missä tahansa kohdassa)
# qk_H = 0.4 kN/m² (hajakuorma)
# Huolto EI yhdisty täysimääräiseen lumikuormaan:
#   Yhdistelmä: 1.35*G + 1.5*Qk_huolto + 1.5*ψ0_snow*Sk_distrib
# Yhdistelmäkertoimet FI NA: ψ0_snow = 0.7, ψ0_wind = 0.6
Qk_huolto   = 1.0     # kN, pistekuorma
qk_H        = 0.4     # kN/m², hajakuorma (Kategoria H)
psi0_snow_accomp = 0.7   # lumi liitännäiskuormana

# KP450x51 – huolto dominant:
# UDL-osa (pysyvä + lumi ψ0):
q_g_s_psi1  = 1.35 * gk1 + 1.5 * psi0_snow_accomp * qk_snow1
q_H_1 = 1.5 * qk_H * trib_w1               # kN/m
beam1_huolto_purlin_point_loads_abs_x_mm = purlin_reaction_point_loads(
    1.35 * gk_roofing + 1.5 * psi0_snow_accomp * s_roof + 1.5 * qk_H,
    gammaG,
    "beam.kp450.y900",
)
beam1_huolto_response = simple_span_beam_response(
    q_g_s_psi1 + q_H_1,
    [*beam1_huolto_purlin_point_loads_abs_x_mm, (float(min(_col_xs)) + 0.5 * L_mm, 1.5 * Qk_huolto)],
    q_g_s_psi1 + q_H_1,
    EI1,
)
M_huolto1 = beam1_huolto_response["M_gov"]["value_kNm"]
eta_huolto1 = M_huolto1 / MRd1 * 100.0

# 2×KP360×51 – huolto dominant:
q_g_s_psi2  = 1.35 * gk2 + 1.5 * psi0_snow_accomp * qk_snow2
q_H_2 = 1.5 * qk_H * trib_w2
beam2_huolto_purlin_point_loads_abs_x_mm = purlin_reaction_point_loads(
    1.35 * gk_roofing + 1.5 * psi0_snow_accomp * s_roof + 1.5 * qk_H,
    gammaG,
    "beam.kp360x2",
)
beam2_huolto_response = simple_span_beam_response(
    q_g_s_psi2 + q_H_2,
    [*beam2_huolto_purlin_point_loads_abs_x_mm, (float(min(_col_xs)) + 0.5 * L_mm, 1.5 * Qk_huolto)],
    q_g_s_psi2 + q_H_2,
    EI2,
)
M_huolto2 = beam2_huolto_response["M_gov"]["value_kNm"]
eta_huolto2 = M_huolto2 / MRd2 * 100.0

# ── 2) Tuulen nostokuorma – minimikapasiteetti ───────────
# EN 1990 kaava 6.10 min: 0.9*Gk + 1.5*Wk (nosto ylöspäin)
# Jos qmin < 0 → tukireaktio on nostava → kiinnitys tarvitaan
gammaG_min = 0.9   # suotuisa pysyvä
qmin1 = _BEAM1_CTX["q_uplift_direct_kNm"]   # kN/m (neg = nosto)
qmin2 = _KP360_CTX["q_uplift_direct_kNm"]
q_roof_min = _ROOF_CTX["q_roof_uplift_kNm2"]   # kN/m²

# Nostoreaktio tukipisteessä (sis. uloke + räystäs + orsireaktiot)
R_uplift1_left = _BEAM1_CTX["reactions_uplift_kN"][float(min(_col_xs))]
R_uplift1_right = _BEAM1_CTX["reactions_uplift_kN"][float(max(_col_xs))]
R_uplift2_left = _KP360_CTX["reactions_uplift_kN"][float(min(_col_xs))]
R_uplift2_right = _KP360_CTX["reactions_uplift_kN"][float(max(_col_xs))]
R_uplift1 = min(R_uplift1_left, R_uplift1_right)
R_uplift2 = min(R_uplift2_left, R_uplift2_right)

# ============================================================
# KOKONAISPILARIKUORMAT  (katos + koko terassi)
# ============================================================
KATOS_TOTAL_COLUMN_LOADS = calculate_katos_total_column_loads()
gk_hollow_slab = KATOS_TOTAL_COLUMN_LOADS["gk_hollow_slab_kNm2"]
gk_hollow_slab_allow = KATOS_TOTAL_COLUMN_LOADS["gk_hollow_slab_allow_kNm2"]
gk_floor_cast = KATOS_TOTAL_COLUMN_LOADS["gk_floor_cast_kNm2"]
qk_terrace_live = KATOS_TOTAL_COLUMN_LOADS["qk_terrace_live_kNm2"]
qk_hollow_slab_allow = KATOS_TOTAL_COLUMN_LOADS["qk_hollow_slab_allow_kNm2"]
hollow_beam_self_kNm = KATOS_TOTAL_COLUMN_LOADS["hollow_beam_self_kNm"]
outer_beam_self_kNm = KATOS_TOTAL_COLUMN_LOADS["outer_beam_self_kNm"]
outer_beam_total_self_kN = KATOS_TOTAL_COLUMN_LOADS["outer_beam_total_self_kN"]
outer_beam_count = KATOS_TOTAL_COLUMN_LOADS["outer_beam_count"]
column_case_totals = KATOS_TOTAL_COLUMN_LOADS["case_totals"]
column_output_order = KATOS_TOTAL_COLUMN_LOADS["column_output_order"]
column_display = KATOS_TOTAL_COLUMN_LOADS["column_display"]
column_group_label = KATOS_TOTAL_COLUMN_LOADS["column_group_label"]
hollow_slabs_sls = KATOS_TOTAL_COLUMN_LOADS["hollow_case_results"]["SLS"]
hollow_cast_util_pct = gk_floor_cast / gk_hollow_slab_allow * 100.0 if gk_hollow_slab_allow > 1e-9 else float("inf")
hollow_live_util_pct = qk_terrace_live / qk_hollow_slab_allow * 100.0 if qk_hollow_slab_allow > 1e-9 else float("inf")
critical_hollow_slab = max(
    hollow_slabs_sls,
    key=lambda row: max(
        abs(row["M_pos"]["value_kNm"]),
        abs(row["M_neg"]["value_kNm"]),
        abs(row["V_abs"]["value_kN"]),
    ),
)

# ── 3) Lumikuorman epätasainen jakautuma (EN 1991-1-3 §6.2) ──
# Yksilappinen katto: epätasaiset tapaukset eivät pääsääntöisesti koske
# yksinkertaista yksilappeista katosta (ei murtumisvaaraa toiselle lappee).
# Huomioitava vain jos rakenne on U- tai L-muotoinen tai vieressä korkeampi rak.
# → Merkitään tiedoksi, ei lasketa erikseen.

# ============================================================
# ORSITARKISTUKSET  (50x100 C24, lovi huomioitu)
# ============================================================
E_mean_C24 = 11000.0
fm_d_C24 = 0.65 * 24.0 / 1.3
fv_d_C24 = 0.65 * 4.0 / 1.3
PURLIN_ANALYSIS_STEP_MM = 100.0
PURLIN_NOTCH_SAMPLE_STEP_MM = 1.0
PURLIN_HUOLTO_POINT_CASE = 1.5 * Qk_huolto
PURLIN_HUOLTO_AREA_CASE = 1.35 * gk_roofing + 1.5 * psi0_snow_accomp * s_roof + 1.5 * qk_H
PURLIN_RAYST_NOTE = "jatkuva KP450-sivutuki"


def purlin_governing_eta(result):
    notch_eta = 0.0 if result["notch"] is None else result["notch"]["eta_gov"]["value_pct"]
    return max(result["eta_M"], result["eta_V"], notch_eta)


def analyse_purlin_member_case(row, member_obj, roof_area_kNm2, self_factor, point_loads_kN=None):
    if point_loads_kN is None:
        point_loads_kN = []
    member_length_mm = float(row["member_length_mm"])
    support_inner_s_mm = float(row["support_inner_s_mm"])
    support_outer_s_mm = float(row["support_outer_s_mm"])
    member_b_mm = profile_b(member_obj)
    member_h_mm = profile_h(member_obj)
    member_props = member_rect_props(member_b_mm, member_h_mm, member_section_rotation_deg(member_obj))
    member_MRd_kNm = fm_d_C24 * member_props["W_mm3"] / 1.0e6
    member_VRd_kN = fv_d_C24 * member_props["A_mm2"] / 1.5e3

    inner_notch_info = bevel_notch_info(row["inner_connection_id"])
    outer_notch_info = bevel_notch_info(row["outer_connection_id"])
    inner_zone_mm, inner_depth_fn, inner_notch_active = make_connection_bevel_notch_depth_fn(
        inner_notch_info,
        support_inner_s_mm,
        member_length_mm,
    )
    outer_zone_mm, outer_depth_fn, outer_notch_active = make_connection_bevel_notch_depth_fn(
        outer_notch_info,
        support_outer_s_mm,
        member_length_mm,
    )
    depth_functions = []
    if inner_notch_active:
        depth_functions.append(inner_depth_fn)
    if outer_notch_active:
        depth_functions.append(outer_depth_fn)
    section_h_fn = combined_section_h(member_h_mm, depth_functions)

    node_points = [0.0, member_length_mm, support_inner_s_mm, support_outer_s_mm]
    if inner_notch_active:
        node_points.extend(inner_zone_mm)
    if outer_notch_active:
        node_points.extend(outer_zone_mm)
    node_points.extend(x_mm for x_mm, _ in point_loads_kN)
    nodes_mm = refine_nodes_mm(node_points, PURLIN_ANALYSIS_STEP_MM)

    line_load_kNm = roof_area_kNm2 * float(row["area_load_factor_m"]) + self_factor * g_purlin
    uniform_loads = uniform_loads_for_nodes(nodes_mm, line_load_kNm / 1000.0)
    EI_by_segment_Nmm2 = [
        E_mean_C24
        * member_rect_props(
            member_b_mm,
            section_h_fn(0.5 * (a_mm + b_mm)),
            member_section_rotation_deg(member_obj),
        )["I_mm4"]
        for a_mm, b_mm in zip(nodes_mm, nodes_mm[1:])
    ]
    response = beam_solver(
        nodes_mm,
        [support_inner_s_mm, support_outer_s_mm],
        point_loads_kN=point_loads_kN,
        uniform_loads_kN_per_mm=uniform_loads,
        EI_by_segment_Nmm2=EI_by_segment_Nmm2,
    )
    internal = sample_internal_forces(response["elements"])
    delta = sample_max_deflection_mm(response["nodes_mm"], response["disp_mm"], response["rot_rad"], step_mm=2.0)
    moment_gov = governing_moment(internal)

    notch_candidates = []
    if inner_notch_active:
        notch_candidates.append(
            {
                "label": "sisalovi",
                **sample_net_section_utilization(
                    response["elements"],
                    member_obj,
                    section_h_fn,
                    fm_d_C24,
                    fv_d_C24,
                    inner_zone_mm[0],
                    inner_zone_mm[1],
                    step_mm=PURLIN_NOTCH_SAMPLE_STEP_MM,
                ),
            }
        )
    if outer_notch_active:
        notch_candidates.append(
            {
                "label": "ulkolovi",
                **sample_net_section_utilization(
                    response["elements"],
                    member_obj,
                    section_h_fn,
                    fm_d_C24,
                    fv_d_C24,
                    outer_zone_mm[0],
                    outer_zone_mm[1],
                    step_mm=PURLIN_NOTCH_SAMPLE_STEP_MM,
                ),
            }
        )
    notch = max(notch_candidates, key=lambda item: item["eta_gov"]["value_pct"]) if notch_candidates else None
    min_h = sample_min_section_height_mm(section_h_fn, 0.0, member_length_mm, step_mm=PURLIN_NOTCH_SAMPLE_STEP_MM)

    return {
        "id": row["id"],
        "kind": row["kind"],
        "member_length_mm": member_length_mm,
        "span_mm": support_outer_s_mm - support_inner_s_mm,
        "overhang_mm": max(0.0, member_length_mm - support_outer_s_mm),
        "tributary_width_m": float(row["tributary_width_m"]),
        "tributary_area_m2": float(row["tributary_area_m2"]),
        "area_load_factor_m": float(row["area_load_factor_m"]),
        "line_load_kNm": line_load_kNm,
        "point_load_total_kN": sum(p_kN for _, p_kN in point_loads_kN),
        "M_gov": moment_gov,
        "V_abs": internal["V_abs"],
        "delta": delta,
        "delta_lim_mm": max(1e-9, (support_outer_s_mm - support_inner_s_mm) / 300.0),
        "MRd_kNm": member_MRd_kNm,
        "VRd_kN": member_VRd_kN,
        "R_inner_kN": response["reactions_kN"][support_inner_s_mm],
        "R_outer_kN": response["reactions_kN"][support_outer_s_mm],
        "eta_M": moment_gov["value_kNm"] / member_MRd_kNm * 100.0,
        "eta_V": abs(internal["V_abs"]["value_kN"]) / member_VRd_kN * 100.0,
        "notch": notch,
        "inner_notch_active": inner_notch_active,
        "outer_notch_active": outer_notch_active,
        "inner_notch_zone_mm": inner_zone_mm if inner_notch_active else None,
        "outer_notch_zone_mm": outer_zone_mm if outer_notch_active else None,
        "h_net_min_mm": min_h["h_mm"],
        "h_net_min_x_mm": min_h["x_mm"],
    }


def analyse_purlin_design(row):
    member_obj = _PURLIN_MEMBERS[row["id"]]
    midspan_s_mm = float(row["support_inner_s_mm"]) + 0.5 * (float(row["support_outer_s_mm"]) - float(row["support_inner_s_mm"]))
    cases = {
        "LUMI": analyse_purlin_member_case(row, member_obj, q_roof_d, gammaG),
        "HUOLTO": analyse_purlin_member_case(
            row,
            member_obj,
            PURLIN_HUOLTO_AREA_CASE,
            gammaG,
            point_loads_kN=[(midspan_s_mm, PURLIN_HUOLTO_POINT_CASE)],
        ),
        "SLS": analyse_purlin_member_case(row, member_obj, gk_roofing + s_roof, 1.0),
        "UPLIFT": analyse_purlin_member_case(row, member_obj, q_roof_min, 0.9),
    }
    governing_case = max(("LUMI", "HUOLTO"), key=lambda case_key: purlin_governing_eta(cases[case_key]))
    governing = dict(cases[governing_case])
    governing["case_key"] = governing_case
    governing["eta_gov"] = purlin_governing_eta(governing)
    governing["sls_delta_mm"] = abs(cases["SLS"]["delta"]["value_mm"])
    governing["sls_case_key"] = "SLS"
    governing["uplift_inner_kN"] = cases["UPLIFT"]["R_inner_kN"]
    governing["uplift_outer_kN"] = cases["UPLIFT"]["R_outer_kN"]
    return governing


purlin_design_results = [analyse_purlin_design(row) for row in _PURLIN_CTX["members"]]
purlin_design_results_main = [row for row in purlin_design_results if row["kind"] == "main"]
purlin_design_results_diag = [row for row in purlin_design_results if row["kind"] == "diag"]
critical_purlin = max(purlin_design_results, key=lambda row: row["eta_gov"]) if purlin_design_results else None

# ============================================================
# TULOSTUS
# ============================================================
W  = 60
dw = "=" * W

print(dw)
print("  KATOKSEN KUORMITUSLASKENTA – ETELÄSUOMI")
print("  EN 1990 / EN 1991-1-3 / EN 1991-1-4 / EN 1995-1-1")
print(dw)

print("\n── GEOMETRIA ──────────────────────────────────────────")
print(f"  Seinän leveys                  {wall_width} mm")
print(f"  Pilarin koko                   {pillar_size}×{pillar_size} mm")
print(f"  Jänneväli (c/c)                {L_mm:.0f} mm  ({L_m:.3f} m)")
print(f"  Katon kaltevuus (seinän suunt.){slope_deg:.0f}°")
print(f"  Katto ulkonee seinästä         {roof_edge_y} mm")
print(f"  Räystäs (x-suunta)             {eave_left_mm:.0f} + {eave_right_mm:.0f} mm")
print(f"  Palkin uloke yli tukien         {a_oh_left_mm:.0f} + {a_oh_right_mm:.0f} mm")
print(f"  KP450×51 sijainti              {beam1_y} mm seinästä")
print(f"  2×KP360×51 sijainti            {beam2_y} mm seinästä (tolpat)")

print("\n── TRIBUTÄÄRIALUEET ───────────────────────────────────")
print(f"  KP450×51 suora kattokaista     y = {trib1_start_mm:.0f} ... {trib1_end_mm:.0f} mm  →  b = {trib_w1*1000:.1f} mm")
print(f"  2×KP360×51 suora kattokaista   ei suoraa kattokaistaa  →  b = {trib_w2*1000:.1f} mm")
print(
    f"  50×100 kuormaorret             {_PURLIN_CTX['count']} kpl "
    f"({ _PURLIN_CTX['count_main']} pääortta + {_PURLIN_CTX['count_diag']} vino-ortta), "
    f"tuet y = {_PURLIN_CTX['support_inner_y_mm']:.0f} / {_PURLIN_CTX['support_outer_y_mm']:.0f} mm"
)
if _RAYST_CTX["count"]:
    print(f"  KP450:n sivujatkeet (rayst)    {_RAYST_CTX['count']} kpl, jatkuvana KP450-palkkien kyljissä")

print("\n── LUMIKUORMA ─────────────────────────────────────────")
print(f"  Maanpintaominaiskuorma  sk     {sk:.1f} kN/m²  (Eteläsuomi)")
print(f"  Muotokerroin μ1 (α={slope_deg:.0f}°)     {mu1:.1f}  (0°–30° katto)")
print(f"  Lumikuorma katolla  s          {s_roof:.2f} kN/m²")
print(f"  KP450×51   q_lumi, suora       {qk_snow1:.3f} kN/m")
print(f"  2×KP360×51 q_lumi, suora       {qk_snow2:.3f} kN/m")

print("\n── TUULIKUORMA ────────────────────────────────────────")
print(f"  Perusnopeus vb0                {vb0:.0f} m/s  (vyöhyke I, Eteläsuomi)")
print(f"  Referenssikorkeus              {z_ref:.0f} m")
print(f"  cr({z_ref:.0f})                        {cr_z:.3f}")
print(f"  Iv({z_ref:.0f})                        {Iv_z:.3f}")
print(f"  Keskinopeus vm                 {vm_z:.1f} m/s")
print(f"  Huippupainekerroin qp({z_ref:.0f})      {qp_z:.3f} kN/m²")
print(f"  Tuulimalli                     {wind_model['description']}")
print(f"  Peruste                        {wind_model['basis']}")
print(
    f"  Nettopainekerroin alaspäin     cp,net = {cp_net_down:.2f}  "
    f"(cpe_top {wind_model['cpe_top_down']:+.2f} - cpe_under {wind_model['cpe_under_down']:+.2f})"
)
print(
    f"  Nettopainekerroin ylöspäin     cp,net = {cp_net_up:.2f}  "
    f"(cpe_top {wind_model['cpe_top_uplift']:+.2f} - cpe_under {wind_model['cpe_under_uplift']:+.2f})"
)
print(f"  Tuulikuorma katolla (alas)     {w_wind_down:.3f} kN/m²")
print(f"  Tuulikuorma katolla (ylös)     {w_wind_up:.3f} kN/m²")
print(f"  KP450×51   q_tuuli, suora      {qk_wind_down1:.3f} kN/m")
print(f"  2×KP360×51 q_tuuli, suora      {qk_wind_down2:.3f} kN/m")

print("\n── PYSYVÄT KUORMAT ────────────────────────────────────")
print(f"  Kate + ruoteet  gk             {gk_roofing:.2f} kN/m²")
print(f"  Kertopuun tiheys               {gamma_lvl:.2f} kN/m³")
print(f"  C24-orren tiheys               {gamma_c24:.2f} kN/m³")
print(f"  KP450×51   omapaino            {g_beam1:.3f} kN/m")
print(f"  2×KP360×51 omapaino            {g_beam2:.3f} kN/m")
print(f"  50×100 orsi omapaino           {g_purlin:.3f} kN/m")
print(f"  KP450×51   Gk, suora           {gk1:.3f} kN/m")
print(f"  2×KP360×51 Gk, suora           {gk2:.3f} kN/m")

print("\n── MITOITUSKUORMAT ULS  (1.35G + 1.5S + 1.5·0.6·W) ──")
print(f"  KP450×51   qd, suora           {qd1:.3f} kN/m")
print(f"  2×KP360×51 qd, suora           {qd2:.3f} kN/m")
print(f"  KP450×51   q_eq, combined      {qd1_eq:.3f} kN/m")
print(f"  2×KP360×51 q_eq, combined      {qd2_eq:.3f} kN/m")

print("\n── ORSIREAKTIOT PÄÄPALKEILLE ULS ──────────────────────")
print(f"  KP450×51   pistekuormat        {format_point_loads(beam1_purlin_point_loads_uls_abs_x_mm)}")
print(f"  2×KP360×51 pistekuormat        {format_point_loads(beam2_purlin_point_loads_uls_abs_x_mm)}")

print("\n── ORSITARKISTUKSET 50×100 C24 (LOVI HUOMIOITU) ──────")
print("  ID               tyyppi tapaus  b_trib   q_avg   span  uloke  h_net      Md    η_M    η_V η_lovi   δ_sls")
print("  ---------------- ------ ------ ------- ------- ------ ------ ------ ------- ------ ------ ------- -------")
for row in purlin_design_results:
    notch_eta = 0.0 if row["notch"] is None else row["notch"]["eta_gov"]["value_pct"]
    type_label = "vino" if row["kind"] == "diag" else "paa"
    print(
        f"  {row['id']:<16} {type_label:<6} {row['case_key']:<6} "
        f"{row['tributary_width_m']*1000:>7.0f} {row['line_load_kNm']:>7.3f} "
        f"{row['span_mm']:>6.0f} {row['overhang_mm']:>6.0f} {row['h_net_min_mm']:>6.1f} "
        f"{row['M_gov']['value_kNm']:>7.2f} {row['eta_M']:>6.1f}% {row['eta_V']:>6.1f}% "
        f"{notch_eta:>7.1f}% {row['sls_delta_mm']:>7.2f}"
    )
if critical_purlin is not None:
    critical_notch_eta = 0.0 if critical_purlin["notch"] is None else critical_purlin["notch"]["eta_gov"]["value_pct"]
    print(
        f"  Governing orsi: {critical_purlin['id']} ({critical_purlin['case_key']})  "
        f"eta = {critical_purlin['eta_gov']:.1f}%  "
        f"(M {critical_purlin['eta_M']:.1f}%, V {critical_purlin['eta_V']:.1f}%, lovi {critical_notch_eta:.1f}%)"
    )
if _RAYST_CTX["count"]:
    print(f"  Rayst-orret: {_RAYST_CTX['count']} kpl, kuorma siirtyy jatkuvana KP450-sivutukena -> ei erillista jannepalkkitarkistusta.")

print("\n── TUKIREAKTIOT ULS  (suora kaista + orret) ───────────")
print(f"  KP450×51   Rd,tuki vas/oik     {R1_left:.2f} / {R1_right:.2f} kN")
print(f"  2×KP360×51 Rd,tuki vas/oik     {R2_left:.2f} / {R2_right:.2f} kN")

print("\n── TAIVUTUSMOMENTTI  combined-kuormituksesta ──────────")
print(f"  Kaltevuuskorjaus (moment_factor)  {moment_factor:.4f}  (kaltevuus jänteen suunnassa → 1.0)")
print(f"  KP450×51   Md                  {Md1:.2f} kNm")
print(f"  2×KP360×51 Md                  {Md2:.2f} kNm")

print("\n── PALKIN KANTOKYKY (Kerto-S, taivutus) ───────────────")
print(f"  fm,k = {fm_k:.0f} N/mm²,  kmod = {kmod},  γM = {gammaM}  →  fm,d = {fm_d:.1f} N/mm²")
print(f"  KP450×51   W  = {W1/1e3:.0f} cm³   MRd = {MRd1:.2f} kNm")
print(f"  2×KP360×51 W  = {W2/1e3:.0f} cm³   MRd = {MRd2:.2f} kNm")

print(f"\n── LATERAALINURJAHDUS §6.3.3  (EN 1995-1-1) ───────────────────────────────")
print(f"  Kaltevuus jänteen suunnassa (x) → M_z = 0 pystysuorasta kuormasta.")
print(f"  §6.2.4 biaksiaali EI sovellu. Kriittinen tarkistus: §6.3.3 LTB.")
print(f"  E_0,05 = {E_005_lvl:.0f} N/mm²  (Kerto-S).  L_eff = 0.9 × L_buckle (UDL, yläreunan kuorma)")
print()
print(f"  KP450×51 #2 (y=900mm)   h/b = {h1/b1:.1f}   σ_m,d = {sigma_md1:.2f} N/mm²")
print(f"    Ilman sivutukea  L_eff={ltb_L_full:.0f}mm: σ_crit={sc1_ns:.2f}  λ={lam1_ns:.3f}  k_crit={kc1_ns:.3f}  η={eta_ltb1_ns:.1f}%  {'OK ✓' if eta_ltb1_ns<=100 else '*** YLITTYY – SIVUTUKI PAKOLLINEN ***'}")
print(f"    Ruodesivutuella  L_eff={ltb_L_ruode:.0f}mm:  σ_crit={sc1_r:.2f}  λ={lam1_r:.3f}  k_crit={kc1_r:.3f}  η={eta_ltb1_r:.1f}%  {'OK ✓' if eta_ltb1_r<=100 else '*** YLITTYY ***'}")
print()
print(f"  2×KP360×51 (y=1675mm)  h/b = {h2/b2:.1f}   σ_m,d = {sigma_md2:.2f} N/mm²")
print(f"    Ilman sivutukea  L_eff={ltb_L_full:.0f}mm: σ_crit={sc2_ns:.2f}  λ={lam2_ns:.3f}  k_crit={kc2_ns:.3f}  η={eta_ltb2_ns:.1f}%  {'OK ✓' if eta_ltb2_ns<=100 else '*** YLITTYY – SIVUTUKI PAKOLLINEN ***'}")
print(f"    Ruodesivutuella  L_eff={ltb_L_ruode:.0f}mm:  σ_crit={sc2_r:.2f}  λ={lam2_r:.3f}  k_crit={kc2_r:.3f}  η={eta_ltb2_r:.1f}%  {'OK ✓' if eta_ltb2_r<=100 else '*** YLITTYY ***'}")
print(f"  *** Ruoteiden kiinnitys palkkeihin on kriittinen LTB-tuelle! ***")
print(f"  Sivutukivoima per ruode (arvio): {F_sivutuki:.2f} kN (ULS) → kiinnitys tarkistettava")

print("\n── LEIKKAUSVOIMATARKISTUS  (Vd = palkin sisäinen leikkaus) ──")
print(f"  fv,d = {fv_d:.2f} N/mm²")
print(f"  KP450×51   Vd,span = {Vd1:.2f} kN   VRd = {VRd1:.2f} kN   η = {eta_V1:.1f}%")
print(f"  2×KP360×51 Vd,span = {Vd2:.2f} kN   VRd = {VRd2:.2f} kN   η = {eta_V2:.1f}%")

print("\n── TAIPUMA SLS  (δ_lim = L/300 = {:.0f} mm) ─────────────".format(delta_lim))
print(f"  EI  KP450×51   = {EI1/1e9:.2f} kN·m²")
print(f"  EI  2×KP360×51 = {EI2/1e9:.2f} kN·m²")
print(f"  KP450×51   δ = {delta1:.1f} mm   {'OK ✓' if delta1 <= delta_lim else 'YLITTYY ✗'}")
print(f"  2×KP360×51 δ = {delta2:.1f} mm   {'OK ✓' if delta2 <= delta_lim else 'YLITTYY ✗'}")

print("\n── LP225×90  PÄÄTYKANNAKE (liimapuu GL30c) ────────────")
print(f"  Jänneväli seinä → pilari       {L_lp_mm:.0f} mm")
print(f"  KP450×51 tukireaktio P         {P_kp1:.2f} kN  @ y={a_lp_mm:.0f} mm (sis. räystäs)")
print(f"  Tukireaktio seinäpäässä        {R_seinä_lp:.2f} kN")
print(f"  Tukireaktio pilaripäässä       {R_pilari_lp:.2f} kN")
print(f"  Taivutusmomentti Md            {Md_lp:.2f} kNm")
print(f"  fm,d = {fm_d_lp:.1f} N/mm²  (GL30c, kmod={kmod_lp}, γM={gammaM_lp})")
print(f"  W  = {W_lp/1e3:.0f} cm³   MRd = {MRd_lp:.2f} kNm   η_M = {eta_lp:.1f}%  {'OK ✓' if eta_lp <= 100 else '*** YLITTYY ***'}")
print(f"  Vd = {Vd_lp:.2f} kN   VRd = {VRd_lp:.2f} kN          η_V = {eta_V_lp:.1f}%  {'OK ✓' if eta_V_lp <= 100 else '*** YLITTYY ***'}")

print("\n── KOKONAISPILARIKUORMAT  (KATOS + TERASSI) ───────────")
print(f"  Ontelolaatta saumattuna h=150  {gk_hollow_slab:.2f} kN/m²  = {hollow_beam_self_kNm:.3f} kN/m / laatta")
print(f"  Pintavalu 60 mm                {gk_floor_cast:.2f} kN/m²")
print(f"  Terassin hyötykuorma           {qk_terrace_live:.2f} kN/m²")
if outer_beam_count == 1:
    print(f"  Alapalkki 350×300              {outer_beam_self_kNm:.3f} kN/m")
else:
    print(
        f"  Alapalkit {outer_beam_count}×350×300            "
        f"{outer_beam_total_self_kN / outer_beam_count:.2f} kN / palkki  = {outer_beam_self_kNm:.3f} kN/m"
    )
print(f"  Betonipilari 250×250           {0.25 * 0.25 * KATOS_TOTAL_COLUMN_LOADS['gamma_concrete_kNm3']:.3f} kN/m")
print("  Kuormareitti                   sisäpilarit suoraan perustuksille, ontelolaatat seinästä alapalkille, alapalkki ulompiin pilareihin")
print()
print(f"  {'Pilari':<22} {'Ryhmä':<10} {'N_sls':>9} {'N_uls':>9} {'N_min':>9}  {'Tila'}")
print(f"  {'-'*22} {'-'*10} {'-'*9} {'-'*9} {'-'*9}  {'-'*18}")
for column_id in column_output_order:
    N_sls = column_case_totals["SLS"][column_id]
    N_uls = column_case_totals["ULS"][column_id]
    N_min = column_case_totals["UPLIFT"][column_id]
    status = f"Puristus {N_min:.2f} kN  OK ✓" if N_min >= 0.0 else f"NOSTO {abs(N_min):.2f} kN"
    print(
        f"  {column_display[column_id]:<22} {column_group_label[column_id]:<10} "
        f"{N_sls:>9.2f} {N_uls:>9.2f} {N_min:>9.2f}  {status}"
    )

print("\n── ONTELOLAATAT 1200×150 SAUMATTU ─────────────────────")
print(f"  Piirustuksen lisäpysyvä g      {gk_hollow_slab_allow:.2f} kN/m²")
print(f"  Piirustuksen hyötykuorma q     {qk_hollow_slab_allow:.2f} kN/m²")
print(f"  Nykyinen pintavalu             {gk_floor_cast:.2f}/{gk_hollow_slab_allow:.2f} kN/m²  ({hollow_cast_util_pct:.0f}%)")
print(f"  Nykyinen hyötykuorma           {qk_terrace_live:.2f}/{qk_hollow_slab_allow:.2f} kN/m²  ({hollow_live_util_pct:.0f}%)")
print(f"  Oma paino vertailuna           {gk_hollow_slab:.2f} kN/m²  = {hollow_beam_self_kNm:.3f} kN/m")
print("  Huom.                          g/q-arvot koskevat tasaista pinta-kuormaa; alla olevat voimasuureet on laskettu yksiaukkoisena seinä → alapalkki -laattana.")
print()
print(f"  {'ID':<14} {'b_trib':>6} {'q_uni':>7} {'R_wall':>8} {'R_out':>8} {'Md':>7} {'Vd':>7}")
print(f"  {'-'*14} {'-'*6} {'-'*7} {'-'*8} {'-'*8} {'-'*7} {'-'*7}")
for row in hollow_slabs_sls:
    print(
        f"  {row['id']:<14} {row['cast_tributary_width_m']:>6.3f} {row['uniform_total_area_load_kNm2']:>7.2f} "
        f"{row['wall_reaction_kN']:>8.2f} {row['outer_reaction_kN']:>8.2f} "
        f"{max(abs(row['M_pos']['value_kNm']), abs(row['M_neg']['value_kNm'])):>7.2f} "
        f"{abs(row['V_abs']['value_kN']):>7.2f}"
    )
print(
    f"  Kriittisin laatta              {critical_hollow_slab['id']}  "
    f"Md = {max(abs(critical_hollow_slab['M_pos']['value_kNm']), abs(critical_hollow_slab['M_neg']['value_kNm'])):.2f} kNm, "
    f"Vd = {abs(critical_hollow_slab['V_abs']['value_kN']):.2f} kN"
)

print()
print(dw)
print("  YHTEENVETO – KÄYTTÖASTEET")
print(dw)
print()
print(f"  {'Palkki':<20} {'Md/Vd':>9} {'MRd/VRd':>9} {'Käyttöaste':>12}  {'Tulos'}")
print(f"  {'-'*20} {'-'*9} {'-'*9} {'-'*12}  {'-'*8}")
print(f"  {'KP450×51 (taivutus)':<20} {Md1:>7.2f}kNm {MRd1:>7.2f}kNm {eta1:>11.1f}%  {'OK ✓' if eta1 <= 100 else '*** YLITTYY ***'}")
print(f"  {'KP450×51 (§6.3.3 LTB)':<24} {'':>7}     {'':>9} {eta_ltb1_r:>11.1f}%  {'OK ✓' if eta_ltb1_r <= 100 else '*** YLITTYY ***'}  (ruodesivutuki)")
print(f"  {'KP450×51 (leikkaus)':<24} {Vd1:>8.2f}kN {VRd1:>8.2f}kN {eta_V1:>11.1f}%  {'OK ✓' if eta_V1 <= 100 else '*** YLITTYY ***'}")
print(f"  {'2×KP360×51 (taivutus)':<24} {Md2:>7.2f}kNm {MRd2:>7.2f}kNm {eta2:>11.1f}%  {'OK ✓' if eta2 <= 100 else '*** YLITTYY ***'}")
print(f"  {'2×KP360×51 (§6.3.3 LTB)':<24} {'':>7}     {'':>9} {eta_ltb2_r:>11.1f}%  {'OK ✓' if eta_ltb2_r <= 100 else '*** YLITTYY ***'}  (ruodesivutuki)")
print(f"  {'2×KP360×51 (leikkaus)':<24} {Vd2:>8.2f}kN {VRd2:>8.2f}kN {eta_V2:>11.1f}%  {'OK ✓' if eta_V2 <= 100 else '*** YLITTYY ***'}")
print(f"  {'LP225×90 (taivutus)':<20} {Md_lp:>7.2f}kNm {MRd_lp:>7.2f}kNm {eta_lp:>11.1f}%  {'OK ✓' if eta_lp <= 100 else '*** YLITTYY ***'}")
print(f"  {'LP225×90 (leikkaus)':<20} {Vd_lp:>8.2f}kN {VRd_lp:>8.2f}kN {eta_V_lp:>11.1f}%  {'OK ✓' if eta_V_lp <= 100 else '*** YLITTYY ***'}")
print()
print(f"  Suurin sallittu mitoitusviivakuorma (taivutus 100%):")
print(f"    KP450×51      qd,max = {qd_max1:.3f} kN/m  (q_eq,combined {qd1_eq:.3f} kN/m)")
print(f"    2×KP360×51    qd,max = {qd_max2:.3f} kN/m  (q_eq,combined {qd2_eq:.3f} kN/m)")
print()
print("  HUOMIOT:")
print("  * Lumikuorma sk = 2.0 kN/m² (Tuusula, FI NA vyöhyke II / YM asetus 6/16).")
print("  * Vuoden 2005 RakMk B1 -laskennassa käytetty 2.2 kN/m² vastasi silloista normia.")
print("  * LP225x90 saa päätypistekuorman KP450×51-palkin yhdistetystä tukireaktiosta.")
print("  * KP450×51 kantaa suoran kattokaistan + pää- ja vino-orsien sisäreaktiot; 2×KP360×51 kuormittuu katon osalta näiden orsien kautta.")
print("  * Rayst-orret on mallinnettu KP450-palkkien sivujatkeina, ei erillisinä KP360-siirtopisteinä.")
print("  * Pää- ja vino-orret on tarkistettu 50x400 mm bevel bottom notch -lovella geometrian notched_over-liitoksista.")
print("  * Räystäskuormat (x-suunta) huomioitu tukireaktioissa (LP225, pilarit).")
print("  * Kokonaispilarikuormat sisältävät nyt myös terassin pintavalun, 1200×150 ontelolaattojen omapainon,")
print("    betonisen alapalkin sekä maahan asti jatkuvien pilarien omapainon geometriasta johdettuna.")
print("  * Leikkaus ja taipuma on laskettu combined-kuormituksesta (suora viivakuorma + orsipistekuormat).")
print("  * Tuulimalli vaihdettu rakennuksen kattolappeen jatkeeksi: yläpinta käsitellään")
print("    rakennuksen kattopintana ja räystään alapinta viereisen seinäpinnan paineella.")
print("  * LTB-tarkistus (§6.3.3) edellyttää ruoteiden riittävää kiinnitystä palkkeihin.")
print("  * Kate-/ruodepaino 0.20 kN/m² on arvio – tarkista suunnitelmista.")
print(dw)

# ── Täydentävät kuormatarkistukset ──────────────────────────────────────────
print()
print(dw)
print("  TÄYDENTÄVÄT KUORMATARKISTUKSET")
print(dw)

print("\n── HUOLTOKUORMA – EN 1991-1-1 Kategoria H ─────────────")
print(f"  Pistekuorma  Qk = {Qk_huolto:.1f} kN  (missä tahansa kohdassa)")
print(f"  Hajakuorma   qk = {qk_H:.1f} kN/m²")
print(f"  Yhdistelmä: 1.35G + 1.5·Qk(piste) + 1.5·qk(haja) + 1.5·0.7·Sk")
print(f"  (Huolto ei yhdisty täyteen lumikuormaan: ψ0,lumi = {psi0_snow_accomp})")
print()
print(f"  {'Palkki':<18} {'Md,huolto':>10} {'MRd':>10} {'Käyttöaste':>12}  vs lumi")
print(f"  {'-'*18} {'-'*10} {'-'*10} {'-'*12}  {'-'*8}")
print(f"  {'KP450×51':<18} {M_huolto1:>8.2f}kNm {MRd1:>8.2f}kNm {eta_huolto1:>11.1f}%  (lumi: {eta1:.1f}%)")
print(f"  {'2×KP360×51':<18} {M_huolto2:>8.2f}kNm {MRd2:>8.2f}kNm {eta_huolto2:>11.1f}%  (lumi: {eta2:.1f}%)")
lumi_governs_1 = eta1 >= eta_huolto1
lumi_governs_2 = eta2 >= eta_huolto2
print(f"\n  KP450×51:   hallitseva kuorma = {'lumikuorma' if lumi_governs_1 else '*** HUOLTOKUORMA ***'}")
print(f"  2×KP360×51: hallitseva kuorma = {'lumikuorma' if lumi_governs_2 else '*** HUOLTOKUORMA ***'}")

print("\n── TUULEN NOSTOKUORMA – min. yhdistelmä ────────────────")
print(f"  0.9·Gk + 1.5·Wk,ylös  (suotuisa pysyvä, tuuli nostosuuntaan)")
print(f"  w_wind,ylös = {w_wind_up:.3f} kN/m²  (cp,net = {cp_net_up:.2f})")
print("  Reaktio sisältää myös palkin ulokkeen ja katon räystään.")
print()
print(f"  {'Palkki':<18} {'qmin [kN/m]':>12} {'Rmin/tuki':>14}  {'Tila'}")
print(f"  {'-'*18} {'-'*12} {'-'*14}  {'-'*20}")
def uplift_status(R):
    if R < 0:
        return f"NOSTO {abs(R):.2f} kN → KIINNITYS!"
    return f"Puristus {R:.2f} kN  OK ✓"
print(f"  {'KP450×51':<18} {qmin1_eq:>12.3f} {R_uplift1:>12.2f} kN  {uplift_status(R_uplift1)}")
print(f"  {'2×KP360×51':<18} {qmin2_eq:>12.3f} {R_uplift2:>12.2f} kN  {uplift_status(R_uplift2)}")
print(f"    vasen/oikea KP450×51         {R_uplift1_left:>12.2f} / {R_uplift1_right:>5.2f} kN")
print(f"    vasen/oikea 2×KP360×51       {R_uplift2_left:>12.2f} / {R_uplift2_right:>5.2f} kN")

print("\n── LUMIKUORMAN EPÄTASAINEN JAKAUTUMA ───────────────────")
print("  EN 1991-1-3 §6.2: yksilappinen katto")
print("  → Tasainen muotokerroin μ1 = 0.8 on ainoa lappeen kuormatapaus.")
print("  Huomio: Jos katos liittyy korkeampaan rakennukseen (U/L-muoto),")
print("  tulee tarkistaa lumivyörykuorma (EN 1991-1-3 §6.3/6.4).")

print()
print(dw)
print("  YHTEENVETO – KAIKKI KUORMATAPAUKSET")
print(dw)
print()
print(f"  Hallitseva mitoitustapaus per palkki:")
print(f"    KP450×51:   {'LUMI (%.1f%%)' % eta1 if lumi_governs_1 else 'HUOLTO (%.1f%%)' % eta_huolto1}  –  max käyttöaste {max(eta1,eta_huolto1):.1f}%")
print(f"    2×KP360×51: {'LUMI (%.1f%%)' % eta2 if lumi_governs_2 else 'HUOLTO (%.1f%%)' % eta_huolto2}  –  max käyttöaste {max(eta2,eta_huolto2):.1f}%")
if R_uplift1 < 0 or R_uplift2 < 0:
    print()
    print("  *** TÄRKEÄÄ: Tuulen nostokuorma aiheuttaa negatiiviset tukireaktiot!")
    print("  *** Palkkien kiinnitykset (ankkurointi) tulee mitoittaa nostolle.")
print(dw)
