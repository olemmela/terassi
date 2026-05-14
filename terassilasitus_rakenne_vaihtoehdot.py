"""
LASITETUN TERASSIN KUORMITUSLASKENTA – ETELÄSUOMI
===================================================
Standardi: EN 1990, EN 1991-1-1, EN 1991-1-3, EN 1991-1-4, EN 1995-1-1
Suomen kansalliset liitteet (FI NA)

Geometria:
  - Terassi: 7200mm leveä, 3600mm syvä pilareista ulospäin
  - Katto: aurinkopaneelit Longi Himo X10 LR7, 1990×1134×30mm, 25kg/kpl
  - 7 rinnan (1134mm väli) × 2 peräkkäin = 14 paneeelia
  - Räystäs leveyssunnassa: (7×1134 - 7200)/2 = 369mm
  - Räystäs syvyyssuunnassa: 2×1990 - 3600 = 380mm (kaltevuuskorjauksen jälkeen < 400mm)
  - Kaltevuus 7.25° talosta ulospäin (y-suunta)  [atan((2800-2343)/3600)]

Kantavat rakenteet:
  - Kattotuolit (rafters): y-suunta, 1134mm välein, jänneväli 3430mm (c/c sisäpalkki → ulkopilari)
    Sisätuki: geometriasta luettu sisäpalkki/pilarilinja; olemassa oleva
    2×KP360×51 on erillisenä siirtopalkkina lähellä pilarilinjaa
    Ulkotuki: uusi ulkoreunanen palkki (y=5275mm)
  - Ulkoreunanen palkki: x-suunta, 3 pilaria → kaksi 3600mm jänneväliä
  - Materiaalit: verrataan liimapuuta (GL30c) ja terästä (HEA/IPE)
"""

import math

from beam_analysis import solve_linear_system
from geometry_loader import load, member, surface, reference, profile_b, profile_h


def clamp_mm(x_mm, x_min_mm, x_max_mm):
    return max(x_min_mm, min(x_mm, x_max_mm))


def pattern_x_positions_mm(m):
    """Palauttaa pattern-jäsenen x-sijainnit axis_start-pisteestä."""
    x0 = float(m["axis_start"]["x"])
    pattern = m.get("pattern")
    if not pattern:
        return [x0]
    count = int(pattern["count"])
    dx = float(pattern["offset"].get("x", 0.0))
    return [x0 + i * dx for i in range(count)]


def tributary_widths_m(x_positions_mm, edge_start_mm, edge_end_mm):
    """Laskee kattotuolien tributäärileveydet x-suunnassa."""
    widths = []
    for i, x_mm in enumerate(x_positions_mm):
        left_mm = edge_start_mm if i == 0 else 0.5 * (x_positions_mm[i - 1] + x_mm)
        right_mm = edge_end_mm if i == len(x_positions_mm) - 1 else 0.5 * (x_mm + x_positions_mm[i + 1])
        widths.append((right_mm - left_mm) / 1000.0)
    return widths


def beam_support_reactions(beam_start_mm, beam_end_mm, support_xs_mm, point_loads):
    """Yksinkertaisesti tuettu palkki pistekuormilla; reaktiot ylöspäin [kN]."""
    nodes = sorted(set([beam_start_mm, beam_end_mm] + list(support_xs_mm) + [x_mm for x_mm, _ in point_loads]))
    n_nodes = len(nodes)
    n_dof = 2 * n_nodes
    K = [[0.0] * n_dof for _ in range(n_dof)]
    F = [0.0] * n_dof

    for x_mm, p_kN in point_loads:
        node_i = nodes.index(x_mm)
        F[2 * node_i] -= p_kN * 1000.0  # N, alaspäin negatiivinen

    for elem_i in range(n_nodes - 1):
        L_mm = nodes[elem_i + 1] - nodes[elem_i]
        k = [
            [12.0 / L_mm**3,   6.0 / L_mm**2,  -12.0 / L_mm**3,   6.0 / L_mm**2],
            [ 6.0 / L_mm**2,   4.0 / L_mm,      -6.0 / L_mm**2,   2.0 / L_mm],
            [-12.0 / L_mm**3, -6.0 / L_mm**2,   12.0 / L_mm**3,  -6.0 / L_mm**2],
            [ 6.0 / L_mm**2,   2.0 / L_mm,      -6.0 / L_mm**2,   4.0 / L_mm],
        ]
        idx = [2 * elem_i, 2 * elem_i + 1, 2 * (elem_i + 1), 2 * (elem_i + 1) + 1]
        for a_i, I in enumerate(idx):
            for b_i, J in enumerate(idx):
                K[I][J] += k[a_i][b_i]

    fixed = [2 * nodes.index(x_mm) for x_mm in support_xs_mm]
    free = [i for i in range(n_dof) if i not in fixed]
    Kff = [[K[i][j] for j in free] for i in free]
    Ff = [F[i] for i in free]
    uf = solve_linear_system(Kff, Ff)

    u = [0.0] * n_dof
    for dof_i, val in zip(free, uf):
        u[dof_i] = val

    Ku = [sum(K[i][j] * u[j] for j in range(n_dof)) for i in range(n_dof)]
    R = [Ku[i] - F[i] for i in range(n_dof)]
    return {x_mm: R[2 * nodes.index(x_mm)] / 1000.0 for x_mm in support_xs_mm}


def beam_point_moments(beam_start_mm, beam_end_mm, reactions_kN, point_loads):
    """Palauttaa pistekuormitetun palkin momentit solmukohdissa [kNm], notko +."""
    force_at = {}
    for x_mm, p_kN in point_loads:
        force_at[x_mm] = force_at.get(x_mm, 0.0) - p_kN
    for x_mm, r_kN in reactions_kN.items():
        force_at[x_mm] = force_at.get(x_mm, 0.0) + r_kN

    positions = sorted(set([beam_start_mm, beam_end_mm] + list(force_at.keys())))
    moments = {positions[0]: 0.0}
    shear_kN = force_at.get(positions[0], 0.0)
    for x_prev, x_cur in zip(positions, positions[1:]):
        moments[x_cur] = moments[x_prev] + shear_kN * ((x_cur - x_prev) / 1000.0)
        shear_kN += force_at.get(x_cur, 0.0)
    return moments


def build_rafter_support_loads(x_positions_mm, trib_widths_m, area_loads_kNm2, reaction_factor_m):
    """Muuntaa kattokuorman kattotuolilta tukipistekuormiksi palkille."""
    if isinstance(area_loads_kNm2, (int, float)):
        area_loads = [float(area_loads_kNm2)] * len(x_positions_mm)
    else:
        area_loads = list(area_loads_kNm2)
    if len(area_loads) != len(x_positions_mm):
        raise ValueError("area_loads_kNm2 length mismatch")
    return [
        (x_mm, area_kNm2 * trib_w_m * reaction_factor_m)
        for x_mm, trib_w_m, area_kNm2 in zip(x_positions_mm, trib_widths_m, area_loads)
    ]

# ============================================================
# ============================================================
# GEOMETRIA  (luetaan geometry/terassi.json:ista)
# ============================================================
_GEO = load("terassi.json")

_wall_ref    = reference(_GEO, "ref.house_wall")
_wall_xs     = [p["x"] for p in _wall_ref["polygon"]]
terrace_width = int(max(_wall_xs) - min(_wall_xs))   # 7200 mm (x-suunta)
drift_obstacle_x0_mm = min(_wall_xs)
drift_obstacle_x1_mm = max(_wall_xs)

_inner_col   = member(_GEO, "columns", "col.existing.inner.x125")
_outer_col_0 = member(_GEO, "columns", "col.outer.x0")
inner_y        = int(_inner_col["base"]["y"])                  # 1675 mm
terrace_depth  = int(_outer_col_0["base"]["y"] - inner_y)      # 3600 mm

# ── KORKEUDET (terassin lattiasta, mm) ───────────────────────
# Terassin kate (y-suunta, talosta ulospäin):
_panels = surface(_GEO, "surf.solar_panels")
# Paneelipolygon kattaa räystäät, joten sen y-ääripäät eivät osu rakennus-
# palkkien linjoille. Lasketaan paneelitason (y,z) parametrit ja interpoloidaan.
_panel_xmin = min(p["x"] for p in _panels["polygon"])
_panel_xmax = max(p["x"] for p in _panels["polygon"])
_panel_ymin = min(p["y"] for p in _panels["polygon"])
_panel_ymax = max(p["y"] for p in _panels["polygon"])
_panel_zmin_y = max(p["z"] for p in _panels["polygon"] if p["y"] == _panel_ymin)
_panel_zmax_y = max(p["z"] for p in _panels["polygon"] if p["y"] == _panel_ymax)
def _panel_z(y):
    t = (y - _panel_ymin) / (_panel_ymax - _panel_ymin)
    return _panel_zmin_y + t * (_panel_zmax_y - _panel_zmin_y)
h_katto_inner   = int(round(_panel_z(inner_y)))                 # 2800 mm – katteen yläpinta seinän kohdalla (sisäpää)
h_katto_outer   = int(round(_panel_z(inner_y + terrace_depth))) # 2343 mm – ulkopää: pilari+palkki+kattotuoli+paneeli

# Rakennuksen räystään korkeus (x-suunta) – luetaan ref.house.roof y=0-reunasta.
# ref.house.roof edustaa talon oman katon alapintaa/räystästä, jolta lumi liukuu
# terassin päälle. Tämä on korkeampi kuin pelkkä seinän yläpinta (ref.house_wall).
_house_roof_ref  = reference(_GEO, "ref.house.roof")
_roof_eave_pts   = sorted([p for p in _house_roof_ref["polygon"] if p["y"] == 0],
                          key=lambda p: p["x"])   # [(x_min, z_min), (x_max, z_max)]
_roof_eave_x0    = _roof_eave_pts[0]["x"]
_roof_eave_x1    = _roof_eave_pts[-1]["x"]
_roof_eave_z0    = _roof_eave_pts[0]["z"]
_roof_eave_z1    = _roof_eave_pts[-1]["z"]

def h_rakennus_at_x(x_mm):
    """Talon räystään korkeus (mm) annetussa x-koordinaatissa (lineaarinen interpolointi)."""
    x_eff_mm = clamp_mm(x_mm, drift_obstacle_x0_mm, drift_obstacle_x1_mm)
    t = (x_eff_mm - _roof_eave_x0) / (_roof_eave_x1 - _roof_eave_x0)
    return _roof_eave_z0 + t * (_roof_eave_z1 - _roof_eave_z0)

_wall_z_x0 = h_rakennus_at_x(0)
_wall_z_xEnd = h_rakennus_at_x(terrace_width)
# Korkea/matala pää luetaan geometriasta — ei oleteta kumpi pää on korkeampi
h_rakennus_korkea = max(_wall_z_x0, _wall_z_xEnd)   # talon räystään ylin kohta (mm)
h_rakennus_matala = min(_wall_z_x0, _wall_z_xEnd)   # talon räystään matala pää (mm)

# ── LASKETUT KALTEVUUDET ─────────────────────────────────────
outer_y         = inner_y + terrace_depth   # = 5275mm seinästä
slope_deg       = math.degrees(math.atan((h_katto_inner - h_katto_outer) / terrace_depth))
slope_rad       = math.radians(slope_deg)
slope_rakennus_deg = math.degrees(math.atan((h_rakennus_korkea - h_rakennus_matala) / terrace_width))

# ── KINOSTUMAN KORKEUDET (talon katto - terassin kate) ───────
h_seinä_korkea = (h_rakennus_korkea - h_katto_inner) / 1000.0  # m
h_seinä_matala = (h_rakennus_matala - h_katto_inner) / 1000.0  # m
# (terassin kate on sama korkeus koko x-suunnassa seinän kohdalla)
# HUOM: Drift kerääntyy paneelitasolle talon seinää vasten. Lumi liukuu
# paneelitasoa pitkin seinää kohti (paneelin korkea pää on seinällä).
# Talon oma katto viettää x-suuntaan, joten siltä ei valu lunta terassin
# kate seinälinjaa vasten – drift-laskennassa "b_upper" on siksi
# paneelin itsensä pituus, ei talon katon.

# Paneelit: Longi Himo X10 LR7, 1990×1134mm
_panel_cnt      = _panels["count"]
panel_w_mm      = float(_panel_cnt["unit_size_mm"]["x"])  # 1134.0 mm – paneelin leveys (= kattotuolijako)
panel_d_mm      = float(_panel_cnt["unit_size_mm"]["y"])  # 1990.0 mm – paneelin syvyys
panel_mass_kg   = float(_panel_cnt["unit_mass_kg"])       # 25.0 kg/kpl
n_panels_w      = int(_panel_cnt["nx"])                   # 7 – rinnan (leveyssuunta)
n_panels_d      = int(_panel_cnt["ny"])                   # 2 – peräkkäin (syvyyssuunta)
n_panels_total  = n_panels_w * n_panels_d                 # = 14

# Katteen todellinen vaakaprojektio luetaan suoraan geometriasta.
roof_loaded_width_m = (_panel_xmax - _panel_xmin) / 1000.0
b_terassi = (_panel_ymax - _panel_ymin) / 1000.0  # m – paneelikentän vaakaprojektio seinästä ulospäin

_rafter_member = member(_GEO, "rafters", "rafter.0")
rafter_x_raw_mm = pattern_x_positions_mm(_rafter_member)
n_rafters = len(rafter_x_raw_mm)
rafter_trib_widths_m = tributary_widths_m(rafter_x_raw_mm, _panel_xmin, _panel_xmax)

# Kattotuolien (rafter) jänneväli
pilari_leveys   = int(_outer_col_0["profile"]["b_mm"])  # 250 mm – ulkopilarin leveys
outer_support_xs_mm = sorted(
    float(member(_GEO, "columns", cid)["base"]["x"])
    for cid in ("col.outer.x0", "col.outer.x3600", "col.outer.x7200")
)
inner_support_xs_mm = sorted(
    float(member(_GEO, "columns", cid)["base"]["x"])
    for cid in ("col.existing.inner.x125", "col.existing.inner.x7075")
)
n_outer_pillars = len(outer_support_xs_mm)

# Sisäpalkin leveys (y-suunnassa): kiinnitetty vanhan pilarin ulkoreunaan
# Kun profiili on TBD (päättämättä), käytetään laskennassa oletusarvoa 90 mm.
_inner_beam = member(_GEO, "beams", "beam.inner.new")
_outer_beam = member(_GEO, "beams", "beam.outer")
_inner_beam_profile = _inner_beam["profile"]
b_inner_beam_mm = int(_inner_beam_profile.get("b_mm", 90))  # oletus 90 mm jos TBD

# Kattotuolin tarkka jänneväli: sisäpalkin keskilinja → ulkopilarin keskilinja
# Sisäpalkin keskilinja: pilarilinja + pilarin puolikas + b_inner/2
rafter_inner_y = inner_y + pilari_leveys / 2.0 + b_inner_beam_mm / 2.0
rafter_outer_y = inner_y + terrace_depth        # = 5275mm ulkopilarin keskikohta
rafter_span_mm  = rafter_outer_y - rafter_inner_y
rafter_span_m   = rafter_span_mm / 1000.0

# Ulkoreunanen palkki: vapaa jänne pilarin reunojen välillä
outer_span_cc_mm  = terrace_width / (n_outer_pillars - 1)   # 3600mm c/c
outer_span_mm     = outer_span_cc_mm - pilari_leveys         # 3350mm vapaa jänne
outer_span_m      = outer_span_mm / 1000.0
rafter_spacing  = panel_w_mm / 1000.0   # m = 1.134m

inner_beam_start_x_mm = float(_inner_beam["axis_start"]["x"])
inner_beam_end_x_mm = float(_inner_beam["axis_end"]["x"])
outer_beam_start_x_mm = float(_outer_beam["axis_start"]["x"])
outer_beam_end_x_mm = float(_outer_beam["axis_end"]["x"])
rafter_beam_load_xs_mm = [
    clamp_mm(x_mm, inner_beam_start_x_mm, inner_beam_end_x_mm)
    for x_mm in rafter_x_raw_mm
]

# Kattotuoli kantaa koko paneelikentän syvyyden, mukaan lukien sisä- ja ulkoräystäät.
rafter_load_centroid_from_inner_support_m = ((_panel_ymin + _panel_ymax) / 2.0 - rafter_inner_y) / 1000.0
rafter_outer_reaction_factor = b_terassi * rafter_load_centroid_from_inner_support_m / rafter_span_m
rafter_inner_reaction_factor = b_terassi - rafter_outer_reaction_factor

# ============================================================
# PYSYVÄT KUORMAT
# ============================================================
g_panels = (n_panels_total * panel_mass_kg * 9.81 / 1000.0)  # kN, yhteensä

# Paneelien pinta-ala (vaakaprojisoitu, sisältää räystäät)
panel_area_horiz = roof_loaded_width_m * b_terassi   # m²
gk_panels = g_panels / panel_area_horiz    # kN/m² (vaakaprojisoitu)

# Kiinnitystarvikkeet ja oheisrakenteet (ei kantavaa profiilia)
gk_fixings = 0.05    # kN/m² (orret, listat, kiinnikkeet)
gk_frame   = 0.15    # kN/m² (sisältää kattotuolin arvioidun omapainon ~0.10 + kiinnikkeet 0.05)
gk_total   = gk_panels + gk_frame   # kN/m² (yleislaskelmat)

# Pohjakuorma per-profiilivertailulle (ilman kantavan profiilin omapainoa)
gk_no_rafter = gk_panels + gk_fixings   # kN/m²

# ============================================================
# LUMIKUORMA (EN 1991-1-3, FI NA)
# ============================================================
sk   = 2.0     # kN/m² Tuusula, Zone II, EN 1991-1-3 FI NA (YM 6/16)
               # Vanha RakMk B1 (2005) käytti 2.2 kN/m² – Eurokoodin FI NA päivitti arvoon 2.0.
# μ1 kaltevuudelle 7° (0°–30°): μ1 = 0.8
mu1  = 0.8
s_roof = mu1 * sk   # kN/m² vaakatasolle projisoituna

# ============================================================
# TUULIKUORMA (EN 1991-1-4, FI NA)
# ============================================================
vb0     = 21.0
rho_air = 1.25
z0      = 0.05
z_ref   = math.ceil(max(
    p["z"] for s in _GEO.get("surfaces", []) for p in s.get("polygon", []) if "z" in p
) / 500.0) * 0.5   # m – korkein kohta geometriasta, pyöristetty ylöspäin 0.5m (EN 1991-1-4 §4.3.2)
kr      = 0.19
cr_z    = kr * math.log(max(z_ref, 2.0) / z0)
Iv_z    = 1.0 / math.log(max(z_ref, 2.0) / z0)
vm_z    = cr_z * vb0
qp_z    = (1.0 + 7.0 * Iv_z) * 0.5 * rho_air * vm_z**2 / 1000.0  # kN/m²

# Yksikalteinen katto 7°, EN 1991-1-4 taulukko 7.7 (vapaa katos, nettokertoimet)
# Perustelu: aurinkopaneelikatto toimii avoimena pinnanaa → virtaus molempien pintojen yli.
# α=5°–10°: cp_net_down = +0.8, cp_net_up = -0.6 (säilyy samana 0°→10° välillä)
# HUOMIO: Kiinnitysten nostokuormatarkistuksessa harkittava avoimemman rakenteen arvoja.
cp_net_down = 0.8
cp_net_up   = -0.6
w_down = cp_net_down * qp_z
w_up   = cp_net_up   * qp_z

# ── TUULIKUORMA: LASIT KIINNI – taulukko 7.4 + sisäpaine ────────────────────
# Suljettu rakennustapaus: ulkopaine (cp,e) taulukosta 7.4 (yksikalteinen katto)
# miinus sisäpaine (cp,i). Tarkistetaan imusuunta joka on hallitseva kiinni-tilassa.
#
# EN 1991-1-4 Taulukko 7.4, yksikalteinen katto, α=7° (interpoloitu 5°→15°, t=0.2):
#   θ=0°   (tuuli matalan räystään puolelta):  H-vyöhyke: cp,e = -0.54 (+0.04)
#   θ=180° (tuuli korkean räystään / rakennuksen puolelta): H-vyöhyke: cp,e = -0.92
# Hallitseva imu on θ=180° (tuuli rakennuksen yli) → cp,e,H = -0.92
cpe_H_t0   = -0.54  # θ=0°  (tuuli ulkoa→sisään, matala räystäs edessä)
cpe_H_t180 = -0.92  # θ=180° (tuuli rakennuksen yli, korkea räystäs edessä)
cpe_H_up   = min(cpe_H_t0, cpe_H_t180)   # hallitseva imu (negatiivisempi)
cpe_H_down = max(cpe_H_t0, +0.04)        # paras mahdollinen alaspäin (lähes nolla)

# Sisäpaine cp,i (EN 1991-1-4 §7.2.9):
#   Lasit kiinni, ei dominoivaa aukkoa → cp,i ∈ [-0.3, +0.2] (RIL 201-1)
#   Imusuunnan maksimointiin: cp,i = +0.2 (paine sisällä lisää imua ulkona)
#   Alaspäin suunnan maksimointiin: cp,i = -0.3 (imu sisällä lisää painetta ulkona)
cpi_unfav_up   = +0.2   # pahin cp,i imusuuntaan (paine sisällä)
cpi_unfav_down = -0.3   # pahin cp,i alaspäin (imu sisällä)

# Nettopaineet, suljettu tila:
cp_net_up_closed   = cpe_H_up   - cpi_unfav_up    # = -0.92 - 0.20 = -1.12  (imu)
cp_net_down_closed = cpe_H_down - cpi_unfav_down  # = +0.04 - (-0.30) = +0.34 (paine)
w_up_closed   = cp_net_up_closed   * qp_z   # kN/m²
w_down_closed = cp_net_down_closed * qp_z   # kN/m²

# HUOM: qd_uplift_closed ja R_uplift_closed lasketaan myöhemmin gk_line:n jälkeen

# ── VAAKAKUORMA SIVULASITUKSELTA ULKOPALKKIIN ────────────────
# Terassilasit kiinnittyvät lattiaan ja ulkoreunaiseen yläpalkkiin.
# Tuuli siirtää pystyiselle lasille puolikkaan vaakakuormasta yläpalkkiin.
cp_wall_net   = 1.0                        # nettokertoin (paine+imu), EN 1991-1-4
h_lasitus_m   = 2.0                          # m – sivulasituksen korkeus (lattia → ulkopalkki alapuoli)
h_trib_lasit  = h_lasitus_m / 2.0          # m – tributaarinen korkeus yläpalkille
# Karakteristinen vaakaviivakuorma ulkopalkkiin [kN/m]
q_outer_wind_h_char = qp_z * cp_wall_net * h_trib_lasit

# ============================================================
# KUORMAYHDISTELMÄT KATTOTUOLILLE
# ============================================================
gammaG  = 1.35
gammaQ  = 1.50
psi0_W  = 0.6
psi0_snow = 0.7

# UDL kattotuolille [kN/m] (kuorma × rafter-väli)
b = rafter_spacing   # m

gk_line   = gk_total * b
qk_snow   = s_roof   * b
qk_w_down = w_down   * b
qk_w_up   = w_up     * b

# ULS-nostokuorma kattotuolille (lasit kiinni, tuuli johtava) – lasketaan nyt kun gk_line tunnetaan:
# EN 1990 eq. 6.10: γG,fav=0.9 × gk (edullinen pysyvä alas) − 1.5 × |w_up_closed|
qk_w_up_closed   = abs(w_up_closed) * rafter_spacing    # kN/m (ominais, per kattotuoli)
qd_uplift_closed = 1.5 * qk_w_up_closed - 0.9 * gk_line  # kN/m net (+ = ylöspäin = nosto)
R_uplift_closed  = qd_uplift_closed * rafter_span_m / 2.0  # kN per tuki (+ = veto tukikiinnityksessä)

qd_down = gammaG * gk_line + gammaQ * qk_snow + gammaQ * psi0_W * qk_w_down
roof_load_area_uls = qd_down / rafter_spacing
roof_load_area_char = (gk_line + qk_snow) / rafter_spacing
roof_load_area_outer_sls = (gk_line + qk_snow + psi0_W * qk_w_down) / rafter_spacing
roof_load_area_outer_B = (gammaG * gk_line + gammaQ * psi0_snow * qk_snow) / rafter_spacing

# Taivutusmomentti (yksinkertaisesti tuettu, kuorma vaakaprojisoitu → ei 1/cos-kerrointa)
# Kattotuoli on kallistettu α=7.25° y-suunnassa (sama kuin jännevälisuunta).
# Kuormat [kN/m²] ovat vaakaprojisoituja → M = q × L_h² / 8  (ei 1/cos-korjausta).
# Laskennallinen huomio: kallistetussa poikkileikkauksessa on pieni kaksitaivutuskomponentti
# M_z ≈ M_y × tan(7.25°) ≈ 12.7% × M_y. Käytännön vaikutus on pieni (M_Rd,z >> M_z
# leveälaippaisilla profiileilla), mutta tarkistettava EN 1995-1-1 §6.1.6 tai EN 1993-1-1 §6.2.9.
moment_factor_rafter = 1.0  # vaakaprojisoitu kuorma, ei 1/cos-korjausta
Md_rafter = qd_down * rafter_span_m**2 / 8.0 * moment_factor_rafter

# Huoltokuorma (EN 1991-1-1, Kat H)
# LASIKATTO: ei kuljeta päällä → huoltokuorma jätetään pois rakenneosien mitoituksesta.
# Huolto tapahtuu tikkaalta/erilliseltä tasolta. Lasitus mitoitetaan erikseen.
lasikatto = True   # True = ei huoltokuormaa kantaville rakenteille

Qk_huolto = 1.0   # kN (tiedoksi)
qd_huolto_line = gammaG * gk_line + gammaQ * (qk_snow * psi0_snow)
M_huolto_Q = gammaQ * Qk_huolto * rafter_span_m / 4.0
Md_rafter_huolto = qd_huolto_line * rafter_span_m**2 / 8.0 * moment_factor_rafter + M_huolto_Q

# Hallitseva tapaus: lasikatolle lumi+tuuli (ei huoltokuormaa)
if lasikatto:
    Md_rafter_gov = Md_rafter
    gov_case = "lumi+tuuli (lasikatto, ei huoltokuormaa)"
else:
    Md_rafter_gov = max(Md_rafter, Md_rafter_huolto)
    gov_case = "lumi+tuuli" if Md_rafter >= Md_rafter_huolto else "huolto"

# Tukireaktiot kattotuolilta sisä- ja ulkopalkille (tyypillinen 1134 mm tributääri)
R_inner_rafter = qd_down * rafter_inner_reaction_factor   # kN (ULS, per kattotuoli)
R_outer_rafter = qd_down * rafter_outer_reaction_factor   # kN (ULS, per kattotuoli)
R_inner_rafter_char = (gk_line + qk_snow) * rafter_inner_reaction_factor  # kN (ominais)

# ============================================================
# MATERIAALIVERTAILU – KATTOTUOLI (3600mm jänneväli, b=1134mm)
# ============================================================

def check_section(name, b_sec, h_sec, fm_k, fv_k, E_mean, gamma_M, kmod,
                   Md, L_mm, gk_self_kNm, q_sls_ext=None):
    """Tarkistaa taivutuksen, leikkauksen ja taipuman."""
    fm_d = kmod * fm_k / gamma_M
    fv_d = kmod * fv_k / gamma_M
    W    = b_sec * h_sec**2 / 6.0       # mm³
    I    = b_sec * h_sec**3 / 12.0      # mm⁴
    A    = b_sec * h_sec
    MRd  = fm_d * W / 1.0e6             # kNm
    VRd  = fv_d * A / 1.5e3             # kN
    Vd   = (Md * 2.0 / (L_mm/1000.0))   # approx  Vd = qd*L/2, ja Md=qd*L²/8 → qd=8Md/L²
    # Tarkempi Vd:
    qd_eq = Md * 8.0 / (L_mm/1000.0)**2
    Vd   = qd_eq * (L_mm/1000.0) / 2.0
    eta_M = Md / MRd * 100.0
    eta_V = Vd / VRd * 100.0
    # Taipuma (SLS, ominaisyhdistelmä)
    q_sls = q_sls_ext if q_sls_ext is not None else (gk_line + qk_snow)  # kN/m
    EI = E_mean * I
    delta = 5.0 * q_sls * L_mm**4 / (384.0 * EI)
    delta_lim = L_mm / 300.0
    # Omapaino tarkistus
    g_self = b_sec/1000.0 * h_sec/1000.0 * gk_self_kNm
    return {
        'name': name, 'b': b_sec, 'h': h_sec,
        'fm_d': fm_d, 'W_cm3': W/1e3, 'MRd': MRd,
        'fv_d': fv_d, 'A': A, 'VRd': VRd, 'Vd': Vd,
        'eta_M': eta_M, 'eta_V': eta_V,
        'I_cm4': I/1e4,
        'EI_kNm2': EI/1e9, 'delta': delta, 'delta_lim': delta_lim,
        'g_self': g_self,
        'ok_M': eta_M <= 100, 'ok_V': eta_V <= 100,
        'ok_delta': delta <= delta_lim
    }

# Liimapuu GL30c vaihtoehdot
# HUOM: Ulkorakenne = palveluluokka 3 (SC3) → kmod = 0.65 (EN 1995 Taulukko 3.1)
# Jos rakenne on sateelta suojattu lasituksella → voidaan harkita SC2 (kmod=0.8)
lp_kmod = 0.8    # SC2: kattolasituksen ja sivulasituksen suojaama rakenne
lp_options = [
    # name,      b,    h,  fm_k, fv_k, E_mean, gammaM, kmod
    ("LP180×90",  90, 180, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP225×90",  90, 225, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP315x90",  90, 315, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP225x115",115, 225, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP270x115",115, 270, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP315x115",115, 315, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP360x115",115, 360, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP405x115",115, 405, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP225x140",140, 225, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP270x140",140, 270, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP315x140",140, 315, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP360x140",140, 360, 30.0, 3.5, 13000, 1.25, lp_kmod),
    ("LP405x140",140, 405, 30.0, 3.5, 13000, 1.25, lp_kmod),
]
lp_density = 5.0   # kN/m³ (GL30c)

def rafter_profile_md(g_self_line_kNm):
    """Palauttaa (Md_gov, q_sls) kattotuolille profiilikohtaisella omapainolla.
    g_self_line_kNm: profiilin omapaino kN/m (lineaarinen)."""
    gk_l  = gk_no_rafter * rafter_spacing + g_self_line_kNm   # kN/m
    qk_s  = s_roof * rafter_spacing
    qk_wd = w_down * rafter_spacing
    qd_d  = gammaG * gk_l + gammaQ * qk_s + gammaQ * psi0_W * qk_wd
    Md    = qd_d * rafter_span_m**2 / 8.0 * moment_factor_rafter
    q_sls = gk_l + qk_s
    return Md, q_sls

# Teräsvaihtoehdot (EN 10025-2, S235/S355)
# IPE-profiileja – arvot standarditaulukoista
# format: name, b, h, fy, fv, E, gammaM0, kmod (=1 teräkselle)
# Käytetään yksinkertaistettua taivutustarkistusta
# IPE: W_el (cm³), I (cm⁴), m (kg/m), A (cm²), fv=fy/sqrt(3)
steel_options = [
    # name,   h, b_flange, W_el_cm3, I_cm4, m_kgm, fy_N/mm2
    ("IPE100",  100,  55,  34.2,  171.0,   8.1, 235),
    ("IPE120",  120,  64,  53.0,  318.0,  10.4, 235),
    ("IPE140",  140,  73,  77.3,  541.0,  12.9, 235),
    ("IPE160",  160,  82, 123.0,  869.3,  15.8, 235),
    ("HEA120",  120, 120, 101.0,  606.2,  19.9, 235),   # W_el,y ja I_y korjattu (ArcelorMittal EN 10365)
    ("HEA140",  140, 140, 155.4, 1033.0,  24.7, 235),   # W_el,y ja I_y korjattu
    ("HEA160",  160, 160, 220.1, 1673.0,  30.4, 235),   # W_el,y korjattu (I_y oli oikein)
]
gammaM0_steel = 1.0
E_steel = 210000.0   # N/mm², teräs
E_lp    = 13000.0    # N/mm², liimapuu GL30c

results_lp    = []
results_steel = []

for (nm, b, h, fm_k, fv_k, E, gM, km) in lp_options:
    g_self_lp = b / 1000.0 * h / 1000.0 * lp_density   # kN/m
    Md_lp_i, q_sls_lp_i = rafter_profile_md(g_self_lp)
    r = check_section(nm, b, h, fm_k, fv_k, E, gM, km,
                      Md_lp_i, rafter_span_mm, lp_density, q_sls_lp_i)
    results_lp.append(r)

for (nm, h_mm, b_f, W_el, I_cm4, m_kgm, fy) in steel_options:
    # Teräs: fm_d = fy/gammaM0, fv_d = fy/(sqrt(3)*gammaM0)
    fm_d_s  = fy / gammaM0_steel
    fv_d_s  = fy / (math.sqrt(3) * gammaM0_steel)
    W_mm3   = W_el * 1e3       # mm³
    I_mm4   = I_cm4 * 1e4     # mm⁴
    E_s     = 210000.0         # N/mm²
    MRd_s   = fm_d_s * W_mm3 / 1.0e6   # kNm
    VRd_s   = fv_d_s * (h_mm * 7.0)    / 1.5e3  # approx web shear (7mm web)
    g_self_s = m_kgm * 9.81 / 1000.0   # kN/m, profiilin omapaino
    Md_s_i, q_sls_s_i = rafter_profile_md(g_self_s)
    qd_eq   = Md_s_i * 8.0 / rafter_span_m**2
    Vd_s    = qd_eq * rafter_span_m / 2.0
    eta_M_s = Md_s_i / MRd_s * 100.0
    eta_V_s = Vd_s / VRd_s * 100.0
    EI_s    = E_s * I_mm4
    delta_s = 5.0 * q_sls_s_i * rafter_span_mm**4 / (384.0 * EI_s)
    delta_lim_s = rafter_span_mm / 300.0
    results_steel.append({
        'name': nm, 'h': h_mm, 'W_cm3': W_el, 'MRd': MRd_s,
        'VRd': VRd_s, 'Vd': Vd_s,
        'eta_M': eta_M_s, 'eta_V': eta_V_s,
        'EI_kNm2': EI_s/1e9, 'delta': delta_s, 'delta_lim': delta_lim_s,
        'g_self': g_self_s,
        'ok_M': eta_M_s <= 100, 'ok_V': eta_V_s <= 100,
        'ok_delta': delta_s <= delta_lim_s
    })

# ============================================================
# ULKOREUNANEN PALKKI (x-suunta, 2 × 3600mm jänneväli)
# ============================================================
# Ulkoreunapalkki mallinnetaan jatkuvana 3-tukisena palkkina.
# Kattotuolikuormat siirretään siihen pistekuormina todellisilla tributäärileveyksillä.
outer_rafter_point_loads_uls = build_rafter_support_loads(
    rafter_beam_load_xs_mm, rafter_trib_widths_m, roof_load_area_uls, rafter_outer_reaction_factor)
outer_rafter_point_loads_sls = build_rafter_support_loads(
    rafter_beam_load_xs_mm, rafter_trib_widths_m, roof_load_area_outer_sls, rafter_outer_reaction_factor)
outer_rafter_point_loads_B = build_rafter_support_loads(
    rafter_beam_load_xs_mm, rafter_trib_widths_m, roof_load_area_outer_B, rafter_outer_reaction_factor)

outer_beam_reactions_uls = beam_support_reactions(
    outer_beam_start_x_mm, outer_beam_end_x_mm, outer_support_xs_mm, outer_rafter_point_loads_uls)
outer_beam_moments_uls = beam_point_moments(
    outer_beam_start_x_mm, outer_beam_end_x_mm, outer_beam_reactions_uls, outer_rafter_point_loads_uls)
Md_outer_beam_pos = max(outer_beam_moments_uls.values())
Md_outer_beam_neg = min(outer_beam_moments_uls.values())
Md_outer_beam = max(Md_outer_beam_pos, -Md_outer_beam_neg)

outer_beam_reactions_B = beam_support_reactions(
    outer_beam_start_x_mm, outer_beam_end_x_mm, outer_support_xs_mm, outer_rafter_point_loads_B)
outer_beam_moments_B = beam_point_moments(
    outer_beam_start_x_mm, outer_beam_end_x_mm, outer_beam_reactions_B, outer_rafter_point_loads_B)
Md_outer_v_B = max(max(outer_beam_moments_B.values()), -min(outer_beam_moments_B.values()))

q_outer_beam = sum(load_kN for _, load_kN in outer_rafter_point_loads_uls) / roof_loaded_width_m
q_outer_sls = sum(load_kN for _, load_kN in outer_rafter_point_loads_sls) / roof_loaded_width_m

inner_rafter_point_loads_normal = build_rafter_support_loads(
    rafter_beam_load_xs_mm, rafter_trib_widths_m, roof_load_area_uls, rafter_inner_reaction_factor)
inner_rafter_point_loads_char = build_rafter_support_loads(
    rafter_beam_load_xs_mm, rafter_trib_widths_m, roof_load_area_char, rafter_inner_reaction_factor)
q_inner_add = sum(load_kN for _, load_kN in inner_rafter_point_loads_normal) / roof_loaded_width_m

# ── VAAKATAIVUTUSMOMENTTI ULKOPALKKISSA (sivulasituksen tuulikuorma) ──
# Yhdistelmä A – lumi johtava, tuuli toissijainen (ψ0_W = 0.6):
Md_outer_h_A = gammaQ * psi0_W * q_outer_wind_h_char * outer_span_m**2 / 8.0
# Yhdistelmä B – tuuli johtava, lumi toissijainen (ψ0_snow = 0.7):
Md_outer_h_B  = gammaQ * q_outer_wind_h_char * outer_span_m**2 / 8.0

# RHS-profiilit ulkoreunapalkkiin (S235, 90mm korkea)
# (b×h×t): b=leveys, h=korkeus (kantavaan suuntaan)
rhs_outer = [
    ("RHS 150×90×5", 150, 90, 5),
    ("RHS 150×90×6", 150, 90, 6),
    ("RHS 180×90×6", 180, 90, 6),
    ("RHS 200×90×5", 200, 90, 5),
    ("RHS 200×90×6", 200, 90, 6),
]

# ============================================================
# LISÄKUORMA OLEMASSA OLEVAAN 2×KP360×51
# ============================================================
# Kattotuolien sisäpäiden reaktiot siirtyvät 2×KP360×51-palkkiin pistekkuormina
# Pistekuormat 1134mm välein → ekvivalentti q lasketaan todellisista tributäärileveyksistä.

# ============================================================
# LUMIKINOSTUMA – EN 1991-1-3 §6.3 (talon seinä viereinen drift)
# ============================================================
# Talon katto on eri suunnassa (seinän suuntaisesti) kuin terassi (ulospäin).
# Kolmiokulman kohdalla talon seinä on korkeampi → kinostuma terassin katoille.
# h_seinä ja kaltevuudet laskettu automaattisesti yllä olevista korkeusmitoista.

def laske_kinostuma(h, b_panel, sk_, gamma_s=2.0, mu1_=0.8):
    """Laskee kinostuma-arvot EN 1991-1-3 §5.3.6 / §6.3 mukaan paneelitasolle,
    joka rajautuu korkeampaan talon seinään. b_panel = paneelin horisontaalinen
    pituus seinästä ulospäin (= sekä lower- että upper-roof -pituus, koska
    drift tapahtuu samalla paneelitasolla)."""
    ls = min(5.0 * h, b_panel, 15.0)
    ls = max(ls, 0.5 * h)
    mu2_h_  = gamma_s * h / sk_
    mu2_    = min(max(mu2_h_, mu1_), 2.0)
    s1_     = mu1_ * sk_
    s_dr_   = mu2_ * sk_
    return ls, mu2_, mu2_h_, s1_, s_dr_

# Korkea pää
ls_korkea, mu2_korkea, mu2_h_korkea, s1, s_drift_korkea = laske_kinostuma(
    h_seinä_korkea, b_terassi, sk, mu1_=mu1)
# Matala pää
ls_matala, mu2_matala, mu2_h_matala, _,  s_drift_matala  = laske_kinostuma(
    h_seinä_matala, b_terassi, sk, mu1_=mu1)

# Hallitseva: käytetään korkean pään arvoja mitoituksessa (max kuorma)
h_seinä  = h_seinä_korkea   # yhteensopivuus muun koodin kanssa
ls_drift = ls_korkea
mu2      = mu2_korkea
s_drift  = s_drift_korkea

# Lisäkuorma tasaiseen lumikuormaan nähden
delta_s_drift     = s_drift - s1
q_drift_udl_equiv = 0.5 * delta_s_drift * ls_drift

# ============================================================
# UUSI SEINÄ y=1800mm KOHDALLA  (kolmiolasi + laudoitus)
# ============================================================
# Uusi vaakapalkki (LP225×90) muodostaa uuden sisäseinän kohdalla y=1800mm.
# Seinä kulkee x-suunnassa (seinän suuntaisesti) ja täyttää aukon rakennuksen
# katon ja uuden palkin välillä.
#
# GEOMETRIA korkean pään poikkileikkauksessa (x=terrace_width):
#   Rakennuksen räystäs:      h_rakennus_korkea (luetaan geometriasta)
#   Laudoituksen alaraja:     h_rakennus_korkea - h_laudoitus
#   Uuden palkin yläpinta:    h_katto_inner
#   → Kolmiolasin korkeus max = h_rakennus_korkea - h_laudoitus - h_katto_inner
#
# Kolmio muodostuu koska rakennuksen katto viettää x-suunnassa:
#   matala pää: räystäs lähes katon tasolla → lasi häviää (laskettu x₀)
#
# PARAMETRISET LASKUT:
# Kolmiolasin ja laudoituksen mitat luetaan JSON-geometriasta (surf.triangle_glazing.gable
# ja surf.boarding.gable placement.u/v). Jos talon katon kaltevuus tai laudoituksen korkeus
# muuttuu, nämä arvot päivitetään suoraan geometry/terassi.json:iin.
_triangle = surface(_GEO, "surf.triangle_glazing.gable")
_boarding = surface(_GEO, "surf.boarding.gable")
b_kolmio_lasi_mm = float(_triangle["placement"]["u"]["length_mm"])
h_kolmio_lasi_mm = float(_triangle["placement"]["v"]["length_mm"])
h_laudoitus_mm   = float(_boarding["placement"]["v"]["length_mm"])
gable_gap_h_mm = h_rakennus_korkea - h_laudoitus_mm - h_katto_inner
b_kolmio_tot_mm = float(terrace_width)   # mm – laudoituksen koko leveys

# Pinta-alat
A_lasi      = 0.5 * (b_kolmio_lasi_mm/1000.0) * (h_kolmio_lasi_mm/1000.0)  # m² kolmio
A_laudoitus = (b_kolmio_tot_mm/1000.0) * (h_laudoitus_mm/1000.0)            # m² suorakaide (todellisuudessa suunnikas 12° kaltevuudella, ero <2%)

A_triangle  = A_lasi                       # m² (vain lasi → uusi palkki)

# ============================================================
# TUULIKUORMA UUDELLE SEINÄLLE (y=1800mm)
# ============================================================
# Tuulipaine (EN 1991-1-4 seinäpaine)
cp_end_wall = 0.8
w_end_wall  = cp_end_wall * qp_z   # kN/m²

# Tuulivoima: lasi → uusi päätypalkki, laudoitus → 2×KP350
F_wind_triangle  = w_end_wall * A_lasi       # kN (uusi palkki)
F_wind_laudoitus = w_end_wall * A_laudoitus  # kN (olemassa oleva 2×KP350)


# ============================================================
# PILARIKUORMAT (yhteenveto katokselle + terassille)
# ============================================================
# Olemassa oleva katos: kuorma pilareille (per pilari, 2 pilaria)
# Tähän tarvitaan katos-laskelmasta pilarireaktiot.
# Yksinkertaistus: kuorma katokselta per pilari = qd2 * L / 2 (approx)
# Mutta nämä tulevat katosskriptistä – tässä lasketaan vain terassikuorma.

# Sisäpilari (olemassa oleva 250×250mm):

# Ulkopilari (uusi): jatkuvan ulkoreunapalkin tarkat reaktiot
R_outer_per_pillar_left = outer_beam_reactions_uls[outer_support_xs_mm[0]]
R_outer_per_pillar_middle = outer_beam_reactions_uls[outer_support_xs_mm[1]]
R_outer_per_pillar_right = outer_beam_reactions_uls[outer_support_xs_mm[-1]]

# ============================================================
# PÄÄTYPALKKI – KATTOTUOLIREAKTIOT + KINOSTUMA + KOLMIOLASI
# ============================================================
# Palkki kantaa:
#   1) Kattotuolien sisäpään reaktiot (rafter-väli × koko leveys 7200mm)
#   2) Lumikinostuman lisäkuorman kattotuoleihin
#   3) Kolmiolasin omapainon (pysty) + tuulen (vaaka)
# Jänneväli: 6700mm (pilari–pilari)

L_paaty_mm   = 6700.0
L_paaty_m    = L_paaty_mm / 1000.0

# --- 1) Kattotuolireaktiot ---
# q_inner_add on ekvivalentti kuorma geometrian mukaisista kattotuolipistekuormista.
q_rafter_beam = q_inner_add   # kN/m (rafter-reaktiot palkille)
inner_beam_reactions_normal = beam_support_reactions(
    inner_beam_start_x_mm, inner_beam_end_x_mm, inner_support_xs_mm, inner_rafter_point_loads_normal)
inner_beam_reactions_normal_char = beam_support_reactions(
    inner_beam_start_x_mm, inner_beam_end_x_mm, inner_support_xs_mm, inner_rafter_point_loads_char)
inner_beam_moments_normal = beam_point_moments(
    inner_beam_start_x_mm, inner_beam_end_x_mm, inner_beam_reactions_normal, inner_rafter_point_loads_normal)
Md_rafter_beam_normal = max(inner_beam_moments_normal.values())

# --- 2) Lumikinostuma lisäkuorma ---
# Kinostuma kasvattaa rafter-reaktioita terassin sisäpäässä.
# Drift on triangulaarinen: huippu rakennuksen seinällä, nolla ls_drift-päässä.
# Rafter-terassin sisäpään drift-arvo luetaan pilarilinjan y-koordinaatista:
y_inner_drift = inner_y / 1000.0    # m
if ls_drift > y_inner_drift:
    s_drift_inner = s_drift * (1.0 - y_inner_drift / ls_drift)  # kN/m² palkin kohdalla
else:
    s_drift_inner = 0.0
# Drift-tapaus: lasketaan kokonaan (yksinkertaisempi kuin delta-lähestymistapa)
q_drift_rafter = rafter_spacing * s_drift_inner   # kN/m (char drift-snow per kattotuoli)

# Yksinkertaisemmin: lasketaan drift-tapaus kokonaan
qd_rafter_drift = gammaG * gk_line + gammaQ * (rafter_spacing * s_drift_inner)
R_inner_drift   = qd_rafter_drift * rafter_inner_reaction_factor
q_inner_drift   = R_inner_drift / rafter_spacing
inner_rafter_point_loads_drift = build_rafter_support_loads(
    rafter_beam_load_xs_mm,
    rafter_trib_widths_m,
    gammaG * gk_total + gammaQ * s_drift_inner,
    rafter_inner_reaction_factor,
)
inner_beam_reactions_drift = beam_support_reactions(
    inner_beam_start_x_mm, inner_beam_end_x_mm, inner_support_xs_mm, inner_rafter_point_loads_drift)
inner_beam_moments_drift = beam_point_moments(
    inner_beam_start_x_mm, inner_beam_end_x_mm, inner_beam_reactions_drift, inner_rafter_point_loads_drift)
Md_rafter_beam_drift = max(inner_beam_moments_drift.values())

# --- 2b) TARKENNETTU KINOSTUMA: paikallinen h(x) palkin matkalla ---
# EN 1991-1-3 §6.3 + EC0 §2.1: tarkempi analyysi on perusteltu kun h
# vaihtelee merkittävästi palkin matkalla. Seinä viettää 12° → h(x) lineaarinen.
# Kattotuolien pistekuormat lasketaan yksitellen paikallisesta h(x)-arvosta.

pillar_size_mm = 250.0
x_wall_start   = pillar_size_mm            # palkin alku seinäkoordinaatissa (mm)

refined_snow_loads = []
for x_beam_mm in rafter_beam_load_xs_mm:
    x_wall = x_beam_mm - inner_beam_start_x_mm + x_wall_start

    # Paikallinen h(x) seinällä — suoraan geometriasta
    h_local = (h_rakennus_at_x(x_wall) - h_katto_inner) / 1000.0
    h_local = max(h_local, 0.0)

    # Paikallinen kinostuma
    ls_loc, _, _, _, s_drift_loc = laske_kinostuma(h_local, b_terassi, sk, mu1_=mu1)

    # Drift-arvo rafteri-sisäpään kohdalla (y = inner_y)
    if ls_loc > y_inner_drift:
        s_drift_inner_loc = s_drift_loc * (1.0 - y_inner_drift / ls_loc)
    else:
        s_drift_inner_loc = 0.0

    refined_snow_loads.append(max(mu1 * sk, s_drift_inner_loc))

inner_rafter_point_loads_refined = build_rafter_support_loads(
    rafter_beam_load_xs_mm,
    rafter_trib_widths_m,
    [gammaG * gk_total + gammaQ * s_loc for s_loc in refined_snow_loads],
    rafter_inner_reaction_factor,
)
inner_rafter_point_loads_refined_char = build_rafter_support_loads(
    rafter_beam_load_xs_mm,
    rafter_trib_widths_m,
    [gk_total + s_loc for s_loc in refined_snow_loads],
    rafter_inner_reaction_factor,
)
inner_beam_reactions_refined = beam_support_reactions(
    inner_beam_start_x_mm, inner_beam_end_x_mm, inner_support_xs_mm, inner_rafter_point_loads_refined)
inner_beam_reactions_refined_char = beam_support_reactions(
    inner_beam_start_x_mm, inner_beam_end_x_mm, inner_support_xs_mm, inner_rafter_point_loads_refined_char)
inner_beam_moments_refined = beam_point_moments(
    inner_beam_start_x_mm, inner_beam_end_x_mm, inner_beam_reactions_refined, inner_rafter_point_loads_refined)

Md_refined_max = max(inner_beam_moments_refined.values())
x_Md_refined_max_abs_mm = max(inner_beam_moments_refined, key=inner_beam_moments_refined.get)
x_Md_refined_max = (x_Md_refined_max_abs_mm - inner_beam_start_x_mm) / 1000.0
h_at_Mmax = (
    h_rakennus_at_x(x_Md_refined_max_abs_mm - inner_beam_start_x_mm + x_wall_start) - h_katto_inner
) / 1000.0
q_beam_drift_refined_avg = sum(load_kN for _, load_kN in inner_rafter_point_loads_refined) / roof_loaded_width_m
q_beam_drift_refined_avg_char = (
    sum(load_kN for _, load_kN in inner_rafter_point_loads_refined_char) / roof_loaded_width_m
)

# --- 3) Kolmiolasi + laudoitus ---
# Lasi: vain lasitettu osa (5500×1250mm) → paino + tuuli lasille
# Laudoitus: jäljelle jäävä osa → paino kevyempi, sama tuuli
gk_lasi      = 0.30     # kN/m² (karkaistu lasi ~30 kg/m²)
gk_laudoitus = 0.15     # kN/m² (peltilaudoitus/paneli ~15 kg/m²)
cp_paaty     = 0.8

# Lasi-osuus
h_kolmio_max  = h_kolmio_lasi_mm / 1000.0   # m

# Laudoitus-osuus

# Kolmio jakaa kuorman triangulaarisena (max korkea pää, 0 matala pää)
# Lasi: triangulaarinen Md = 0.0642×q_max×L² (q_max = kuorma max-kohdassa)
q_lasi_max   = gammaG * gk_lasi * h_kolmio_max              # kN/m (pysty, lasi kolmio)
q_laud_max   = gammaG * gk_laudoitus * (h_laudoitus_mm/1000.0)  # kN/m (pysty, laudoitus suorakaide)
q_wind_max   = gammaQ * cp_paaty * qp_z * h_kolmio_max      # kN/m (vaaka, vain lasi → uusi palkki)

# --- Kokonaiskuorma palkille (ULS) ---
# Kolmiolasin ja laudoituksen pystykuorma (triangulaarinen)
q_beam_normal  = q_rafter_beam
q_beam_drift   = q_inner_drift
Md_lasi_vert   = 0.0642 * q_lasi_max * L_paaty_m**2
Md_laud_vert   = 0.0642 * q_laud_max * L_paaty_m**2
Md_lasi_horiz  = 0.0642 * (gammaQ * cp_paaty * qp_z * h_kolmio_max) * L_paaty_m**2  # vaaka, vain lasi

# Normaali lumi+tuuli + lasi (laudoitus → olemassa oleville 2×KP350, ei tähän palkkiin)
Md_beam_normal = Md_rafter_beam_normal + Md_lasi_vert

# Drift-tapaus (konservatiivinen: h_max koko matkalla)
Md_beam_drift  = Md_rafter_beam_drift + Md_lasi_vert
Md_beam_horiz  = Md_lasi_horiz

# Drift-tapaus (tarkennettu: paikallinen h(x), EC0 §2.1)
Md_beam_drift_refined = Md_refined_max + Md_lasi_vert

# Hallitseva tapaus pystysuunnassa (tarkennettu analyysi)
Md_beam_gov    = max(Md_beam_normal, Md_beam_drift_refined)
if Md_beam_normal >= Md_beam_drift_refined:
    gov_beam_case = "normaali lumi+tuuli"
else:
    gov_beam_case = f"kinostuma tarkennettu (h(x) vaihteleva)"

# --- L-profiilien ominaisuudet ---
def angle_props(a_mm, t_mm):
    """Tasainen L-profiili a×a×t: ominaisuudet taivutuksessa vaakasuunnassa."""
    a = float(a_mm); t = float(t_mm)
    A = (2*a - t) * t
    y_bar = (a*t*(a/2.0) + (a-t)*t*(t + (a-t)/2.0)) / A
    Ix = (a*t**3/12 + a*t*(y_bar - t/2.0)**2 +
          t*(a-t)**3/12 + (a-t)*t*(t + (a-t)/2.0 - y_bar)**2)
    Wy_bot = Ix / y_bar
    Wy_top = Ix / (a - y_bar)
    W_crit = min(Wy_bot, Wy_top)
    return A, y_bar, Ix, Wy_bot, Wy_top, W_crit

def unequal_angle_props(a_long_mm, a_short_mm, t_mm):
    """Epätasainen L-profiili: pitkä haara (a_long) pystysuoraan,
    lyhyt haara (a_short) vaakasuoraan (kattotuolien tukipinta).
    Palauttaa (A, y_bar, Ix, Wy_bot, Wy_top, W_crit, Iz, Wz_crit)."""
    al = float(a_long_mm); ash = float(a_short_mm); t = float(t_mm)
    A_h  = ash * t
    A_v  = (al - t) * t
    A    = A_h + A_v
    y_h  = t / 2.0
    y_v  = t + (al - t) / 2.0
    y_bar = (A_h * y_h + A_v * y_v) / A
    Ix = (ash*t**3/12 + A_h*(y_bar-y_h)**2 +
          t*(al-t)**3/12 + A_v*(y_v-y_bar)**2)
    Wy_bot = Ix / y_bar
    Wy_top = Ix / (al - y_bar)
    W_crit = min(Wy_bot, Wy_top)
    # Iz – taivutus vaaka-akselista (vaakatuuli päätypalkkiin)
    x_bar = (A_h * ash/2.0 + A_v * t/2.0) / A
    Iz = (t * ash**3/12 + A_h*(ash/2.0 - x_bar)**2 +
          (al-t)*t**3/12 + A_v*(t/2.0 - x_bar)**2)
    Wz_crit = min(Iz / x_bar, Iz / (ash - x_bar))
    return A, y_bar, Ix, Wy_bot, Wy_top, W_crit, Iz, Wz_crit

fy_S235  = 235.0   # N/mm²
fy_S355  = 355.0   # N/mm²

# Epätasaiset L-profiilit S355J2 (päätypalkki: pitkä haara pysty)
# (nimi, a_long, a_short, t)
unequal_l_profiles_S355 = [
    ("L150×100×10", 150, 100, 10),
    ("L150×100×12", 150, 100, 12),
    ("L180×110×12", 180, 110, 12),
    ("L200×100×12", 200, 100, 12),
    ("L200×150×15", 200, 150, 15),
]

# ============================================================
# SAHATAVARA C24 – KATTOTUOLIVAIHTOEHDOT
# ============================================================
# SC2 (lasituksen alla, suojattu suoralta sateelta): kmod = 0.8
# SC3 (täysin ulkona):  kmod = 0.65
# → Käytetään SC2 (kmod=0.8), koska paneelilasitus suojaa rakenteen
fm_k_C24   = 24.0     # N/mm² EN 338 C24
fv_k_C24   =  4.0     # N/mm²
E_C24      = 11000.0  # N/mm²
gammaM_C24 =  1.3
kmod_C24   =  0.8     # SC2, lumi (keskipitkäaikainen)
rho_C24    = 420.0    # kg/m³
gamma_C24  = rho_C24 * 9.81 / 1000.0  # kN/m³

fm_d_C24 = kmod_C24 * fm_k_C24 / gammaM_C24
fv_d_C24 = kmod_C24 * fv_k_C24 / gammaM_C24

# Standard Finnish sawn timber (sahatavara), height × width
# "n" = lukumäärä rinnakkain (1 tai 2)
timber_sizes = [
    # (nimi,        h_mm, b_eff_mm, kpl)
    ("148×48",       148,  48, 1),
    ("173×48",       173,  48, 1),
    ("198×48",       198,  48, 1),
    ("223×48",       223,  48, 1),
    ("2×148×48",     148,  96, 2),   # kaksi limittäin/liimattu
    ("2×173×48",     173,  96, 2),
    ("2×198×48",     198,  96, 2),
]

results_timber = []
for (nm, h_t, b_t, n_t) in timber_sizes:
    W_t     = b_t * h_t**2 / 6.0
    I_t     = b_t * h_t**3 / 12.0
    A_t     = b_t * h_t
    MRd_t   = fm_d_C24 * W_t / 1.0e6
    VRd_t   = fv_d_C24 * A_t / 1.5e3
    g_self_t = (b_t / 1000.0) * (h_t / 1000.0) * gamma_C24   # kN/m
    Md_t_i, q_sls_t_i = rafter_profile_md(g_self_t)
    qd_eq   = Md_t_i * 8.0 / rafter_span_m**2
    Vd_t    = qd_eq * rafter_span_m / 2.0
    eta_M_t = Md_t_i / MRd_t * 100.0
    eta_V_t = Vd_t / VRd_t * 100.0
    EI_t    = E_C24 * I_t
    delta_t = 5.0 * q_sls_t_i * rafter_span_mm**4 / (384.0 * EI_t)
    delta_lim_t = rafter_span_mm / 300.0
    hb_single = h_t / 48.0 if n_t == 1 and b_t == 48 else h_t / b_t
    results_timber.append({
        'name': nm, 'h': h_t, 'b': b_t, 'n': n_t,
        'W_cm3': W_t / 1e3, 'MRd': MRd_t, 'VRd': VRd_t, 'Vd': Vd_t,
        'eta_M': eta_M_t, 'eta_V': eta_V_t,
        'delta': delta_t, 'delta_lim': delta_lim_t,
        'g_self': g_self_t, 'hb': hb_single,
        'ok_M': eta_M_t <= 100,
        'ok_V': eta_V_t <= 100,
        'ok_delta': delta_t <= delta_lim_t,
    })

# ── KATTOTUOLIN KAKSITAIVUTUS α=7.25° (EN 1995-1-1 §6.1.6 / EN 1993-1-1 §6.2.9) ──
# Kallistettu kattotuoli: pystysuora kuorma → My (vahva) + Mz = My×tan(α) (heikko).
# Puu suorakaide, km=0.7:  η_biax = My/MRd,y + 0.7 × Mz/MRd,z
#   = η_M × (1 + 0.7 × tan(α) × h/b)
_tan_slope = math.tan(slope_rad)
for r in results_lp + results_timber:
    h_r, b_r = r['h'], r['b']
    r['eta_biax'] = r['eta_M'] * (1.0 + 0.7 * _tan_slope * h_r / b_r)
    r['ok_biax']  = r['eta_biax'] <= 100.0

# Teräs EN 1993-1-1 §6.2.9:  η_biax = My/MRd,y + Mz/MRd,z = η_M × (1 + Wy/Wz × tan(α))
_Wz_steel_rafter = {
    "IPE100": 5.79, "IPE120": 8.65, "IPE140": 12.31, "IPE160": 16.66,
    "HEA120": 38.5, "HEA140": 55.6, "HEA160": 77.0,
}
for r in results_steel:
    Wz = _Wz_steel_rafter.get(r['name'])
    if Wz:
        r['eta_biax'] = r['eta_M'] * (1.0 + r['W_cm3'] / Wz * _tan_slope)
    else:
        r['eta_biax'] = r['eta_M']
    r['ok_biax'] = r['eta_biax'] <= 100.0

# ============================================================
# TULOSTUS
# ============================================================
W = 62
dw = "=" * W

print(dw)
print("  LASITETTU TERASSI – KUORMITUSLASKENTA – ETELÄSUOMI")
print("  EN 1990 / EN 1991-1-1/3/4 / EN 1995-1-1")
print(dw)

print("\n── GEOMETRIA ──────────────────────────────────────────────")
print(f"  Terassin leveys            {terrace_width} mm  (x-suunta)")
print(f"  Syvyys pilareista          {terrace_depth} mm  (y-suunta, ulospäin)")
print(f"  Kaltevuus (y, ulospäin)    {slope_deg:.1f}°  ({h_katto_inner:.0f}mm → {h_katto_outer:.0f}mm, Δ={h_katto_inner-h_katto_outer:.0f}mm / {terrace_depth:.0f}mm)")
print(f"  Kaltevuus (x, rakennus)   {slope_rakennus_deg:.1f}°  ({h_rakennus_korkea:.0f}mm → {h_rakennus_matala:.0f}mm / {terrace_width:.0f}mm)")
print(f"  h_seinä (kinostuma)       {h_seinä_korkea:.2f}m (korkea) / {h_seinä_matala:.2f}m (matala)")
print(f"  Kinostuman x-raja         este rajattu pääseinälle x = {drift_obstacle_x0_mm:.0f}…{drift_obstacle_x1_mm:.0f} mm")
print(f"  Sisäpilari seinästä        {inner_y} mm  (olemassa oleva 250×250)")
print(f"  Ulkopilari seinästä        {outer_y} mm  (uusi)")
print(f"  Kattotuolijako             {rafter_spacing*1000:.0f} mm  (paneelin leveys, tuplat kattotuolit liitoskohdissa)")
print(f"  Kattotuoleja               {n_rafters} kpl  (geometriasta)")
print(f"  Kattotuolin jänneväli      {rafter_span_mm:.0f} mm  (sisäpalkin kl {rafter_inner_y:.0f}mm → ulkopilarin kl {rafter_outer_y:.0f}mm, b_inner={b_inner_beam_mm}mm)")
print(f"  Ulkoreun. palkki jänneväli {outer_span_mm:.0f} mm  (2 väliä, 3 pilaria)")

print("\n── PANEELIT ───────────────────────────────────────────────")
print(f"  Longi Himo X10 LR7:  {panel_w_mm:.0f}×{panel_d_mm:.0f}×30mm, {panel_mass_kg:.0f}kg/kpl")
print(f"  Paneeleja:           {n_panels_w}×{n_panels_d} = {n_panels_total} kpl")
print(f"  Kokonaispaino:       {n_panels_total}×{panel_mass_kg:.0f}kg = {n_panels_total*panel_mass_kg:.0f}kg = {g_panels:.2f} kN")
print(f"  Paneelien hajakuorma:{gk_panels:.3f} kN/m²  +  runko {gk_frame:.2f} kN/m²")
print(f"  Pysyvä yhteensä  gk: {gk_total:.3f} kN/m²")

print("\n── LUMIKUORMA ─────────────────────────────────────────────")
print(f"  sk = {sk} kN/m²,  μ1 = {mu1} (α={slope_deg:.0f}°)  →  s = {s_roof:.2f} kN/m²")

print("\n── TUULIKUORMA ────────────────────────────────────────────")
print(f"  qp(z={z_ref:.0f}m) = {qp_z:.3f} kN/m²  (vb0={vb0} m/s, maasto II)")
print(f"  Tapaus A – LASIT AUKI (EN 1991-1-4 taulukko 7.7, vapaa katos Φ=1):")
print(f"    cp,net alaspäin = {cp_net_down:+.2f}  →  w_down = {w_down:.3f} kN/m²  (hallitseva lumi+tuuli)")
print(f"    cp,net ylöspäin = {cp_net_up:+.2f}  →  w_up   = {w_up:.3f} kN/m²")
print(f"  Tapaus B – LASIT KIINNI (EN 1991-1-4 taulukko 7.4 + cp,i):")
print(f"    cp,e,H (θ=0°)   = {cpe_H_t0:+.2f}  (tuuli matalan räystään puolelta)")
print(f"    cp,e,H (θ=180°) = {cpe_H_t180:+.2f}  (tuuli rakennuksen yli) ← hallitseva imu")
print(f"    cp,i (imu pahin)= {cpi_unfav_up:+.2f}  (paine sisällä, ei dominoivaa aukkoa)")
print(f"    cp,net imu      = {cp_net_up_closed:+.2f}  →  w_up   = {w_up_closed:.3f} kN/m²  ← PAHEMPI kuin auki!")
print(f"    cp,i (alas)     = {cpi_unfav_down:+.2f}  →  cp,net alas = {cp_net_down_closed:+.2f}  →  {w_down_closed:.3f} kN/m²  (pienempi kuin auki)")
print(f"  → Imusuunta: kiinni-tapaus hallitsee ({w_up_closed:.3f} vs {w_up:.3f} kN/m²)")
print(f"  → Alaspäin:  auki-tapaus hallitsee   ({w_down:.3f} vs {w_down_closed:.3f} kN/m²)")
print(f"")
print(f"  NOSTOKUORMA (ULS, lasit kiinni, tuuli johtava):")
print(f"    qk,w,up = {qk_w_up_closed:.3f} kN/m  →  qd,nosto = 1.5×{qk_w_up_closed:.3f} − 0.9×{gk_line:.3f} = {qd_uplift_closed:.3f} kN/m")
if qd_uplift_closed > 0:
    print(f"    Tukireaktio nostoon per kattotuoli: R_uplift = {R_uplift_closed:.2f} kN  ← kiinnitys suunniteltava!")
else:
    print(f"    Nettokuorma negatiivinen ({qd_uplift_closed:.3f} kN/m) → pysyvä kuorma riittää pitämään → ei nostoa ✓")

print("\n── KATTOTUOLIN KUORMAT ────────────────────────────────────")
print(f"  Rafter-väli b = {rafter_spacing:.3f} m")
print(f"  Pysyvä       gk = {gk_line:.3f} kN/m")
print(f"  Lumi         qk = {qk_snow:.3f} kN/m")
print(f"  Tuuli (alas) qk = {qk_w_down:.3f} kN/m")
print(f"  Mitoituskuorma (ULS): qd = {qd_down:.3f} kN/m")
print(f"  Hallitseva tapaus:    {gov_case}")
print(f"  Mitoitusmomentti Md = {Md_rafter_gov:.2f} kNm")
print(f"  Huoltokuorma tapaus Md = {Md_rafter_huolto:.2f} kNm")
print(f"  Lumi+tuuli tapaus  Md = {Md_rafter:.2f} kNm")

print(f"\n── KATTOTUOLIN MATERIAALIVERTAILU  (jänneväli {rafter_span_mm:.0f}mm) ───")
print(f"  Mitoitusmomentti Md: profiilikohtainen (oma-paino + paneeli + kiinnikkeet + lumi + tuuli)")
print(f"  Pohjakuorma (ilman kattotuolia): gk_no_rafter = {gk_no_rafter:.3f} kN/m²  (yleislaskelmat: gk_frame = {gk_frame:.2f} kN/m²)")
print(f"  η_biax: kaksitaivutus α={slope_deg:.1f}° (EN 1995-1-1 §6.1.6 / EN 1993-1-1 §6.2.9)")
print()
print(f"  {'Profiili':<13} {'kg/m':>6} {'W [cm³]':>8} {'MRd':>9} {'η_M':>7} {'η_biax':>8} {'δ/lim':>9}  {'OK?'}")
print(f"  {'-'*13} {'-'*6} {'-'*8} {'-'*9} {'-'*7} {'-'*8} {'-'*9}  {'-'*10}")
print("  --- LIIMAPUU GL30c (SC2, kmod=0.8) ---")
for r in results_lp:
    ok_all = r['ok_M'] and r['ok_V'] and r['ok_delta'] and r['ok_biax']
    ok = '✓' if ok_all else '✗'
    tag = ' ← suositus' if ok_all and \
          all(not (rr['ok_M'] and rr['ok_V'] and rr['ok_delta'] and rr['ok_biax']) for rr in results_lp[:results_lp.index(r)]) else ''
    print(f"  {r['name']:<13} {r['g_self']*1000/9.81:>6.1f} {r['W_cm3']:>8.0f} {r['MRd']:>8.2f}kNm {r['eta_M']:>6.1f}% {r['eta_biax']:>7.1f}% {r['delta']:>5.1f}/{r['delta_lim']:.1f}mm  {ok}{tag}")
print("  --- TERÄS S235 ---")
for r in results_steel:
    ok_all = r['ok_M'] and r['ok_V'] and r['ok_delta'] and r['ok_biax']
    ok = '✓' if ok_all else '✗'
    tag = ' ← suositus' if ok_all and \
          all(not (rr['ok_M'] and rr['ok_V'] and rr['ok_delta'] and rr['ok_biax']) for rr in results_steel[:results_steel.index(r)]) else ''
    print(f"  {r['name']:<13} {r['g_self']*1000/9.81:>6.1f} {r['W_cm3']:>8.1f} {r['MRd']:>8.2f}kNm {r['eta_M']:>6.1f}% {r['eta_biax']:>7.1f}% {r['delta']:>5.1f}/{r['delta_lim']:.1f}mm  {ok}{tag}")
print("  --- SAHATAVARA C24 (SC2, kmod=0.8, fm,d=14.8 N/mm²) ---")
for r in results_timber:
    ok_all = r['ok_M'] and r['ok_V'] and r['ok_delta'] and r['ok_biax']
    ok = '✓' if ok_all else '✗'
    warn = f"  h/b={r['hb']:.1f}" if r['b'] == 48 and r['hb'] > 5 else ''
    tag = ' ← suositus' if ok_all and \
          all(not (rr['ok_M'] and rr['ok_V'] and rr['ok_delta'] and rr['ok_biax']) for rr in results_timber[:results_timber.index(r)]) else ''
    print(f"  {r['name']:<13} {r['g_self']*1000/9.81:>6.1f} {r['W_cm3']:>8.0f} {r['MRd']:>8.2f}kNm {r['eta_M']:>6.1f}% {r['eta_biax']:>7.1f}% {r['delta']:>5.1f}/{r['delta_lim']:.1f}mm  {ok}{tag}{warn}")

# ── LOVETUSTARKISTUS ULKOTUELLA (EN 1995-1-1 §6.5.2) ────────────────────────
# Kattotuoli jatkaa ulkopalkin yli räystääksi → pääpalkki lovetettu alapinnasta
# ulkopalkin LP225×90 kohdalla, jotta kattotuoli saadaan 50mm alemmaksi.
#
# Parametrit – muuta näitä:
notch_depth_mm  = 50    # mm  lovennuksen syvyys (alapinnasta ylöspäin)
notch_rafter_h  = 173   # mm  kattotuolin korkeus (h)
notch_rafter_b  = 48    # mm  kattotuolin leveys (b), 48 tai 96 (tuplat)
# LP225×90 tukipinnan leveys (y-suunnassa) = 90mm → konservatiivisesti x=0
# (lovennuksen sisäreuna tasan tukireunan kanssa)

# EN 1995-1-1 §6.5.2: lovennetun tukialueen leikkaustarkistus
# k_n: 5.0 sahatavara (C24), 6.5 liimapuu (GL)
# x = 0 (konservatiivinen, lovennuksen sisäreuna tukireunalla)
h_ef_notch   = notch_rafter_h - notch_depth_mm   # mm, jäävä korkeus
alpha_notch  = h_ef_notch / notch_rafter_h        # = h_ef / h
i_notch      = notch_depth_mm / notch_rafter_h    # = (h-h_ef) / h  (= 1-alpha)

# k_v kaava (EN 1995-1-1 eq. 6.60), x=0 (konservatiivinen):
def kv_notch(k_n, alpha, i, h_mm):
    denom = math.sqrt(h_mm * (alpha**2 * (1 - alpha)))
    if denom < 1e-9:
        return 1.0
    kv = k_n * (1.0 + 1.1 * i**1.5 / math.sqrt(alpha)) / denom
    return min(kv, 1.0)

kv_C24 = kv_notch(5.0, alpha_notch, i_notch, notch_rafter_h)
kv_GL  = kv_notch(6.5, alpha_notch, i_notch, notch_rafter_h)

# Mitoittava leikkausvoima ulkotuella (ULS, per kattotuoli) = R_outer_rafter
Vd_notch = R_outer_rafter   # kN, per kattotuoli

# Leikkauskestävyys lovennetulla poikkileikkauksella:
# V_Rd = (2/3) × b × h_ef × k_v × f_v,d
fvd_C24  = kmod_C24 * fv_k_C24 / gammaM_C24     # N/mm²
fvd_GL   = lp_kmod  * 3.5      / 1.25            # N/mm², GL30c

VRd_notch_C24 = (2.0/3.0) * notch_rafter_b * h_ef_notch * kv_C24 * fvd_C24 / 1000.0  # kN
VRd_notch_GL  = (2.0/3.0) * notch_rafter_b * h_ef_notch * kv_GL  * fvd_GL  / 1000.0  # kN

eta_notch_C24 = Vd_notch / VRd_notch_C24 * 100.0
eta_notch_GL  = Vd_notch / VRd_notch_GL  * 100.0

# Tulostus
print(f"\n── LOVETUSTARKISTUS ULKOTUELLA (EN 1995-1-1 §6.5.2) ────────────────")
print(f"  Lovennuksen geometria: h={notch_rafter_h}mm, lovetus={notch_depth_mm}mm alta")
print(f"  → h_ef = {h_ef_notch}mm, α = {alpha_notch:.3f}, i = {i_notch:.3f}")
print(f"  Nyrkkisääntö: lovetus ≤ h/3 = {notch_rafter_h/3:.0f}mm  → {notch_depth_mm}mm {'OK ✓' if notch_depth_mm <= notch_rafter_h/3 else '✗ YLITTYY'}")
print(f"  Mitoittava leikkausvoima Vd = {Vd_notch:.2f} kN/kattotuoli (= R_outer_rafter)")
print(f"  Kattotuolin leveys b = {notch_rafter_b}mm  (muuta notch_rafter_b: 48=yksi, 96=tuplat)")
print(f"")
print(f"  Materiaali    k_n   k_v    f_v,d    V_Rd     η       Tulos")
print(f"  {'-'*60}")
for (mat, kn, kv, fvd, VRd, eta) in [
        ("C24 sah.  ", 5.0, kv_C24, fvd_C24, VRd_notch_C24, eta_notch_C24),
        ("GL30c lp. ", 6.5, kv_GL,  fvd_GL,  VRd_notch_GL,  eta_notch_GL),
    ]:
    ok = "OK ✓" if eta <= 100 else "✗"
    print(f"  {mat}  {kn:.1f}  {kv:.3f}  {fvd:.2f}N/mm²  {VRd:.2f}kN  {eta:5.1f}%  {ok}")
print(f"")
print(f"  HUOM: x=0 konservatiivinen (lovennuksen sisäreuna = tukireunan sisäreuna)")
print(f"  Jos lovennuksen sisäreuna on LP225 tukipinnan sisäpuolella:")
print(f"  x = puolet tukipintaa = ~45mm → k_v kasvaa hieman (epäkonservatiivisempi)")

# Heikko akseli W_z arvot IPE/HEA (cm³), ArcelorMittal: Wz = Iz/(bf/2)
Wz_steel_outer = {
    "IPE100": 5.79, "IPE120": 8.65, "IPE140": 12.31, "IPE160": 16.66,
    "HEA120": 38.5, "HEA140": 55.6, "HEA160": 77.0,
}

print(f"\n── ULKOREUNANEN PALKKI (2 × {outer_span_mm:.0f}mm jännevälit) ────────")
print(f"  Tyypillinen kattotuolireaktio ulkopäässä: {R_outer_rafter:.2f} kN / {rafter_spacing*1000:.0f}mm")
print(f"  Ekv. q ulkopalkkiin (ULS):        {q_outer_beam:.3f} kN/m")
print(f"  Ekv. q ulkopalkkiin (SLS):        {q_outer_sls:.3f} kN/m")
print(f"  Md (pysty, maks.abs.):            {Md_outer_beam:.2f} kNm  (kenttä +{Md_outer_beam_pos:.2f} / keskituella {Md_outer_beam_neg:.2f} kNm)")
print(f"  Taipumaraja: δ_lim(L/300)={outer_span_mm/300:.0f}mm (EN 1990 §A1.4)")
print(f"  Sivulasituksen tuulikuorma (EN 1991-1-4, cp,wall={cp_wall_net:.1f}):")
print(f"    h_lasitus={h_lasitus_m:.2f}m, h_trib={h_trib_lasit:.2f}m, qk,h={q_outer_wind_h_char:.3f} kN/m")
print(f"    Md,h,A (lumi johtava, ψ0_W=0.6): {Md_outer_h_A:.2f} kNm")
print(f"    Md,h,B (tuuli johtava):           {Md_outer_h_B:.2f} kNm,  Md,v,B={Md_outer_v_B:.2f} kNm")
print(f"  Kaksiakseli: η=Md_v/MRd_v + km×Md_h/MRd_h ≤ 1.0  (km=1.0 teräs, km=0.7 puu)")
print()
print(f"  {'Profiili':<30} {'W [cm³]':>8} {'MRd':>9} {'η_M':>7} {'η_biax':>7}  {'kg/m':>6}  {'δ/L300':>8}  {'OK?'}")
print(f"  {'-'*30} {'-'*8} {'-'*9} {'-'*7} {'-'*7}  {'-'*6}  {'-'*8}  {'-'*8}  {'-'*4}")
# IPE/HEA teräsprofiilit (pystyasennossa)
for (nm, h_mm, b_f, W_el, I_cm4, m_kgm, fy) in steel_options:
    fm_d_s = fy / gammaM0_steel
    MRd_o  = fm_d_s * (W_el * 1e3) / 1.0e6
    eta_o  = Md_outer_beam / MRd_o * 100.0
    I_mm4  = I_cm4 * 1e4
    d_o    = 5 * q_outer_sls * outer_span_mm**4 / (384 * E_steel * I_mm4)
    dlim_o = outer_span_mm / 300.0
    # Heikko akseli (vaakataivutus): Wz = Iz / (bf/2), arvot ArcelorMittal
    Wz_cm3_o = Wz_steel_outer.get(nm, W_el * 0.15)  # fallback: 15% Wy (konservatiivinen)
    MRd_ho   = fm_d_s * (Wz_cm3_o * 1e3) / 1.0e6
    eta_A_o  = eta_o + Md_outer_h_A / MRd_ho * 100.0
    eta_B_o  = Md_outer_v_B / MRd_o * 100.0 + Md_outer_h_B / MRd_ho * 100.0
    eta_bx_o = max(eta_A_o, eta_B_o)
    ok_o = '✓' if eta_o <= 100 and d_o <= dlim_o and eta_bx_o <= 100 else '✗'
    print(f"  {nm:<30} {W_el:>8.1f} {MRd_o:>8.2f}kNm {eta_o:>6.1f}% {eta_bx_o:>7.1f}%  {m_kgm:>5.1f}  {d_o:>5.1f}/{dlim_o:.0f}mm  {'✓' if eta_o<=100 and eta_bx_o<=100 else ok_o}")
# LP-profiilit pystyasennossa
for r in results_lp:
    eta_o  = Md_outer_beam / r['MRd'] * 100.0
    I_mm4  = r['I_cm4'] * 1e4
    d_o    = 5 * q_outer_sls * outer_span_mm**4 / (384 * E_lp * I_mm4)
    dlim_o = outer_span_mm / 300.0
    # Heikko akseli: Wz = h*b²/6
    Wz_lp   = r['h'] * r['b']**2 / 6.0
    fm_d_lp = lp_kmod * 30.0 / 1.25
    MRd_ho  = fm_d_lp * Wz_lp / 1.0e6
    eta_A_o = eta_o + 0.7 * Md_outer_h_A / MRd_ho * 100.0
    eta_B_o = Md_outer_v_B / r['MRd'] * 100.0 + 0.7 * Md_outer_h_B / MRd_ho * 100.0
    eta_bx_o = max(eta_A_o, eta_B_o)
    ok_o = '✓' if eta_o <= 100 and d_o <= dlim_o and eta_bx_o <= 100 else '✗'
    m_kgm_lp = r['b'] * r['h'] * 500 / 1e6
    print(f"  {r['name']:<30} {r['W_cm3']:>8.0f} {r['MRd']:>8.2f}kNm {eta_o:>6.1f}% {eta_bx_o:>7.1f}%  {m_kgm_lp:>5.1f}  {d_o:>5.1f}/{dlim_o:.0f}mm  {'✓' if eta_o<=100 and eta_bx_o<=100 else ok_o}")
# RHS-profiilit, 90mm korkea (soveltuu lappeeltaan asennukseen)
print(f"\n  --- RHS teräsputki S235, 90mm korkea (tasainen ulkopinta pilarien kanssa) ---")
for (nm, bw, h_r, t) in rhs_outer:
    I_r   = (bw*h_r**3 - (bw-2*t)*(h_r-2*t)**3) / 12.0
    W_r   = I_r / (h_r / 2.0)
    MRd_r = 235 / gammaM0_steel * W_r / 1e6
    eta_r = Md_outer_beam / MRd_r * 100.0
    d_r   = 5 * q_outer_sls * outer_span_mm**4 / (384 * E_steel * I_r)
    dlim_r = outer_span_mm / 300.0
    # Heikko akseli RHS: Iz = (h_r*bw³ - (h_r-2t)*(bw-2t)³)/12
    Iz_r  = (h_r*bw**3 - (h_r-2*t)*(bw-2*t)**3) / 12.0
    Wz_r  = Iz_r / (bw / 2.0)
    MRd_hr = 235 / gammaM0_steel * Wz_r / 1e6
    eta_A_r  = eta_r + Md_outer_h_A / MRd_hr * 100.0
    eta_B_r  = Md_outer_v_B / MRd_r * 100.0 + Md_outer_h_B / MRd_hr * 100.0
    eta_bx_r = max(eta_A_r, eta_B_r)
    ok_r = '✓' if eta_r <= 100 and d_r <= dlim_r and eta_bx_r <= 100 else '✗'
    A_rhs = bw*h_r - (bw-2*t)*(h_r-2*t)
    m_kgm_rhs = A_rhs * 7.85e-3
    print(f"  {nm:<30} {W_r/1e3:>8.1f} {MRd_r:>8.2f}kNm {eta_r:>6.1f}% {eta_bx_r:>7.1f}%  {m_kgm_rhs:>5.1f}  {d_r:>5.1f}/{dlim_r:.0f}mm  {'✓' if eta_r<=100 and eta_bx_r<=100 else ok_r}")
# Liimapuu neliöpoikkileikkaukset vaakapalkkina

print(f"\n  --- Sahatavara C24, 3 kpl vierekkäin (SC2, kmod=0.8, 144mm leveä) ---")
for (nm_3t, h_3t) in [("3×148×48", 148), ("3×173×48", 173), ("3×198×48", 198), ("3×223×48", 223)]:
    b_3t  = 3 * 48   # 144 mm
    W_3t  = b_3t * h_3t**2 / 6.0
    I_3t  = b_3t * h_3t**3 / 12.0
    MRd_3t = fm_d_C24 * W_3t / 1.0e6
    eta_3t = Md_outer_beam / MRd_3t * 100.0
    d_3t   = 5 * q_outer_sls * outer_span_mm**4 / (384 * E_C24 * I_3t)
    dlim_3t = outer_span_mm / 300.0
    # Heikko akseli: Wz = h*b²/6
    Wz_3t   = h_3t * b_3t**2 / 6.0
    MRd_h3t = fm_d_C24 * Wz_3t / 1.0e6
    eta_A_3t  = eta_3t + 0.7 * Md_outer_h_A / MRd_h3t * 100.0
    eta_B_3t  = Md_outer_v_B / MRd_3t * 100.0 + 0.7 * Md_outer_h_B / MRd_h3t * 100.0
    eta_bx_3t = max(eta_A_3t, eta_B_3t)
    ok_3t = '✓' if eta_3t <= 100 and d_3t <= dlim_3t and eta_bx_3t <= 100 else '✗'
    m_kgm_3t = b_3t * h_3t * rho_C24 / 1e6
    print(f"  {nm_3t:<30} {W_3t/1e3:>8.0f} {MRd_3t:>8.2f}kNm {eta_3t:>6.1f}% {eta_bx_3t:>7.1f}%  {m_kgm_3t:>5.1f}  {d_3t:>5.1f}/{dlim_3t:.0f}mm  {'✓' if eta_3t<=100 and eta_bx_3t<=100 else ok_3t}")
print(f"\n── LUMIKINOSTUMA – EN 1991-1-3 §6.3 ─────────────────────")
print(f"  Rakennuksen katto 12° seinänsuuntaisesti → h_seinä vaihtelee!")
print(f"  *** Rakennuksen katto 12° seinänsuuntaisesti → h_seinä vaihtelee ***")
print()
print(f"  {'Parametri':<30} {'Korkea pää':>12} {'Matala pää':>12}")
print(f"  {'-'*30} {'-'*12} {'-'*12}")
print(f"  {'h_seinä [m]':<30} {h_seinä_korkea:>12.2f} {h_seinä_matala:>12.2f}")
print(f"  {'b_panel [m]':<30} {b_terassi:>12.2f} {b_terassi:>12.2f}")
print(f"  {'ls_drift [m]':<30} {ls_korkea:>12.2f} {ls_matala:>12.2f}")
print(f"  {'μ2 (lask. {:.2f}/{:.2f})':<30} {mu2_korkea:>12.2f} {mu2_matala:>12.2f}".format(
      mu2_h_korkea, mu2_h_matala))
print(f"  {'s_drift [kN/m²]':<30} {s_drift_korkea:>12.2f} {s_drift_matala:>12.2f}")
print(f"  {'Δs (lisäkuorma) [kN/m²]':<30} {s_drift_korkea-s1:>12.2f} {s_drift_matala-s1:>12.2f}")
print()
print(f"  Konservatiivinen (h_max koko matkalla):")
print(f"  s_drift = {s_drift:.2f} kN/m²  vs tasainen {s1:.2f} kN/m²")
print()
print(f"  TARKENNETTU ANALYYSI (EC0 §2.1): h(x) vaihtelee lineaarisesti palkin matkalla")
print(f"  Seinän korkeus muuttuu 12° kaltevuuden myötä → paikallinen kinostuma pienenee")
print(f"  kohti matalaa päätä. Kattotuolien pistekuormat lasketaan yksitellen paikallisella h(x)-arvolla.")
print(f"  Md,max sijaitsee x = {x_Md_refined_max*1000:.0f}mm palkin alusta (h ≈ {h_at_Mmax:.2f}m)")
print(f"  q_eq (tarkennettu) = {q_beam_drift_refined_avg:.2f} kN/m  vs q_eq (konservat.) = {q_inner_drift:.2f} kN/m")
print(f"  Md (konservat.) = {Md_beam_drift:.1f} kNm  →  Md (tarkennettu) = {Md_beam_drift_refined:.1f} kNm  ({(1-Md_beam_drift_refined/Md_beam_drift)*100:.0f}% pienempi)")
print(f"  *** Kinostuma sijoittuu talon seinän viereen. ***")

print(f"\n── UUSI SEINÄ y={inner_y + pilari_leveys//2}mm  (kolmiolasi + laudoitus) ──────────────────")
print(f"  Uusi vaakapalkki muodostaa sisäseinän kohdalla y={inner_y + pilari_leveys//2}mm.")
print(f"  Rakennuksen katto viettää {slope_rakennus_deg:.1f}° → kolmio häviää matalassa päässä.")
print(f"  Laudoitus:   {b_kolmio_tot_mm:.0f}mm × {h_laudoitus_mm:.0f}mm (täysi leveys, 500mm räystäältä alaspäin)")
print(f"  Kolmiolasi:  {b_kolmio_lasi_mm:.0f}mm × {h_kolmio_lasi_mm:.0f}mm  → A={A_lasi:.3f}m²")
print(f"    korkea pää (x={terrace_width:.0f}mm): aukko katon/palkin mukaan {h_rakennus_korkea:.0f}-{h_laudoitus_mm:.0f}-{h_katto_inner:.0f} = {gable_gap_h_mm:.0f}mm")
print(f"    lasipinnan korkeus geometriasta: {h_kolmio_lasi_mm:.0f}mm")
print(f"    matala pää (x=x₀):  h = 0  @  x₀={b_kolmio_lasi_mm:.0f}mm")
print(f"  Tuulipaine seinälle: cp,e = {cp_end_wall:.1f}, qp = {qp_z:.3f} kN/m²")
print(f"  Tuulivoima lasille: F = {F_wind_triangle:.2f} kN  (→ uusi palkki)")
print(f"  → Ei vaikuta kattotuolien taivutukseen, mutta")
print(f"     kantavien palkkien liitokset ja pilarijäykistys tulee tarkistaa.")

print(f"\n── PÄÄTYPALKKI – KATTOTUOLIREAKTIOT + KINOSTUMA + KOLMIOLASI ─")
print(f"  Jänneväli: {L_paaty_mm:.0f}mm (pilari-pilari)")
print()
print(f"  Kuormat:")
print(f"    Kattotuolireaktiot (normaali lumi):  q_eq = {q_rafter_beam:.2f} kN/m")
print(f"    Kattotuolireaktiot (kinostuma):      q_eq = {q_inner_drift:.2f} kN/m  (drift {s_drift_inner:.2f} kN/m² @ {y_inner_drift:.2f}m seinästä)")
print(f"    Lasi omapaino (ULS, triangul.):      Md = {Md_lasi_vert:.2f} kNm → uusi palkki")
print(f"    Laudoitus omapaino (ULS, triangul.): Md = {Md_laud_vert:.2f} kNm → 2×KP350 (ei tähän)")
print(f"    Tuuli lasi (vaaka, triangul.):       Md = {Md_lasi_horiz:.2f} kNm vaaka → uusi palkki")
print(f"    Tuuli laudoitus → 2×KP350 (ei tähän)")
print()
print(f"  Mitoitusmomentit:")
print(f"    Pysty Md (normaali):              {Md_beam_normal:.1f} kNm")
print(f"    Pysty Md (kinostuma konservat.):   {Md_beam_drift:.1f} kNm  (h_max koko matkalla)")
print(f"    Pysty Md (kinostuma tarkennettu):  {Md_beam_drift_refined:.1f} kNm  (h(x) vaihteleva, EC0 §2.1)")
print(f"    Vaaka Md (tuuli):                  {Md_beam_horiz:.1f} kNm")
print(f"  → Hallitseva: {gov_beam_case},  Md_gov = {Md_beam_gov:.1f} kNm")
print(f"  (h_seinä_korkea={h_seinä_korkea:.2f}m, h_seinä_matala={h_seinä_matala:.2f}m)")
print()

# IPE-profiilit (taivutus pystyakselilla, teräs S235)
ipe_options = [
    # name,   W [cm³],  I [cm⁴],  m [kg/m]
    ("IPE100",   34.2,  171.0,   8.1),
    ("IPE120",   53.0,  318.0,  10.4),
    ("IPE140",   77.3,  541.0,  12.9),
    ("IPE160",  123.0,  869.3,  15.8),
    ("IPE180",  146.3, 1317.0,  18.8),
    ("IPE200",  194.3, 1943.0,  22.4),
    ("IPE220",  252.0, 2770.0,  26.2),
    ("IPE240",  324.3, 3892.0,  30.7),
]
hea_options = [
    # (nimi, W_el,y [cm³], I_y [cm⁴], m [kg/m]) – arvot ArcelorMittal EN 10365
    ("HEA120",  101.0,  606.2,  19.9),
    ("HEA140",  155.4, 1033.0,  24.7),
    ("HEA160",  220.1, 1673.0,  30.4),
    ("HEA180",  294.1, 2510.0,  35.5),
    ("HEA200",  388.6, 3692.0,  42.3),
]

# Kiepahdusominaisuudet IPE/HEA (EN 1993-1-1 §6.3.2)
# {nimi: (Iz [cm⁴], It [cm⁴], Iw [cm⁶], W_pl,y [cm³])}
ltb_props = {
    "IPE100": (15.92, 1.20,   124,  39.41),
    "IPE120": (27.67, 1.74,   348,  60.73),
    "IPE140": (44.92, 2.45,   756,  91.04),
    "IPE160": (68.3,  3.60,  3960, 124.0),
    "IPE180": (101.0, 4.79,  7430, 166.4),
    "IPE200": (142.0, 6.98, 12990, 220.6),
    "IPE220": (205.0, 9.07, 22670, 285.4),
    "IPE240": (284.0, 12.90, 37390, 366.6),
    "HEA120": (231.0, 5.99,  9410, 106.3),
    "HEA140": (389.0, 8.13, 20030, 155.4),
    "HEA160": (616.0, 12.2, 39270, 220.1),
    "HEA180": (924.0, 14.8, 60200, 294.1),
    "HEA200": (1336.0, 20.8, 108000, 388.6),
}

def calc_chi_LT(profile_name, L_cr_mm, fy_):
    """Kiepahdusreduktiokerroin χ_LT, EN 1993-1-1 §6.3.2.2."""
    if profile_name not in ltb_props:
        return 1.0, 0.0, 999.0
    Iz_c, It_c, Iw_c, Wpl_c = ltb_props[profile_name]
    E_ = 210000.0; G_ = 81000.0
    Mcr = (math.pi / L_cr_mm) * math.sqrt(
        E_ * Iz_c * 1e4 * (G_ * It_c * 1e4 + math.pi**2 * E_ * Iw_c * 1e6 / L_cr_mm**2))
    lam_LT = math.sqrt(fy_ * Wpl_c * 1e3 / Mcr)
    alpha_LT = 0.34  # nurjahduskäyrä b (valssattu I-profiili)
    phi = 0.5 * (1 + alpha_LT * (lam_LT - 0.2) + lam_LT**2)
    chi = min(1.0, 1.0 / (phi + math.sqrt(max(0, phi**2 - lam_LT**2))))
    Mb_Rd = chi * fy_ * Wpl_c * 1e3 / 1e6  # kNm
    return chi, lam_LT, Mb_Rd

print(f"  Pystysuuntainen taivutus (Md={Md_beam_gov:.1f} kNm) + vaakataivutus (Md={Md_beam_horiz:.1f} kNm)")
print(f"  Vuorovaikutusehto: My/MRd_y + Mz/MRd_z ≤ 1.0  (EN 1993-1-1 §6.2.9.1)")
print()

# W_el,z (heikko akseli) IPE ja HEA – ArcelorMittal EN 10365
Wz_end_beam = {
    "IPE100":  5.79, "IPE120":  8.65, "IPE140": 12.31, "IPE160": 16.66,
    "IPE180": 24.3,  "IPE200": 28.5,  "IPE220": 37.3,  "IPE240": 47.3,
    "HEA120": 38.5,  "HEA140": 55.6,  "HEA160": 77.0,
    "HEA180": 102.7, "HEA200": 133.6,
}

for span_label, L_eff in [
    (f"6700mm (ei tukia)", L_paaty_m),
    (f"3350mm (1 välituki katolta)", L_paaty_m/2),
    (f"2233mm (2 välitukea katolta)", L_paaty_m/3),
]:
    Md_v = q_beam_drift * L_eff**2/8 + 0.0642*q_lasi_max*L_eff**2
    Md_v_norm = q_rafter_beam * L_eff**2/8
    Md_v = max(Md_v, Md_v_norm)
    Md_h = 0.0642 * q_wind_max * L_eff**2
    print(f"  ── {span_label}  Md_v={Md_v:.1f} kNm, Md_h={Md_h:.2f} kNm ─")
    L_cr_ltb = rafter_spacing * 1000  # mm, sivutuki kattotuoleilta
    print(f"  Kiepahdus: L_cr = {L_cr_ltb:.0f}mm (sivutuki kattotuoleilta, kiinnitys osoitettava)")
    print(f"  {'Profiili':<16} {'Teräs':>6} {'W_crit':>8} {'MRd':>9} {'η_v':>6} {'η_int':>7}  {'kg/m':>6}  δ/lim  OK?")
    print(f"  {'-'*16} {'-'*6} {'-'*8} {'-'*9} {'-'*6} {'-'*7}  {'-'*6}  {'-'*7} {'-'*4}")
    # IPE/HEA S235 – kiepahdus huomioitu (L_cr = kattotuoliväli)
    for nm, W_cm3, I_cm4, m_kgm in ipe_options + hea_options:
        MRd_sec = fy_S235 * W_cm3 * 1e3 / 1e6
        chi_LT, lam_LT, MbRd = calc_chi_LT(nm, L_cr_ltb, fy_S235)
        MRd = min(MRd_sec, MbRd)
        # Heikko akseli: käytä Wz-sanakirjaa (EN 1993-1-1 §6.2.9.1)
        Wz_cm3 = Wz_end_beam.get(nm, W_cm3 * 0.15)
        MRd_z = fy_S235 * Wz_cm3 * 1e3 / 1e6
        eta_v = Md_v / MRd * 100
        eta_i = eta_v + Md_h / MRd_z * 100   # korjattu: MRd_z heikkoa akselia varten
        ok = '✓' if eta_i <= 100 else '✗'
        EI = 210000.0 * I_cm4 * 1e4
        delta = 5 * (q_rafter_beam/gammaQ) * (L_eff*1000)**4 / (384 * EI)
        dlim = L_eff*1000/300
        d_ok = '✓' if delta <= dlim else '✗'
        if eta_i <= 100:
            print(f"  {nm:<16} {'S235':>6} {W_cm3:>8.1f} {MRd:>9.1f} {eta_v:>5.0f}% {eta_i:>6.0f}%  {m_kgm:>5.1f}  {delta:.1f}/{dlim:.0f}mm {d_ok}")
    # Epätasaiset L-profiilit S355J2
    for nm, al, ash, t_ in unequal_l_profiles_S355:
        A_l, yb, Ix_, _, _, W_c, Iz_, Wz_c = unequal_angle_props(al, ash, t_)
        MRd   = fy_S355 * W_c   / 1e6
        MRd_z = fy_S355 * Wz_c  / 1e6
        eta_v = Md_v / MRd * 100
        eta_i = eta_v + Md_h / MRd_z * 100  # EN 1993-1-1 §6.2.9, oikea heikko akseli
        EI = 210000.0 * Ix_
        delta = 5 * (q_rafter_beam/gammaQ) * (L_eff*1000)**4 / (384 * EI)
        dlim = L_eff*1000/300
        d_ok = '✓' if delta <= dlim else '✗'
        m_kgm_l = A_l * 7.85e-3  # mm² × 7850 kg/m³ / 1e6 → kg/m
        if eta_i <= 100:
            print(f"  {nm:<16} {'S355J2':>6} {W_c/1e3:>8.1f} {MRd:>9.1f} {eta_v:>5.0f}% {eta_i:>6.0f}%  {m_kgm_l:>5.1f}  {delta:.1f}/{dlim:.0f}mm {d_ok}")
    # Liimapuu GL30c (SC2, kmod=0.8)
    for (nm_lp, b_lp, h_lp, fm_k_lp, fv_k_lp, E_lp, gM_lp, kmod_lp) in lp_options:
        fm_d_lp  = kmod_lp * fm_k_lp / gM_lp
        W_lp     = b_lp * h_lp**2 / 6.0    # vahva akseli (pysty)
        W_z_lp   = h_lp * b_lp**2 / 6.0    # heikko akseli (vaaka)
        I_lp     = b_lp * h_lp**3 / 12.0
        MRd_lp   = fm_d_lp * W_lp   / 1e6
        MRd_z_lp = fm_d_lp * W_z_lp / 1e6
        eta_v = Md_v / MRd_lp * 100
        eta_i = (Md_v / MRd_lp + 0.7 * Md_h / MRd_z_lp) * 100  # EN 1995-1-1 §6.1.6, km=0.7
        EI_lp = E_lp * I_lp
        delta = 5 * (q_rafter_beam/gammaQ) * (L_eff*1000)**4 / (384 * EI_lp)
        dlim = L_eff*1000/300
        d_ok = '✓' if delta <= dlim else '✗'
        m_kgm_lp = b_lp * h_lp * 500 / 1e6  # GL30c ~500 kg/m³
        if eta_i <= 100:
            print(f"  {nm_lp:<16} {'GL30c':>6} {W_lp/1e3:>8.0f} {MRd_lp:>9.1f} {eta_v:>5.0f}% {eta_i:>6.0f}%  {m_kgm_lp:>5.1f}  {delta:.1f}/{dlim:.0f}mm {d_ok}")

    # Sahatavara C24, 3 kpl vierekkäin (SC2, kmod=0.8) – biaksinen taivutus EN 1995-1-1
    # km=0.7 suorakaiteen biaksinen tarkistus: σy/fm + km×σz/fm ≤ 1.0
    for (nm_3t, h_3t) in [("3×148×48", 148), ("3×173×48", 173), ("3×198×48", 198), ("3×223×48", 223)]:
        b_3t    = 3 * 48           # 144 mm (leveys)
        W_v_3t  = b_3t * h_3t**2 / 6.0   # vahva akseli (pysty)
        W_h_3t  = h_3t * b_3t**2 / 6.0   # heikko akseli (vaaka)
        I_3t    = b_3t * h_3t**3 / 12.0
        MRd_v_3t = fm_d_C24 * W_v_3t / 1.0e6
        MRd_h_3t = fm_d_C24 * W_h_3t / 1.0e6
        eta_v   = Md_v / MRd_v_3t * 100
        eta_i   = (Md_v / MRd_v_3t + 0.7 * Md_h / MRd_h_3t) * 100  # EN 1995-1-1 biaksinen
        EI_3t   = E_C24 * I_3t
        delta   = 5 * (q_rafter_beam / gammaQ) * (L_eff * 1000)**4 / (384 * EI_3t)
        dlim    = L_eff * 1000 / 300
        d_ok    = '✓' if delta <= dlim else '✗'
        m_kgm_3t = b_3t * h_3t * rho_C24 / 1e6   # kg/m
        if eta_i <= 100:
            print(f"  {nm_3t:<16} {'C24/SC2':>6} {W_v_3t/1e3:>8.0f} {MRd_v_3t:>9.1f} {eta_v:>5.0f}% {eta_i:>6.0f}%  {m_kgm_3t:>5.1f}  {delta:.1f}/{dlim:.0f}mm {d_ok}")
    print()

print(f"\n── KIEPAHDUS – PÄÄTYPALKKI (EN 1993-1-1 §6.3.2) ─────────────")
print(f"  Yllä olevissa taulukoissa MRd sisältää kiepahdusreduktion χ_LT")
print(f"  (sivutuki kattotuoleilta, L_cr = {rafter_spacing*1000:.0f}mm).")
print(f"  Vertailu: Md = {Md_beam_gov:.1f} kNm (hallitseva, kinostuma)")
print()
print(f"  {'Profiili':<10}  {'Ei sivutukea (L_cr=6700mm)':>28}  {'Sivutuella (L_cr=1134mm)':>26}")
print(f"  {'':10}  {'χ_LT':>5} {'Mb,Rd':>9} {'η':>5}       {'χ_LT':>5} {'Mb,Rd':>9} {'η':>5}")
print(f"  {'-'*10}  {'-'*5} {'-'*9} {'-'*5} {'-'*3}   {'-'*5} {'-'*9} {'-'*5} {'-'*3}")
for nm, W_cm3, I_cm4, m_kgm in ipe_options + hea_options:
    chi0, _, MbRd0 = calc_chi_LT(nm, L_paaty_mm, fy_S235)
    chi1, _, MbRd1 = calc_chi_LT(nm, rafter_spacing*1000, fy_S235)
    eta0 = Md_beam_gov / MbRd0 * 100 if MbRd0 > 0 else 9999
    eta1 = Md_beam_gov / MbRd1 * 100 if MbRd1 > 0 else 9999
    ok0 = '✓' if eta0 <= 100 else '✗'
    ok1 = '✓' if eta1 <= 100 else '✗'
    print(f"  {nm:<10}  {chi0:>5.2f} {MbRd0:>8.1f}kNm {eta0:>4.0f}% {ok0}   {chi1:>5.2f} {MbRd1:>8.1f}kNm {eta1:>4.0f}% {ok1}")
print()
print(f"  *** HUOMIO: Ilman sivutukea IPE-profiilit eivät riitä kiepahduksen vuoksi!")
print(f"  Kattotuolien kiinnitys päätypalkin ylälaippaan tulee osoittaa")
print(f"  riittäväksi vaakavoiman siirtoon (EN 1993-1-1 §6.3.2). ***")

print(f"\n── PILARIKUORMAT (ULS) ────────────────────────────────────")
print(f"  Kaikki kattotuolireaktiot menevät uudelle päätypalkille.")
print(f"  Päätypalkki siirtää kuorman ulkopilareille (+ välituet 2×KP360×51:lle jos käytetty).")
print()
print(f"  Ulkopilari (uusi, päätypalkki päällä):")
print(f"    Päätypilarit (vas/oik):    {R_outer_per_pillar_left:.1f} / {R_outer_per_pillar_right:.1f} kN  (ULS, ulkoreunap.)")
print(f"    Keskipilari (1 kpl):       {R_outer_per_pillar_middle:.1f} kN  (ULS, ulkoreunap.)")
print(f"    (Hallitseva: keskipilari {R_outer_per_pillar_middle:.1f} kN)")
print(f"  Pilarit: olemassa olevat betonipilaarit 250×250mm")

# ── Rakennuksen rakenteille siirtyvät kuormat ─────────────────
# Ominaisarvot päätypalkille (toiseen laskuriin)
q_inner_char = sum(load_kN for _, load_kN in inner_rafter_point_loads_char) / roof_loaded_width_m

# Päätypalkki tukireaktiot (täysi jänne 6700mm)
# Hallitseva kuorma valitaan: normaali tai tarkennettu kinostuma
if Md_beam_normal >= Md_beam_drift_refined:
    q_beam_gov_uls = q_beam_normal
    q_beam_gov_char = q_inner_char
    gov_inner_point_loads_uls = inner_rafter_point_loads_normal
    gov_inner_point_loads_char = inner_rafter_point_loads_char
    gov_inner_beam_reactions_uls = inner_beam_reactions_normal
    gov_inner_beam_reactions_char = inner_beam_reactions_normal_char
else:
    q_beam_gov_uls = q_beam_drift_refined_avg
    q_beam_gov_char = q_beam_drift_refined_avg_char
    gov_inner_point_loads_uls = inner_rafter_point_loads_refined
    gov_inner_point_loads_char = inner_rafter_point_loads_refined_char
    gov_inner_beam_reactions_uls = inner_beam_reactions_refined
    gov_inner_beam_reactions_char = inner_beam_reactions_refined_char

R_paaty_v_rafter_left_u = gov_inner_beam_reactions_uls[inner_support_xs_mm[0]]
R_paaty_v_rafter_right_u = gov_inner_beam_reactions_uls[inner_support_xs_mm[-1]]
R_paaty_v_rafter_left_c = gov_inner_beam_reactions_char[inner_support_xs_mm[0]]
R_paaty_v_rafter_right_c = gov_inner_beam_reactions_char[inner_support_xs_mm[-1]]

# Kolmiolasi triangulaarinen: max korkea päässä → R_korkea = q_max*L/3, R_matala = q_max*L/6
R_paaty_glass_max  = q_lasi_max * L_paaty_m / 3.0                # kN ULS (korkea pää)
R_paaty_glass_min  = q_lasi_max * L_paaty_m / 6.0                # kN ULS (matala pää)
q_lasi_max_char    = gk_lasi * h_kolmio_max
R_paaty_glass_max_char = q_lasi_max_char * L_paaty_m / 3.0
# Vaakavoima ominais
F_wind_triangle_char = (cp_end_wall * qp_z) * A_lasi  # kN (ilman gammaQ)
R_paaty_h_char     = F_wind_triangle_char / 2.0        # kN per tuki ominais

print(f"\n── KUORMAT RAKENNUKSEN RAKENTEILLE (syöttöarvot) ──────────")
print(f"  Kuormitustapaus: ULS (γG={gammaG}, γQ={gammaQ})  |  Ominais (Gk, Qk)")
print(f"  Kaikki kattotuolireaktiot → uusi päätypalkki → ulkopilarit.")
print(f"  2×KP360×51 saa pistekuorman VAIN jos päätypalkki tuetaan siihen välituella.")
print()
print(f"  Päätypalkki: q_eq={q_beam_gov_char:.2f} kN/m ominais / {q_beam_gov_uls:.2f} kN/m ULS ({gov_beam_case})")
print(f"  Lasi (triangul.): q_max={q_lasi_max_char:.2f} kN/m ominais / {q_lasi_max:.2f} kN/m ULS")
print()
# Lasi-osuudet (triangulaarinen, approksimaatio)
R_lasi_korkea_c = q_lasi_max_char * L_paaty_m / 3.0
R_lasi_matala_c = q_lasi_max_char * L_paaty_m / 6.0
R_lasi_korkea_u = q_lasi_max * L_paaty_m / 3.0
R_lasi_matala_u = q_lasi_max * L_paaty_m / 6.0

for case_label, support_case_xs_mm in [
    ("Ei välitukia (jänne 6700mm)", list(inner_support_xs_mm)),
    ("1 välituki = 2×KP360×51 (jänne 3350mm)", [inner_support_xs_mm[0], inner_support_xs_mm[0] + L_paaty_mm / 2.0, inner_support_xs_mm[-1]]),
    ("2 välitukea = 2×KP360×51 (jänne 2233mm)", [inner_support_xs_mm[0], inner_support_xs_mm[0] + L_paaty_mm / 3.0, inner_support_xs_mm[0] + 2.0 * L_paaty_mm / 3.0, inner_support_xs_mm[-1]]),
]:
    if support_case_xs_mm == list(inner_support_xs_mm):
        case_reactions_u = gov_inner_beam_reactions_uls
        case_reactions_c = gov_inner_beam_reactions_char
    else:
        case_reactions_u = beam_support_reactions(
            inner_beam_start_x_mm, inner_beam_end_x_mm, support_case_xs_mm, gov_inner_point_loads_uls)
        case_reactions_c = beam_support_reactions(
            inner_beam_start_x_mm, inner_beam_end_x_mm, support_case_xs_mm, gov_inner_point_loads_char)

    R_ulkopilari_left_u = case_reactions_u[support_case_xs_mm[0]]
    R_ulkopilari_right_u = case_reactions_u[support_case_xs_mm[-1]]
    R_ulkopilari_left_c = case_reactions_c[support_case_xs_mm[0]]
    R_ulkopilari_right_c = case_reactions_c[support_case_xs_mm[-1]]
    kp360_reactions_u = [case_reactions_u[x_mm] for x_mm in support_case_xs_mm[1:-1]]
    kp360_reactions_c = [case_reactions_c[x_mm] for x_mm in support_case_xs_mm[1:-1]]

    print(f"   ── {case_label}")
    print(f"      Ulkopilari (korkea pää): {R_ulkopilari_right_c + R_lasi_korkea_c:.1f} kN ominais  / {R_ulkopilari_right_u + R_lasi_korkea_u:.1f} kN ULS")
    print(f"      Ulkopilari (matala pää): {R_ulkopilari_left_c + R_lasi_matala_c:.1f} kN ominais  / {R_ulkopilari_left_u + R_lasi_matala_u:.1f} kN ULS")
    if kp360_reactions_u:
        avg_kp360_c = sum(kp360_reactions_c) / len(kp360_reactions_c)
        avg_kp360_u = sum(kp360_reactions_u) / len(kp360_reactions_u)
        print(f"      2×KP360×51 pistekuorma:  {avg_kp360_c:.1f} kN ominais  / {avg_kp360_u:.1f} kN ULS  (× {len(kp360_reactions_u)} pistettä)")
    else:
        print(f"      2×KP360×51:              0 kN pystykuormaa  (ei välitukea)")
    print()

print(f"  Vaakavoima (tuuli, päätykolmio → rakennuksen jäykistys):")
print(f"     Kokonaisvoima:           {F_wind_triangle_char:.1f} kN (ominais)  /  {gammaQ * F_wind_triangle_char:.1f} kN (ULS)")
print(f"     Per tukipiste (~puolet): {R_paaty_h_char:.1f} kN (ominais)  /  {gammaQ * R_paaty_h_char:.1f} kN (ULS)")
print(f"     (cp,e={cp_end_wall}, qp={qp_z:.3f} kN/m², A_lasi={A_lasi:.2f} m²)")
print()

print(f"\n── AURINKOPANEELIT – KINOSTUMAKUORMA (EN 1991-1-3 §6.3) ──────")
print(f"  Kinostuma alkaa UUDESTA SEINÄSTÄ y=1800mm ja leviää ulospäin.")
print(f"  Huippu on paneelien sisäreunassa (y=1800mm), ei talon seinällä.")
print()
# Huippuarvo korkean pään mukaan (konservatiivinen)
# h_seinä tarkka y=1800mm kohdalla:
h_katto_at_y1800 = (h_katto_inner - (h_katto_inner - h_katto_outer)
                    * (inner_y + pilari_leveys/2) / (inner_y + terrace_depth))  # mm
h_seinä_y1800 = (h_rakennus_korkea - h_laudoitus_mm - h_katto_at_y1800) / 1000.0  # m
ls_panel, mu2_panel, mu2h_panel, _, s_peak_panel = laske_kinostuma(
    h_seinä_korkea, b_terassi, sk, mu1_=mu1)   # konservatiivinen (h_max)
ls_panel_exact, mu2_panel_exact, _, _, s_peak_panel_exact = laske_kinostuma(
    h_seinä_y1800, b_terassi, sk, mu1_=mu1)    # tarkka y=1800mm kohdalla

panel_snow_cap = 5.40  # kN/m² (Longi Hi-MO X10 LR7, etupuoli)
print(f"  h_seinä konservatiivinen (korkea pää, y=0):  {h_seinä_korkea:.2f}m → s_peak={s_peak_panel:.2f} kN/m²")
print(f"  h_seinä tarkka (y=1800mm):                   {h_seinä_y1800:.2f}m → s_peak={s_peak_panel_exact:.2f} kN/m²")
print(f"  Käytetään tarkkaa: s_peak = {s_peak_panel_exact:.2f} kN/m²,  ls = {ls_panel_exact:.2f}m")
print(f"  (Konservatiivisella h={h_seinä_korkea:.2f}m: s_peak={s_peak_panel:.2f} kN/m², ULS={gammaQ*s_peak_panel:.2f} kN/m²)")
print(f"  Paneelin kapasiteetti: {panel_snow_cap:.2f} kN/m²")
print()
print(f"  {'Etäisyys seinästä':<30} {'s [kN/m²]':>10} {'ULS=1.5s':>10} {'η %':>8}  OK?")
print(f"  {'-'*60}")
check_points = [
    (0,    "y=1800mm (paneelin sisäreuna)"),
    (200,  "y=2000mm"),
    (995,  f"y={1800+995}mm (1. paneelin puoliväli)"),
    (1990, f"y={1800+1990}mm (1. paneelirivi ulkoreuna)"),
    (2000, f"y={1800+2000}mm (2. paneelirivin alku)"),
    (3985, f"y={1800+3985}mm (2. paneelirivi ulkoreuna)"),
]
panel_ok = True
for dy_mm, label in check_points:
    dy_m = dy_mm / 1000.0
    # Kinostuma alkaa y=1800mm (uusi sisäseinä) ja leviää ulospäin → dy mitataan seinästä
    s_loc = max(s1, s_peak_panel_exact * (1.0 - dy_m / ls_panel_exact)) if dy_m < ls_panel_exact else s1
    uls_loc = gammaQ * s_loc
    eta_loc = uls_loc / panel_snow_cap * 100.0
    ok_str = "✓" if uls_loc <= panel_snow_cap else "✗ YLITTYY"
    if uls_loc > panel_snow_cap:
        panel_ok = False
    print(f"  {label:<30} {s_loc:>10.2f} {uls_loc:>10.2f} {eta_loc:>7.0f}%  {ok_str}")
print()
uls_max_panel = gammaQ * s_peak_panel_exact
uls_max_panel_cons = gammaQ * s_peak_panel
if panel_ok:
    print(f"  ✓ Kaikki pisteet OK. Tarkka arvo sisäreunan ULS={uls_max_panel:.2f}/{panel_snow_cap:.2f} kN/m² η={uls_max_panel/panel_snow_cap*100:.0f}%")
    if uls_max_panel_cons > panel_snow_cap:
        print(f"  *** HUOM: Konservatiivisella h={h_seinä_korkea:.2f}m: ULS={uls_max_panel_cons:.2f} kN/m² ({uls_max_panel_cons/panel_snow_cap*100:.0f}%) ylittyisi,")
        print(f"      mutta tarkka h={h_seinä_y1800:.2f}m @ y=1800mm on mitoittava.")
else:
    print(f"  ✗ Kapasiteetti ylittyy – paneelit tarvitsevat lisätukea tai kinostuma on rajattava!")

qmin_rafter = 0.9 * gk_line + gammaQ * qk_w_up
R_uplift_rafter = qmin_rafter * rafter_span_m / 2.0
print(f"  0.9·Gk + 1.5·Wk,ylös (kattotuoli) = {qmin_rafter:.3f} kN/m")
if R_uplift_rafter < 0:
    print(f"  *** NOSTO: {abs(R_uplift_rafter):.2f} kN per tukipiste – kiinnitys tarvitaan!")
else:
    print(f"  Tukireaktio {R_uplift_rafter:.2f} kN (puristus) – OK ✓")

print()
print(dw)
print("  YHTEENVETO – SUOSITUKSET")
print(dw)
ok_lp     = [r for r in results_lp     if r['ok_M'] and r['ok_V'] and r['ok_delta']]
ok_steel  = [r for r in results_steel  if r['ok_M'] and r['ok_V'] and r['ok_delta']]
ok_timber = [r for r in results_timber if r['ok_M'] and r['ok_V'] and r['ok_delta']]
print()
print(f"  Mitoitusmomentti kattotuolille:  {Md_rafter_gov:.2f} kNm  ({gov_case})")
print(f"  Ulkoreun. palkki Md:             {Md_outer_beam:.2f} kNm  (maks.abs., jatkuva palkki)")
print()
if ok_lp:
    r = ok_lp[0]
    print(f"  LIIMAPUU GL30c (SC2):   pienin → {r['name']}  η={r['eta_M']:.0f}%  δ={r['delta']:.1f}mm")
if ok_steel:
    r = ok_steel[0]
    print(f"  TERÄS S235:             pienin → {r['name']}  η={r['eta_M']:.0f}%  δ={r['delta']:.1f}mm")
if ok_timber:
    r = ok_timber[0]
    hb_warn = f"  (h/b={r['hb']:.1f} → tarkista sivutuki!)" if r['hb'] > 5 else ""
    print(f"  SAHATAVARA C24 (SC2):   pienin → {r['name']}  η={r['eta_M']:.0f}%  δ={r['delta']:.1f}mm{hb_warn}")
print()
print("  HUOMIOT:")
print(f"  * Lumikinostuma: s_drift={s_drift:.2f} kN/m² vs tasainen {s1:.2f} kN/m²  (h_korkea={h_seinä_korkea:.2f}m, h_matala={h_seinä_matala:.2f}m)")
print("  * Liimapuu GL30c: SC2 (kmod=0.8), kattolasitus ja sivulasitus suojaavat rakenteen.")
print("  * Sahatavara C24 SC2: kmod=0.8. Kaikki puuprofiilit SC2 – kattolasitus suojaa.")
print("  * 48mm leveillä kertopuilla h/b > 5 → sivutuki paneelilevyistä tarkistettava.")
print("  * Päätykolmion tuulivoima vaatii rakenteiden jäykistyssuunnitelman.")
print("  * Paneelikuorma tasaisena, ei dynaamisia/seismisiä kuormia.")
print(dw)
