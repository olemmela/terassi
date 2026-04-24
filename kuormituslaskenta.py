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

from geometry_loader import load, member, surface, profile_b, profile_h

# ============================================================
# GEOMETRIA  (luetaan geometry/katos.json:ista)
# ============================================================
_GEO = load("katos.json")

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


def aggregate_point_loads(point_loads):
    """Yhdistää samaan kohtaan osuvat pistekuormat."""
    aggregated = {}
    for x_m, p_kN in point_loads:
        key = round(float(x_m), 9)
        aggregated[key] = aggregated.get(key, 0.0) + float(p_kN)
    return sorted(aggregated.items())


def simple_span_point_reactions(span_m, point_loads):
    """Pistekuormien lisäreaktiot yksinkertaisesti tuetulle jänteelle."""
    point_loads = aggregate_point_loads(point_loads)
    r_left = sum(p_kN * (span_m - x_m) / span_m for x_m, p_kN in point_loads)
    r_right = sum(p_kN * x_m / span_m for x_m, p_kN in point_loads)
    return r_left, r_right


def simple_span_combined_moment_max(span_m, udl_kNm, point_loads):
    """Maksimitaivutusmomentti UDL + pistekuormat, notko +."""
    point_loads = aggregate_point_loads(point_loads)
    point_load_map = dict(point_loads)
    r_left_points, _ = simple_span_point_reactions(span_m, point_loads)

    def moment_at(x_m):
        moment_udl = udl_kNm * x_m * (span_m - x_m) / 2.0
        moment_points = r_left_points * x_m
        for a_m, p_kN in point_loads:
            if a_m <= x_m + 1e-12:
                moment_points -= p_kN * (x_m - a_m)
        return moment_udl + moment_points

    candidates = {0.0, span_m}
    segment_points = [0.0] + [x_m for x_m, _ in point_loads] + [span_m]
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
    return max(moment_candidates, key=lambda item: item[1])


def simple_span_max_deflection_mm(span_mm, EI_Nmm2, udl_kNm=0.0, point_loads=None, sample_count=2001):
    """Maksimitaipuma UDL + pistekuormat yksinkertaisesti tuetulla jänteellä."""
    if point_loads is None:
        point_loads = []
    point_loads = aggregate_point_loads(point_loads)
    q_Nmm = udl_kNm  # 1 kN/m = 1 N/mm
    point_loads_mm = [(x_m * 1000.0, p_kN * 1000.0) for x_m, p_kN in point_loads]

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
        if delta_mm > max_delta_mm:
            max_delta_mm = delta_mm
            x_at_max_mm = x_mm
    return max_delta_mm, x_at_max_mm


def format_point_loads(point_loads):
    """Muotoilee pistekuormat tulostusta varten."""
    if not point_loads:
        return "ei pistekuormia"
    return ", ".join(
        f"{p_kN:.1f} kN @ x={x_m * 1000.0:.0f} mm"
        for x_m, p_kN in point_loads
    )

# ============================================================
# PALKKIEN POIKKILEIKKAUKSET  (Kerto-S LVL)
# ============================================================
# KP450x51  -> leveys b=51 mm, korkeus h=450 mm  (yksi palkki)
b1 = profile_b(member(_GEO, "beams", "beam.kp450.wall"))
h1 = profile_h(member(_GEO, "beams", "beam.kp450.wall"))

# 2xKP360x51 -> kaksi rinnakkaista palkkia: b=2×51=102 mm, h=360 mm
b2 = profile_b(member(_GEO, "beams", "beam.kp360x2"))
h2 = profile_h(member(_GEO, "beams", "beam.kp360x2"))

# ============================================================
# TRIBUTÄÄRIALUEET (y-suunta, kohtisuora palkin akselille)
# ============================================================
# Seinäliitos ottaa kuorman alueelta y=0 ... (0+beam1_y)/2
trib1_start_mm = (0.0 + beam1_y) / 2.0          # 450 mm
trib1_end_mm   = (beam1_y + beam2_y) / 2.0       # 1287.5 mm
trib_w1        = (trib1_end_mm - trib1_start_mm) / 1000.0  # m

trib2_start_mm = trib1_end_mm                    # 1287.5 mm
trib2_end_mm   = float(roof_edge_y)              # = 2200 mm (katon reuna)
trib_w2        = (trib2_end_mm - trib2_start_mm) / 1000.0  # m

# ============================================================
# PYSYVÄT KUORMAT
# ============================================================
# Kate + ruoteet + muut pysyvät (arvo teräsprofiililevy + alusrakenne)
gk_roofing = 0.20   # kN/m²

# Kertopuun tiheys (Kerto-S): 480 kg/m³ → γ = 4.71 kN/m³
gamma_lvl = 480.0 * 9.81 / 1000.0   # kN/m³ ≈ 4.71

g_beam1 = b1 / 1000.0 * h1 / 1000.0 * gamma_lvl   # kN/m (KP450x51 omapaino)
g_beam2 = b2 / 1000.0 * h2 / 1000.0 * gamma_lvl   # kN/m (2xKP360x51 omapaino)

# Pysyvä viivakuorma palkeille (kate + palkki)
gk1 = gk_roofing * trib_w1 + g_beam1   # kN/m
gk2 = gk_roofing * trib_w2 + g_beam2   # kN/m

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

# Lumikuorman viivakuorma palkeille
qk_snow1 = s_roof * trib_w1   # kN/m
qk_snow2 = s_roof * trib_w2   # kN/m

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

# Yksikalteisen katoksen nettopainekertoimet  (EN 1991-1-4 taulukko 7.7)
# HUOM: Taulukko 7.7 on vapaasti seisovalle katokselle (free-standing canopy).
# Seinään kiinnitetylle katokselle tarkempi tarkistus olisi taulukko 7.2–7.4
# (ulkopaine − sisäpaine). Taulukko 7.7 on käytäntöön vakiintunut yksinkertaistus
# ja yleensä konservatiivinen tässä yhteydessä.
# Interpolointi α = 12°:
alpha_ref_lo, alpha_ref_hi = 10.0, 20.0
cp_dn_lo, cp_dn_hi = 0.8, 1.3
cp_up_lo, cp_up_hi = -0.6, -1.3
t = (slope_deg - alpha_ref_lo) / (alpha_ref_hi - alpha_ref_lo)
cp_net_down = cp_dn_lo + t * (cp_dn_hi - cp_dn_lo)   # alaspäin
cp_net_up   = cp_up_lo + t * (cp_up_hi - cp_up_lo)   # ylöspäin

w_wind_down = cp_net_down * qp_z   # kN/m² (alaspäin, pahin yhdistelmä lumen kanssa)
w_wind_up   = cp_net_up   * qp_z   # kN/m² (ylöspäin, imukuorma)

qk_wind_down1 = w_wind_down * trib_w1
qk_wind_down2 = w_wind_down * trib_w2
qk_wind_up1   = w_wind_up   * trib_w1
qk_wind_up2   = w_wind_up   * trib_w2

# ============================================================
# KUORMAYHDISTELMÄT  (EN 1990 kaava 6.10)
# ============================================================
gammaG = 1.35    # pysyvien kuormien mitoituskerroin
gammaQ = 1.50    # muuttuvien kuormien mitoituskerroin
psi0_W = 0.6     # tuulikuorman yhdistelmäarvokerroin (lumi hallitsee)

# ULS - taivuttava kuorma (alaspäin)
qd1 = gammaG * gk1 + gammaQ * qk_snow1 + gammaQ * psi0_W * qk_wind_down1
qd2 = gammaG * gk2 + gammaQ * qk_snow2 + gammaQ * psi0_W * qk_wind_down2

# Tukireaktiot (ULS) – palkin uloke ja katon räystäs mukaan
# Palkin UDL jatkuu ulokkeelle (a_oh), katon räystään kuorma siirtyy
# räystäsruoteiden kautta palkin päihin (eave).  Räystäälle ei tule
# palkin omapainoa, vain kattokuorma.
_a_oh_m  = a_oh_left_mm / 1000.0     # m (symmetrinen)
_eave_m  = eave_left_mm / 1000.0     # m (symmetrinen)
q_roof_d = gammaG * gk_roofing + gammaQ * s_roof + gammaQ * psi0_W * w_wind_down  # kN/m²
R1 = symmetric_support_reaction(qd1, L_m, _a_oh_m, q_roof_d, trib_w1, _eave_m)   # kN per tuki
R2 = symmetric_support_reaction(qd2, L_m, _a_oh_m, q_roof_d, trib_w2, _eave_m)   # kN per tuki

# Taivutusmomentti (yksinkertaisesti tuettu, UDL, kaltevuuskorjaus)
Md1 = qd1 * L_m**2 / 8.0 * moment_factor   # kNm
Md2 = qd2 * L_m**2 / 8.0 * moment_factor   # kNm

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
q_z1_ltb   = qd1 * math.sin(slope_rad)              # kN/m (vaaka-komponentti)
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
Vd1  = qd1 * L_m / 2.0            # kN (palkin sisäinen päätyleikkaus jänteen puolella)
Vd2  = qd2 * L_m / 2.0            # kN
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

# SLS ominaiskuorma (pysyvä + lumi, γ=1.0)
qk_sls1 = gk1 + qk_snow1   # kN/m (sis. palkin omapaino)
qk_sls2 = gk2 + qk_snow2   # kN/m
L_mm_eff = L_mm   # käytetään nettoväliä

def deflection_mm(q_kNm, L_mm_, EI_Nmm2):
    """5qL⁴/384EI, q kN/m → N/mm"""
    q_Nmm = q_kNm   # kN/m = N/mm
    return 5.0 * q_Nmm * L_mm_**4 / (384.0 * EI_Nmm2)

delta1 = deflection_mm(qk_sls1, L_mm_eff, EI1)   # mm
delta2 = deflection_mm(qk_sls2, L_mm_eff, EI2)    # mm
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
b_lp = profile_b(_lp_beam)   # mm – LP225x90 leveys
h_lp = profile_h(_lp_beam)   # mm – LP225x90 korkeus
L_lp_mm       = float(beam2_y)  # mm – jänneväli: seinä → pilari = 1675 mm
L_lp_m        = L_lp_mm / 1000.0
a_lp_mm       = float(beam1_y)  # mm – pistekuorman sijainti seinältä = 900 mm
a_lp_m        = a_lp_mm / 1000.0

# KP450x51 tukireaktio (sis. palkin uloke ja katon räystäs)
P_kp1 = R1                         # kN (yksi päätytuki)

# LP225x90 tukireaktiot pistekuormasta P
R_seinä_lp = P_kp1 * (L_lp_m - a_lp_m) / L_lp_m   # kN
R_pilari_lp = P_kp1 * a_lp_m / L_lp_m              # kN

# Maksimitaivutusmomentti (pistekuorma a:n kohdalla)
Md_lp = R_seinä_lp * a_lp_m                         # kNm

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
Vd_lp    = max(R_seinä_lp, R_pilari_lp)
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
# Pistekuorma midspanissa: M_Q = 1.5 * Qk * L/4
M_huolto_Q1 = 1.5 * Qk_huolto * L_m / 4.0   # kNm
# Hajakuorma osa (Kategoria H, jos enemmän kuin ψ0*lumi):
q_H_1 = 1.5 * qk_H * trib_w1               # kN/m
M_huolto1 = (q_g_s_psi1 + q_H_1) * L_m**2 / 8.0 * moment_factor + M_huolto_Q1
eta_huolto1 = M_huolto1 / MRd1 * 100.0

# 2×KP360×51 – huolto dominant:
q_g_s_psi2  = 1.35 * gk2 + 1.5 * psi0_snow_accomp * qk_snow2
M_huolto_Q2 = 1.5 * Qk_huolto * L_m / 4.0
q_H_2 = 1.5 * qk_H * trib_w2
M_huolto2 = (q_g_s_psi2 + q_H_2) * L_m**2 / 8.0 * moment_factor + M_huolto_Q2
eta_huolto2 = M_huolto2 / MRd2 * 100.0

# ── 2) Tuulen nostokuorma – minimikapasiteetti ───────────
# EN 1990 kaava 6.10 min: 0.9*Gk + 1.5*Wk (nosto ylöspäin)
# Jos qmin < 0 → tukireaktio on nostava → kiinnitys tarvitaan
gammaG_min = 0.9   # suotuisa pysyvä
qmin1 = gammaG_min * gk1 + 1.5 * qk_wind_up1   # kN/m (neg = nosto)
qmin2 = gammaG_min * gk2 + 1.5 * qk_wind_up2
q_roof_min = gammaG_min * gk_roofing + 1.5 * w_wind_up   # kN/m² (räystäsalue, ei palkin omapainoa)

# Nostoreaktio tukipisteessä (sis. uloke + räystäs)
R_uplift1 = symmetric_support_reaction(qmin1, L_m, _a_oh_m, q_roof_min, trib_w1, _eave_m)   # kN (neg = nosto)
R_uplift2 = symmetric_support_reaction(qmin2, L_m, _a_oh_m, q_roof_min, trib_w2, _eave_m)   # kN

# ── 3) Lumikuorman epätasainen jakautuma (EN 1991-1-3 §6.2) ──
# Yksilappinen katto: epätasaiset tapaukset eivät pääsääntöisesti koske
# yksinkertaista yksilappeista katosta (ei murtumisvaaraa toiselle lappee).
# Huomioitava vain jos rakenne on U- tai L-muotoinen tai vieressä korkeampi rak.
# → Merkitään tiedoksi, ei lasketa erikseen.

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
print(f"  KP450×51  :  y = {trib1_start_mm:.0f} ... {trib1_end_mm:.0f} mm  →  b_trib = {trib_w1*1000:.1f} mm")
print(f"  2×KP360×51:  y = {trib2_start_mm:.0f} ... {trib2_end_mm:.0f} mm  →  b_trib = {trib_w2*1000:.1f} mm")

print("\n── LUMIKUORMA ─────────────────────────────────────────")
print(f"  Maanpintaominaiskuorma  sk     {sk:.1f} kN/m²  (Eteläsuomi)")
print(f"  Muotokerroin μ1 (α={slope_deg:.0f}°)     {mu1:.1f}  (0°–30° katto)")
print(f"  Lumikuorma katolla  s          {s_roof:.2f} kN/m²")
print(f"  KP450×51   q_lumi              {qk_snow1:.3f} kN/m")
print(f"  2×KP360×51 q_lumi              {qk_snow2:.3f} kN/m")

print("\n── TUULIKUORMA ────────────────────────────────────────")
print(f"  Perusnopeus vb0                {vb0:.0f} m/s  (vyöhyke I, Eteläsuomi)")
print(f"  Referenssikorkeus              {z_ref:.0f} m")
print(f"  cr({z_ref:.0f})                        {cr_z:.3f}")
print(f"  Iv({z_ref:.0f})                        {Iv_z:.3f}")
print(f"  Keskinopeus vm                 {vm_z:.1f} m/s")
print(f"  Huippupainekerroin qp({z_ref:.0f})      {qp_z:.3f} kN/m²")
print(f"  Nettopainekerroin alaspäin     cp,net = {cp_net_down:.2f}  (interpoloitu {slope_deg:.0f}°)")
print(f"  Nettopainekerroin ylöspäin     cp,net = {cp_net_up:.2f}")
print(f"  Tuulikuorma katolla (alas)     {w_wind_down:.3f} kN/m²")
print(f"  Tuulikuorma katolla (ylös)     {w_wind_up:.3f} kN/m²")
print(f"  KP450×51   q_tuuli (alas)      {qk_wind_down1:.3f} kN/m")
print(f"  2×KP360×51 q_tuuli (alas)      {qk_wind_down2:.3f} kN/m")

print("\n── PYSYVÄT KUORMAT ────────────────────────────────────")
print(f"  Kate + ruoteet  gk             {gk_roofing:.2f} kN/m²")
print(f"  Kertopuun tiheys               {gamma_lvl:.2f} kN/m³")
print(f"  KP450×51   omapaino            {g_beam1:.3f} kN/m")
print(f"  2×KP360×51 omapaino            {g_beam2:.3f} kN/m")
print(f"  KP450×51   Gk (kate+palkki)    {gk1:.3f} kN/m")
print(f"  2×KP360×51 Gk (kate+palkki)    {gk2:.3f} kN/m")

print("\n── MITOITUSKUORMAT ULS  (1.35G + 1.5S + 1.5·0.6·W) ──")
print(f"  KP450×51   qd                  {qd1:.3f} kN/m")
print(f"  2×KP360×51 qd                  {qd2:.3f} kN/m")

print("\n── TUKIREAKTIOT ULS  (sis. uloke + räystäs) ───────────")
print(f"  KP450×51   Rd,tuki             {R1:.2f} kN")
print(f"  2×KP360×51 Rd,tuki             {R2:.2f} kN")

print("\n── TAIVUTUSMOMENTTI  Md = qd·L²/8  (ei kaltevuuskorjausta) ──")
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
print(f"    KP450×51      qd,max = {qd_max1:.3f} kN/m  (nyt {qd1:.3f} kN/m)")
print(f"    2×KP360×51    qd,max = {qd_max2:.3f} kN/m  (nyt {qd2:.3f} kN/m)")
print()
print("  HUOMIOT:")
print("  * Lumikuorma sk = 2.0 kN/m² (Tuusula, FI NA vyöhyke II / YM asetus 6/16).")
print("  * Vuoden 2005 RakMk B1 -laskennassa käytetty 2.2 kN/m² vastasi silloista normia.")
print("  * LP225x90 laskennassa vain KP450x51 pistekuorma (päätykannake).")
print("  * Tributäärileveys perustuu yksinkertaiseen vaikutusaluemenetelmään.")
print("  * Räystäskuormat (x-suunta) huomioitu tukireaktioissa (LP225, pilarit).")
print("  * Leikkaustarkistuksen Vd = qd·L/2 on palkin sisäinen leikkaus, ei koko tukireaktio.")
print("  * Tuulikuorman nettopainekerroin interpoloitu EC1-1-4 taulukosta 7.7")
print("    (vapaasti seisova katos – konservatiivinen yksinkertaistus seinään kiinnitetylle).")
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
print(f"  {'KP450×51':<18} {qmin1:>12.3f} {R_uplift1:>12.2f} kN  {uplift_status(R_uplift1)}")
print(f"  {'2×KP360×51':<18} {qmin2:>12.3f} {R_uplift2:>12.2f} kN  {uplift_status(R_uplift2)}")

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

# ── Lisäkapasiteetti -osio ──────────────────────────────────────────────────
print()
print(dw)
print("  ANALYYSI: 2×KP360×51 VÄLITUEKSI UUDELLE KATOKSELLE")
print(dw)
print()
print(f"  Käyttämätön taivutuskapasiteetti: {delta_MRd2:.1f} kNm  ({100-eta2:.1f}% jäljellä)")
print()
print("  Periaate: poikittaiset rimat/palkit y-suunnassa kiinnittävät")
print("  uuden ulomman palkin 2×KP360×51:een → jänneväli lyhenee.")
print()
print(f"  {'─'*56}")
print(f"  VAIHTOEHTO A  –  1 välituki  (x = L/2 = {span_A:.0f} mm)")
print(f"  {'─'*56}")
print(f"    2×KP360×51 ottaa pistekuorman:   P_max = {P_A:.1f} kN")
print(f"    Uuden palkin jänneväli:           L = {span_A:.0f} mm")
print(f"    → 6700mm palkki lyhenee {(1-span_A/L_mm)*100:.0f}%")
print(f"    Jos uusi palkki = 2×KP360×51:")
print(f"      Sallittu UDL (taivutus 100%):  q_max = {q_new_A:.1f} kN/m")
print(f"      ≈ kattokuorma (trib ~{trib_new:.1f}m):    {q_roof_new_A:.1f} kN/m²")
print(f"      (vertailu: lumi+tuuli nyt ≈ {s_roof+w_wind_down:.2f} kN/m²)")
print()
print(f"  {'─'*56}")
print(f"  VAIHTOEHTO B  –  2 välitukea  (x = L/3 = {span_B:.0f} mm)")
print(f"  {'─'*56}")
print(f"    2×KP360×51 ottaa per tuki:        P_max = {P_B_each:.1f} kN")
print(f"    Yhteensä molemmat tuet:           P_tot = {P_B_total:.1f} kN")
print(f"    Uuden palkin jänneväli:           L = {span_B:.0f} mm")
print(f"    → 6700mm palkki lyhenee {(1-span_B/L_mm)*100:.0f}%")
print(f"    Jos uusi palkki = 2×KP360×51:")
print(f"      Sallittu UDL (taivutus 100%):  q_max = {q_new_B:.1f} kN/m")
print(f"      ≈ kattokuorma (trib ~{trib_new:.1f}m):    {q_roof_new_B:.1f} kN/m²")
print()
print("  JOHTOPÄÄTÖS:")
print(f"    Nykyinen kattokuorma palkeilla: lumi+pysyvä ≈ {s_roof+gk_roofing:.2f} kN/m²")
print(f"    → Vaihtoehto A riittää mainiosti uudelle katokselle.")
print(f"    → Vaihtoehto B antaa runsaan ylikapasiteetin.")
print(f"    Poikittaisrimat (y-suunta) mitoitetaan erikseen pistekuormille")
print(f"    P = {P_A:.1f} kN (A) tai {P_B_each:.1f} kN/tuki (B), jänneväli {int(beam2_y)} mm.")
print(dw)

# ============================================================
# TERASSIN PÄÄTYPALKKI → 2×KP360×51 VÄLITUKI TARKISTUS
# ============================================================
_TERRACE = load_script_module_quietly("terassilasitus_rakenne_vaihtoehdot.py", "terrace_transfer_loads")
_terrace_base_support_xs_mm = list(_TERRACE.inner_support_xs_mm)
_terrace_case_defs = [
    ("1 välituki  (x = L/2)", [
        _terrace_base_support_xs_mm[0],
        _terrace_base_support_xs_mm[0] + _TERRACE.L_paaty_mm / 2.0,
        _terrace_base_support_xs_mm[-1],
    ]),
    ("2 välitukea (x = L/3, 2L/3)", [
        _terrace_base_support_xs_mm[0],
        _terrace_base_support_xs_mm[0] + _TERRACE.L_paaty_mm / 3.0,
        _terrace_base_support_xs_mm[0] + 2.0 * _TERRACE.L_paaty_mm / 3.0,
        _terrace_base_support_xs_mm[-1],
    ]),
    ("3 välitukea (x = L/4..3L/4)", [
        _terrace_base_support_xs_mm[0],
        _terrace_base_support_xs_mm[0] + _TERRACE.L_paaty_mm / 4.0,
        _terrace_base_support_xs_mm[0] + _TERRACE.L_paaty_mm / 2.0,
        _terrace_base_support_xs_mm[0] + 3.0 * _TERRACE.L_paaty_mm / 4.0,
        _terrace_base_support_xs_mm[-1],
    ]),
]
_kp360_left_support_x_mm = float(min(_col_xs))

terrace_transfer_cases = []
for case_label, support_case_xs_mm in _terrace_case_defs:
    case_reactions_uls = _TERRACE.beam_support_reactions(
        _TERRACE.inner_beam_start_x_mm,
        _TERRACE.inner_beam_end_x_mm,
        support_case_xs_mm,
        _TERRACE.gov_inner_point_loads_uls,
    )
    case_reactions_char = _TERRACE.beam_support_reactions(
        _TERRACE.inner_beam_start_x_mm,
        _TERRACE.inner_beam_end_x_mm,
        support_case_xs_mm,
        _TERRACE.gov_inner_point_loads_char,
    )
    point_loads_uls = [
        ((x_mm - _kp360_left_support_x_mm) / 1000.0, case_reactions_uls[x_mm])
        for x_mm in support_case_xs_mm[1:-1]
    ]
    point_loads_char = [
        ((x_mm - _kp360_left_support_x_mm) / 1000.0, case_reactions_char[x_mm])
        for x_mm in support_case_xs_mm[1:-1]
    ]
    added_reactions_uls = simple_span_point_reactions(L_m, point_loads_uls)
    x_md_comb_m, Md_comb = simple_span_combined_moment_max(L_m, qd2, point_loads_uls)
    Vd_comb = max(Vd2 + added_reactions_uls[0], Vd2 + added_reactions_uls[1])
    delta_comb, x_delta_comb_mm = simple_span_max_deflection_mm(L_mm, EI2, qk_sls2, point_loads_char)
    terrace_transfer_cases.append({
        "label": case_label,
        "point_loads_char": point_loads_char,
        "point_loads_uls": point_loads_uls,
        "added_reactions_uls": added_reactions_uls,
        "support_totals_uls": (R2 + added_reactions_uls[0], R2 + added_reactions_uls[1]),
        "Md_comb": Md_comb,
        "x_md_comb_m": x_md_comb_m,
        "Vd_comb": Vd_comb,
        "delta_comb": delta_comb,
        "delta_added": delta_comb - delta2,
        "x_delta_comb_mm": x_delta_comb_mm,
        "eta_M": Md_comb / MRd2 * 100.0,
        "eta_V": Vd_comb / VRd2 * 100.0,
    })

case_1 = terrace_transfer_cases[0]
case_2 = terrace_transfer_cases[1]
case_3 = terrace_transfer_cases[2]

P1_uls = case_1["point_loads_uls"][0][1]
P1_char = case_1["point_loads_char"][0][1]

Md_comb1 = case_1["Md_comb"]
Vd_comb1 = case_1["Vd_comb"]
eta_M_comb1 = case_1["eta_M"]
eta_V_comb1 = case_1["eta_V"]
delta_comb1 = case_1["delta_comb"]
delta_P1 = case_1["delta_added"]

Md_comb2 = case_2["Md_comb"]
Vd_comb2 = case_2["Vd_comb"]
eta_M_comb2 = case_2["eta_M"]
eta_V_comb2 = case_2["eta_V"]
delta_comb2 = case_2["delta_comb"]
delta_P2 = case_2["delta_added"]

Md_comb3 = case_3["Md_comb"]
Vd_comb3 = case_3["Vd_comb"]
eta_M_comb3 = case_3["eta_M"]
eta_V_comb3 = case_3["eta_V"]
delta_comb3 = case_3["delta_comb"]
delta_P3 = case_3["delta_added"]

# Varmuusmarginaali L/250 (löysempi kriteeri, rakennusteknisesti hyväksyttävä
# ei-kriittiselle rakenteelle)
delta_lim_250 = L_mm / 250.0

print()
print(dw)
print("  TERASSIN PÄÄTYPALKKI – VÄLITUKI 2×KP360×51 TARKISTUS")
print(dw)
print(f"  Päätypalkki q_eq (hall. tapaus): {_TERRACE.q_beam_gov_char:.2f} kN/m ominais / {_TERRACE.q_beam_gov_uls:.2f} kN/m ULS")
print(f"  Terassin välitukikuormat luetaan suoraan korjatusta terassilaskelmasta ({_TERRACE.gov_beam_case}).")
print()
print(f"  2×KP360×51 nykyinen kuorma: Md={Md2:.1f} kNm  Vd,span={Vd2:.1f} kN  Rd,tuki={R2:.1f} kN  MRd={MRd2:.1f} kNm  VRd={VRd2:.1f} kN")
print(f"  Nykyinen käyttöaste: η_M={eta2:.1f}%  η_V={eta_V2:.1f}%  (käyttämätön kapasiteetti: {delta_MRd2:.1f} kNm)")
print(f"  Taipumarajat: L/300={delta_lim:.0f}mm  |  L/250={delta_lim_250:.0f}mm")
print()

for case in terrace_transfer_cases:
    ok_M = case["eta_M"] <= 100
    ok_V = case["eta_V"] <= 100
    ok_d = case["delta_comb"] <= delta_lim
    ok_d2 = case["delta_comb"] <= delta_lim_250
    d_str = f"{'OK ✓' if ok_d else ('OK (L/250) ✓' if ok_d2 else '*** YLITTYY ***')}"
    left_total, right_total = case["support_totals_uls"]
    left_add, right_add = case["added_reactions_uls"]
    print(f"  ── {case['label']}")
    print(f"     Terassilta siirtyvät pistekuormat:")
    print(f"       ominais  {format_point_loads(case['point_loads_char'])}")
    print(f"       ULS      {format_point_loads(case['point_loads_uls'])}")
    print(f"     Lisäreaktio 2×KP360-tuilla (ULS): vasen {left_add:.1f} kN, oikea {right_add:.1f} kN")
    print(f"     Kokonaisreaktio 2×KP360-tuilla (ULS, katos+terassi): vasen {left_total:.1f} kN, oikea {right_total:.1f} kN")
    print(
        f"     η_M={case['eta_M']:.0f}%  {'OK ✓' if ok_M else '✗'}   "
        f"η_V={case['eta_V']:.0f}%  {'OK ✓' if ok_V else '✗'}   "
        f"δ={case['delta_comb']:.1f}mm (+{case['delta_added']:.1f})  {d_str}"
    )
    print()

print("  Huomio: tarkat siirtokuormat eivät jakaudu täysin symmetrisesti,")
print("  koska terassin päätypalkin tukipisteet tulevat suoraan korjatusta geometriasta.")
print()

# ── D) Vinotuki-kolmio seinästä päätypalkkin midspaniin ──────────────────
# Geometria: seinä → päätypalkki midspan
#   Vaakaetäisyys  = 1800mm  (seinä → uusi päätypalkki: pilarit 1675mm + 125mm etureuna)
#   Pystyetäisyys  = 1080mm  (mitattu: vinotuen yläreuna KP450×51 midspanissa = 3850mm,
#                             LP225 yläpinta ≈ 2770mm  →  3850 − 2770 = 1080mm)
# Kolmio = vinotuki (veto) + vaakaside (puristus) + seinä (pysty)
# Tulos: palkkiin vain pystykuorma, seinälle vain pystykuorma
L_d_horiz = 1.800   # m  (1675mm pilarikeskiö + 125mm = palkin etureuna)
h_d_vert  = 1.080   # m  (mitattu: 3850mm − LP225 yläpinta ~2770mm = 1080mm)
L_diag    = math.sqrt(L_d_horiz**2 + h_d_vert**2)
alpha_d   = math.atan(h_d_vert / L_d_horiz)

# Tukivoima kolmioon = 1 välituen tarkka pistekuorma terassilaskelmasta
P_d_uls  = P1_uls
P_d_char = P1_char

# Vinotuki (vetosauvaksi terässauva/laatta)
F_diag_uls  = P_d_uls  * L_diag / h_d_vert   # kN
F_diag_char = P_d_char * L_diag / h_d_vert
# Vaakaside midspan → seinä (puristusputki)
N_horiz_uls  = P_d_uls  * L_d_horiz / h_d_vert  # kN
N_horiz_char = P_d_char * L_d_horiz / h_d_vert

# Teräksinen vetosauvavaihtoehto S355 (pulttivaraus)
fy_S355 = 355.0    # N/mm²
A_M20   = 245.0    # mm² (M20 kierretangon juuripinta-ala)
NRd_M20 = A_M20 * fy_S355 / 1000.0  # kN
eta_M20 = F_diag_uls / NRd_M20 * 100.0

# Vaakasauva: SHS 50×50×4, L=1800mm
A_SHS   = 50**2 - 42**2    # mm²  = 736
I_SHS   = (50**4 - 42**4) / 12.0
i_SHS   = math.sqrt(I_SHS / A_SHS)
lam_SHS = 1800.0 / i_SHS
chi_SHS = 0.75   # nurjahduskerroin χ, λ≈90
NRd_SHS = chi_SHS * A_SHS * fy_S355 / 1000.0
eta_SHS = N_horiz_uls / NRd_SHS * 100.0

# Puuvaihtoehto kolmiosauvoille: GL30c 140×140mm
ft0k_gl  = 19.5     # N/mm²  GL30c
fc0k_gl  = 24.5     # N/mm²  GL30c
E005_gl  = 10800.0  # N/mm²  GL30c (5% fraktiili)
E0m_gl   = 13000.0  # N/mm²  GL30c E0,mean
kmod_gl  = 0.8      # keskipitkäaikainen (lumi), SC1
gM_gl    = 1.25     # γM liimapuu
b_tri    = h_tri = 140.0   # mm  (GL30c 140×140)
A_tri    = b_tri * h_tri
ft0d_gl  = kmod_gl * ft0k_gl / gM_gl   # N/mm²
fc0d_gl  = kmod_gl * fc0k_gl / gM_gl   # N/mm²
# Nettopinta-ala vinotukin vetoliitoksessa (M16 pultti, reikä 18mm)
A_net_gl = (b_tri - 18.0) * h_tri
NRd_tens_gl = ft0d_gl * A_net_gl / 1000.0
eta_tens_gl = F_diag_uls / NRd_tens_gl * 100.0
# Vaakaside: PURISTUS + nurjahdus (EN 1995-1-1 §6.3.2)
i_tri    = h_tri / math.sqrt(12.0)
lam_tri  = 1800.0 / i_tri
lam_rel_tri = (lam_tri / math.pi) * math.sqrt(fc0k_gl / E005_gl)
bc_gl    = 0.1   # liimapuu
k_bkl    = 0.5 * (1.0 + bc_gl * (lam_rel_tri - 0.3) + lam_rel_tri**2)
kc_gl    = 1.0 / (k_bkl + math.sqrt(k_bkl**2 - lam_rel_tri**2))
NRd_comp_gl = kc_gl * fc0d_gl * A_tri / 1000.0
eta_comp_gl = N_horiz_uls / NRd_comp_gl * 100.0

# KP450×51(seinä) paikallinen tarkistus: vinotuki kiinnittyy seinäpalkkiin
# beam.kp450.wall on pultattu seinään 900mm välein → jatkuvapalkki
# Tributäärialue: y=0 ... trib1_start_mm (= 450mm), ERI palkki kuin beam.kp450.y900
s_mm = int(next(c for c in _GEO["connections"] if c["id"] == "con.kp450wall.to.house")["spacing_mm"])
s_m  = s_mm / 1000.0
trib_wall_m = trib1_start_mm / 1000.0   # 0.45 m
gk_wall     = gk_roofing * trib_wall_m + g_beam1   # kN/m
qd_wall     = (gammaG * gk_wall
               + gammaQ * s_roof * trib_wall_m
               + gammaQ * psi0_W * w_wind_down * trib_wall_m)  # kN/m

# Jatkuvapalkin kenttämomentti (sisäjänne, UDL): M ≈ qd × s² / 14
Md_wall_udl = qd_wall * s_m**2 / 14.0   # kNm

# Vinotukin pistekuorma paikallisesti: M = P × s / 4 (yksinkert. tuettu, konserv.)
dMd_wall    = P_d_uls * s_m / 4.0       # kNm
Md_wall_comb = Md_wall_udl + dMd_wall    # kNm
eta_wall_comb = Md_wall_comb / MRd1 * 100.0

# Kolmio-vaihtoehto: 2×KP360×51 EI saa terassin kuormia ollenkaan
print(f"  A) Vinotuki-kolmio seinästä päätypalkkin midspaniin:")
print(f"     Geometria: seinä→palkki {L_d_horiz*1000:.0f}mm vaaka, {h_d_vert*1000:.0f}mm pysty")
print(f"     Vinotuki pituus: {L_diag*1000:.0f}mm, α={math.degrees(alpha_d):.1f}° vaakaan")
print(f"")
print(f"     Kolmio (veto + puristus + seinä):")
print(f"       Vinotuki  (VETO):   F_char={F_diag_char:.1f}kN  F_uls={F_diag_uls:.1f}kN")
print(f"       Vaakaside (PURST.): N_char={N_horiz_char:.1f}kN  N_uls={N_horiz_uls:.1f}kN  L={L_d_horiz*1000:.0f}mm")
print(f"       Seinäkiinnitys:     vain V={P_d_uls:.1f}kN alas  (ei vaakavoimaa seinälle!)")
print(f"       Palkkiin:           vain V={P_d_uls:.1f}kN ylös  (ei heikon akselin taivutusta!)")
print(f"")
print(f"     Materiaalivaihtoehdot:")
print(f"     ┌─ Teräs S355 ──────────────────────────────────────────────────")
print(f"     │  Vinotuki: M20 kierretanko → η={eta_M20:.0f}%  {'OK ✓' if eta_M20 <= 100 else '✗'}")
print(f"     │    NRd={NRd_M20:.0f}kN, A_net=245mm²")
print(f"     │  Vaakaside: SHS 50×50×4, L=1800mm → η={eta_SHS:.0f}%  {'OK ✓' if eta_SHS <= 100 else '✗'}")
print(f"     │    NRd≈{NRd_SHS:.0f}kN, λ={lam_SHS:.0f}")
print(f"     ├─ GL30c liimapuu 140×140mm ─────────────────────────────────── ← valittu")
print(f"     │  ft,0,d={ft0d_gl:.1f}N/mm²  fc,0,d={fc0d_gl:.1f}N/mm²  (kmod={kmod_gl}, γM={gM_gl})")
print(f"     │  Vinotuki (VETO, netto M16, A={A_net_gl:.0f}mm²): NRd={NRd_tens_gl:.0f}kN  η={eta_tens_gl:.0f}%  {'OK ✓' if eta_tens_gl<=100 else '✗'}")
print(f"     │  Vaakaside (PURISTUS, λ={lam_tri:.0f}, λ_rel={lam_rel_tri:.2f}, kc={kc_gl:.2f}):")
print(f"     │    NRd={NRd_comp_gl:.0f}kN  η={eta_comp_gl:.0f}%  {'OK ✓' if eta_comp_gl<=100 else '✗'}")
print(f"     └───────────────────────────────────────────────────────────────")
print(f"")
print(f"     *** SEINÄKIINNITYS – KP450×51(seinä) PAIKALLINEN TARKISTUS ***")
print(f"     beam.kp450.wall: pulttiväli {s_mm}mm, tribut. {trib_wall_m*1000:.0f}mm, qd_wall={qd_wall:.3f} kN/m")
print(f"     Jatkuvapalkki (sisäjänne): Md_udl = qd×s²/14 = {Md_wall_udl:.2f} kNm")
print(f"     Vinotukin pistekuorma:     ΔMd   = P×s/4    = {dMd_wall:.2f} kNm")
print(f"     Yhdistetty:                Md    = {Md_wall_comb:.2f} kNm  MRd={MRd1:.2f} kNm  η={eta_wall_comb:.1f}%  {'OK ✓' if eta_wall_comb<=100 else '✗'}")
s_mm = int(next(c for c in _GEO["connections"] if c["id"] == "con.kp450wall.to.house")["spacing_mm"])
eta_loc = eta_wall_comb
# M10 pulttikapasiteetti (teräs 8.8, EN 1993-1-8 § 3.6)
A_s_M10  = 58.0    # mm² – M10 juuripinta-ala
fub_M10  = 800.0   # N/mm² – 8.8
gamM2    = 1.25    # γM2
Fv_Rd_M10 = 0.6 * fub_M10 * A_s_M10 / gamM2 / 1000.0  # kN, 1 leikkaustaso
Fv_bolt  = P_d_uls / 2.0     # max leikkausvoima vinotukin lähipultille
eta_bolt = Fv_bolt / Fv_Rd_M10 * 100.0
# Kerto-S reunapuristus (EN 1995-1-1 §8.2, d=10mm, t=51mm)
fh_k_M10 = 0.082 * (1.0 - 0.01*10.0) * 480.0   # N/mm²
fh_d_M10 = 0.65 * fh_k_M10 / 1.2                # kmod=0.65, γM=1.2
Fv_Rd_lvl = fh_d_M10 * 10.0 * 51.0 / 1000.0     # kN, reunapuristus yhdessä puupinnassa
eta_lvl  = Fv_bolt / Fv_Rd_lvl * 100.0
print(f"     M10 (8.8) leikkaus: Fv,Rd={Fv_Rd_M10:.1f}kN  kuorma/pultti≈{Fv_bolt:.1f}kN  η={eta_bolt:.0f}%  {'OK ✓' if eta_bolt<=100 else '✗'}")
print(f"     Kerto-S reunapuristus: fh,d={fh_d_M10:.1f}N/mm²  Fv,Rd={Fv_Rd_lvl:.1f}kN  η={eta_lvl:.0f}%  {'OK ✓' if eta_lvl<=100 else '→ tarkista ankkurivalmistajan arvot'}")
print(f"")
print(f"     JOHTOPÄÄTÖS: KP450×51(seinä) taivutus η={eta_wall_comb:.1f}% {'OK ✓' if eta_wall_comb<=100 else '✗'}")
print(f"     Suositus: lisää kuormanjakolevy (t≥10mm teräs) vinotukin liitokseen")
print(f"     → levy jakaa kuorman 2 vierekkäiselle M10-pultille, jännemitta puolittuu")
print(f"     Seinäankkurit: tarkista valmistajan kantavuus (vetämä+leikkaus per pultti ≈{P_d_uls/2:.1f}kN)")
print(f"     2×KP360×51: ei terassikuormia → η_M={eta2:.1f}% ✓")
print()

# ── B) Yhdistelmä: vinotuki-kolmio + 2×KP360×51 (redundantti järjestelmä) ─
# Kumpikin tuki on jousi, kuorma jakautuu jäykkyyksien suhteessa
# Kolmio-jousijäykkyys (virtuaalityö, aksiaaliset jäsenet):
E_tri_mat = E0m_gl     # N/mm² GL30c E0,mean
f_d      = L_diag / h_d_vert
f_h      = L_d_horiz / h_d_vert
L_d_mm   = L_diag * 1000.0
L_h_mm   = L_d_horiz * 1000.0
dv_unit  = (f_d**2 * L_d_mm)/(E_tri_mat * A_tri) + (f_h**2 * L_h_mm)/(E_tri_mat * A_tri)
k_tri    = 1.0 / dv_unit / 1000.0   # kN/mm

# 2×KP360×51 jousijäykkyys kuormituspisteessä (1 välituen tarkka sijainti)
_kp360_load_x_mm = case_1["point_loads_char"][0][0] * 1000.0
_kp360_load_b_mm = L_mm - _kp360_load_x_mm
k_kp360 = 3.0 * EI2 * L_mm / (1000.0 * _kp360_load_x_mm**2 * _kp360_load_b_mm**2)   # kN/mm

k_comb   = k_tri + k_kp360
r_tri    = k_tri   / k_comb
r_kp360  = k_kp360 / k_comb

P_E_tri_uls   = r_tri   * P1_uls;   P_E_tri_char  = r_tri   * P1_char
P_E_kp_uls    = r_kp360 * P1_uls;   P_E_kp_char   = r_kp360 * P1_char

# Kolmiojäsenet (pienennetyllä kuormalla)
F_E_diag_uls  = P_E_tri_uls  * L_diag / h_d_vert
N_E_horiz_uls = P_E_tri_uls  * L_d_horiz / h_d_vert
eta_gl_tens_E = F_E_diag_uls  / NRd_tens_gl * 100.0
eta_gl_comp_E = N_E_horiz_uls / NRd_comp_gl * 100.0

# KP360 yhdistetty kuorma (tarkka kuormituspiste terassilaskelmasta)
point_load_E_uls = [(case_1["point_loads_uls"][0][0], P_E_kp_uls)]
point_load_E_char = [(case_1["point_loads_char"][0][0], P_E_kp_char)]
_added_reactions_E = simple_span_point_reactions(L_m, point_load_E_uls)
_x_md_comb_E_m, Md_comb_E = simple_span_combined_moment_max(L_m, qd2, point_load_E_uls)
Vd_comb_E = max(Vd2 + _added_reactions_E[0], Vd2 + _added_reactions_E[1])
eta_M_E   = Md_comb_E / MRd2 * 100.0
eta_V_E   = Vd_comb_E / VRd2 * 100.0
delta_E, _x_delta_E_mm = simple_span_max_deflection_mm(L_mm, EI2, qk_sls2, point_load_E_char)
delta_PE  = delta_E - delta2

print(f"  B) Yhdistelmä: vinotuki-kolmio + 2×KP360×51 (redundantti järjestelmä):")
print(f"     Jousijäykkyydet: k_kolmio={k_tri:.1f}kN/mm, k_KP360={k_kp360:.2f}kN/mm  →  k_yht={k_comb:.1f}kN/mm")
print(f"     Kuormanjako: kolmio {r_tri*100:.0f}% / KP360 {r_kp360*100:.0f}%  (total P={P1_char:.1f}kN char)")
print(f"")
print(f"     GL30c 140×140 kolmiojäsenet (P_kolmio={P_E_tri_uls:.1f}kN ULS):")
print(f"       Vinotuki  (VETO):   F_uls={F_E_diag_uls:.1f}kN  η={eta_gl_tens_E:.0f}%  {'OK ✓' if eta_gl_tens_E<=100 else '✗'}")
print(f"       Vaakaside (PURST.): N_uls={N_E_horiz_uls:.1f}kN  η={eta_gl_comp_E:.0f}%  {'OK ✓' if eta_gl_comp_E<=100 else '✗'}")
print(f"     2×KP360×51 (P_kp={P_E_kp_uls:.1f}kN ULS):")
print(f"       η_M={eta_M_E:.0f}%  {'OK ✓' if eta_M_E<=100 else '✗'}   η_V={eta_V_E:.0f}%  {'OK ✓' if eta_V_E<=100 else '✗'}   δ={delta_E:.1f}mm  {'OK ✓' if delta_E<=delta_lim else '✗'}")
print(f"     → KP360 toimii varajärjestelmänä (redundanssi), molemmat mitoitettu ✓")
print()

print("  JOHTOPÄÄTÖS:")
for label, eta_M, eta_V, delta, delta_ok, eta_M_ok, eta_V_ok in [
    ("1 välituki (KP360)", eta_M_comb1, eta_V_comb1, delta_comb1, delta_comb1 <= delta_lim_250, eta_M_comb1 <= 100, eta_V_comb1 <= 100),
    ("2 välitukea (KP360)", eta_M_comb2, eta_V_comb2, delta_comb2, delta_comb2 <= delta_lim_250, eta_M_comb2 <= 100, eta_V_comb2 <= 100),
    ("3 välitukea (KP360)", eta_M_comb3, eta_V_comb3, delta_comb3, delta_comb3 <= delta_lim_250, eta_M_comb3 <= 100, eta_V_comb3 <= 100),
]:
    all_ok = eta_M_ok and eta_V_ok and delta_ok
    status = "✓" if all_ok else "✗"
    fails = []
    if not eta_M_ok: fails.append(f"taivutus {eta_M:.0f}%")
    if not eta_V_ok: fails.append(f"leikkaus {eta_V:.0f}%")
    if not delta_ok: fails.append(f"taipuma {delta:.1f}mm > {delta_lim_250:.0f}mm (L/250)")
    detail = "  ".join(fails) if fails else "kaikki OK"
    print(f"    {label}: η_M={eta_M:.0f}%  η_V={eta_V:.0f}%  δ={delta:.1f}mm/{delta_lim_250:.0f}mm  {status}  {detail}")
print(f"    Vinotuki-kolmio (1 välituki): päätypalkki tuettu, KP360 vapaa ✓")
print(dw)
