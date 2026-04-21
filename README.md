# Terassin kuormituslaskelmat

Rakennetekniset kuormituslaskelmat lasitetulle terassille ja sen yhteydessä olevalle katokselle.
Kohde: 04330 Lahela, Tuusula.

Laskelmissa noudatetaan eurokoodeja (EN 1990, EN 1991, EN 1995) ja Suomen kansallisia liitteitä (FI NA).

---

## Tiedostot

### `kuormituslaskenta.py` – Olemassa olevan katoksen kuormituslaskenta

Laskee kuormat ja mitoitustarkistukset nykyiselle yksikalteiselle katokselle (kaltevuus 12°, jänneväli 6 700 mm).

**Rakennejärjestelmä:**
- `KP450×51` (Kerto-S) – seinään pultattu palkki, 900 mm seinästä
- `2×KP360×51` (Kerto-S) – kaksoispalkkina tolpilla, 1 675 mm seinästä
- `LP225×90` (liimapuu GL30c) – päätykannake seinältä pilarille (y-suunta, 1 675 mm)

**Laskenta sisältää:**
- Pysyvät kuormat (kate + palkkien omapainot)
- Lumikuorma (EN 1991-1-3, FI NA, vyöhyke II, `sk = 2,0 kN/m²`)
- Tuulikuorma (EN 1991-1-4, nettopainekertoimet yksikalteiselle katokselle)
- Kuormayhdistelmät EN 1990 kaavan 6.10 mukaan
- Taivutus- ja leikkaustarkistukset (EN 1995-1-1, Kerto-S)
- Lateraalinurjahdus (LTB) EN 1995-1-1 §6.3.3
- Taipumatarkistus (SLS, L/300)
- Huoltokuorma (EN 1991-1-1, kategoria H)
- Tuulen nostokuorma ja kiinnitystarkistus
- LP225×90-päätykannakkeen mitoitus
- 2×KP360×51:n jäljellä oleva kapasiteetti laajennusta varten

---

### `terassilasitus_rakenne_vaihtoehdot.py` – Lasitetun terassin kuormituslaskenta

Laskee kuormat ja vertailee rakennevaihtoehtoja suunnitteilla olevalle lasitetulle terassille
(leveys 7 200 mm, syvyys 3 600 mm pilareista ulospäin, kaltevuus 7,25° ulospäin).

**Katteena** aurinkopaneelit Longi Himo X10 LR7 (1 990×1 134 mm, 25 kg/kpl), 7×2 = 14 kpl.

**Kantavat rakenteet:**
- Kattotuolit (y-suunta, 1 134 mm välein, jänneväli ~3 430 mm)
  - Sisätuki: olemassa oleva `2×KP360×51`
  - Ulkotuki: uusi ulkoreunanen palkki
- Ulkoreunanen palkki (x-suunta, kaksi 3 600 mm jänneväliä, 3 pilaria)

**Laskenta sisältää:**
- Pysyvät kuormat (aurinkopaneelit, kiinnikkeet, kattotuolien omapaino)
- Lumikuorma ja lumikinostuma talon seinältä (EN 1991-1-3 §6.3)
- Tuulikuorma avoimelle katokselle (taulukko 7.7) sekä suljetulle lasitukselle (taulukko 7.4 + sisäpaine)
- Sivulasituksen vaakakuorma ulkoreunapalkille
- Kuormayhdistelmät ja nostokuormatarkistus
- **Materiaalivertailu kattotuoleille:** liimapuu GL30c (LP180×90, LP225×90, LP315×90) vs. teräs (IPE100–IPE160, HEA120–HEA160)
- Ulkoreunanen palkki: RHS-teräsprofiilivertailu (S235)
- Pilarikuormat
- Päätypalkki: kattotuolireaktiot + kinostuma + kolmiolasikuorma

---

### `geometry/` – Rakennelmien geometria JSON-muodossa

Molempien laskelmien **geometria** (pilarit, palkit, kattotuolit, liitokset ja lasitukset)
on kuvattu yksiselitteisessä, LLM-ystävällisessä JSON-muodossa:

- `geometry/schema.json` – yhteinen JSON Schema (draft 2020-12)
- `geometry/katos.json` – nykyisen 12° katoksen geometria
- `geometry/terassi.json` – lasitetun terassin geometria

**Koordinaatisto:** yhteinen globaali origo talon ulkoseinän nurkassa
(x = seinän suuntaisesti, y = seinästä ulospäin, z = pystysuoraan ylöspäin),
yksikkö mm.

**Sisältö:** `project`-metatiedot, `reference_surfaces` (talon seinä/katto),
`members` (`columns`, `beams`, `rafters`, `purlins`), `connections`
(liitosten topologia) ja `surfaces` (kate, aurinkopaneelit, sivu- ja
kolmiolasit, laudoitukset, aukot). Liitosten yksityiskohdat (pultit,
kannakkeet, ruuvijaot) pysyvät laskelmien proosassa, eivät JSON:issa.

**JSON on totuuden lähde geometrialle:** Python-laskelmat lukevat
primitiiviset geometria-arvot (leveydet, jännevälit, profiilimitat ym.)
suoraan JSON-tiedostoista `geometry_loader.py`:n kautta. Johdettu laskenta
(kaltevuudet, tributary-alueet, kuormat, statiikka) pysyy Pythonissa.
Jos muutat rakenteen geometriaa, muokkaa JSON-tiedostoa ja aja Python
uudelleen – tulokset päivittyvät automaattisesti.

**Validointi:**
```bash
python -c "import json; from jsonschema import Draft202012Validator as V; \
  s=json.load(open('geometry/schema.json')); \
  [V(s).validate(json.load(open(p))) for p in ['geometry/katos.json','geometry/terassi.json']]; \
  print('OK')"
```

---

## Standardit ja viitteet

| Standardi | Käyttötarkoitus |
|---|---|
| EN 1990 | Kuormayhdistelmät |
| EN 1991-1-1 | Hyötykuormat (huoltokuorma) |
| EN 1991-1-3 + FI NA | Lumikuorma, Tuusula vyöhyke II, `sk = 2,0 kN/m²` |
| EN 1991-1-4 + FI NA | Tuulikuorma, Eteläsuomi vyöhyke I, `vb0 = 21 m/s` |
| EN 1995-1-1 | Puurakenteet (Kerto-S LVL, liimapuu GL30c) |
| EN 1993-1-1 | Teräsrakenteet (S235) |

---

## Ajaminen

```bash
python kuormituslaskenta.py
python terassilasitus_rakenne_vaihtoehdot.py
```

Molemmat skriptit tulostavat laskentatulokset suoraan konsoliin.
Rakenteen geometria luetaan automaattisesti `geometry/`-kansion JSON-tiedostoista.
Riippuvuudet: Pythonin standardikirjasto (`math`, `json`) sekä valinnaisesti
`jsonschema` JSON-tiedostojen validointiin.
