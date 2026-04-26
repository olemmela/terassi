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

### `terassilasitus_kuormituslaskenta.py` – Lopullisen puurakenteen kuormituslaskenta

Laskee lopullisen puuratkaisun kuormat ja mitoitustarkistukset geometriasta
`geometry/terassi_puu.json`.

**Rakennejärjestelmä:**
- Sisäkattotuolit `198×48` C24 y-suunnassa
- Reunakattotuolit `LP225×140` GL30c y-suunnassa
- Orret / purlins `98×48` C24 x-suunnassa reunakaistoille
- Vinot nurkka- ja reunaorret `98×48` C24 kulma-alueilla
- Sisäpalkki `LP315×140` (GL30c), ulkopalkki `LP225×140` (GL30c)

**Laskenta sisältää:**
- Pysyvät kuormat, lumikuorma ja tuulen alas-/nostotapaukset
- Talon seinää vasten syntyvän kinostuman geometriasta muuttuvalla `h(x)`-korkeudella
- Aurinkopaneelien reunakaistojen kuormansiirron orsien kautta reunakattotuoleille paatypistekuormina noin 15 mm paassa ulkoreunasta
- X-suuntaisten orsien ulomman tuen reunakattotuolilla kuormapuolen tukireunaan asti, ei kattotuolin akselikeskilinjalle
- Ulkokulmien vinojen orsien kuormansiirron geometrian mukaisten liitosten kautta toiseksi uloimpaan kattotuoliin, reunakattotuoliin ja ulkopalkille
- Sisäpään `beam.inner.new`-liitoksen heikosti kiertymää jäykistävän puolijäykän mallin
  (`N`-mallin palkkikenkä 48×136, 5.0×40 ankkuriruuvit, täyskiinnitys; EC5 Kser -likimalli)
- Orsien molempien päiden niveltuet
- Reunimmaisten kattotuolien molempien päiden puolijäykät `N 48×136` -liitokset
- Orsien, nurkkaorten, sisäkattotuolien ja reunakattotuolien taivutus-, leikkaus- ja taipumatarkistukset
- Geometriasta luetut lovi-/nettoh-tarkistukset (`birdsmouth_notch`, `bevel_bottom_notch`, `rect_notch`)
- Ulko- ja sisäpalkin pystysuuntainen kuormitus, ulkopalkin sivulasituksen vaakakuorma
- Pilarireaktiot ja nostotarpeet

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

### `portaikko_kuormituslaskenta.py` – Portaikon katoksen kuormituslaskenta

Laskee portaikon yläpuolisen olemassa olevan katoksen kuormat ja mitoitustarkistukset
geometriasta `geometry/portaikko.json` (leveys 6 100 mm, kaltevuus 12° y-suuntaan,
7 kpl sahatavarakattotuoleja ja jatkuva `LP225×90`-palkki neljällä tuella).

**Rakennejärjestelmä:**
- Kattotuolit `175×50` C24 y-suunnassa
  - Sisätuki: talon seinä / sisäänkäynnin seinä, mallinnettu kiertymäjäykkänä
  - Ulkotuki: olemassa oleva `LP225×90`-palkki, mallinnettu nivellettyenä
  - Puu-uloke palkin yli 225 mm
- `LP225×90` (liimapuu GL30c) x-suunnassa, tuettu `250×250` betonipilarille
  ja kolmelle `LP90×90` puupilarille

**Laskenta sisältää:**
- Pysyvät kuormat (kate + kattotuolien/palkin omapainot)
- Lumikuorma (EN 1991-1-3, FI NA, vyöhyke II, `sk = 2,0 kN/m²`)
- Rafterikohtainen lumikinostuma korkeamman talorakenteen kohdalta
- Huoltokuorma (`Qk = 1,0 kN`, `qk,H = 0,4 kN/m²`)
- Rakennukseen kiinnittyvän avoimen pulpettikatoksen tuulikuorma
- Geometriasta johdetut epäsymmetriset tributäärileveydet kattotuoleille
- Kattotuolien taivutus-, leikkaus- ja taipumatarkistukset (C24)
- LP225-tuella olevan loven nettoleikkaus- ja nettotaivutustarkistuksen geometriasta
- Jatkuvan `LP225×90`-palkin pistekuormatodellisuus, momentit, leikkaus ja taipuma (GL30c)
- Seinä- ja pilarireaktiot sekä nostokuorma

---

### `geometry/` – Rakennelmien geometria JSON-muodossa

Laskelmien **geometria** (pilarit, palkit, kattotuolit, liitokset ja lasitukset)
on kuvattu yksiselitteisessä, LLM-ystävällisessä JSON-muodossa:

- `geometry/schema.json` – yhteinen JSON Schema (draft 2020-12)
- `geometry/katos.json` – nykyisen 12° katoksen geometria
- `geometry/terassi.json` – lasitetun terassin geometria
- `geometry/terassi_puu.json` – lopullisen puuratkaisun geometria
- `geometry/portaikko.json` – portaikon katoksen geometria

**Koordinaatisto:** yhteinen globaali origo talon ulkoseinän nurkassa
(x = seinän suuntaisesti, y = seinästä ulospäin, z = pystysuoraan ylöspäin),
yksikkö mm.

**Sisältö:** `project`-metatiedot, `reference_surfaces` (talon seinä/katto),
`members` (`columns`, `beams`, `rafters`, `purlins`), `connections`
(liitosten topologia) ja `surfaces` (kate, aurinkopaneelit, sivu- ja
kolmiolasit, laudoitukset, aukot). Liitosten yksityiskohdat (pultit,
kannakkeet, ruuvijaot) pysyvät laskelmien proosassa, eivät JSON:issa.
`connections` voi lisäksi sisältää loveus-/leikkausmetatietoa:
vanha yksittäinen `notch` toimii yhä, mutta `notched_over`-liitokselle voi
nyt antaa myös `cuts`-listan. Listan itemit voivat olla
`rect_notch`, `birdsmouth_notch`, `end_bevel_cut` tai
`bevel_bottom_notch`, ja ne voivat käyttää sijaintiviitteitä
`support_inner_edge`, `support_outer_edge`, `support_centerline` tai
`member_end` yhdessä `offset_mm`-kentän kanssa. `rect_notch` käyttää lisäksi
`side`-kenttää (`top` tai `bottom`), jolloin suorakulmainen kolo voidaan
mallintaa joko jäsenen ylä- tai alapintaan. `viewer.py` näyttää nämä
overlay-muotoina; `birdsmouth_notch` johdetaan viewerissä tukijäsenen
geometriasta, joten seat/plumb-kulmat seuraavat oikeaa tukikulmaa.
`surfaces`-objektin `thickness_mm` näkyy
viewerissä tilavuutena, joten esimerkiksi valu- ja levyrakenteet voidaan
mallintaa oikealla paksuudella.

**JSON on totuuden lähde geometrialle:** Python-laskelmat lukevat
primitiiviset geometria-arvot (leveydet, jännevälit, profiilimitat ym.)
suoraan JSON-tiedostoista `geometry_loader.py`:n kautta. Johdettu laskenta
(kaltevuudet, tributary-alueet, kuormat, statiikka) pysyy Pythonissa.
Jos muutat rakenteen geometriaa, muokkaa JSON-tiedostoa ja aja Python
uudelleen – tulokset päivittyvät automaattisesti.

**Validointi:**
```bash
python -c "import json; from pathlib import Path; \
  from jsonschema import Draft202012Validator as V; \
  s=json.load(open('geometry/schema.json')); \
  [V(s).validate(json.load(open(p))) for p in sorted(Path('geometry').glob('*.json')) if p.name != 'schema.json']; \
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
python terassilasitus_kuormituslaskenta.py
python terassilasitus_rakenne_vaihtoehdot.py
python portaikko_kuormituslaskenta.py
```

Skriptit tulostavat laskentatulokset suoraan konsoliin.
Rakenteen geometria luetaan automaattisesti `geometry/`-kansion JSON-tiedostoista.
Riippuvuudet: Pythonin standardikirjasto (`math`, `json`) sekä valinnaisesti
`jsonschema` JSON-tiedostojen validointiin.
