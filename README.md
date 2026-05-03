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
- Tuulikuorma (EN 1991-1-4, rakennuksen kattolappeen jatke / räystäsuloke:
  yläpinnan kattopaine + alapinnan viereisen seinän paine)
- Kuormayhdistelmät EN 1990 kaavan 6.10 mukaan
- `geometry/katos.json`:n pää- ja vino-orsien geometriasta luetut 50×100-orsien
  tukipisteet KP450- ja KP360-palkeille sekä `rayst`-orsien jatkuvat KP450-sivuliitokset
- Katon ulomman vyöhykkeen kuormansiirron KP360-palkille pää- ja vino-orsien
  viivakuorman tukireaktioina, ei suorana tributäärikaistana
- Pää- ja vino-orsien oman taivutus-, leikkaus-, taipuma- ja nettoh/lovi-tarkistuksen
  `notched_over`-liitosten `bevel_notch`-metadatasta
- `rayst`-orsien käsittelyn jatkuvana KP450-sivutukena, ei erillisenä jännepalkkina
- Taivutus- ja leikkaustarkistukset (EN 1995-1-1, Kerto-S)
- Lateraalinurjahdus (LTB) EN 1995-1-1 §6.3.3
- Taipumatarkistus (SLS, L/300)
- Huoltokuorma (EN 1991-1-1, kategoria H)
- Tuulen nostokuorma ja kiinnitystarkistus
- LP225×90-päätykannakkeen mitoitus
- Koko terassin kokonaispilarikuormat kaikille `katos.json`:n pilareille:
  tehdasvalmisteiset yhtenäiset sisäpilarit sekä betonisen alapalkin varassa
  olevat ulommat maapilarit
- Terassin pintavalun, saumattujen `1200×150` ontelolaattojen,
  terassin hyötykuorman (`q = 2.5 kN/m²`), betonisen alapalkin ja
  betonipilarien kuormareitin geometriasta johdetulla mallilla
- Ontelolaattojen SLS-kuormitustaulukon, jossa näkyvät laattakohtaiset
  seinä- ja ulkopalkkireaktiot sekä vertailu piirustuksen tasaisiin
  kapasiteetteihin `g = 2.0 kN/m²` ja `q = 2.5 kN/m²`

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
- Aurinkopaneelien reuna- ja kulma-alueiden kuormansiirron
  `surfaces[*].load_transfer.member_refs`-metadatasta (nykygeometriassa
  pistekuormina `axis_end`-päästä 15 mm sisään,
  eli `reference = axis_end`, `offset_mm = -15`)
- Aurinkopaneelien kinostumatarkistuksen valmistajan etupuolen 5.40 kN/m²
  mekaanista rajaa vasten, nykygeometrian mitoittavalla paneelisarakkeella
- Seinän viereisen 8 mm lasisen täytekaistan omapaino-, lumi-/kinostuma-,
  noste-, taivutus- ja taipumatarkistuksen sekä ulkoreunan tukiorren kuormareitin
  kattotuoleille
- Pystylasitusten omapaino- ja tuulikuormayhteenvedon geometriasta sekä
  alustavan lasipaneelien oman tuulitaivutus-/jännitystarkistuksen
  yksisuuntaisella ylä-/alakiskojen välisellä kaistamallilla
- Pystylasitusten kuormia vastaanottavien rakenteiden tarkistukset:
  ulkoreunan `beam.outer`-palkin, seinänpuolen aukkolasien LP225×90-
  yläreunapalkkien ja sivulasien reunakattotuolien vaakatuulitarkistukset;
  sivulasien lopullinen kiinnitys- ja omapainon kuormareitti pitää varmistaa
  lasitusjärjestelmän toimittajalta
- Päätykolmiolasin omapainon alapalkille sekä tuulikuorman 50/50-jaon
  alapalkille ja yläreunan `2×KP360×51`-palkille, vaakataivutuksena tarkistettuna
- X-suuntaisten orsien ulomman tuen geometrian `support_line_ref`-metadatasta
  kuormapuolen tukireunaan asti, ei kattotuolin akselikeskilinjalle
- Jäsenten poikkileikkauksen kierrot `section_rotation_deg`-kentästä niin viewerissä
  kuin orsien/nurkkaorsien mitoituksessa
- Ulkokulmien vinojen orsien kuormansiirron geometrian mukaisten liitosten kautta toiseksi uloimpaan kattotuoliin, reunakattotuoliin ja ulkopalkille
- Liitosten `connections.analysis`-metadatasta luetut tuki- ja rotaatiomallit
  (`support_model`, `support_line_ref`, `rotation_spring`)
- Nykygeometrian mukaiset orsien niveltuet, sisäkattotuolien puolijäykkä sisäpään
  palkkikenkä sekä reunimmaisten kattotuolien puolijäykät `N 48×136` -liitokset
- Orsien, nurkkaorten, sisäkattotuolien ja reunakattotuolien taivutus-, leikkaus- ja taipumatarkistukset
- Geometriasta luetut lovi-/nettoh-tarkistukset (`birdsmouth_notch`, `bevel_notch`, `rect_notch`)
- Ulko- ja sisäpalkin pystysuuntainen kuormitus, ulkopalkin sivulasituksen vaakakuorma
- Pilarireaktiot ja nostotarpeet
- `geometry/katos.json`:n olemassa olevan terassin kokonaispilarikuormat täydennettynä
  uuden puuratkaisun lisäkuormilla samoille pilarilinjoille, jolloin raportti näyttää
  kuormat terassin jälkeen myös maahan asti, mukaan lukien ontelolaatan
  omapaino, pintavalu ja terassin hyötykuorma

---

### `terassilasitus_kuormituslaskenta_v2.py` – beam to beam -variantin kuormitus- ja liitoslaskenta

Laskee `geometry/terassi_puu2.json`-vaihtoehdon kuormat ja mitoitustarkistukset,
jossa uusi sisäpalkki `LP315×115` kulkee olemassa olevan `2×KP360×51`-palkin alla
ja vasen reunakattotuoli tukeutuu olemassa olevaan `beam.lp225.x125`-palkkiin samalla
kun sisäpalkin vasemman pään kuormaa siirretään KP360-palkille diskreeteillä
kaistalevyliitoksilla.

**Rakennejärjestelmä:**
- Sisäkattotuolit `198×48` C24 y-suunnassa
- Reunakattotuolit `LP225×115` GL30c y-suunnassa
- Orret / purlins `98×48` C24 x-suunnassa reunakaistoille
- Vinot nurkka- ja reunaorret `98×48` C24 kulma-alueilla
- Sisäpalkki `LP315×115` (GL30c), ulkopalkki `LP225×115` (GL30c)
- Olemassa oleva `LP225×90` vasemmalla reunatuella ja `2×KP360×51` siirtovyöhykkeen alla
- Olemassa oleva `2×KP360×51` (Kerto-S LVL) mallinnettuna lisäkuormaa kantavana siirtopalkkina

**Laskenta sisältää lisäksi:**
- Sisäpalkin ja olemassa olevan `2×KP360×51`-palkin kytketyn 1D-beam-mallin
- `transfer_link`-liitospisteistä luettujen kaistalevyjen jousijäykkyyden
- Kaistalevyjen M12-pulttien ja puun reunapuristuksen kapasiteettitarkistukset
- Kaistalevyjen teräslevyn in-plane shear -tarkistuksen
- Seinän viereisen 8 mm lasisen täytekaistan, sen ulkoreunan tukiorren sekä
  pystylasitusten, seinänpuolen aukkolasien LP225×90-yläreunapalkkien,
  sivulasien reunakattotuolien ja päätykolmiolasin omapaino- ja tuulikuormien
  tarkistukset kuten pääpuuratkaisussa
- Siirtovyöhykkeen perusteella johdetun ekvivalentin vasemman tukipisteen ja
  sisäpalkin efektiivisen jännevälin
- Vasen reunakattotuoli → `beam.lp225.x125` ja oikea reunakattotuoli → `col.existing.inner.x7075`
- Olemassa olevien `LP225×90`- ja `2×KP360×51`-palkkien combined-checkin
  samoilla peruskuormilla kuin `kuormituslaskenta.py`
- Geometriasta luetun sisäpalkin ainoan suoran pilarituen sekä vasemman pään
  transfer_link-pohjaisen kuormansiirron existing-rungolle
- Sisäpalkin `beam.inner.new` -> `beam.existing.kp360x2` sovitusloven
  nettoh-tarkistuksen sekä transfer_linkien `h_net`-marginaalin lovialueella
- `geometry/katos.json`:n olemassa olevan terassin kokonaispilarikuormat täydennettynä
  puu2-variantin lisäkuormilla samoille pilarilinjoille, jolloin raportti näyttää
  kuormat terassin jälkeen myös maahan asti, mukaan lukien ontelolaatan
  omapaino, pintavalu ja terassin hyötykuorma

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

### `beam_analysis.py` – Jaettu FE-palkkianalyysin ydinkirjasto

Yhteinen Euler-Bernoulli-palkin FE-apurikerros laskureille. Sisältää
lineaariratkaisijan, node refinement -apun, elementtikohtaiset viivakuormat,
sisäisten voimien näytteistyksen sekä taipumalaskennan. Sama moduuli tukee
sekä rotaatiojousia että kiinteitä kiertymärajoituksia.

---

### `timber_member_checks.py` – Jaetut puujäsenen poikkileikkaus- ja nettoh-tarkistukset

Yhteinen apumoduuli puujäsenten poikkileikkausominaisuuksille, nettoh-/
lovitarkistuksille ja momentin hallitsevan pisteen poiminnalle. Tätä käyttävät
nyt `kuormituslaskenta.py`, `terassilasitus_kuormituslaskenta.py`,
`terassilasitus_kuormituslaskenta_v2.py` ja `portaikko_kuormituslaskenta.py`.

---

### `terrace_column_loads.py` – Jaettu koko terassin pilarikuormien kuormareitti

Yhteinen apumoduuli `geometry/katos.json`:n koko terassin pilarikuormille. Laskee
tehdasvalmisteisten jatkuvien sisäpilarien sekä betonisen alapalkin varassa
olevien ulompien maapilarien kuormat. Sama moduuli laskee ontelolaatat
yksiaukkoisina seinältä alapalkille, tukee yhtä tai useampaa erillistä
betonista alapalkkia ja käyttää beam-jäsenen `mass_kg`-arvoa omapainoon, jos
se on annettu geometriassa. Terassivaihtoehtojen lisäkuormat voidaan syöttää
samoille pilarilinjoille tapauskohtaisesti.

---

### `geometry/` – Rakennelmien geometria JSON-muodossa

Laskelmien **geometria** (pilarit, palkit, kattotuolit, liitokset ja lasitukset)
on kuvattu yksiselitteisessä, LLM-ystävällisessä JSON-muodossa:

- `geometry/schema.json` – yhteinen JSON Schema (draft 2020-12)
- `geometry/katos.json` – nykyisen 12° katoksen geometria
- `geometry/terassi.json` – lasitetun terassin geometria
- `geometry/terassi_puu.json` – lopullisen puuratkaisun geometria
- `geometry/terassi_puu2.json` – variantti beam-to-beam-siirtovyöhykkeellä
- `geometry/portaikko.json` – portaikon katoksen geometria

**Koordinaatisto:** yhteinen globaali origo talon ulkoseinän nurkassa
(x = seinän suuntaisesti, y = seinästä ulospäin, z = pystysuoraan ylöspäin),
yksikkö mm.

**Sisältö:** `project`-metatiedot, `reference_surfaces` (talon seinä/katto/maanpinta),
`members` (`columns`, `beams`, `rafters`, `purlins`), `connections`
(liitosten topologia + analyysimetatieto) ja `surfaces` (kate,
aurinkopaneelit, sivu- ja kolmiolasit, laudoitukset, aukot). Lisäksi
`foundations` voi kuvata pilarikohtaiset anturat: tuettu pilari (`supports`),
mitat (`size_mm`), maanpintaviite (`ground_ref`), routaeristys,
maanpeitteen ominaispaino ja ankkurointitapa. Perustustarkistus laskee
peitesyvyydet `ref.ground`-pinnasta, joten syvyyttä ei tarvitse ylläpitää
erillisenä rinnakkaisena arvona.
`connections` voi lisäksi sisältää loveus-/leikkausmetatietoa:
vanha yksittäinen `notch` toimii yhä, mutta `notched_over`-liitokselle voi
nyt antaa myös `cuts`-listan. Listan itemit voivat olla
`rect_notch`, `birdsmouth_notch`, `end_bevel_cut` tai
`bevel_notch`, ja ne voivat käyttää sijaintiviitteitä
`support_inner_edge`, `support_outer_edge`, `support_centerline` tai
`axis_start` / `axis_end` yhdessä `offset_mm`-kentän kanssa. `reference` on
pakollinen kaikille cuteille. `rect_notch` ja
`bevel_notch` käyttävät lisäksi `side`-kenttää (`top` tai `bottom`),
jolloin loveus voidaan mallintaa joko jäsenen ylä- tai alapintaan. `viewer.py` näyttää nämä
overlay-muotoina; `birdsmouth_notch` johdetaan viewerissä tukijäsenen
geometriasta, joten seat/plumb-kulmat seuraavat oikeaa tukikulmaa.
Lisäksi `connections.analysis` voi kuvata analyysikäyttäytymisen
(`support_model`, `support_line_ref`, `reaction_distribution`, `rotation_spring`) ja
`surfaces[*].load_transfer` voi kuvata, siirtyykö pinta-kuorma jäsenille
pistekuormana, viivakuormana vai osaviivakuormana; säännöt osoittavat
kohdejäseniin eksplisiittisesti `member_refs`-listalla. Tämä on toistaiseksi
otettu käyttöön lopullisen puurakenteen laskurissa
`terassilasitus_kuormituslaskenta.py` tiedostolle `geometry/terassi_puu.json`.
`reaction_distribution` kuvaa jäsenliitoksen tukireaktion paikallisen
jakautumisen: esimerkiksi ontelolaatan päätyreaktio voidaan siirtää alapalkille
tasaisena kuormana laatan profiilileveyden matkalle
(`uniform_over_supported_member_width`).
Lisäksi `connections[*].detail` voi kuvata fyysisen liitosdetaljin, jota ei
vielä käytetä suoraan laskennassa. Tällä hetkellä käytössä on
`plate_bracket`, jolla voidaan tallentaa esimerkiksi LP225×90-tuennan
erilliset teräslevyt (`plates`) ja niiden reikä-/liitospisteet (`points`):
kumpaan jäseneen latat on valettu (`host_member`), latan leveys/paksuus,
näkyvä + upotettu pituus, levyjen poikittaisoffsetit tukikeskilinjasta sekä
levykohtaiset pisteet.
`viewer.py` näyttää ja piirtää nämä liitospisteen yhteyteen ilman että ne
osallistuvat varsinaiseen kuormituslaskentaan.
Lisäksi `connections` voi käyttää tyyppiä `transfer_link`, jonka `transfer`-aliosio
kuvaa beam-to-beam -siirtolinkin (esim. kaistalevyn) geometrian ja kiinnityksen:
`strip_width_mm`, `plate_height_mm`, `outer_plate_thickness_mm`,
`inner_plate_thickness_mm`, `fastener_d_mm`, `fastener_grade` ja
`fastener_count_per_member`. Optionaalinen `fasteners`-lista voi lisäksi kuvata
eksplisiittiset pulttipaikat suhteessa levyn keskipisteeseen (`offset_along_strip_mm`,
`offset_height_mm`) ja mille jäsenelle kukin pultti kuuluu (`member_ref`).
`viewer.py` näyttää nämä kaistalevyt nyt myös 3D-näkymässä itse levyinä ja
pultit levyn läpi, ei vain liitospisteinä. Levyt ja transfer_link-pultit
näkyvät aina, vaikka liitospisteiden näyttö olisi pois päältä; varsinaiset
liitospistepallot seuraavat edelleen liitospistenäkymän kytkintä. Myös
`plate_bracket`-detailien pultti-/reikämarkerit näkyvät aina. Tätä käyttää puu2-variantin laskuri
`terassilasitus_kuormituslaskenta_v2.py` tiedostolle
`geometry/terassi_puu2.json`.
Kuormansiirron `reference` käyttää eksplisiittisiä jäsenviitteitä
`axis_start` / `axis_end` tai tukiviitteitä `support_*`; `offset_mm` tulkitaan
jäsenen paikallisen akselin suunnassa (positiivinen = `axis_start → axis_end`).
Lineaariset jäsenet voivat lisäksi antaa `section_rotation_deg`-kentän, jossa
`0` tarkoittaa profiilin `h_mm`-suunnan olevan pystyssä ja positiivinen kulma
kiertää poikkileikkausta oikean käden säännöllä akselin `axis_start → axis_end`
suuntaan.
`surfaces`-objektin `thickness_mm` näkyy
viewerissä tilavuutena, joten esimerkiksi valu- ja levyrakenteet voidaan
mallintaa oikealla paksuudella.
`viewer.py` näyttää myös `foundations`-anturalaatikot läpikuultavina 3D-kappaleina
pilarien alla.

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
python terassilasitus_kuormituslaskenta_v2.py
python terassilasitus_rakenne_vaihtoehdot.py
python portaikko_kuormituslaskenta.py
```

Skriptit tulostavat laskentatulokset suoraan konsoliin.
Rakenteen geometria luetaan automaattisesti `geometry/`-kansion JSON-tiedostoista.
Riippuvuudet: Pythonin standardikirjasto (`math`, `json`) sekä valinnaisesti
`jsonschema` JSON-tiedostojen validointiin.
