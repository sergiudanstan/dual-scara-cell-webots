# Două SCARA Epson T6 — celulă Webots R2025a

Celulă de ambalare cu două roboți **Epson SCARA T6-602S**. Un conveior aduce piese rotunde la un singur punct de preluare. Cei doi roboți se alternează la preluare și umplu fiecare câte o tavă cu 6 alveole. Tava plină iese spre nord, prin gard. Modelul pornește de la structura unei celule de paletizare (PLC, senzori, gard, scanner, opriri), dar brațul de paletizare cu 6 axe este înlocuit de două SCARA.

Epson și T6-602S sunt mărci ale proprietarilor lor. Geometria este modelul Webots R2025a `ScaraT6`, licență Apache 2.0, cu limitele J1/J2 din fișa tehnică (±132° / ±150°). Nu este un produs Epson.

## Demo

Animația 3D în browser (orbită, panoramare, zoom, bară de redare):

https://sergiudanstan.github.io/dual-scara-cell-webots/dual_scara.html

Se reînregistrează din lumea curentă cu `tools/build_site.sh` (fereastra 2–90 s, 25 fps). Publicarea paginii este `git subtree push --prefix site origin gh-pages`.

## Pornire

```
webots worlds/dual_scara_cell.wbt
```

Controloarele sunt Python 3, fără pachete suplimentare. La prima rulare Webots descarcă mesh-urile robotului din biblioteca R2025a.

Ca să refaci lumea după o modificare de layout:

```
python3 tools/gen_world.py
```

## Ciclu

1. Banda merge spre nord cu 0,22 m/s. **B1** vede piesa de la opritor. **B5** vede coada plină și oprește alimentarea.
2. PLC-ul dă coloana de preluare robotului care a așezat mai puține piese. Celălalt poate să lucreze doar pe jumătatea lui.
3. Robotul coboară pe piesă, face vidul, ridică, iese din coloană prin punctul intermediar și raportează `clear`. Abia atunci celălalt poate intra.
4. Piesa ajunge într-o alveolă. După 6 piese, tava iese pe banda de nord, iar o tavă goală vine din magazie.
5. În afara celulei tava este golită (piesele se întorc în alimentator) și revine în magazie.

## Siguranță

| Dispozitiv | Efect |
|---|---|
| Gard cu ușă interblocată pe latura de est | Ușa deschisă oprește totul și reține defectul. Închizi ușa și apeși **R**. |
| Tunel pe gura conveiorului | Nu se poate ajunge la robot prin deschiderea benzii. |
| Oprire de urgență | Roboții țin poziția, banda se oprește, defectul rămâne până la **R**. **R** este refuzat cât timp butonul e apăsat. |
| Scanner de zonă la ieșirea tavilor (nord) | Câmp de avertizare 1,4 m: viteză 40 %. Câmp de protecție 0,75 m: oprire de categoria 2, repornire automată la 1,5 s după eliberare. |
| Lampă de semnalizare | Verde = automat, galben = avertizare sau oprire de ciclu, roșu intermitent = defect. |

## Taste

Click în vederea 3D, apoi:

| Tastă | Acțiune |
|---|---|
| `E` | Oprire de urgență, apăsat / eliberat |
| `D` | Deschide / închide ușa |
| `R` | Reset |
| `P` | Operatorul traversează câmpurile scannerului și se întoarce |
| `S` | Oprire la sfârșit de ciclu / repornire |

Aceleași litere se pot scrie în câmpul `customData` al robotului `cell PLC`.

## Verificări

```
python3 tests/test_cell.py
tests/run_smoke.sh
```

Testul din `tests/test_cell.py` nu pornește Webots. Verifică cinematica, că fiecare țintă a celulei e în spațiul de lucru, că traiectoriile nu trec pe jumătatea celuilalt robot și că dispecerul alternează preluarea. `tests/run_smoke.sh` rulează lumea în Webots, fără fereastră, și cere ca ambii roboți să așeze cel puțin o piesă, ca eroarea de model direct față de simulator să fie sub 2 mm și ca oprirea de urgență și scannerul să reacționeze.

Pe macOS 27, Webots R2025a raportează fals că porturile 1234–1244 sunt ocupate ([#6976](https://github.com/cyberbotics/webots/issues/6976), corectat în Webots după 3 octombrie 2026, PR 7032). `tests/run_smoke.sh` și `tools/build_site.sh` compilă `tools/webots_connect_fix.c` și îl încarcă cu `DYLD_INSERT_LIBRARIES`. Binarul `.dylib` nu se versiona.

## Program

```
lib/layout.py                 cote, un singur loc
lib/geom.py                   cadre și alegerea cotului
lib/dispatch.py               cine preia următoarea piesă
lib/motion.py                 interpolare în articulații și pe dreaptă
controllers/scara_robot/      același program pe ambii roboți
controllers/cell_plc/         secvențe, siguranță, bandă, tăvi
controllers/operator/         pietonul din câmpul scannerului
protos/ScaraT6Cell.proto      T6-602S; articulațiile pornesc de la 0, controlerul le duce acasă
tools/gen_world.py            scrie worlds/dual_scara_cell.wbt
tools/build_site.sh           înregistrează animația HTML5 din site/
controllers/animation_recorder/  pornește și oprește înregistrarea
controllers/demo_cues/        miniatura și traversarea operatorului
```

PLC-ul și roboții schimbă JSON prin Emitter/Receiver, canalele 1–2 pentru A și 3–4 pentru B.
