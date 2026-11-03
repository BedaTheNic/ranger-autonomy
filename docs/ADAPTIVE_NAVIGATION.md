# Adaptivna navigacija — Ranger Mini V2, simulacija

Profil `adaptive` bira između Smac Hybrid-A* putanje i Smac State Lattice
putanje. MPPI upravlja Ackermann dionicama, a DWB holonomskim dionicama.
Postojeći profili `ackermann` i `holonomic` ostaju dostupni za usporedbu.
Parametri vrijede za Mini V2 u ovoj simulaciji; nisu kalibracija većeg Rangera.

Od 2026-11-02 holonomski planer i kontroler podržavaju izravne dijagonale,
a odabir globalne rute daje prednost duljini. Detalji promjene i testiranje:
[DIAGONAL_NAVIGATION.md](DIAGONAL_NAVIGATION.md).

U aktualnoj simulaciji servis `/is_retained_path_valid` provjerava preostalu
putanju istim obrisom kao planer. U zapisnik sprema vremensku oznaku odbijene
putanje i prvu neprohodnu pozu. Hybrid-A* zadržava izvornu putanju dok se
provjerava zaglađena kopija. Dugi pravci mogu dobiti kandidata s orijentacijom
šasije u smjeru vožnje, uz provjeru obiju rotacija, potrebnog vremena i prostora.
Čvor `navigation_motion_guard` provjerava konačnu naredbu nakon ograničenja i
eventualnog zaglađivanja. Koristi lokalni costmap, starost skena i izmjerenu brzinu.
Naredbe prije i poslije provjere vide se na `/cmd_vel_candidate` i `/cmd_vel`.
[Upute za ručnu provjeru, snimljene vožnje i rezultati](validation/navigation-improvements-20261103/README.md).

## Pokretanje novog svijeta

Zaustavi prethodnu simulaciju prije pokretanja druge na istoj domeni.
Iz terminala:

```bash
cd /home/bedathenic/bedenic_ws/src/ranger_navigation_startup
bash simulation.sh build
bash simulation.sh navigate profile:=adaptive \
  world:=/home/bedathenic/bedenic_ws/src/ugv_sim/ranger/ranger_mini/worlds/corridor_lab.world \
  map:=/home/bedathenic/bedenic_ws/src/ranger_navigation_startup/maps/corridor_lab/map.yaml \
  gui:=true rviz:=true
```

U drugom terminalu, nakon što AMCL postane aktivan:

```bash
cd /home/bedathenic/bedenic_ws/src/ranger_navigation_startup
bash simulation.sh initial_pose 0 0 0
```

U RVizu koristi `Nav2 Goal`. Početni niz ciljeva je `(3, 0, 0°)`,
`(4, 2.3, 0°)`, `(7, 3.65, 0°)`. Zadržana orijentacija 0° u uskom
poprečnom hodniku traži bočnu vožnju. Duže ravne dionice dopuštaju Ackermann.
Prikaz `Navigation modes` uključuje `HOLONOMIC_ROTATE` za rotaciju na mjestu
i `HOLONOMIC_RECOVERY` za lokalni povratak na putanju. Označava Ackermann zeleno, holonomske dionice
narančasto i ispisuje trenutačni način iznad robota.

Za prekid cilja:

```bash
bash simulation.sh cancel
```

`corridor_lab` ima tri duga hodnika, poprečne prolaze širine 1.20 m,
glavni hodnik širine 1.30 m, prostorije i deset umetnutih prepreka/suženja.
Karta je izvedena iz iste geometrije kao Gazebo svijet; nije rezultat SLAM
snimanja. `generate_corridor_world.py` reproducira svijet, kartu i popis
ciljeva. Za test SLAM-a može se pokrenuti isti `world` uz `simulation.sh slam`.

## Kako se bira način vožnje

1. Globalni planer generira dva kandidata kada je stvarno potreban novi plan.
   Oba koriste costmap i footprint robota. Ackermann kandidat procjenjuje se uz svoju
   orijentaciju i redoslijed promjena smjera, a Lattice kandidat uz dopuštenu
   mješovitu/holonomsku izvedbu. Bira se kraći izvedivi kandidat. Kod numerički
   jednakih duljina prednost ima veća dodatna rezerva oko cijele putanje,
   mjerena u koracima od pola ćelije do šest ćelija. Ako je i rezerva jednaka,
   odluku donosi procjena trajanja. U slobodnom prostoru holonomski
   kandidat povezuje početak i cilj ravnom linijom uz provjeru footprinta.
2. Iz promjene položaja i orijentacije računa se zakrivljenost i potreban
   radijus `R = 1 / |kappa|`. Provjerava se i kut između tangente putanje i
   uzdužne osi šasije. Bočna dionica s konstantnom orijentacijom ima beskonačan
   radijus, ali svejedno nije izvediva Ackermannom.
3. Lokalni limit radijusa je 0,80 m, a globalni Hybrid-A* koristi 1,10 m
   kako bi ostavio prostor za korekciju praćenja. Na pravcu sa stalnom
   orijentacijom dopuštena razlika između smjera translacije i uzdužne osi
   (naprijed ili natrag) iznosi 0,025 rad. Veća razlika zahtijeva bočnu vožnju.
   Ackermann može promijeniti smjer naprijed/natrag na označenoj točki putanje.
   Holonomski kontroler preuzima rotacije na mjestu.
4. Procjena vremena uključuje duljinu, brzine naprijed/natrag/bočno, kutnu
   brzinu, poravnanje šasije, promjene kuta kotača i trošak Ackermann promjene
   smjera. Uspoređuje se mješovita izvedba s potpuno holonomskom izvedbom iste
   preostale putanje. Referenca koja zahtijeva Ackermann orijentacije i
   promjene smjera mora zadržati taj redoslijed; za nju se ne nudi bočno
   praćenje samo XY crte. Kod praktično jednakog vremena (0.25 s) prednost ima
   izvediva Ackermann dionica. To je kinematička procjena, ne jamstvo globalno
   najkraćeg stvarnog vremena.
5. Kontroler ponovno procjenjuje preostalu dionicu bez traženja novog globalnog
   plana. Minimalno zadržavanje holonomskog načina je 2 s; Ackermann otoci
   kraći od 0.65 m se spajaju s okolnim holonomskim dionicama. Već
   započeta Ackermann dionica ne prekida se samo zato što joj preostala
   duljina padne ispod tog praga.

Brzine korištene u procjeni: naprijed 0.20 m/s, Ackermann unatrag 0.15 m/s,
holonomski 0.20 m/s, poravnanje 0.25 rad/s, zakretanje kotača 1 rad/s.
Stvarna brzina ovisi o clearanceu, kontroleru i potrebnom poravnanju kotača.

Lattice skup gibanja usklađen je s lokalnim ograničenjem radijusa 0.80 m.
Uklonjeni su izvorni lukovi radijusa 0.50–0.69 m. Trenutačni skup ima 160
primitiva: 128 translacija (osam smjerova po orijentaciji) i 32 rotacije na mjestu. Holonomski kandidat stoga
može izvesti oštar zavoj translacijom i zasebnim zakretom; Ackermann kandidat
i dalje koristi glatke Hybrid-A* lukove radijusa najmanje 1.10 m.

## Zastoji i ponovno planiranje

Od 3. studenoga 2026. MPPI u svakom ciklusu uzima referentnu točku do 45 cm
ispred projekcije robota na trenutačnu dionicu. Simulira niz brzina i lukova
uz ograničenja brzine, ubrzanja i kočenja. Kako se približava kraju dionice,
referentna brzina opada. Ako cijeli taj pokret ima slobodan obris, početna
procjena za uzorkovanje kombinira 75 % te reference i 25 % prethodnog rješenja.
MPPI ocjenjuje jedan skup od 1200 uzoraka po ciklusu. Kad referenca prolazi
kroz prepreku, uzorkovanje kreće od prethodnog rješenja.

Pri promjeni smjera kontroler posebno provjerava slučaj u kojem je preostala
samo završna točka dolazne dionice. Ako je robot prošao tu točku ili na
preostaloj duljini više ne može postići zadan kut unutar tolerancije,
adaptivni profil zakoči. Unutar 15 cm može se holonomski vratiti na točku
i poravnati provjerenim zakretom prije nastavka iste putanje. Čisti
Ackermann profil tada traži novi prilaz. Pomoć pri praćenju dopušta kretanje
samo ako je referentna točka ispred robota u odabranom smjeru vožnje.

U zabilježenoj vožnji 3. studenoga prvi izbor bila je mješovita putanja
duljine 7,55 m, s procjenom trajanja 49,46 s. Ackermann alternativa
imala je 8,95 m i procjenu 48,07 s. Prednost je imala kraća ruta.
Kad adaptivna Lattice pretraga može upotrijebiti prošireni obris, koristi
još 5 cm izvan osnovne rezerve od 4 cm i čuva taj prostor pri skraćivanju
obilaska oko prepreke.

U Gazebu je izvorni kontroler prekinuo praćenje snimljene putanje od 100
poza na prvoj promjeni smjera. Ažurirana verzija prošla je istu putanju za
85,1 s bez sudara. U 95 % uzoraka odstupanje od plana bilo je unutar 3,9 cm.
Zabilježena su dva lokalna poravnanja prije nastavka u Ackermann načinu.
Tri samostalna Ackermann cilja završila su s jednim planom po cilju.
Ponovljena vožnja do zadanog odredišta `(7.05, -1.11, 1.596 rad)` u
adaptivnom profilu koristila je holonomsku putanju, s jednim globalnim
planom i završnom pogreškom položaja 3,0 cm.
[Mjerenja, snimljena putanja i rezultati provjere](validation/ackermann-continuity-20261103.json)

U novom terminalu možeš očitati zadnju globalnu odluku i zatim pratiti
lokalne odluke dok traje vožnja u adaptivnom profilu.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
ros2 topic echo /navigation_plan_decision std_msgs/msg/String --once --qos-durability transient_local
ros2 topic echo /navigation_mode_decision std_msgs/msg/String
```

MPPI može pri kraju dionice prije promjene smjera pronaći gotovo nultu
brzinu. DWB uz novu prepreku može oscilirati između suprotnih naredbi.
Dodana je pomoć pri praćenju zadržane putanje: nakon izostanka napredovanja
provjerava se spora geometrijska naredba, cijeli footprint i središte robota
uzduž predviđenog gibanja. Nepoznat ili zauzet prostor odbacuje se. DWB mjeri
napredovanje po putanji, tako da njihanje naprijed/natrag ne glumi napredak.
Pomoć sinkronizira stanje DWB kritika s naredbom koju robot stvarno dobije.

BT zadržava globalni plan dok je valjan. Nova prepreka koja presijeca plan
opravdano uzrokuje novi plan; sama promjena načina vožnje to ne radi. Lokalni
kontroler i dalje računa naredbe na 10 Hz i mora reagirati na promjene okoline.
Kad nema sigurne naredbe, smije stati i pokrenuti oporavak. Prebacivanje
između bočne vožnje, rotacije na mjestu i zakrivljene Ackermann vožnje može
zahtijevati kočenje i poravnanje kotača; takav je kratki zastoj fizički potreban.

## Rotacija na mjestu i provjera prolaznosti

Holonomski kontroler sada prati i orijentaciju referentne putanje. Prije
zavoja provjerava željenu orijentaciju do 2 m unaprijed, tako da se poravna
prije ulaska u prolaz u kojem više nema mjesta za zakret. Pogled unaprijed
završava na sljedećoj planiranoj rotaciji na mjestu: njezina izlazna
orijentacija ne primjenjuje se dok robot još prilazi toj točki. Zakret se izvodi
samo ako provjera cijelog zakreta u koracima od 0.025 rad ne nalazi prepreku.
Provjera se ponavlja tijekom manevra; novo opažena prepreka zaustavlja zakret.
Robot najprije koči translaciju, zatim rotira bez bočne naredbe i nastavlja
praćenje istog globalnog plana. Korekcija završava unutar 0.06 rad.
Planirane rotacije bez pomaka središta obavezne su međutočke. DWB dobiva
samo dionicu do sljedeće takve točke, prilazi joj do 2.5 cm i završava zakret
prije nastavka. Projekcija ne smije preskočiti zakret na preklapajućem
odlaznom/povratnom XY dijelu putanje. To vrijedi i u samostalnom holonomskom
i u adaptivnom profilu.
Tijekom započetog zakreta vrijedi prag zadržavanja 4 cm, usklađen s
tolerancijom prolaska kroz međutočku. Time mali pomaci procijenjene poze ne
prebacuju kotače stalno između translacije i rotacije. Veće odstupanje
prekida zakret i ponovno poravnava položaj.
Obavezni zakret koristi završnu pozu te dionice, čak i kada je ona više od
3 m od početka putanje. Ne uspoređuje prolaznost s već prijeđenim hodnikom
i ne čeka pauzu namijenjenu opcionalnim promjenama orijentacije. I dalje
provjerava sav prostor koji footprint zauzima tijekom tog zakreta.

Ako trenutačna orijentacija blokira footprint unutar sljedećih 0.6 m,
provjera drugih orijentacija pokreće se prije zastoja. Provjerava se do
0.8 m referentne putanje. Ista provjera radi i kada DWB ne napreduje ili
ne nalazi dopuštenu brzinu. Uspoređuju se drugi
položaji šasije: tangenta putanje, suprotna tangenta i zakreti za ±90°.
Promjena mora otvoriti barem 0.20 m dodatne prolazne putanje; ne vraća se
odmah na referentnu orijentaciju koja bi opet zatvorila taj prolaz.
`PoseProgressChecker` priznaje i napredovanje kutom, uz pomak središta.
Pri oštrom kutu projekcija na referencu ne preskače vrh zavoja: holonomski
kontroler prilazi vrhu do 5 cm prije promjene smjera. Prijelaz iz holonomskog
u Ackermann način traži odstupanje od putanje najviše 6 cm. Time se izbjegava
ranije izmjereno rezanje kuta za 13 cm i neprikladan Ackermann izlaz iz zavoja.
Kod MPPI pogreške zbog neprohodnih uzoraka prvo se provjerava spora Ackermann
korekcija; adaptivni profil zatim može pokušati kratki holonomski povratak na
referencu. Taj pokušaj ne preskače promjenu smjera naprijed/natrag.
Povratne informacije kotača ostaju valjane i u pivot/crab položaju. Tada
nepostojanje konačnog Ackermann radijusa ne smije proizvesti lažnu poruku
`Missing or stale steering joint feedback`; stvarni nedostatak ili zastarjelost
`joint_states` i dalje zaustavlja Ackermann kontroler.

Globalni planer dodatno provjerava cijeli footprint cilja te interpolirane
poze oba kandidata nakon izvornog planiranja/zaglađivanja. Ne šalje kontroleru
putanju koja već tada pada na provjeri kolizije. Obje costmape koriste footprint s paddingom 0.02 m. Planer interno dodaje
0.04 m rezerve praćenja pri provjeri kandidata i cilja. Provjera zadržanog
plana koristi izvršni footprint, pa sama potrošnja te rezerve ne pokreće novi
plan dok je putanja i dalje prohodna za robot.
Lattice pretraga koristi još 0.025 m rezerve za diskretizaciju. Ako taj
dodatak blokira diskretiziranu početnu ili završnu pozu, pretraga zadržava
obaveznu rezervu praćenja bez dodatka za diskretizaciju. Ti se dodaci ne
upisuju u zajednički footprint costmape. Time pretraga dobiva
rezervu za provjeru interpolirane putanje, umjesto ponovnog slanja istog
kandidata koji padne na završnoj provjeri.
Prag za detaljnu provjeru Lattice footprinta računa se iz opisane kružnice
proširenog footprinta, uz dijagonalu ćelije. U slobodnijem prostoru ostaje
gradijent troška središnje ćelije, koji daje prednost sredini hodnika.
Stalno korištenje maksimuma troška rubova izjednačavalo je prohodne položaje
u uskom hodniku i u ponovljenom testu odabralo putanju preblizu zidu.
Cilj `(13.7986, 5.83077, 0)` uz prepreku u `corridor_lab` nema tu rezervu.
Za probu završnog prolaza koristi `(13.8, 6.0, 0)`; planer ne premješta
korisnikov cilj automatski.
Adaptivni profil koristi završnu XY toleranciju 0.06 m, usklađenu u goal
checkeru i DWB-u. Prethodnih 0.15 m dopuštalo je zaustavljanje prije cilja
uz prepreku, pa početna poza sljedećeg plana više nije imala rezervu
praćenja. Tolerancija se odnosi na lokaliziranu pozu u karti; odstupanje od
stvarne Gazebo poze zasebno se mjeri u testu.
Adaptivna provjera dolaska ima `stateful: false`: XY položaj i završni kut
moraju istodobno biti unutar tolerancije. Na posljednjih najviše 0.30 m
jednosmjerne putanje, unutar 0.18 m od cilja, završni prilaz prelazi u
holonomski način. Prvo se korigira položaj, zatim zakret uz provjeru cijelog
footprinta. Time Ackermann luk radi poravnanja ne odnosi robot od već
dosegnutog položaja. Taj prijelaz ne preskače planirani pivot, promjenu
smjera ni preostali dio petlje koji samo prolazi blizu cilja.

Adaptivni Lattice planer također provjerava završni prilaz točnoj zadanoj
pozi. Ako zaokruživanje položaja ili kuta na lattice mrežu zatvori cilj,
traži prohodnu međupozu unutar `GridBased.goal_connection_distance` (0.25 m).
Tamo može zakrenuti šasiju, a zatim translacijom s konstantnim kutom doći do
izvornog cilja. Cijeli zakret i spoj provjeravaju se s rezervom planiranja;
zauzet cilj i dalje se odbija. To nije tolerancija koja premješta cilj:
zadnja poza plana zadržava zadane koordinate i orijentaciju.

Označavanje novih prepreka iz LiDAR-a ograničeno je na 3 m, a brisanje
raytracingom na 3.5 m. U dugom testu opažene su pomaknute ćelije dalekih
zidova, koje su zatvarale inače prohodan cilj. Statička karta i dalje pokriva
cijeli svijet; ograničenje se odnosi samo na costmap opažanja, ne na SLAM ili
AMCL doseg senzora. BT timeout povećan je s 200 ms na 3000 ms kako dvije
planerske pretrage ne bi izazvale prerani timeout servisnog poziva ili otkaza.

Za snimanje dugog testa, nakon inicijalizacije svježe simulacije hodnika u
domeni 88:

```bash
RANGER_SIM_DOMAIN=88 bash simulation.sh validate_long_route \
  --goal '[13.8,6.0,0.0]' --output /tmp/ranger-long-route.json
```

Test sprema poze, naredbe, promjene načina, stvarne rotacije iz Gazeba,
razloge replaniranja i snimke obje costmape. Željeni rezultat nije samo poruka
`Goal succeeded`, nego i provjerena odsutnost kolizija te istodobne bočne
brzine i kutne brzine, koje baza u ovom načinu ne izvršava zajedno.

## Dijagnostika

- `/navigation_mode`: trenutačni način, uključujući poravnanje i kočenje.
- `/navigation_mode_decision`: radijus (`null` znači ravno / beskonačno),
  geometrijska izvedivost, procjene vremena i broj promjena načina.
  `holonomic_eta: null` označava da ta referenca traži Ackermann slijed poza.
- `/navigation_plan_decision`: odabrani globalni kandidat i njegove procjene.
- `/navigation_modes`: RViz MarkerArray.
- `/plan`, `/local_plan`, `/cmd_vel`, `/odometry/filtered`: putanja i praćenje.

Dijagnostičke ROS naredbe moraju koristiti isti instalacijski overlay i
`ROS_DOMAIN_ID=87`, `ROS_LOCALHOST_ONLY=1` kao interaktivna simulacija.
Automatske validacije rade isključivo u zasebnoj domeni 88.

## Ponovljiva validacija

Nakon pokretanja **svježe** simulacije hodnika uz `RANGER_SIM_DOMAIN=88`
i slanja početne poze u istoj domeni:

```bash
RANGER_SIM_DOMAIN=88 bash simulation.sh validate_tracking \
  --profile adaptive --require-mode-switch \
  --world /home/bedathenic/bedenic_ws/src/ugv_sim/ranger/ranger_mini/worlds/corridor_lab.world \
  --goals '[[3.0,0.0,0.0],[4.0,2.3,0.0],[7.0,3.65,0.0]]' \
  --output /tmp/ranger-corridor-results.json
```

Test mjeri stvarno kretanje u oba načina, promjenu unutar istog cilja,
razliku između putanje i lokalizirane poze, footprint kolizije prema Gazebo
geometriji, ograničenja naredbi, TF roditelje i zaustavljanje nakon otkaza.
`validate_path_retention --profile adaptive` zasebno provjerava novu prepreku
u svijetu `slam_lab`: očekuje početni plan i jedan opravdani novi plan,
bez `FollowPath` oporavka. Taj test također mora krenuti iz svježe simulacije.

Automatske provjere svijeta, geometrije i baznog upravljanja pokreće
`bash simulation.sh test`. C++ regresijski testovi se uključuju buildom
`ranger_nav_plugins` s `-DBUILD_TESTING=ON`, a pokreću kroz CTest.

## Cilj iz loga i redoslijed zakreta (2026-11-01)

Ispravljena je početna odbijenica `goal is occupied with no tolerance` za
cilj `(14.3401, 5.96356, -1.03037)`: Lattice mreža zaokruživala je prohodnu
zadanu pozu na neprohodnu diskretnu pozu. Završni spoj sada zadržava točan
cilj i provjerava cijeli zakret/prilaz. Dodane su provjere rotacije prije
zastoja, poštovanje mjesta planiranih zakreta, korištenje stvarne završne
međutočke na dugim putanjama i izuzimanje obaveznih zakreta iz pauze za
opcionalne promjene orijentacije.

**Cijela vožnja do toga cilja još nije potvrđena uspješnom.** Početna
reprodukcija prekinuta je prije kretanja. Nakon ispravaka robot je slijedio
jedan plan i stigao do završnog zakreta, ali nova LiDAR opažanja označila su
rub završne putanje zauzetim. `IsPathValid` je prekinuo akciju; nije bilo
`FollowPath` neuspjeha ni detektirane kolizije. U posljednjem pokusu s
pragom zadržavanja rotacije RMS praćenja bio je 1.35 cm, a završna poza
udaljena 2.81 cm od cilja uz preostalu kutnu pogrešku 0.493 rad (28.3°).
Akcija je završila statusom 6 nakon 237.1 s; to nije dosegnut cilj.

U prethodnom pokusu, prije praga zadržavanja rotacije, na prvom poništenju
putanje dvije ćelije uz gornji zid, sa središtima
`(14.475, 6.425)` i `(14.525, 6.425)`, promijenile su trošak s 253 na 254.
Geometrijski rub zida je `y=6.5`. Trenutačna lokalizacija i LiDAR opažanja
zato ne daju stabilnu rezervu za taj tijesan završni zakret. Izvršni padding
0.02 m i dodatna rezerva planiranja 0.04 m nisu smanjeni. Potrebna je zasebna
provjera podudaranja skena i karte uz zid prije pouzdanog korištenja ove poze.
I naknadni pokušaj odlaska do kontrolnog cilja iz te tijesne završne poze
odbijen je tijekom planiranja; sačuvan je kao zaseban neuspjeli pokus.

Sirovi rezultati prije i nakon ispravka, međupokušaji i snimke costmapa
sačuvani su u `validation/navigation-goal-*-20261101*`.
`navigation-goal-clearance-20261101.json` bilježi ćelije koje su poništile
završetak. Kontrolni testovi rotacije i nove prepreke navode se zasebno;
njihov uspjeh ne zamjenjuje neuspjeli pokus izvornog cilja.

Odvojeni kontrolni cilj `(0, 0, pi/2)` iz svježe simulacije završen je za
36.4 s uz jedan plan, bez kolizije i bez BT neuspjeha. Završna pogreška
položaja u karti bila je 1.86 cm, a kuta 0.141 rad (8.1°, unutar zadane
tolerancije 0.15 rad). Prije praga zadržavanja rotacije isti test nije
završio u 90 s i imao je 55 promjena načina; nakon ispravka zabilježeno je
13 promjena. To je poboljšanje, a ne tvrdnja da su sva zaustavljanja uklonjena.
Zapisi su `navigation-goal-rotation-before-hysteresis-20261101.json` i
`navigation-goal-rotation-control-20261101.json`.
Ovi pokusi prethode završnoj dopuni provjere dolaska. Ta je dopuna dodana
nakon testa zamjene cilja koji je otkrio lažni `SUCCEEDED`: uz `stateful`
provjeru robot je završio 0.56 m od novog cilja nakon Ackermann poravnanja.
Neuspjeli zapis je `navigation-goal-before-final-alignment-20261101.json`.

Završni regresijski niz na posljednjim bibliotekama prošao je u svježem
`slam_lab` svijetu: nepromijenjeni cilj i kroz-točke po jedan plan;
umetnuta prepreka ukupno dva plana, reakcija 0.947 s i bez `FollowPath`
oporavka; zamjena cilja dva plana i završna XY pogreška 5.48 cm uz kutnu
pogrešku 0.0044 rad. Otkazivanje je vratilo `CANCELED`, nultu naredbu i
mirovanje. Nije zabilježena kolizija. Zapis je
`navigation-goal-obstacle-20261101.json`.

Prošlo je 50 C++ i 24 Python provjera. Testovi su izvedeni bez GUI-ja u
izoliranoj domeni 88; fizički robot nije testiran. Manifest izvora i
instaliranih biblioteka je `validation/navigation-goal-summary-20261101.json`.
Biblioteke su postavljene u zadani `.runtime`; za učitavanje izmjena treba
potpuno ponovno pokrenuti navigacijski launch.

![Posljednji prolaz do izvornog cilja, zaustavljen prije završetka](validation/navigation-goal-20261101.png)

## Validacija dopune rotacije (2026-10-31)

Gazebo Classic, bez GUI-ja, izolirana ROS domena 88. Dva uspješna cilja
izvedena su u istoj simulaciji, bez resetiranja lokalizacije između vožnji:

| Cilj `(x, y, yaw rad)` | Vrijeme | Globalni planovi | RMS praćenja | Završna pogreška u karti |
| --- | --- | --- | --- | --- |
| `(13.7986, 6.0, 0)` | 206.0 s | 1 | 1.60 cm | 8.95 cm |
| `(13.85, 4.45, -1.54592)` | 70.2 s | 1 | 1.59 cm | 9.05 cm |

Obje vožnje prošle su bez `FollowPath`/`IsPathValid` neuspjeha, detektiranih
footprint kolizija i istodobne bočne/kutne naredbe. Druga je uključivala
izmjerenu rotaciju na mjestu od ukupno 1.577 rad, približno 90°.
Prva ruta nije trebala zakret šasije: prohodna bočna vožnja ostaje dopuštena.
Završne pogreške prema Gazebo pozi bile su 5.3 i 7.1 cm. To su pojedinačne
vožnje, a ne statističko jamstvo ponovljivosti ili optimalnog trajanja.

Izvorni drugi cilj `(14.5994, 4.5625, -1.54592)` nakon duge rute odbijen je
jer opaženi footprint nema rezervu uz prepreke. Odbijena je i probna poza
`(14.5, 4.35, -1.54592)`, koja zahvaća donji zid. Te se odbijenice ne računaju
kao uspješna navigacija; između prve i druge uspješne vožnje robot se zbog
njih nije pomaknuo. Raniji izolirani test izvornog drugog cilja prošao je za
44.1 s uz približno 94° zakreta, ali to ne potvrđuje njegovu prohodnost nakon
novih LiDAR opažanja. Za ponavljanje završnog manevra koristiti gore navedenu
provjerenu pozu `(13.85, 4.45, -1.54592)`.

Prošlo je 40 C++ i 24 Python provjera. Dodatno je ispravljeno rušenje planera
pri izlasku: Humble Hybrid i Lattice učitavaju se u odvojenim plugin opsezima,
čime se izbjegava dvostruko oslobađanje zajedničkih statičkih podataka.
Završni `planner_server` nakon navigacije uredno je ugašen.

Ponovljeni test u `slam_lab` potvrdio je jedan globalni plan za nepromijenjeni
cilj i jedan za prolazak kroz međutočke. Prepreka dodana tijekom vožnje
izazvala je jedan opravdani novi plan nakon 0.962 s; cilj je dosegnut s ukupno
dva plana, bez `FollowPath` oporavka i bez kolizije. Prošli su i zamjena cilja
te otkazivanje uz zaustavljanje robota. Zapis je
`validation/navigation-rotation-obstacle-20261031.json`.

Podaci: `validation/navigation-long-rotation-final-20261031.json`,
`validation/navigation-second-after-route-20261031.json` i
`validation/navigation-second-user-goal-after-route-20261031.json`.
Ranije neuspjele probe i rezultati prije ispravka gradijenta troška/završne
tolerancije ostaju sačuvani zasebno. Parametri i provjera prostora vrijede za
ovu simulaciju; ponašanje na fizičkom robotu nije ispitano.
Sažetak provjera i SHA-256 izvora/instaliranih biblioteka nalazi se u
`validation/navigation-rotation-summary-20261031.json`.
Provjerene biblioteke postavljene su u zadani `.runtime`; za učitavanje
izmjena treba potpuno ponovno pokrenuti navigacijski launch.

![Duga ruta i izmjereno zakretanje](validation/navigation-rotation-20261031.png)

## Prethodna validacija, prije dopune rotacije (2026-10-31)

Svi navedeni pokusi izvedeni su bez GUI-ja, na domeni 88. RViz konfiguracija
ima novi prikaz načina vožnje; ova provjera nije vizualni GUI test.

- Samostalni Ackermann: tri referentna cilja, jedan plan po cilju. Ranije
  problematičan treći manevar skraćen je s 104.4 s / dva plana na 62.5 s /
  jedan plan, uz RMS pogrešku 2.1 cm. Nije bilo kolizija.
- Samostalni holonomski test nove prepreke: početni plan i jedan novi plan
  nakon blokiranja putanje; reakcija 0.95 s, bez `FollowPath` oporavka.
  Prošli su i kroz-točke, promjena cilja te otkazivanje.
- Adaptivni test nove prepreke također je prošao s dva plana ukupno,
  reakcijom 0.96 s i bez `FollowPath` oporavka. Prošli su kroz-točke,
  promjena cilja i otkazivanje; bez kolizija.
- Adaptivni `corridor_lab`: ciljevi G1/G2/G3 završeni su za 29.5/31.2/45.8 s,
  svaki s jednim planom. RMS praćenja: 1.8/3.8/5.0 cm; maksimalno odstupanje
  u trećem prolazu 15.6 cm. Oba načina potvrđena su mjerenim kretanjem,
  uključujući prebacivanje unutar trećeg cilja. Nula footprint kolizija i
  nula kršenja Ackermann radijusa; otkazivanje je zaustavilo robot.

Dodatni adaptivni cilj `(1.2, 1.2, 90°)` izvršen je Ackermannom za 16.2 s,
s jednim planom i RMS pogreškom 2.0 cm. Time je provjeren
i zakrivljeni Ackermann slijed orijentacija u adaptivnom kontroleru.

RMS opisuje praćenje putanje, ne završnu točnost cilja. U ovom profilu je
završna pogreška poze u karti približno 16–18 cm. ETA je nominalni model:
primjerice početna procjena ravnog cilja bila je 15.6 s, a stvarno trajanje
29.5 s. Model ne dokazuje da je odabrana izvedba vremenski optimalna.
Poravnanje kotača pri prelasku između crab smjerova i načina vožnje i dalje
zahtijeva kratka zaustavljanja. Nisu potvrđene sve moguće kombinacije uskih
prolaza ni prepreke koje se neprestano kreću.

Podaci: `validation/navigation-progress-ackermann-20261031.json`,
`validation/navigation-progress-obstacle-20261031.json` i
`validation/navigation-adaptive-corridor-20261031.json` te
`validation/navigation-adaptive-obstacle-20261031.json`.
Prijašnji pokušaji ostaju sačuvani kao `trial1` i `trial2`; `trial2` bilježi
odbačenu procjenu holonomskog praćenja Ackermann reference s promjenom smjera.

![Izmjereni prolaz](validation/navigation-adaptive-corridor-20261031.png)

![Raspored cijelog svijeta](validation/corridor-lab-layout-20261031.png)
