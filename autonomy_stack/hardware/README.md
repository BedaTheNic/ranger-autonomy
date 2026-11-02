# Ranger autonomy — stvarni robot, ROS 2 Jazzy

Ovaj deployment pakira lokalno razvijeni adaptivni autonomy stack i njegov
`ranger_logging`. Kanonski izvori su `ugv_sim/ranger/ranger_nav_plugins`,
`ugv_sim/ranger/ranger_mini/config` i `ugv_sim/ranger/ranger_logging`.
Dockerfile preuzima te datoteke izravno. Manifest SHA-256 ulaza nalazi se u
imageu na `/opt/autonomy/source-manifest.json`.

Zadržano ponašanje: izbor Hybrid-A*/Lattice putanje prema ETA, MPPI za
Ackermann dionice, DWB za holonomske dionice, provjera footprinta, planirani
zakreti, završni holonomski prilaz i zadržavanje valjanog globalnog plana.
Nova prepreka/neizvediva putanja, novi cilj ili neuspjelo praćenje mogu pokrenuti
novi plan. Migracija na Jazzy mijenja API-je, nazive pluginova i BT format;
ne zamjenjuje vlastite kontrolere standardnim Nav2 primjerom.

Detalji ponašanja i ranijih provjera su u `ranger_navigation_startup/ADAPTIVE_NAVIGATION.md`
i `LOGGING.md` (u izvezenom GitHub projektu: `docs/`). Ti dokumenti sadrže i
poznate neuspjele simulacijske pokuse; oni nisu potvrda fizičke navigacije.

## Kontejneri

Svaki red predstavlja zaseban kontejner s jednim aplikacijskim ROS procesom.
Svi koriste isti image; ne gradi se druga kopija imagea za svaki servis.

| Servis | Proces / uloga | Kada radi |
|---|---|---|
| planner | planner_server, AdaptivePlanner | uvijek |
| controller | controller_server, AdaptiveController MPPI/DWB | uvijek |
| behaviors | behavior_server | uvijek |
| bt-navigator | bt_navigator, vlastiti BT | uvijek |
| navigation-lifecycle | Nav2 lifecycle manager | uvijek |
| telemetry | ranger_logging, pasivna telemetrija | uvijek |
| slam | SLAM Toolbox | mapping |
| slam-lifecycle | lifecycle manager za SLAM | mapping |
| map-server | map_server | localization |
| amcl | AMCL | localization |
| localization-lifecycle | manager za map_server/AMCL | localization |
| scan-adapter | PointCloud2 → LaserScan | opcionalno |
| velocity-smoother | velocity_smoother | samostalni ackermann profil |
| command-guard | završne Ackermann granice | samostalni ackermann profil |

Launch/konzolni omotač u svakom servisu bilježi njegove procese. Samo jedan
`telemetry` kontejner snima zajedničke ROS topice. Postojeći driveri baze,
Livoxa, robot_state_publisher i ostale aplikacije robota ostaju u postojećem
robotovom deploymentu. Novi stack ne pokreće Gazebo niti drugi EKF.

## Potvrđeni podaci robota

Dijagnostika `robot_diagnostics_20261102_104903__0bzdr18.tar.gz` potvrđuje:

- ROS 2 Jazzy, Ubuntu 24.04, x86_64/amd64, ROS domena **9**.
- `/livox/lidar`: PointCloud2, oko 10 Hz, frame `livox_frame`.
- `/livox/imu`: Imu, približno 200 Hz; jedinice i kalibracija još nisu potvrđene.
- `/odom`: Odometry, oko 50 Hz, `odom → ranger_base_link`.
- Postojeći TF do LiDAR-a: `(0.288, -0.007, 0.046)` m i yaw `+90°`.
- Postojeći `/scan`, `/scan_obstacles`, `/slam_toolbox`, `/map` i `map → odom`.
- `/ranger_base_node` prima `/cmd_vel` tipa Twist. Na taj ulaz već objavljuju
  teleop, spray_executor_node i kinova_state_machine_node.

Costmape koriste `/scan_obstacles` (`range_min≈0.4 m`); lokalizacija `/scan`
(`range_min=1.0 m`). Ne objavljujemo još jedan LiDAR TF.

Naknadna provjera procesa na robotu potvrdila je domenu 9 i stvarno učitanu
biblioteku `librmw_fastrtps_cpp.so` za bazu i Livox. Postojeći kontejner
`ranger_hero_jazzy` koristi `Network=host`, `IPC=private`. Naši Fast DDS procesi
zato zadano koriste `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`; postojeći kontejner se
ne mora ponovno stvarati ili gasiti. Varijabla je u zajedničkom Compose
okruženju svih naših servisa. Sam dodatan zapis u `.env` bez prosljeđivanja kroz
Compose ne bi bio dovoljan. Ako koristiš vlastiti Fast DDS XML koji isključuje
builtin transporte, provjeri i njegovu konfiguraciju.

Deployment arhiva `jazzy-r3` objavljena je prije ove provjere IPC-a. Za nju je
dovoljno preuzeti aktualni `autonomy_stack/hardware/compose.yaml` iz repozitorija
u lokalni deployment direktorij. Image ostaje `jazzy-r3`; promjena je samo u
Compose okruženju. Pokrenute naše kontejnere treba ponovno stvoriti naredbom
`bash robot.sh external` (ili odabranim mapping/localization načinom).

**Adaptivni profil je zadan.** Za Ackermann dio potrebni su svježi izmjereni
kutovi `fl_steering_wheel`, `fr_steering_wheel`, `rl_steering_wheel`,
`rr_steering_wheel` u JointState poruci. `STEERING_FEEDBACK_TOPIC` određuje ulaz.
Sama prisutnost `/joint_states` nije dovoljna: poruke mogu sadržavati samo
zglobove Kinove. Nije dodana zamjena mjerenja procijenjenim kutovima iz naredbi.

Provjera na robotu potvrdila je prijem svih senzora i traženih TF veza iz našeg
kontejnera. Jedina neuspjela stavka bila je izmjerena povratna informacija
upravljanja: `/joint_states` sadrži Kinovu. Driver je konfiguriran za
`ranger_mini_v3` i objavljuje `/actuator_state`; svih osam zapisa u jednom
uzorku ponavlja iste vrijednosti. To samo po sebi, dok robot miruje, ne dokazuje
grešku drivera niti potvrđuje četiri neovisna kuta.

Za pasivan pregled izvora postojećeg drivera i CAN feedbacka pokreni na hostu:

```bash
docker exec -i ranger_hero_jazzy python3 - < inspect_drive.py > ranger-drive-details.json
```

`inspect_drive.py` radi unutar već pokrenutog kontejnera, koristi Pythonovu
standardnu biblioteku i ne treba ROS setup. Čita samo odabrane C++ datoteke,
putanje aktivnog drivera i do 8 sekundi standardnih CAN poruka `0x200..0x2ff`
na postojećem `can0`. Ne piše u taj kontejner, ne mijenja CAN sučelje, ne
šalje naredbe, ne pokreće drugi driver i ne dekodira pretpostavljeni redoslijed
kotača. Nedostupan izvor ili CAN pristup ostaje zabilježen u JSON-u. Pregled
izvora na disku ne potvrđuje da je upravo iz njega izgrađen aktivni binary.
Skripta se može preuzeti zasebno iz ovog direktorija bez nove verzije imagea;
stara arhiva `jazzy-r3` još je ne sadrži.

Nav2 izlaz ostaje `/cmd_vel_nav`. Za fizičku vožnju još treba povezati izbor
izvora naredbi, timeout i zaustavljanje s postojećim pogonom, provjeriti
izmjerenu povratnu informaciju kotača i stvarni footprint uključujući opremu.
Početne granice i footprint su naslijeđeni iz razvijenog stacka, ne izmjereni
na ovoj konfiguraciji fizičkog robota. Na amd64 vlastiti MPPI zahtijeva AVX2/FMA.

## Izvorni paket za GitHub

Repozitorij je predviđen kao `BedaTheNic/ranger-autonomy`, a image kao
`ghcr.io/bedathenic/ranger-autonomy`. Imageovi i deployment arhive objavljuju se u GHCR i GitHub Releases nakon
uspješnog workflowa.

Iz direktorija `autonomy_stack/hardware`:

```bash
bash release.sh source "$PWD/dist/github-source"
```

Ako ta mapa već postoji, odaberi drugi naziv; izvoz ne prepisuje stare pakete.

Izvoz kopira samo potrebne pakete, konfiguracije, testove i dokumentaciju.
Ne uključuje lokalne karte, logove, .env, Git metapodatke ili dijagnostičku
arhivu. Ne stvara Git submoduleove: novi repo može se izgraditi samostalno.

Na GitHubu napravi prazan repozitorij `ranger-autonomy`, pa na ovom računalu:

```bash
cd dist/github-source
git init -b main
git add .
git commit -m "Package Ranger Jazzy autonomy and structured logging"
git remote add origin https://github.com/BedaTheNic/ranger-autonomy.git
git push -u origin main
```

Git za push mora biti prijavljen kao vlasnik repozitorija. Ne koristi se remote
postojećeg `ugv_sim`, koji pokazuje na upstream AgileX repozitorij.

### Opcija A: build na ovom računalu, objava imagea u GHCR

Host treba Docker Engine i Buildx; ROS instalacija na hostu nije potrebna.
Za native amd64 lokalni build:

```bash
bash release.sh build ranger-autonomy:jazzy-r3
```

Gradi se samo `linux/amd64`, prema dijagnostici robota. Na ovom amd64
računalu nisu potrebni QEMU ni ARM build.

Prijavi se u registry (pri upitu za lozinku koristi svoj GHCR token):

```bash
docker login ghcr.io -u BedaTheNic
bash release.sh publish ghcr.io/bedathenic/ranger-autonomy:jazzy-r3
```

Zadano se gradi i objavljuje samo `linux/amd64`, arhitektura stvarnog robota.
Za drugi registry promijeni referencu, npr. `docker.io/USER/ranger-autonomy:jazzy-r3`
ili `registry.example.com/TEAM/ranger-autonomy:jazzy-r3`, i prijavi se u njega.
Više tagova/registryja može se zadati nakon platformi:

```bash
bash release.sh publish ghcr.io/bedathenic/ranger-autonomy:jazzy-r3 \
  linux/amd64 docker.io/USER/ranger-autonomy:jazzy-r3
```

Build kontekst je nadređeni direktorij s `autonomy_stack`, `ugv_sim` i
`ranger_ros2`. `release.sh` sam pronalazi taj direktorij.

### Opcija B: GitHub Actions gradi i objavljuje

Izvezeni repo već sadrži `.github/workflows/publish.yaml`. Na GitHubu otvori
Actions → Verify and publish Ranger autonomy → Run workflow, tag `jazzy-r3`.
Alternativno objavi Git tag `jazzy-r3`.

Workflow prvo gradi amd64 image, pokreće izvorne C++/logger regresije i izolirani
adaptivni test planiranja/logiranja. Nakon prolaza objavljuje isti provjereni amd64 image u GHCR, bez ponovnog builda. Koristi repo `GITHUB_TOKEN` s `packages: write`; token se ne
sprema u izvorni kod. Repo/org postavke moraju dopuštati Actions i taj pristup
paketima. Deployment arhiva dostupna je kao workflow artifact
`ranger-hardware-deploy` i, nakon objave imageova, kao GitHub Release asset.
Arhiva sadrži i ne-tajni `.env` preset s image referencom tog releasea. Ovaj workflow ne gradi ARM varijantu.

## Robot: samo pull i pokretanje

Na build računalu napravi deployment arhivu:

```bash
bash release.sh bundle
```

Prenesi `dist/ranger-hardware-deploy.tar.gz` na robot (ili preuzmi CI artifact).
Na Docker hostu robota:

```bash
tar -xzf ranger-hardware-deploy.tar.gz
cd ranger-hardware
# Deployment arhiva već sadrži .env; za izvorni checkout robot.sh ga stvara.
# U .env provjeri AUTONOMY_IMAGE, domenu 9 i RMW.
# Za privatni image prvo: docker login ghcr.io -u BedaTheNic
bash robot.sh pull
```

Image je izgrađen za robotovu amd64 arhitekturu. Na robotu nisu potrebni izvorni kod,
colcon, Buildx ni build imagea.

Ako želiš zadržati već pokrenuti SLAM s robota:

```bash
bash robot.sh external
bash robot.sh check
```

Za cijeli naš stack s vlastitim mapiranjem prvo kroz postojeći startup
zaustavi **samo njegov SLAM**, uz zadržavanje senzora/TF-a/baze. Prethodno spremi
postojeću kartu i posegraph ako ih želiš zadržati. Zatim:

```bash
bash robot.sh mapping
bash robot.sh check
```

SLAM i AMCL ne smiju istodobno posjedovati `map → odom`. Skripta prebacuje samo
servise vlastitog Compose projekta i ne gasi vanjski robotov kontejner.

Spremanje karte našeg SLAM-a:

```bash
bash robot.sh ros slam ros2 run nav2_map_server map_saver_cli -f /maps/map
```

Za AMCL stavi YAML i pripadajuću sliku u `maps/`, u `.env` postavi `MAP_FILE`, pa:

```bash
bash robot.sh localization
```

Postavi AMCL početnu pozu prije cilja. Za samostalni Ackermann profil postavi
`NAV_PROFILE=ackermann`; `robot.sh` automatski uključuje njegove zasebne smoother
i guard kontejnere. Razvijeni adaptivni profil koristi vlastite MPPI/DWB granice.

Opcionalni scan adapter nije potreban uz postojeće pretvarače. Uključi ga s
`ENABLE_SCAN_ADAPTER=1 bash robot.sh mapping` (ili external/localization).
Izlaz mu je `/autonomy/scan`; u `.env` uskladi `OBSTACLE_SCAN_TOPIC` i/ili
`LOCALIZATION_SCAN_TOPIC` ako ga želiš koristiti umjesto postojećih skenova.

## Logovi i provjere

Svaki poziv mapping/localization/external dobiva ID pokretanja. Logovi su na
hostu u `logs/<ID>/GGGG-MM-DD/<sesija-servisa>/`:

- manifest naredbe, vremena i izlaznog statusa;
- odvojeni segmenti senzora, odometrije, TF-a, planiranja, kontrole i lifecyclea;
- učitani parametri i njihove hash vrijednosti, planovi, ETA i promjene načina;
- snimke karte/costmapa pri planu, grešci ili otkazu;
- rotacija po 20 MiB, bez prepisivanja prethodnih dijelova;
- sažeci gubitaka zapisa i grešaka pisanja.

Telemetrija je uzorkovana dijagnostika, ne puni rosbag. Livox oblak bilježi se
kao metapodaci oblaka; skenovi, IMU i odometrija uzorkuju se kao poruke. Logger
ne pretvara neprovjerene IMU jedinice i ne objavljuje upravljačke naredbe.

```bash
bash robot.sh status
bash robot.sh logs telemetry
bash robot.sh ros controller ros2 lifecycle get /controller_server
bash robot.sh ros telemetry ros2 run ranger_logging read_logs.py \
  /logs/ID/GGGG-MM-DD/SESIJA --segment diagnostics --event health
bash robot.sh down
```

`check` samo čita ROS podatke. Provjerava senzore, odometrijske frameove, TF i
potrebne upravljačke zglobove. Ne potvrđuje fizikalne granice ni sigurnost vožnje.
Karte i logovi ostaju nakon gašenja. Za novi release promijeni tag u `.env`, pa
ponovi `pull` i odabrani način pokretanja.

Lokalno je provjeren amd64 build/start/planiranje i strukturirano logiranje.
Fizička navigacija, mapiranje povratne informacije kotača, izlaz prema pogonu,
Registry pull i stvarni robot zahtijevaju zasebnu provjeru.

Izvori:
- https://docs.docker.com/build/ci/github-actions/multi-platform/
- https://docs.github.com/en/actions/tutorials/publish-packages/publish-docker-images
