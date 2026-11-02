# Dijagnostički zapisi

`simulation.sh` automatski uključuje logger. Svako pokretanje dobiva zasebnu
sesiju u `ranger_navigation_startup/logs/GGGG-MM-DD/`, s datumom, vremenom,
ROS domenom, naredbom i jedinstvenim sufiksom u nazivu. Putanja se ispisuje
na početku. Postojeći pokrenuti procesi trebaju se ponovno pokrenuti da bi
koristili novi logger.

```bash
cd /home/bedathenic/bedenic_ws/src/ranger_navigation_startup
bash simulation.sh logs
ls -l logs/latest-*
```

Poveznica `latest-navigate-d87` pokazuje na posljednju navigacijsku sesiju
domene 87; `latest-slam-d87` na posljednji SLAM. Naredbe `keyboard`,
`initial_pose`, validacije, `build` i `test` imaju vlastite sesije. Telemetriju
vožnje prikuplja logger pokrenut uz simulaciju. Pokretanje samo tipkovnice
bilježi njezin izlaz, ali ne pokreće dodatni snimač telemetrije.

## Datoteke i rotacija

- `manifest.json`: naredba, domena, početak, završetak i izlazni status.
- `segments/<segment>__<datum-vrijeme>__<proces>__000001.jsonl`: strukturirani
  zapisi po segmentima i izvoru (launch, ROS telemetrija ili konzola).
- `*-summary.json`: broj zapisa, gubici, greške pisanja i stanje pražnjenja reda.
- `telemetry-*-metadata.json`: postavke uzorkovanja.

Granica je zadano **20 MiB po JSONL datoteci**. Nova datoteka otvara se prije
prelaska granice; stare se ne prepisuju niti automatski brišu. Veliki pojedinačni
zapisi dijele se u numerirane `record_chunk` dijelove s SHA-256 provjerom.
Čitač iz nastavka ponovno ih sastavlja. Za promjenu veličine, lokacije ili
uzorkovanja postavi varijable prije uobičajene naredbe za pokretanje:

```bash
export RANGER_LOG_MAX_BYTES=10485760  # 10 MiB; minimum 4096 bajtova
export RANGER_LOG_ROOT=/home/bedathenic/ranger_logs
export RANGER_LOG_SAMPLE_HZ=5
# Zatim pokreni svoju postojeću simulation.sh sim/slam/localize/navigate naredbu.
```

`RANGER_LOG_ENABLED=0` isključuje ovaj sustav i ostavlja standardno ROS
zapisivanje. Izravni `ros2 launch ranger_mini ...` također uključuje logger,
ali bez konzolnog omotača; zadana lokacija tada je `~/.ros/ranger_logs`.

## Što se bilježi

| Segment | Podaci |
|---|---|
| `sensors` | LiDAR skenovi, IMU, kovarijance i vremenske oznake |
| `base_control` | Stanja kotača i upravljanja, zadane pozicije i brzine zglobova, poruke upravljača |
| `odometry` | Enkoderska, EKF i referentna Gazebo odometrija, udaljenost, brzina, kut i rezidual kinematike |
| `tf` | Statičke i uzorkovane dinamičke transformacije |
| `slam`, `localization` | SLAM/AMCL poruke, karta, procjena i početna poza s kovarijancom |
| `planning` | Globalni plan, odluka o ruti/ETA, globalni costmap i footprint |
| `control` | `cmd_vel` prije/poslije zaglađivanja gdje postoji, lokalni plan/costmap, promjene načina vožnje i njihovi razlozi |
| `navigation` | Statusi i UUID navigacijskih akcija, feedback i promjene stanja behavior treeja |
| `lifecycle` | Konzolne poruke, otkriveni čvorovi i lifecycle prijelazi nakon priključenja loggera |
| `configuration` | Argumenti pokretanja, verzije ROS paketa/Pythona/RMW-a, odabrane ulazne tekstualne datoteke s hashom, učitani parametri i promjene parametara |
| `simulation`, `system`, `console` | Gazebo/RViz i ostali procesi, pokretanje/gašenje, izlazni status, stdout/stderr |
| `diagnostics` | Učestalost i starost primljenih poruka, nedostajući očekivani topici, TF roditelji/publisheri, slobodan disk i gubici loggera |

Zapisi nose UTC vrijeme, lokalno vrijeme Europe/Zagreb, monotono vrijeme
i ROS vrijeme gdje je dostupno. Izvorna poruka zadržava vlastiti timestamp.
ROS poruke bilježe se na razini koju emitira čvor (zadano INFO i više).
Sadržaj konzole i `/rosout` namjerno se može preklapati: konzola pokriva i
rušenja prije nego što čvor počne objavljivati ROS poruke.

Telemetrija je dijagnostički uzorak, ne potpuni rosbag: zadano 5 Hz za
odometriju/IMU/zglobove, 10 Hz za brzinske naredbe, 1 Hz za LiDAR i lokalni
plan. Događaji, globalni planovi i ROS poruke nemaju vremensko prorjeđivanje.
Zdravlje sustava provjerava se svake dvije sekunde stvarnog vremena,
uključujući dok je Gazebo pauziran.

Pune komprimirane snimke karte i costmapa spremaju se svakih 30 sekundi,
pri novom globalnom planu, ROS ERROR/FATAL poruci, BT neuspjehu i
otkazanoj/neuspjeloj navigacijskoj akciji. Razmak snimki je najmanje jednu
sekundu; okidači unutar tog razmaka ostaju zapisani kao događaji. Zapis
`grid_snapshot` sadrži metapodatke, starost, SHA-256 i `zlib/base64` podatke
ćelija. Karta koristi `int8`, a costmap `uint8`. Konfiguracija ovog projekta
objavljuje pune costmape; za drugi sustav koji objavljuje samo inkrementalne
izmjene ovaj logger ne rekonstruira `costmap_updates`.

Logger samo čita topice i parametre; ne šalje upravljačke naredbe. Parametri
se dohvaćaju postupno, najviše dva čvora istodobno. TF publisheri se zapisuju
na razini topica: Humble Python API ne daje identitet publishera uz pojedinu
transformaciju, pa to nije dokaz koji čvor objavljuje određeni TF okvir.
`conflicting_tf_parents` označava viđene različite roditelje istog okvira.
Akcijski status/feedback ne sadrži uvijek izvorni zahtjev cilja; uz njega su
dostupni `/goal_pose` kada se objavljuje te cilj uspješno generiranog plana.

Pisanje ide kroz ograničen red u zasebnoj dretvi. Pri punom redu ili grešci
diska mogu se izgubiti zapisi: brojači su u `health`, `log_loss` i završnom
sažetku, a greška diska ispisuje se i na stderr. Naglo gašenje/SIGKILL može
ostaviti nepotpun zadnji zapis ili snimku; čitač to prijavljuje. Nema jamstva
trajnosti pri nestanku napajanja i nema automatskog brisanja starih sesija.

Standardni rastući ROS disk-log isključen je za čvorove ovih launch datoteka;
njihove poruke preuzima rotirajući sustav. Mali početni `launch.log` zatvara
se pri učitavanju našeg launcha. Postojeći logovi, vanjski čvorovi pokrenuti
drugim naredbama i izvorni colcon build artefakti nisu pod ovom rotacijom.

## Pregled i slanje za debugging

```bash
cd /home/bedathenic/bedenic_ws/src/ranger_navigation_startup
python3 ../ugv_sim/ranger/ranger_logging/scripts/read_logs.py \
  logs/latest-navigate-d87 --segment control --event ros_log

python3 ../ugv_sim/ranger/ranger_logging/scripts/read_logs.py \
  logs/latest-navigate-d87 --level ERROR

python3 ../ugv_sim/ranger/ranger_logging/scripts/read_logs.py \
  logs/latest-navigate-d87 --segment diagnostics --event health
```

Čitač prolazi datoteke po nazivu; za vremensko povezivanje različitih izvora
koristi `wall_time_utc`/`monotonic_ns`. Za analizu pošalji cijeli direktorij
sesije, uključujući sve dijelove `segments`, manifest i sažetke. Log sadrži
karte, skenove, putanje i parametre robota.

Testovi pohrane, rotacije, rekonstrukcije velikih zapisa, greške diska,
punog reda i prosljeđivanja signala uključeni su u `bash simulation.sh test`.

Provjera 2026-11-01: 35 Python testova prošlo je (11 za logiranje), kao i
izolirana Gazebo/Nav2 vožnja, bilježenje lifecycle događaja, snimka na
namjerno označenu testnu grešku, detekcija pauziranog sata i nastavak rada.
U sesiji je nastalo 144 rotiranih JSONL datoteka, svaka najviše 65.536
bajtova, bez prijavljenih gubitaka. Posebno je provjereno uredno gašenje
loggera pri ponovljenom SIGINT-u. Detalji:
[logging-summary-20261101.json](validation/logging-summary-20261101.json).
