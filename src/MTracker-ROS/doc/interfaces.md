# Interfejsy i parametry MTracker

[Powrót do README](../README.md)

## Węzły i launch

| Plik wykonywalny | Nazwa węzła | Zadanie |
| --- | --- | --- |
| `mtracker_drv` | `mtracker` | Port szeregowy, kinematyka różnicowa, odometria, TF, diagnostyka. |
| `controller_simple` | `mtracker_controller_simple` | Stała prędkość `Twist` przy 50 Hz. |
| `controller_tracking` | `mtracker_controller_tracking` | Obliczanie prędkości ze sprzężeniem z TF przy 50 Hz. |
| `trajectory_demo` | `mtracker_trajectory` | Ślad z odometrii, publikacja `Path` przy 10 Hz. |
| `set_odometry_demo` | `set_odometry_demo` | Jednorazowe ustawienie postury. |

| Launch | Uruchamiane elementy |
| --- | --- |
| `mtracker_master.launch.py` | Sterownik i statyczne TF testowe/czujników. |
| `controller_simple.launch.py` | Tylko stały kontroler. |
| `circle_demo.launch.py` | Sterownik, stały kontroler, ślad odometrii, RViz. |
| `feedback_circle.launch.py` | Sterownik, kontroler TF ze śladem i trajektorią zadaną, RViz. |

Dwa launche okręgu umożliwiają wyłączenie sterownika przez `start_driver:=false`
i GUI przez `rviz:=false`. Ruch zawsze wymaga `enable_motion:=true`.
Listę argumentów można wyświetlić bez uruchamiania węzłów:

```bash
ros2 launch mtracker feedback_circle.launch.py --show-args
```

## Tematy

Nazwy bez początkowego `/` są względne względem przestrzeni nazw węzła.
Poniższe pełne ścieżki dem zakładają uruchomienie dostarczonych launchy.

| Temat | Typ | Kierunek / znaczenie |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Wejście sterownika; używane `linear.x` i `angular.z`. Wyjście kontrolerów. |
| `set_odometry` | `geometry_msgs/msg/Pose2D` | Wejście sterownika; x, y [m], theta [rad]. |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | Domyślne wyjście odometrii; nazwa konfigurowalna. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Stan `mtracker/serial` i liczniki, co 1 s. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamiczna postura `odom → base_link`. |
| `/mtracker/trajectory` | `nav_msgs/msg/Path` | Ślad pomiarowy; odometria w open-loop, TF w feedback. |
| `/mtracker/reference_path` | `nav_msgs/msg/Path` | Zadany okrąg, tylko feedback. |
| `/mtracker/reference_pose` | `geometry_msgs/msg/PoseStamped` | Bieżący cel na okręgu, tylko feedback. |

Sterownik publikuje `voltage`, `current`, `power` (`std_msgs/msg/Float32`,
jednostki V, A, W), `power_valid` (`std_msgs/msg/Bool`) oraz `power_flags`
(`std_msgs/msg/UInt8`). Przy nieważnym pomiarze `current` i `power` mają wartość
NaN, a `power_valid=false`; zero z firmware nie jest traktowane jako pomiar. Zwykłe tematy sterownika
mają kolejkę 10 (ustawienie odometrii: 1). Ślady i zadany okrąg używają
`Reliable`, `Transient Local`, głębokość 1, aby RViz otrzymał ostatnią ścieżkę
również po późniejszym uruchomieniu. Surowe nazwy tematów węzła feedback to
`trajectory`, `reference_path`, `reference_pose`; launch remapuje je na powyższe.

## Parametry sterownika

Przy definicji `LIGHT_INTENSITY_BH1750` dostępne są dodatkowe względne tematy
`light_intensity` (`std_msgs/msg/Float32`, lx, NaN przy braku pomiaru) oraz
`light_intensity_valid` (`std_msgs/msg/Bool`). Komenda 3 i starsze odpowiedzi
komendy 6 publikują światło jako nieważne. Bez definicji te tematy nie istnieją.

Domyślne wartości w tabeli pochodzą z kodu. Plik
[`mtracker.yaml`](../config/mtracker.yaml) może je nadpisać.

| Parametr | Domyślnie w kodzie | Znaczenie |
| --- | --- | --- |
| `device` | `/dev/ttyACM0` | Urządzenie szeregowe. |
| `serial_port` | pusty tekst | Alias; niepusta wartość ma pierwszeństwo nad `device`. |
| `baud_rate` | `921600` | Obsługiwane: 115200, 460800, 921600; format 8N1, bez sprzętowego sterowania przepływem. |
| `robot_address` | `256` (`0x0100`) | Niezerowa 16-bitowa maska adresowa zgodna z poprzednim sterownikiem. |
| `command_type` | `6` | Komenda cykliczna: `6` (kompaktowa z POWER) lub `3` (legacy, tylko napięcie). |
| `enable_drive` | `true` | `false` wyłącza nadawanie komend załączających napęd; tryb diagnostyczny. |
| `response_timeout_ms` | `50` | Czas oczekiwania na odpowiedź kontrolera. |
| `command_rate_hz` | `100.0` | Częstotliwość timera komunikacji; kolejna komenda czeka na odpowiedź lub timeout. |
| `wheel_base` | `0.15` | Rozstaw kół [m]. |
| `wheel_radius` | `0.025` | Promień koła [m]. |
| `command_timeout` | `0.5` | Maksymalny wiek polecenia `cmd_vel` [s]. |
| `odometry_topic` | `/wheel/odom` | Temat publikacji odometrii. |
| `odom_frame` | `odom` | Domyślna ramka nadrzędna odometrii. |
| `odom_frame_id` | wartość `odom_frame` | Nadpisuje `odom_frame`, jeśli podano obie wartości. |
| `base_frame` | `base_link` | Domyślna ramka robota. |
| `frame_id` | wartość `base_frame` | Nadpisuje `base_frame`, jeśli podano obie wartości. |
| `pose_sigma_x`, `pose_sigma_y` | `0.01` | Odchylenia standardowe pozycji [m]. |
| `pose_sigma_theta` | `0.02` | Odchylenie standardowe yaw [rad]. |
| `twist_sigma_linear` | `0.02` | Odchylenie standardowe prędkości liniowej [m/s]. |
| `twist_sigma_angular` | `0.03` | Odchylenie standardowe prędkości kątowej [rad/s]. |

Parametry są odczytywane podczas tworzenia węzła, bez obsługi zmiany w locie.
`wheel_base` i `wheel_radius` muszą odpowiadać geometrii robota i być dodatnie.
W macierzach kowariancji sterownik zapisuje kwadraty podanych odchyleń;
nieobserwowane stopnie swobody otrzymują wariancję `1e6`.

W lokalnym YAML pozostały `odometry_command_repetitions` i
`odometry_command_delay_ms`. **Aktualny kod ich nie odczytuje** i nie realizuje
powtórzeń ustawiania odometrii. Nie należy traktować tych wpisów jako działających
opcji. Ustawienie odometrii jest pojedynczą oczekującą komendą; nowa postura może
zastąpić poprzednią, jeśli nie została jeszcze wysłana.

## Ramki TF

Sam sterownik publikuje tylko dynamiczny transform:

```text
odom ──> base_link
```

Nazwy są konfigurowalne powyższymi parametrami. Postura pochodzi z telemetrii
robota; sterownik nie integruje ponownie pozycji z prędkości kół.

`mtracker_master.launch.py` dodatkowo publikuje stałe transformacje:

| Rodzic → dziecko | Translacja x, y, z [m] | Obrót |
| --- | --- | --- |
| `map → odom` | 0, 0, 0 | Identyczność; transform testowy. |
| `map → base_link_camera` | 1, −1, 0 | Yaw 0,7 rad; wartość testowa. |
| `base_link → laser` | 0,01, 0, 0 | Identyczność. |
| `base_link → base_footprint` | 0, 0, 0 | Identyczność. |

Launche okręgu nie dodają tych statycznych TF. Przy lokalizacji publikującej
`map → odom` nie uruchamiaj równolegle testowej transformacji o tej samej relacji.
Do dema z lokalizacją można użyć `feedback_circle.launch.py`, które uruchamia
sam sterownik bez testowych transformacji.

## Komunikacja i zatrzymanie

Sterownik przelicza `v`, `w` na prędkości kątowe kół:

```text
left  = (v - wheel_base * w / 2) / wheel_radius
right = (v + wheel_base * w / 2) / wheel_radius
```

Komendy napędu wysyła dopiero po otrzymaniu poprawnej telemetrii i przy aktualnym
`cmd_vel`. Po przekroczeniu `command_timeout` lub utracie potwierdzenia komunikacji
wysyła komendę zerową, o ile transmisja jest dostępna. Po utracie fizycznego łącza
nie może dostarczyć komendy zatrzymania do robota; zachowanie napędu zależy wtedy
również od firmware. Kontroler feedback ma dodatkowo własny limit wieku TF.

Diagnostyka zawiera `valid_frames`, `crc_errors`, `format_errors`,
`incomplete_frame_timeouts`, `response_timeouts`, `disconnects`, `reconnections`.
Stan to `OK` przy potwierdzonej komunikacji, w przeciwnym razie `WARN`.

## Pozostałe parametry węzłów

- `controller_simple`: `enable_motion=false`, `linear_velocity=0.05`,
  `angular_velocity=0.2`.
- `set_odometry_demo`: `x=0.0`, `y=0.0`, `theta=0.0`.
- `trajectory_demo`: `max_points=10000`, `min_distance=0.002` m.
  Wejście `odom` jest remapowane w launch na `/wheel/odom`; wyjście
  `trajectory` na `/mtracker/trajectory`. Zmiana ramki lub cofnięcie czasu
  odometrii czyści ślad.
- `controller_tracking`: [osobna tabela i opis](controller_tracking.md).
