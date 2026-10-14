# MTracker — sterownik i demonstracje ROS 2

Pakiet `mtracker` łączy robota różnicowego MTracker z ROS 2 przez port szeregowy.
Odbiera polecenia prędkości, publikuje odometrię, TF, telemetrię zasilania i diagnostykę.
Zawiera również demonstracje ruchu oraz sterownik śledzenia okręgu ze sprzężeniem
zwrotnym z postury `base_link` odczytywanej z TF.

Dokumentacja opisuje aktualną implementację w tym repozytorium:

- [Parametry, tematy i ramki TF](doc/interfaces.md)
- [Sterownik feedback: algorytm, strojenie i ograniczenia](doc/controller_tracking.md)
- [Protokół szeregowy i organizacja kodu](doc/protocol.md)

## Budowanie

Środowisko używane do weryfikacji: ROS 2 Humble, C++17, Python 3 i `colcon`.
Zależności są zadeklarowane w [package.xml](package.xml); obejmują m.in.
`rclcpp`, `geometry_msgs`, `nav_msgs`, `diagnostic_msgs`, TF2 i RViz2.
Polecenia wykonuj z katalogu głównego workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select mtracker --symlink-install
source install/setup.bash
```

W każdym nowym terminalu wczytaj `install/setup.bash`.

### Opcjonalny czujnik BH1750

Obsługa światła jest domyślnie wyłączona. Włącz definicję
`LIGHT_INTENSITY_BH1750` dla całego pakietu przez CMake:

```bash
colcon build --packages-select mtracker --symlink-install --cmake-args -DLIGHT_INTENSITY_BH1750=ON
source install/setup.bash
```

To odpowiada `#define LIGHT_INTENSITY_BH1750` we wszystkich jednostkach kompilacji.
Definicja w firmware jest niezależna od definicji w ROS; trzeba włączyć obie.
Sterownik odczytuje odpowiedź komendy 6 z `LEN=30` i publikuje `light_intensity`
(`std_msgs/msg/Float32`, lx) oraz `light_intensity_valid` (`std_msgs/msg/Bool`).
Nieprawidłowy pomiar jest publikowany jako NaN; poprawne zero oznacza 0 lx.
Starsze odpowiedzi `LEN=27` i komenda 3 nadal działają, ale nie dostarczają światła.
Wyłączenie przez `-DLIGHT_INTENSITY_BH1750=OFF` przywraca dotychczasowy parser
i usuwa oba tematy światła. Aktualne firmware wysyła `LEN=30` również przy
wyłączonym czujniku (wartość niedostępna), więc wymaga opcji ON po stronie ROS.

## Uruchomienie sterownika

Sprawdź ścieżkę portu w [config/mtracker.yaml](config/mtracker.yaml). Plik zawiera
lokalną ścieżkę `/dev/serial/by-id/usb-FTDI_Single_RS232-HS-if00-port0`, rozstaw
kół 0,15 m i promień koła 0,025 m. Dostosuj te wartości do robota.

```bash
ros2 launch mtracker mtracker_master.launch.py
```

Ten launch uruchamia sterownik i statyczne transformacje testowe/czujników;
nie uruchamia kontrolera ruchu ani RViz. Zobacz [opis TF](doc/interfaces.md#ramki-tf),
zwłaszcza przed podłączeniem zewnętrznej lokalizacji.

Można uruchomić sam sterownik z wybranym portem:

```bash
ros2 run mtracker mtracker_drv --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p baud_rate:=921600
```

Bez pliku YAML lub nadpisania parametrów urządzeniem domyślnym jest
`/dev/ttyACM0`. Nie uruchamiaj dwóch sterowników korzystających z tego samego portu.

## Demo okręgu ze sprzężeniem zwrotnym

```bash
ros2 launch mtracker feedback_circle.launch.py enable_motion:=true
```

Uruchamia sterownik robota, kontroler TF i RViz. Zadany promień to **10 cm**,
a prędkość trajektorii odniesienia to **10 cm/s**. Kontroler odczytuje
`odom → base_link` i oblicza `cmd_vel` z błędów położenia oraz orientacji.
Okrąg zaczyna się w początkowej posturze robota, stycznie do jego kierunku jazdy,
i biegnie przeciwnie do ruchu wskazówek zegara. Demo działa do `Ctrl+C`.

RViz pokazuje zielony ślad postury z TF, pomarańczowy okrąg zadany oraz
pomarańczową strzałkę bieżącej postury odniesienia. Siatka ma oczka 10 cm.

Przykład ze sterownikiem uruchomionym osobno:

```bash
ros2 launch mtracker feedback_circle.launch.py \
  start_driver:=false enable_motion:=true radius:=0.10 speed:=0.10
```

Jeżeli posturę zapewnia lokalizacja w `map`, dodaj `reference_frame:=map`.
Musi istnieć aktualny łańcuch TF `map → … → base_link`. Wybranie `map` samo
nie uruchamia lokalizacji. Nie publikuj konkurencyjnych transformacji dla tej
samej ramki potomnej.

Bez `enable_motion:=true` demo publikuje zerowe prędkości. Brak, niepoprawność
lub zestarzenie TF również powoduje zadanie zera i wstrzymanie czasu trajektorii.
Parametry oraz zachowanie po powrocie TF opisano w
[dokumentacji kontrolera](doc/controller_tracking.md).

## Demo okręgu bez sprzężenia zwrotnego

```bash
ros2 launch mtracker circle_demo.launch.py enable_motion:=true
```

Publikuje stałe `v = 0.10 m/s`, `w = 1.0 rad/s`. Teoretyczny promień wynosi
`v/w = 0.10 m`, a czas obiegu około 6,28 s. Nie koryguje błędu toru.
RViz pokazuje ślad z `/wheel/odom`; zadany okrąg i punkt odniesienia są
publikowane tylko przez demo feedback.

W obu demach dostępne są argumenty `radius`, `speed`, `start_driver`, `rviz`,
`enable_motion` i `config`. Przykład pracy bez GUI z własną konfiguracją:

```bash
ros2 launch mtracker circle_demo.launch.py \
  config:=/pełna/ścieżka/mtracker.yaml rviz:=false enable_motion:=true
```

Argumenty `radius` i `speed` podaje się w metrach i metrach na sekundę.
Launch wymusza dla swojego sterownika `/wheel/odom` oraz ramki `odom`, `base_link`.

## Prosty kontroler stałych prędkości

```bash
ros2 launch mtracker controller_simple.launch.py \
  enable_motion:=true linear_velocity:=0.05 angular_velocity:=0.2
```

Uruchamia tylko węzeł `controller_simple`; sterownik robota musi już działać.
Parametry to prędkość liniowa w m/s i kątowa w rad/s. Publikacja odbywa się
co 20 ms. Gdy ruch jest wyłączony, węzeł nadal publikuje zera.

Przed uruchomieniem dowolnego dema zatrzymaj inne węzły publikujące `cmd_vel`.
Kontrolery nie mają arbitrażu poleceń. Parametry ruchu są odczytywane na starcie;
po ich zmianie uruchom węzeł ponownie.

## Ustawienie odometrii

Po zatrzymaniu kontrolera ruchu wyślij pojedynczą posturę:

```bash
ros2 topic pub --once /set_odometry geometry_msgs/msg/Pose2D \
  '{x: 1.0, y: 2.0, theta: 0.5}'
```

Alternatywnie:

```bash
ros2 run mtracker set_odometry_demo --ros-args \
  -p x:=1.0 -p y:=2.0 -p theta:=0.5
```

Jednostki to metry i radiany. Węzeł demo publikuje raz po około 0,5 s i kończy
pracę po kolejnym wywołaniu timera. Sterownik wysyła ustawienie w następnym
możliwym cyklu komunikacji; nie potwierdza aplikacji ustawienia osobną usługą.
Po resecie postury uruchom ponownie demo feedback, aby wyznaczyć nowy okrąg.

## Diagnostyka

```bash
ros2 topic echo /diagnostics
ros2 topic hz /wheel/odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic info /cmd_vel --verbose
```

| Objaw | Co sprawdzić |
| --- | --- |
| `Cannot open …` | Ścieżkę urządzenia, podłączenie adaptera i uprawnienia użytkownika do portu. |
| `no confirmed response` | Zasilanie kontrolera, `baud_rate`, `robot_address`; liczniki CRC i timeoutów w `/diagnostics`. |
| Robot nie rusza w demo | `enable_motion:=true`, potwierdzoną komunikację i brak innych wydawców `cmd_vel`. |
| Feedback publikuje zera | Dostępność i znaczniki czasu TF wybranej ramki względem `base_link`; domyślny limit wieku to 0,25 s. |
| Brak śladu w RViz | W open-loop sprawdź `/wheel/odom`, w feedback sprawdź TF; następnie `/mtracker/trajectory` i Fixed Frame w RViz. |
| Ślad różni się od rzeczywistego toru | Kalibrację `wheel_base`, `wheel_radius`, poślizg i źródło lokalizacji. TF z odometrii kół nadal jest odometrią kół. |
| `launch … not found` | Ponowny build pakietu i wczytanie `install/setup.bash`. |

Sterownik próbuje ponownie otworzyć port co około sekundę. Odometrię i TF
publikuje po odebraniu poprawnej telemetrii; zadana częstotliwość komunikacji
nie gwarantuje identycznej częstotliwości odpowiedzi robota.

## Testy i ograniczenia weryfikacji

```bash
colcon test --packages-select mtracker
colcon test-result --test-result-base build/mtracker --verbose
```

Zakres związany z demami i interfejsami można uruchomić osobno:

```bash
colcon test --packages-select mtracker --ctest-args \
  -R 'test_protocol|test_circle_demo|test_demo_control|flake8|pep257|lint_cmake|xmllint'
```

Testy obejmują ramki protokołu, próbkowanie śladu, parametry launch, odczyt
postury z TF, ograniczanie komend, utratę i zamrożenie TF oraz numeryczne
śledzenie okręgu po zakłóceniu. Nie wymagają fizycznego robota. W środowisku
z katalogiem domowym tylko do odczytu ustaw np. `ROS_LOG_DIR=/tmp/mtracker-tests`.

Build i wskazany zakres testów są sprawdzane lokalnie. Test numeryczny nie
potwierdza dokładności rzeczywistego robota. Renderowanie RViz i jazda na
sprzęcie wymagają oddzielnej weryfikacji; nie zostały tu potwierdzone.

## Protokół 0x06 i telemetria zasilania

Domyślnie `command_type=6`: żądanie ma `LEN=20` z obowiązkowym
`RESERVED:uint16=0`, odpowiedź `LEN=27` z polem `POWER[5]`.
Konfiguracja dla obecnego FTDI/RS232 zachowuje maskę `robot_address=256`
(`0x0100`). Nie jest to nagłówek ROBOT/SEQ mostu USB CDC opisanego w specyfikacji;
nie należy zamieniać tych dwóch sposobów adresowania.

Tematy zasilania:

| Temat | Typ | Znaczenie |
| --- | --- | --- |
| `voltage` | `std_msgs/msg/Float32` | Napięcie [V]. |
| `current` | `std_msgs/msg/Float32` | Prąd [A], ze znakiem. |
| `power` | `std_msgs/msg/Float32` | Moc [W]. |
| `power_valid` | `std_msgs/msg/Bool` | Ważność prądu i mocy według bitu 0 flag. |
| `power_flags` | `std_msgs/msg/UInt8` | Pełny zestaw czterech flag firmware. |

Jeśli bit ważności jest wyzerowany, `current` i `power` zawierają **NaN**,
nie zero sugerujące poprawny pomiar. Napięcie nadal może pochodzić z ADC.
Flagi: bit 0 — ważny prąd/moc, bit 1 — napięcie z INA219, bit 2 — INA219
skonfigurowany, bit 3 — obsługa monitora w firmware.

Test bez ruchu, po zatrzymaniu innych instancji sterownika:

```bash
ros2 run mtracker mtracker_drv --ros-args \
  --params-file src/mtracker/config/mtracker.yaml -p enable_drive:=false
```

Obserwacja w drugim terminalu:

```bash
ros2 topic echo /voltage --once
ros2 topic echo /power_valid --once
ros2 topic echo /power_flags --once
```

Opcjonalne `command_type:=3` wybiera wcześniejszy format float32 i napięcie
`SUPPLY_MV/1000`. Ten wariant nie dostarcza prądu i mocy, więc zgłasza
`power_valid=false`. Oba warianty zachowują tematy odometrii i TF.

Aktualny test robota potwierdził odpowiedzi `0x06`, `LEN=27`, poprawne CRC
i napięcie około 16,48 V. Flagi `0x08` oznaczały brak ważnego pomiaru prądu/mocy.
Nie uruchamiano napędu ani nie zmieniano konfiguracji regulatora lub Flash.
