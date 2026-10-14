# Sterownik feedback okręgu

[Powrót do README](../README.md)

## Uruchomienie i źródło postury

```bash
ros2 launch mtracker feedback_circle.launch.py enable_motion:=true
```

Węzeł `controller_tracking` jest zaimplementowany w C++:
[`src/control/controller_tracking.cpp`](../src/control/controller_tracking.cpp). Algorytm i bufor
śladu znajdują się w [`include/mtracker/control/demo_control.hpp`](../include/mtracker/control/demo_control.hpp).
Węzeł `trajectory_demo` również jest napisany w C++. Pliki launch pozostają
standardowymi plikami launch ROS 2; nie wykonują obliczeń sterowania.

Kontroler odpytuje TF o posturę `base_link` w `reference_frame`, domyślnie `odom`.
Nie subskrybuje tematu odometrii. Dzięki temu źródłem postury może być odometria
kół albo zewnętrzna lokalizacja, pod warunkiem poprawnego drzewa TF.

```bash
ros2 launch mtracker feedback_circle.launch.py \
  start_driver:=false reference_frame:=map enable_motion:=true
```

Ten przykład zakłada działający sterownik i aktualną relację TF między `map`
a `base_link`. Jeśli źródłem `odom → base_link` pozostaje sterownik kół, feedback
koryguje błąd względem tej odometrii; nie usuwa błędu lokalizacji względem podłoża.
Ramka odniesienia powinna być nieruchoma. Skoki lokalizacji i reset odometrii
zmieniają znaczenie błędu, dlatego po takim resecie należy ponownie uruchomić demo.

## Trajektoria zadana

Pierwszy poprawny pomiar przy włączonym ruchu wyznacza posturę początkową
`(x0, y0, theta0)`. Dla promienia `r`, prędkości `vr` i `wr = vr/r`:

```text
theta_r(t) = theta0 + wr*t
x_r(t) = x0 + r*(sin(theta_r(t)) - sin(theta0))
y_r(t) = y0 - r*(cos(theta_r(t)) - cos(theta0))
```

Środek okręgu leży po lewej stronie robota:
`(x0 - r*sin(theta0), y0 + r*cos(theta0))`. Ruch jest przeciwny do ruchu
wskazówek zegara. Domyślne `r=0.1 m`, `vr=0.1 m/s` dają `wr=1 rad/s`
i okres około 6,28 s. Demo nie kończy się automatycznie po jednym obiegu.

## Prawo sterowania

Błąd postury jest wyrażony w układzie robota, a błąd kąta zawijany do `[-pi, pi]`:

```text
ex =  cos(theta)*(xr-x) + sin(theta)*(yr-y)
ey = -sin(theta)*(xr-x) + cos(theta)*(yr-y)
etheta = wrap(theta_r-theta)

v = vr*cos(etheta) + k_x*ex
w = wr + vr*(k_y*ey + k_theta*sin(etheta))
```

Jest to klasyczne nieliniowe śledzenie postury z rodziny regulatorów Kanayamy;
źródło koncepcji: [Kanayama i in., A Stable Tracking Control Method for an
Autonomous Mobile Robot](https://doi.org/10.1109/ROBOT.1990.126006).
Implementacja dodatkowo ogranicza moduł `v` i `w` niezależnie.

Prędkość **referencyjna** wynosi 10 cm/s, natomiast komenda może być większa,
mniejsza, a nawet ujemna, gdy wymaga tego korekcja błędu. Ograniczenia prędkości,
opóźnienia i dynamika napędu mogą wpływać na zbieżność. Nie ma tu estymatora
poślizgu, planowania przeszkód ani ogranicznika przyspieszenia.

## Parametry

Wszystkie są odczytywane przy starcie. Parametry liczbowe muszą być skończone
i dodatnie; `speed` oraz `speed/radius` nie mogą przekraczać limitów komend.

| Parametr / argument launch | Wartość domyślna | Znaczenie |
| --- | --- | --- |
| `radius` | `0.10` | Promień okręgu [m]. |
| `speed` | `0.10` | Prędkość trajektorii odniesienia [m/s]. |
| `k_x` | `2.0` | Wzmocnienie błędu wzdłużnego [1/s]. |
| `k_y` | `40.0` | Wzmocnienie błędu poprzecznego [1/m²]. |
| `k_theta` | `10.0` | Wzmocnienie błędu orientacji [1/m]. |
| `max_linear` | `0.20` | Limit modułu komendy liniowej [m/s]. |
| `max_angular` | `2.0` | Limit modułu komendy kątowej [rad/s]. |
| `tf_timeout` | `0.25` | Maksymalny wiek / czas zamrożenia TF [s]. |
| `enable_motion` | `false` | Zezwolenie na niezerowe komendy. |
| `reference_frame` | `odom` | Ramka odniesienia i Fixed Frame RViz. |
| `base_frame` | `base_link` | Parametr węzła; dostarczony launch ustawia `base_link`. |

Pozostałe argumenty launch: `start_driver=true`, `rviz=true`,
`config=<udział pakietu>/config/mtracker.yaml`.

Przykład jawnego ustawienia wzmocnień:

```bash
ros2 launch mtracker feedback_circle.launch.py enable_motion:=true \
  k_x:=2.0 k_y:=40.0 k_theta:=10.0 max_linear:=0.20 max_angular:=2.0
```

Przy strojeniu obserwuj błąd i komendy: `k_x` zwiększa korekcję wzdłużną,
`k_y` korekcję poprzeczną przez skręt, a `k_theta` reakcję na błąd orientacji.
Domyślne wartości przeszły test idealnego modelu kinematycznego; nie są
wynikiem strojenia na sprzęcie. Częste osiąganie limitu prędkości oznacza, że
zachowanie istotnie różni się od nieograniczonego prawa sterowania.

## Brak pomiarów i zatrzymanie

Timer sterowania działa co 20 ms według zegara monotonicznego. Odczyt TF jest
nieblokujący. Kontroler publikuje zera, gdy:

- ruch jest wyłączony lub nie ma transformu;
- postura zawiera wartości niefinitywne albo niepoprawny kwaternion;
- znacznik czasu jest zerowy, starszy niż `tf_timeout` lub ponad 50 ms w przyszłości;
- znacznik TF nie zmienia się przez `tf_timeout` czasu monotonicznego;
- wykryto cofnięcie czasu albo przerwę pętli dłuższą niż `tf_timeout`.

W czasie przerwy faza trajektorii jest zamrożona. Po powrocie poprawnego TF
sterownik kontynuuje ten sam okrąg od zatrzymanej fazy; nie ustala nowego środka.
Sprawdzenie czasu monotonicznego wykrywa także zatrzymanie zegara ROS.

`Ctrl+C` kończy proces. Sterownik sprzętowy ma osobny `command_timeout` i po
utracie aktualnych komend wysyła zero przy dostępnej komunikacji szeregowej.

## Wizualizacja

- `/mtracker/reference_path`: 201 próbek zamkniętego okręgu, pomarańczowa linia.
- `/mtracker/reference_pose`: bieżąca postura zadana, pomarańczowa strzałka.
- `/mtracker/trajectory`: ślad z tych samych transformów TF co feedback,
  zielona linia; odstęp próbek minimum 2 mm, maksymalnie 10000 punktów.

Ślad jest publikowany przy 10 Hz, bieżący cel przy 50 Hz po uzyskaniu poprawnego TF.
Zadany okrąg powstaje dopiero po zakotwiczeniu trajektorii przy włączonym ruchu.
Restart węzła czyści ślad i okrąg. Widok RViz domyślnie podąża za `base_link`;
aby zatrzymać kamerę względem świata, wybierz ramkę odniesienia jako Target Frame.

## Testy

[`test/test_demo_control.cpp`](../test/test_demo_control.cpp) używa GoogleTest.
Sprawdza ekstrakcję postury TF, geometrię okręgu, limity prędkości, obsługę braku,
zestarzenia i zamrożenia TF oraz zachowanie po skoku czasu. Test numeryczny
wprowadza zakłócenie pozycji 4 cm i orientacji 0,25 rad, a następnie weryfikuje
spadek błędu poniżej 5 mm i 0,03 rad w idealnym modelu robota różnicowego.
Testy obejmują również próbkowanie i ograniczenie długości śladu.

```bash
colcon test --packages-select mtracker --ctest-args -R test_demo_control
colcon test-result --test-result-base build/mtracker --verbose
```

Testy nie potwierdzają rzeczywistej dokładności, opóźnień lokalizacji ani
renderowania RViz.
