# Kod i protokół szeregowy

[Powrót do README](../README.md)

Opis poniżej odzwierciedla obecną implementację pakietu. Nie zastępuje
specyfikacji firmware robota i nie jest potwierdzeniem zgodności ze wszystkimi
wersjami kontrolera MTracker.

## Organizacja kodu

Kod jest podzielony według odpowiedzialności: `communication/` zawiera
protokół i transport szeregowy niezależne od ROS, `control/` kontrolery ruchu,
a `nodes/` węzły sterownika sprzętowego i pomocnicze. Nagłówki publiczne
w `include/mtracker/` stosują ten sam podział. Nazwy programów i polecenia
uruchamiania nie zależą od położenia plików źródłowych.

| Plik | Odpowiedzialność |
| --- | --- |
| `src/nodes/mtracker.cpp` | Węzeł sterownika, kinematyka, stan komunikacji, telemetria ROS. |
| `src/communication/protocol.cpp`, `include/mtracker/communication/protocol.hpp` | Kodowanie ramek, CRC, parser i dekodowanie telemetrii. |
| `src/communication/serial_port.cpp`, `include/mtracker/communication/serial_port.hpp` | RAII portu POSIX, konfiguracja 8N1 i operacje odczytu/zapisu. |
| `src/control/controller_simple.cpp` | Stałe komendy prędkości. |
| `src/control/controller_tracking.cpp` | Kontroler TF i publikacja trajektorii referencyjnej. |
| `src/nodes/trajectory_demo.cpp` | Osobny węzeł śladu odometrii dla dema bez feedback. |
| `include/mtracker/control/demo_control.hpp` | Obliczenia feedback, ekstrakcja postury i bufor śladu. |
| `src/nodes/set_odometry_demo.cpp` | Jednorazowe polecenie ustawienia postury. |
| `launch/`, `config/` | Uruchamianie węzłów, YAML sterownika i konfiguracja RViz. |
| `test/` | GoogleTest dla C++, testy konfiguracji launch w pytest. |

Węzły sterujące i wizualizujące trajektorię są napisane w C++. Biblioteka
`mtracker_core` zawiera protokół i obsługę portu. Pliki `build/`, `install/`,
`log/` są generowane przez `colcon` i nie są źródłami pakietu.

## Format ramki

```text
0xAA | length | group | command | robot_low | robot_high | data... | CRC_hi | CRC_lo
```

`length` wynosi `4 + liczba bajtów data`; cała ramka ma `length + 4` bajty.
Parser akceptuje długości 4–31. Adres jest **16-bitową maską**, przesyłaną
młodszym bajtem najpierw, a nie parą „identyfikator + numer sekwencji”.
Domyślny adres komendy to `0x0100` (parametr `robot_address=256`), grupa 1.
Robot podłączony przez RS232 zwraca adres `0x0000`; zostało to potwierdzone
odczytem rzeczywistych odpowiedzi. Parser akceptuje adres zerowy lub równy
masce skonfigurowanej dla połączenia. Inne niezerowe adresy odrzuca.

CRC16 obejmuje bajty od `length` do końca danych: inicjalizacja `0xFFFF`,
wielomian `0x8005`, bez odbicia i końcowego XOR. Starszy bajt CRC jest pierwszy.
Parser ma osobny limit 10 ms na niekompletną ramkę, odrębny od czasu odpowiedzi.

## Komenda 0x06 — format kompaktowy

Domyślne `command_type=6`. Żądanie ma 16 bajtów DATA, `LEN=20`, łącznie 24 bajty.

| Offset DATA | Rozmiar | Pole / kodowanie |
| --- | --- | --- |
| 0 | 2 | STATUS uint16: 0 = wyłączenie, 3 = napęd, 4 = ustawienie odometrii. |
| 2, 4 | po 2 | WL, WR: int16 Q8 (rad/s × 256). |
| 6, 9 | po 3 | X, Y: signed int24 Q15 (metry × 32768). |
| 12 | 2 | TH: signed int16 Q10 (radiany × 1024). |
| 14 | 2 | RESERVED uint16, zawsze zero. |

Odpowiedź ma 23 bajty DATA, `LEN=27`, łącznie 31 bajtów.

| Offset DATA | Rozmiar | Pole / dekodowanie |
| --- | --- | --- |
| 0, 2 | po 2 | WL, WR: signed int16 / 256. |
| 4, 7 | po 3 | X, Y: signed int24 / 32768. |
| 10 | 2 | TH: signed int16 / 1024. |
| 12, 14 | po 2 | E_PHI_L, E_PHI_R: signed int16 / 1024. |
| 16, 17 | po 1 | UL, UR: signed int8 / 127. |
| 18 | 5 | POWER — upakowane napięcie, prąd, moc i flagi. |

Dla pięciu bajtów `b0..b4` pola są dekodowane następująco:

```text
Vraw = (b0 << 4) | (b1 >> 4)
Iraw = ((b1 & 15) << 8) | b2
jeśli Iraw & 0x800: Iraw -= 4096
Praw = (b3 << 4) | (b4 >> 4)
FLAGS = b4 & 15
V = Vraw * 0.01 V
I = Iraw * 0.002 A
P = Praw * 0.025 W
```

Bit 0 flag określa ważność prądu i mocy, bit 1 źródło napięcia INA219,
bit 2 poprawną konfigurację INA219, bit 3 włączoną obsługę monitora.
Węzeł publikuje V/A/W na `voltage`, `current`, `power` oraz flagi na
`power_valid` i `power_flags`. Przy nieważnym pomiarze prąd i moc to NaN.
Błędy regulatorów i sygnały UL/UR są dekodowane, ale nie mają osobnych tematów.

## Rozszerzenie BH1750 komendy 0x06

Przy zdefiniowanym `LIGHT_INTENSITY_BH1750` parser przyjmuje także `LEN=30`
(26 bajtów DATA). Bajty DATA 23..25 zawierają unsigned uint24 little-endian
w jednostkach 0,01 lx; dzielenie przez 100 daje lx. `0xFFFFFF` oznacza
pomiar niedostępny i jest publikowane jako NaN z ważnością false. Zero jest
poprawnym pomiarem. Offsety odometrii i zasilania pozostają bez zmian.
Bez definicji obowiązuje dotychczasowe `LEN=27`; rozszerzona telemetria jest
odrzucana. Żądanie komendy 6 i jego CRC nie zmieniają się.

## Komenda 0x03 — zgodność legacy

Opcjonalne `command_type=3` zachowuje żądanie `LEN=22`:
`STATUS:uint16 | WL:int16 | WR:int16 | X:float | Y:float | TH:float`.
Odpowiedź `LEN=26` dodaje `SUPPLY_MV:uint16 | UR:int16` po posturze.
Napięcie to `SUPPLY_MV/1000` V, UR to Q12. Prąd i moc nie są dostępne.
Wcześniejsza interpretacja pola SUPPLY_MV jako lewego sygnału sterowania
pochodziła ze starego kodu i została poprawiona według dokumentacji.

## Specyfikacja i weryfikacja

Źródło formatów danych: [przekazana dokumentacja COMMANDS.md](COMMANDS.md).
Opis nagłówka USB CDC (ROBOT/SEQ) nie zastępuje adresowania FTDI/RS232:
bieżący transport używa sprawdzonej maski `0x0100`. Nie zaimplementowano tutaj
obsługi mostu CDC ani komend konfiguracji 4, 7 i zapisu Flash 8.

Testy obejmują rzeczywistą ramkę `0x06`, CRC, wymagane RESERVED, signed int24,
ujemny prąd, granice 12-bitowe, flagi ważności, fragmentację i kilka ramek
w buforze. Ramki `0x06` o innych długościach są odrzucane bez zgadywania pól.
Sterownik akceptuje telemetrię tylko dla wybranego `command_type`.

Ponowiony test na robocie potwierdził `LEN=27`, 23 bajty DATA, poprawne CRC
oraz około 16,48 V przy flagach `0x08` (prąd/moc nieważne). Wcześniejsze próby
bez RESERVED i wcześniejsza odpowiedź LEN=25 nie opisują aktualnej ramki.

Po czystej kompilacji sprawdzono pełny węzeł ROS z `command_type=6` i
`enable_drive=false`: diagnostyka `communicating`, 249 poprawnych ramek,
zero błędów CRC, formatu i timeoutów. Odczyt `/voltage` wynosił około 16,36 V,
`/power_flags=8`, `/power_valid=false`, a `/current` i `/power` poprawnie
publikowały NaN. Potwierdzono także TF `odom → base_link`. Węzeł testowy
został zatrzymany po weryfikacji; napęd pozostał wyłączony.
