Rozkazy komunikacji z robotem

Format ramki USB

AA | LEN | GROUP | CMD | ROBOT | SEQ | DATA... | CRC\_H | CRC\_L

LEN — liczba bajtów po polu LEN należących do content; całe content ma LEN + 1 bajtów.

GROUP — 1.

CMD — kod rozkazu.

ROBOT — numer robota 1..16.

SEQ — numer transakcji. W żądaniu może mieć dowolną wartość; most radiowy zastępuje go własną wartością 1..255. Odpowiedź zawiera tę samą wartość.

Liczby wielobajtowe w DATA są przesyłane od najmłodszego bajtu (little-endian).

float oznacza IEEE-754 binary32.

CRC-16 obejmuje całe content, od LEN do końca DATA: wartość początkowa 0xFFFF, wielomian 0x8005, bez odbicia i bez końcowego XOR. CRC jest wysyłane jako starszy, potem młodszy bajt.

Most wysyła do komputera tylko świeżą odpowiedź robota, odebraną przez mechanizm POLL. Brak poprawnej odpowiedzi po ponowieniach oznacza timeout — nie jest generowana osobna ramka błędu.



Rozkazy

1 — ustaw prędkości kół (stary, bez odpowiedzi)

Żądanie DATA:



VL:int16 | VR:int16

Rozkaz ustawia prędkości zadane. Aktualny kod nie buduje dla niego odpowiedzi, dlatego nie należy używać go przez most radiowy wymagający odpowiedzi.



2 — LED (nieobsługiwany)

Kod jest zarezerwowany, ale interpreter nie zawiera jego implementacji ani odpowiedzi.



3 — prędkości kół i odometria (format legacy)

Żądanie DATA (LEN = 22):



STATUS:uint16 | WL:int16 | WR:int16 | X:float | Y:float | TH:float

Odpowiedź CMD = 3, LEN = 26:



STATUS:uint16 | WL:int16 | WR:int16 | X:float | Y:float | TH:float |

SUPPLY\_MV:uint16 | UR:int16

WL, WR są w formacie Q8, a UR w Q12. Pole STATUS odpowiedzi ma obecnie zawsze wartość 0.



4 — ustaw parametry regulatora

Żądanie DATA (LEN = 20):



WC:float | WO:float | ZETA:float | J:float

Zakresy: WC=1..200, WO=WC..1000, ZETA=0.1..5, J=0.00001..1.



Odpowiedź CMD = 4, LEN = 22:



STATUS:uint16 | ACTIVE\_WC:float | ACTIVE\_WO:float | ACTIVE\_ZETA:float | ACTIVE\_J:float

Odpowiedź zawsze zwraca parametry aktualnie aktywne. STATUS: 0 — OK, 1 — nieprawidłowe dane.



5 — sterowanie samochodowe (format legacy)

Żądanie DATA (LEN = 22):



STATUS:uint16 | WHEEL\_VELOCITY:int16 | SERVO:int16 |

X:float | Y:float | TH:float

Odpowiedź CMD = 3, LEN = 22:



STATUS:uint16 | W\_REAR:int16 | X:float | Y:float | TH:float | UL:int16

Uwaga: dla zgodności ze starym protokołem odpowiedź ma CMD = 3, nie 5. WHEEL\_VELOCITY jest w Q8, SERVO i UL w Q12. Pole STATUS odpowiedzi ma wartość 0.



6 — prędkości kół i odometria (format kompaktowy)

Zalecany rozkaz cyklicznego sterowania. Żądanie DATA (LEN = 20):



STATUS:uint16 | WL:int16 | WR:int16 | X:int24\_Q15 | Y:int24\_Q15 |

TH:int16\_Q10 | RESERVED:uint16

Pole RESERVED musi mieć wartość 0.



Odpowiedź CMD = 6, LEN = 27:



WL:int16 | WR:int16 | X:int24\_Q15 | Y:int24\_Q15 | TH:int16\_Q10 |

E\_PHI\_L:int16\_Q10 | E\_PHI\_R:int16\_Q10 | UL:int8\_Q7 | UR:int8\_Q7 |

POWER\[5]

int24\_Q15 / 32768 daje X lub Y.

int16\_Q10 / 1024 daje TH i błędy E\_PHI\_\*.

int8\_Q7 / 127 daje znormalizowane sterowanie UL, UR w zakresie -1..1.

POWER\[0..4] zawiera kolejno upakowane V\[11:0], I\[11:0], P\[11:0] oraz FLAGS\[3:0]:

POWER\[0] = V\[11:4]

POWER\[1] = V\[3:0] | I\[11:8]

POWER\[2] = I\[7:0]

POWER\[3] = P\[11:4]

POWER\[4] = P\[3:0] | FLAGS\[3:0]

W POWER\[1] i POWER\[4] pierwsze pole zajmuje starszy półbajt, a drugie młodszy. Napięcie V jest bez znaku i ma skalę 10 mV/LSB. Prąd I jest 12-bitową liczbą ze znakiem w kodzie uzupełnień do dwóch o skali 2 mA/LSB. Moc P jest bez znaku i ma skalę 25 mW/LSB.



Flagi zasilania:



bit 0 (0x01) — pomiar prądu i mocy jest ważny,

bit 1 (0x02) — napięcie pochodzi z INA219; w przeciwnym razie z ADC,

bit 2 (0x04) — INA219 został poprawnie skonfigurowany,

bit 3 (0x08) — obsługa monitora mocy jest włączona w firmware.

Jeżeli pomiar INA219 nie jest dostępny lub ważny, firmware zwraca napięcie z ADC oraz zerowy prąd i moc.



7 — ustaw geometrię odometrii

Żądanie DATA (LEN = 16):



LEFT\_RADIUS\_MM:float | RIGHT\_RADIUS\_MM:float | WHEEL\_BASE\_MM:float

Odpowiedź CMD = 7, LEN = 18:



STATUS:uint16 | ACTIVE\_LEFT\_RADIUS\_MM:float |

ACTIVE\_RIGHT\_RADIUS\_MM:float | ACTIVE\_WHEEL\_BASE\_MM:float

STATUS: 0 — OK, 1 — nieprawidłowe dane. Odpowiedź zawiera aktualnie aktywną geometrię.



8 — zapisz konfigurację do Flash

Żądanie bez DATA (LEN = 4). Dla ochrony przed przypadkowym zapisem należy wysłać rozkaz dwa razy z rzędu. Dowolny inny rozkaz rozbraja zapis.



Odpowiedź CMD = 8, LEN = 6:



STATUS:uint16

0 — zapis zakończony; robot zresetuje się po około 100 ms,

1 — nieprawidłowe dane,

2 — błąd zapisu Flash,

3 — zapis uzbrojony; należy ponownie wysłać CMD = 8.

Pole STATUS w rozkazach 3, 5 i 6

bit 0 (0x0001) — włącz regulator i zastosuj prędkość z żądania,

bit 1 (0x0002) — włącz stopnie mocy silników,

bit 2 (0x0004) — ustaw odometrię na X, Y, TH z żądania.

Wyzerowanie bitu 1 wyłącza napęd i zeruje prędkość zadaną.

