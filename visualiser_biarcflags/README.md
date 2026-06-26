# Arc-Flags
Aby zkompilować:
1) `mkdir build && cd build`
2) `cmake .. && cmake --build .`

Aby uruchomić:
1) `./PathVisualizer`
2) Kliknij LOAD i wybierz plik .osm.pbf
3) Kliknij na mapie na punkt by zaznaczyć punkt startowy. Naciśnij drugi raz aby wybrać końcowy.
4) Naciśnij START aby uruchomić algorytm. Prędkość można regulować odpowiednim suwakiem (uwaga: skala logarytmiczna). Przycisk PAUSE pauzuje symulację. Przycisk RESET resetuje ją.

## LogBook
Za wszystkie zmiany odpowiada naturalna głupota (nie sztuczna inteligencja).

### Entry 1
Zawartość skopiowana ze zwykłych argflags'ów. Pora połączyć to z dwukierunkowym dijkstrą.

### Entry 2
Przepisywanie kodu wyszło mało uciążliwe. Za to wyniki są obiecujące - *blazingly fast* byłoby tu idealnym określeniem (przypominam, nie chodzi o preprocessing; ten jest jaki jest). Ponownie, mieli się ok 3 minuty, potem można się bawić. Dokładniejszy opis można znaleźć przy jednokierunkowych arcflagach. Miłej zabawy :).
