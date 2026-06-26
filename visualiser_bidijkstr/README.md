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
Zawartość skopiowana z `visualiser_arcflags`, ale bez preprocessingu. Cel na tu: dwukierunkowy dijkstra.

### Entry 2
Udało się pozmieniać tak, że algorytm działa dwukierunkowo. Po prostu.
