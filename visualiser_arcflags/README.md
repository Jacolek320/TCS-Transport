# Arc-Flags
Aby zkompilować:
1) mkdir build && cd build
2) cmake .. && cmake --build .

Aby uruchomić:
1) ./PathVisualizer
2) Kliknij LOAD i wybieramy plik .osm.pbf
3) Kliknij na mapie na punkt by zaznaczyć punkt startowy. Naciśnij drugi raz aby wybrać końcowy.
4) Naciśnij START aby uruchomić algorytm. Prędkość można regulować odpowiednim suwakiem (uwaga: skala logarytmiczna). Przycisk PAUSE pauzuje symulację. Przycisk RESET resetuje ją.

## LogBook
Za wszystkie zmiany odpowiada naturalna głupota (nie sztuczna inteligencja).

### Entry 1
Zawartość skopiowana z visualizera dijkstry + próba przerobienia. Cel: arc-flags.

### Entry 2
Udało się wydzielić dijkstrę od visualisera. Rysowanie dalej jest zbyt wolne.

### Entry 3
Udało się przyspieszyć visualiser, ale wciąż jest wolny.

### Entry 4
Część preprocessingu odpowiadająca za podział na regiony zrobiona. Najleniwiej jak tylko możliwe, czyli kratą. Podział jest jaki jest, przy 726423 wierzchołkach (tak, mapa Krakowa) i kracie 3x3 rozmiary regionów są pomiędzy 40k na obrzeżach a 189k w centrum. Przy większej liczbie komórek będzie pewnie jeszcze większy rozjazd (PS: jest), ale może coś z tego wyjdzie.


