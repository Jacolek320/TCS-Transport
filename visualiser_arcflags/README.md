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
Zawartość skopiowana z visualizera dijkstry + próba przerobienia. Cel: arc-flags.

### Entry 2
Udało się wydzielić dijkstrę od visualisera. Rysowanie dalej jest zbyt wolne.

### Entry 3
Udało się przyspieszyć visualiser, ale wciąż jest wolny.

### Entry 4
Część preprocessingu odpowiadająca za podział na regiony zrobiona. Najleniwiej jak tylko możliwe, czyli kratą. Podział jest jaki jest, przy 726423 wierzchołkach (tak, mapa Krakowa) i kracie 3x3 rozmiary regionów są pomiędzy 40k na obrzeżach a 189k w centrum. Przy większej liczbie komórek będzie pewnie jeszcze większy rozjazd (PS: jest), ale może coś z tego wyjdzie.

### Entry 5
Całość kodu napisana. Sanity check, pod tytułem 1 region, przechodzi. Niestety, preprocessing rzeczywiście jest kosztowny. Każdy jeden dijkstra z wierzchołka brzegowego trwa ok. 0.7s, co daje pardziesiąt minut (jak nie ponad godzinę) nawet dla małych podziałów.

### Entry 6
Po zmniejszeniu mapki preprocessing wygląda jakby miał trwać zjadliwe parę minut.

### Entry 6a
Rzeczywiście trwało to mniej więcej tyle co przerwa na kanapki (`graph.cpp::percent = 0.3`, `algorithm.cpp::side = 5`), a wynik wygląda nieźle - rozgałęzia się bardzo blisko celu.

### Entry 6b
Po kolejnej przerwie wyniki dla `graph.cpp::percent = 0.3`, `algorithm.cpp::side = 10` również są wyśmienite, ale nie ma aż tak dużego przeskoku jak wcześniej. Dodatkowo, przy zabawie programem udało mi się ponatrafiać na niespójności. Jak to się mówi - skill issue.

### Entry 7
Podsumowując, zabawę oceniam na +30°C w cieniu na 10. Wygląda fajnie, działa też. Preprocessing może i trwa długo, ale jak się człowiekowi nie spieszy, to do przeżycia. Możnaby to serializować, ale chyba nie pokuszę się, żeby wrzucać ogromne pliki do repozytorium.

Obecnie całość ustawiona jest na `graph.cpp::percent = 0.2`, `algorithm.cpp::side = 5` (regiony są całkiem nieźle zbalansowanie). Preprocessing trwa (u mnie) niecałe 3 minuty, co powinno być zjadliwą dla czytelnika wartością. Może i cudów nie ma, ale zdecydowanie da się zobaczyć poprawę względem zwykłego dijkstry (który wykonywany jest pod podszewką). W terminalu można obserwować logi :).

### Entry 8
Dla podkręconej wartości `graph.cpp::percent = 0.2`, `algorithm.cpp::side = 20` większość wyszukań jest prawie natychmiastowa. Ale trzeba sobie paręnaście minut na to poczekać.
