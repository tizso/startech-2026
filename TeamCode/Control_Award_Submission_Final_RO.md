# Documentație Finală Control Award - Echipa 18338 StarTech

## 1. Filozofia Software: Robotul ca Partener

Filozofia noastră software se concentrează pe trei principii de bază: **Robustețe, Inteligență și Asistență pentru Pilot**. Credem că cel mai eficient robot este cel care își poate executa sarcinile complexe în mod fiabil, se poate adapta inteligent la dinamica jocului și poate extinde fără probleme capabilitățile piloților noștri umani. Sistemul nostru de control este proiectat de la zero pentru a fi un partener real pe teren, nu doar o unealtă.

---

## 2. Autonomul "Super": O Strategie Dinamică, Multi-fazică

Programul nostru autonom de referință, `AutonomusSuper.java`, nu este o singură cale pre-programată. Este o mașină de stări (state machine) sofisticată, multi-fazică, ce utilizează fuziunea avansată a senzorilor pentru a executa o rutină completă, cu un scor ridicat.

### Faza 1: Primul Scorat de Precizie
*   **Start Bazat pe Viziune:** Rutina începe prin identificarea unuia dintre cele trei AprilTag-uri randomizate (ID 21-23), determinând instantaneu starea jocului. Pe baza poziției tag-ului, robotul își determină partea de start (stânga sau dreapta), decizie care dictează toate strategiile ulterioare.
*   **Poziționare Dinamică:** Folosind o buclă de control de tip PID, robotul efectuează "vizual-servoing" pentru a se poziționa cu precizie atât față de tag-ul inițial, cât și față de tag-ul coșului (ID 20 sau 24). Această navigație bazată pe viziune este foarte rezistentă la erorile de odometrie.
*   **Prima Lovitură:** Odată ajuns în poziție, robotul scorează obiectele pre-încărcate.

### Faza 2: Colectarea Autonomă de Mingi
Aici robotul nostru își demonstrează cu adevărat inteligența. În loc să parcheze, trece într-o fază de colectare.
*   **Procesare Vizuală Duală:** Portalul nostru de viziune rulează simultan doi procesori: `AprilTagProcessor` și procesorul nostru personalizat, `BallDetectionProcessor`. Acest lucru permite robotului să caute mingi colorate (mov și verzi) fără a-și pierde capacitatea de a vedea AprilTag-uri.
*   **Sub-rutină de Căutare și Achiziție:** Robotul intră într-o sub-mașină de stări dedicată colectării. Își scanează activ împrejurimile, iar la detectarea unei mingi, se centrează pe ea, estimează distanța pe baza dimensiunii conturului și inițiază o secvență de admisie (intake).
*   **Fuziunea Senzorilor pentru Confirmare:** Un senzor de distanță din interiorul sistemului de admisie oferă confirmarea definitivă a unei colectări reușite, făcând procesul extrem de fiabil. Robotul repetă acest ciclu până când colectează numărul necesar de mingi noi.

### Faza 3: Al Doilea Scorat și Parcare Inteligentă
*   **Re-achiziție și Scorat:** Cu mingile noi colectate, robotul re-achiziționează AprilTag-ul coșului, navighează înapoi la poziția optimă de scorat și lansează a doua salvă.
*   **Parcare Asistată de Viziune:** Manevra finală de parcare nu este o mișcare "oarbă". Robotul menține AprilTag-ul coșului în câmpul vizual, executând o mișcare laterală precisă la o distanță calculată. Acest lucru asigură o poziție finală constantă și precisă, de fiecare dată.

---

## 3. Tranziție Lină și TeleOp cu Asistență pentru Pilot

Programul nostru `TeleOpStarTech.java` continuă filozofia controlului inteligent, împuternicind piloții.

### 3.1. Persistența Robustă a Datelor între Autonom și TeleOp
Pentru a menține conștientizarea poziției pe teren, am implementat un **sistem robust, hibrid, de persistență a datelor**.
*   **Metoda Principală (Stocare în Fișier):** La finalul autonomului, `Pose`-ul final al robotului și `Side`-ul (partea) de start sunt salvate într-un fișier persistent.
*   **Metoda de Rezervă (Variabilă Statică):** Aceleași date sunt salvate simultan într-o variabilă `static` în memorie (`OpModeData`).
*   **Încărcare Inteligentă:** Programul TeleOp încearcă mai întâi să încarce din fișier. Dacă acest lucru eșuează, trece fără probleme la copia de rezervă statică. Acest sistem pe două niveluri face robotul excepțional de rezistent la erori, asigurând că pilotul începe **întotdeauna** cu o poziție precisă pe teren.

### 3.2. Asistență pentru Pilot Dependentă de Context
*   **Navigație "Smart" Go-To-Point:** O singură apăsare de buton (`dpad_left`) trimite robotul la o poziție cheie. Destinația este aleasă inteligent pe baza părții de start încărcate din autonom (`(24, 24)` pentru stânga, `(84, 84)` pentru dreapta).
*   **Asistență la Scorat în Timp Real:** În timpul controlului manual, sistemul de viziune rămâne activ. La țintire, robotul detectează AprilTag-ul coșului și scalează automat puterea motorului de aruncare pe baza distanței măsurate, crescând consistența scorului.

### 3.3. "Auto-Aim" în Timp Real și Sistem de Blocare a Tragerii
Apogeul funcționalităților noastre de asistență pentru pilot este modul "Auto-Aim", activat prin butonul `start`. Acesta transformă robotul într-o platformă de scorat semi-autonomă:
*   **Menținerea Persistentă a Țintei:** Odată activat, software-ul preia controlul complet al transmisiei. Folosește continuu datele de viziune în timp real pentru a menține o distanță perfectă (`GOAL_TAG_DISTANCE`) și o aliniere optimă față de AprilTag-ul țintă. Dacă robotul este împins sau lovit, acesta **își corectează autonom poziția** pentru a re-dobândi ținta.
*   **Sistem de Blocare a Tragerii (Interlock):** Pentru a preveni tirurile ratate, servomotoarele de tragere sunt **complet dezactivate** în timp ce robotul caută tag-ul sau își ajustează poziția. Doar atunci când eroarea de aliniere este în interiorul `POSITIONING_TOLERANCE` (toleranței noastre stricte), comenzile pentru servo devin active, acest lucru fiind semnalat pilotului printr-un mesaj de telemetrie "Locked On! Ready to fire.". Acest lucru asigură că un obiect poate fi lansat doar atunci când o lovitură reușită este garantată.

Acest sistem permite pilotului să se concentreze exclusiv pe momentul tragerii, având încredere că alinierea robotului este perfectă.

## 4. Fundamentul unui Cod de Calitate

Aceste funcționalități avansate sunt construite pe o fundație de practici profesionale de inginerie software, incluzând clase ajutătoare modulare (`PoseStorage`, `BallDetectionProcessor`), aderarea strictă la un cod lizibil și mentenabil, și o gestionare robustă a erorilor pentru a asigura fiabilitate maximă în timpul competiției.
