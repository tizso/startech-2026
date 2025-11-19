# Documentație Control Award - Inovații TeleOp - Echipa 18338 StarTech

## 1. Filozofia Noastră de Control Centrată pe Pilot

Filozofia noastră TeleOp este simplă: **împuternicim pilotul**. Credem că pilotul ar trebui să acționeze ca un strateg de nivel înalt, luând decizii într-o fracțiune de secundă, în timp ce robotul se ocupă de sarcinile complexe, repetitive și care necesită precizie. Programul nostru `TeleOpStarTech` nu este doar o telecomandă; este un partener inteligent, conștient de context, care îmbunătățește capacitatea pilotului și consistența scorului.

---

## 2. Inovație Cheie: Tranziție Lină de la Autonom la TeleOp

Una dintre cele mai mari provocări în FTC este pierderea conștientizării poziției pe teren atunci când meciul trece de la autonom la controlat de pilot. Software-ul nostru rezolvă această problemă cu un **sistem robust, hibrid, de persistență a datelor**.

*   **Problema:** Când OpMode-ul autonom se termină și începe TeleOp, odometria internă a robotului este resetată, lăsând pilotul să se reorienteze de la o coordonată (0,0,0), pierzând timp prețios și precizie.

*   **Soluția Noastră (Un Sistem pe Două Niveluri):
    1.  **Metoda Principală (Stocare în Fișier):** La finalul programului nostru `AutonomusStarTech`, `Pose`-ul (Poziția) finală a robotului (X, Y, Unghi) și `Side`-ul (Partea) de start (stânga sau dreapta) sunt salvate într-un fișier persistent (`last_pose.txt`) pe controlerul robotului. Această metodă supraviețuiește unei reporniri a aplicației.
    2.  **Metoda de Rezervă (Variabilă Statică):** Pentru a ne proteja împotriva erorilor rare ale sistemului de fișiere, exact aceleași date sunt salvate simultan într-o variabilă `public static` în clasa noastră ajutătoare `OpModeData`. Aceste date persistă în memorie între OpMode-uri.

*   **Încărcare Inteligentă:** La începutul `TeleOpStarTech`, secvența noastră `init()` efectuează o verificare de siguranță. Încearcă mai întâi să încarce din fișier. Dacă datele din fișier sunt invalide, trece fără probleme la copia de rezervă statică. Acest lucru asigură că piloții noștri încep **întotdeauna** perioada TeleOp cu cea mai precisă poziție posibilă, permițând mișcări strategice imediate.

## 3. Asistență Avansată pentru Pilot, Dependentă de Context

Programul nostru TeleOp oferă piloților instrumente puternice și inteligente care se adaptează în funcție de evenimentele din perioada autonomă.

### 3.1. Navigație "Go-To-Point" Dependentă de Context

Cu o **singură apăsare de buton (`dpad_left`)**, pilotul poate comanda robotului să navigheze către o poziție cheie de scorat sau de manevră. Aceasta nu este o țintă statică, pre-programată. Robotul ia o decizie inteligentă:

*   Folosește variabila `autoStartingSide` încărcată din perioada autonomă.
*   Dacă robotul a pornit pe partea **stângă**, trasează automat o cale către `(24, 24)`.
*   Dacă robotul a pornit pe partea **dreaptă**, trasează automat o cale către `(84, 84)`.

Această funcționalitate permite pilotului nostru să execute manevre complexe, pe tot terenul, cu viteză și precizie garantate, permițându-i să se concentreze pe fluxul jocului, nu pe alinierea manuală a robotului.

### 3.2. Asistență la Scorat Bazată pe Senzori, în Timp Real

În timpul TeleOp, software-ul nostru continuă să utilizeze sistemul de viziune pentru a asista pilotul.

*   **Scalare Automată a Puterii:** Când pilotul țintește spre coș, software-ul caută activ AprilTag-ul țintă (ID 20 sau 24). Dacă un tag este detectat, robotul preia controlul puterii de aruncare. Folosește datele de `range` (distanță) de la tag pentru a **scala automat puterea motorului** — mai puțină putere de aproape, mai multă putere de departe.
*   **Reducerea Sarcinii Cognitive a Pilotului:** Aceasta elimină presupunerile și variabilitatea ajustării manuale a puterii, crescând semnificativ consistența scorului. Pilotul trebuie doar să îndrepte robotul în direcția generală a coșului, iar software-ul se asigură că obiectul este lansat cu forța corectă.

## 4. Calitatea Codului și Robustețe

Angajamentul nostru pentru funcționalități avansate este construit pe o fundație de cod de înaltă calitate, profesional.

*   **Claritate și Lizibilitate:** Întreaga bază de cod este scrisă în engleză, cu comentarii clare și mesaje de telemetrie. Variabilele sunt inițializate explicit, iar "numerele magice" sunt înlocuite cu constante denumite.
*   **Mapare Curată a Butoanelor:** Comenzile sunt logice și separate. Pentru traiectoriile automate, avem un buton de activare dedicat (`dpad_left`) și un buton de anulare separat și intuitiv (`dpad_down`), prevenind suprapunerea accidentală a funcțiilor.
*   **Modularitate și Siguranță:** Codul nostru este organizat în clase ajutătoare logice și reutilizabile (`PoseStorage`, `OpModeData`). Parsarea datelor critice este încapsulată în blocuri `try-catch` pentru a gestiona elegant erorile potențiale (cum ar fi un fișier de date corupt), prevenind blocarea OpMode-ului în timpul unui meci.

Această arhitectură software robustă este ceea ce face posibile funcționalitățile noastre avansate de asistare a pilotului și de legătură cu autonomul, oferind echipei noastre un avantaj competitiv semnificativ pe teren.
