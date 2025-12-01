# Prezentare pentru Control Award - Echipa StarTech

## 1. Introducere: O Abordare Avansată a Controlului Robotului

Acest document prezintă arhitectura software și algoritmii avansați care stau la baza robotului nostru. Strategia noastră se concentrează pe **fiabilitate, precizie și modularitate**, folosind un sistem de control sofisticat pentru a executa sarcini complexe atât în perioada autonomă, cât și în cea teleoperată.

Codul nostru este construit în jurul a două componente cheie:
1.  **O mașină de stări (state machine) robustă, non-blocantă,** pentru o execuție fiabilă și predictibilă a secvențelor autonome.
2.  **Localizare precisă bazată pe viziune,** folosind AprilTag-uri fixe de pe teren pentru a obține o poziție absolută, eliminând erorile cauzate de alunecare sau de referințe de pe teren nesigure.

---

## 2. Perioada Autonomă: Precizie și Inteligență

Programul nostru `AutonomousNextGen.java` este proiectat pentru a fi mai mult decât o simplă secvență de comenzi. Este un sistem dinamic care se adaptează la condițiile de pe teren.

### 2.1. Localizarea Hibridă: Strategie vs. Navigație

Am identificat o problemă critică în regulamentul jocului: AprilTag-urile centrale ("Obelisk") **nu au o poziție garantată** și nu pot fi folosite pentru navigație precisă. Soluția noastră este o abordare hibridă inovatoare, implementată în `handleSetup()`:

1.  **Determinarea Strategiei:** La inițializare, robotul caută un tag "Obelisk" (ID 21-23) cu un singur scop: să determine **strategia** - partea de teren (`initialSide`) și secvența de aruncare (`foundID`).
2.  **Localizarea Absolută:** Ulterior, robotul caută un tag de pe **backdrop** (ID 20 sau 24). Deoarece poziția acestor tag-uri este **fixă și cunoscută**, folosim oricare dintre ele ca **punct de referință absolut (anchor)** pentru a ne calcula poziția pe teren. Acest lucru ne permite să obținem o poziție de start precisă, chiar dacă vedem tag-ul "greșit" de pe partea opusă a terenului.
3.  **Corecția Poziției Camerei:** Calculul ia în considerare și offset-ul fizic al camerei față de centrul de rotație al robotului (`CAMERA_FORWARD_OFFSET`), asigurând o localizare de înaltă precizie a robotului, nu doar a camerei.

### 2.2. Mișcare în Două Etape: Apropiere și "Andocare" de Precizie

În loc să mergem direct la o poziție de tragere pre-calculată, folosim o abordare în două etape pentru o precizie maximă:

1.  **`START_MOVE`:** Robotul execută o traiectorie lină (Bezier) către o **zonă generală de tragere**.
2.  **`FINAL_APPROACH`:** Odată ajuns în zonă, robotul:
    *   Caută din nou tag-ul de pe backdrop.
    *   **Recalculează dinamic poziția de tragere ideală** pe baza poziției în timp real a tag-ului, a offset-urilor de tragere dorite (`SHOOTING_X_OFFSET`, `SHOOTING_Y_OFFSET`) și a **limitelor terenului** (liniile de centru, `BLUE_SIDE_MAX_X`), asigurându-se că robotul rămâne mereu în zona legală.
    *   Execută o a doua mișcare scurtă și precisă ("docking maneuver") către această poziție perfectă.

### 2.3. Ciclul de Aruncare Non-Blocant

Am eliminat complet comenzile `sleep()` din logica de aruncare, înlocuindu-le cu o secvență de stări fiabilă care verifică viteza roților înainte de fiecare aruncare:

- **`CHECK_SPEED_FOR_SHOT`:** Înainte de fiecare aruncare, robotul intră într-o stare de așteptare. Doar atunci când senzorul encoder al motorului raportează că roțile au atins turația necesară (ex: 95% din `SHOOTER_RPM`), programul avansează la starea `SHOOT`.
- **Controlul Vitezei (Velocity Control):** Folosim comanda `motor.setVelocity()` în loc de `setPower()`. Acest lucru permite controlerului PIDF intern al motorului să mențină o **turație constantă**, indiferent de fluctuațiile de tensiune ale bateriei, garantând o forță de aruncare consistentă de la începutul până la sfârșitul meciului.

---

## 3. Perioada TeleOperată: Asistență Inteligentă pentru Pilot

Programul `TeleOpStarTech.java` este proiectat pentru a oferi pilotului control manual complet, dar cu un strat de inteligență care simplifică sarcinile complexe.

### 3.1. Tranziție Autonom -> TeleOp Fără Întreruperi

La finalul perioadei autonome, poziția finală precisă a robotului (`finalShootingPose` sau, ca alternativă, `follower.getPose()`) și partea de teren (`initialSide`) sunt salvate într-un fișier (`PoseStorage`) și într-o variabilă statică (`OpModeData`). La inițializarea programului TeleOp, aceste date sunt încărcate, permițând ca sistemul de odometrie `PedroPathing` să continue urmărirea poziției absolute a robotului pe teren fără a necesita o resetare.

### 3.2. Navigare Automată la Punct (Driver-Assist)

La apăsarea butonului `START` de pe gamepad, pilotul poate activa o funcție de navigare automată. Robotul:
1.  Identifică partea de teren pe care se află (încărcată din autonom).
2.  Stabilește o coordonată țintă pre-definită (ex: o poziție sigură de încărcare sau de apărare).
3.  Generează și execută autonom o traiectorie către acel punct, permițând pilotului să se concentreze pe alte sarcini.
4.  Pilotul poate anula oricând mișcarea automată prin apăsarea `dpad_down`, preluând controlul manual instantaneu.

---

## 4. Concluzie: Inovație și Fiabilitate

Software-ul nostru demonstrează o înțelegere profundă a principiilor de control avansate:
- **Senzori și Feedback:** Folosim extensiv datele de la encoderele motoarelor (`setVelocity()`, odometrie) și de la cameră (AprilTag `ftcPose`) pentru a crea un sistem de control cu feedback în buclă închisă (closed-loop).
- **Modelarea Lumii Reale:** Algoritmii noștri de localizare iau în considerare constrângerile fizice ale robotului (dimensiuni, offset-ul camerei) și regulile terenului (linii de centru, poziții fixe ale tag-urilor), rezultând un comportament autonom robust și predictibil.
- **Fiabilitate:** Prin eliminarea comenzilor `sleep()` și implementarea unei mașini de stări complexe, non-blocante, cu timeout-uri și mecanisme de siguranță (failsafe), am creat un cod care funcționează fiabil chiar și în condiții neprevăzute.

Această abordare ne oferă un avantaj strategic semnificativ, permițând robotului să execute sarcini cu o precizie și o viteză pe care controlul pur manual nu le-ar putea atinge.
