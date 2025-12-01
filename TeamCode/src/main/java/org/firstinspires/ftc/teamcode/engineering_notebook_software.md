# Caiet de Inginerie Software - Echipa StarTech: De la Concept la Realitate

## Rezumat Executiv

Acest document prezintă călătoria inginerească din spatele software-ului robotului nostru pentru sezonul 2025-2026. Nu este doar o descriere a codului final, ci o cronică a procesului iterativ de design, testare și perfecționare. Pornind de la concepte de bază, am întâmpinat provocări complexe legate de fizica reală a mediului de joc, pe care le-am depășit prin soluții software inovatoare. Fiecare decizie a fost luată cu scopul de a crea un sistem de control robust, precis și, în final, un partener inteligent pentru piloții noștri. Documentul este structurat în capitole distincte pentru programul autonom și cel teleoperat, culminând cu o secțiune dedicată pentru Control Award, care subliniază excelența tehnică a software-ului nostru.

---

## Capitolul 1: Programul Autonom - Căutarea Neîncetată a Preciziei Absolute

Obiectivul nostru fundamental pentru perioada autonomă a fost crearea unui program care să nu se bazeze pe noroc sau pe condiții ideale. Am dorit un sistem capabil să se localizeze precis, indiferent de poziția de start, și să execute sarcini complexe cu o fiabilitate de neegalat. Această căutare a preciziei ne-a purtat printr-un proces riguros de identificare și rezolvare a problemelor.

### Etapa 1: Cea Mai Dificilă Provocare - Criza de Localizare

Cea mai mare provocare tehnică a fost, fără îndoială, obținerea unei poziții de start fiabile. Întreaga secvență autonomă depinde de o estimare corectă a poziției inițiale a robotului pe teren.

**Conceptul Inițial (Greșit):** Prima noastră idee a fost să folosim AprilTag-ul central (de pe "Obelisk") ca punct de referință. Logica părea simplă: dacă știm poziția teoretică a tag-ului, putem calcula cu ușurință poziția robotului față de acesta.

**Problema Descoperită în Teste:** Primele teste pe teren au fost un eșec. Robotul se poziționa haotic, traiectoriile fiind complet imprevizibile. Investigând sursa acestor erori masive, am descoperit o regulă critică în manualul oficial al jocului: **"The location of the OBELISK is not intended to be deterministic relative to the field coordinate system and should not be used for navigation."**

Această realizare a fost un moment de cotitură. Am înțeles că întreaga noastră fundație software era construită pe o premisă falsă. A trebuit să abandonăm complet această abordare și să regândim strategia de la zero.

### Etapa 2: Soluția Finală Inovatoare - Localizarea Hibridă și Universală

După multiple iterații și teste, am dezvoltat o strategie care separă complet **strategia** de **navigație** și gestionează toate scenariile de start posibile.

1.  **Strategia (cu Obelisk):** La inițializare, robotul caută tag-ul Obelisk (21-23) cu **un singur scop**: să determine partea de teren (`initialSide`) și secvența de aruncare (`foundID`).
2.  **Navigația (cu Backdrop):** Ulterior, robotul caută **oricare** tag de pe backdrop (20 sau 24). Aici a fost inovația: am realizat că **nu contează dacă vedem tag-ul "greșit"**. Deoarece cunoaștem pozițiile absolute ale **ambelor** tag-uri, oricare dintre ele poate servi ca un "anchor" de înaltă precizie.
3.  **Cel Mai Dificil Calcul:** Am perfecționat formula trigonometrică pentru a include și **offset-ul fizic al camerei** față de centrul de rotație al robotului. Acesta a fost cel mai complex calcul implementat, crucial pentru precizie.

    **Formula de Rotație a Vectorului:**
    `worldVectorX = relX * cos(θ) - relY * sin(θ)`
    `worldVectorY = relX * sin(θ) + relY * cos(θ)`

    **Codul Final, Robust (`handleSetup`):**
    ```java
    // Definim pozițiile fixe, cunoscute, ale ambelor tag-uri de pe backdrop
    private static final Pose BLUE_BACKDROP_POSE = new Pose(16.01, 133.28, Math.toRadians(-126.0));
    private static final Pose RED_BACKDROP_POSE = new Pose(127.99, 133.28, Math.toRadians(-54.0));

    // ... în interiorul handleSetup() ...
    if (obeliskTag != null) {
        // 1. Determină strategia din Obelisk
        initialSide = (obeliskTag.ftcPose.x < 0) ? -1 : 1;

        Pose landmarkPose = null;
        // 2. Identifică CE tag de backdrop este vizibil
        if (backdropTag != null) {
            if (backdropTag.id == BLUE_GOAL_TAG_ID) landmarkPose = BLUE_BACKDROP_POSE;
            else if (backdropTag.id == RED_GOAL_TAG_ID) landmarkPose = RED_BACKDROP_POSE;
        }

        if (landmarkPose != null) {
            // 3. Calculează poziția absolută folosind oricare tag vizibil
            // ... (calcule trigonometrice precise) ...
            currentState = State.START_MOVE;
        } else {
            // 4. Oferă feedback pilotului dacă nu se poate localiza
            telemetry.addData("ACTION", "<--- PAN to find a Backdrop Tag --->");
        }
    }
    ```

---

## Capitolul 2: Optimizarea Mecanismului de Aruncare

Performanța în meci este direct legată de viteza și consistența aruncărilor. Am dedicat o mare parte din procesul de dezvoltare optimizării acestui mecanism.

### 2.1. `setPower` vs. `setVelocity`: Bătălia pentru Consistență

**Problemă:** Testele inițiale cu `motor.setPower(0.8)` au arătat o inconsistență majoră. Aruncările de la începutul meciului (baterie la ~14V) erau mult mai puternice decât cele de la final (~11V).

**Soluția (Control Closed-Loop):** Am înlocuit `setPower` (control open-loop) cu `motor.setVelocity()`. Această comandă activează **controller-ul PIDF intern al motorului**:
1.  Noi îi dăm o **turație țintă** (ex: 3600 RPM).
2.  Controller-ul citește **turația reală** de la encoder.
3.  Calculează **eroarea** și ajustează automat puterea pentru a o anula.

Acest sistem garantează o turație constantă, indiferent de voltaj. A necesitat un proces de **reglaj al coeficienților PIDF** cu REV Hardware Client pentru a obține un răspuns rapid și stabil.

### 2.2. Un Motor sau Două? Analiza Inerției și a Cuplului

**Problemă:** După o aruncare, turația roților scade. Cât de repede putem reveni la turația optimă?

**Analiza:** Răspunsul stă în Accelerația Unghiulară (`α`), definită de `α = τ / I`, unde `τ` este cuplul și `I` este momentul de inerție. Două motoare oferă un cuplu dublu (`2τ`), rezultând o accelerație dublă și un timp de recuperare mult mai scurt.

### 2.3. Raportul de Transmisie: De ce 1:1.5 este Mai Bun decât 1:1

**Problemă:** Cum obținem cea mai rapidă accelerație a roților?

**Intuiția (Greșită):** Un raport de accelerare (pinion mare la motor) ar trebui să tureze roata mai repede. Acest lucru este echivalent cu a pleca cu mașina în viteza a cincea - cuplu insuficient.

**Soluția (Fizică):** Un motor DC produce **putere maximă** la ~50% din turația sa. Pentru a accelera o sarcină grea, avem nevoie de cuplu. Un raport de **încetinire de 1:1.5** (pinion mic la motor, mare la roată) acționează ca o pârghie mecanică: multiplică cuplul la axul roții cu 1.5, rezultând o accelerație superioară.

---

## Capitolul 3: Programul TeleOperat - Parteneriatul Om-Mașină

### 3.1. Tranziție Fără Întreruperi și Automatizare la Cerere

Sinergia dintre autonom și teleoperat este un pilon al strategiei noastre. La finalul autonomiei, poziția precisă a robotului este salvată (`PoseStorage`). La începutul TeleOp, această poziție este încărcată, oferind robotului cunoaștere absolută a terenului din prima secundă. Această fundație permite funcția noastră de asistență:

**Automatizare la Cerere:** La apăsarea butonului `START` de pe gamepad, robotul navighează autonom la o poziție predefinită, eliberând pilotul pentru sarcini strategice.

### 3.2. Schema de Control Gamepad

| Buton/Stick          | Funcție                                                    |
| -------------------- | ---------------------------------------------------------- |
| **Left Stick Y/X**   | Deplasare Față/Spate și Laterală (Strafe)                   |
| **Right Stick X**    | Rotație Stânga / Dreapta                                   |
| **Buton A**          | Comutare Mod Lent (Slow Mode) On/Off                       |
| **Buton B**          | Comutare Colectare (Intake) On/Off                         |
| **Buton X**          | Comutare Aruncător (Outtake) On/Off                        |
| **Buton Y**          | Comutare Mod Invers (Reverse) pentru Colectare             |
| **D-Pad Sus**        | Activare Servo Separator                                   |
| **Left/Right Bumper**| Control Manual Servo-uri de Colectare                      |
| **Buton START**      | **Activare Navigare Automată către Poziție Predefinită**   |

### 3.3. Inovație Maximă: Calculatorul Balistic în Timp Real

Cea mai avansată funcționalitate este calcularea dinamică a turației de aruncare. În TeleOp, camera caută constant tag-ul țintă. Când este găsit, programul rezolvă ecuația mișcării proiectilului pentru a găsi viteza necesară:

`v_ball = sqrt( (g * x²) / (2 * cos²(θ) * (x*tan(θ) - y)) )`

Această viteză este convertită în RPM și trimisă motoarelor, permițând aruncări precise de la orice distanță.

---

## Capitolul 4: Prezentare pentru Control Award

### Abstract
Software-ul robotului nostru este un sistem de control avansat, stratificat, care demonstrează o înțelegere profundă a principiilor de control în buclă închisă, a modelării fizice și a strategiei de joc. De la localizarea hibridă robustă în autonomie, la calculatorul balistic în timp real din teleoperat, fiecare funcție a fost dezvoltată iterativ pentru a rezolva probleme reale și a oferi un avantaj competitiv decisiv.

### Elemente de Control și Senzori
- **Senzori:** Folosim extensiv feedback-ul de la **encoderele motoarelor** (pentru odometrie și controlul vitezei `setVelocity`) și de la **cameră** (pentru detecția AprilTag-urilor).
- **Control în Buclă Închisă (Closed-Loop):**
  - **Control PIDF:** Utilizat implicit prin `setVelocity()` pentru a garanta o turație constantă a aruncătorului, eliminând efectul variației de tensiune a bateriei.
  - **Navigație:** Biblioteca `PedroPathing` folosește odometria (din encodere) pentru a urmări poziția, iar noi corectăm această estimare la începutul autonomiei folosind datele de la AprilTag-uri pentru o precizie absolută.

### Inovație și Strategie
- **Localizare Hibridă:** Abordarea noastră de a separa tag-urile de strategie de cele de navigație este o soluție inovatoare și robustă la o problemă fundamentală a acestui sezon.
- **"Software Turret":** În loc să construim un mecanism de țintire complex, folosim întregul șasiu al robotului ca pe un sistem de țintire de înaltă precizie prin manevra software de "andocare" (`FINAL_APPROACH`).
- **Sinergie Autonom-TeleOp:** Transferul de date de poziție între cele două moduri de operare creează o continuitate care maximizează eficiența pe tot parcursul meciului.

### Concluzie
Prin îmbinarea riguroasă a teoriei matematice, a principiilor fizice și a testării iterative pe teren, am creat un software care nu doar că funcționează, ci excelează prin fiabilitate, precizie și un grad înalt de inteligență artificială. Credem cu tărie că aceste elemente fac din software-ul nostru un candidat de top pentru Control Award.
