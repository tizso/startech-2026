# Analiza Mecanismului de Aruncare: Cuplu vs. Viteză

Acest document explică, cu calcule fizice concrete, de ce un raport de transmisie de **încetinire (torque gearing)** este superior pentru a obține o accelerație rapidă a roților de aruncare (flywheels).

---

## Întrebarea: De ce este mai bun un raport de 1:1.5 (încetinire) decât 1:1?

Răspunsul stă în fizica motoarelor DC și în relația dintre **cuplu (torque)** și **viteză (speed)**. Propunerea de a pune pinionul mare la motor pentru ca roata să se tureze mai repede, deși pare logică, în realitate ar încetini dramatic sistemul.

### Analogia cu Mașina și Cutia de Viteze

Imaginați-vă sistemul de aruncare ca pe o mașină:
- **Motorul (REV HD Hex):** Motorul mașinii.
- **Roata de Aruncare (Flywheel):** Roata mașinii.
- **Transmisia (Lanț și Pinioane):** Cutia de viteze.
- **Timpul de Ambalare:** Accelerația mașinii de la 0 la 100 km/h.

Folosirea unui raport de **accelerare** (pinion mare la motor) este ca și cum ați încerca să plecați de pe loc în **viteza a cincea**. Motorul se va chinui și va accelera foarte lent, deoarece nu are suficient **cuplu (forță)** pentru a învinge inerția mașinii.

Folosirea unui raport de **încetinire** (pinion mic la motor, pinion mare la roată) este ca și cum ați pleca de pe loc în **viteza întâi**. Motorul se turează rapid într-o zonă unde produce multă putere, iar cutia de viteze (transmisia) transformă această turație într-un **cuplu uriaș** la roată. Mașina accelerează exploziv.

---

### Calculul Fizic: Puterea (Power) este Cheia

Puterea mecanică a unui motor este produsul dintre cuplu și viteză (`P = τ * ω`). Un motor DC nu livrează aceeași putere la toate turațiile. Curba de putere are o formă de parabolă, atingând un **vârf (Peak Power)** la aproximativ 50% din turația maximă.

**Sarcina:** Accelerarea sistemului de la 0 RPM (sau de la o turație joasă după o aruncare) la 3600 RPM. Această accelerare necesită o cantitate mare de energie într-un timp scurt, adică o **putere mare**.

- **Scenariul A (Raport 1:1 - Ineficient):**
  - Pentru a accelera roata, motorul trebuie să treacă prin zona sa de turație joasă (0-3000 RPM) unde are cuplu mare, dar apoi trebuie să continue să accelereze până la 3600 RPM, unde puterea sa deja a început să scadă față de vârf. Forța de accelerare scade pe măsură ce se apropie de turația țintă.

- **Scenariul B (Raport 1:1.5 - Încetinire, Recomandat):**
  - Motorul trebuie să accelereze până la `3600 * 1.5 = 5400 RPM`.
  - **Avantajul crucial:** Transmisia de 1:1.5 acționează ca o pârghie. Permite motorului să petreacă mai mult timp în **zona sa de putere maximă (~3000 RPM)** în timp ce livrează un cuplu *multiplicat* la ax. Chiar dacă motorul în sine trebuie să ajungă la o turație mai mare, sarcina "resimțită" de motor este mai mică datorită avantajului mecanic.

### Demonstrația prin Accelerație Unghiulară (`α`)

Accelerația unghiulară (`α`) ne spune cât de repede crește viteza de rotație. Formula este:

`α = Cuplu la Ax / Moment de Inerție`

Deoarece raportul de 1:1.5 livrează un **cuplu efectiv mai mare** la axul roții pe parcursul accelerației, în timp ce momentul de inerție al sistemului rămâne constant, **accelerația unghiulară (`α`) va fi semnificativ mai mare**.

### Concluzie

Raportul de încetinire permite motorului să opereze mai eficient, transformându-și viteza în cuplu brut la ax, ceea ce duce direct la o accelerație superioară. Timpul de recuperare după o aruncare (când turația scade brusc) va fi, de asemenea, mult mai scurt, deoarece motorul poate aplica o "lovitură" de cuplu mult mai puternică pentru a readuce sistemul la viteza optimă.