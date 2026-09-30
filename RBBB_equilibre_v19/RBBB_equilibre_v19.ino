// ================================================================
//  RBBB - Robot en equilibre sur 2 roues  (version Arduino 1.9 - commande par le Pi)
//  1.3.1 : sens du virage corrige
//  1.3.2 : coupure de l'alimentation du Pi a la fin de son extinction
//          (sinon le Pi 3 se rallume : GPIO3 = SCL du bus de l'IMU)
//  1.4   : mesures pour etudier l'oscillation. La ligne "E" est envoyee
//          50 fois par seconde (au lieu de 5) avec plus de details :
//          E i=<inclinaison> r=<0|1> g=<vitesse de chute deg/s>
//            v=<vitesse des roues pas/s> t=<millis du MM1>
//          L'equilibre lui-meme est IDENTIQUE a la version 1.3.2.
//  1.5   : filtre passe-bas sur la vitesse de chute utilisee par KD.
//          Mesure du 2026-09-28 : vibration a ~15 Hz entretenue par le
//          terme KD (le gyroscope capte la vibration, KD la renvoie aux
//          moteurs). Reglage en direct : "f <Hz>" = frequence de coupure,
//          "f 0" = pas de filtre (comportement de la 1.4, valeur au
//          demarrage). La ligne "R" contient aussi f=<Hz>.
//  1.6   : calibration du gyroscope protegee. Incident du 2026-09-28 :
//          deux redemarrages pendant qu'on manipulait le robot ont donne
//          un zero faux (-14 puis +9 degres/s) ; le robot partait d'un
//          cote et aucun angle d'equilibre ne convenait.
//          Pendant les 3 s de calibration, le MM1 verifie maintenant que
//          le gyroscope est stable ET que l'angle ne change pas ; sinon
//          il annonce "robot en mouvement" et recommence.
//          Nouvelle commande "c" : refait la calibration sans RESET,
//          seulement quand le robot est couche (equilibre inactif).
//  1.7   : plus de force et de vitesse pour se deplacer (2026-09-29).
//          Sur un tapis, le robot n'avancait presque pas : pour rouler,
//          il doit rester penche vers l'avant, et avec KV = 0,0007 la
//          boucle de vitesse ne le penchait que de 0,34 degre au plus.
//          - Terme INTEGRAL dans la boucle de vitesse, reglage en direct
//            "i <val>" (degres par pas de retard, 0 = comme la 1.6,
//            valeur au demarrage). Tant que le robot roule moins vite
//            que demande, l'angle vise augmente petit a petit jusqu'a
//            vaincre le tapis. Corrige aussi une petite erreur de
//            l'angle d'equilibre. Limite a +/- 3 degres, remis a zero
//            au depart et a chaque chute.
//          - DEPLACEMENT_MAX : 3200 pas/s a 100 % (2 tours/s) au lieu de 1600.
//          - La ligne "R" contient i=<val>, la ligne "E" contient
//            n=<angle ajoute par le terme integral, en degres>.
//  1.8   : ANTICIPATION (2026-09-29). Essai de la 1.7 : commandes
//          "molles", lentes a demarrer et a s'arreter ("un sous-marin").
//          Cause : le robot ne se penchait qu'une fois en retard sur la
//          consigne (constante de temps ~1,3 s avec KV = 0,0007).
//          Maintenant, pendant que la consigne accelere (rampe), le MM1
//          ajoute tout de suite :
//            - a l'angle vise, le penche qu'il faut pour cette
//              acceleration (1 degre ~ 1063 pas/s2, roues de 82 mm) ;
//            - aux roues, cette meme acceleration.
//          Reglage "e <0 a 2>" : 0 = aucune anticipation (comme la 1.7,
//          valeur au demarrage), 1 = valeur calculee.
//          Reglage "r <pas/s2>" : rampe d'acceleration de la consigne
//          (1000 en 1.7 ; 2000 au demarrage). La ligne "R" contient
//          e= et r=.
//  1.9   : RAMPE DE FREINAGE (2026-09-29). Le robot est difficile a
//          arreter avant un mur. Reglage "b <pas/s2>" : rampe utilisee
//          quand la consigne se rapproche de zero (commande relachee,
//          ralentissement, chien de garde). 0 = meme rampe qu'a
//          l'acceleration (comme la 1.8, valeur au demarrage).
//          Avec l'anticipation, 1063 pas/s2 = 1 degre de penche vers
//          l'arriere : limite a 8000 (7,5 degres, sous ANGLE_VISE_MAX).
//          La ligne "R" contient b=.
//  Robo HAT MM1 (SAMD51) + MPU9250 integre + 2 TB6600 (micropas 1/8)
// ================================================================
//
//  BRANCHEMENT (inchange) :
//    SERVO1 jaune -> PUL+ gauche     SERVO2 jaune -> DIR+ gauche
//    SERVO3 jaune -> PUL+ droit      SERVO4 jaune -> DIR+ droit
//    Pins noires  -> PUL- et DIR-    Pins rouges  : RIEN
//
//  PRINCIPE :
//    - Les impulsions STEP sont produites par une interruption de
//      minuterie (50 000 fois par seconde) : regularite parfaite.
//    - Le regulateur commande une ACCELERATION des roues. La vitesse
//      change donc progressivement : le moteur pas-a-pas ne perd pas
//      de pas et la correction est douce.
//    - Une deuxieme boucle surveille la vitesse des roues et incline
//      legerement l'angle vise pour amener le robot a la vitesse
//      demandee (0 = sur place).
//
//  NOUVEAU EN 1.3 : COMMANDE PAR LE RASPBERRY PI
//    Le Pi envoie ses ordres par le port serie du connecteur
//    (Serial1 : PA16/PA17 <-> GPIO15/GPIO14 du Pi, 115200 bauds).
//    - "m <vitesse> <virage>" (de -100 a 100 %) 10 fois par seconde.
//      Avancer/reculer : vitesse visee par la boucle de vitesse.
//      Tourner : une roue plus vite, l'autre moins vite.
//      Les changements de consigne suivent une rampe douce.
//    - CHIEN DE GARDE : sans ordre du Pi depuis 0,5 s, le robot
//      revient a l'arret sur place. Sans Pi, il se comporte donc
//      exactement comme la version 1.2.
//    - Les reglages p, d, v, a, ? sont aussi acceptes depuis le Pi.
//    - Le MM1 renvoie au Pi son inclinaison et ses reglages.
//    - "coupe-pi" : envoye par le Pi a la toute fin de son extinction.
//      3 s plus tard, le MM1 coupe l'alimentation 5 V du Pi (POWER_OFF).
//      L'equilibre continue. Pour rallumer le Pi : bouton POWER du MM1
//      ou couper/remettre la batterie.
//    ATTENTION : avec KV = 0, la boucle de vitesse est coupee et le
//    robot ne peut plus avancer ni reculer sur commande (il tourne
//    encore sur place).
//
//  DEROULEMENT :
//    1. Calibration du gyroscope : robot IMMOBILE 3 secondes (DEL allumee).
//       Si le robot bouge, la DEL clignote et la calibration recommence.
//    2. Attente : tenez le robot a son point d'equilibre. Les moteurs
//       s'activent apres une demi-seconde a moins de 3 degres.
//    3. Equilibre. Au-dela de 40 degres, les moteurs s'arretent.
//       Relevez le robot pour repartir.
//
//  REGLAGE EN DIRECT, dans le Moniteur serie (115200 bauds,
//  "Nouvelle ligne") ou depuis la page web du Pi :
//     p 1800   -> force de correction selon l'angle (KP)
//     d 500    -> amortissement selon la vitesse de chute (KD)
//     v 0.0015 -> boucle de vitesse (KV), 0 pour la desactiver
//     a 12.4   -> angle du point d'equilibre
//     f 10     -> filtre de la vitesse de chute pour KD, en Hz (0 = aucun)
//     i 0.0002 -> terme integral de la boucle de vitesse (KI), 0 = aucun
//     e 1      -> anticipation du penche et de l'acceleration (0 = aucune)
//     r 2000   -> rampe d'acceleration de la consigne, pas/s2
//     b 6000   -> rampe de freinage, pas/s2 (0 = comme r)
//     c       -> refaire la calibration du gyroscope (robot couche, immobile)
//     ?        -> affiche les valeurs actuelles
//  Les changements sont perdus au redemarrage du MM1. Les valeurs
//  "Enregistrees sur le Pi" depuis la page web lui sont renvoyees
//  automatiquement a chaque demarrage.
// ================================================================

#include <Arduino.h>
#include <Wire.h>

// Etat d'un moteur (defini ici, avant les fonctions, pour l'IDE Arduino)
struct Canal {
  volatile uint32_t increment;   // 0 = arret
  volatile bool avantVoulu;
  uint32_t phase;
  bool avantActuel;
  bool impulsionHaute;
  volatile uint32_t *outset, *outclr;
  uint32_t maskPul, maskDir;
  bool sensAvant;
};

Canal canal[2];

// Ligne de commande en cours de reception (une par port serie)
struct Tampon {
  char texte[40];
  uint8_t n;
  bool tropLong;
};

Tampon tamponUsb = {"", 0, false};
Tampon tamponPi  = {"", 0, false};

// ------------------------- REGLAGES -----------------------------

float ANGLE_EQUILIBRE = 12.4;   // angle X au point d'equilibre
const float SIGNE = -1.0;       // l'angle X diminue vers l'avant

const bool SENS_GAUCHE = false; // niveau de DIR pour avancer
const bool SENS_DROIT  = false;

float KP = 1800.0;   // pas/s2 par degre d'inclinaison
float KD = 500.0;    // pas/s2 par degre/s de vitesse de chute (valeur reglee)
float KV = 0.0015;   // degres d'inclinaison visee par pas/s d'ecart de vitesse
float FILTRE_D = 0;  // Hz, coupure du filtre de la vitesse de chute (0 = aucun)
float KI = 0;        // degres d'inclinaison visee par pas de retard accumule (0 = aucun)
float ANTICIPATION = 0;          // 0 = aucune, 1 = penche et acceleration calcules
float RAMPE_DEPLACEMENT = 2000;  // pas/s gagnes par seconde par la consigne
float RAMPE_FREINAGE = 0;        // pas/s perdus par seconde en freinant (0 = RAMPE_DEPLACEMENT)

const float VITESSE_MAX    = 8000.0;   // pas/s (1600 pas = 1 tour)
const float ACCEL_MAX      = 40000.0;  // pas/s2
const float ANGLE_VISE_MAX = 8.0;      // limite de la boucle de vitesse
const float INTEGRALE_MAX  = 3.0;      // degres, limite du terme integral
// Penche d'equilibre pendant une acceleration : tan(angle) = a / g.
// 1 pas = 82 mm x pi / 1600 = 0,161 mm ; 1 degre -> 9,81 x 0,01745 / 0,000161
const float PAS_S2_PAR_DEGRE = 1063.0;
const float ANGLE_CHUTE    = 40.0;
const float ANGLE_DEPART   = 3.0;

// Deplacement commande par le Pi
const float DEPLACEMENT_MAX   = 3200.0;  // pas/s a 100 % (2 tours de roue par seconde)
const float VIRAGE_MAX        = 800.0;   // pas/s ajoutes/retires a 100 %
const float RAMPE_VIRAGE      = 2000.0;  // pas/s gagnes par seconde
const uint32_t CHIEN_DE_GARDE_MS = 500;  // arret si plus d'ordre du Pi
const uint32_t PERIODE_ETAT_MS   = 20;   // ligne "E" vers le Pi : 50 fois par seconde

// Coupure de l'alimentation du Pi apres son extinction
const uint32_t DELAI_COUPURE_MS    = 3000;  // attente apres "coupe-pi"
const uint32_t IMPULSION_COUPURE_MS = 500;  // duree de POWER_OFF a HIGH

// Limites acceptees pour les reglages (protection contre une faute de frappe)
const float KP_MIN = 0, KP_MAX = 20000;
const float KD_MIN = 0, KD_MAX = 5000;
const float KV_MIN = 0, KV_MAX = 0.05;
const float ANGLE_MIN = -30, ANGLE_MAX = 30;
const float FILTRE_MIN = 0, FILTRE_MAX = 100;   // Hz (0 = aucun filtre)
const float KI_MIN = 0, KI_MAX = 0.01;
const float ANTICIPATION_MIN = 0, ANTICIPATION_MAX = 2;
const float RAMPE_MIN = 200, RAMPE_MAX = 8000;   // pas/s2
const float FREINAGE_MIN = 0, FREINAGE_MAX = 8000;  // pas/s2 (0 = comme la rampe)

// Calibration du gyroscope : conditions pour accepter la mesure du zero
const uint32_t CALIB_DUREE_MS = 3000;
const float CALIB_ECART_MAX = 1.0;   // deg/s : ecart-type du gyroscope (repos : ~0,3)
const float CALIB_ANGLE_MAX = 1.0;   // deg : variation de l'angle (accelerometre)
                                     // entre la premiere et la derniere demi-seconde
const float CALIB_ZERO_NORMAL = 10;  // deg/s : au-dela, avertissement seulement

// ----------------------------------------------------------------

#define PIN_PUL_G  SERVO1
#define PIN_DIR_G  SERVO2
#define PIN_PUL_D  SERVO3
#define PIN_DIR_D  SERVO4

const uint32_t PERIODE_BOUCLE_US = 5000;   // 200 corrections par seconde
const float    DT = PERIODE_BOUCLE_US / 1000000.0;
const float    FREQ_ISR = 50000.0;         // interruptions par seconde
const float    ALPHA = 0.996;              // poids du gyroscope

// ------------------------- IMU ----------------------------------

TwoWire *bus = nullptr;
uint8_t adresseImu = 0;
float decalageGyro = 0;

void ecrireImu(uint8_t reg, uint8_t valeur) {
  bus->beginTransmission(adresseImu);
  bus->write(reg);
  bus->write(valeur);
  bus->endTransmission();
}

bool lireImu(float &ay, float &az, float &gx) {
  bus->beginTransmission(adresseImu);
  bus->write(0x3B);
  if (bus->endTransmission(false) != 0) return false;
  if (bus->requestFrom(adresseImu, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = bus->read();
  int16_t ryA = (b[2] << 8) | b[3];
  int16_t rzA = (b[4] << 8) | b[5];
  int16_t rxG = (b[8] << 8) | b[9];
  ay = ryA / 8192.0;   // +/- 4 g
  az = rzA / 8192.0;
  gx = rxG / 65.5;     // +/- 500 degres/s
  return true;
}

// L'IMU est sur le bus interne Wire (SERCOM3). On ne cherche plus
// sur Wire1 : Wire1 utilise SERCOM1, le meme circuit que Serial1
// (le lien avec le Pi), et le demarrer couperait ce lien.
bool chercherSurBus(TwoWire *w) {
  w->begin();
  w->setClock(400000);
  for (uint8_t a = 0x68; a <= 0x69; a++) {
    w->beginTransmission(a);
    if (w->endTransmission() == 0) {
      bus = w;
      adresseImu = a;
      return true;
    }
  }
  return false;
}

// ------------------------- MOTEURS ------------------------------
// Generateur d'impulsions par accumulateur de phase : a chaque
// interruption, on ajoute l'increment ; au debordement, un pas.

void preparerCanal(Canal &c, uint8_t pinPul, uint8_t pinDir, bool sensAvant) {
  pinMode(pinPul, OUTPUT);
  pinMode(pinDir, OUTPUT);
  digitalWrite(pinPul, LOW);
  digitalWrite(pinDir, sensAvant ? HIGH : LOW);
  // les 4 broches sont sur le port A (PA18 a PA21)
  c.outset = &PORT->Group[g_APinDescription[pinPul].ulPort].OUTSET.reg;
  c.outclr = &PORT->Group[g_APinDescription[pinPul].ulPort].OUTCLR.reg;
  c.maskPul = 1ul << g_APinDescription[pinPul].ulPin;
  c.maskDir = 1ul << g_APinDescription[pinDir].ulPin;
  c.sensAvant = sensAvant;
  c.increment = 0;
  c.avantVoulu = true;
  c.avantActuel = true;
  c.phase = 0;
  c.impulsionHaute = false;
}

inline void traiterCanal(Canal &c) {
  if (c.impulsionHaute) {
    *c.outclr = c.maskPul;
    c.impulsionHaute = false;
    return;
  }
  uint32_t inc = c.increment;
  if (inc == 0) return;
  bool voulu = c.avantVoulu;
  if (voulu != c.avantActuel) {
    // changer DIR et attendre la prochaine interruption avant le pas
    bool niveau = voulu ? c.sensAvant : !c.sensAvant;
    if (niveau) *c.outset = c.maskDir; else *c.outclr = c.maskDir;
    c.avantActuel = voulu;
    return;
  }
  uint32_t avant = c.phase;
  c.phase += inc;
  if (c.phase < avant) {           // debordement = un pas
    *c.outset = c.maskPul;
    c.impulsionHaute = true;
  }
}

void TC3_Handler() {
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
  traiterCanal(canal[0]);
  traiterCanal(canal[1]);
}

void demarrerMinuterie() {
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3;
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN; // 48 MHz
  while (!(GCLK->PCHCTRL[TC3_GCLK_ID].reg & GCLK_PCHCTRL_CHEN));

  TC3->COUNT16.CTRLA.bit.ENABLE = 0;
  while (TC3->COUNT16.SYNCBUSY.bit.ENABLE);
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC3->COUNT16.SYNCBUSY.bit.SWRST);

  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1;
  TC3->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
  TC3->COUNT16.CC[0].reg = (uint16_t)(48000000 / (uint32_t)FREQ_ISR - 1);
  while (TC3->COUNT16.SYNCBUSY.bit.CC0);

  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  NVIC_SetPriority(TC3_IRQn, 0);
  NVIC_EnableIRQ(TC3_IRQn);

  TC3->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC3->COUNT16.SYNCBUSY.bit.ENABLE);
}

void vitesseMoteur(Canal &c, float pasParSeconde) {
  bool avant = pasParSeconde >= 0;
  float f = fabsf(pasParSeconde);
  uint32_t inc = 0;
  if (f >= 5.0) inc = (uint32_t)(f / FREQ_ISR * 4294967296.0);
  noInterrupts();
  c.avantVoulu = avant;
  c.increment = inc;
  interrupts();
}

void arreterMoteurs() {
  vitesseMoteur(canal[0], 0);
  vitesseMoteur(canal[1], 0);
}

// ------------------------- COMMANDES ----------------------------

// Consignes demandees par le Pi (pas/s) et consignes adoucies par la rampe
float vitesseCible = 0, virageCible = 0;
float vitesseConsigne = 0, virageConsigne = 0;
uint32_t dernierOrdrePi = 0;

// Commande "c" : calibration demandee, faite par loop() si le robot est couche
bool calibrationDemandee = false;

// Coupure de l'alimentation du Pi (sans bloquer l'equilibre)
bool coupurePrevue = false;
bool impulsionCoupure = false;
uint32_t momentCoupure = 0;

void gererCoupurePi() {
  if (coupurePrevue && millis() - momentCoupure >= DELAI_COUPURE_MS) {
    coupurePrevue = false;
    impulsionCoupure = true;
    momentCoupure = millis();
    digitalWrite(POWER_OFF, HIGH);
    Serial.println(">>> Alimentation du Pi coupee.");
  }
  if (impulsionCoupure && millis() - momentCoupure >= IMPULSION_COUPURE_MS) {
    impulsionCoupure = false;
    digitalWrite(POWER_OFF, LOW);   // pret pour un rallumage par le bouton POWER
  }
}

// Message pour la console USB et, precede de "I ", pour le Pi
void annoncer(const char *texte) {
  Serial.println(texte);
  Serial1.print("I ");
  Serial1.println(texte);
}

void afficherReglages() {
  Serial.print("--- KP=");  Serial.print(KP, 1);
  Serial.print("  KD=");     Serial.print(KD, 1);
  Serial.print("  KV=");     Serial.print(KV, 5);
  Serial.print("  ANGLE_EQUILIBRE="); Serial.print(ANGLE_EQUILIBRE, 2);
  Serial.print("  FILTRE_D=");        Serial.print(FILTRE_D, 1);
  Serial.print(" Hz  KI=");           Serial.print(KI, 6);
  Serial.print("  ANTICIPATION=");    Serial.print(ANTICIPATION, 2);
  Serial.print("  RAMPE=");           Serial.print(RAMPE_DEPLACEMENT, 0);
  Serial.print("  FREINAGE=");        Serial.print(RAMPE_FREINAGE, 0);
  Serial.println(" ---");
}

void envoyerReglagesPi() {
  Serial1.print("R p=");  Serial1.print(KP, 1);
  Serial1.print(" d=");   Serial1.print(KD, 1);
  Serial1.print(" v=");   Serial1.print(KV, 5);
  Serial1.print(" a=");   Serial1.print(ANGLE_EQUILIBRE, 2);
  Serial1.print(" f=");   Serial1.print(FILTRE_D, 1);
  Serial1.print(" i=");   Serial1.print(KI, 6);
  Serial1.print(" e=");   Serial1.print(ANTICIPATION, 2);
  Serial1.print(" r=");   Serial1.print(RAMPE_DEPLACEMENT, 0);
  Serial1.print(" b=");   Serial1.println(RAMPE_FREINAGE, 0);
}

// Lit un nombre ; faux s'il n'y en a pas
bool lireNombre(const char *texte, float &valeur) {
  char *fin;
  double v = strtod(texte, &fin);
  if (fin == texte) return false;
  valeur = (float)v;
  return true;
}

bool dansLimites(float v, float mini, float maxi) {
  return v >= mini && v <= maxi;
}

void executer(char *ligne, bool depuisPi) {
  while (*ligne == ' ') ligne++;
  if (*ligne == 0) return;

  // Fin d'extinction du Pi : mot exact, accepte seulement depuis le Pi
  if (strcmp(ligne, "coupe-pi") == 0) {
    if (!depuisPi) {
      Serial.println("coupe-pi : commande reservee au Pi (utilisez le bouton de la page).");
    } else if (!coupurePrevue && !impulsionCoupure) {
      coupurePrevue = true;
      momentCoupure = millis();
      Serial.println(">>> Le Pi est eteint : coupure de son alimentation dans 3 s.");
    }
    return;
  }

  char cle = tolower(ligne[0]);
  char *reste = ligne + 1;

  // Ordre de deplacement : "m <vitesse %> <virage %>"
  if (cle == 'm') {
    char *fin1, *fin2;
    long v = strtol(reste, &fin1, 10);
    long t = strtol(fin1, &fin2, 10);
    if (fin1 == reste || fin2 == fin1) return;   // ligne abimee : ignoree
    v = constrain(v, -100, 100);
    t = constrain(t, -100, 100);
    vitesseCible = v / 100.0 * DEPLACEMENT_MAX;
    virageCible  = t / 100.0 * VIRAGE_MAX;
    dernierOrdrePi = millis();
    return;
  }

  if (cle == '?') {
    if (depuisPi) envoyerReglagesPi(); else afficherReglages();
    return;
  }

  // Calibration du gyroscope : "c" seul (une ligne abimee ne la declenche pas)
  if (cle == 'c' && (reste[0] == 0 || reste[0] == ' ')) {
    calibrationDemandee = true;     // faite par loop(), seulement robot couche
    return;
  }

  float valeur;
  bool connue = (cle == 'p' || cle == 'd' || cle == 'v' || cle == 'a' || cle == 'f' || cle == 'i'
                 || cle == 'e' || cle == 'r' || cle == 'b');
  if (!connue || !lireNombre(reste, valeur)) {
    // Une ligne abimee venant du Pi est simplement ignoree
    if (!depuisPi) { Serial.print("Commande inconnue : "); Serial.println(ligne); }
    return;
  }

  bool ok = false;
  if (cle == 'p' && dansLimites(valeur, KP_MIN, KP_MAX))            { KP = valeur; ok = true; }
  else if (cle == 'd' && dansLimites(valeur, KD_MIN, KD_MAX))       { KD = valeur; ok = true; }
  else if (cle == 'v' && dansLimites(valeur, KV_MIN, KV_MAX))       { KV = valeur; ok = true; }
  else if (cle == 'a' && dansLimites(valeur, ANGLE_MIN, ANGLE_MAX)) { ANGLE_EQUILIBRE = valeur; ok = true; }
  else if (cle == 'f' && dansLimites(valeur, FILTRE_MIN, FILTRE_MAX)) { FILTRE_D = valeur; ok = true; }
  else if (cle == 'i' && dansLimites(valeur, KI_MIN, KI_MAX))       { KI = valeur; ok = true; }
  else if (cle == 'e' && dansLimites(valeur, ANTICIPATION_MIN, ANTICIPATION_MAX)) { ANTICIPATION = valeur; ok = true; }
  else if (cle == 'r' && dansLimites(valeur, RAMPE_MIN, RAMPE_MAX)) { RAMPE_DEPLACEMENT = valeur; ok = true; }
  else if (cle == 'b' && dansLimites(valeur, FREINAGE_MIN, FREINAGE_MAX)) { RAMPE_FREINAGE = valeur; ok = true; }

  if (ok) {
    afficherReglages();     // console USB
    envoyerReglagesPi();    // page web toujours a jour
  } else {
    Serial.print("Valeur hors limites, refusee : ");
    Serial.println(ligne);
    if (depuisPi) { Serial1.print("I Valeur refusee : "); Serial1.println(ligne); }
  }
}

void lireFlux(Stream &s, Tampon &t, bool depuisPi) {
  while (s.available()) {
    char c = s.read();
    if (c == '\n' || c == '\r') {
      t.texte[t.n] = 0;
      if (!t.tropLong && t.n > 0) executer(t.texte, depuisPi);
      t.n = 0;
      t.tropLong = false;
    } else if (t.n < sizeof(t.texte) - 1) {
      t.texte[t.n++] = c;
    } else {
      t.tropLong = true;    // ligne trop longue : ignoree en entier
    }
  }
}

// Rapproche "valeur" de "cible" d'au plus "pas"
float rampe(float valeur, float cible, float pas) {
  if (valeur < cible) return min(valeur + pas, cible);
  if (valeur > cible) return max(valeur - pas, cible);
  return valeur;
}

// ------------------------- PROGRAMME ----------------------------

enum Etat { ATTENTE, EQUILIBRE };
Etat etat = ATTENTE;

float angleX = 0;
float vitesse = 0;          // pas/s commandes aux roues (partie commune)
float vitesseFiltree = 0;
float tauxFiltre = 0;       // vitesse de chute filtree, pour KD
float integrale = 0;        // degres ajoutes a l'angle vise par le terme integral
uint32_t prochainTour = 0;
uint32_t debutVertical = 0;
bool verticalEnCours = false;
uint32_t dernierAffichage = 0;
uint32_t dernierEtatPi = 0;

void clignoterErreur(int periodeMs) {
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(periodeMs);
  }
}

// Une mesure du zero du gyroscope (3 s, DEL allumee).
// Vrai si le robot est reste immobile : gyroscope stable et angle constant.
bool mesurerZeroGyro(float &zero, float &ecart, float &variationAngle) {
  digitalWrite(LED_BUILTIN, HIGH);
  double somme = 0, sommeCarres = 0;
  float angleDebut = 0, angleFin = 0;
  int nb = 0, nbDebut = 0, nbFin = 0;
  uint32_t debut = millis();
  while (millis() - debut < CALIB_DUREE_MS) {
    float ay, az, gx;
    if (lireImu(ay, az, gx)) {
      somme += gx;
      sommeCarres += (double)gx * gx;
      nb++;
      float angle = atan2f(ay, az) * RAD_TO_DEG;
      uint32_t ecoule = millis() - debut;
      if (ecoule < 500) { angleDebut += angle; nbDebut++; }
      else if (ecoule >= CALIB_DUREE_MS - 500) { angleFin += angle; nbFin++; }
    }
    delay(3);
  }
  digitalWrite(LED_BUILTIN, LOW);
  if (nb < 100 || nbDebut == 0 || nbFin == 0) {
    zero = 0; ecart = 999; variationAngle = 999;
    return false;
  }
  zero = somme / nb;
  ecart = sqrt(max(0.0, sommeCarres / nb - (double)zero * zero));
  variationAngle = fabsf(angleFin / nbFin - angleDebut / nbDebut);
  return ecart <= CALIB_ECART_MAX && variationAngle <= CALIB_ANGLE_MAX;
}

// Calibration complete : recommence tant que le robot bouge.
void calibrerGyro() {
  arreterMoteurs();
  annoncer("Calibration : robot IMMOBILE 3 secondes...");
  float zero, ecart, variation;
  while (!mesurerZeroGyro(zero, ecart, variation)) {
    String texte = "Robot en mouvement (gyro " + String(ecart, 2) + " deg/s, angle "
                   + String(variation, 2) + " deg) : on recommence. Ne le touchez pas.";
    annoncer(texte.c_str());
    for (int k = 0; k < 10; k++) {             // 1 s de clignotement rapide
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }
  decalageGyro = zero;
  String texte = "Calibration terminee : zero du gyroscope " + String(zero, 2)
                 + " deg/s (stabilite " + String(ecart, 2) + ").";
  annoncer(texte.c_str());
  if (fabsf(zero) > CALIB_ZERO_NORMAL) {
    annoncer("Attention : zero du gyroscope inhabituel. Refaites la calibration (c).");
  }

  // angle de depart d'apres l'accelerometre
  float ay, az, gx;
  if (lireImu(ay, az, gx)) angleX = atan2f(ay, az) * RAD_TO_DEG;
  tauxFiltre = 0;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Allume automatiquement l'alimentation 5 V du Raspberry Pi
  // (meme effet qu'un appui sur le bouton POWER de la carte).
  // Necessite une tension sur le bornier IN (12 V) : l'USB seul
  // n'alimente pas le Pi.
  pinMode(POWER_OFF, OUTPUT);
  digitalWrite(POWER_OFF, LOW);
  pinMode(BUTTON, OUTPUT);
  digitalWrite(BUTTON, HIGH);
  delay(100);
  pinMode(BUTTON, INPUT);

  Serial.begin(115200);     // console USB
  Serial1.begin(115200);    // lien avec le Raspberry Pi

  preparerCanal(canal[0], PIN_PUL_G, PIN_DIR_G, SENS_GAUCHE);
  preparerCanal(canal[1], PIN_PUL_D, PIN_DIR_D, SENS_DROIT);
  demarrerMinuterie();

  delay(1500);   // laisser le temps d'ouvrir le Moniteur serie
  Serial.println();
  annoncer("RBBB - equilibre (Arduino v1.9)");

  if (!chercherSurBus(&Wire)) {
    annoncer("ERREUR : IMU introuvable sur Wire.");
    clignoterErreur(100);
  }
  Serial.print("IMU trouvee sur Wire, adresse 0x");
  Serial.println(adresseImu, HEX);

  ecrireImu(0x6B, 0x80); delay(100);   // remise a zero
  ecrireImu(0x6B, 0x01); delay(100);   // reveil
  ecrireImu(0x1A, 0x03);               // filtre passe-bas gyro ~41 Hz
  ecrireImu(0x1B, 0x08);               // gyro +/- 500 degres/s
  ecrireImu(0x1C, 0x08);               // accel +/- 4 g
  ecrireImu(0x1D, 0x03);               // filtre passe-bas accel ~41 Hz
  delay(100);

  calibrerGyro();          // recommence tant que le robot bouge
  afficherReglages();

  // Oublier les ordres du Pi recus pendant la calibration,
  // puis annoncer au Pi que le MM1 est pret (il renvoie ses reglages).
  while (Serial1.available()) Serial1.read();
  Serial1.println("DEMARRAGE");
  envoyerReglagesPi();

  annoncer("Tenez le robot a son point d'equilibre pour demarrer.");
  prochainTour = micros();
}

void loop() {
  lireFlux(Serial, tamponUsb, false);
  lireFlux(Serial1, tamponPi, true);
  gererCoupurePi();

  // Calibration demandee par "c" : seulement si l'equilibre est inactif
  if (calibrationDemandee) {
    calibrationDemandee = false;
    if (etat == EQUILIBRE) {
      annoncer("Calibration refusee : couchez d'abord le robot.");
    } else {
      calibrerGyro();              // bloque environ 3 s (moteurs arretes)
      verticalEnCours = false;
      while (Serial1.available()) Serial1.read();   // ordres recus pendant ce temps
      tamponPi.n = 0;
      tamponPi.tropLong = false;
      annoncer("Tenez le robot a son point d'equilibre pour demarrer.");
      prochainTour = micros();
    }
  }

  // cadence fixe de 200 tours par seconde
  if ((int32_t)(micros() - prochainTour) < 0) return;
  prochainTour += PERIODE_BOUCLE_US;

  // Chien de garde : plus d'ordre du Pi -> arret sur place
  if (millis() - dernierOrdrePi > CHIEN_DE_GARDE_MS) {
    vitesseCible = 0;
    virageCible = 0;
  }

  float ay, az, gx;
  if (!lireImu(ay, az, gx)) return;

  float taux = gx - decalageGyro;
  float angleAccel = atan2f(ay, az) * RAD_TO_DEG;
  angleX = ALPHA * (angleX + taux * DT) + (1.0 - ALPHA) * angleAccel;

  // inclinaison positive vers l'avant, 0 au point d'equilibre
  float inclinaison = SIGNE * (angleX - ANGLE_EQUILIBRE);
  float tauxAvant = SIGNE * taux;

  // filtre passe-bas du premier ordre sur la vitesse de chute (pour KD) :
  // ecarte les vibrations rapides que KD renverrait aux moteurs
  if (FILTRE_D > 0) {
    float k = 1.0 - expf(-2.0 * PI * FILTRE_D * DT);
    tauxFiltre += k * (tauxAvant - tauxFiltre);
  } else {
    tauxFiltre = tauxAvant;
  }

  if (etat == ATTENTE) {
    vitesseConsigne = 0;
    virageConsigne = 0;
    if (fabsf(inclinaison) < ANGLE_DEPART) {
      if (!verticalEnCours) { verticalEnCours = true; debutVertical = millis(); }
      else if (millis() - debutVertical > 500) {
        etat = EQUILIBRE;
        vitesse = 0;
        vitesseFiltree = 0;
        integrale = 0;
        digitalWrite(LED_BUILTIN, HIGH);
        annoncer(">>> Equilibre ACTIF");
      }
    } else {
      verticalEnCours = false;
    }
  } else {
    if (fabsf(inclinaison) > ANGLE_CHUTE) {
      arreterMoteurs();
      vitesse = 0;
      integrale = 0;
      vitesseConsigne = 0;
      virageConsigne = 0;
      etat = ATTENTE;
      verticalEnCours = false;
      digitalWrite(LED_BUILTIN, LOW);
      annoncer(">>> CHUTE - moteurs arretes. Relevez le robot.");
    } else {
      // consignes du Pi, adoucies par une rampe
      float consigneAvant = vitesseConsigne;
      // freinage : la consigne se rapproche de zero (ou change de sens)
      bool freinage = (vitesseConsigne > 0 && vitesseCible < vitesseConsigne)
                   || (vitesseConsigne < 0 && vitesseCible > vitesseConsigne);
      float pente = (freinage && RAMPE_FREINAGE > 0) ? RAMPE_FREINAGE : RAMPE_DEPLACEMENT;
      if (freinage && (vitesseCible > 0) != (vitesseConsigne > 0) && vitesseCible != 0) {
        // changement de sens : freiner jusqu'a zero seulement dans ce tour,
        // l'acceleration dans l'autre sens suit la rampe normale
        vitesseConsigne = rampe(vitesseConsigne, 0, pente * DT);
      } else {
        vitesseConsigne = rampe(vitesseConsigne, vitesseCible, pente * DT);
      }
      // acceleration demandee par la rampe en ce moment (pas/s2)
      float accelConsigne = (vitesseConsigne - consigneAvant) / DT;
      virageConsigne  = rampe(virageConsigne, virageCible, RAMPE_VIRAGE * DT);

      // boucle de vitesse : si le robot roule plus vite que demande,
      // on vise un leger angle vers l'arriere pour le ralentir
      // (et inversement pour le faire avancer)
      vitesseFiltree = 0.95 * vitesseFiltree + 0.05 * vitesse;
      float ecartVitesse = vitesseFiltree - vitesseConsigne;

      // terme integral : le retard (ou l'avance) accumule penche le robot
      // de plus en plus, jusqu'a vaincre le frottement du tapis
      if (KI > 0) {
        integrale += KI * ecartVitesse * DT;
        integrale = constrain(integrale, -INTEGRALE_MAX, INTEGRALE_MAX);
      } else {
        integrale = 0;
      }

      // anticipation : pencher tout de suite de l'angle qu'il faut pour
      // suivre la rampe (vers l'avant pour accelerer, vers l'arriere
      // pour freiner), au lieu d'attendre d'etre en retard
      float angleVise = -KV * ecartVitesse - integrale
                        + ANTICIPATION * accelConsigne / PAS_S2_PAR_DEGRE;
      angleVise = constrain(angleVise, -ANGLE_VISE_MAX, ANGLE_VISE_MAX);

      // boucle d'angle : commande une acceleration des roues
      float accel = KP * (inclinaison - angleVise) + KD * tauxFiltre
                    + ANTICIPATION * accelConsigne;
      accel = constrain(accel, -ACCEL_MAX, ACCEL_MAX);
      vitesse += accel * DT;
      vitesse = constrain(vitesse, -VITESSE_MAX, VITESSE_MAX);

      // virage vers la droite (virageConsigne > 0) : la roue droite va
      // plus vite, la gauche moins vite (sens verifie sur le robot)
      float gauche = constrain(vitesse - virageConsigne, -VITESSE_MAX, VITESSE_MAX);
      float droite = constrain(vitesse + virageConsigne, -VITESSE_MAX, VITESSE_MAX);
      vitesseMoteur(canal[0], gauche);
      vitesseMoteur(canal[1], droite);
    }
  }

  // Etat pour le Pi, 50 fois par seconde (environ 45 octets par ligne :
  // 2,3 ko/s, soit 20 % de la capacite du lien a 115200 bauds)
  if (millis() - dernierEtatPi >= PERIODE_ETAT_MS) {
    dernierEtatPi = millis();
    Serial1.print("E i=");
    Serial1.print(inclinaison, 2);
    Serial1.print(" r=");
    Serial1.print(etat == EQUILIBRE ? 1 : 0);
    Serial1.print(" g=");
    Serial1.print(tauxAvant, 1);
    Serial1.print(" v=");
    Serial1.print((int)(etat == EQUILIBRE ? vitesse : 0));
    Serial1.print(" t=");
    Serial1.print(millis());
    Serial1.print(" n=");
    Serial1.println(integrale, 2);
  }

  if (millis() - dernierAffichage >= 500) {
    dernierAffichage = millis();
    Serial.print("Inclinaison : ");
    Serial.print(inclinaison, 1);
    Serial.print("   Vitesse : ");
    Serial.print((int)(etat == EQUILIBRE ? vitesse : 0));
    Serial.print(" pas/s   Consigne : ");
    Serial.print((int)vitesseConsigne);
    Serial.print(" / virage ");
    Serial.print((int)virageConsigne);
    Serial.print("   ");
    Serial.println(etat == EQUILIBRE ? "ACTIF" : "attente");
  }
}
