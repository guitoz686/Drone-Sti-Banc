#include <Servo.h>
#include <HX711.h>   // Bibliotheque "HX711" de Bogdan Necula (bogde) - Gestionnaire de bibliotheques
#include <EEPROM.h>

// ================= REGLAGES UTILISATEUR =================

// Broche reliee au fil signal de l'ESC
const int ESC_PIN = 9;

// Plage PWM de l'ESC (1000 = arret, 2000 = pleine puissance)
const int MIN_US = 1000;
const int MAX_US = 2000;

// Rampe : toutes les RAMP_INTERVAL_MS ms, on bouge de RAMP_STEP_US us
const unsigned long RAMP_INTERVAL_MS = 20;
const int RAMP_STEP_US = 10;

// ---- Capteur de force (cellule de charge + module HX711) ----
// Branchement : HX711 DT -> broche 3, HX711 SCK -> broche 4, VCC -> 5V, GND -> GND
const int HX_DOUT_PIN = 3;
const int HX_SCK_PIN  = 4;

// Facteur de calibration par defaut (unites brutes par gramme).
// Il sera ecrase par la valeur sauvegardee en EEPROM apres la commande "cal".
const float DEFAULT_CAL_FACTOR = 420.0f;

// Pesanteur terrestre pour convertir grammes-force en Newtons
const float GRAVITY = 9.80665f;

// Intervalle d'affichage automatique de la force (commande "stream on")
const unsigned long STREAM_INTERVAL_MS = 200;

// Filtrage de la mesure (0.0 a 1.0) : petit = tres lisse, grand = reactif
const float FILTER_ALPHA = 0.3f;

// =========================================================

struct CalData {
  uint16_t magic;
  float factor;
};

const uint16_t CAL_MAGIC = 0xF0CE;
const int EEPROM_ADDR = 0;

Servo esc;
HX711 scale;

bool armed = false;

int targetUs = MIN_US;
int currentUs = MIN_US;

unsigned long lastRampMs = 0;

String inputBuffer;

// ---- Etat de la mesure de force ----
bool scaleOk = false;          // capteur detecte
float calFactor = DEFAULT_CAL_FACTOR;
float filteredG = 0.0f;        // force filtree en grammes-force
float maxG = 0.0f;             // force maximale mesuree
bool forceInit = false;
bool streaming = false;
unsigned long lastStreamMs = 0;

// ================= ESC =================

void setTargetUs(int us) {
  targetUs = constrain(us, MIN_US, MAX_US);
}

void setTargetPercent(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);

  int us = (int)(MIN_US + (percent * (MAX_US - MIN_US) / 100.0f) + 0.5f);
  setTargetUs(us);
}

void writeCurrentUs() {
  esc.writeMicroseconds(currentUs);
}

void disarm() {
  armed = false;
  targetUs = MIN_US;
  currentUs = MIN_US;
  writeCurrentUs();
}

void armEsc() {
  disarm();

  Serial.println(F("Sending minimum throttle for arming..."));
  Serial.println(F("If your ESC requires min throttle at power-up, power the ESC now."));

  // Laisse le temps a l'ESC de voir le minimum.
  delay(3000);

  armed = true;
  writeCurrentUs();

  Serial.println(F("Armed."));
  Serial.println(F("Try: speed 0, speed 5, speed 10, stop, disarm"));
}

// ================= FORCE =================

float gramsToNewtons(float g) {
  return g * GRAVITY / 1000.0f;
}

void saveCalibration() {
  CalData data;
  data.magic = CAL_MAGIC;
  data.factor = calFactor;
  EEPROM.put(EEPROM_ADDR, data);
}

void loadCalibration() {
  CalData data;
  EEPROM.get(EEPROM_ADDR, data);

  if (data.magic == CAL_MAGIC && !isnan(data.factor) && data.factor != 0.0f) {
    calFactor = data.factor;
    Serial.println(F("Calibration chargee depuis l'EEPROM."));
  } else {
    calFactor = DEFAULT_CAL_FACTOR;
    Serial.println(F("Pas de calibration sauvegardee: facteur par defaut (imprecis). Tapez: cal <poids_en_g>"));
  }
}

void resetForceMeasure() {
  forceInit = false;
  filteredG = 0.0f;
  maxG = 0.0f;
}

// Le moteur doit etre a l'arret pour tarer ou calibrer.
bool motorIsIdle() {
  return currentUs <= MIN_US && targetUs <= MIN_US;
}

void tareScale() {
  if (!scaleOk) {
    Serial.println(F("Capteur de force non detecte."));
    return;
  }
  if (!motorIsIdle()) {
    Serial.println(F("Impossible de tarer: moteur en marche. Tapez stop d'abord."));
    return;
  }

  Serial.println(F("Tare en cours, ne touchez pas au banc..."));
  scale.tare(10);
  resetForceMeasure();
  Serial.println(F("Tare faite (force = 0)."));
}

// Calibration : poser un poids connu (en grammes) sur le capteur, puis "cal <poids>"
void calibrateScale(float knownG) {
  if (!scaleOk) {
    Serial.println(F("Capteur de force non detecte."));
    return;
  }
  if (!motorIsIdle()) {
    Serial.println(F("Impossible de calibrer: moteur en marche."));
    return;
  }
  if (knownG <= 0.0f) {
    Serial.println(F("Usage: cal <poids_connu_en_grammes>  (ex: cal 500)"));
    return;
  }

  Serial.println(F("Calibration en cours, gardez le poids immobile..."));

  // Valeur brute moyenne moins l'offset de tare
  float raw = scale.get_value(15);
  float newFactor = raw / knownG;

  if (newFactor == 0.0f || isnan(newFactor)) {
    Serial.println(F("Calibration impossible (signal nul). Verifiez le cablage et le poids pose."));
    return;
  }

  calFactor = newFactor;
  scale.set_scale(calFactor);
  saveCalibration();
  resetForceMeasure();

  Serial.print(F("Calibration OK. Facteur = "));
  Serial.println(calFactor, 4);
  Serial.println(F("Retirez le poids. Si besoin, retapez tare."));
}

// Lecture non bloquante : ne lit que si le HX711 a une mesure prete.
void updateForce() {
  if (!scaleOk) {
    return;
  }
  if (!scale.is_ready()) {
    return;
  }

  float g = scale.get_units(1);

  if (!forceInit) {
    filteredG = g;
    forceInit = true;
  } else {
    filteredG += FILTER_ALPHA * (g - filteredG);
  }

  if (filteredG > maxG) {
    maxG = filteredG;
  }
}

void printForce() {
  if (!scaleOk) {
    Serial.println(F("force: capteur non detecte"));
    return;
  }

  Serial.print(F("force="));
  Serial.print(filteredG, 1);
  Serial.print(F(" g ("));
  Serial.print(gramsToNewtons(filteredG), 3);
  Serial.print(F(" N), max="));
  Serial.print(maxG, 1);
  Serial.print(F(" g ("));
  Serial.print(gramsToNewtons(maxG), 3);
  Serial.println(F(" N)"));
}

void streamForce() {
  if (!streaming) {
    return;
  }

  unsigned long now = millis();
  if (now - lastStreamMs < STREAM_INTERVAL_MS) {
    return;
  }
  lastStreamMs = now;

  float pct = (float)(currentUs - MIN_US) * 100.0f / (MAX_US - MIN_US);

  Serial.print(F("t="));
  Serial.print(now);
  Serial.print(F(" ms | "));
  Serial.print(currentUs);
  Serial.print(F(" us | "));
  Serial.print(pct, 1);
  Serial.print(F(" % | "));
  Serial.print(filteredG, 1);
  Serial.print(F(" g | "));
  Serial.print(gramsToNewtons(filteredG), 3);
  Serial.println(F(" N"));
}

// ================= AFFICHAGE / COMMANDES =================

void printStatus() {
  Serial.print(F("armed="));
  if (armed) {
    Serial.print(F("yes"));
  } else {
    Serial.print(F("no"));
  }

  Serial.print(F(", current="));
  Serial.print(currentUs);
  Serial.print(F(" us, target="));
  Serial.print(targetUs);
  Serial.print(F(" us, percent="));

  float pct = (float)(targetUs - MIN_US) * 100.0f / (MAX_US - MIN_US);
  Serial.println(pct, 1);

  printForce();
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F(" help              show this help"));
  Serial.println(F(" arm               arm ESC at minimum throttle"));
  Serial.println(F(" disarm            set minimum and disable output"));
  Serial.println(F(" stop              set target to minimum"));
  Serial.println(F(" speed 0-100       set speed percent"));
  Serial.println(F(" us 1000-2000      set pulse width directly"));
  Serial.println(F(" status            show current state + force"));
  Serial.println(F(" force             show current force (g and N)"));
  Serial.println(F(" tare              zero the force sensor (motor stopped)"));
  Serial.println(F(" cal <grams>       calibrate with a known weight on the sensor"));
  Serial.println(F(" stream on|off     continuous display of speed + force"));
  Serial.println(F(" resetmax          reset maximum force"));
}

void processCommand(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) {
    return;
  }

  String lower = cmd;
  lower.toLowerCase();

  if (lower == "help" || lower == "?") {
    printHelp();
    return;
  }

  if (lower == "arm") {
    armEsc();
    return;
  }

  if (lower == "disarm") {
    disarm();
    Serial.println(F("Disarmed."));
    return;
  }

  if (lower == "stop") {
    setTargetUs(MIN_US);
    Serial.print(F("Stop requested. Target: "));
    Serial.print(targetUs);
    Serial.println(F(" us"));
    return;
  }

  if (lower == "status") {
    printStatus();
    return;
  }

  // ---- Commandes de mesure de force ----

  if (lower == "force") {
    printForce();
    return;
  }

  if (lower == "tare") {
    tareScale();
    return;
  }

  if (lower == "resetmax") {
    maxG = filteredG;
    Serial.println(F("Force max remise a zero."));
    return;
  }

  if (lower.startsWith("cal")) {
    String arg = cmd.substring(3);
    arg.trim();
    calibrateScale(arg.toFloat());
    return;
  }

  if (lower.startsWith("stream")) {
    if (lower.endsWith("on")) {
      streaming = true;
      Serial.println(F("Stream ON."));
    } else if (lower.endsWith("off")) {
      streaming = false;
      Serial.println(F("Stream OFF."));
    } else {
      Serial.println(F("Usage: stream on | stream off"));
    }
    return;
  }

  // ---- Commandes ESC ----

  // Speed command: speed 0 to speed 100
  bool isSpeedCommand = lower.startsWith("speed") || lower.startsWith("s ");

  if (isSpeedCommand) {
    int start = lower.startsWith("speed") ? 5 : 1;

    String arg = cmd.substring(start);
    arg.trim();

    if (arg.length() == 0) {
      Serial.println(F("Usage: speed 0 to 100"));
      return;
    }

    float percent = arg.toFloat();
    setTargetPercent(percent);

    Serial.print(F("Speed target: "));
    Serial.print(percent, 1);
    Serial.print(F("% ("));
    Serial.print(targetUs);
    Serial.println(F(" us)"));

    if (!armed) {
      Serial.println(F("Not armed. Type arm after ESC is connected/secured."));
    }

    return;
  }

  // Direct pulse width command: us 1000 to us 2000
  bool isUsCommand = lower.startsWith("us ") || lower == "us";

  if (isUsCommand) {
    String arg = cmd.substring(2);
    arg.trim();

    if (arg.length() == 0) {
      Serial.println(F("Usage: us 1000 to 2000"));
      return;
    }

    int us = arg.toInt();
    setTargetUs(us);

    Serial.print(F("PWM target: "));
    Serial.print(targetUs);
    Serial.println(F(" us"));

    if (!armed) {
      Serial.println(F("Not armed. Type arm after ESC is connected/secured."));
    }

    return;
  }

  Serial.println(F("Unknown command. Type help."));
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      if (inputBuffer.length() < 80) {
        inputBuffer += c;
      }
    }
  }
}

void updateRamp() {
  if (!armed) {
    return;
  }

  unsigned long now = millis();

  if (now - lastRampMs < RAMP_INTERVAL_MS) {
    return;
  }

  lastRampMs = now;

  if (currentUs == targetUs) {
    return;
  }

  if (currentUs < targetUs) {
    currentUs += RAMP_STEP_US;

    if (currentUs > targetUs) {
      currentUs = targetUs;
    }
  } else {
    currentUs -= RAMP_STEP_US;

    if (currentUs < targetUs) {
      currentUs = targetUs;
    }
  }

  writeCurrentUs();
}

void setup() {
  Serial.begin(115200);

  esc.attach(ESC_PIN, MIN_US, MAX_US);

  disarm();

  Serial.println(F("ESC serial control ready."));
  Serial.println(F("WARNING: remove propellers for testing."));

  // ---- Initialisation du capteur de force ----
  scale.begin(HX_DOUT_PIN, HX_SCK_PIN);

  if (scale.wait_ready_timeout(1000)) {
    scaleOk = true;
    loadCalibration();
    scale.set_scale(calFactor);

    Serial.println(F("Tare automatique, ne touchez pas au banc..."));
    scale.tare(10);
    Serial.println(F("Capteur de force pret."));
  } else {
    scaleOk = false;
    Serial.println(F("HX711 non detecte: verifiez le cablage (DT, SCK, VCC, GND)."));
  }

  printHelp();
}

void loop() {
  readSerial();
  updateRamp();
  updateForce();
  streamForce();
}
