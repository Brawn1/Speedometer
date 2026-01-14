#include <Servo.h>

// Pin-Definitionen
const int SENSOR1_PIN = 2;  // Erster IR-Sensor (Digital Pin)
const int SENSOR2_PIN = 3;  // Zweiter IR-Sensor (Digital Pin)
const int SERVO_PIN = 9;    // Servo Motor Pin
const int BATTERY_PIN = A0; // Batterie-Spannungsmessung (Analog Pin)
const int LED_GREEN_PIN = 4; // Grüne LED
const int LED_RED_PIN = 5;   // Rote LED

// Konstanten
const float DISTANCE_CM = 10.0;  // Abstand zwischen Sensoren in cm
const int SERVO_MIN_ANGLE = 30;   // Minimaler Servo-Winkel
const int SERVO_MAX_ANGLE = 170; // Maximaler Servo-Winkel
const float MAX_SPEED_KMH = 150.0; // Maximale erwartete Geschwindigkeit in km/h
const int DEBOUNCE_DELAY = 50; // Entprellzeit in Millisekunden
const int SERVO_MOVE_DELAY = 500; // Zeit in ms zum Erreichen der Servo-Position
// multiplikator, für den Servo Ausschlag.
const float SERVO_ANGLE_MULTI = 2.6;


// Batterie-Konstanten
const float BATTERY_LOW_VOLTAGE = 3.3;  // Warnung bei 3.3V (Li-Ion fast leer)
const float BATTERY_FULL_VOLTAGE = 4.2; // Voll geladen
const unsigned long BATTERY_CHECK_INTERVAL = 30000; // Batterie alle 30 Sekunden prüfen
const float ADC_REFERENCE_VOLTAGE = 4.600; // Arduino Nano ADC Referenzspannung (gemessen: 4.457V DC)
const float VOLTAGE_DIVIDER_RATIO = 2.0; // Spannungsteiler-Verhältnis (R1=10kΩ, R2=10kΩ → Faktor 2)
const float ADC_MAX_VALUE = 1023.0;      // 10-bit ADC maximaler Wert

// Variablen
Servo speedServo;
unsigned long sensor1Time = 0;
unsigned long sensor2Time = 0;
bool sensor1Triggered = false;
bool measurementActive = false;
unsigned long lastSensor1Trigger = 0;
unsigned long lastSensor2Trigger = 0;
unsigned long lastBatteryCheck = 0;

void setup() {
  // Serial Monitor initialisieren
  Serial.begin(9600);
  Serial.println(F("Geschwindigkeitsmessung gestartet"));
  Serial.print(F("Sensorabstand: "));
  Serial.print(DISTANCE_CM, 0);
  Serial.println(F(" cm"));
  Serial.println(F("Entprellung aktiviert"));
  Serial.println(F("Battery-Monitoring aktiviert"));
  Serial.println(F("------------------------"));
  
  // Pins konfigurieren
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  
  // Servo initialisieren und zur Startposition bewegen
  resetServoToStartPosition();
  
  // Erste Batterie-Prüfung
  checkBattery();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Batterie regelmäßig prüfen
  if (currentTime - lastBatteryCheck > BATTERY_CHECK_INTERVAL) {
    checkBattery();
    lastBatteryCheck = currentTime;
  }
  
  // Sensor 1 prüfen (Start der Messung) mit Entprellung
  if (digitalRead(SENSOR1_PIN) == LOW && !measurementActive) {
    if (currentTime - lastSensor1Trigger > DEBOUNCE_DELAY) {
      sensor1Time = currentTime;
      sensor1Triggered = true;
      measurementActive = true;
      lastSensor1Trigger = currentTime;
      Serial.println(F("Sensor 1 ausgelöst - Messung gestartet"));
    }
  }
  
  // Sensor 2 prüfen (Ende der Messung) mit Entprellung
  if (digitalRead(SENSOR2_PIN) == LOW && measurementActive && sensor1Triggered) {
    if (currentTime - lastSensor2Trigger > DEBOUNCE_DELAY) {
      sensor2Time = currentTime;
      lastSensor2Trigger = currentTime;
      
      // Zeit zwischen Sensoren berechnen
      unsigned long timeDiff = sensor2Time - sensor1Time;
      
      // Geschwindigkeit berechnen
      if (timeDiff > 0) {
        // Geschwindigkeit in cm/ms
        float speedCmPerMs = DISTANCE_CM / (float)timeDiff;
        
        // Umrechnung in km/h
        // cm/ms → m/s → km/h: cm/ms * 10 * 3.6 = cm/ms * 36
        float speedKmh = speedCmPerMs * 36.0;
        
        // Umrechnung in m/s
        // cm/ms → m/s: cm/ms * (1m/100cm) * (1000ms/1s) = cm/ms * 10
        float speedMs = speedCmPerMs * 10.0;
        
        // Ergebnisse ausgeben
        Serial.println(F("------------------------"));
        Serial.print(F("Zeit: "));
        Serial.print((timeDiff / 1000.0), 4);
        Serial.println(F(" s"));
        Serial.print(F("Geschwindigkeit: "));
        Serial.print(speedKmh, 2);
        Serial.print(F(" km/h ("));
        Serial.print(speedMs, 4);
        Serial.println(F(" m/s)"));
        Serial.println(F("------------------------"));
        
        // Servo-Position berechnen und setzen
        int servoAngle = calculateServoAngle(speedKmh);
        speedServo.attach(SERVO_PIN);  // Servo aktivieren vor dem Schreiben
        speedServo.write(servoAngle);
        Serial.print(F("Servo-Position: "));
        Serial.print(servoAngle);
        Serial.println(F("°"));
        Serial.println();
        
        // Servo 2 Sekunden anzeigen lassen
        delay(2000);
        
        // Zurück zur Startposition fahren und Servo deaktivieren
        resetServoToStartPosition();
      }
      
      // Reset für nächste Messung
      sensor1Triggered = false;
      measurementActive = false;
      
      // Kurze Verzögerung um Mehrfachauslösungen zu vermeiden
      delay(500);
    }
  }
  
  // Timeout: Wenn Sensor 2 nicht innerhalb von 4 Sekunden ausgelöst wird
  if (measurementActive && (currentTime - sensor1Time > 4000)) {
    Serial.println(F("Timeout - Messung abgebrochen"));
    measurementActive = false;
    sensor1Triggered = false;
    resetServoToStartPosition();
  }
  
  delay(5);  // Kleine Verzögerung für Stabilität
}

// Funktion zum Zurücksetzen des Servos zur Startposition und Deaktivieren
void resetServoToStartPosition() {
  speedServo.attach(SERVO_PIN);
  speedServo.write(SERVO_MIN_ANGLE);
  delay(SERVO_MOVE_DELAY);  // Zeit zum Erreichen der Position
  speedServo.detach();
}

// Funktion zur Berechnung des Servo-Winkels basierend auf Geschwindigkeit
int calculateServoAngle(float speedKmh) {
  // Begrenze Geschwindigkeit auf Maximum
  if (speedKmh > MAX_SPEED_KMH) {
    speedKmh = MAX_SPEED_KMH;
  }
  
  // Lineare Abbildung von 0-MAX_SPEED_KMH auf SERVO_MIN_ANGLE-SERVO_MAX_ANGLE
  int angle = SERVO_MIN_ANGLE + (int)((speedKmh / MAX_SPEED_KMH) * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE));
  
  // Begrenze Winkel auf gültigen Bereich
  if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
  if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
  
  return round(float(angle) * SERVO_ANGLE_MULTI);
}

// Funktion zur Batterie-Spannungsmessung und LED-Steuerung
void checkBattery() {
  // Spannung messen
  int sensorValue = analogRead(BATTERY_PIN);
  float voltage = (sensorValue / ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE * VOLTAGE_DIVIDER_RATIO;
  
  // Debug-Ausgabe
  Serial.print(F("🔋 Batterie: "));
  Serial.print(voltage, 2);
  Serial.print(F("V"));
  
  // LED-Status setzen
  if (voltage < BATTERY_LOW_VOLTAGE) {
    // Batterie schwach - Rote LED
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, HIGH);
    Serial.println(F(" - ⚠️ BATTERIE SCHWACH!"));
  } else {
    // Batterie OK - Grüne LED
    digitalWrite(LED_GREEN_PIN, HIGH);
    digitalWrite(LED_RED_PIN, LOW);
    Serial.println(F(" - ✅ OK"));
  }
}
