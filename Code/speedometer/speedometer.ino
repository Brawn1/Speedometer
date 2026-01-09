#include <Servo.h>

// Pin-Definitionen
const int SENSOR1_PIN = 2;  // Erster IR-Sensor (Digital Pin)
const int SENSOR2_PIN = 3;  // Zweiter IR-Sensor (Digital Pin)
const int SERVO_PIN = 9;    // Servo Motor Pin

// Konstanten
const float DISTANCE_CM = 10.0;  // Abstand zwischen Sensoren in cm
const int SERVO_MIN_ANGLE = 30;   // Minimaler Servo-Winkel
const int SERVO_MAX_ANGLE = 160; // Maximaler Servo-Winkel
const float MAX_SPEED_KMH = 100.0; // Maximale erwartete Geschwindigkeit in km/h
const int DEBOUNCE_DELAY = 50; // Entprellzeit in Millisekunden

// Variablen
Servo speedServo;
unsigned long sensor1Time = 0;
unsigned long sensor2Time = 0;
bool sensor1Triggered = false;
bool measurementActive = false;
unsigned long lastSensor1Trigger = 0;
unsigned long lastSensor2Trigger = 0;

void setup() {
  // Serial Monitor initialisieren
  Serial.begin(9600);
  Serial.println("Geschwindigkeitsmessung gestartet");
  Serial.println("Sensorabstand: 10cm");
  Serial.println("Entprellung aktiviert");
  Serial.println("------------------------");
  
  // Pins konfigurieren
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  
  // Servo initialisieren
  speedServo.attach(SERVO_PIN);
  speedServo.write(30);  // Servo auf 30° setzen
  
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Sensor 1 prüfen (Start der Messung) mit Entprellung
  if (digitalRead(SENSOR1_PIN) == LOW && !measurementActive) {
    if (currentTime - lastSensor1Trigger > DEBOUNCE_DELAY) {
      sensor1Time = currentTime;
      sensor1Triggered = true;
      measurementActive = true;
      lastSensor1Trigger = currentTime;
      Serial.println("Sensor 1 ausgelöst - Messung gestartet");
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
        float speedKmh = (speedCmPerMs) * 360.0;
        
        // Umrechnung in m/s
        float speedMs = (speedCmPerMs * 100.0);
        
        // Ergebnisse ausgeben
        Serial.println("------------------------");
        Serial.print("Zeit: ");
        Serial.print((timeDiff / 1000.0), 4);
        Serial.println(" s");
        Serial.print("Geschwindigkeit: ");
        Serial.print(speedKmh, 2);
        Serial.print(" km/h (");
        Serial.print(speedMs, 4);
        Serial.println(" m/s)");
        Serial.println("------------------------");
        
        // Servo-Position berechnen und setzen
        int servoAngle = calculateServoAngle(speedKmh);
        speedServo.write(servoAngle);
        Serial.print("Servo-Position: ");
        Serial.print(servoAngle);
        Serial.println("°");
        Serial.println();
        
        // Servo 2 Sekunden anzeigen, dann deaktivieren
        delay(2000);
        speedServo.detach();  // Servo ausschalten = weniger Rauschen
        delay(500);
        speedServo.attach(SERVO_PIN);  // Servo wieder aktivieren
        speedServo.write(30);  // Zurück zur Startposition
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
    Serial.println("Timeout - Messung abgebrochen");
    measurementActive = false;
    sensor1Triggered = false;
    speedServo.write(30);
  }
  
  delay(5);  // Kleine Verzögerung für Stabilität
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
  
  return angle;
}