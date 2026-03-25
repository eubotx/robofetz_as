#include <HardwareSerial.h>

HardwareSerial HC12(2); 

const int RX_PIN  = 22; 
const int TX_PIN  = 21; 
const int SET_PIN = 15; 

float linearX = 0.0;
float angularZ = 0.0;
float weaponSpeed = 0.0;

void setup() {
  Serial.begin(115200);
  pinMode(SET_PIN, OUTPUT);
  digitalWrite(SET_PIN, HIGH); 

  HC12.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("--- ESP32 Robot Receiver: PARSING ACTIVE ---");
}

void loop() {
  // --- 1. DATEN VOM FUNKMODUL EMPFANGEN & PARSEN ---
  if (HC12.available() > 0) {
    // Wir lesen die Funk-Daten bis zum Zeilenumbruch
    String input = HC12.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      // Zeige die Rohdaten zur Kontrolle
      Serial.print("Funk-Rohdaten: "); Serial.println(input);

      // --- EXTRAKTION JETZT HIER FÜR FUNK-DATEN ---
      if (input.startsWith("CMD:")) {
        String data = input.substring(4);
        int commaIndex = data.indexOf(',');
        
        if (commaIndex != -1) {
          linearX = data.substring(0, commaIndex).toFloat();
          angularZ = data.substring(commaIndex + 1).toFloat();

          // JETZT triggert diese Ausgabe!
          Serial.print(">>> DRIVE -> Lin: "); Serial.print(linearX);
          Serial.print(" | Ang: "); Serial.println(angularZ);
        }
      }
      else if (input.startsWith("WPN:")) {
        weaponSpeed = input.substring(4).toFloat();
        Serial.print(">>> WEAPON -> Speed: "); Serial.println(weaponSpeed);
      }
    }
  }

  // --- 2. OPTIONAL: DATEN VOM PC AN FUNK SENDEN (Für manuelle Tests) ---
  if (Serial.available()) {
    String pcInput = Serial.readStringUntil('\n');
    HC12.println(pcInput);
  }
}