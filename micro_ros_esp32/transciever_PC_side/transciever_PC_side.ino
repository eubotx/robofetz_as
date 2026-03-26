#include <HardwareSerial.h>

HardwareSerial HC12(2); 

// Definition nach deinem Bild (Lila Nummern verwenden!)
const int RX_PIN = 22; // Physikalisch Pin 39 (Verbunden mit HC-12 TX)
const int TX_PIN = 21; // Physikalisch Pin 42 (Verbunden mit HC-12 RX)
const int SET_PIN = 15;

void setup() {
  Serial.begin(115200);
  
  pinMode(SET_PIN, OUTPUT);
  digitalWrite(SET_PIN, HIGH); 

  HC12.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  // WICHTIG: Puffer leeren, um Garbage vom Booten zu entfernen
  delay(500);
  while(Serial.available()) Serial.read();
  
  Serial.println("\n--- ESP32 Text-Bridge BEREIT ---");
}

void loop() {
  // 1. VOM PC (ROS) -> FUNK
  if (Serial.available() > 0) {
    // Wir lesen die Zeile manuell, um sicherzugehen
    String input = Serial.readStringUntil('\n');
    input.trim(); // Entfernt unsichtbare Zeichen wie \r oder Leerzeichen

    if (input.length() > 0) {
      // Bestätigung an den Monitor
      Serial.print(">>> Sende an Funk: ");
      Serial.println(input);
      
      // An das Funkmodul senden
      HC12.println(input);
    }
  }

  // 2. VOM FUNK -> PC (ROS)  
  if (HC12.available() > 0) {
    String fromRadio = HC12.readStringUntil('\n');
    fromRadio.trim();

    if (fromRadio.length() > 0) {
      // Wenn das Funkmodul was empfängt, schicken wir es an ROS/PC
      Serial.print("<<< Empfangen ueber Funk: ");
      Serial.println(fromRadio);
    }
  }
}