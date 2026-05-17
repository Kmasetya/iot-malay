// RELAY TEST - Upload this to ESP32 to diagnose relay behavior
// Wires on: GPIO 18 = pump, GPIO 19 = valve

#define RELAY_PUMP  18
#define RELAY_VALVE 19

void setup() {
  Serial.begin(115200);
  
  // Set pins as output and OFF immediately
  pinMode(RELAY_PUMP, OUTPUT);
  pinMode(RELAY_VALVE, OUTPUT);
  digitalWrite(RELAY_PUMP, HIGH);  // try HIGH = OFF (active-LOW)
  digitalWrite(RELAY_VALVE, HIGH);
  
  delay(2000);
  Serial.println("=== RELAY TEST ===");
  Serial.println("Both relays should be OFF right now.");
  Serial.println("Are they off? Check the LEDs on relay module.");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  1 = pump ON (LOW)");
  Serial.println("  2 = pump OFF (HIGH)");
  Serial.println("  3 = valve ON (LOW)");
  Serial.println("  4 = valve OFF (HIGH)");
  Serial.println("  5 = pump ON (HIGH) - test active-HIGH");
  Serial.println("  6 = pump OFF (LOW) - test active-HIGH");
  Serial.println("  7 = valve ON (HIGH) - test active-HIGH");
  Serial.println("  8 = valve OFF (LOW) - test active-HIGH");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch(c) {
      case '1': digitalWrite(RELAY_PUMP, LOW);  Serial.println("Pump -> LOW");  break;
      case '2': digitalWrite(RELAY_PUMP, HIGH); Serial.println("Pump -> HIGH"); break;
      case '3': digitalWrite(RELAY_VALVE, LOW);  Serial.println("Valve -> LOW");  break;
      case '4': digitalWrite(RELAY_VALVE, HIGH); Serial.println("Valve -> HIGH"); break;
      case '5': digitalWrite(RELAY_PUMP, HIGH);  Serial.println("Pump -> HIGH"); break;
      case '6': digitalWrite(RELAY_PUMP, LOW);   Serial.println("Pump -> LOW");  break;
      case '7': digitalWrite(RELAY_VALVE, HIGH); Serial.println("Valve -> HIGH"); break;
      case '8': digitalWrite(RELAY_VALVE, LOW);  Serial.println("Valve -> LOW");  break;
    }
  }
}
