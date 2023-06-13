#include <Wire.h>

const int MPU_addr = 0x68;  // Adresa akcelerometra

int16_t accelerometer_x, accelerometer_y, accelerometer_z;
unsigned long lastMovementTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 10000;  // Vreme u milisekundama (30 sekundi)

void setup() {
  Wire.begin();        // Inicijalizacija I2C komunikacije
  Wire.beginTransmission(MPU_addr); // Slanje adrese uređaja
  Wire.write(0x6B);   // Pisanje na registar za napajanje (Power Management)
  Wire.write(0);      // Omogućavanje rada akcelerometra
  Wire.endTransmission(true);
  Serial.begin(9600); // Inicijalizacija serijske komunikacije
}

void loop() {
  checkMovement();  // Provjera pokreta

  Wire.beginTransmission(MPU_addr); // Slanje adrese uređaja
  Wire.write(0x3B);   // Početni registar za čitanje podataka akcelerometra
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true); // Zahtev za 6 bajtova podataka

  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();

  Serial.print("X: ");
  Serial.print(accelerometer_x);
  Serial.print(" Y: ");
  Serial.print(accelerometer_y);
  Serial.print(" Z: ");
  Serial.println(accelerometer_z);

  delay(250);  // Pauza između očitanja podataka
}

void checkMovement() {
  if (isMovementDetected()) {
    lastMovementTime = millis();  // Ažuriranje vremena poslednjeg pokreta
  } else {
    if (millis() - lastMovementTime >= WATCHDOG_TIMEOUT) {
      // Prošlo je više od 10 sekundi bez pokreta
      Serial.println("Isključite peglu!");
      //ovde cemo dodati kod za iskljucivanje pegle
    }
  }
}

bool isMovementDetected() {
  int16_t threshold =25000;  // Prag vrednosti za detekciju pokreta (može se prilagoditi)

  int16_t acceleration = abs(accelerometer_x) + abs(accelerometer_y) + abs(accelerometer_z);
  
  if (acceleration > threshold) {
    return true;
  } else {
    return false;
  }
}
