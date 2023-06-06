#include <util/twi.h>

#define MPU_addr 0x68  // Adresa akcelerometra

int16_t accelerometer_x, accelerometer_y, accelerometer_z;
unsigned long lastMovementTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 10000;  // Vreme u milisekundama (30 sekundi)

void twi_init() {
  // Inicijalizacija TWI (I2C) komunikacije
  TWSR = 0;  // Postavljanje preskaler faktora na 1
  TWBR = ((F_CPU / 400000L) - 16) / 2;  // Postavljanje brzine na 400kHz
}

void twi_start(uint8_t address) {
  // Slanje početnog stanja (START condition) na TWI (I2C) magistralu
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  // Slanje adrese uređaja za komunikaciju
  TWDR = address;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

void twi_write(uint8_t data) {
  // Slanje bajta podataka (data) preko TWI (I2C) magistrale
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
}

void twi_stop() {
  // Slanje stop stanja (STOP condition) na TWI (I2C) magistralu
  TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

uint8_t twi_read_ack() {
  // Čitanje bajta podataka sa ACK signalom (ACK = 1)
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

uint8_t twi_read_nack() {
  // Čitanje bajta podataka sa NACK signalom (ACK = 0)
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  return TWDR;
}

void setup() {
  TWBR = ((F_CPU / 400000L) - 16) / 2;  // Postavljanje brzine I2C komunikacije na 400kHz
  TWCR = 0;  // Onemogućavanje I2C interapta

  twi_init();  // Inicijalizacija I2C komunikacije
  twi_start(MPU_addr<<1);  // Slanje adrese uređaja
  twi_write(0x6B);   // Pisanje na registar za napajanje (Power Management)
  twi_write(0);      // Omogućavanje rada akcelerometra
  twi_stop();

  Serial.begin(9600); // Inicijalizacija serijske komunikacije
}

void loop() {
  checkMovement();  // Provera pokreta

  twi_start(MPU_addr<<1);  // Slanje adrese uređaja
  twi_write(0x3B);   // Početni registar za čitanje podataka akcelerometra
  twi_stop();
  twi_start((MPU_addr<<1) | 1);  // Slanje adrese uređaja u režimu čitanja
  accelerometer_x = (twi_read_ack() << 8) | twi_read_ack();
  accelerometer_y = (twi_read_ack() << 8) | twi_read_ack();
  accelerometer_z = (twi_read_ack() << 8) | twi_read_nack();
  twi_stop();

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
      // Dodajte ovde kod za isključivanje pegle
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
