// Physical Wiring:
// MOSI --> D11
// MISO --> D12
// To test our transmission we will feed MISO into MOSI so that we can test both at the same time. So physically connect both pins togehter.
// SCK  --> D13
// GND
//////// Note: Setting Pin 10 as OUTPUT so MCU stays in Master Mode

#include <Arduino.h>
#include <SPI.h>
#include "SPI.h"

#define CS_PIN 9   // choose any free digital pin

void setup() {
  Serial.begin(9600);
  delay(1000);

  //Forcing SS pin as output to stay in master mode
  pinMode(10, OUTPUT);

  spi_init();
  spi_config_cs(CS_PIN);

  Serial.println("  SPI Loopback Test Starting   ");
}

void loop() {
  uint8_t tx = 0xA5;
  uint8_t rx = 0;

  spi_begin(CS_PIN, SPI_MODE0); 
  rx = spi_txrx(tx);
  spi_end(CS_PIN);

  Serial.print("TX: 0x");
  Serial.print(tx, HEX);
  Serial.print("   RX: 0x");
  Serial.println(rx, HEX);

  delay(1000);
}
