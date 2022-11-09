#include <SPI.h>
#include <LoRa.h>


void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.write(0x44);             // Byte to say ground station is starting
  while (!LoRa.begin(915E6)) {
    Serial.write(0xFA);
  }
}


void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // read packet
    while (LoRa.available()) {
      Serial.write(LoRa.read());
    }
    Serial.write("\r\n");
  }

  if (Serial.available() > 0) {
    byte readByte = Serial.read();
    LoRa.beginPacket();
    LoRa.write(readByte);
    LoRa.endPacket(false);
    }
}
