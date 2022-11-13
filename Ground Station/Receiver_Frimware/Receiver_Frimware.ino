#include <SPI.h>
#include <LoRa.h>


void setup() {
  Serial.begin(9600);
  while (!Serial);
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
    if(readByte == 0x66) {
      // ACK received from ground station computer
      // Respond with ACK
      Serial.write(0x66);
    }
    else {
      LoRa.beginPacket();
      LoRa.write(readByte);
      LoRa.endPacket(false);
    }
  }
}
