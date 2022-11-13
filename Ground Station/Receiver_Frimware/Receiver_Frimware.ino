#include <SPI.h>
#include <LoRa.h>
byte readByte;

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
    if(0 /*Respond to a "are you here" request*/) {
      // ACK received from ground station computer
      // Respond with ACK
      // Serial.write(0x66);
    }
    else {
      size_t packet_length = 0;
      uint8_t packet[1024];
      while(Serial.available()) {
        // Read data into buffer
        readByte = Serial.read();
        packet[packet_length] = readByte;
        packet_length++;        
      }
      // Transmit buffer
      LoRa.beginPacket();
      LoRa.write(packet, packet_length+1);
      LoRa.endPacket(false);
    }
  }
}
