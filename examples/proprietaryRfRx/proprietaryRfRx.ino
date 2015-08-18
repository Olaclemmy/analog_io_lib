#include <analog_io.h>
#include <SPI.h>

analog_io radio(P3_5, P3_6, P2_5);
const uint8_t rxaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };

void setup() {
  Serial.begin(9600);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  
  radio.begin();  // Defaults 1Mbps, channel 0, max TX power

  radio.setRXaddress((void*)rxaddr);
  
  radio.enableRX();  // Start listening
}

void loop() {
  char inbuf[33];
 
  while (!radio.available(true))
    ;
  if (radio.read(inbuf)) {
    Serial.print("Received packet: ");
    Serial.println(inbuf);
  }
}