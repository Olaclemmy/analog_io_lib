#include <analog_io.h>
#include <SPI.h>

uint8_t x = 0;
analog_io radio(P3_5, P3_6, P2_5);
const uint8_t txaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };

void setup()
{
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  
  radio.begin();  // Defaults 1Mbps, channel 0, max TX power

  radio.setTXaddress((void*)txaddr);
}

void loop()
{
  // put your main code here, to run repeatedly:
  radio.print("TX #");
  radio.print(x);
  radio.flush();
  x++;
  delay(500);
}