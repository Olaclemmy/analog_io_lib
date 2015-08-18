#include <analog_io.h>
#include <SPI.h>

uint8_t x = 0;
analog_io radio(P3_5, P3_6, P2_5);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
}

void loop()
{
  // put your main code here, to run repeatedly:
  radio.print("Beacon #");
  radio.print(x);
  radio.flushBle();
  x++;
  delay(500);
}