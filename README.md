analog.io lib
======

This library is inspired by the Enrf24 library but is expanded to encompass specific hardware features of the analog.io MSP430 Sensor node.  This hardware uses the HopeRF RF75 Transceiver which is compatable with Nordic nRF24.

What is improved in this library is that users are also able to:
1. Transmit 14byte Bluetooth Low Energy Beacons using the bleTransmit()
2. Publish data to MQTT using the analog.io hub*
3. Subscribe to MQTT streams using the analog.io hub*

*Requires analog.io hub