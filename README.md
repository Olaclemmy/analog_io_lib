analog.io lib
======

This library is inspired by the Enrf24 library but is expanded to encompass specific hardware features of the analog.io MSP430 Sensor node.  This hardware uses the HopeRF RF75 Transceiver which is compatable with Nordic nRF24.

What is improved in this library is that users are also able to:
  * Transmit 14byte Bluetooth Low Energy Beacons using the bleTransmit()
  * TODO: Publish data to MQTT using the analog.io hub*
  * TODO: Subscribe to MQTT streams using the analog.io hub*

*Requires analog.io hub

Installation Instructions
======
  * Download zip repository from this github page
  * Unzip the package and rename the folder to "analog_io"
  * move/copy this folder to the Energia Libraries folder, this is typically found in:
    * <Energia Location>/Contents/Resources/Java/hardware/msp430/libraries
