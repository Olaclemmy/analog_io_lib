/* analog_io_lib for Energia and the MSP430 Sensor Node Hardware
 * Forked from nRF24L01+ I/O for Energia
 *
 * Copyright (c) 2013 Eric Brundick <spirilis [at] linux dot com>
 * Copyright (c) 2015 Luke Beno <lgbeno [at] analog dot io>
 *
 *  Permission is hereby granted, free of charge, to any person 
 *  obtaining a copy of this software and associated documentation 
 *  files (the "Software"), to deal in the Software without 
 *  restriction, including without limitation the rights to use, copy, 
 *  modify, merge, publish, distribute, sublicense, and/or sell copies 
 *  of the Software, and to permit persons to whom the Software is 
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be 
 *  included in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 *  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
 *  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 *  DEALINGS IN THE SOFTWARE.
 */

#include <Arduino.h>
#include <stdint.h>
#include "analog_io.h"
#define MY_MAC_0  0xEF
#define MY_MAC_1  0xFF
#define MY_MAC_2  0xC0
#define MY_MAC_3  0xAA
#define MY_MAC_4  0x18
#define MY_MAC_5  0x00
/* Constructor */
analog_io::analog_io(uint8_t cePin, uint8_t csnPin, uint8_t irqPin)
{
  _cePin = cePin;
  _csnPin = csnPin;
  _irqPin = irqPin;
  mode = UNDEFINED_MODE;
  rf_status = 0;
  rf_addr_width = 5;
  txbuf_len = 0;
  readpending = 0;
  backup_regs[0] = RF24_CONFIG;
  backup_regs[1] = RF24_EN_AA;
  backup_regs[2] = RF24_EN_RXADDR;
  backup_regs[3] = RF24_SETUP_AW;
  backup_regs[4] = RF24_SETUP_RETR;
  backup_regs[5] = RF24_RF_SETUP;
  backup_regs[6] = RF24_STATUS;
  backup_regs[7] = RF24_DYNPD;
  backup_regs[8] = RF24_FEATURE;
  backup_regs[9] = RF24_RX_PW_P0;
  backup_regs[10] = RF24_EN_RXADDR; //TODO convert to static const
  backup_regs[11] = RF24_RF_CH;
  chRf[0] = 2; //TODO convert to static const
  chRf[1] = 26;
  chRf[2] = 80;
  chLe[0] = 37;
  chLe[1] = 38;
  chLe[2] = 39;
}

/* Initialization */
void analog_io::begin(uint32_t datarate, uint8_t channel)
{
  //txbuf_len = 0;
  pinMode(_cePin, OUTPUT);
  digitalWrite(_cePin, LOW);
  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  pinMode(_irqPin, INPUT);
  digitalWrite(_irqPin, LOW);  // No pullups; the transceiver provides this!

  SPI.transfer(0);  // Strawman transfer, fixes USCI issue on G2553

  // Is the transceiver present/alive?
  if (!_isAlive())
    return;  // Nothing more to do here...

  // Wait 100ms for module to initialize
  while ( millis() < 100 )
    ;

  //Write to FEATURE Register and see if changes stick 
  _writeReg(RF24_FEATURE, RF24_EN_DPL);
  
  if (_readReg(RF24_FEATURE) == 0x00) {
    //If changes do not stick, issue an activate command
    uint8_t _activate_data = 0x73;
    _issueCmdPayload(RF24_ACTIVATE, &_activate_data, 1);
  }

  // Init certain registers
  _writeReg(RF24_CONFIG, 0x00);  // Deep power-down, everything disabled
  _writeReg(RF24_EN_AA, 0x03);
  _writeReg(RF24_EN_RXADDR, 0x03);
  _writeReg(RF24_RF_SETUP, 0x00);
  _writeReg(RF24_STATUS, ENRF24_IRQ_MASK);  // Clear all IRQs
  _writeReg(RF24_DYNPD, 0x03);
  _writeReg(RF24_FEATURE, RF24_EN_DPL);  // Dynamic payloads enabled by default

  // Set all parameters
  if (channel > 125)
    channel = 125;
  deepsleep();
  _issueCmd(RF24_FLUSH_TX);
  _issueCmd(RF24_FLUSH_RX);
  readpending = 0;
  _irq_clear(ENRF24_IRQ_MASK);
  setChannel(channel);
  setSpeed(datarate);
  setTXpower();
  setAutoAckParams();
  setAddressLength(rf_addr_width);
  setCRC(true);  // Default = CRC on, 8-bit
  mode = PROPRIETARY_MODE;
}

/* Formal shut-down/clearing of library state */
void analog_io::end()
{
  txbuf_len = 0;
  rf_status = 0;
  rf_addr_width = 5;

  if (!_isAlive())
    return;
  deepsleep();
  _issueCmd(RF24_FLUSH_TX);
  _issueCmd(RF24_FLUSH_RX);
  readpending = 0;
  _irq_clear(ENRF24_IRQ_MASK);
  digitalWrite(_cePin, LOW);
  digitalWrite(_csnPin, HIGH);
}

/* Basic SPI I/O */
uint8_t analog_io::_readReg(uint8_t addr)
{
  uint8_t result;

  digitalWrite(_csnPin, LOW);
  rf_status = SPI.transfer(RF24_R_REGISTER | addr);
  result = SPI.transfer(RF24_NOP);
  digitalWrite(_csnPin, HIGH);
  return result;
}

void analog_io::_readRegMultiLSB(uint8_t addr, uint8_t *buf, size_t len)
{
  uint8_t i;
  digitalWrite(_csnPin, LOW);
  rf_status = SPI.transfer(RF24_R_REGISTER | addr);
  for (i=0; i<len; i++) {
    buf[len-i-1] = SPI.transfer(RF24_NOP);
  }
  digitalWrite(_csnPin, HIGH);
}

void analog_io::_writeReg(uint8_t addr, uint8_t val)
{
  digitalWrite(_csnPin, LOW);
  rf_status = SPI.transfer(RF24_W_REGISTER | addr);
  SPI.transfer(val);
  digitalWrite(_csnPin, HIGH);
}

void analog_io::_writeRegMultiLSB(uint8_t addr, uint8_t *buf, size_t len)
{
  size_t i;

  digitalWrite(_csnPin, LOW);
  rf_status = SPI.transfer(RF24_W_REGISTER | addr);
  for (i=0; i<len; i++) {
    SPI.transfer(buf[len-i-1]);
  }
  digitalWrite(_csnPin, HIGH);
}

void analog_io::_issueCmd(uint8_t cmd)
{
  digitalWrite(_csnPin, LOW);
  rf_status = SPI.transfer(cmd);
  digitalWrite(_csnPin, HIGH);
}

void analog_io::_issueCmdPayload(uint8_t cmd, uint8_t *buf, size_t len)
{
  size_t i;

  digitalWrite(_csnPin, LOW);
  rf_status = SPI.transfer(cmd);
  for (i=0; i<len; i++) {
    SPI.transfer(buf[i]);
  }
  digitalWrite(_csnPin, HIGH);
}

void analog_io::_readCmdPayload(uint8_t cmd, uint8_t *buf, size_t len, size_t maxlen)
{
  size_t i;

  digitalWrite(_csnPin, LOW);
  rf_status = SPI.transfer(cmd);
  for (i=0; i<len; i++) {
    if (i < maxlen) {
      buf[i] = SPI.transfer(RF24_NOP);
    } else {
      SPI.transfer(RF24_NOP);  // Beyond maxlen bytes, just discard the remaining data.
    }
  }
  digitalWrite(_csnPin, HIGH);
}

boolean analog_io::_isAlive()
{
  uint8_t aw;

  aw = _readReg(RF24_SETUP_AW);
  return ((aw & 0xFC) == 0x00 && (aw & 0x03) != 0x00);
}

uint8_t analog_io::_irq_getreason()
{
  lastirq = _readReg(RF24_STATUS) & ENRF24_IRQ_MASK;
  return lastirq;
}

// Get IRQ from last known rf_status update without querying module over SPI.
uint8_t analog_io::_irq_derivereason()
{
  lastirq = rf_status & ENRF24_IRQ_MASK;
  return lastirq;
}

void analog_io::_irq_clear(uint8_t irq)
{
  _writeReg(RF24_STATUS, irq & ENRF24_IRQ_MASK);
}

#define ENRF24_CFGMASK_CRC(a) (a & (RF24_EN_CRC | RF24_CRCO))

void analog_io::_readTXaddr(uint8_t *buf)
{
  _readRegMultiLSB(RF24_TX_ADDR, buf, rf_addr_width);
}


void analog_io::_writeRXaddrP0(uint8_t *buf)
{
  _writeRegMultiLSB(RF24_RX_ADDR_P0, buf, rf_addr_width);
}


/* nRF24 I/O maintenance--called as a "hook" inside other I/O functions to give
 * the library a chance to take care of its buffers et al
 */
void analog_io::_maintenanceHook()
{
  uint8_t i;

  _irq_getreason();

  if (lastirq & ENRF24_IRQ_TXFAILED) {
    lastTXfailed = true;
    _issueCmd(RF24_FLUSH_TX);
    _irq_clear(ENRF24_IRQ_TXFAILED);
  }

  if (lastirq & ENRF24_IRQ_TX) {
    lastTXfailed = false;
    _irq_clear(ENRF24_IRQ_TX);
  }

  if (lastirq & ENRF24_IRQ_RX) {
    if ( !(_readReg(RF24_FIFO_STATUS) & RF24_RX_FULL) ) {  /* Don't feel it's necessary
                                                            * to be notified of new
                                                            * incoming packets if the RX
                                                            * queue is full.
                                                            */
      _irq_clear(ENRF24_IRQ_RX);
    }

    /* Check if RX payload is 0-byte or >32byte (erroneous conditions)
     * Also check if data was received on pipe#0, which we are ignoring.
     * The reason for this is pipe#0 is needed for receiving AutoACK acknowledgements,
     * its address gets reset to the module's default and we do not care about data
     * coming in to that address...
     */
    _readCmdPayload(RF24_R_RX_PL_WID, &i, 1, 1);
    if (i == 0 || i > 32 || ((rf_status & 0x0E) >> 1) == 0) {
                             /* Zero-width RX payload is an error that happens a lot
                              * with non-AutoAck, and must be cleared with FLUSH_RX.
                              * Erroneous >32byte packets are a similar phenomenon.
                              */
      _issueCmd(RF24_FLUSH_RX);
      _irq_clear(ENRF24_IRQ_RX);
      readpending = 0;
    } else {
      readpending = 1;
    }
    // Actual scavenging of RX queues is performed by user-directed use of read().
  }
}



/* Public functions */
boolean analog_io::available(boolean checkIrq)
{
  if (checkIrq && digitalRead(_irqPin) == HIGH && readpending == 0)
    return false;
  _maintenanceHook();
  if ( !(_readReg(RF24_FIFO_STATUS) & RF24_RX_EMPTY) ) {
    return true;
  }
  if (readpending) {
    return true;
  }
  return false;
}

size_t analog_io::read(void *inbuf, uint8_t maxlen)
{
  uint8_t *buf = (uint8_t *)inbuf;
  uint8_t plwidth;

  _maintenanceHook();
  readpending = 0;
  if ((_readReg(RF24_FIFO_STATUS) & RF24_RX_EMPTY) || maxlen < 1) {
    return 0;
  }
  _readCmdPayload(RF24_R_RX_PL_WID, &plwidth, 1, 1);
  _readCmdPayload(RF24_R_RX_PAYLOAD, buf, plwidth, maxlen);
  buf[plwidth] = '\0';  // Zero-terminate in case this is a string.
  if (_irq_derivereason() & ENRF24_IRQ_RX) {
    _irq_clear(ENRF24_IRQ_RX);
  }

  return (size_t) plwidth;
}

// Perform TX of current ring-buffer contents
void analog_io::flush()
{
  if (mode == BLE_MODE) bleEnd();
  uint8_t reg, addrbuf[5];
  boolean enaa=false, origrx=false;

  if (!txbuf_len)
    return;  // Zero-length buffer?  Nothing to send!

  reg = _readReg(RF24_FIFO_STATUS);
  if (reg & BIT5) {  // RF24_TX_FULL #define is BIT0, which is not the correct bit for FIFO_STATUS.
    // Seen this before with a user whose CE pin was messed up.
    _issueCmd(RF24_FLUSH_TX);
    txbuf_len = 0;
    return;  // Should never happen, but nonetheless a precaution to take.
  }

  _maintenanceHook();

  if (reg & RF24_TX_REUSE) {
    // If somehow TX_REUSE is enabled, we need to flush the TX queue before loading our new payload.
    _issueCmd(RF24_FLUSH_TX);
  }

  if (_readReg(RF24_EN_AA) & 0x01 && (_readReg(RF24_RF_SETUP) & 0x28) != 0x20) {
    /* AutoACK enabled, must write TX addr to RX pipe#0
     * Note that 250Kbps doesn't support auto-ack, so we check RF24_RF_SETUP to verify that.
     */
    enaa = true;
    _readTXaddr(addrbuf);
    _writeRXaddrP0(addrbuf);
  }

  reg = _readReg(RF24_CONFIG);
  if ( !(reg & RF24_PWR_UP) ) {
    //digitalWrite(_cePin, HIGH);  // Workaround for SI24R1 knockoff chips
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP);
    delay(5);  // 5ms delay required for nRF24 oscillator start-up
    //digitalWrite(_cePin, LOW);
  }
  if (reg & RF24_PRIM_RX) {
    origrx=true;
    digitalWrite(_cePin, LOW);
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP);
  }

  _issueCmdPayload(RF24_W_TX_PAYLOAD, txbuf, txbuf_len);
  digitalWrite(_cePin, HIGH);
  delayMicroseconds(100);
  digitalWrite(_cePin, LOW);

  txbuf_len = 0;  // Reset TX ring buffer

  while (digitalRead(_irqPin) == HIGH)  // Wait until IRQ fires
    ;
  // IRQ fired
  _maintenanceHook();  // Handle/clear IRQ

  // Purge Pipe#0 address (set to module's power-up default)
  if (enaa) {
    addrbuf[0] = 0xE7; addrbuf[1] = 0xE7; addrbuf[2] = 0xE7; addrbuf[3] = 0xE7; addrbuf[4] = 0xE7;
    _writeRXaddrP0(addrbuf);
  }

  // If we were in RX mode before writing, return back to RX mode.
  if (origrx) {
    enableRX();
  }
}

void analog_io::purge()
{
  txbuf_len = 0;
}

size_t analog_io::write(uint8_t c)
{
  if (txbuf_len == 32) {  // If we're trying to stuff an already-full buffer...
    flush();  // Blocking OTA TX
  }

  txbuf[txbuf_len] = c;
  txbuf_len++;

  return 1;
}

uint8_t analog_io::radioState()
{
  uint8_t reg;

  if (!_isAlive())
    return ENRF24_STATE_NOTPRESENT;
  
  reg = _readReg(RF24_CONFIG);
  if ( !(reg & RF24_PWR_UP) )
    return ENRF24_STATE_DEEPSLEEP;

  // At this point it's either Standby-I, II or PRX.
  if (reg & RF24_PRIM_RX) {
    if (digitalRead(_cePin))
      return ENRF24_STATE_PRX;
    // PRIM_RX=1 but CE=0 is a form of idle state.
    return ENRF24_STATE_IDLE;
  }
  // Check if TX queue is empty, if so it's idle, if not it's PTX.
  if (_readReg(RF24_FIFO_STATUS) & RF24_TX_EMPTY)
    return ENRF24_STATE_IDLE;
  return ENRF24_STATE_PTX;
}

void analog_io::deepsleep()
{
  uint8_t reg;

  reg = _readReg(RF24_CONFIG);
  if (reg & (RF24_PWR_UP | RF24_PRIM_RX)) {
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg));
  }
  digitalWrite(_cePin, LOW);
}

void analog_io::enableRX()
{
  uint8_t reg;

  reg = _readReg(RF24_CONFIG);
  _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP | RF24_PRIM_RX);
  digitalWrite(_cePin, HIGH);

  if ( !(reg & RF24_PWR_UP) ) {  // Powering up from deep-sleep requires 5ms oscillator start delay
    delay(5);
  }
}

void analog_io::disableRX()
{
  uint8_t reg;

  digitalWrite(_cePin, LOW);

  reg = _readReg(RF24_CONFIG);
  if (reg & RF24_PWR_UP) {  /* Keep us in standby-I if we're coming from RX mode, otherwise stay
                             * in deep-sleep if we call this while already in PWR_UP=0 mode.
                             */
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg) | RF24_PWR_UP);
  } else {
    _writeReg(RF24_CONFIG, ENRF24_CFGMASK_IRQ | ENRF24_CFGMASK_CRC(reg));
  }
}

void analog_io::autoAck(boolean onoff)
{
  uint8_t reg;

  reg = _readReg(RF24_EN_AA);
  if (onoff) {
    if ( !(reg & 0x01) || !(reg & 0x02) ) {
      _writeReg(RF24_EN_AA, 0x03);
    }
  } else {
    if (reg & 0x03) {
      _writeReg(RF24_EN_AA, 0x00);
    }
  }
}

void analog_io::setChannel(uint8_t channel)
{
  if (channel > 125)
    channel = 125;
  _writeReg(RF24_RF_CH, channel);
}

void analog_io::setTXpower(int8_t dBm)
{
  uint8_t reg, pwr;

  reg = _readReg(RF24_RF_SETUP) & 0xF8;  // preserve RF speed settings
  pwr = 0x06;
  if (dBm >= 7)
    pwr = 0x07;
  if (dBm < 0)
    pwr = 0x04;
  if (dBm < -6)
    pwr = 0x02;
  if (dBm < -12)
    pwr = 0x00;
  _writeReg(RF24_RF_SETUP, reg | pwr);
}

void analog_io::setSpeed(uint32_t rfspeed)
{
  uint8_t reg, spd;

  reg = _readReg(RF24_RF_SETUP) & 0xD7;  // preserve RF power settings
  spd = 0x01;
  if (rfspeed < 2000000)
    spd = 0x00;
  if (rfspeed < 1000000)
    spd = 0x04;
  _writeReg(RF24_RF_SETUP, reg | (spd << 3));
}

void analog_io::setCRC(boolean onoff, boolean crc16bit)
{
  uint8_t reg, crcbits=0;

  reg = _readReg(RF24_CONFIG) & 0xF3;  // preserve IRQ mask, PWR_UP/PRIM_RX settings
  if (onoff)
    crcbits |= RF24_EN_CRC;
  if (crc16bit)
    crcbits |= RF24_CRCO;
  _writeReg(RF24_CONFIG, reg | crcbits);
}

void analog_io::setAutoAckParams(uint8_t autoretry_count, uint16_t autoretry_timeout)
{
  uint8_t setup_retr=0;

  setup_retr = autoretry_count & 0x0F;
  autoretry_timeout -= 250;
  setup_retr |= ((autoretry_timeout / 250) & 0x0F) << 4;
  _writeReg(RF24_SETUP_RETR, setup_retr);
}

void analog_io::setAddressLength(size_t len)
{
  if (len < 3)
    len = 3;
  if (len > 5)
    len = 5;

  _writeReg(RF24_SETUP_AW, len-2);
  rf_addr_width = len;
}

void analog_io::setRXaddress(const void *rxaddr)
{
  _writeRegMultiLSB(RF24_RX_ADDR_P1, (uint8_t*)rxaddr, rf_addr_width);
}

void analog_io::setTXaddress(const void *rxaddr)
{
  _writeRegMultiLSB(RF24_TX_ADDR, (uint8_t*)rxaddr, rf_addr_width);
}

boolean analog_io::rfSignalDetected()
{
  uint8_t rpd;

  rpd = _readReg(RF24_RPD);
  return (boolean)rpd;
}

uint32_t analog_io::getSpeed()
{
  uint8_t reg = _readReg(RF24_RF_SETUP) & 0x28;

  switch (reg) {
    case 0x00:
      return 1000000UL;
    case 0x08:
      return 2000000UL;
    case 0x20:
      return 250000UL;
  }
  return 0UL;
}

int8_t analog_io::getTXpower()
{
  uint8_t reg = _readReg(RF24_RF_SETUP) & 0x07;

  if (reg & 0x01)
    return 7;  // SI24R1-only +7dBm mode
  switch (reg) {
    case 0x02:
      return -12;
    case 0x04:
      return -6;
    case 0x06:
      return 0;
  }
  return -18;
}

boolean analog_io::getAutoAck()
{
  uint8_t reg = _readReg(RF24_EN_AA);

  if (reg)
    return true;
  return false;
}

void analog_io::getRXaddress(void *buf)
{
  _readRegMultiLSB(RF24_RX_ADDR_P1, (uint8_t*)buf, rf_addr_width);
}

void analog_io::getTXaddress(void *buf)
{
  _readRegMultiLSB(RF24_TX_ADDR, (uint8_t*)buf, rf_addr_width);
}

unsigned int analog_io::getCRC()
{
  uint8_t reg = _readReg(RF24_CONFIG) & 0x0C;

  switch (reg) {
    case 0x08:
      return 8;
    case 0x0C:
      return 16;
  }

  return 0;
}

void analog_io::_btLeCrc(const uint8_t* data, uint8_t len, uint8_t* dst){

  uint8_t v, t, d;

  while(len--){
  
    d = *data++;
    for(v = 0; v < 8; v++, d >>= 1){
    
      t = dst[0] >> 7;
      
      dst[0] <<= 1;
      if(dst[1] & 0x80) dst[0] |= 1;
      dst[1] <<= 1;
      if(dst[2] & 0x80) dst[1] |= 1;
      dst[2] <<= 1;
      
    
      if(t != (d & 1)){
      
        dst[2] ^= 0x5B;
        dst[1] ^= 0x06;
      }
    } 
  }
}

uint8_t  analog_io::_swapbits(uint8_t a){

  uint8_t v = 0;
  
  if(a & 0x80) v |= 0x01;
  if(a & 0x40) v |= 0x02;
  if(a & 0x20) v |= 0x04;
  if(a & 0x10) v |= 0x08;
  if(a & 0x08) v |= 0x10;
  if(a & 0x04) v |= 0x20;
  if(a & 0x02) v |= 0x40;
  if(a & 0x01) v |= 0x80;

  return v;
}

void analog_io::_btLeWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff){

  uint8_t  m;
  
  while(len--){
  
    for(m = 1; m; m <<= 1){
    
      if(whitenCoeff & 0x80){
        
        whitenCoeff ^= 0x11;
        (*data) ^= m;
      }
      whitenCoeff <<= 1;
    }
    data++;
  }
}

void analog_io::_btLePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan){
  //length is of packet, including crc. pre-populate crc in packet with initial crc value!

  uint8_t i, dataLen = len - 3;
  
  _btLeCrc(packet, dataLen, packet + dataLen);
  for(i = 0; i < 3; i++, dataLen++) packet[dataLen] = _swapbits(packet[dataLen]);
  _btLeWhiten(packet, len, (_swapbits(chan) | 2)); //_btLeWhitenStart
  for(i = 0; i < len; i++) packet[i] = _swapbits(packet[i]);
  
}

// A function to read all current registers and save them in this object so that they can be restored at a later time
void analog_io::_backupRegisters(void) {
  for (i=0; i<12; i++) {
    backup[i] = _readReg(backup_regs[i]);
  }
  _readRegMultiLSB(RF24_TX_ADDR,txaddr_bak,5);
}

// A function to restore all registers from a backup 
void analog_io::_restoreRegisters(void) {
  for (i=0; i<12; i++) {
    _writeReg(backup_regs[i],backup[i]);
  }
  _writeRegMultiLSB(RF24_TX_ADDR,txaddr_bak,5);
}

// A function to setup the radio for a BLE transmission
void analog_io::_configBle(void) {
  pinMode(_cePin, OUTPUT);
  digitalWrite(_cePin, LOW);
  pinMode(_csnPin, OUTPUT);
  digitalWrite(_csnPin, HIGH);
  pinMode(_irqPin, INPUT);
  digitalWrite(_irqPin, LOW);

  _writeReg(RF24_CONFIG, 0x12); //on, no crc, int on RX/TX done
  _writeReg(RF24_EN_AA, 0x00);  //no auto-acknowledge
  _writeReg(RF24_EN_RXADDR, 0x00);  //no RX
  _writeReg(RF24_SETUP_AW, 0x02); //5-byte address
  _writeReg(RF24_SETUP_RETR, 0x00); //no auto-retransmit
  _writeReg(RF24_RF_SETUP, 0x06); //1MBps at 0dBm
  _writeReg(RF24_STATUS, 0x3E); //clear various flags
  _writeReg(RF24_DYNPD, 0x00);  //no dynamic payloads
  _writeReg(RF24_FEATURE, 0x00);  //no features
  _writeReg(RF24_RX_PW_P0, 32); //always RX 32 bytes
  _writeReg(RF24_EN_RXADDR, 0x01);  //RX on pipe 0

  uint8_t buf[4];

  buf[3] = _swapbits(0x8E);
  buf[2] = _swapbits(0x89);
  buf[1] = _swapbits(0xBE);
  buf[0] = _swapbits(0xD6);
  _writeRegMultiLSB(RF24_TX_ADDR, buf, 4);

  mode = BLE_MODE;
  //_writeRegMultiLSB(RF24_RX_ADDR_P0, txbuf, 4);
}
void analog_io::bleTransmit(char* name, uint8_t* payload, uint8_t payload_len) {
  _bleTx(name, 0xFF, payload, payload_len);
}

void analog_io::_bleTx(char* name, uint8_t name_len, uint8_t* payload, uint8_t payload_len) {
  if (mode != BLE_MODE) bleStart();
    if (name_len == 0xFF) {
      name_len = 0;
      while (name[name_len]!=0x00) {
        txbuf[13+name_len] = name[name_len];
        // Limit name length to 14 bytes due to BLE limitation
        if (name_len < 13)
          name_len++;
        else
          break;
      }
    }
    else {
      if (name_len>13) name_len = 13;
      for (i=0;i<name_len;i++) {
        txbuf[13+i] = name[i];
      }
    } 

    txbuf[0] = 0x42;  //PDU type, given address is random

    txbuf[2] = 0xEF;
    txbuf[3] = 0xFF;
    txbuf[4] = 0xC0;
    txbuf[5] = 0xAA;
    txbuf[6] = 0x18;
    txbuf[7] = 0x00;
    
    txbuf[8] = 2;   //flags (LE-only, limited discovery mode)
    txbuf[9] = 0x01;
    txbuf[10] = 0x05;
    
    txbuf[12] = 0x08;

    if (name_len+payload_len>13) payload_len = 14-name_len;

    txbuf[1] = 13+name_len+payload_len; // Total Packet length
    txbuf[11] = name_len+1; // Length of Name only

    txbuf_len = 13+name_len;

    txbuf[txbuf_len++] = payload_len+1; // Payload length
    txbuf[txbuf_len++] = 0xff;

    if (payload_len == 0) {
      payload_len = 1;
      txbuf[txbuf_len++] = 0xFF;
    }
    else {
      for (i=0;i<payload_len;i++) {
        txbuf[txbuf_len++] = payload[i];
      } 
    }
    
    txbuf[txbuf_len++] = 0x55;  //CRC start value: 0x555555
    txbuf[txbuf_len++] = 0x55;
    txbuf[txbuf_len++] = 0x55;
    
    
    if(++ch == sizeof(chRf)) ch = 0;
    

    _writeReg(RF24_RF_CH, chRf[ch]);
    _writeReg(RF24_STATUS, 0x6E);

    _btLePacketEncode(txbuf, txbuf_len, chLe[ch]);
    
    _issueCmd(RF24_FLUSH_RX);
    _issueCmd(RF24_FLUSH_TX);

    _issueCmdPayload(RF24_W_TX_PAYLOAD, txbuf, txbuf_len);
    txbuf_len = 0;

    _writeReg(RF24_STATUS, 0x6E);
    _writeReg(RF24_CONFIG, 0x12); //tx on

    digitalWrite(_cePin, HIGH); // sbi(PORTB, PIN_CE);   //do tx
    delay(10); //delay_ms(10);
    digitalWrite(_cePin, LOW); // cbi(PORTB, PIN_CE); (in preparation of switching to RX quickly)
}

void analog_io::bleStart() {
  //_backupRegisters();

  _readRegMultiLSB(RF24_TX_ADDR,txaddr_bak,5);
  _configBle();
}

void analog_io::bleEnd(void) {
  //_restoreRegisters();
  begin();
  _writeRegMultiLSB(RF24_TX_ADDR,txaddr_bak,5);
}


void analog_io::bleSingleTransmit(char* name, uint8_t* payload, uint8_t payload_len) {
  bleStart();
  bleTransmit(name,payload,payload_len);
  bleEnd();
}

void analog_io::flushBle()
{

  if (txbuf_len < 13) txbuf[txbuf_len] = 0x00;
  else txbuf[13] = 0x00;

  bleTransmit((char*) txbuf);
  txbuf_len = 0;  // Reset TX ring buffer

  while (digitalRead(_irqPin) == HIGH)  // Wait until IRQ fires
    ;
  // IRQ fired
  _maintenanceHook();  // Handle/clear IRQ

}