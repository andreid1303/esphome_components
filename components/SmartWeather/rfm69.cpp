#include "rfm69.h"
#include "rfm69_registers.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace smartweather {

uint8_t RFM69::DATA[PAYLOADLEN];
bool RFM69::valid = false;
uint8_t RFM69::_mode;        // current transceiver state
int16_t RFM69::rssi;          // most accurate rssi during reception (closest to the reception)
volatile bool RFM69::_haveData;

RFM69::RFM69(uint8_t slaveSelectPin, uint8_t interruptPin) {
  _slaveSelectPin = slaveSelectPin;
  _interruptPin = interruptPin;
  _mode = RF69_MODE_STANDBY;
}

bool RFM69::initialize() {
  _interruptNum = digitalPinToInterrupt(_interruptPin);
  if (_interruptNum == (uint8_t)NOT_AN_INTERRUPT) {
    Serial.print(_interruptNum);
    Serial.println(" not interuupt!");
    return false;
  }

  clearFifo();

  digitalWrite(_slaveSelectPin, HIGH);
  pinMode(_slaveSelectPin, OUTPUT);

  SPI.begin();

  uint32_t start = millis();
  uint8_t timeout = 50;
  do writeReg(REG_SYNCVALUE1, 0xAA); while (readReg(REG_SYNCVALUE1) != 0xaa && millis()-start < timeout);
  start = millis();
  do writeReg(REG_SYNCVALUE1, 0x55); while (readReg(REG_SYNCVALUE1) != 0x55 && millis()-start < timeout);

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++)
    writeReg(CONFIG[i][0], CONFIG[i][1]);

  setMode(RF69_MODE_STANDBY);
  start = millis();
  while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis()-start < timeout); // wait for ModeReady
  if (millis()-start >= timeout)
    return false;
  attachInterrupt(digitalPinToInterrupt(_interruptPin), RFM69::isr0, RISING);

  return true;
}

void RFM69::setFrequency(uint32_t freqHz) {
  uint8_t oldMode = _mode;
  if (oldMode == RF69_MODE_TX) {
    setMode(RF69_MODE_RX);
  }
  freqHz /= RF69_FSTEP; // divide down by FSTEP to get FRF
  writeReg(REG_FRFMSB, freqHz >> 16);
  writeReg(REG_FRFMID, freqHz >> 8);
  writeReg(REG_FRFLSB, freqHz);
  if (oldMode == RF69_MODE_RX) {
    setMode(RF69_MODE_SYNTH);
  }
  setMode(oldMode);
}

uint32_t RFM69::getFrequency()
{
  return RF69_FSTEP * (((uint32_t) readReg(REG_FRFMSB) << 16) + ((uint16_t) readReg(REG_FRFMID) << 8) + readReg(REG_FRFLSB));
}

void RFM69::setDataRate(uint16_t dataRate) {
  word r = ((32000000UL + (dataRate / 2)) / dataRate);
  writeReg(REG_BITRATEMSB, r >> 8);
  writeReg(REG_BITRATELSB, r & 0xFF);
}

void RFM69::setMode(uint8_t newMode)
{
  if (newMode == _mode)
    return;

  switch (newMode) {
    case RF69_MODE_TX:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      break;
    case RF69_MODE_RX:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      break;
    case RF69_MODE_SYNTH:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default:
      return;
  }
  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady

  _mode = newMode;
}

//put transceiver in sleep mode to save battery - to wake or resume receiving just call receiveDone()
void RFM69::sleep() {
  setMode(RF69_MODE_SLEEP);
}

// internal function - interrupt gets called when a packet is received
void RFM69::interruptHandler() {
  valid = false;
  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY))
  {
    setMode(RF69_MODE_STANDBY);
    select();
    // read byte from FIFO
    SPI.transfer(REG_FIFO & 0x7F);
    for (uint8_t i = 0; i < PAYLOADLEN; i++) DATA[i] = SPI.transfer(0);
    valid = (DATA[PAYLOADLEN - 1] == calcCRC(DATA,PAYLOADLEN - 1));

    unselect();
    // clearFifo();
    setMode(RF69_MODE_RX);

  }
  rssi = readRSSI();
}

// internal function
ISR_PREFIX void RFM69::isr0() { _haveData = true; }

// internal function
void RFM69::receiveBegin() {
  rssi = 0;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  }
  
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode
  setMode(RF69_MODE_RX);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
bool RFM69::receiveDone() {
  if (_haveData) {
  	_haveData = false;
  	interruptHandler();
    // if (valid) return true;
    return true;
  }
  if (_mode == RF69_MODE_RX) // already in RX no payload yet
  {
    return false;
  }
  receiveBegin();
  return false;
}

// get the received signal strength indicator (RSSI)
int16_t RFM69::readRSSI(bool forceTrigger) {
  int16_t rssi = 0;
  if (forceTrigger)
  {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  rssi = -readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

uint8_t RFM69::readReg(uint8_t addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  uint8_t regval = SPI.transfer(0);
  unselect();
  return regval;
}

void RFM69::writeReg(uint8_t addr, uint8_t value)
{
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}

// select the RFM69 transceiver (save SPI settings, set CS low)
void RFM69::select() {
  // set RFM69 SPI settings explicitly
  SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE0));
  digitalWrite(RF69_SPI_CS, LOW);
}

// unselect the RFM69 transceiver (set CS high, restore SPI settings)
void RFM69::unselect() {
  digitalWrite(RF69_SPI_CS, HIGH);
  SPI.endTransaction();
}

void RFM69::clearFifo() {
    digitalWrite(REG_IRQFLAGS2, 16);
}

uint8_t RFM69::readTemperature(uint8_t calFactor) // returns centigrade
{
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING));
  return ~readReg(REG_TEMP2) + COURSE_TEMP_COEF + calFactor; // 'complement' corrects the slope, rising temp = rising val
} // COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction


//CRC checksum > Poly 0x31, init 0xff, revin&revout, xorout 0x00. Like Maxim 1-wire but with a 0xff init value
uint8_t RFM69::calcCRC(uint8_t *buffer, uint8_t data_size)
{
  uint8_t crc = 0xff;                                   // init = 0xff
  uint8_t data;

  for (uint8_t n = 0; n < data_size; n++)
  {
    data = buffer[n];
    for (uint8_t i = 0; i < 8; i++)
    {
      uint8_t tmp = (crc ^ data) & 0x01;
      crc >>= 1;
      if (tmp) crc ^= 0x8C;
      data >>= 1;
    }
  }
  return crc;
}


}  // namespace smartweather
}  // namespace esphome