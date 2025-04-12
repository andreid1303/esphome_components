#pragma once

#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <map>
#include <utility>
#include <vector>

#include <SPI.h>

namespace esphome {
namespace smartweather {

#if defined(USE_ESP8266) || defined(USE_ESP32)
  #define ISR_PREFIX ICACHE_RAM_ATTR
#else
  #define uint8_t ISR_PREFIX
#endif

#if defined(USE_ESP32)
  #define RF69_SPI_CS           5 // SS is the SPI slave select pin, for instance D10 on ATmega328
  #define RF69_IRQ_PIN          2
#else 
  #if defined(USE_ESP8266)
    #define RF69_SPI_CS         15 // SS is the SPI slave select pin, for instance D10 on ATmega328
    #define RF69_IRQ_PIN        4
  #endif
#endif


static const uint8_t RF69_MAX_DATA_LEN = 61; // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
static const uint8_t CSMA_LIMIT = -90; // upper RX signal sensitivity threshold in dBm for carrier sense access
static const uint8_t RF69_MODE_SLEEP = 0; // XTAL OFF
static const uint8_t RF69_MODE_STANDBY = 1; // XTAL ON
static const uint8_t RF69_MODE_SYNTH = 2; // PLL ON
static const uint8_t RF69_MODE_RX = 3; // RX MODE
static const uint8_t RF69_MODE_TX = 4; // TX MODE

// available frequency bands
static const uint8_t RF69_315MHZ = 31; // non trivial values to avoid misconfiguration
static const uint8_t RF69_433MHZ = 43;
static const uint8_t RF69_868MHZ = 86;
static const uint8_t RF69_915MHZ = 91;

static const uint8_t RF69_MAX_DATA_LEN = 61; // to take advantage of the built in AES/CRC we want to limit the frame size to the internal FIFO size (66 bytes - 3 bytes overhead - 2 bytes crc)
static const uint8_t COURSE_TEMP_COEF = -90; // puts the temperature reading in the ballpark, user can fine tune the returned value
static const uint8_t RF69_BROADCAST_ADDR = 0;
static const uint8_t RF69_CSMA_LIMIT_MS = 1000;
static const uint8_t RF69_TX_LIMIT_MS = 1000;
static const uint8_t RF69_FSTEP = 61.03515625; // == FXOSC / 2^19 = 32MHz / 2^19 (p13 in datasheet)

static const uint8_t PAYLOADLEN = 11;

static const uint8_t CONFIG[][2] =
  {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_4800 }, //default:4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_4800 },
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_60000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_60000},
    // /* 0x07 */ { REG_FRFMSB, 0xE4}, // 870 MHz 
    // /* 0x08 */ { REG_FRFMID, 0xC0},
    // /* 0x08 */ { REG_FRFLSB, 0x00},
    /* 0x07 */ { REG_FRFMSB, 0xD9}, // 869.85 MHz 
    /* 0x08 */ { REG_FRFMID, 0x76},
    /* 0x08 */ { REG_FRFLSB, 0x66},
    /* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    // /* 0x12 */ { REG_PARAMP, RF_PARAMP_40 | RF_OCP_TRIM_120 },
    /* 0x13 */ { REG_OCP, RF_OCP_OFF }, // over current protection (default is 95mA)
    // /* 0x18 */ { REG_LNA,  RF_LNA_ZIN_50 | RF_LNA_GAINSELECT_AUTO },
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw, RxBw > 2 * Fdev)
    // /* 0x1A */ { REG_AFCBW, RF_AFCBW_DCCFREQAFC_100 | RF_AFCBW_MANTAFC_20 | RF_AFCBW_EXPAFC_3 },
    // /* 0x1E */ { REG_AFCFEI,  RF_AFCFEI_AFCAUTOCLEAR_OFF | RF_AFCFEI_AFCAUTO_OFF },
    // /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    // /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    // /* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE }, // default 3 preamble bytes 0xAAAAAA
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    // /* 0x2D */ { REG_PREAMBLELSB, 3 }, // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },
    /* 0x30 */ { REG_SYNCVALUE2, 0xD4 },
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF | RF_PACKET1_ADRSFILTERING_OFF},
    /* 0x38 */ { REG_PAYLOADLENGTH, PAYLOADLEN },
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };

class RFM69 {
  public:
    static uint8_t DATA[PAYLOADLEN]; // RX/TX payload buffer, including end of string NULL char
    static bool valid;
    static int16_t rssi; // most accurate RSSI during reception (closest to the reception). RSSI of last packet.
    static uint8_t _mode; // should be protected?

    RFM69(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN);

    bool initialize();
    virtual bool receiveDone();
    void setFrequency(uint32_t freqHz);
    uint32_t getFrequency();
    void setDataRate(uint16_t dataRate);
    int16_t readRSSI(bool forceTrigger=false); // *current* signal strength indicator; e.g. < -90dBm says the frequency channel is free + ready to transmit
    void sleep();
    uint8_t readTemperature(uint8_t calFactor=0); // get CMOS temperature (8bit)

  protected:
    static void isr0();
    void interruptHandler();
    uint8_t readReg(uint8_t addr);
    void writeReg(uint8_t addr, uint8_t val);
    void setMode(uint8_t mode);
    void select();
    void unselect();
    void clearFifo();
    void receiveBegin();
    uint8_t calcCRC(uint8_t *buffer, uint8_t data_size);
    
    static volatile bool _haveData;
    uint8_t _slaveSelectPin;
    uint8_t _interruptPin;
    uint8_t _interruptNum;
};

}  // namespace smartweather
}  // namespace esphome