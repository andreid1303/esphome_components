#pragma once

namespace esphome {
namespace smartweather {

static const uint8_t REG_FIFO = 0x00;
static const uint8_t REG_OPMODE = 0x01;
static const uint8_t REG_DATAMODUL = 0x02;
static const uint8_t REG_BITRATEMSB = 0x03;
static const uint8_t REG_BITRATELSB = 0x04;
static const uint8_t REG_FDEVMSB = 0x05;
static const uint8_t REG_FDEVLSB = 0x06;
static const uint8_t REG_FRFMSB = 0x07;
static const uint8_t REG_FRFMID = 0x08;
static const uint8_t REG_FRFLSB = 0x09;
static const uint8_t REG_OSC1 = 0x0A;
static const uint8_t REG_AFCCTRL = 0x0B;
static const uint8_t REG_LOWBAT = 0x0C;
static const uint8_t REG_LISTEN1 = 0x0D;
static const uint8_t REG_LISTEN2 = 0x0E;
static const uint8_t REG_LISTEN3 = 0x0F;
static const uint8_t REG_VERSION = 0x10;
static const uint8_t REG_PALEVEL = 0x11;
static const uint8_t REG_PARAMP = 0x12;
static const uint8_t REG_OCP = 0x13;
static const uint8_t REG_AGCREF = 0x14; // not present on RFM69/SX1231
static const uint8_t REG_AGCTHRESH1 = 0x15;  // not present on RFM69/SX1231
static const uint8_t REG_AGCTHRESH2 = 0x16;  // not present on RFM69/SX1231
static const uint8_t REG_AGCTHRESH3 = 0x17;  // not present on RFM69/SX1231
static const uint8_t REG_LNA = 0x18;
static const uint8_t REG_RXBW = 0x19;
static const uint8_t REG_AFCBW = 0x1A;
static const uint8_t REG_OOKPEAK = 0x1B;
static const uint8_t REG_OOKAVG = 0x1C;
static const uint8_t REG_OOKFIX = 0x1D;
static const uint8_t REG_AFCFEI = 0x1E;
static const uint8_t REG_AFCMSB = 0x1F;
static const uint8_t REG_AFCLSB = 0x20;
static const uint8_t REG_FEIMSB = 0x21;
static const uint8_t REG_FEILSB = 0x22;
static const uint8_t REG_RSSICONFIG = 0x23;
static const uint8_t REG_RSSIVALUE = 0x24;
static const uint8_t REG_DIOMAPPING1 = 0x25;
static const uint8_t REG_DIOMAPPING2 = 0x26;
static const uint8_t REG_IRQFLAGS1 = 0x27;
static const uint8_t REG_IRQFLAGS2 = 0x28;
static const uint8_t REG_RSSITHRESH = 0x29;
static const uint8_t REG_RXTIMEOUT1 = 0x2A;
static const uint8_t REG_RXTIMEOUT2 = 0x2B;
static const uint8_t REG_PREAMBLEMSB = 0x2C;
static const uint8_t REG_PREAMBLELSB = 0x2D;
static const uint8_t REG_SYNCCONFIG = 0x2E;
static const uint8_t REG_SYNCVALUE1 = 0x2F;
static const uint8_t REG_SYNCVALUE2 = 0x30;
static const uint8_t REG_SYNCVALUE3 = 0x31;
static const uint8_t REG_SYNCVALUE4 = 0x32;
static const uint8_t REG_SYNCVALUE5 = 0x33;
static const uint8_t REG_SYNCVALUE6 = 0x34;
static const uint8_t REG_SYNCVALUE7 = 0x35;
static const uint8_t REG_SYNCVALUE8 = 0x36;
static const uint8_t REG_PACKETCONFIG1 0x37;
static const uint8_t REG_PAYLOADLENGTH 0x38;
static const uint8_t REG_NODEADRS = 0x39;
static const uint8_t REG_BROADCASTADRS 0x3A;
static const uint8_t REG_AUTOMODES = 0x3B;
static const uint8_t REG_FIFOTHRESH = 0x3C;
static const uint8_t REG_PACKETCONFIG2 0x3D;
static const uint8_t REG_AESKEY1 = 0x3E;
static const uint8_t REG_AESKEY2 = 0x3F;
static const uint8_t REG_AESKEY3 = 0x40;
static const uint8_t REG_AESKEY4 = 0x41;
static const uint8_t REG_AESKEY5 = 0x42;
static const uint8_t REG_AESKEY6 = 0x43;
static const uint8_t REG_AESKEY7 = 0x44;
static const uint8_t REG_AESKEY8 = 0x45;
static const uint8_t REG_AESKEY9 = 0x46;
static const uint8_t REG_AESKEY10 = 0x47;
static const uint8_t REG_AESKEY11 = 0x48;
static const uint8_t REG_AESKEY12 = 0x49;
static const uint8_t REG_AESKEY13 = 0x4A;
static const uint8_t REG_AESKEY14 = 0x4B;
static const uint8_t REG_AESKEY15 = 0x4C;
static const uint8_t REG_AESKEY16 = 0x4D;
static const uint8_t REG_TEMP1 = 0x4E;
static const uint8_t REG_TEMP2 = 0x4F;
static const uint8_t REG_TESTLNA = 0x58;
static const uint8_t REG_TESTPA1 = 0x5A;  // only present on RFM69HW/SX1231H
static const uint8_t REG_TESTPA2 = 0x5C;  // only present on RFM69HW/SX1231H
static const uint8_t REG_TESTDAGC = 0x6F;

//******************************************************
// RF69/SX1231 bit control definition
//******************************************************

// RegOpMode
static const uint8_t RF_OPMODE_SEQUENCER_OFF = 0x80;
static const uint8_t RF_OPMODE_SEQUENCER_ON = 0x00;  // Default

static const uint8_t RF_OPMODE_LISTEN_ON = 0x40;
static const uint8_t RF_OPMODE_LISTEN_OFF = 0x00;  // Default

static const uint8_t RF_OPMODE_LISTENABORT = 0x20;

static const uint8_t RF_OPMODE_MASK = 0xE3; // Rinie assumed

static const uint8_t RF_OPMODE_SLEEP = 0x00;
static const uint8_t RF_OPMODE_STANDBY = 0x04;  // Default
static const uint8_t RF_OPMODE_SYNTHESIZER = 0x08;
static const uint8_t RF_OPMODE_TRANSMITTER = 0x0C;
static const uint8_t RF_OPMODE_RECEIVER = 0x10;


// RegDataModul
static const uint8_t RF_DATAMODUL_DATAMODE_PACKET = 0x00;  // Default
static const uint8_t RF_DATAMODUL_DATAMODE_CONTINUOUS = 0x40;
static const uint8_t RF_DATAMODUL_DATAMODE_CONTINUOUSNOBSYNC = 0x60;

static const uint8_t RF_DATAMODUL_MODULATIONTYPE_FSK = 0x00;  // Default
static const uint8_t RF_DATAMODUL_MODULATIONTYPE_OOK = 0x08;

static const uint8_t RF_DATAMODUL_MODULATIONSHAPING_00 = 0x00;  // Default
static const uint8_t RF_DATAMODUL_MODULATIONSHAPING_01 = 0x01;
static const uint8_t RF_DATAMODUL_MODULATIONSHAPING_10 = 0x02;
static const uint8_t RF_DATAMODUL_MODULATIONSHAPING_11 = 0x03;


// RegBitRate (bits/sec) example bit rates
static const uint8_t RF_BITRATEMSB_1200 = 0x68;
static const uint8_t RF_BITRATELSB_1200 = 0x2B;
static const uint8_t RF_BITRATEMSB_2400 = 0x34;
static const uint8_t RF_BITRATELSB_2400 = 0x15;
static const uint8_t RF_BITRATEMSB_4800 = 0x1A;  // Default
static const uint8_t RF_BITRATELSB_4800 = 0x0B;  // Default
static const uint8_t RF_BITRATEMSB_9600 = 0x0D;
static const uint8_t RF_BITRATELSB_9600 = 0x05;
static const uint8_t RF_BITRATEMSB_19200 = 0x06;
static const uint8_t RF_BITRATELSB_19200 = 0x83;
static const uint8_t RF_BITRATEMSB_38400 = 0x03;
static const uint8_t RF_BITRATELSB_38400 = 0x41;

static const uint8_t RF_BITRATEMSB_38323 = 0x03;
static const uint8_t RF_BITRATELSB_38323 = 0x43;

static const uint8_t RF_BITRATEMSB_34482 = 0x03;
static const uint8_t RF_BITRATELSB_34482 = 0xA0;

static const uint8_t RF_BITRATEMSB_76800 = 0x01;
static const uint8_t RF_BITRATELSB_76800 = 0xA1;
static const uint8_t RF_BITRATEMSB_153600 = 0x00;
static const uint8_t RF_BITRATELSB_153600 = 0xD0;
static const uint8_t RF_BITRATEMSB_57600 = 0x02;
static const uint8_t RF_BITRATELSB_57600 = 0x2C;
static const uint8_t RF_BITRATEMSB_115200 = 0x01;
static const uint8_t RF_BITRATELSB_115200 = 0x16;
static const uint8_t RF_BITRATEMSB_12500 = 0x0A;
static const uint8_t RF_BITRATELSB_12500 = 0x00;
static const uint8_t RF_BITRATEMSB_25000 = 0x05;
static const uint8_t RF_BITRATELSB_25000 = 0x00;
static const uint8_t RF_BITRATEMSB_50000 = 0x02;
static const uint8_t RF_BITRATELSB_50000 = 0x80;
static const uint8_t RF_BITRATEMSB_100000 = 0x01;
static const uint8_t RF_BITRATELSB_100000 = 0x40;
static const uint8_t RF_BITRATEMSB_150000 = 0x00;
static const uint8_t RF_BITRATELSB_150000 = 0xD5;
static const uint8_t RF_BITRATEMSB_200000 = 0x00;
static const uint8_t RF_BITRATELSB_200000 = 0xA0;
static const uint8_t RF_BITRATEMSB_250000 = 0x00;
static const uint8_t RF_BITRATELSB_250000 = 0x80;
static const uint8_t RF_BITRATEMSB_300000 = 0x00;
static const uint8_t RF_BITRATELSB_300000 = 0x6B;
static const uint8_t RF_BITRATEMSB_32768 = 0x03;
static const uint8_t RF_BITRATELSB_32768 = 0xD1;
// custom bit rates
static const uint8_t RF_BITRATEMSB_55555 = 0x02;
static const uint8_t RF_BITRATELSB_55555 = 0x40;
static const uint8_t RF_BITRATEMSB_200KBPS = 0x00;
static const uint8_t RF_BITRATELSB_200KBPS = 0xA0;
static const uint8_t RF_BITRATEMSB_9579 = 0x0C;
static const uint8_t RF_BITRATELSB_9579 = 0x3C;


// RegFdev - frequency deviation (Hz)
static const uint8_t RF_FDEVMSB_2000 = 0x00;
static const uint8_t RF_FDEVLSB_2000 = 0x21;
static const uint8_t RF_FDEVMSB_5000 = 0x00; // Default
static const uint8_t RF_FDEVLSB_5000 = 0x52;  // Default
static const uint8_t RF_FDEVMSB_7500 = 0x00;
static const uint8_t RF_FDEVLSB_7500 = 0x7B;
static const uint8_t RF_FDEVMSB_10000 = 0x00;
static const uint8_t RF_FDEVLSB_10000 = 0xA4;
static const uint8_t RF_FDEVMSB_15000 = 0x00;
static const uint8_t RF_FDEVLSB_15000 = 0xF6;
static const uint8_t RF_FDEVMSB_20000 = 0x01;
static const uint8_t RF_FDEVLSB_20000 = 0x48;
static const uint8_t RF_FDEVMSB_25000 = 0x01;
static const uint8_t RF_FDEVLSB_25000 = 0x9A;
static const uint8_t RF_FDEVMSB_30000 = 0x01;
static const uint8_t RF_FDEVLSB_30000 = 0xEC;
static const uint8_t RF_FDEVMSB_35000 = 0x02;
static const uint8_t RF_FDEVLSB_35000 = 0x3D;
static const uint8_t RF_FDEVMSB_40000 = 0x02;
static const uint8_t RF_FDEVLSB_40000 = 0x8F;
static const uint8_t RF_FDEVMSB_45000 = 0x02;
static const uint8_t RF_FDEVLSB_45000 = 0xE1;
static const uint8_t RF_FDEVMSB_50000 = 0x03;
static const uint8_t RF_FDEVLSB_50000 = 0x33;
static const uint8_t RF_FDEVMSB_55000 = 0x03;
static const uint8_t RF_FDEVLSB_55000 = 0x85;
static const uint8_t RF_FDEVMSB_60000 = 0x03;
static const uint8_t RF_FDEVLSB_60000 = 0xD7;
static const uint8_t RF_FDEVMSB_65000 = 0x04;
static const uint8_t RF_FDEVLSB_65000 = 0x29;
static const uint8_t RF_FDEVMSB_70000 = 0x04;
static const uint8_t RF_FDEVLSB_70000 = 0x7B;
static const uint8_t RF_FDEVMSB_75000 = 0x04;
static const uint8_t RF_FDEVLSB_75000 = 0xCD;
static const uint8_t RF_FDEVMSB_80000 = 0x05;
static const uint8_t RF_FDEVLSB_80000 = 0x1F;
static const uint8_t RF_FDEVMSB_85000 = 0x05;
static const uint8_t RF_FDEVLSB_85000 = 0x71;
static const uint8_t RF_FDEVMSB_90000 = 0x05;
static const uint8_t RF_FDEVLSB_90000 = 0xC3;
static const uint8_t RF_FDEVMSB_95000 = 0x06;
static const uint8_t RF_FDEVLSB_95000 = 0x14;
static const uint8_t RF_FDEVMSB_100000 = 0x06;
static const uint8_t RF_FDEVLSB_100000 = 0x66;
static const uint8_t RF_FDEVMSB_110000 = 0x07;
static const uint8_t RF_FDEVLSB_110000 = 0x0A;
static const uint8_t RF_FDEVMSB_120000 = 0x07;
static const uint8_t RF_FDEVLSB_120000 = 0xAE;
static const uint8_t RF_FDEVMSB_130000 = 0x08;
static const uint8_t RF_FDEVLSB_130000 = 0x52;
static const uint8_t RF_FDEVMSB_140000 = 0x08;
static const uint8_t RF_FDEVLSB_140000 = 0xF6;
static const uint8_t RF_FDEVMSB_150000 = 0x09;
static const uint8_t RF_FDEVLSB_150000 = 0x9A;
static const uint8_t RF_FDEVMSB_160000 = 0x0A;
static const uint8_t RF_FDEVLSB_160000 = 0x3D;
static const uint8_t RF_FDEVMSB_170000 = 0x0A;
static const uint8_t RF_FDEVLSB_170000 = 0xE1;
static const uint8_t RF_FDEVMSB_180000 = 0x0B;
static const uint8_t RF_FDEVLSB_180000 = 0x85;
static const uint8_t RF_FDEVMSB_190000 = 0x0C;
static const uint8_t RF_FDEVLSB_190000 = 0x29;
static const uint8_t RF_FDEVMSB_200000 = 0x0C;
static const uint8_t RF_FDEVLSB_200000 = 0xCD;
static const uint8_t RF_FDEVMSB_210000 = 0x0D;
static const uint8_t RF_FDEVLSB_210000 = 0x71;
static const uint8_t RF_FDEVMSB_220000 = 0x0E;
static const uint8_t RF_FDEVLSB_220000 = 0x14;
static const uint8_t RF_FDEVMSB_230000 = 0x0E;
static const uint8_t RF_FDEVLSB_230000 = 0xB8;
static const uint8_t RF_FDEVMSB_240000 = 0x0F;
static const uint8_t RF_FDEVLSB_240000 = 0x5C;
static const uint8_t RF_FDEVMSB_250000 = 0x10;
static const uint8_t RF_FDEVLSB_250000 = 0x00;
static const uint8_t RF_FDEVMSB_260000 = 0x10;
static const uint8_t RF_FDEVLSB_260000 = 0xA4;
static const uint8_t RF_FDEVMSB_270000 = 0x11;
static const uint8_t RF_FDEVLSB_270000 = 0x48;
static const uint8_t RF_FDEVMSB_280000 = 0x11;
static const uint8_t RF_FDEVLSB_280000 = 0xEC;
static const uint8_t RF_FDEVMSB_290000 = 0x12;
static const uint8_t RF_FDEVLSB_290000 = 0x8F;
static const uint8_t RF_FDEVMSB_300000 = 0x13;
static const uint8_t RF_FDEVLSB_300000 = 0x33;


// RegFrf (MHz) - carrier frequency
// 315Mhz band
static const uint8_t RF_FRFMSB_314 = 0x4E;
static const uint8_t RF_FRFMID_314 = 0x80;
static const uint8_t RF_FRFLSB_314 = 0x00;
static const uint8_t RF_FRFMSB_315 = 0x4E;
static const uint8_t RF_FRFMID_315 = 0xC0;
static const uint8_t RF_FRFLSB_315 = 0x00;
static const uint8_t RF_FRFMSB_316 = 0x4F;
static const uint8_t RF_FRFMID_316 = 0x00;
static const uint8_t RF_FRFLSB_316 = 0x00;
// 433mhz band
static const uint8_t RF_FRFMSB_433 = 0x6C;
static const uint8_t RF_FRFMID_433 = 0x40;
static const uint8_t RF_FRFLSB_433 = 0x00;
static const uint8_t RF_FRFMSB_434 = 0x6C;
static const uint8_t RF_FRFMID_434 = 0x80;
static const uint8_t RF_FRFLSB_434 = 0x00;
static const uint8_t RF_FRFMSB_435 = 0x6C;
static const uint8_t RF_FRFMID_435 = 0xC0;
static const uint8_t RF_FRFLSB_435 = 0x00;
// 868Mhz band
static const uint8_t RF_FRFMSB_863 = 0xD7;
static const uint8_t RF_FRFMID_863 = 0xC0;
static const uint8_t RF_FRFLSB_863 = 0x00;
static const uint8_t RF_FRFMSB_864 = 0xD8;
static const uint8_t RF_FRFMID_864 = 0x00;
static const uint8_t RF_FRFLSB_864 = 0x00;
static const uint8_t RF_FRFMSB_865 = 0xD8;
static const uint8_t RF_FRFMID_865 = 0x40;
static const uint8_t RF_FRFLSB_865 = 0x00;
static const uint8_t RF_FRFMSB_866 = 0xD8;
static const uint8_t RF_FRFMID_866 = 0x80;
static const uint8_t RF_FRFLSB_866 = 0x00;
static const uint8_t RF_FRFMSB_867 = 0xD8;
static const uint8_t RF_FRFMID_867 = 0xC0;
static const uint8_t RF_FRFLSB_867 = 0x00;
static const uint8_t RF_FRFMSB_868 = 0xD9;
static const uint8_t RF_FRFMID_868 = 0x00;
static const uint8_t RF_FRFLSB_868 = 0x00;
static const uint8_t RF_FRFMSB_869 = 0xD9;
static const uint8_t RF_FRFMID_869 = 0x40;
static const uint8_t RF_FRFLSB_869 = 0x00;
static const uint8_t RF_FRFMSB_870 = 0xD9;
static const uint8_t RF_FRFMID_870 = 0x80;
static const uint8_t RF_FRFLSB_870 = 0x00;
// 915Mhz band
static const uint8_t RF_FRFMSB_902 = 0xE1;
static const uint8_t RF_FRFMID_902 = 0x80;
static const uint8_t RF_FRFLSB_902 = 0x00;
static const uint8_t RF_FRFMSB_903 = 0xE1;
static const uint8_t RF_FRFMID_903 = 0xC0;
static const uint8_t RF_FRFLSB_903 = 0x00;
static const uint8_t RF_FRFMSB_904 = 0xE2;
static const uint8_t RF_FRFMID_904 = 0x00;
static const uint8_t RF_FRFLSB_904 = 0x00;
static const uint8_t RF_FRFMSB_905 = 0xE2;
static const uint8_t RF_FRFMID_905 = 0x40;
static const uint8_t RF_FRFLSB_905 = 0x00;
static const uint8_t RF_FRFMSB_906 = 0xE2;
static const uint8_t RF_FRFMID_906 = 0x80;
static const uint8_t RF_FRFLSB_906 = 0x00;
static const uint8_t RF_FRFMSB_907 = 0xE2;
static const uint8_t RF_FRFMID_907 = 0xC0;
static const uint8_t RF_FRFLSB_907 = 0x00;
static const uint8_t RF_FRFMSB_908 = 0xE3;
static const uint8_t RF_FRFMID_908 = 0x00;
static const uint8_t RF_FRFLSB_908 = 0x00;
static const uint8_t RF_FRFMSB_909 = 0xE3;
static const uint8_t RF_FRFMID_909 = 0x40;
static const uint8_t RF_FRFLSB_909 = 0x00;
static const uint8_t RF_FRFMSB_910 = 0xE3;
static const uint8_t RF_FRFMID_910 = 0x80;
static const uint8_t RF_FRFLSB_910 = 0x00;
static const uint8_t RF_FRFMSB_911 = 0xE3;
static const uint8_t RF_FRFMID_911 = 0xC0;
static const uint8_t RF_FRFLSB_911 = 0x00;
static const uint8_t RF_FRFMSB_912 = 0xE4;
static const uint8_t RF_FRFMID_912 = 0x00;
static const uint8_t RF_FRFLSB_912 = 0x00;
static const uint8_t RF_FRFMSB_913 = 0xE4;
static const uint8_t RF_FRFMID_913 = 0x40;
static const uint8_t RF_FRFLSB_913 = 0x00;
static const uint8_t RF_FRFMSB_914 = 0xE4;
static const uint8_t RF_FRFMID_914 = 0x80;
static const uint8_t RF_FRFLSB_914 = 0x00;
static const uint8_t RF_FRFMSB_915 = 0xE4;  // Default
static const uint8_t RF_FRFMID_915 = 0xC0;  // Default
static const uint8_t RF_FRFLSB_915 = 0x00;  // Default
static const uint8_t RF_FRFMSB_916 = 0xE5;
static const uint8_t RF_FRFMID_916 = 0x00;
static const uint8_t RF_FRFLSB_916 = 0x00;
static const uint8_t RF_FRFMSB_917 = 0xE5;
static const uint8_t RF_FRFMID_917 = 0x40;
static const uint8_t RF_FRFLSB_917 = 0x00;
static const uint8_t RF_FRFMSB_918 = 0xE5;
static const uint8_t RF_FRFMID_918 = 0x80;
static const uint8_t RF_FRFLSB_918 = 0x00;
static const uint8_t RF_FRFMSB_919 = 0xE5;
static const uint8_t RF_FRFMID_919 = 0xC0;
static const uint8_t RF_FRFLSB_919 = 0x00;
static const uint8_t RF_FRFMSB_920 = 0xE6;
static const uint8_t RF_FRFMID_920 = 0x00;
static const uint8_t RF_FRFLSB_920 = 0x00;
static const uint8_t RF_FRFMSB_921 = 0xE6;
static const uint8_t RF_FRFMID_921 = 0x40;
static const uint8_t RF_FRFLSB_921 = 0x00;
static const uint8_t RF_FRFMSB_922 = 0xE6;
static const uint8_t RF_FRFMID_922 = 0x80;
static const uint8_t RF_FRFLSB_922 = 0x00;
static const uint8_t RF_FRFMSB_923 = 0xE6;
static const uint8_t RF_FRFMID_923 = 0xC0;
static const uint8_t RF_FRFLSB_923 = 0x00;
static const uint8_t RF_FRFMSB_924 = 0xE7;
static const uint8_t RF_FRFMID_924 = 0x00;
static const uint8_t RF_FRFLSB_924 = 0x00;
static const uint8_t RF_FRFMSB_925 = 0xE7;
static const uint8_t RF_FRFMID_925 = 0x40;
static const uint8_t RF_FRFLSB_925 = 0x00;
static const uint8_t RF_FRFMSB_926 = 0xE7;
static const uint8_t RF_FRFMID_926 = 0x80;
static const uint8_t RF_FRFLSB_926 = 0x00;
static const uint8_t RF_FRFMSB_927 = 0xE7;
static const uint8_t RF_FRFMID_927 = 0xC0;
static const uint8_t RF_FRFLSB_927 = 0x00;
static const uint8_t RF_FRFMSB_928 = 0xE8;
static const uint8_t RF_FRFMID_928 = 0x00;
static const uint8_t RF_FRFLSB_928 = 0x00;


// RegOsc1
static const uint8_t RF_OSC1_RCCAL_START = 0x80;
static const uint8_t RF_OSC1_RCCAL_DONE = 0x40;


// RegAfcCtrl
static const uint8_t RF_AFCCTRL_LOWBETA_OFF = 0x00;  // Default
static const uint8_t RF_AFCCTRL_LOWBETA_ON = 0x20;


// RegLowBat
static const uint8_t RF_LOWBAT_MONITOR = 0x10;
static const uint8_t RF_LOWBAT_ON = 0x08;
static const uint8_t RF_LOWBAT_OFF = 0x00;  // Default

static const uint8_t RF_LOWBAT_TRIM_1695 = 0x00;
static const uint8_t RF_LOWBAT_TRIM_1764 = 0x01;
static const uint8_t RF_LOWBAT_TRIM_1835 = 0x02;  // Default
static const uint8_t RF_LOWBAT_TRIM_1905 = 0x03;
static const uint8_t RF_LOWBAT_TRIM_1976 = 0x04;
static const uint8_t RF_LOWBAT_TRIM_2045 = 0x05;
static const uint8_t RF_LOWBAT_TRIM_2116 = 0x06;
static const uint8_t RF_LOWBAT_TRIM_2185 = 0x07;


// RegListen1
static const uint8_t RF_LISTEN1_RESOL_64 = 0x50;
static const uint8_t RF_LISTEN1_RESOL_4100 = 0xA0;  // Default
static const uint8_t RF_LISTEN1_RESOL_262000 = 0xF0;

static const uint8_t RF_LISTEN1_RESOL_IDLE_64 = 0x40;
static const uint8_t RF_LISTEN1_RESOL_IDLE_4100 = 0x80;  // Default
static const uint8_t RF_LISTEN1_RESOL_IDLE_262000 0xC0;

static const uint8_t RF_LISTEN1_RESOL_RX_64 = 0x10;
static const uint8_t RF_LISTEN1_RESOL_RX_4100 = 0x20;  // Default
static const uint8_t RF_LISTEN1_RESOL_RX_262000 = 0x30;

static const uint8_t RF_LISTEN1_CRITERIA_RSSI = 0x00;  // Default
static const uint8_t RF_LISTEN1_CRITERIA_RSSIANDSYNC = 0x08;

static const uint8_t RF_LISTEN1_END_00 = 0x00;
static const uint8_t RF_LISTEN1_END_01 = 0x02;  // Default
static const uint8_t RF_LISTEN1_END_10 = 0x04;


// RegListen2
static const uint8_t RF_LISTEN2_COEFIDLE_VALUE = 0xF5; // Default


// RegListen3
static const uint8_t RF_LISTEN3_COEFRX_VALUE = 0x20; // Default


// RegVersion
static const uint8_t RF_VERSION_VER = 0x24;  // Default


// RegPaLevel
static const uint8_t RF_PALEVEL_PA0_ON = 0x80;  // Default
static const uint8_t RF_PALEVEL_PA0_OFF = 0x00;
static const uint8_t RF_PALEVEL_PA1_ON = 0x40;
static const uint8_t RF_PALEVEL_PA1_OFF = 0x00;  // Default
static const uint8_t RF_PALEVEL_PA2_ON = 0x20;
static const uint8_t RF_PALEVEL_PA2_OFF = 0x00;  // Default

static const uint8_t RF_PALEVEL_OUTPUTPOWER_00000 = 0x00;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_00001 = 0x01;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_00010 = 0x02;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_00011 = 0x03;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_00100 = 0x04;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_00101 = 0x05;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_00110 = 0x06;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_00111 = 0x07;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_01000 = 0x08;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_01001 = 0x09;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_01010 = 0x0A;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_01011 = 0x0B;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_01100 = 0x0C;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_01101 = 0x0D;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_01110 = 0x0E;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_01111 = 0x0F;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_10000 = 0x10;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_10001 = 0x11;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_10010 = 0x12;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_10011 = 0x13;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_10100 = 0x14;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_10101 = 0x15;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_10110 = 0x16;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_10111 = 0x17;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_11000 = 0x18;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_11001 = 0x19;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_11010 = 0x1A;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_11011 = 0x1B;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_11100 = 0x1C;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_11101 = 0x1D;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_11110 = 0x1E;
static const uint8_t RF_PALEVEL_OUTPUTPOWER_11111 = 0x1F; // Default


// RegPaRamp
static const uint8_t RF_PARAMP_3400 = 0x00;
static const uint8_t RF_PARAMP_2000 = 0x01;
static const uint8_t RF_PARAMP_1000 = 0x02;
static const uint8_t RF_PARAMP_500 = 0x03;
static const uint8_t RF_PARAMP_250 = 0x04;
static const uint8_t RF_PARAMP_125 = 0x05;
static const uint8_t RF_PARAMP_100 = 0x06;
static const uint8_t RF_PARAMP_62 = 0x07;
static const uint8_t RF_PARAMP_50 = 0x08;
static const uint8_t RF_PARAMP_40 = 0x09; // Default
static const uint8_t RF_PARAMP_31 = 0x0A;
static const uint8_t RF_PARAMP_25 = 0x0B;
static const uint8_t RF_PARAMP_20 = 0x0C;
static const uint8_t RF_PARAMP_15 = 0x0D;
static const uint8_t RF_PARAMP_12 = 0x0E;
static const uint8_t RF_PARAMP_10 = 0x0F;


// RegOcp
static const uint8_t RF_OCP_OFF = 0x0F;
static const uint8_t RF_OCP_ON = 0x1A;  // Default

static const uint8_t RF_OCP_TRIM_45 = 0x00;
static const uint8_t RF_OCP_TRIM_50 = 0x01;
static const uint8_t RF_OCP_TRIM_55 = 0x02;
static const uint8_t RF_OCP_TRIM_60 = 0x03;
static const uint8_t RF_OCP_TRIM_65 = 0x04;
static const uint8_t RF_OCP_TRIM_70 = 0x05;
static const uint8_t RF_OCP_TRIM_75 = 0x06;
static const uint8_t RF_OCP_TRIM_80 = 0x07;
static const uint8_t RF_OCP_TRIM_85 = 0x08;
static const uint8_t RF_OCP_TRIM_90 = 0x09;
static const uint8_t RF_OCP_TRIM_95 = 0x0A;  // Default
static const uint8_t RF_OCP_TRIM_100 = 0x0B;
static const uint8_t RF_OCP_TRIM_105 = 0x0C;
static const uint8_t RF_OCP_TRIM_110 = 0x0D;
static const uint8_t RF_OCP_TRIM_115 = 0x0E;
static const uint8_t RF_OCP_TRIM_120 = 0x0F;


// RegAgcRef - not present on RFM69/SX1231
static const uint8_t RF_AGCREF_AUTO_ON = 0x40;  // Default
static const uint8_t RF_AGCREF_AUTO_OFF = 0x00;

static const uint8_t RF_AGCREF_LEVEL_MINUS80 = 0x00;  // Default
static const uint8_t RF_AGCREF_LEVEL_MINUS81 = 0x01;
static const uint8_t RF_AGCREF_LEVEL_MINUS82 = 0x02;
static const uint8_t RF_AGCREF_LEVEL_MINUS83 = 0x03;
static const uint8_t RF_AGCREF_LEVEL_MINUS84 = 0x04;
static const uint8_t RF_AGCREF_LEVEL_MINUS85 = 0x05;
static const uint8_t RF_AGCREF_LEVEL_MINUS86 = 0x06;
static const uint8_t RF_AGCREF_LEVEL_MINUS87 = 0x07;
static const uint8_t RF_AGCREF_LEVEL_MINUS88 = 0x08;
static const uint8_t RF_AGCREF_LEVEL_MINUS89 = 0x09;
static const uint8_t RF_AGCREF_LEVEL_MINUS90 = 0x0A;
static const uint8_t RF_AGCREF_LEVEL_MINUS91 = 0x0B;
static const uint8_t RF_AGCREF_LEVEL_MINUS92 = 0x0C;
static const uint8_t RF_AGCREF_LEVEL_MINUS93 = 0x0D;
static const uint8_t RF_AGCREF_LEVEL_MINUS94 = 0x0E;
static const uint8_t RF_AGCREF_LEVEL_MINUS95 = 0x0F;
static const uint8_t RF_AGCREF_LEVEL_MINUS96 = 0x10;
static const uint8_t RF_AGCREF_LEVEL_MINUS97 = 0x11;
static const uint8_t RF_AGCREF_LEVEL_MINUS98 = 0x12;
static const uint8_t RF_AGCREF_LEVEL_MINUS99 = 0x13;
static const uint8_t RF_AGCREF_LEVEL_MINUS100 = 0x14;
static const uint8_t RF_AGCREF_LEVEL_MINUS101 = 0x15;
static const uint8_t RF_AGCREF_LEVEL_MINUS102 = 0x16;
static const uint8_t RF_AGCREF_LEVEL_MINUS103 = 0x17;
static const uint8_t RF_AGCREF_LEVEL_MINUS104 = 0x18;
static const uint8_t RF_AGCREF_LEVEL_MINUS105 = 0x19;
static const uint8_t RF_AGCREF_LEVEL_MINUS106 = 0x1A;
static const uint8_t RF_AGCREF_LEVEL_MINUS107 = 0x1B;
static const uint8_t RF_AGCREF_LEVEL_MINUS108 = 0x1C;
static const uint8_t RF_AGCREF_LEVEL_MINUS109 = 0x1D;
static const uint8_t RF_AGCREF_LEVEL_MINUS110 = 0x1E;
static const uint8_t RF_AGCREF_LEVEL_MINUS111 = 0x1F;
static const uint8_t RF_AGCREF_LEVEL_MINUS112 = 0x20;
static const uint8_t RF_AGCREF_LEVEL_MINUS113 = 0x21;
static const uint8_t RF_AGCREF_LEVEL_MINUS114 = 0x22;
static const uint8_t RF_AGCREF_LEVEL_MINUS115 = 0x23;
static const uint8_t RF_AGCREF_LEVEL_MINUS116 = 0x24;
static const uint8_t RF_AGCREF_LEVEL_MINUS117 = 0x25;
static const uint8_t RF_AGCREF_LEVEL_MINUS118 = 0x26;
static const uint8_t RF_AGCREF_LEVEL_MINUS119 = 0x27;
static const uint8_t RF_AGCREF_LEVEL_MINUS120 = 0x28;
static const uint8_t RF_AGCREF_LEVEL_MINUS121 = 0x29;
static const uint8_t RF_AGCREF_LEVEL_MINUS122 = 0x2A;
static const uint8_t RF_AGCREF_LEVEL_MINUS123 = 0x2B;
static const uint8_t RF_AGCREF_LEVEL_MINUS124 = 0x2C;
static const uint8_t RF_AGCREF_LEVEL_MINUS125 = 0x2D;
static const uint8_t RF_AGCREF_LEVEL_MINUS126 = 0x2E;
static const uint8_t RF_AGCREF_LEVEL_MINUS127 = 0x2F;
static const uint8_t RF_AGCREF_LEVEL_MINUS128 = 0x30;
static const uint8_t RF_AGCREF_LEVEL_MINUS129 = 0x31;
static const uint8_t RF_AGCREF_LEVEL_MINUS130 = 0x32;
static const uint8_t RF_AGCREF_LEVEL_MINUS131 = 0x33;
static const uint8_t RF_AGCREF_LEVEL_MINUS132 = 0x34;
static const uint8_t RF_AGCREF_LEVEL_MINUS133 = 0x35;
static const uint8_t RF_AGCREF_LEVEL_MINUS134 = 0x36;
static const uint8_t RF_AGCREF_LEVEL_MINUS135 = 0x37;
static const uint8_t RF_AGCREF_LEVEL_MINUS136 = 0x38;
static const uint8_t RF_AGCREF_LEVEL_MINUS137 = 0x39;
static const uint8_t RF_AGCREF_LEVEL_MINUS138 = 0x3A;
static const uint8_t RF_AGCREF_LEVEL_MINUS139 = 0x3B;
static const uint8_t RF_AGCREF_LEVEL_MINUS140 = 0x3C;
static const uint8_t RF_AGCREF_LEVEL_MINUS141 = 0x3D;
static const uint8_t RF_AGCREF_LEVEL_MINUS142 = 0x3E;
static const uint8_t RF_AGCREF_LEVEL_MINUS143 = 0x3F;


// RegAgcThresh1 - not present on RFM69/SX1231
static const uint8_t RF_AGCTHRESH1_SNRMARGIN_000 = 0x00;
static const uint8_t RF_AGCTHRESH1_SNRMARGIN_001 = 0x20;
static const uint8_t RF_AGCTHRESH1_SNRMARGIN_010 = 0x40;
static const uint8_t RF_AGCTHRESH1_SNRMARGIN_011 = 0x60;
static const uint8_t RF_AGCTHRESH1_SNRMARGIN_100 = 0x80;
static const uint8_t RF_AGCTHRESH1_SNRMARGIN_101 = 0xA0;  // Default
static const uint8_t RF_AGCTHRESH1_SNRMARGIN_110 = 0xC0;
static const uint8_t RF_AGCTHRESH1_SNRMARGIN_111 = 0xE0;

static const uint8_t RF_AGCTHRESH1_STEP1_0 = 0x00;
static const uint8_t RF_AGCTHRESH1_STEP1_1 = 0x01;
static const uint8_t RF_AGCTHRESH1_STEP1_2 = 0x02;
static const uint8_t RF_AGCTHRESH1_STEP1_3 = 0x03;
static const uint8_t RF_AGCTHRESH1_STEP1_4 = 0x04;
static const uint8_t RF_AGCTHRESH1_STEP1_5 = 0x05;
static const uint8_t RF_AGCTHRESH1_STEP1_6 = 0x06;
static const uint8_t RF_AGCTHRESH1_STEP1_7 = 0x07;
static const uint8_t RF_AGCTHRESH1_STEP1_8 = 0x08;
static const uint8_t RF_AGCTHRESH1_STEP1_9 = 0x09;
static const uint8_t RF_AGCTHRESH1_STEP1_10 = 0x0A;
static const uint8_t RF_AGCTHRESH1_STEP1_11 = 0x0B;
static const uint8_t RF_AGCTHRESH1_STEP1_12 = 0x0C;
static const uint8_t RF_AGCTHRESH1_STEP1_13 = 0x0D;
static const uint8_t RF_AGCTHRESH1_STEP1_14 = 0x0E;
static const uint8_t RF_AGCTHRESH1_STEP1_15 = 0x0F;
static const uint8_t RF_AGCTHRESH1_STEP1_16 = 0x10;  // Default
static const uint8_t RF_AGCTHRESH1_STEP1_17 = 0x11;
static const uint8_t RF_AGCTHRESH1_STEP1_18 = 0x12;
static const uint8_t RF_AGCTHRESH1_STEP1_19 = 0x13;
static const uint8_t RF_AGCTHRESH1_STEP1_20 = 0x14;
static const uint8_t RF_AGCTHRESH1_STEP1_21 = 0x15;
static const uint8_t RF_AGCTHRESH1_STEP1_22 = 0x16;
static const uint8_t RF_AGCTHRESH1_STEP1_23 = 0x17;
static const uint8_t RF_AGCTHRESH1_STEP1_24 = 0x18;
static const uint8_t RF_AGCTHRESH1_STEP1_25 = 0x19;
static const uint8_t RF_AGCTHRESH1_STEP1_26 = 0x1A;
static const uint8_t RF_AGCTHRESH1_STEP1_27 = 0x1B;
static const uint8_t RF_AGCTHRESH1_STEP1_28 = 0x1C;
static const uint8_t RF_AGCTHRESH1_STEP1_29 = 0x1D;
static const uint8_t RF_AGCTHRESH1_STEP1_30 = 0x1E;
static const uint8_t RF_AGCTHRESH1_STEP1_31 = 0x1F;


// RegAgcThresh2 - not present on RFM69/SX1231
static const uint8_t RF_AGCTHRESH2_STEP2_0 = 0x00;
static const uint8_t RF_AGCTHRESH2_STEP2_1 = 0x10;
static const uint8_t RF_AGCTHRESH2_STEP2_2 = 0x20;
static const uint8_t RF_AGCTHRESH2_STEP2_3 = 0x30; // XXX wrong -- Default
static const uint8_t RF_AGCTHRESH2_STEP2_4 = 0x40;
static const uint8_t RF_AGCTHRESH2_STEP2_5 = 0x50;
static const uint8_t RF_AGCTHRESH2_STEP2_6 = 0x60;
static const uint8_t RF_AGCTHRESH2_STEP2_7 = 0x70; // default
static const uint8_t RF_AGCTHRESH2_STEP2_8 = 0x80;
static const uint8_t RF_AGCTHRESH2_STEP2_9 = 0x90;
static const uint8_t RF_AGCTHRESH2_STEP2_10 = 0xA0;
static const uint8_t RF_AGCTHRESH2_STEP2_11 = 0xB0;
static const uint8_t RF_AGCTHRESH2_STEP2_12 = 0xC0;
static const uint8_t RF_AGCTHRESH2_STEP2_13 = 0xD0;
static const uint8_t RF_AGCTHRESH2_STEP2_14 = 0xE0;
static const uint8_t RF_AGCTHRESH2_STEP2_15 = 0xF0;

static const uint8_t RF_AGCTHRESH2_STEP3_0 = 0x00;
static const uint8_t RF_AGCTHRESH2_STEP3_1 = 0x01;
static const uint8_t RF_AGCTHRESH2_STEP3_2 = 0x02;
static const uint8_t RF_AGCTHRESH2_STEP3_3 = 0x03;
static const uint8_t RF_AGCTHRESH2_STEP3_4 = 0x04;
static const uint8_t RF_AGCTHRESH2_STEP3_5 = 0x05;
static const uint8_t RF_AGCTHRESH2_STEP3_6 = 0x06;
static const uint8_t RF_AGCTHRESH2_STEP3_7 = 0x07;
static const uint8_t RF_AGCTHRESH2_STEP3_8 = 0x08;
static const uint8_t RF_AGCTHRESH2_STEP3_9 = 0x09;
static const uint8_t RF_AGCTHRESH2_STEP3_10 = 0x0A;
static const uint8_t RF_AGCTHRESH2_STEP3_11 = 0x0B; // Default
static const uint8_t RF_AGCTHRESH2_STEP3_12 = 0x0C;
static const uint8_t RF_AGCTHRESH2_STEP3_13 = 0x0D;
static const uint8_t RF_AGCTHRESH2_STEP3_14 = 0x0E;
static const uint8_t RF_AGCTHRESH2_STEP3_15 = 0x0F;


// RegAgcThresh3 - not present on RFM69/SX1231
static const uint8_t RF_AGCTHRESH3_STEP4_0 = 0x00;
static const uint8_t RF_AGCTHRESH3_STEP4_1 = 0x10;
static const uint8_t RF_AGCTHRESH3_STEP4_2 = 0x20;
static const uint8_t RF_AGCTHRESH3_STEP4_3 = 0x30;
static const uint8_t RF_AGCTHRESH3_STEP4_4 = 0x40;
static const uint8_t RF_AGCTHRESH3_STEP4_5 = 0x50;
static const uint8_t RF_AGCTHRESH3_STEP4_6 = 0x60;
static const uint8_t RF_AGCTHRESH3_STEP4_7 = 0x70;
static const uint8_t RF_AGCTHRESH3_STEP4_8 = 0x80;
static const uint8_t RF_AGCTHRESH3_STEP4_9 = 0x90; // Default
static const uint8_t RF_AGCTHRESH3_STEP4_10 = 0xA0;
static const uint8_t RF_AGCTHRESH3_STEP4_11 = 0xB0;
static const uint8_t RF_AGCTHRESH3_STEP4_12 = 0xC0;
static const uint8_t RF_AGCTHRESH3_STEP4_13 = 0xD0;
static const uint8_t RF_AGCTHRESH3_STEP4_14 = 0xE0;
static const uint8_t RF_AGCTHRESH3_STEP4_15 = 0xF0;

static const uint8_t RF_AGCTHRESH3_STEP5_0 = 0x00;
static const uint8_t RF_AGCTHRESH3_STEP5_1 = 0x01;
static const uint8_t RF_AGCTHRESH3_STEP5_2 = 0x02;
static const uint8_t RF_AGCTHRESH3_STEP5_3 = 0x03;
static const uint8_t RF_AGCTHRESH3_STEP5_4 = 0x04;
static const uint8_t RF_AGCTHRESH3_STEP5_5 = 0x05;
static const uint8_t RF_AGCTHRESH3_STEP5_6 = 0x06;
static const uint8_t RF_AGCTHRESH3_STEP5_7 = 0x07;
static const uint8_t RF_AGCTHRES33_STEP5_8 = 0x08;
static const uint8_t RF_AGCTHRESH3_STEP5_9 = 0x09;
static const uint8_t RF_AGCTHRESH3_STEP5_10 = 0x0A;
static const uint8_t RF_AGCTHRESH3_STEP5_11 = 0x0B; // Default
static const uint8_t RF_AGCTHRESH3_STEP5_12 = 0x0C;
static const uint8_t RF_AGCTHRESH3_STEP5_13 = 0x0D;
static const uint8_t RF_AGCTHRESH3_STEP5_14 = 0x0E;
static const uint8_t RF_AGCTHRESH3_STEP5_15 = 0x0F;


// RegLna
static const uint8_t RF_LNA_ZIN_50 = 0x00;  // Reset value
static const uint8_t RF_LNA_ZIN_200 = 0x80;  // Recommended default

static const uint8_t RF_LNA_LOWPOWER_OFF = 0x00;  // Default
static const uint8_t RF_LNA_LOWPOWER_ON = 0x40;

static const uint8_t RF_LNA_CURRENTGAIN = 0x08;

static const uint8_t RF_LNA_GAINSELECT_AUTO = 0x00;  // Default
static const uint8_t RF_LNA_GAINSELECT_MAX = 0x01;
static const uint8_t RF_LNA_GAINSELECT_MAXMINUS6 = 0x02;
static const uint8_t RF_LNA_GAINSELECT_MAXMINUS12 = 0x03;
static const uint8_t RF_LNA_GAINSELECT_MAXMINUS24 = 0x04;
static const uint8_t RF_LNA_GAINSELECT_MAXMINUS36 = 0x05;
static const uint8_t RF_LNA_GAINSELECT_MAXMINUS48 = 0x06;


// RegRxBw
static const uint8_t RF_RXBW_DCCFREQ_000 = 0x00;
static const uint8_t RF_RXBW_DCCFREQ_001 = 0x20;
static const uint8_t RF_RXBW_DCCFREQ_010 = 0x40; // Recommended default
static const uint8_t RF_RXBW_DCCFREQ_011 = 0x60;
static const uint8_t RF_RXBW_DCCFREQ_100 = 0x80;  // Reset value
static const uint8_t RF_RXBW_DCCFREQ_101 = 0xA0;
static const uint8_t RF_RXBW_DCCFREQ_110 = 0xC0;
static const uint8_t RF_RXBW_DCCFREQ_111 = 0xE0;

static const uint8_t RF_RXBW_MANT_16 = 0x00;  // Reset value
static const uint8_t RF_RXBW_MANT_20 = 0x08;
static const uint8_t RF_RXBW_MANT_24 = 0x10; // Recommended default

static const uint8_t RF_RXBW_EXP_0 = 0x00;
static const uint8_t RF_RXBW_EXP_1 = 0x01;
static const uint8_t RF_RXBW_EXP_2 = 0x02;
static const uint8_t RF_RXBW_EXP_3 = 0x03;
static const uint8_t RF_RXBW_EXP_4 = 0x04;
static const uint8_t RF_RXBW_EXP_5 = 0x05; // Recommended default
static const uint8_t RF_RXBW_EXP_6 = 0x06;  // Reset value
static const uint8_t RF_RXBW_EXP_7 = 0x07;


// RegAfcBw
static const uint8_t RF_AFCBW_DCCFREQAFC_000 = 0x00;
static const uint8_t RF_AFCBW_DCCFREQAFC_001 = 0x20;
static const uint8_t RF_AFCBW_DCCFREQAFC_010 = 0x40;
static const uint8_t RF_AFCBW_DCCFREQAFC_011 = 0x60;
static const uint8_t RF_AFCBW_DCCFREQAFC_100 = 0x80; // Default
static const uint8_t RF_AFCBW_DCCFREQAFC_101 = 0xA0;
static const uint8_t RF_AFCBW_DCCFREQAFC_110 = 0xC0;
static const uint8_t RF_AFCBW_DCCFREQAFC_111 = 0xE0;

static const uint8_t RF_AFCBW_MANTAFC_16 = 0x00;
static const uint8_t RF_AFCBW_MANTAFC_20 = 0x08;  // Default
static const uint8_t RF_AFCBW_MANTAFC_24 = 0x10;

static const uint8_t RF_AFCBW_EXPAFC_0 = 0x00;
static const uint8_t RF_AFCBW_EXPAFC_1 = 0x01;
static const uint8_t RF_AFCBW_EXPAFC_2 = 0x02;  // Reset value
static const uint8_t RF_AFCBW_EXPAFC_3 = 0x03;  // Recommended default
static const uint8_t RF_AFCBW_EXPAFC_4 = 0x04;
static const uint8_t RF_AFCBW_EXPAFC_5 = 0x05;
static const uint8_t RF_AFCBW_EXPAFC_6 = 0x06;
static const uint8_t RF_AFCBW_EXPAFC_7 = 0x07;


// RegOokPeak
static const uint8_t RF_OOKPEAK_THRESHTYPE_FIXED = 0x00;
static const uint8_t RF_OOKPEAK_THRESHTYPE_PEAK = 0x40;  // Default
static const uint8_t RF_OOKPEAK_THRESHTYPE_AVERAGE = 0x80;

static const uint8_t RF_OOKPEAK_PEAKTHRESHSTEP_000 = 0x00;  // Default
static const uint8_t RF_OOKPEAK_PEAKTHRESHSTEP_001 = 0x08;
static const uint8_t RF_OOKPEAK_PEAKTHRESHSTEP_010 = 0x10;
static const uint8_t RF_OOKPEAK_PEAKTHRESHSTEP_011 = 0x18;
static const uint8_t RF_OOKPEAK_PEAKTHRESHSTEP_100 = 0x20;
static const uint8_t RF_OOKPEAK_PEAKTHRESHSTEP_101 = 0x28;
static const uint8_t RF_OOKPEAK_PEAKTHRESHSTEP_110 = 0x30;
static const uint8_t RF_OOKPEAK_PEAKTHRESHSTEP_111 = 0x38;

static const uint8_t RF_OOKPEAK_PEAKTHRESHDEC_000 = 0x00;  // Default
static const uint8_t RF_OOKPEAK_PEAKTHRESHDEC_001 = 0x01;
static const uint8_t RF_OOKPEAK_PEAKTHRESHDEC_010 = 0x02;
static const uint8_t RF_OOKPEAK_PEAKTHRESHDEC_011 = 0x03;
static const uint8_t RF_OOKPEAK_PEAKTHRESHDEC_100 = 0x04;
static const uint8_t RF_OOKPEAK_PEAKTHRESHDEC_101 = 0x05;
static const uint8_t RF_OOKPEAK_PEAKTHRESHDEC_110 = 0x06;
static const uint8_t RF_OOKPEAK_PEAKTHRESHDEC_111 = 0x07;


// RegOokAvg
static const uint8_t RF_OOKAVG_AVERAGETHRESHFILT_00 = 0x00;
static const uint8_t RF_OOKAVG_AVERAGETHRESHFILT_01 = 0x40;
static const uint8_t RF_OOKAVG_AVERAGETHRESHFILT_10 = 0x80; // Default
static const uint8_t RF_OOKAVG_AVERAGETHRESHFILT_11 = 0xC0;


// RegOokFix
static const uint8_t RF_OOKFIX_FIXEDTHRESH_VALUE = 0x06;  // Default


// RegAfcFei
static const uint8_t RF_AFCFEI_FEI_DONE = 0x40;
static const uint8_t RF_AFCFEI_FEI_START = 0x20;
static const uint8_t RF_AFCFEI_AFC_DONE = 0x10;
static const uint8_t RF_AFCFEI_AFCAUTOCLEAR_ON = 0x08;
static const uint8_t RF_AFCFEI_AFCAUTOCLEAR_OFF = 0x00;  // Default

static const uint8_t RF_AFCFEI_AFCAUTO_ON = 0x04;
static const uint8_t RF_AFCFEI_AFCAUTO_OFF = 0x00;  // Default

static const uint8_t RF_AFCFEI_AFC_CLEAR = 0x02;
static const uint8_t RF_AFCFEI_AFC_START = 0x01;


// RegRssiConfig
static const uint8_t RF_RSSI_FASTRX_ON = 0x08;  // not present on RFM69/SX1231
static const uint8_t RF_RSSI_FASTRX_OFF = 0x00;  // Default

static const uint8_t RF_RSSI_DONE = 0x02;
static const uint8_t RF_RSSI_START = 0x01;


// RegDioMapping1
static const uint8_t RF_DIOMAPPING1_DIO0_00 = 0x00;  // Default
static const uint8_t RF_DIOMAPPING1_DIO0_01 = 0x40;
static const uint8_t RF_DIOMAPPING1_DIO0_10 = 0x80;
static const uint8_t RF_DIOMAPPING1_DIO0_11 = 0xC0;

static const uint8_t RF_DIOMAPPING1_DIO1_00 = 0x00;  // Default
static const uint8_t RF_DIOMAPPING1_DIO1_01 = 0x10;
static const uint8_t RF_DIOMAPPING1_DIO1_10 = 0x20;
static const uint8_t RF_DIOMAPPING1_DIO1_11 = 0x30;

static const uint8_t RF_DIOMAPPING1_DIO2_00 = 0x00;  // Default
static const uint8_t RF_DIOMAPPING1_DIO2_01 = 0x04;
static const uint8_t RF_DIOMAPPING1_DIO2_10 = 0x08;
static const uint8_t RF_DIOMAPPING1_DIO2_11 = 0x0C;

static const uint8_t RF_DIOMAPPING1_DIO3_00 = 0x00;  // Default
static const uint8_t RF_DIOMAPPING1_DIO3_01 = 0x01;
static const uint8_t RF_DIOMAPPING1_DIO3_10 = 0x02;
static const uint8_t RF_DIOMAPPING1_DIO3_11 = 0x03;


// RegDioMapping2
static const uint8_t RF_DIOMAPPING2_DIO4_00 = 0x00;  // Default
static const uint8_t RF_DIOMAPPING2_DIO4_01 = 0x40;
static const uint8_t RF_DIOMAPPING2_DIO4_10 = 0x80;
static const uint8_t RF_DIOMAPPING2_DIO4_11 = 0xC0;

static const uint8_t RF_DIOMAPPING2_DIO5_00 = 0x00;  // Default
static const uint8_t RF_DIOMAPPING2_DIO5_01 = 0x10;
static const uint8_t RF_DIOMAPPING2_DIO5_10 = 0x20;
static const uint8_t RF_DIOMAPPING2_DIO5_11 = 0x30;

static const uint8_t RF_DIOMAPPING2_CLKOUT_32 = 0x00;
static const uint8_t RF_DIOMAPPING2_CLKOUT_16 = 0x01;
static const uint8_t RF_DIOMAPPING2_CLKOUT_8 = 0x02;
static const uint8_t RF_DIOMAPPING2_CLKOUT_4 = 0x03;
static const uint8_t RF_DIOMAPPING2_CLKOUT_2 = 0x04;
static const uint8_t RF_DIOMAPPING2_CLKOUT_1 = 0x05;  // Reset value
static const uint8_t RF_DIOMAPPING2_CLKOUT_RC = 0x06;
static const uint8_t RF_DIOMAPPING2_CLKOUT_OFF = 0x07;  // Recommended default


// RegIrqFlags1
static const uint8_t RF_IRQFLAGS1_MODEREADY = 0x80;
static const uint8_t RF_IRQFLAGS1_RXREADY = 0x40;
static const uint8_t RF_IRQFLAGS1_TXREADY = 0x20;
static const uint8_t RF_IRQFLAGS1_PLLLOCK = 0x10;
static const uint8_t RF_IRQFLAGS1_RSSI = 0x08;
static const uint8_t RF_IRQFLAGS1_TIMEOUT = 0x04;
static const uint8_t RF_IRQFLAGS1_AUTOMODE = 0x02;
static const uint8_t RF_IRQFLAGS1_SYNCADDRESSMATCH = 0x01;


// RegIrqFlags2
static const uint8_t RF_IRQFLAGS2_FIFOFULL = 0x80;
static const uint8_t RF_IRQFLAGS2_FIFONOTEMPTY = 0x40;
static const uint8_t RF_IRQFLAGS2_FIFOLEVEL = 0x20;
static const uint8_t RF_IRQFLAGS2_FIFOOVERRUN = 0x10;
static const uint8_t RF_IRQFLAGS2_PACKETSENT = 0x08;
static const uint8_t RF_IRQFLAGS2_PAYLOADREADY = 0x04;
static const uint8_t RF_IRQFLAGS2_CRCOK = 0x02;
static const uint8_t RF_IRQFLAGS2_LOWBAT = 0x01;  // not present on RFM69/SX1231


// RegRssiThresh
static const uint8_t RF_RSSITHRESH_VALUE = 0xE4; // Default


// RegRxTimeout1
static const uint8_t RF_RXTIMEOUT1_RXSTART_VALUE = 0x00;  // Default


// RegRxTimeout2
static const uint8_t RF_RXTIMEOUT2_RSSITHRESH_VALUE = 0x00;  // Default


// RegPreamble
static const uint8_t RF_PREAMBLESIZE_MSB_VALUE = 0x00;  // Default
static const uint8_t RF_PREAMBLESIZE_LSB_VALUE = 0x03;  // Default


// RegSyncConfig
static const uint8_t RF_SYNC_ON = 0x80;  // Default
static const uint8_t RF_SYNC_OFF = 0x00;

static const uint8_t RF_SYNC_FIFOFILL_AUTO = 0x00;  // Default -- when sync interrupt occurs
static const uint8_t RF_SYNC_FIFOFILL_MANUAL = 0x40;

static const uint8_t RF_SYNC_SIZE_1 = 0x00;
static const uint8_t RF_SYNC_SIZE_2 = 0x08;
static const uint8_t RF_SYNC_SIZE_3 = 0x10;
static const uint8_t RF_SYNC_SIZE_4 = 0x18; // Default
static const uint8_t RF_SYNC_SIZE_5 = 0x20;
static const uint8_t RF_SYNC_SIZE_6 = 0x28;
static const uint8_t RF_SYNC_SIZE_7 = 0x30;
static const uint8_t RF_SYNC_SIZE_8 = 0x38;

static const uint8_t RF_SYNC_TOL_0 = 0x00;  // Default
static const uint8_t RF_SYNC_TOL_1 = 0x01;
static const uint8_t RF_SYNC_TOL_2 = 0x02;
static const uint8_t RF_SYNC_TOL_3 = 0x03;
static const uint8_t RF_SYNC_TOL_4 = 0x04;
static const uint8_t RF_SYNC_TOL_5 = 0x05;
static const uint8_t RF_SYNC_TOL_6 = 0x06;
static const uint8_t RF_SYNC_TOL_7 = 0x07;


// RegSyncValue1-8
static const uint8_t RF_SYNC_BYTE1_VALUE = 0x00;  // Default
static const uint8_t RF_SYNC_BYTE2_VALUE = 0x00;  // Default
static const uint8_t RF_SYNC_BYTE3_VALUE = 0x00;  // Default
static const uint8_t RF_SYNC_BYTE4_VALUE = 0x00;  // Default
static const uint8_t RF_SYNC_BYTE5_VALUE = 0x00;  // Default
static const uint8_t RF_SYNC_BYTE6_VALUE = 0x00;  // Default
static const uint8_t RF_SYNC_BYTE7_VALUE = 0x00;  // Default
static const uint8_t RF_SYNC_BYTE8_VALUE = 0x00;  // Default


// RegPacketConfig1
static const uint8_t RF_PACKET1_FORMAT_FIXED = 0x00;  // Default
static const uint8_t RF_PACKET1_FORMAT_VARIABLE = 0x80;

static const uint8_t RF_PACKET1_DCFREE_OFF = 0x00;  // Default
static const uint8_t RF_PACKET1_DCFREE_MANCHESTER = 0x20;
static const uint8_t RF_PACKET1_DCFREE_WHITENING = 0x40;

static const uint8_t RF_PACKET1_CRC_ON = 0x10;  // Default
static const uint8_t RF_PACKET1_CRC_OFF = 0x00;

static const uint8_t RF_PACKET1_CRCAUTOCLEAR_ON = 0x00;  // Default
static const uint8_t RF_PACKET1_CRCAUTOCLEAR_OFF = 0x08;

static const uint8_t RF_PACKET1_ADRSFILTERING_OFF = 0x00;  // Default
static const uint8_t RF_PACKET1_ADRSFILTERING_NODE = 0x02;
static const uint8_t RF_PACKET1_ADRSFILTERING_NODEBROADCAST = 0x04;


// RegPayloadLength
static const uint8_t RF_PAYLOADLENGTH_VALUE = 0x40;  // Default


// RegBroadcastAdrs
static const uint8_t RF_BROADCASTADDRESS_VALUE = 0x00;


// RegAutoModes
static const uint8_t RF_AUTOMODES_ENTER_OFF = 0x00;  // Default
static const uint8_t RF_AUTOMODES_ENTER_FIFONOTEMPTY = 0x20;
static const uint8_t RF_AUTOMODES_ENTER_FIFOLEVEL = 0x40;
static const uint8_t RF_AUTOMODES_ENTER_CRCOK = 0x60;
static const uint8_t RF_AUTOMODES_ENTER_PAYLOADREADY = 0x80;
static const uint8_t RF_AUTOMODES_ENTER_SYNCADRSMATCH = 0xA0;
static const uint8_t RF_AUTOMODES_ENTER_PACKETSENT = 0xC0;
static const uint8_t RF_AUTOMODES_ENTER_FIFOEMPTY = 0xE0;

static const uint8_t RF_AUTOMODES_EXIT_OFF = 0x00;  // Default
static const uint8_t RF_AUTOMODES_EXIT_FIFOEMPTY = 0x04;
static const uint8_t RF_AUTOMODES_EXIT_FIFOLEVEL = 0x08;
static const uint8_t RF_AUTOMODES_EXIT_CRCOK = 0x0C;
static const uint8_t RF_AUTOMODES_EXIT_PAYLOADREADY = 0x10;
static const uint8_t RF_AUTOMODES_EXIT_SYNCADRSMATCH = 0x14;
static const uint8_t RF_AUTOMODES_EXIT_PACKETSENT = 0x18;
static const uint8_t RF_AUTOMODES_EXIT_RXTIMEOUT = 0x1C;

static const uint8_t RF_AUTOMODES_INTERMEDIATE_SLEEP = 0x00;  // Default
static const uint8_t RF_AUTOMODES_INTERMEDIATE_STANDBY = 0x01;
static const uint8_t RF_AUTOMODES_INTERMEDIATE_RECEIVER = 0x02;
static const uint8_t RF_AUTOMODES_INTERMEDIATE_TRANSMITTER 0x03;


// RegFifoThresh
static const uint8_t RF_FIFOTHRESH_TXSTART_FIFOTHRESH = 0x00;  // Reset value
static const uint8_t RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY = 0x80;  // Recommended default

static const uint8_t RF_FIFOTHRESH_VALUE = 0x0F;  // Default


// RegPacketConfig2
static const uint8_t RF_PACKET2_RXRESTARTDELAY_1BIT = 0x00;  // Default
static const uint8_t RF_PACKET2_RXRESTARTDELAY_2BITS = 0x10;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_4BITS = 0x20;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_8BITS = 0x30;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_16BITS = 0x40;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_32BITS = 0x50;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_64BITS = 0x60;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_128BITS = 0x70;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_256BITS = 0x80;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_512BITS = 0x90;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_1024BITS = 0xA0;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_2048BITS = 0xB0;
static const uint8_t RF_PACKET2_RXRESTARTDELAY_NONE = 0xC0;
static const uint8_t RF_PACKET2_RXRESTART = 0x04;

static const uint8_t RF_PACKET2_AUTORXRESTART_ON = 0x02;  // Default
static const uint8_t RF_PACKET2_AUTORXRESTART_OFF = 0x00;

static const uint8_t RF_PACKET2_AES_ON = 0x01;
static const uint8_t RF_PACKET2_AES_OFF = 0x00;  // Default


// RegAesKey1-16
static const uint8_t RF_AESKEY1_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY2_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY3_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY4_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY5_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY6_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY7_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY8_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY9_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY10_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY11_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY12_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY13_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY14_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY15_VALUE = 0x00;  // Default
static const uint8_t RF_AESKEY16_VALUE = 0x00;  // Default


// RegTemp1
static const uint8_t RF_TEMP1_MEAS_START = 0x08;
static const uint8_t RF_TEMP1_MEAS_RUNNING = 0x04;
// not present on RFM69/SX1231
static const uint8_t RF_TEMP1_ADCLOWPOWER_ON = 0x01;  // Default
static const uint8_t RF_TEMP1_ADCLOWPOWER_OFF = 0x00;


// RegTestLna
static const uint8_t RF_TESTLNA_NORMAL = 0x1B;
static const uint8_t RF_TESTLNA_HIGH_SENSITIVITY =0x2D;


// RegTestDagc
static const uint8_t RF_DAGC_NORMAL = 0x00;  // Reset value
static const uint8_t RF_DAGC_IMPROVED_LOWBETA1 = 0x20;
static const uint8_t RF_DAGC_IMPROVED_LOWBETA0 = 0x30;  // Recommended default

}  // namespace smartweather
}  // namespace esphome