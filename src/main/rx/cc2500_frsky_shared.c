/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>

#include "platform.h"

#ifdef USE_RX_FRSKY_SPI

#include "build/debug.h"

#include "common/maths.h"

#include "drivers/rx/rx_spi.h"
#include "drivers/rx/rx_cc2500.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/rx_spi_cc2500.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/rx_spi_common.h"

#include "rx/cc2500_common.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_frsky_d.h"
#include "rx/cc2500_frsky_x.h"
#include "rx/cc2500_redpine.h"

#include "cc2500_frsky_shared.h"

static rx_spi_protocol_e spiProtocol;

static timeMs_t start_time;
static uint8_t protocolState;

uint32_t missingPackets;
timeDelta_t timeoutUs;

static uint8_t calData[255][3];
static timeMs_t timeTunedMs;
uint8_t listLength;
static uint8_t bindIdx;
static int8_t bindOffset;

typedef uint8_t handlePacketFn(uint8_t * const packet, uint8_t * const protocolState);
typedef void setRcDataFn(uint16_t *rcData, const uint8_t *payload);

static handlePacketFn *handlePacket;
static setRcDataFn *setRcData;

const uint16_t crcTable[] = {
        0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
        0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
        0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
        0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
        0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
        0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
        0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
        0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
        0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
        0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
        0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
        0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
        0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
        0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
        0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
        0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
        0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
        0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
        0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
        0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
        0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
        0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
        0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
        0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
        0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
        0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
        0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
        0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
        0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
        0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
        0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
        0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78
};

uint16_t calculateCrc(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0;
    for (unsigned i = 0; i < len; i++) {
        crc = (crc << 8) ^ (crcTable[((uint8_t)(crc >> 8) ^ *data++) & 0xFF]);
    }
    return crc;
}

static void initialise() {
    cc2500Reset();

    if (RX_SPI_FRSKY_D || RX_SPI_FRSKY_X || RX_SPI_FRSKY_X_LBT) {
        cc2500WriteReg(CC2500_02_IOCFG0,   0x01);
        cc2500WriteReg(CC2500_18_MCSM0,    0x18);
        cc2500WriteReg(CC2500_07_PKTCTRL1, 0x04);
        cc2500WriteReg(CC2500_3E_PATABLE,  0xFF);
        cc2500WriteReg(CC2500_0C_FSCTRL0,  0x00);
        cc2500WriteReg(CC2500_0D_FREQ2,    0x5C);
        cc2500WriteReg(CC2500_13_MDMCFG1,  0x23);
        cc2500WriteReg(CC2500_14_MDMCFG0,  0x7A);
        cc2500WriteReg(CC2500_19_FOCCFG,   0x16);
        cc2500WriteReg(CC2500_1A_BSCFG,    0x6C);
        cc2500WriteReg(CC2500_1B_AGCCTRL2, 0x03);
        cc2500WriteReg(CC2500_1C_AGCCTRL1, 0x40);
        cc2500WriteReg(CC2500_1D_AGCCTRL0, 0x91);
        cc2500WriteReg(CC2500_21_FREND1,   0x56);
        cc2500WriteReg(CC2500_22_FREND0,   0x10);
        cc2500WriteReg(CC2500_23_FSCAL3,   0xA9);
        cc2500WriteReg(CC2500_24_FSCAL2,   0x0A);
        cc2500WriteReg(CC2500_25_FSCAL1,   0x00);
        cc2500WriteReg(CC2500_26_FSCAL0,   0x11);
        cc2500WriteReg(CC2500_29_FSTEST,   0x59);
        cc2500WriteReg(CC2500_2C_TEST2,    0x88);
        cc2500WriteReg(CC2500_2D_TEST1,    0x31);
        cc2500WriteReg(CC2500_2E_TEST0,    0x0B);
        cc2500WriteReg(CC2500_03_FIFOTHR,  0x07);
        cc2500WriteReg(CC2500_09_ADDR,     0x00);
    }

    switch (spiProtocol) {
    case RX_SPI_FRSKY_D:
        cc2500WriteReg(CC2500_17_MCSM1,    0x0C);
        cc2500WriteReg(CC2500_0E_FREQ1,    0x76);
        cc2500WriteReg(CC2500_0F_FREQ0,    0x27);
        cc2500WriteReg(CC2500_06_PKTLEN,   0x19);
        cc2500WriteReg(CC2500_08_PKTCTRL0, 0x05);
        cc2500WriteReg(CC2500_0B_FSCTRL1,  0x08);
        cc2500WriteReg(CC2500_10_MDMCFG4,  0xAA);
        cc2500WriteReg(CC2500_11_MDMCFG3,  0x39);
        cc2500WriteReg(CC2500_12_MDMCFG2,  0x11);
        cc2500WriteReg(CC2500_15_DEVIATN,  0x42);

        break;
    case RX_SPI_FRSKY_X:
        cc2500WriteReg(CC2500_17_MCSM1,    0x0C);
        cc2500WriteReg(CC2500_0E_FREQ1,    0x76);
        cc2500WriteReg(CC2500_0F_FREQ0,    0x27);
        cc2500WriteReg(CC2500_06_PKTLEN,   0x1E);
        cc2500WriteReg(CC2500_08_PKTCTRL0, 0x01);
        cc2500WriteReg(CC2500_0B_FSCTRL1,  0x0A);
        cc2500WriteReg(CC2500_10_MDMCFG4,  0x7B);
        cc2500WriteReg(CC2500_11_MDMCFG3,  0x61);
        cc2500WriteReg(CC2500_12_MDMCFG2,  0x13);
        cc2500WriteReg(CC2500_15_DEVIATN,  0x51);

        break;
    case RX_SPI_FRSKY_X_LBT:
        cc2500WriteReg(CC2500_17_MCSM1,    0x0E);
        cc2500WriteReg(CC2500_0E_FREQ1,    0x80);
        cc2500WriteReg(CC2500_0F_FREQ0,    0x00);
        cc2500WriteReg(CC2500_06_PKTLEN,   0x23);
        cc2500WriteReg(CC2500_08_PKTCTRL0, 0x01);
        cc2500WriteReg(CC2500_0B_FSCTRL1,  0x08);
        cc2500WriteReg(CC2500_10_MDMCFG4,  0x7B);
        cc2500WriteReg(CC2500_11_MDMCFG3,  0xF8);
        cc2500WriteReg(CC2500_12_MDMCFG2,  0x03);
        cc2500WriteReg(CC2500_15_DEVIATN,  0x53);

        break;   
#if defined(USE_RX_REDPINE_SPI)        
    case RX_SPI_REDPINE:
            cc2500WriteReg(CC2500_02_IOCFG0,   0x01);
            cc2500WriteReg(CC2500_03_FIFOTHR,  0x07);       
            cc2500WriteReg(CC2500_06_PKTLEN,   0x1E);
            cc2500WriteReg(CC2500_07_PKTCTRL1, 0x04);
            cc2500WriteReg(CC2500_08_PKTCTRL0, 0x01);
            cc2500WriteReg(CC2500_09_ADDR,     0x00); 
   
        if (isRedpineFast()) {
            cc2500WriteReg(CC2500_0B_FSCTRL1,  0x0A);
            cc2500WriteReg(CC2500_0C_FSCTRL0,  0x00);        
            cc2500WriteReg(CC2500_0D_FREQ2,    0x5D);
            cc2500WriteReg(CC2500_0E_FREQ1,    0x93);
            cc2500WriteReg(CC2500_0F_FREQ0,    0xB1);        
            cc2500WriteReg(CC2500_10_MDMCFG4,  0x2D);
            cc2500WriteReg(CC2500_11_MDMCFG3,  0x3B);
            cc2500WriteReg(CC2500_12_MDMCFG2,  0x73);
            cc2500WriteReg(CC2500_13_MDMCFG1,  0x23);
            cc2500WriteReg(CC2500_14_MDMCFG0,  0x56);        
            cc2500WriteReg(CC2500_15_DEVIATN,  0x00); 
            cc2500WriteReg(CC2500_17_MCSM1,    0x0C);
            cc2500WriteReg(CC2500_18_MCSM0,    0x18);     
            cc2500WriteReg(CC2500_19_FOCCFG,   0x1D);
            cc2500WriteReg(CC2500_1A_BSCFG,    0x1C);                
            cc2500WriteReg(CC2500_1B_AGCCTRL2, 0xC7);
            cc2500WriteReg(CC2500_1C_AGCCTRL1, 0x00);
            cc2500WriteReg(CC2500_1D_AGCCTRL0, 0xB0);   
            cc2500WriteReg(CC2500_21_FREND1,   0xB6);   
            cc2500WriteReg(CC2500_22_FREND0,   0x10);
            cc2500WriteReg(CC2500_23_FSCAL3,   0xA9);
            cc2500WriteReg(CC2500_24_FSCAL2,   0x0A);
            cc2500WriteReg(CC2500_25_FSCAL1,   0x00);
            cc2500WriteReg(CC2500_26_FSCAL0,   0x11);        
            cc2500WriteReg(CC2500_29_FSTEST,   0x59);                  
            cc2500WriteReg(CC2500_2C_TEST2,    0x88);
            cc2500WriteReg(CC2500_2D_TEST1,    0x31);
            cc2500WriteReg(CC2500_2E_TEST0,    0x0B);
            cc2500WriteReg(CC2500_3E_PATABLE,  0xFF);    
        } else {
            cc2500WriteReg(CC2500_07_PKTCTRL1, 0x04);
            cc2500WriteReg(CC2500_08_PKTCTRL0, 0x01);
            cc2500WriteReg(CC2500_09_ADDR,     0x00); 
            cc2500WriteReg(CC2500_0B_FSCTRL1,  0x0A);
            cc2500WriteReg(CC2500_0C_FSCTRL0,  0x00);        
            cc2500WriteReg(CC2500_0D_FREQ2,    0x5C);
            cc2500WriteReg(CC2500_0E_FREQ1,    0x76);
            cc2500WriteReg(CC2500_0F_FREQ0,    0x27);        
            cc2500WriteReg(CC2500_10_MDMCFG4,  0x7B);
            cc2500WriteReg(CC2500_11_MDMCFG3,  0x61);
            cc2500WriteReg(CC2500_12_MDMCFG2,  0x13);
            cc2500WriteReg(CC2500_13_MDMCFG1,  0x23);
            cc2500WriteReg(CC2500_14_MDMCFG0,  0x7a);        
            cc2500WriteReg(CC2500_15_DEVIATN,  0x51); 
            cc2500WriteReg(CC2500_17_MCSM1,    0x0C);
            cc2500WriteReg(CC2500_18_MCSM0,    0x18);     
            cc2500WriteReg(CC2500_19_FOCCFG,   0x16);
            cc2500WriteReg(CC2500_1A_BSCFG,    0x6C);                
            cc2500WriteReg(CC2500_1B_AGCCTRL2, 0x43);
            cc2500WriteReg(CC2500_1C_AGCCTRL1, 0x40);
            cc2500WriteReg(CC2500_1D_AGCCTRL0, 0x91);   
            cc2500WriteReg(CC2500_21_FREND1,   0x56);   
            cc2500WriteReg(CC2500_22_FREND0,   0x10);
            cc2500WriteReg(CC2500_23_FSCAL3,   0xA9);
            cc2500WriteReg(CC2500_24_FSCAL2,   0x0A);
            cc2500WriteReg(CC2500_25_FSCAL1,   0x00);
            cc2500WriteReg(CC2500_26_FSCAL0,   0x11);        
            cc2500WriteReg(CC2500_29_FSTEST,   0x59);                  
            cc2500WriteReg(CC2500_2C_TEST2,    0x88);
            cc2500WriteReg(CC2500_2D_TEST1,    0x31);
            cc2500WriteReg(CC2500_2E_TEST0,    0x0B);
            cc2500WriteReg(CC2500_3E_PATABLE,  0xFF);  
        }

#endif
    default:

        break;
    }

    for(unsigned c = 0;c < 0xFF; c++)
    { //calibrate all channels
        cc2500Strobe(CC2500_SIDLE);
        cc2500WriteReg(CC2500_0A_CHANNR, c);
        cc2500Strobe(CC2500_SCAL);
        delayMicroseconds(900); //
        calData[c][0] = cc2500ReadReg(CC2500_23_FSCAL3);
        calData[c][1] = cc2500ReadReg(CC2500_24_FSCAL2);
        calData[c][2] = cc2500ReadReg(CC2500_25_FSCAL1);
    }
}

void initialiseData(uint8_t adr)
{
    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)rxCc2500SpiConfig()->bindOffset);
    cc2500WriteReg(CC2500_18_MCSM0, 0x8);
    cc2500WriteReg(CC2500_09_ADDR, adr ? 0x03 : rxCc2500SpiConfig()->bindTxId[0]);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0D);
    cc2500WriteReg(CC2500_19_FOCCFG, 0x16);
    delay(10);
}

static void initTuneRx(void)
{
    cc2500WriteReg(CC2500_19_FOCCFG, 0x14);
    timeTunedMs = millis();
    bindOffset = -126;
    cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
    cc2500WriteReg(CC2500_07_PKTCTRL1, 0x0C);
    cc2500WriteReg(CC2500_18_MCSM0, 0x8);

    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    cc2500WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    cc2500WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, 0);
    cc2500Strobe(CC2500_SFRX);
    cc2500Strobe(CC2500_SRX);
}

static bool tuneRx(uint8_t *packet)
{
    DEBUG_SET(DEBUG_RX_FRSKY_SPI, 0, bindOffset);

    if (bindOffset >= 126) {
        bindOffset = -126;
    }
    if ((millis() - timeTunedMs) > 50) {
        timeTunedMs = millis();
        bindOffset += 5;
        cc2500WriteReg(CC2500_0C_FSCTRL0, (uint8_t)bindOffset);
    }
    if (cc2500getGdo()) {
        uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        DEBUG_SET(DEBUG_RX_FRSKY_SPI, 1, ccLen);

        if (ccLen > RX_SPI_MAX_PAYLOAD_SIZE) {
            cc2500Strobe(CC2500_SFRX);
        } else if (ccLen) {
            cc2500ReadFifo(packet, ccLen);
            if (packet[ccLen - 1] & 0x80) {
                if (packet[2] == 0x01) {
                    uint8_t Lqi = packet[ccLen - 1] & 0x7F;
                    DEBUG_SET(DEBUG_RX_FRSKY_SPI, 2, Lqi);

                    // higher lqi represent better link quality
                    if (Lqi > 50) {
                        rxCc2500SpiConfigMutable()->bindOffset = bindOffset;
                        return true;
                    }
                }
            }
        }
    }

    return false;
}

static void initGetBind(void)
{
    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3, calData[0][0]);
    cc2500WriteReg(CC2500_24_FSCAL2, calData[0][1]);
    cc2500WriteReg(CC2500_25_FSCAL1, calData[0][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, 0);
    cc2500Strobe(CC2500_SFRX);
    delayMicroseconds(20); // waiting flush FIFO

    cc2500Strobe(CC2500_SRX);
    listLength = 0;
    bindIdx = 0x05;
}

static bool getBind1(uint8_t *packet)
{
    // len|bind |tx
    // id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
    // Start by getting bind packet 0 and the txid
    if (cc2500getGdo()) {
        uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
        if (ccLen > RX_SPI_MAX_PAYLOAD_SIZE) {
            cc2500Strobe(CC2500_SFRX);
        } else if (ccLen) {
            cc2500ReadFifo(packet, ccLen);
            if (packet[ccLen - 1] & 0x80) {
                if (packet[2] == 0x01) {
                    if (packet[5] == 0x00) {
                        rxCc2500SpiConfigMutable()->bindTxId[0] = packet[3];
                        rxCc2500SpiConfigMutable()->bindTxId[1] = packet[4];
                        for (uint8_t n = 0; n < 5; n++) {
                            rxCc2500SpiConfigMutable()->bindHopData[packet[5] + n] =
                                packet[6 + n];
                        }

                        rxCc2500SpiConfigMutable()->rxNum = packet[12];

                        return true;
                    }
                }
            }
        }
    }

    return false;
}

static bool getBind2(uint8_t *packet)
{
    if (bindIdx <= 120) {
        if (cc2500getGdo()) {
            uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            if (ccLen > RX_SPI_MAX_PAYLOAD_SIZE) {
                cc2500Strobe(CC2500_SFRX);
            } else if (ccLen) {
                cc2500ReadFifo(packet, ccLen);
                if (packet[ccLen - 1] & 0x80) {
                    if (packet[2] == 0x01) {
                        if ((packet[3] == rxCc2500SpiConfig()->bindTxId[0]) &&
                            (packet[4] == rxCc2500SpiConfig()->bindTxId[1])) {
                            if (packet[5] == bindIdx) {
#if defined(DJTS)
                                if (packet[5] == 0x2D) {
                                    for (uint8_t i = 0; i < 2; i++) {
                                        rxCc2500SpiConfigMutable()->bindHopData[packet[5] + i] = packet[6 + i];
                                    }
                                    listLength = 47;

                                    return true;
                                }
#endif

                                for (uint8_t n = 0; n < 5; n++) {
                                    if (packet[6 + n] == packet[ccLen - 3] || (packet[6 + n] == 0)) {
                                        if (bindIdx >= 0x2D) {
                                            listLength = packet[5] + n;

                                            return true;
                                        }
                                    }

                                    rxCc2500SpiConfigMutable()->bindHopData[packet[5] + n] = packet[6 + n];
                                }

                                bindIdx = bindIdx + 5;

                                return false;
                            }
                        }
                    }
                }
            }
        }

        return false;
    } else {
        return true;
    }
}

rx_spi_received_e frSkySpiDataReceived(uint8_t *packet)
{
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    switch (protocolState) {
    case STATE_INIT:
        if ((millis() - start_time) > 10) {
            initialise();

            protocolState = STATE_BIND;
        }

        break;
    case STATE_BIND:
        if (rxSpiCheckBindRequested(true) || rxCc2500SpiConfig()->autoBind) {
            rxSpiLedOn();
            initTuneRx();

            protocolState = STATE_BIND_TUNING;
        } else {
            protocolState = STATE_STARTING;
        }

        break;
    case STATE_BIND_TUNING:
       if (tuneRx(packet)) {
            initGetBind();
            initialiseData(1);

            protocolState = STATE_BIND_BINDING1;
        }

        break;
    case STATE_BIND_BINDING1:
        if (getBind1(packet)) {
            protocolState = STATE_BIND_BINDING2;
        }

        break;
    case STATE_BIND_BINDING2:
        if (getBind2(packet)) {
            cc2500Strobe(CC2500_SIDLE);

            protocolState = STATE_BIND_COMPLETE;
        }

        break;
    case STATE_BIND_COMPLETE:
        if (!rxCc2500SpiConfig()->autoBind) {
            writeEEPROM();
        } else {
            uint8_t ctr = 80;
            while (ctr--) {
                rxSpiLedToggle();
                delay(50);
            }
        }

        ret = RX_SPI_RECEIVED_BIND;
        protocolState = STATE_STARTING;

        break;
    default:
        ret = handlePacket(packet, &protocolState);

        break;
    }

    return ret;
}

void frSkySpiSetRcData(uint16_t *rcData, const uint8_t *payload)
{
    setRcData(rcData, payload);
}

void nextChannel(uint8_t skip)
{
    static uint8_t channr = 0;

    channr += skip;
    while (channr >= listLength) {
        channr -= listLength;
    }
    cc2500Strobe(CC2500_SIDLE);
    cc2500WriteReg(CC2500_23_FSCAL3,
                    calData[rxCc2500SpiConfig()->bindHopData[channr]][0]);
    cc2500WriteReg(CC2500_24_FSCAL2,
                    calData[rxCc2500SpiConfig()->bindHopData[channr]][1]);
    cc2500WriteReg(CC2500_25_FSCAL1,
                    calData[rxCc2500SpiConfig()->bindHopData[channr]][2]);
    cc2500WriteReg(CC2500_0A_CHANNR, rxCc2500SpiConfig()->bindHopData[channr]);
    if (spiProtocol == RX_SPI_FRSKY_D) {
        cc2500Strobe(CC2500_SFRX);
    }
}

bool frSkySpiInit(const rxSpiConfig_t *rxSpiConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxSpiCommonIOInit(rxSpiConfig);
    cc2500SpiInit();

    spiProtocol = rxSpiConfig->rx_spi_protocol;

    switch (spiProtocol) {
    #if defined(USE_RX_FRSKY_SPI_D)        
    case RX_SPI_FRSKY_D:
        rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT_FRSKY_D;

        handlePacket = frSkyDHandlePacket;
        setRcData = frSkyDSetRcData;
        frSkyDInit();

        break;
    #endif
    #if defined(USE_RX_FRSKY_SPI_X)        
    case RX_SPI_FRSKY_X:
    case RX_SPI_FRSKY_X_LBT:
        rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT_FRSKY_X;

        handlePacket = frSkyXHandlePacket;
        setRcData = frSkyXSetRcData;
        frSkyXInit(spiProtocol);

        break;
    #endif
    #if defined(USE_RX_REDPINE_SPI)
    case RX_SPI_REDPINE:
        rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT_REDPINE;

        handlePacket = redpineHandlePacket;
        setRcData = redpineSetRcData;
        redpineInit();

        break;  
    #endif      
    default:

        break;
    }

#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }
#endif

    missingPackets = 0;
    timeoutUs = 50;

    start_time = millis();
    protocolState = STATE_INIT;

    return true;
}
#endif
