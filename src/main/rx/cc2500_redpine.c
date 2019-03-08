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

#include <string.h>
#include <sys/_stdint.h>

#include "platform.h"

#ifdef USE_RX_REDPINE_SPI

#include "build/build_config.h"
#include "build/debug.h"

#include "pg/rx.h"
#include "pg/rx_spi.h"

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/rx/rx_spi.h"
#include "drivers/rx/rx_cc2500.h"
#include "drivers/io.h"
#include "drivers/io_def.h"
#include "drivers/io_types.h"
#include "drivers/resource.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "fc/config.h"

#include "rx/rx_spi_common.h"
#include "rx/cc2500_common.h"
#include "rx/cc2500_frsky_common.h"
#include "rx/cc2500_frsky_shared.h"

#include "cc2500_redpine.h"

extern const uint16_t crcTable[];


#define TOTAL_PACKET_TIME 1500
#define CHANNEL_START 3
void redpineSetRcData(uint16_t *rcData, const uint8_t *packet)
{
    uint16_t channelValue;

    //4 stick channels (11-bit)
    channelValue = (uint16_t)((packet[CHANNEL_START+1] << 8) & 0x700) | packet[CHANNEL_START];
    rcData[0] = channelValue * 2.0f / 3 + 860 - 64 * 2 / 3;
    
    channelValue = (uint16_t)((packet[CHANNEL_START+2] << 4) & 0x7F0) | ((packet[CHANNEL_START+1] >> 4) & 0xF);
    rcData[1] = channelValue * 2.0f / 3 + 860 - 64 * 2 / 3;
    
    channelValue = (uint16_t)((packet[CHANNEL_START+4] << 8) & 0x700) | packet[CHANNEL_START+3];
    rcData[2] = channelValue * 2.0f / 3 + 860 - 64 * 2 / 3;
    
    channelValue = (uint16_t)((packet[CHANNEL_START+5] << 4) & 0x7F0) | ((packet[CHANNEL_START+4] >> 4) & 0xF);
    rcData[3] = channelValue * 2.0f / 3 + 860 - 64 * 2 / 3;    
    //12 1-bit aux channels - first 4 are interleaved with stick channels
    rcData[4]= (packet[CHANNEL_START + 1] & 0x08) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[5]= (packet[CHANNEL_START + 2] & 0x80) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[6]= (packet[CHANNEL_START + 4] & 0x08) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[7]= (packet[CHANNEL_START + 5] & 0x80) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[8]= (packet[CHANNEL_START + 6] & 0x01) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[9]= (packet[CHANNEL_START + 6] & 0x02) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[10]= (packet[CHANNEL_START + 6] & 0x04) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[11]= (packet[CHANNEL_START + 6] & 0x08) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[12]= (packet[CHANNEL_START + 6] & 0x10) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[13]= (packet[CHANNEL_START + 6] & 0x20) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[14]= (packet[CHANNEL_START + 6] & 0x40) ? PWM_RANGE_MAX : PWM_RANGE_MIN; 
    rcData[15]= (packet[CHANNEL_START + 6] & 0x80) ? PWM_RANGE_MAX : PWM_RANGE_MIN;         
}

rx_spi_received_e redpineHandlePacket(uint8_t * const packet, uint8_t * const protocolState)
{
    static bool ledIsOn;
    static uint16_t looptime = TOTAL_PACKET_TIME;
    static timeUs_t packetTimerUs;

    static timeUs_t totalTimerUs;
 
    static uint8_t channelsToSkip = 1;
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    switch (*protocolState) {
    case STATE_STARTING:
        listLength = 47;
        initialiseData(0);
        *protocolState = STATE_UPDATE;
        nextChannel(1);
        cc2500Strobe(CC2500_SRX);
#ifdef USE_RX_FRSKY_SPI_PA_LNA
        cc2500TxDisable();
#endif // USE_RX_FRSKY_SPI_PA_LNA

        break;
    case STATE_UPDATE:
        packetTimerUs = 0;
        totalTimerUs = micros();
        *protocolState = STATE_DATA;
        if (rxSpiCheckBindRequested(false)) {
            packetTimerUs = 0;
            missingPackets = 0;
            *protocolState = STATE_INIT;
            break;
        }

        FALLTHROUGH;
        // here FS code could be
    case STATE_DATA:
        if (cc2500getGdo()) {
            uint8_t ccLen = cc2500ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
            DEBUG_SET(DEBUG_RX_FRSKY_SPI, 3, ccLen);
            if (ccLen > RX_SPI_MAX_PAYLOAD_SIZE) {
                 cc2500Strobe(CC2500_SFRX);
            } else if (ccLen) {
                cc2500ReadFifo(packet, ccLen);
                uint16_t lcrc= calculateCrc(&packet[0], CHANNEL_START+9);

                if(((lcrc >> 8) == packet[CHANNEL_START+9]) && 
                    ((lcrc&0x00FF) == packet[CHANNEL_START+10]) &&
                    (packet[1] == rxFrSkySpiConfig()->bindTxId[0]) &&
                    (packet[2] == rxFrSkySpiConfig()->bindTxId[1])) {
                    
                    looptime = packet[CHANNEL_START+7] * 100;

                    DEBUG_SET(DEBUG_RX_FRSKY_SPI, 0, looptime);
                    DEBUG_SET(DEBUG_RX_FRSKY_SPI, 1, packet[CHANNEL_START+8] * 10);
                    packetTimerUs = micros();
                    totalTimerUs = micros();                         

                    missingPackets = 0;
                    rxSpiLedOn();

                    cc2500setRssiDbm(packet[ccLen - 2]);

                    ret = RX_SPI_RECEIVED_DATA;
                    nextChannel(channelsToSkip);
                    cc2500Strobe(CC2500_SRX);
                } 
            }
        }

        if (cmpTimeUs(micros(), totalTimerUs) > 50 * looptime) {
            //out of sync with packets - do a complete resysnc
            if (ledIsOn) {
                rxSpiLedOff();
            } else {
                rxSpiLedOn();
            }
            ledIsOn = !ledIsOn;
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL);
            nextChannel(1);
            cc2500Strobe(CC2500_SRX);
            *protocolState = STATE_UPDATE;
        } else if ((cmpTimeUs(micros(), packetTimerUs) > looptime) && packetTimerUs) {
            //missed a packet
            packetTimerUs = micros();
            nextChannel(channelsToSkip);
            cc2500Strobe(CC2500_SRX);
            missingPackets++;
            DEBUG_SET(DEBUG_RX_FRSKY_SPI, 2, missingPackets);
#if defined(USE_RX_FRSKY_SPI_DIVERSITY)
            if (missingPackets >= 2) {
                if (missingPackets >= 2) {
                    cc2500switchAntennae();
                }
            }            
#endif            
        }
        break;
    }
    return ret;
}

void redpineInit(void)
{

}

#endif
