#include "rx_spi.h"
#include "cc2500.h"
#include "time.h"
#include "flash.h"
#include "defines.h"
#include "util.h"

rx_spi_protocol_e spiProtocol;

uint8_t protocolState;
uint32_t start_time;
uint8_t packetLength;
uint16_t telemetryDelayUs;
uint32_t missingPackets;
int32_t timeoutUs;

uint8_t packet[35];
uint16_t rcData[18];
int rxmode;


uint16_t rcRaw[18];


handlePacketFn *handlePacket;
processFrameFn *processFrame;
setRcDataFn *setRcData;

extern rx_spi_received_e frSkySpiDataReceived(uint8_t *packet);
extern void frSkyXSetRcData(uint16_t *rcData, const uint8_t *packet);
extern void frSkyDSetRcData(uint16_t *rcData, const uint8_t *packet);
extern rx_spi_received_e frSkyDHandlePacket(uint8_t *const packet, uint8_t *const protocolState);
extern rx_spi_received_e frSkyXHandlePacket(uint8_t *const packet, uint8_t *const protocolState);


bool rxSpiGetExtiState(void)
{
    if ((GPIOB->IDR & GPIO_Pin_2) != (uint32_t)Bit_RESET)
    {
        return true;
    }
    else
    {
        return false;
    }
}


bool rxSpiCheckBindRequested(void)
{
    if ((GPIOB->IDR & GPIO_Pin_8) != (uint32_t)Bit_RESET)
    {
        return true;
    }
    else
    {
        return false;
    }

}
int8_t bindOffset, bindOffset_min, bindOffset_max;

rx_spi_received_e frSkySpiDataReceived(uint8_t *packet)
{
    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;
    switch (protocolState)
    {
    case STATE_INIT:
        if ((millis() - start_time) > 10)
        {
            cc2500_init();
            protocolState = STATE_BIND;
        }
        break;

    case STATE_BIND:
        if (rxCc2500SpiConfigMutable.autoBind)
        {
            initTuneRx();

            protocolState = STATE_BIND_TUNING_LOW;
        }
        else
        {
            protocolState = STATE_STARTING;
        }

        break;

    case STATE_BIND_TUNING_LOW:
        if (tuneRx(packet, 2))
        {
            bindOffset_min = bindOffset;
            bindOffset = 126;

            protocolState = STATE_BIND_TUNING_HIGH;
        }

        break;
    case STATE_BIND_TUNING_HIGH:
        if (tuneRx(packet, -2))
        {
            bindOffset_max = bindOffset;
            bindOffset = ((int16_t)bindOffset_max + (int16_t)bindOffset_min) / 2;
            rxCc2500SpiConfigMutable.bindOffset = bindOffset;
            initGetBind();
            initialiseData(true);

            if (bindOffset_min < bindOffset_max)
                protocolState = STATE_BIND_BINDING;
            else
                protocolState = STATE_BIND;
        }

        break;
    case STATE_BIND_BINDING:
        if (getBind(packet))
        {
            cc2500Strobe(CC2500_SIDLE);

            rxCc2500SpiConfigMutable.autoBind = false;
            protocolState = STATE_BIND_COMPLETE;
        }

        break;
    case STATE_BIND_COMPLETE:
        if (!rxCc2500SpiConfigMutable.autoBind)
        {
            flash_save();
        }
        else
        {
            uint8_t ctr = 80;
            while (ctr--)
            {
                delay_ms(50);
            }
        }
        rxmode = !RXMODE_BIND;
        ret = RX_SPI_RECEIVED_BIND;
        protocolState = STATE_STARTING;
        break;
    default:
        ret = handlePacket(packet, &protocolState);
        break;
    }
    return ret;
}


void frSkyDInit(const rx_spi_protocol_e Protocol)
{
    packetLength = 20;

}

bool frSkySpiInit(rx_spi_protocol_e Protocol)
{
    spiProtocol = Protocol;

    handlePacket = frSkyDHandlePacket;
    setRcData = frSkyDSetRcData;
    frSkyDInit(Protocol);


#if defined(USE_RX_FRSKY_SPI_TELEMETRY)
    if (rssiSource == RSSI_SOURCE_NONE)
    {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL;
    }
#endif

    missingPackets = 0;
    timeoutUs = 50;

    start_time = millis();
    protocolState = STATE_INIT;

    return true;
}

uint16_t rssi = 0;
int16_t rssiDbm;
#define RSSI_SAMPLE_COUNT 16
uint16_t updateRssiSamples(uint16_t value)
{
    static uint16_t samples[RSSI_SAMPLE_COUNT];
    static uint8_t sampleIndex = 0;
    static unsigned sum = 0;

    sum += value - samples[sampleIndex];
    samples[sampleIndex] = value;
    sampleIndex = (sampleIndex + 1) % RSSI_SAMPLE_COUNT;
    return sum / RSSI_SAMPLE_COUNT;
}

void setRssi(uint16_t rssiValue)
{
    rssi = updateRssiSamples(rssiValue);
}

void cc2500setRssiDbm(uint8_t value)
{
    if (value >= 128)
    {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) - 82;
    }
    else
    {
        rssiDbm = ((((uint16_t)value) * 18) >> 5) + 65;
    }

    setRssi(rssiDbm << 3);
}
#define RSSI_MAX_VALUE 1023
uint8_t rssip = 0;

uint8_t getRssiPercent(void)
{
    rssip = scaleRange(rssi, 0, RSSI_MAX_VALUE, 0, 100);
    return rssip;
}

uint32_t lasttime, looptime1;
uint8_t frskyUpdate(void)
{
    uint8_t status = RX_FRAME_PENDING;

    rx_spi_received_e result = frSkySpiDataReceived(packet);

    if (result & RX_SPI_RECEIVED_DATA)
    {
         setRcData(rcRaw, packet);
        
        rcData[0] = rcRaw[0];
        rcData[1] = rcRaw[1];
        rcData[2] = rcRaw[2];
        rcData[3] = rcRaw[3];
        rcData[4] = rcRaw[4];
        rcData[5] = rcRaw[5];
        rcData[6] = rcRaw[6];
        rcData[7] = rcRaw[7];
        
            // AETR channel order
      state.rx.axis[0] = (rcData[0] - 1500);
      state.rx.axis[1] = (rcData[1] - 1500);
      state.rx.axis[2] = -(rcData[3] - 1500);
      state.rx.axis[3] = rcData[2] - 1000;

      for (int i = 0; i < 3; i++) {
        state.rx.axis[i] *= 0.002f;
      }
      state.rx.axis[3] *= 0.001f;

      if (state.rx.axis[3] > 1)
        state.rx.axis[3] = 1;
      if (state.rx.axis[3] < 0)
        state.rx.axis[3] = 0;

      rx_apply_expo();

      //Here we have the AUX channels Silverware supports
      state.aux[AUX_CHANNEL_0] = (rcData[4] > 1200) ? ((rcData[4] < 1800) ? 1 : 2) : 0;
      state.aux[AUX_CHANNEL_1] = (rcData[5] > 1200) ? ((rcData[5] < 1800) ? 1 : 2) : 0;
      state.aux[AUX_CHANNEL_2] = (rcData[6] > 1200) ? ((rcData[6] < 1800) ? 1 : 2) : 0;
      state.aux[AUX_CHANNEL_3] = (rcData[7] > 1200) ? ((rcData[7] < 1800) ? 1 : 2) : 0;

      
      state.rx_rssi = getRssiPercent();
      
      flags.rx_ready = 1;
      flags.failsafe = 0;
  
        lasttime = gettime();
    }
    else{
        if (gettime() - lasttime > 1000000)
        {
            flags.rx_ready = 0;
            flags.failsafe = 1;
            
            state.rx.axis[0] = 0;
            state.rx.axis[1] = 0;
            state.rx.axis[2] = 0;
            state.rx.axis[3] = 0;
        }
    }
    return status;
}

