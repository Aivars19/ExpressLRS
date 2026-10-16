#include "targets.h"
#include "common.h"
#include "device.h"

#include "config.h"
#include "CRSF.h"
#include "msp.h"
#include "logging.h"

#include "devButton.h"
#include "handset.h"

#define PITMODE_NOT_INITIALISED    -1
#define PITMODE_OFF                 0
#define PITMODE_ON                  1

// Period after a disconnect to wait in the VTXSS_DEBUOUNCING state so if
// we get a connection we don't resend the VTX commands.
// Needs to be long enough to reconnect, but short enough to
// reset between the user switching equipment. This is so we don't get into
// a loop of connect -> send -> write eeprom -> disconnect -> ...
// See https://github.com/ExpressLRS/ExpressLRS/issues/2976
#define VTX_DISCONNECT_DEBOUNCE_MS (10 * 1000)

extern Stream *TxBackpack;
static int pitmodeAuxState = PITMODE_NOT_INITIALISED;
static int vtxChannelAuxState = PITMODE_NOT_INITIALISED;
static bool vtxChannelAuxActive = false;
static int vtxPowerAuxState = PITMODE_NOT_INITIALISED;
static bool vtxPowerAuxActive = false;
static bool sendEepromWrite = true;

static enum VtxSendState_e
{
  VTXSS_UNKNOWN,   // Status of the remote side is unknown, so we should send immediately if connected
  VTXSS_MODIFIED,  // Config is edited, should always be sent regardless of connect state
  VTXSS_SENDING1, VTXSS_SENDING2, VTXSS_SENDING3,  VTXSS_SENDINGDONE, // Send the config 3x
  VTXSS_CONFIRMED, // Status of remote side is consistent with our config
  VTXSS_DEBOUNCING // After a disconnect, go to debouncing state so a reconnect during the debounce period won't re-send the VTX command and eeprom write and get another disconnect
} VtxSendState;

void VtxTriggerSend()
{
    VtxSendState = VTXSS_MODIFIED;
    sendEepromWrite = true;
    devicesTriggerEvent();
}

void VtxPitmodeSwitchUpdate()
{
    uint8_t powerAux = config.GetVtxPowerAux();
    if (powerAux > 12)
        powerAux = 12;

    if (powerAux == 0)
    {
        if (vtxPowerAuxActive)
        {
            vtxPowerAuxActive = false;
            vtxPowerAuxState = PITMODE_NOT_INITIALISED;
            VtxTriggerSend();
            sendEepromWrite = false;
        }
    }
    else
    {
        uint32_t powerValue = ChannelData[powerAux + 3];
        if (powerValue < CRSF_CHANNEL_VALUE_MIN)
            powerValue = CRSF_CHANNEL_VALUE_MIN;
        if (powerValue > CRSF_CHANNEL_VALUE_MAX)
            powerValue = CRSF_CHANNEL_VALUE_MAX;
        uint8_t newVtxPower = 1 + ((powerValue - CRSF_CHANNEL_VALUE_MIN) * 8) /
                              (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1);

        if (!vtxPowerAuxActive)
        {
            vtxPowerAuxActive = true;
            vtxPowerAuxState = newVtxPower;
            if (newVtxPower != config.GetVtxPower())
            {
                VtxTriggerSend();
                sendEepromWrite = false;
            }
        }
        else if (vtxPowerAuxState != newVtxPower)
        {
            vtxPowerAuxState = newVtxPower;
            VtxTriggerSend();
            sendEepromWrite = false;
        }
    }

    uint8_t channelAux = config.GetVtxChanAux();
    if (channelAux > 12)
        channelAux = 12;

    if (channelAux == 0)
    {
        if (vtxChannelAuxActive)
        {
            vtxChannelAuxActive = false;
            vtxChannelAuxState = PITMODE_NOT_INITIALISED;
            VtxTriggerSend();
            sendEepromWrite = false;
        }
    }
    else
    {
        uint32_t channelValue = ChannelData[channelAux + 3];
        if (channelValue < CRSF_CHANNEL_VALUE_MIN)
            channelValue = CRSF_CHANNEL_VALUE_MIN;
        if (channelValue > CRSF_CHANNEL_VALUE_MAX)
            channelValue = CRSF_CHANNEL_VALUE_MAX;
        uint8_t newVtxChannel = ((channelValue - CRSF_CHANNEL_VALUE_MIN) * 8) /
                                (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN + 1);

        if (!vtxChannelAuxActive)
        {
            vtxChannelAuxActive = true;
            vtxChannelAuxState = newVtxChannel;
            if (newVtxChannel != config.GetVtxChannel())
            {
                VtxTriggerSend();
                sendEepromWrite = false;
            }
        }
        else if (vtxChannelAuxState != newVtxChannel)
        {
            vtxChannelAuxState = newVtxChannel;
            VtxTriggerSend();
            sendEepromWrite = false;
        }
    }

    if (config.GetVtxPitmode() <= PITMODE_ON)
    {
        pitmodeAuxState = config.GetVtxPitmode();
        return;
    }

    uint8_t auxInverted = config.GetVtxPitmode() % 2;
    uint8_t auxNumber = (config.GetVtxPitmode() / 2) + 3;
    uint8_t newPitmodeAuxState = CRSF_to_BIT(ChannelData[auxNumber]) ^ auxInverted;

    if (pitmodeAuxState == PITMODE_NOT_INITIALISED)
    {
        pitmodeAuxState = newPitmodeAuxState;
    }
    else if (pitmodeAuxState != newPitmodeAuxState)
    {
        pitmodeAuxState = newPitmodeAuxState;
        VtxTriggerSend();
        // No forced EEPROM saving of Pit Mode
        sendEepromWrite = false;
    }
}

uint8_t VtxGetCurrentChannel()
{
    return vtxChannelAuxActive ? vtxChannelAuxState : config.GetVtxChannel();
}

uint8_t VtxGetCurrentPower()
{
    return vtxPowerAuxActive ? vtxPowerAuxState : config.GetVtxPower();
}

static void eepromWriteToMSPOut()
{
    mspPacket_t packet;
    packet.reset();
    packet.function = MSP_EEPROM_WRITE;

    CRSF::AddMspMessage(&packet, CRSF_ADDRESS_FLIGHT_CONTROLLER);
}

static void VtxConfigToMSPOut()
{
    DBGLN("Sending VtxConfig");
    uint8_t vtxChannel = VtxGetCurrentChannel();
    uint8_t vtxIdx = (config.GetVtxBand()-1) * 8 + vtxChannel;

    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.function = MSP_SET_VTX_CONFIG;
    packet.addByte(vtxIdx);     // band/channel or frequency low byte
    packet.addByte(0);          // frequency high byte, if frequency mode
    uint8_t vtxPower = VtxGetCurrentPower();
    if (vtxPower)
    {
        packet.addByte(vtxPower);
        packet.addByte(pitmodeAuxState);
    }

    CRSF::AddMspMessage(&packet, CRSF_ADDRESS_FLIGHT_CONTROLLER);

    if (!handset->IsArmed()) // Do not send while armed.  There is no need to change the video frequency while armed.  It can also cause VRx modules to flash up their OSD menu e.g. Rapidfire.
    {
        MSP::sendPacket(&packet, TxBackpack); // send to tx-backpack as MSP
    }
}

static void initialize()
{
    registerButtonFunction(ACTION_SEND_VTX, VtxTriggerSend);
}

static int event()
{
    // 0 = VTX Admin disabled in the configuration
    if (config.GetVtxBand() == 0)
    {
        VtxSendState = VTXSS_CONFIRMED;
        return DURATION_NEVER;
    }

    if (VtxSendState == VTXSS_MODIFIED ||
        (VtxSendState == VTXSS_UNKNOWN && connectionState == connected))
    {
        VtxSendState = VTXSS_SENDING1;
        return 1000;
    }

    if (connectionState == disconnected && VtxSendState != VTXSS_DEBOUNCING && VtxSendState != VTXSS_UNKNOWN)
    {
        VtxSendState = VTXSS_DEBOUNCING;
        return VTX_DISCONNECT_DEBOUNCE_MS;
    }

    return DURATION_IGNORE;
}

static int timeout()
{
    if (VtxSendState == VTXSS_DEBOUNCING)
    {
        if (connectionState == disconnected)
        {
            // Still disconnected after the debounce, assume this is a new connection on next connect
            VtxSendState = VTXSS_UNKNOWN;
            sendEepromWrite = true;
        }
        else
        {
            // Reconnect within the debounce, move back to CONFIRMED
            VtxSendState = VTXSS_CONFIRMED;
        }
    }

    if (VtxSendState == VTXSS_UNKNOWN || VtxSendState == VTXSS_MODIFIED || VtxSendState == VTXSS_CONFIRMED)
    {
        return DURATION_NEVER;
    }

    VtxConfigToMSPOut();

    VtxSendState = (VtxSendState_e)((int)VtxSendState + 1);
    if (VtxSendState < VTXSS_SENDINGDONE)
    {
        return 500; // repeat send in 500ms
    }

    if (connectionState == connected)
    {
        // Connected while sending, assume the MSP got to the RX
        VtxSendState = VTXSS_CONFIRMED;
        if (sendEepromWrite)
        {
            eepromWriteToMSPOut();
        }
        sendEepromWrite = true;
    }
    else
    {
        CRSF::ResetMspQueue();
        VtxSendState = VTXSS_UNKNOWN;
    }

    return DURATION_NEVER;
}

device_t VTX_device = {
    .initialize = initialize,
    .start = nullptr,
    .event = event,
    .timeout = timeout
};
