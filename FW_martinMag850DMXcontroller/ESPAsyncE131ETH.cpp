/*
* ESPAsyncE131ETH.cpp
*
* Project: ESPAsyncE131 - Asynchronous E.131 (sACN) library for Arduino ESP8266 and ESP32
* Copyright (c) 2019 Shelby Merrick
* http://www.forkineye.com
*
* Modified for Ethernet (ETH.h / lwIP): ring buffer removed, callback only,
* packet length validation added.
*
*  This program is provided free for you to use in any way that you wish,
*  subject to the laws and regulations where you are using it.  Due diligence
*  is strongly suggested before using this code.  Please give credit where due.
*
*  The Author makes no warranty of any kind, express or implied, with regard
*  to this program or the documentation contained in this document.  The
*  Author shall not be liable in any event for incidental or consequential
*  damages in connection with, or arising out of, the furnishing, performance
*  or use of these programs.
*/

#include "ESPAsyncE131ETH.h"
#include <string.h>

// E1.17 ACN Packet Identifier
const uint8_t ESPAsyncE131ETH::ACN_ID[12] = { 0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00 };

ESPAsyncE131ETH::ESPAsyncE131ETH() {
    stats.num_packets = 0;
    stats.packet_errors = 0;
    stats.last_clientPort = 0;
    stats.last_seen = 0;
}

/////////////////////////////////////////////////////////
// Public begin() members
/////////////////////////////////////////////////////////

bool ESPAsyncE131ETH::begin(e131_listen_t type, uint16_t universe, uint8_t n)
{
    return begin(type, E131_ListenPort, universe, n);
}

bool ESPAsyncE131ETH::begin(e131_listen_t type, uint16_t udpPort, uint16_t universe, uint8_t n)
{
    bool success = false;
    E131_ListenPort = udpPort;

    if (type == E131_UNICAST)
        success = initUnicast();
    else if (type == E131_MULTICAST)
        success = initMulticast(universe, n);

    return success;
}

/////////////////////////////////////////////////////////
// Private init() members
/////////////////////////////////////////////////////////

bool ESPAsyncE131ETH::initUnicast() {
    if (udp.listen(E131_ListenPort)) {
        udp.onPacket([this](AsyncUDPPacket &packet) { parsePacket(packet); });
        return true;
    }
    return false;
}

bool ESPAsyncE131ETH::initMulticast(uint16_t universe, uint8_t n) {
    IPAddress address = IPAddress(239, 255, ((universe >> 8) & 0xff), ((universe >> 0) & 0xff));

    if (udp.listenMulticast(address, E131_ListenPort)) {
        // Extra universes (only used when n > 1, not tested)
        ip4_addr_t ifaddr;
        ip4_addr_t multicast_addr;
        ifaddr.addr = static_cast<uint32_t>(ETH.localIP());
        for (uint8_t i = 1; i < n; i++) {
            multicast_addr.addr = static_cast<uint32_t>(IPAddress(239, 255,
                    (((universe + i) >> 8) & 0xff), (((universe + i) >> 0) & 0xff)));
            igmp_joingroup(&ifaddr, &multicast_addr);
        }

        udp.onPacket([this](AsyncUDPPacket &packet) { parsePacket(packet); });
        return true;
    }
    return false;
}

/////////////////////////////////////////////////////////
// Packet parsing - Private
/////////////////////////////////////////////////////////

void ESPAsyncE131ETH::parsePacket(AsyncUDPPacket &packet)
{
    e131_error_t error = ERROR_NONE;
    const size_t len = packet.length();
    sbuff = reinterpret_cast<e131_packet_t *>(packet.data());

    do // once
    {
        // Must at least contain headers + start code
        if (len < (size_t)E131_DMP_DATA + 1)
        {
            error = ERROR_PACKET_SIZE;
            break;
        }

        if (memcmp(sbuff->acn_id, ESPAsyncE131ETH::ACN_ID, sizeof(sbuff->acn_id)))
        {
            error = ERROR_ACN_ID;
            break;
        }

        if (htonl(sbuff->root_vector) != ESPAsyncE131ETH::VECTOR_ROOT)
        {
            error = ERROR_VECTOR_ROOT;
            break;
        }

        if (htonl(sbuff->frame_vector) != ESPAsyncE131ETH::VECTOR_FRAME)
        {
            error = ERROR_VECTOR_FRAME;
            break;
        }

        if (sbuff->dmp_vector != ESPAsyncE131ETH::VECTOR_DMP)
        {
            error = ERROR_VECTOR_DMP;
            break;
        }

        // Only DMX start code 0 is dimmer data
        if (sbuff->property_values[0] != 0)
        {
            error = ERROR_IGNORE;
            break;
        }

        // property_value_count includes the start code
        const uint16_t count = ntohs(sbuff->property_value_count);
        if (count < 1 || count > 513 || ((size_t)E131_DMP_DATA + count) > len)
        {
            error = ERROR_PACKET_SIZE;
            break;
        }

        // Are we looking for a new source?
        if ((0 != LastAcceptedSource.PdusToIgnore) &&
            (sbuff->priority < LastAcceptedSource.priority))
        {
            // Lower priority source: ignore until the higher priority one is silent for a while
            LastAcceptedSource.PdusToIgnore--;
            error = ERROR_IGNORE;
            break;
        }

        LastAcceptedSource.PdusToIgnore = NumPdusToIgnoreBeforeSwitchingSources;
        LastAcceptedSource.priority = sbuff->priority;

        if (PacketCallback) { (*PacketCallback)(sbuff, UserInfo); }

        stats.num_packets++;
        stats.last_clientIP = packet.remoteIP();
        stats.last_clientPort = packet.remotePort();
        stats.last_seen = millis();

    } while (false);

    if (error != ERROR_IGNORE && error != ERROR_NONE)
    {
        dumpError(error);
        stats.packet_errors++;
    }
}

/////////////////////////////////////////////////////////
// Debugging functions - Public
/////////////////////////////////////////////////////////

void ESPAsyncE131ETH::dumpError(e131_error_t error) {
    switch (error) {
        case ERROR_ACN_ID:
            Serial.print(F("sACN INVALID PACKET ID: "));
            for (size_t i = 0; i < sizeof(ACN_ID); i++)
                Serial.print(sbuff->acn_id[i], HEX);
            Serial.println("");
            break;
        case ERROR_PACKET_SIZE:
            Serial.println(F("sACN INVALID PACKET SIZE"));
            break;
        case ERROR_VECTOR_ROOT:
            Serial.print(F("sACN INVALID ROOT VECTOR: 0x"));
            Serial.println(htonl(sbuff->root_vector), HEX);
            break;
        case ERROR_VECTOR_FRAME:
            Serial.print(F("sACN INVALID FRAME VECTOR: 0x"));
            Serial.println(htonl(sbuff->frame_vector), HEX);
            break;
        case ERROR_VECTOR_DMP:
            Serial.print(F("sACN INVALID DMP VECTOR: 0x"));
            Serial.println(sbuff->dmp_vector, HEX);
            break;
        case ERROR_NONE:
        case ERROR_IGNORE:
            break;
    }
}
