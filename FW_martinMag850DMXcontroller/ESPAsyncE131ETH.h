/*
* ESPAsyncE131ETH.h
*
* Modified for Ethernet (ETH.h / W5500 on lwIP) instead of WiFi.
* Ring buffer removed: data is delivered through a callback only.
*
* Project: ESPAsyncE131 - Asynchronous E.131 (sACN) library for Arduino ESP8266 and ESP32
* Copyright (c) 2019 Shelby Merrick
* http://www.forkineye.com
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

#ifndef ESPASYNCE131ETH_H_
#define ESPASYNCE131ETH_H_

#ifndef ESP32
#error This version only supports ESP32
#endif

#include <Arduino.h>
#include <ETH.h>
#include <AsyncUDP.h>
#include <lwip/ip_addr.h>
#include <lwip/igmp.h>

// Defaults
#define E131_DEFAULT_PORT 5568

// E1.31 Packet Offsets
#define E131_ROOT_PREAMBLE_SIZE 0
#define E131_ROOT_POSTAMBLE_SIZE 2
#define E131_ROOT_ID 4
#define E131_ROOT_FLENGTH 16
#define E131_ROOT_VECTOR 18
#define E131_ROOT_CID 22

#define E131_FRAME_FLENGTH 38
#define E131_FRAME_VECTOR 40
#define E131_FRAME_SOURCE 44
#define E131_FRAME_PRIORITY 108
#define E131_FRAME_RESERVED 109
#define E131_FRAME_SEQ 111
#define E131_FRAME_OPT 112
#define E131_FRAME_UNIVERSE 113

#define E131_DMP_FLENGTH 115
#define E131_DMP_VECTOR 117
#define E131_DMP_TYPE 118
#define E131_DMP_ADDR_FIRST 119
#define E131_DMP_ADDR_INC 121
#define E131_DMP_COUNT 123
#define E131_DMP_DATA 125          // offset of property_values[0] (DMX start code)

// E1.31 Packet Structure
typedef union {
    struct {
        // Root Layer
        uint16_t preamble_size;
        uint16_t postamble_size;
        uint8_t  acn_id[12];
        uint16_t root_flength;
        uint32_t root_vector;
        uint8_t  cid[16];

        // Frame Layer
        uint16_t frame_flength;
        uint32_t frame_vector;
        uint8_t  source_name[64];
        uint8_t  priority;
        uint16_t reserved;
        uint8_t  sequence_number;
        uint8_t  options;
        uint16_t universe;

        // DMP Layer
        uint16_t dmp_flength;
        uint8_t  dmp_vector;
        uint8_t  type;
        uint16_t first_address;
        uint16_t address_increment;
        uint16_t property_value_count;
        uint8_t  property_values[513];   // [0] = start code, [1..512] = channels
    } __attribute__((packed));

    uint8_t raw[638];
} e131_packet_t;

// Error Types
typedef enum {
    ERROR_NONE,
    ERROR_IGNORE,
    ERROR_ACN_ID,
    ERROR_PACKET_SIZE,
    ERROR_VECTOR_ROOT,
    ERROR_VECTOR_FRAME,
    ERROR_VECTOR_DMP
} e131_error_t;

// E1.31 Listener Types
typedef enum {
    E131_UNICAST,
    E131_MULTICAST
} e131_listen_t;

// Status structure
typedef struct {
    uint32_t       num_packets;
    uint32_t       packet_errors;
    IPAddress      last_clientIP;
    uint16_t       last_clientPort;
    unsigned long  last_seen;
} e131_stats_t;

class ESPAsyncE131ETH {
 private:
    static const uint8_t  ACN_ID[12];
    static const uint32_t VECTOR_ROOT  = 4;
    static const uint32_t VECTOR_FRAME = 2;
    static const uint8_t  VECTOR_DMP   = 2;

    e131_packet_t *sbuff = nullptr;      // points to the packet currently being parsed
    AsyncUDP       udp;
    void          *UserInfo = nullptr;
    void         (*PacketCallback)(e131_packet_t *ReceivedData, void *UserInfo) = nullptr;
    uint16_t       E131_ListenPort = E131_DEFAULT_PORT;

#define NumPdusToIgnoreBeforeSwitchingSources 5
    struct LastAcceptedSource_t {
        uint8_t priority = 0;
        uint8_t PdusToIgnore = 0;
    } LastAcceptedSource;

    bool initUnicast();
    bool initMulticast(uint16_t universe, uint8_t n = 1);
    void parsePacket(AsyncUDPPacket &packet);

 public:
    e131_stats_t stats;
    ESPAsyncE131ETH();

    // Must be called once the Ethernet interface has its IP address.
    // NOTE: in MULTICAST mode the socket is bound to the port on any address,
    // so unicast sACN sent to the device IP is received as well.
    bool begin(e131_listen_t type, uint16_t universe = 1, uint8_t n = 1);
    bool begin(e131_listen_t type, uint16_t udpPort, uint16_t universe, uint8_t n);

    // The callback is called from the AsyncUDP task: keep it short, no blocking calls.
    void registerCallback(void *_UserInfo, void (*cbFunction)(e131_packet_t *, void *)) {
        PacketCallback = cbFunction;
        UserInfo = _UserInfo;
    }

    void dumpError(e131_error_t error);
};

#endif  // ESPASYNCE131ETH_H_
