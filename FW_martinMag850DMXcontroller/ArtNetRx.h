// ArtNetRx.h
// Minimal Art-Net receiver (ArtDmx + ArtPoll/ArtPollReply) for ESP32 + ETH.h (lwIP).
// Works with broadcast and unicast Art-Net on UDP 6454.

#ifndef ARTNETRX_H_
#define ARTNETRX_H_

#include <Arduino.h>
#include <ETH.h>
#include <AsyncUDP.h>

#define ARTNET_PORT 6454

class ArtNetRx {
 public:
  // portAddress = 15 bit Art-Net universe (Net << 8 | SubNet << 4 | Universe)
  // Called from the AsyncUDP task: keep it short.
  typedef void (*DmxCallback)(uint16_t portAddress, const uint8_t *data, uint16_t length);

  volatile bool dataActive = false;   // set by the application, reported in ArtPollReply
  uint32_t packets = 0;

  bool begin(DmxCallback cb, const char *shortName, const char *longName) {
    _cb = cb;
    strlcpy(_shortName, shortName, sizeof(_shortName));
    strlcpy(_longName, longName, sizeof(_longName));
    if (!_udp.listen(ARTNET_PORT)) return false;
    _udp.onPacket([this](AsyncUDPPacket &packet) { handlePacket(packet); });
    return true;
  }

  // Port-address advertised in ArtPollReply (the filtering itself is done in the callback)
  void setPortAddress(uint16_t pa) { _portAddress = pa; }

 private:
  AsyncUDP _udp;
  DmxCallback _cb = nullptr;
  uint16_t _portAddress = 0;
  char _shortName[18] = "";
  char _longName[64] = "";

  void handlePacket(AsyncUDPPacket &packet) {
    const uint8_t *d = packet.data();
    const size_t len = packet.length();

    if (len < 12 || memcmp(d, "Art-Net\0", 8) != 0) return;
    const uint16_t opCode = d[8] | (d[9] << 8);   // little endian

    if (opCode == 0x5000) {                        // ArtDmx
      if (len < 18) return;
      const uint16_t dlen = (d[16] << 8) | d[17];  // big endian
      if (dlen > 512 || len < (size_t)(18 + dlen)) return;
      const uint16_t portAddress = ((d[15] & 0x7F) << 8) | d[14];
      packets++;
      if (_cb) _cb(portAddress, d + 18, dlen);
    }
    else if (opCode == 0x2000) {                   // ArtPoll
      sendPollReply(packet);
    }
  }

  void sendPollReply(AsyncUDPPacket &packet) {
    uint8_t b[239];
    memset(b, 0, sizeof(b));

    const IPAddress ip = ETH.localIP();
    uint8_t mac[6] = {0};
    ETH.macAddress(mac);

    memcpy(b, "Art-Net\0", 8);
    b[8] = 0x00; b[9] = 0x21;                      // OpPollReply
    for (int i = 0; i < 4; i++) b[10 + i] = ip[i];
    b[14] = ARTNET_PORT & 0xFF; b[15] = ARTNET_PORT >> 8;   // port, little endian
    b[16] = 0; b[17] = 1;                          // firmware version
    b[18] = (_portAddress >> 8) & 0x7F;            // NetSwitch
    b[19] = (_portAddress >> 4) & 0x0F;            // SubSwitch
    b[20] = 0x00; b[21] = 0xFF;                    // OEM (unspecified)
    b[23] = 0xD0;                                  // Status1: indicators normal, front-panel programming
    b[24] = 0xF0; b[25] = 0x7F;                    // ESTA code 0x7FF0 (prototyping), little endian
    strlcpy((char *)&b[26], _shortName, 18);
    strlcpy((char *)&b[44], _longName, 64);
    snprintf((char *)&b[108], 64, "#0001 [%04u] OK", (unsigned)(packets % 10000));
    b[172] = 0; b[173] = 1;                        // 1 port
    b[174] = 0x80;                                 // port 1: DMX512 output (Art-Net -> DMX)
    b[182] = dataActive ? 0x80 : 0x00;             // GoodOutput: data being transmitted
    b[190] = _portAddress & 0x0F;                  // SwOut
    b[200] = 0x00;                                 // StNode
    memcpy(&b[201], mac, 6);
    for (int i = 0; i < 4; i++) b[207 + i] = ip[i];
    b[211] = 1;                                    // BindIndex
    b[212] = 0x0A;                                 // Status2: 15-bit port-address + web config

    _udp.writeTo(b, sizeof(b), packet.remoteIP(), ARTNET_PORT);
  }
};

#endif  // ARTNETRX_H_
