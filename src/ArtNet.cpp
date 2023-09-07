#include "ArtNet.hpp"

#include <cstring>
#include <Arduino.h>

#define ARTNET_PORT 6454
#define ARTNET_BUFFER_SIZE (512+18)

static int parseOpcode(uint8_t* buffer)
{
    if (std::memcmp("Art-Net", buffer, 8) == 0)
    {
        if (buffer[11] >= 14)
        {
            return (buffer[9] << 8) | buffer[8];
        }
    }
    return 0;
}

void ArtNet::begin()
{
    _udp.begin(ARTNET_PORT);
    _udp.flush();
}

void ArtNet::loop()
{
    const auto packetSize = _udp.parsePacket();
    if (packetSize > 0)
    {
        if (packetSize > ARTNET_BUFFER_SIZE || packetSize < 18)
        {
            _udp.flush();
            return;
        }
    
        uint8_t buffer[ARTNET_BUFFER_SIZE];
        _udp.read(buffer, packetSize);

        const auto opcode = parseOpcode(buffer);

        if (opcode != 0)
        {
            Serial.print("Art-Net packet opcode=");
            Serial.println(opcode);
        }
    }
}
