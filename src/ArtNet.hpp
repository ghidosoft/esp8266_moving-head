#ifndef SRC_ARTNET_HPP_
#define SRC_ARTNET_HPP_

#include <WiFiUdp.h>

class ArtNet final
{
    public:
        void begin();
        void loop();
    private:
        WiFiUDP _udp;
};

#endif
