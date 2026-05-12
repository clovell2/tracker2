#pragma once
#include "adsb.hpp"
#include "hardware_interface.hpp"

class ADSBInterface : HardwareInterface<ADSB> {
    public:
        ADSBInterface();
        void init_interface() override;
        ADSB read_buffer() override;
        void write_buffer() override;
        void clear_buffer() override;
};