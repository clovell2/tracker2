#pragma once

template <typename T>
class HardwareInterface{
public:
    HardwareInterface(){};
    virtual void init_interface() = 0;
    virtual T read_buffer() = 0;
    virtual void write_buffer() = 0;
    virtual void clear_buffer() = 0;
};
