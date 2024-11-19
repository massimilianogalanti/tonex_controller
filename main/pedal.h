#pragma once

#include <cstdint>

class Tonex;
namespace pedal
{
    void pedal_receiver(void *arg);
    void init(Tonex *tonex);
}