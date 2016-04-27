#ifndef DATA_EXCHANGE_H
#define DATA_EXCHANGE_H

#include <bitset>
#include <atomic>

namespace VisionForceExhange
{
    extern float nextPositionGCS[2];
    extern std::atomic_bool isWriteData;
}

#endif
