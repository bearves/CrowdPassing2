#include "DataExchange.h"

namespace VisionForceExhange
{
    float nextPositionGCS[2] = {-200, 0};
    std::atomic_bool isWriteData(false);
}
