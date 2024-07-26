#include <array>

#include "TWO_DOF_PID.h"
#include "SBUS_Handller.h"


auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,1,0.1,0.05,0.2,-0.2);
auto *yawRateParam = new TWO_DOF_PID_PARAM<float>(0,1,0.1,0.05,0.2,-0.2);
auto *altitudeParam = new TWO_DOF_PID_PARAM<float>(0,1,0.1,0.05,0.2,-0.2);

std::array<uint16_t, 18> center = {
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

std::array<uint16_t, 18> upper = {
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

std::array<uint16_t, 18> lower = {
    1000, 1000, 1000, 1000,
    1000, 1000, 1000, 1000,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};
