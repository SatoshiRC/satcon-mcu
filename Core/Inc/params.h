#include <array>
#include <cmath>

#include "TWO_DOF_PID.h"
#include "SBUS_Handller.h"
#include "Quaternion/Quaternion.h"


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

Quaternion<float> imuFrame(1/std::sqrt(2),0,0,1/std::sqrt(2));
