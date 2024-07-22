#include <array>

#include "TWO_DOF_PID.h"
#include "SBUS_Handller.h"


TWO_DOF_PID_PARAM *rollParam = new TWO_DOF_PID_PARAM(0,1,0.1,0.05,0.2,-0.2);
TWO_DOF_PID_PARAM *yawRateParam = new TWO_DOF_PID_PARAM(0,1,0.1,0.05,0.2,-0.2);
TWO_DOF_PID_PARAM *altitudeParam = new TWO_DOF_PID_PARAM(0,1,0.1,0.05,0.2,-0.2);

nokolat::SBUS_DATA center({
    1500, 1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500, 1500,
    1500, 0, 0
});

std::array<uint16_t, 18> upper = {
    1800, 1800, 1800, 1800, 1800,
    1800, 1800, 1800, 1800, 1800,
    1500, 1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500, 1500,
    1500, 1, 1
};

std::array<uint16_t, 18> lower = {
    1000, 1000, 1000, 1000, 1000,
    1000, 1000, 1000, 1000, 1000,
    1500, 1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500, 1500,
    1500, 0, 0
};