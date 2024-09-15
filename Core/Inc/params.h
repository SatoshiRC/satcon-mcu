#include <array>
#include <cmath>

#include "TWO_DOF_PID.h"
#include "SBUS_Handller.h"
#include "Quaternion/Quaternion.h"


auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.009 ,0.0028,0.000018,0.1,-0.1);
//auto *pitchParam = new TWO_DOF_PID_PARAM<float>(0,0.006 ,0.0015,0.000018,0.1,-0.1);
//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,0.0 ,0.000,0.000,0.2,-0.2);
auto *pitchParam = new TWO_DOF_PID_PARAM<float>(0,0.0 ,0.000,0.000,0.2,-0.2);
//auto *rollParam = new TWO_DOF_PID_PARAM<float>(0,1,0.0000,0.0000,0.03,-0.03);
//auto *yawRateParam = new TWO_DOF_PID_PARAM<float>(0.0,0.0,0.00,0.00000,0.05,-0.05);
auto *yawRateParam = new TWO_DOF_PID_PARAM<float>(0.02,0.05,0.03,0.000005,0.05,-0.05);
auto *altitudeParam = new TWO_DOF_PID_PARAM<float>(0,0.1,0,0.05,0.2,-0.2);
multicopter::ALTITUDE_CONTROL_MODE initialAltitudeControl = multicopter::ALTITUDE_CONTROL_MODE::RELATIVE_THROTTLE;
multicopter::FRAME_TYPE initialFrameType = multicopter::FRAME_TYPE::QUAD_TOP;
float initialBankAngleLim = 30*std::numbers::pi / 180.0;
float initialBankAcceleLim = 270*std::numbers::pi / 180.0;
float initialYawRateLim = 90*std::numbers::pi / 180.0;

//sbus calibration values
std::array<uint16_t, 18> center = {
    1500, 1500, 966, 966,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

std::array<uint16_t, 18> upper = {
	1680, 1680, 1680, 1680,
	1904, 1904, 1904, 1904,
	1904, 1904, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

std::array<uint16_t, 18> lower = {
	368, 368, 368, 368,
	144, 144, 144, 144,
	144, 144, 1500, 1500,
    1500, 1500, 1500, 1500,
    0, 0
};

Quaternion<float> imuFrame(1/std::sqrt(2),0,0,-1/std::sqrt(2));
