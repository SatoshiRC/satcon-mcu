#include "SBUS_Handller.h"

SBUS_HANDLE::SBUS_HANDLE(nokolat::SBUS_DATA lower, nokolat::SBUS_DATA center, nokolat::SBUS_DATA upper)
:center(center),lower(lower),upper(upper){

}

void SBUS_HANDLE::onReceive(multicopter::INPUT &input){
    input.sbusRollNorm = getRollNorm();
    input.sbusPitchNorm = getPitchNorm();
    input.sbusYawRateNorm = getYawNorm();
    input.sbusAltitudeNorm = getAltitudeNorm();
    input.updateFlag = true;
}

float SBUS_HANDLE::getNorm(const uint8_t channel){
    float tmp = data.at(channel) - center.at(channel);
    float res = 0;
    if(tmp < 0){
        res = tmp / (center.at(channel) - lower.at(channel));
    }else{
        res = tmp / (upper.at(channel) - center.at(channel));
    }
    return res;
}