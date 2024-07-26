#include "SBUS/sbus.h"
#include "MULTICOPTER.h"

#ifndef INC_SBUS_HANDLLER_
#define INC_SBUS_HANDLLER_

struct SBUS_HANDLE : public nokolat::SBUS{
    SBUS_HANDLE(nokolat::SBUS_DATA lower, nokolat::SBUS_DATA center, nokolat::SBUS_DATA upper);

    void onReceive(multicopter::INPUT &input);

    float getRollNorm(){
        return getNorm(0);
    }

    float getPitchNorm(){
        return getNorm(1);
    }

    float getYawNorm(){
        return getNorm(3);
    }

    float getAltitudeNorm(){
        //In the FUTABA propo, The throttle channel is reversed in default setting.
        return -getNorm(2);
    }

    void setCenter(nokolat::SBUS_DATA &_arg){
        center = _arg;
    };
    void setUpper(nokolat::SBUS_DATA &_arg){
        upper = _arg;
    };
    void setLower(nokolat::SBUS_DATA &_arg){
        lower = _arg;
    };
private:
    float getNorm(const uint8_t channel);

    nokolat::SBUS_DATA center;
    nokolat::SBUS_DATA upper;
    nokolat::SBUS_DATA lower;
};

#endif /*INC_SBUS_HANDLLER_*/
