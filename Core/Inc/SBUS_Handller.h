#include "SBUS/sbus.h"
#include "MULTICOPTER.h"


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
        center = arg;
    };
    void setUpper(nokolat::SBUS_DATA &_arg){
        upper = arg;
    };
    void setLower(nokolat::SBUS_DATA &_arg){
        lower = arg;
    };
private:
    float getNorm(const uint8_t channel);

    nokolat::SBUS_DATA center;
    nokolat::SBUS_DATA upper;
    nokolat::SBUS_DATA lower;
}