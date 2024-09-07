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
        auto raw = data[THROTTLE_POS];
        auto upper = this->upper[THROTTLE_POS];
        auto lower = this->lower[THROTTLE_POS];
        auto center = this->center[THROTTLE_POS];

        float res=0;

        if(raw>=center){
        	res = (raw-center)/float(upper-center);
        }else{
        	res = -((float)raw-center)/((float)lower-center);
        }
        constraint(res, -1, 1);

        return res;
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

    const uint8_t THROTTLE_POS = 2;

    void constraint(float &in, float min, float max);
};

#endif /*INC_SBUS_HANDLLER_*/
