#ifndef ROSPBERRYCAR_H
#define ROSPBERRYCAR_H

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>

typedef enum
{
    FAST_DECAY_COAST = 0, SLOW_DECAY_BRAKE
}HBridgeDriveMode;

class ROSpberryCar
{
public:
    ROSpberryCar();
    ~ROSpberryCar();
    
    bool initialize();
    
    void drive(int linear_velocity, int angular_velocity, HBridgeDriveMode mode = FAST_DECAY_COAST);
    
    bool isReady(){ return car_ready_; }
    
private:
  
    double battery_voltage_;
    double battery_current_;
    
    bool car_ready_;
};

#endif // ROSPBERRYCAR_H
