#ifndef ROSPBERRYCAR_H
#define ROSPBERRYCAR_H

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>

typedef enum
{
    FAST_DECAY_COAST = 0, SLOW_DECAY_BRAKE
}HBridgeDriveMode;

typedef struct ROSCASDataToSTELLARIS_
{
    int8_t v_linear;
    int8_t v_angular;
    
    uint8_t cmd;		// 0b <blinky_bit> <ask_data_bit> <HBridgeDriveMode> <0> <0> <0> <0> <start_stop_car_bit>
}ROSCASDataToSTELLARIS;

typedef struct ROSCASDataFromSTELLARIS_
{
    uint8_t battery_voltage;
    uint8_t battery_current;
    int32_t left_encoder_count;
    int32_t right_encoder_count;
    
}ROSCASDataFromSTELLARIS;


class ROSpberryCar
{
public:
    ROSpberryCar();
    ~ROSpberryCar();
    
    bool initialize();
    
    void drive(int linear_velocity, int angular_velocity, HBridgeDriveMode mode = FAST_DECAY_COAST);
    
    bool getStellarisData(void);
    
    bool isReady(){ return car_ready_; }
    
private:
  
    double battery_voltage_;
    double battery_current_;
    
    bool car_ready_;
};

#endif // ROSPBERRYCAR_H
