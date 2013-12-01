#ifndef ROSPBERRYCAR_H
#define ROSPBERRYCAR_H

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <ros/ros.h>
#include <string>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

using namespace std;

#define BLINKY_BIT          0x80
#define ASK_DATA_BIT        0x40
#define HBRIDGEMODE_BIT     0x20
#define ASK_FIRMWARE_BIT    0x10
#define STARTSTOP_BIT       0x08

typedef enum
{
    FAST_DECAY_COAST = 0, SLOW_DECAY_BRAKE
}HBridgeDriveMode;

typedef struct ROSCASDataToSTELLARIS_
{
    int8_t v_linear;
    int8_t v_angular;
    
    uint8_t cmd;		// 0b <blinky_bit> <ask_data_bit> <HBridgeDriveMode> <ask_firmware_bit> <0> <0> <0> <start_stop_car_bit>
}ROSCASDataToSTELLARIS;

typedef struct ROSCASDataFromSTELLARIS_
{
    int32_t left_encoder_count;
    int32_t right_encoder_count;
    uint8_t battery_voltage;
    uint8_t battery_current;

    uint8_t cenas;
    uint8_t cmd_back;
    
}ROSCASDataFromSTELLARIS;


class ROSpberryCar
{
public:
    ROSpberryCar();
    ~ROSpberryCar();
    
    bool initialize(string dev_path);
    bool deInitialize();
    
    void drive(int8_t linear_velocity, int8_t angular_velocity,
               HBridgeDriveMode mode = FAST_DECAY_COAST);
    
    float getROSpberryCarBatteryVoltage(bool force_new_data = false);
    float getROSpberryCarBatteryCurrent(bool force_new_data = false);
    int16_t getROSpberryCarLeftEncoder(bool force_new_data = false);
    int16_t getROSpberryCarRightEncoder(bool force_new_data = false);
    
    float getStellarisFirmwareVersion(void);
    bool isReady(){ return car_ready_; }
    

private:

    int fd_;

    uint8_t sendSPI(int8_t linear_velocity, int8_t angular_velocity,
                    HBridgeDriveMode mode, bool start_stop_bit = false,
                    bool ask_data_bit = false, bool ask_firmware_version = false);
    void getStellarisData(float rate);
    bool startSPICommThread(float rate = 10.0);

    float battery_voltage_;
    float battery_current_;
    int32_t left_encoder_count_;
    int32_t right_encoder_count_;
    float stellarisFirmwareVersion_;

    uint32_t spi_errors_;
    
    bool car_ready_;

    boost::thread * comm_thread_;
    bool stop_comm_;
    bool pause_comm_;

    bool blinky_bit_;
};

#endif // ROSPBERRYCAR_H
