#ifndef ROSPBERRYCAR_H
#define ROSPBERRYCAR_H

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <ros/ros.h>
#include <string>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <stdexcept>
#include <signal.h>
#include <semaphore.h>

using namespace std;

#define BLINKY_BIT          0x80
#define ASK_DATA_BIT        0x40
#define HBRIDGEMODE_BIT     0x20
#define ASK_FIRMWARE_BIT    0x10
#define CAR_ON_BIT          0x08

#define MAX_LINEAR_VEL      10.0
#define MAX_RAW_LINEAR_VEL  127
#define MAX_ANGULAR_VEL     5.0
#define MAX_RAW_ANGULAR_VEL 127
#define MAX_DELAY_TWIST_MESSAGE 150000000 //nsec

typedef enum
{
    FAST_DECAY_COAST = 0, SLOW_DECAY_BRAKE
}HBridgeDriveMode;

typedef struct __attribute__((__packed__)ROSCASDataToSTELLARIS_
{
    int8_t v_linear;
    int8_t v_angular;    
    uint8_t cmd;		// 0b <blinky_bit> <ask_data_bit> <HBridgeDriveMode> <ask_firmware_bit> <0> <0> <0> <start_stop_car_bit>
    uint8_t useless;
}ROSCASDataToSTELLARIS;

typedef struct __attribute__((__packed__)ROSCASDataFromSTELLARIS_
{
    int32_t left_encoder_count;
    int32_t right_encoder_count;
    uint8_t battery_voltage;
    uint8_t battery_current;
    uint8_t cmd_back;
    uint8_t useless;
}ROSCASDataFromSTELLARIS;

//! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
#define DEF_EXCEPTION(name, parent) \
class name : public parent { \
    public: \
    name(const char* msg) : parent(msg) {} \
}

//! A standard exception
DEF_EXCEPTION(Exception, std::runtime_error);

//! An exception for use when a timeout is exceeded
DEF_EXCEPTION(TimeoutException, Exception);

#undef DEF_EXCEPTION

class ROSpberryCar
{
public:
    ROSpberryCar();
    ~ROSpberryCar();
    
    bool initialize(string dev_path, float rate = 10.0);
    bool deInitialize();
    
    void send_twist_cmd(float linear_velocity, float angular_velocity,
               HBridgeDriveMode mode = FAST_DECAY_COAST);
    
    float getROSpberryCarBatteryVoltage(bool force_new_data = false);
    float getROSpberryCarBatteryCurrent(bool force_new_data = false);
    int32_t getROSpberryCarLeftEncoder(bool force_new_data = false);
    int32_t getROSpberryCarRightEncoder(bool force_new_data = false);
    
    double getStellarisFirmwareVersion(void);
    bool isReady(){ return car_ready_; }
    

private:

    int fd_;

    uint8_t sendSPI(int8_t linear_velocity, int8_t angular_velocity,
                    HBridgeDriveMode mode, bool car_on_bit = false,
                    bool ask_data_bit = false, bool ask_firmware_version = false);
    void getStellarisData(float rate);
    bool startSPICommThread(float rate = 10.0);

    int8_t new_linear_vel_, new_angular_vel_;
    HBridgeDriveMode new_mode_;
    ros::Time last_twist_time_;

    float battery_voltage_;
    float battery_current_;
    int32_t left_encoder_count_;
    int32_t right_encoder_count_;
    double stellarisFirmwareVersion_;

    uint32_t spi_errors_;
    
    bool car_ready_;

    boost::thread * comm_thread_;
    bool stop_comm_;
    bool pause_comm_;

    bool blinky_bit_;
    
    sem_t spi_mutex;
};

float map(float x, float in_min, float in_max, float out_min, float out_max);

#endif // ROSPBERRYCAR_H
