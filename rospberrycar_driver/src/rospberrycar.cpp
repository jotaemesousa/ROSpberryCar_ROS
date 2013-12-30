#include "rospberrycar.h"

//! Macro for throwing an exception with a message, passing args
#define ROSPBERRYCAR_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in ROSpberryCar::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
    }

ROSpberryCar::ROSpberryCar()
{
    spi_errors_ = 0;
    car_ready_ = false;
    battery_voltage_ = 0;
    battery_current_ = 0;
    stop_comm_ = true;
    pause_comm_ = true;
    blinky_bit_ = false;
    fd_ = 0;
    new_linear_vel_ = 0;
    new_angular_vel_ = 0;
    new_mode_ = FAST_DECAY_COAST;
    last_twist_time_.fromSec(0);
    sem_init(&spi_mutex, 0, 1);
}


ROSpberryCar::~ROSpberryCar()
{
}

bool ROSpberryCar::initialize(string dev_path)
{
    fd_ = open(dev_path.c_str(), O_RDWR);

    if (fd_ <= 0)
    {
        const char *extra_msg = "";
        switch(errno)
        {
        case EACCES:
            extra_msg = "You probably don't have premission to open the SPI for reading and writing. Try also changing user access privileges.";
            break;

        case ENOENT:
            extra_msg = "The requested path does not exist. Was the path name misspelled?";
            break;
        }

        ROSPBERRYCAR_EXCEPT(Exception, "Failed to open SPI: %s. %s (errno = %d). %s", dev_path.c_str(), strerror(errno), errno, extra_msg);
        return 1;
    }

    startSPICommThread();
    return 0;
}

bool ROSpberryCar::deInitialize()
{
    comm_thread_->join();
}

uint8_t ROSpberryCar::sendSPI(int8_t linear_velocity, int8_t angular_velocity,
                              HBridgeDriveMode mode, bool car_on_bit,
                              bool ask_data_bit, bool ask_firmware_version)
{
    sem_wait(&spi_mutex);
    ROSCASDataToSTELLARIS cmd_vel;
    cmd_vel.v_linear = linear_velocity;
    cmd_vel.v_angular = angular_velocity;

    blinky_bit_ ^= 1;
    uint8_t temp = 0;
    temp |= blinky_bit_ << 7;
    temp |= mode << 5;
    temp |= ask_data_bit << 6;
    temp |= ask_firmware_version << 4;
    temp |= car_on_bit;
    cmd_vel.cmd = temp;

    //ROS_INFO("cenas %d %d %d", sizeof(ROSCASDataToSTELLARIS), sizeof(ROSCASDataFromSTELLARIS), cmd_vel.cmd);
    
    uint8_t *p_valor = (uint8_t *)&cmd_vel;

    for(int i = 0; i < sizeof(ROSCASDataToSTELLARIS); i++)
    {
        write(fd_, p_valor + i, 1);
    }

    if(ask_data_bit || ask_firmware_version)
    {
	usleep(200);
        ROSCASDataFromSTELLARIS received;
        received.battery_voltage = 0;
        received.battery_current = 0;
        received.left_encoder_count = 0;
        received.right_encoder_count = 0;
        received.cmd_back = 0;

        uint8_t *p_rec = (uint8_t *)&received;

        for(int i = 0; i < sizeof(ROSCASDataFromSTELLARIS); i++)
        {
            read(fd_, p_rec + i, 1);
	   // ROS_INFO("%d, %d",i,*(p_rec + i));
        }

  //ROS_INFO("%d, %d",received.cmd_back,cmd_vel.cmd);
        if((received.cmd_back & BLINKY_BIT) != (cmd_vel.cmd & BLINKY_BIT))
        {

            ROS_ERROR("Sync. error.");
            return 1;
        }
        else
        {
            if(!ask_firmware_version)
            {
                battery_voltage_ = received.battery_voltage;
                battery_current_ = received.battery_current;
                left_encoder_count_ = received.left_encoder_count;
                right_encoder_count_ = received.right_encoder_count;
            }
            else
            {
                battery_voltage_ = received.battery_voltage;
                battery_current_ = received.battery_current;
                stellarisFirmwareVersion_ = (double)received.left_encoder_count / 10.0;
            }

        }
    }
    sem_post(&spi_mutex);
    return 0;
}

double ROSpberryCar::getStellarisFirmwareVersion(void)
{
    uint8_t temp = sendSPI(0,0,FAST_DECAY_COAST,0,1,1);
    
    if(!temp)
    {
        return stellarisFirmwareVersion_;
    }
    else
    {
        return -1.0;
    }
}

void ROSpberryCar::send_twist_cmd(float linear_velocity, float angular_velocity, HBridgeDriveMode mode)
{

    new_linear_vel_ = (int8_t)::map(linear_velocity, -MAX_LINEAR_VEL, MAX_LINEAR_VEL, -MAX_RAW_LINEAR_VEL, MAX_RAW_LINEAR_VEL);
    new_angular_vel_ = (int8_t)::map(angular_velocity, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL, -MAX_RAW_ANGULAR_VEL, MAX_RAW_ANGULAR_VEL);
    new_mode_ = mode;
    last_twist_time_ = ros::Time::now();
}

float ROSpberryCar::getROSpberryCarBatteryVoltage(bool force_new_data)
{
    if(force_new_data)
    {
        sendSPI(0,0,FAST_DECAY_COAST,false,true,false);
    }
    return (float)battery_voltage_/10.0;
}

float ROSpberryCar::getROSpberryCarBatteryCurrent(bool force_new_data)
{
    if(force_new_data)
    {
        sendSPI(0,0,FAST_DECAY_COAST,false,true,false);
    }
    return (float)battery_current_/100.0;
}

int32_t ROSpberryCar::getROSpberryCarLeftEncoder(bool force_new_data)
{
    if(force_new_data)
    {
        sendSPI(0,0,FAST_DECAY_COAST,false,true,false);
    }
    return left_encoder_count_;
}

int32_t ROSpberryCar::getROSpberryCarRightEncoder(bool force_new_data)
{
    if(force_new_data)
    {
        sendSPI(0,0,FAST_DECAY_COAST,false,true,false);
    }
    return right_encoder_count_;
}

void ROSpberryCar::getStellarisData(float rate)
{
    ros::Rate loop_r(rate);

    ROS_INFO("Communication thread started.");

    while(ros::ok())
    {


        ros::Time time_now = ros::Time::now();
        if((time_now - last_twist_time_).toNSec() < MAX_DELAY_TWIST_MESSAGE)
        {
            sendSPI(new_linear_vel_, new_angular_vel_, FAST_DECAY_COAST, true, true);
        }
        else
        {
            sendSPI(0, 0, SLOW_DECAY_BRAKE, true, false);
            ROS_INFO("twist timeout");
        }

        loop_r.sleep();
    }
    ROS_INFO("Communication thread stoped.");

}

bool ROSpberryCar::startSPICommThread(float rate)
{
    if(comm_thread_ != NULL) return false;

    stop_comm_ = false;
    pause_comm_ = false;

    comm_thread_ = new boost::thread(boost::bind(&ROSpberryCar::getStellarisData,this,rate));
    return true;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
