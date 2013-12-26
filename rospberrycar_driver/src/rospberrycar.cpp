#include "rospberrycar.h"

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
}


ROSpberryCar::~ROSpberryCar()
{
}

bool ROSpberryCar::initialize(string dev_path)
{
    fd_ = open(dev_path.c_str(), O_RDWR);

    if (fd_ <= 0)
    {
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
                              HBridgeDriveMode mode, bool start_stop_bit,
                              bool ask_data_bit, bool ask_firmware_version)
{
    ROSCASDataToSTELLARIS cmd_vel;
    cmd_vel.v_linear = linear_velocity;
    cmd_vel.v_angular = angular_velocity;

    blinky_bit_ ^= 1;
    uint8_t temp = 0;
    temp |= blinky_bit_ << 7;
    temp |= mode << 5;
    temp |= ask_data_bit << 6;
    temp |= ask_firmware_version << 4;
    temp |= start_stop_bit;
    cmd_vel.cmd = temp;

    uint8_t *p_valor = (uint8_t *)&cmd_vel;

    for(int i = 0; i < sizeof(ROSCASDataToSTELLARIS); i++)
    {
        write(fd_, p_valor + i, 1);
    }

    if(ask_data_bit || ask_firmware_version)
    {
        usleep(2000);

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
        }

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
            stellarisFirmwareVersion_ = (float)received.left_encoder_count / 10.0;
        }

        if((received.cmd_back & BLINKY_BIT) != (cmd_vel.cmd & BLINKY_BIT))
        {
	    ROS_ERROR("Sync. error.");
            return 1;
        }
    }

    return 0;
}

float ROSpberryCar::getStellarisFirmwareVersion(void)
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

void ROSpberryCar::drive(int8_t linear_velocity, int8_t angular_velocity, HBridgeDriveMode mode)
{
    uint8_t temp = sendSPI(linear_velocity, angular_velocity, mode, true);

}

float ROSpberryCar::getROSpberryCarBatteryVoltage(bool force_new_data)
{
    if(force_new_data)
    {
	
}

float ROSpberryCar::getROSpberryCarBatteryCurrent(bool force_new_data)
{

}

int32_t ROSpberryCar::getROSpberryCarLeftEncoder(bool force_new_data)
{

}

int32_t ROSpberryCar::getROSpberryCarRightEncoder(bool force_new_data)
{

}

void ROSpberryCar::getStellarisData(float rate)
{
    ros::Rate loop_r(rate);

    ROS_INFO("Communication thread started.");

    while(ros::ok())
    {
        ROS_INFO("SPI");

//        ROSCASDataToSTELLARIS cmd_vel;
//        cmd_vel.v_linear = l;
//        cmd_vel.v_angular = a;
//        cmd_vel.cmd = cmd;
//        uint8_t *p_valor = (uint8_t *)&cmd_vel;

//        for(int i = 0; i < sizeof(struct ROSCASDataToSTELLARIS); i++)
//        {
//            write(fd_, p_valor + i, 1);
//        }

        loop_r.sleep();
    }
    ROS_INFO("Communication thread stoped.");
    /*
    struct ROSCASDataToSTELLARIS cmd_vel;
    cmd_vel.v_linear = l;
    cmd_vel.v_angular = a;
    cmd_vel.cmd = cmd;
    uint8_t *p_valor = (uint8_t *)&cmd_vel;

    cout << sizeof(struct ROSCASDataToSTELLARIS) << endl;

    for(int i = 0; i < sizeof(struct ROSCASDataToSTELLARIS); i++)
    {
        write(fd,p_valor + i,1);
    }

    //usleep(100);

    struct ROSCASDataFromSTELLARIS received;
    received.var1 = 0;
    received.var2 = 0;
    uint8_t *p_rec = (uint8_t *)&received;

    cout << sizeof(struct ROSCASDataFromSTELLARIS) << endl;

    if(cmd_vel.cmd == 1)
    {
      cout << "A receber .. " << endl;
      for(int i = 0; i < (int)sizeof(struct ROSCASDataFromSTELLARIS); i++)
      {
      read(fd, p_rec + i, 1);
      cout << "cenas\n" << endl;
      printf("byte = %x\n" , p_rec[i]);


      cout << i  << endl;
      }




      cout << "recebeu var1 = " << (int)received.var1 << " e var2 = " << (int)received.var2 << endl;
    }
     */
}

bool ROSpberryCar::startSPICommThread(float rate)
{
    if(comm_thread_ != NULL) return false;

    stop_comm_ = false;
    pause_comm_ = false;

    comm_thread_ = new boost::thread(boost::bind(&ROSpberryCar::getStellarisData,this,rate));
    return true;
}
