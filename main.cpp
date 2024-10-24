#include "mbed.h"
#include <iostream>

#include "constants.h"
#include "YDLidarX4.h"

int main()
{
    int robot_radius = 250;
    auto lidar = new YDLidarX4(PIN_TX, PIN_RX, PIN_MOTOR_ENABLE, PIN_DEVICE_ENABLE, PIN_MOTOR_SPEEDCTRL, robot_radius);
    //std::cout << "SSSSS" << std::endl;
    lidar->StopScan();
    //std::cout << "TTTTT" << std::endl;
    lidar->StartScan();
    //std::cout << "UUUUU" << std::endl;
    //lidar->DeviceInfo();
    //std::cout << "WWWWW PPPP" << std::endl;
    //lidar->HealthStatus();
    //std::cout << "FDFDSFSD" << std::endl;
    //lidar->CloudAngle();
    
    //lidar->CloudData_Show();


    while (true)
    {

    }
}

