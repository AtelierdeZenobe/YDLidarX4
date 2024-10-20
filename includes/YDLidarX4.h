#pragma once

#include "mbed.h"
#include "constants.h"
#include <cstdint>
#include <iostream>
#include <vector>

/**
  * @note Names are given in YDLIDARX4 datasheets
  * @link https://www.ydlidar.com/Public/upload/files/2024-02-01/YDLIDAR%20X4%20Data%20sheet%20V1.2(240125).pdf
  * @link https://www.ydlidar.com/Public/upload/files/2024-02-01/YDLIDAR%20X4%20Development%20Manual%20V1.6(240124).pdf
  */

/**
  * @brief Structure which contains the lidar command response header
  */
struct RespHeader 
{
    uint16_t start;
    uint32_t length:30;
    uint32_t mode:2;
    uint8_t typeCode;
} __attribute__((packed));

/**
  * @brief Structure which contains the cloud values scanned by the lidar
  */
struct CloudHeader
{
    uint16_t ph;
    uint8_t ct;
    uint8_t lsn;
    uint16_t fsa;
    uint16_t lsa;
    uint16_t cs;
    uint16_t si;
} __attribute__((packed));

/**
  * @brief Structure which contains the lidar device info response content
  */
struct DeviceInfo
{
    uint8_t modelNumber;
    uint8_t firmwareVersion_major;
    uint8_t firmwareVersion_minor;
    uint8_t hardwareVersion;
    uint8_t serialNumber[16];
} __attribute((packed));

/**
  * @brief Structure which contains the lidar health status response content
  */
struct HealthStatus
{
    uint8_t statusCode;
    uint8_t errorCode_lsb;
    uint8_t errorCode_msb;
} __attribute((packed));

/**
  * @brief Main class to control an YDLidarX4
  */
class YDLidarX4
{
    public:
        YDLidarX4() = delete;

        /**
          * @brief Main constructor to create the lidar
          *
          * @note By default, during the initialization, m_en and dev_en are disabled
          *
          * @param motor_enable given as m_en in the datasheet
          * @param device_enable given as dev_en in the datasheet
          * @param motor_speedCtrl given as m_sctr in the datasheet 
          */
        YDLidarX4(PinName tx, PinName rx, PinName motor_enable, PinName device_enable, PinName motor_speedCtrl);

        ~YDLidarX4();

        /**
          * @brief Function used to restart the lidar
          *
          * @note It does not switch in any mode and does not respond
          */
        //void Restart(void);

        /**
          * @brief Function used to start the scan
          *
          * @attention Even if the motor is running, it does not mean that the lidar is in scan mode until you call this function
          */
        int StartScan(void);

        /**
          * @brief Function used to stop the scan
          */
        void StopScan(void);

        /**
          * @brief Function to get the hardware and software informations of the device
          */
        void DeviceInfo(void);

        /**
          * @brief Function to get the health status of the device
          */
        void HealthStatus(void);

    private:
        DigitalOut m_motor_enable;
        DigitalOut m_device_enable;
        PwmOut m_motor_speedCtrl;

        //Internal serial used to communicate with the lidar
        BufferedSerial* m_lidar;

        void Flush(int flush);
        void Send(const uint8_t& cmd);
        bool RespHeader(struct RespHeader* respHeader, const uint8_t& cmd);
        void RespDeviceInfo(struct DeviceInfo* const deviceInfo);
        void RespDeviceInfo_Show(const struct DeviceInfo* const deviceInfo);
        void RespHealthStatus(struct HealthStatus* const healthStatus);
        void RespHealthStatus_Show(const struct HealthStatus* const healthStatus);
        void RespStopScan(void);
        void RespStartScan(void);

        //Bauderate used by the lidar to communicate
        const int BAUDERATE = 128'000;

        const int ENABLED = 1;
        const int DISABLED = 0;
        
        //Speed motor variable is used of pull-down (0V .. 5V)
        const int MOTOR_MAX_SPEED = 0;
        const int MOTOR_MIN_SPEED = 5;

        //==== CONSTANTS USED TO SEND COMMANDS TO THE LIDAR ====
        const uint8_t CMD_START = 0xA5;
        const uint8_t CMD_START_SCAN = 0x60;
        const uint8_t CMD_STOP_SCAN = 0x65;
        const uint8_t CMD_DEVICE_INFO = 0x90;
        const uint8_t CMD_HEALTH_STATUS = 0x91;
        const uint8_t CMD_RESTART = 0x80;

        //==== CONSTANTS USED TO GET HEADER RESPONSES FROM THE LIDAR ====
        const uint8_t RESP_HEADER_START_LSB = 0xA5;
        const uint8_t RESP_HEADER_START_MSB = 0x5A;

        const uint8_t RESP_HEADER_LENGTH_LSB_START_SCAN = 0x05;
        const uint8_t RESP_HEADER_LENGTH_LSB_DEVICE = 0x14;
        const uint8_t RESP_HEADER_LENGTH_LSB_HEALTH_STATUS = 0x03;
        const uint8_t RESP_HEADER_LENGTH_MSB = 0x00;

        const uint8_t RESP_HEADER_TYPE_START_SCAN = 0x81;
        const uint8_t RESP_HEADER_TYPE_DEVICE_INFO = 0x04;
        const uint8_t RESP_HEADER_TYPE_HEALTH_STATUS = 0x06;

        const int RESP_SIZE_DEVICE_INFO = 20;
        const int RESP_SIZE_DEVICE_INFO_SERIAL_NUMBER = 16;
        const int RESP_SIZE_HEALTH_STATUS = 3;
        const int RESP_SIZE_STOP_SCAN = 1;

        //==== CONSTANTS USED TO GET THE CLOUD FROM THE LIDAR ====
        const uint8_t CLOUD_HEADER_START_LSB = 0xAA;
        const uint8_t CLOUD_HEADER_START_MSB = 0x55;

        const int CLOUD_DATA_ARRAY_SIZE = 360; //360 values in the array corresponding to each degree of a circle
};