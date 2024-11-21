#include "YDLidarX4.h"
#include <bitset>
#include <cmath>
#include <cstdint>
#include <memory>

YDLidarX4::YDLidarX4(std::unique_ptr<EventQueue> const& evQueue, PinName tx, PinName rx, PinName motor_enable, 
        PinName device_enable, PinName motor_speedCtrl, int const& robot_radius, int const& table_radius)
    : m_evQueue(std::move(evQueue)), m_motor_enable(motor_enable), m_device_enable(device_enable), m_motor_speedCtrl(motor_speedCtrl),
        m_distance_min(robot_radius >= MIN_DISTANCE_SCANNABLE ? robot_radius : MIN_DISTANCE_SCANNABLE),
        m_distance_max(table_radius <= MAX_DISTANCE_SCANNABLE ? table_radius : MAX_DISTANCE_SCANNABLE),
        m_lidar(std::unique_ptr<BufferedSerial>(new BufferedSerial(tx, rx, BAUDERATE)))
{
    m_motor_enable = ENABLED;
    m_device_enable = ENABLED;
    m_motor_speedCtrl.write(MOTOR_MIN_SPEED);
    m_motor_speedCtrl.period_us(100);
   
    //Flush(0); //FLushes any remaining value in the buffer of the lidar before using

    //StopScan();

    //Flush(0);
}

YDLidarX4::~YDLidarX4()
{
    StopScan();

    std::cout << "==== OVER ====" << std::endl;
}

void YDLidarX4::Send(uint8_t const& cmd)
{
    m_lidar->write(&CMD_START, sizeof(CMD_START));
    m_lidar->write(&cmd, sizeof(cmd));
}

int YDLidarX4::StartScan(void)
{
    m_motor_enable = ENABLED;
    m_device_enable = ENABLED;

    //std::cout << "AAAAA" << std::endl;

    //Flush(1);
    //while(m_lidar->readable()) m_lidar->sync();

    //std::cout << "BBBBB" << std::endl;
    
    Send(CMD_START_SCAN);

    //struct RespHeader respHeader;
    std::unique_ptr<struct RespHeader> respHeader;


    std::shared_ptr<struct CloudHeader> cloudHeader = std::make_shared<struct CloudHeader>();
    
    RespHeader(std::move(respHeader), CMD_START_SCAN);



    for(int i = 0; i < 30; i++)
    {
        RespStartScan(cloudHeader);
    }

    StopScan();
    //RespHeader contains a resp value to ensure that the scan has correctly started 
   /*int resp = 0;

    if(respHeader.typeCode == RESP_HEADER_TYPE)
    {
        resp = 1;
    }

    return resp;
    */
    //Flush(2);

    return 1;
}

bool YDLidarX4::RespStartScan(std::shared_ptr<struct CloudHeader> const& cloudHeader)
{
    uint16_t currentPos = 0x0;
    uint16_t currentByte16;
    std::shared_ptr<std::vector<uint16_t>> cloudBytes16 = std::make_shared<std::vector<uint16_t>>();

    bool started = false;
    std::cout << "Cloud frame: ";
    while(currentPos < CLOUD_HEADER_SIZE)
    {
        while(started == false && m_lidar->read(&currentByte16, sizeof(currentByte16)) > 0)
        {
            if(currentByte16 == CLOUD_HEADER_START)
            {
                std::cout << std::hex << std::bitset<16>(currentByte16).to_ullong() << " ";
                cloudBytes16->push_back(currentByte16);
                currentPos++;
                started = true;
                break;
            }
            else
            {
                continue;
            }
        }

        std:cout << " /////////////////// "; //Temporization?

        if(m_lidar->read(&currentByte16, sizeof(currentByte16)) > 0)
        {
            std::cout << std::hex << std::bitset<16>(currentByte16).to_ullong() << " | ";
            cloudBytes16->push_back(currentByte16);
        }

        currentPos++;
    }
    std::cout << std::endl;

//==== DEBUG =====
#if _DEBUG_ == TRUE
    std::cout << std::hex;
    for(auto const& cloudByte16 : cloudBytes16)
    {
        std::cout << std::bitset<16>(cloudByte16).to_ullong() << " - ";
    }
    std::cout << std::endl;
#endif
//================

    cloudHeader->ph = (*cloudBytes16)[0];
    cloudHeader->ct = (*cloudBytes16)[1] & 0xff;
    cloudHeader->lsn = (*cloudBytes16)[1] >> 8;
    cloudHeader->fsa = (*cloudBytes16)[2];
    cloudHeader->lsa = (*cloudBytes16)[3];
    cloudHeader->cs = (*cloudBytes16)[4];

    std::cout << "Cloud data: ";
    std::cout << "---- " << std::dec << +cloudHeader->lsn << " ----" << std::endl;
    cloudBytes16->clear();
    currentPos = 0;
    while(currentPos < cloudHeader->lsn)
    {
        if(m_lidar->read(&currentByte16, sizeof(currentByte16)) > 0)
        {
            std::cout << std::hex << std::bitset<16>(currentByte16).to_ullong() << " ";
            cloudBytes16->push_back(currentByte16);
        }

        currentPos++;
    }
    std::cout << std::endl;

//==== DEBUG =====
#if _DEBUG_ == TRUE
    std::cout << std::hex;
    std::cout << "ph: " << std::bitset<16>(cloudHeader->ph).to_ullong() << std::endl;
    std::cout << "ct: " << std::bitset<8>(cloudHeader->ct).to_ullong() << std::endl;
    std::cout << "lsn: " << std::bitset<8>(cloudHeader->lsn).to_ullong() << std::endl;
    std::cout << "fsa: " << std::bitset<16>(cloudHeader->fsa).to_ullong() << std::endl;
    std::cout << "lsa: " << std::bitset<16>(cloudHeader->lsa).to_ullong() << std::endl;
    std::cout << "cs: " << std::bitset<16>(cloudHeader->cs).to_ullong() << std::endl;
#endif
//================

    if(!Checksum(cloudHeader, cloudBytes16))
    {
        std::cout << "Error: Cloud checksum failed" << std::endl;
        return false;
    }

    CloudData_Compute(cloudHeader, cloudBytes16);

    return true;
}

void YDLidarX4::StopScan(void)
{
    Flush(3);
    
    m_motor_enable = DISABLED;
    m_device_enable = DISABLED;

    Send(CMD_STOP_SCAN);

    std::cout << "DDZDZDFEZFEZFEZ" << std::endl;

    /*struct respHeader respHeader;
    RespHeader(&respHeader, &RESP_HEADER_TYPE_DEVICE_INFO);

    std::cout << "Dfjlhfljerf" << std::endl;*/
    //RespStopScan();
    std::cout << "kmazkfùeakf" << std::endl;
}

void YDLidarX4::RespStopScan(void)
{
    int currentPos = 0;
    uint8_t currentByte;
    //std::vector<uint8_t> healthStatus;

    std::cout << "Health Status: ";
    while(currentPos < RESP_SIZE_STOP_SCAN)
    {
        if(m_lidar->read(&currentByte, sizeof(currentByte)) > 0)
        {
            std::cout << std::hex << std::bitset<8>(currentByte).to_ullong() << " ";
            //healthStatus.push_back(currentByte);
        }
        currentPos++;
    }
    std::cout << std::endl;
}

void YDLidarX4::DeviceInfo(bool show)
{
    Flush(4);
    
    Send(CMD_DEVICE_INFO);

    std::cout << "DDS" << std::endl;

    std::shared_ptr<struct RespHeader> respHeader = std::make_shared<struct RespHeader>();
    std::shared_ptr<struct DeviceInfo> deviceInfo = std::make_shared<struct DeviceInfo>();

    RespHeader(respHeader, RESP_HEADER_TYPE_DEVICE_INFO);
    RespDeviceInfo(deviceInfo);

    if(show)
    {
        RespDeviceInfo_Show(deviceInfo);
    }
        

    std::cout << "LLLLL" << std::endl;
}

void YDLidarX4::RespDeviceInfo(std::shared_ptr<struct DeviceInfo> const& deviceInfo)
{
    int currentPos = 0;
    uint8_t currentByte;
    std::vector<uint8_t> deviceInfoBuffer;

// ==== DEBUG ====
    std::cout << "Device Info: ";

    while(currentPos < RESP_SIZE_DEVICE_INFO)
    {
        if(m_lidar->read(&currentByte, sizeof(currentByte)) > 0)
        {
            std::cout << std::hex << std::bitset<8>(currentByte).to_ullong() << " ";
            deviceInfoBuffer.push_back(currentByte);

        }
        currentPos++;
    }
    std::cout << std::endl;
// ===============

    deviceInfo->modelNumber = deviceInfoBuffer[0];
    deviceInfo->firmwareVersion_major = deviceInfoBuffer[1];
    deviceInfo->firmwareVersion_minor = deviceInfoBuffer[2];
    deviceInfo->hardwareVersion = deviceInfoBuffer[3];
    for(int i = 0, offset = 4 ; i < RESP_SIZE_DEVICE_INFO_SERIAL_NUMBER; i++)
    {
        deviceInfo->serialNumber[i] = deviceInfoBuffer[i + offset];
    }
}
  
void YDLidarX4::RespDeviceInfo_Show(std::shared_ptr<struct DeviceInfo const> const& deviceInfo)
{
    std::cout << "==== Device Info ====" << std::endl;
    std::cout << std::hex;
    std::cout << "Model number: " << std::bitset<8>(deviceInfo->modelNumber).to_ullong() << std::endl;
    std::cout << "Firmware version: " << std::bitset<8>(deviceInfo->firmwareVersion_major).to_ullong() 
        << "." << std::bitset<8>(deviceInfo->firmwareVersion_minor).to_ullong() << std::endl;
    std::cout << "Hardware version: " << std::bitset<8>(deviceInfo->hardwareVersion).to_ullong() << std::endl;
    std::cout << "Serial number: ";
    for(int i = 0; i < RESP_SIZE_DEVICE_INFO_SERIAL_NUMBER; i++)
    {
        std::cout << std::bitset<8>(deviceInfo->serialNumber[i]).to_ullong();
    }
    std::cout << std::endl;
    std::cout << "=====================" << std::endl;
}

void YDLidarX4::HealthStatus(bool show)
{
    Flush(5);

    std::cout << "SZFZ" << std::endl;
    
    Send(CMD_HEALTH_STATUS);

    std::cout << "OLPALK" << std::endl;

    std::shared_ptr<struct RespHeader> respHeader = std::make_shared<struct RespHeader>();
    std::shared_ptr<struct HealthStatus> healthStatus = std::make_shared<struct HealthStatus>();
    //RespHeader(&respHeader, RESP_HEADER_TYPE_HEALTH_STATUS);
    RespHealthStatus(healthStatus);

    if(show)
    {
        RespHealthStatus_Show(healthStatus);
    }

    std::cout << "VUEEEKEO" << std::endl;
}

void YDLidarX4::RespHealthStatus(std::shared_ptr<struct HealthStatus> const& healthStatus)
{
    int currentPos = 0;
    uint8_t currentByte;
    std::vector<uint8_t> healthStatusBuffer;

//==== DEBUG ====
    std::cout << "Health Status: ";
    while(currentPos < RESP_SIZE_HEALTH_STATUS)
    {
        if(m_lidar->read(&currentByte, sizeof(currentByte)) > 0)
        {
            std::cout << std::hex << std::bitset<8>(currentByte).to_ullong() << " ";
            healthStatusBuffer.push_back(currentByte);
        }
        currentPos++;
    }
    std::cout << std::endl;
//===============

    healthStatus->statusCode = healthStatusBuffer[0];
    healthStatus->errorCode_lsb = healthStatusBuffer[1];
    healthStatus->errorCode_msb = healthStatusBuffer[2];
}

void YDLidarX4::RespHealthStatus_Show(std::shared_ptr<struct HealthStatus const> const& healthStatus)
{
    std::cout << "==== Health Status ====" << std::endl;
    std::cout << std::hex;
    std::cout << "Status code: " << std::bitset<8>(healthStatus->statusCode).to_ullong() << std::endl;
    std::cout << "Error code: " << std::bitset<8>(healthStatus->errorCode_lsb).to_ullong()
        << std::bitset<8>(healthStatus->errorCode_msb).to_ullong() << std::endl;
    std::cout << "=======================" << std::endl;
}









void YDLidarX4::Flush(int flush)
{
    //Send(&CMD_RESTART);
    /*
    if(flush > 3)
    {
        std::cout << "> 3" << std::endl;
        if(m_lidar->sync() == 0)
        {
            std::cout << "KLKLKL" << std::endl;
        }
        return;
    }
    */

    //if(flush > 3)
        //std::cout << "READABLE: " << m_lidar->readable() << std::endl;

    uint8_t buffer; 
    int i = 0;   

    while(m_lidar->readable())
    {
        //std::cout << "LOST HERE" << std::endl;
        //std::cout << "Value:" << m_lidar->sync() << std::endl;
       /* if(m_lidar->sync() < 0)
        {
            std::cout << "DEZDFZE" << std::endl;
            return;
        }*/
        m_lidar->read(&buffer, sizeof(buffer));
        //std::cout << std::hex << std::bitset<8>(buffer).to_ullong() << " ";
        i++;
    }
    std::cout << std::endl;
    m_lidar->sync();

    //m_lidar->sync();

    //Send(&CMD_RESTART);

    std::cout << "Count " << i << " - " << m_lidar->readable() << std::endl;
    std::cout << "Flush " << flush << std::endl;

    //wait_us(1'000'000);
}





bool YDLidarX4::RespHeader(std::shared_ptr<struct RespHeader> const& respHeader, uint8_t const& cmd)
{
    uint8_t respHeaderSize = sizeof(*respHeader); //Response header frame is made of 7 bytes
    uint8_t currentHeaderByte;
    uint8_t currentHeaderPos = 0;
    std::vector<uint8_t> currentHeader;

    std::cout << "HEEEELLO" << std::endl;

    while(currentHeaderPos < respHeaderSize)
    {
        if(m_lidar->read(&currentHeaderByte, sizeof(currentHeaderByte)) > 0)
        {
            currentHeader.push_back(currentHeaderByte);
            currentHeaderPos++;
        }
    }

    std::cout << "Command: ";
    for(int i = 0; i < currentHeader.size(); i++)
    {
        std::cout << std::hex << std::bitset<8>(currentHeader[i]).to_ullong() << " ";
    }

    std::cout << std::endl;


//CHECK VALUES
    if(currentHeader[0] != RESP_HEADER_START_LSB)
    {
        std::cout << "Error: RESP_HEADER_START_LSB failed" << std::endl;
        return false;
    }

    if(currentHeader[1] != RESP_HEADER_START_MSB)
    {
        std::cout << "Error: RESP_HEADER_START_MSB failed" << std::endl;
        return false;
    }

    if(cmd == CMD_START_SCAN)
    {
        if(currentHeader[6] != RESP_HEADER_TYPE_START_SCAN)
        {
            std::cout << "Error: RESP_HEADER_TYPE_START_SCAN failed" << std::endl;
            return false;
        }

        std:: cout << "DONE" << std::endl;
    }



    std::cout << "SUCCESS" << std::endl;

    return true;
}

bool YDLidarX4::CloudData_Compute(std::shared_ptr<struct CloudHeader const> const& cloudHeader, std::shared_ptr<std::vector<uint16_t> const> const& cloudData)
{
    double angle_fsa = (cloudHeader->fsa >> 1) / 64.0;
    double angle_lsa = (cloudHeader->lsa >> 1) / 64.0;
    double angle_i = cloudHeader->lsn != 1 ?
        std::fmod(angle_lsa - angle_fsa + 360, 360) / (cloudHeader->lsn - 1) : 0;
    
    if(cloudData->size() != cloudHeader->lsn)
    {
        std::cout << "Error: cloudData and lsn are not the same size" << std::endl;
        return false;
    }

#if _DEBUG_ == TRUE
    std::cout << "Angle: " << angle_fsa << " " << angle_lsa << " " << angle_i << std::endl;
    std::cout << "Points: ";
#endif

    for(int i = 0; i < cloudData->size(); i++)
    {
        int distance = (*cloudData)[i] / 4;
        
        if(distance <= m_distance_min || distance >= m_distance_max) //No need to compute invalid values
        {
            continue;
        }

        double angle_correction = distance != 0.0 ? 
            RAD_TO_DEG * std::atan2(21.8 * (155.3 - distance), 155.3 * distance) : 0;
        int angle = std::fmod(angle_fsa + angle_i * (i) + angle_correction + 360, 360);
        
        m_cloudData[angle] = distance;
        
#if _DEBUG_ == TRUE
        std::cout << std::dec << angle << " " << distance << " | ";
#endif

    }
#if _DEBUG_ == TRUE
    std::cout << std::endl;
#endif

    return true;
}

void YDLidarX4::CloudData_Show(void)
{
    std::cout << "==== Points ====" << std::endl;
    std::cout << std::dec;
    for(int i = 0; i < CLOUD_DATA_ARRAY_SIZE; i++)
    {
        std::cout << i << " " << m_cloudData[i] << " | ";
    }
    std::cout << std::endl;
    std::cout << "================";
    std::cout << std::endl;
}

bool YDLidarX4::Checksum(std::shared_ptr<struct CloudHeader const> const& cloudHeader, std::shared_ptr<std::vector<uint16_t> const> const& cloudData)
{
    //Swap the byte to get the order used by the developers to make their checksum
    //Example : 0xa1b2 -> 0xb2a1
    auto swapByte16 = [](uint16_t byte16){
        return (byte16 & 0xff) << 8 | byte16 >> 8;
    };

    uint16_t checksum = 0x0;

    checksum ^= cloudHeader->ph;
    checksum ^= swapByte16(cloudHeader->fsa);
    checksum ^= swapByte16(cloudHeader->lsa);
    checksum ^= cloudHeader->ct << 8 | cloudHeader->lsn;

    for(auto const& it : *cloudData)
    {
        checksum ^= swapByte16(it);
    }

//==== DEBUG ====
#if _DEBUG_ == TRUE
    std::cout << "Checksum: " << std::hex << std::bitset<16>(swapByte16(checksum)).to_ullong() << std::endl;
#endif
//===============

    return checksum == swapByte16(cloudHeader->cs);
}