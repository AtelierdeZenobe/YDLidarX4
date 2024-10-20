#include "YDLidarX4.h"

YDLidarX4::YDLidarX4(PinName tx, PinName rx, PinName motor_enable, PinName device_enable, PinName motor_speedCtrl)
    : m_motor_enable(motor_enable), m_device_enable(device_enable), m_motor_speedCtrl(motor_speedCtrl)
{
    m_lidar = new BufferedSerial(tx, rx, BAUDERATE); // Lidar is only able to communicate through baud 128'000

    m_motor_enable = ENABLED;
    m_device_enable = ENABLED;
    m_motor_speedCtrl.write(MOTOR_MIN_SPEED);
    m_motor_speedCtrl.period_us(100);
   
    Flush(0); //FLushes any remaining value in the buffer of the lidar before using
}

YDLidarX4::~YDLidarX4()
{
    StopScan();
    
    delete m_lidar;
}

void YDLidarX4::Send(const uint8_t* cmd)
{
    m_lidar->write(&CMD_START, sizeof(CMD_START));
    m_lidar->write(cmd, sizeof(*cmd));
}

int YDLidarX4::StartScan(void)
{
    m_motor_enable = ENABLED;
    m_device_enable = ENABLED;

    //std::cout << "AAAAA" << std::endl;

    //Flush(1);
    //while(m_lidar->readable()) m_lidar->sync();

    //std::cout << "BBBBB" << std::endl;
    
    Send(&CMD_START_SCAN);

    struct respHeader respHeader;
    RespHeader(&respHeader, &CMD_START_SCAN);


    //RespStartScan();

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

void YDLidarX4::RespStartScan(void)
{
    int currentPos = 0;
    uint8_t currentByte;
    std::vector<uint8_t> deviceInfo;

    std::cout << "Cloud: ";

    while(currentPos < 12)
    {
        if(m_lidar->readable() && m_lidar->read(&currentByte, sizeof(currentByte)) > 0)
        {
            std::cout << std::hex << std::bitset<8>(currentByte).to_ullong() << " ";
            deviceInfo.push_back(currentByte);
        }
        currentPos++;
    }
    std::cout << std::endl;
}

void YDLidarX4::StopScan(void)
{
    Flush(3);
    
    m_motor_enable = DISABLED;
    m_device_enable = DISABLED;

    Send(&CMD_STOP_SCAN);

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
        if(m_lidar->readable() && m_lidar->read(&currentByte, sizeof(currentByte)) > 0)
        {
            std::cout << std::hex << std::bitset<8>(currentByte).to_ullong() << " ";
            //healthStatus.push_back(currentByte);
        }
        currentPos++;
    }
    std::cout << std::endl;
}


void YDLidarX4::DeviceInfo(void)
{
    Flush(4);
    
    Send(&CMD_DEVICE_INFO);

    std::cout << "DDS" << std::endl;
    struct respHeader respHeader;
    RespHeader(&respHeader, &RESP_HEADER_TYPE_DEVICE_INFO);
    std::cout << "FZFZEFZEFZEFZEFEZFZEFEZF" << std::endl;
    RespDeviceInfo();

    std::cout << "LLLLL" << std::endl;
}

void YDLidarX4::RespDeviceInfo(void)
{
    int currentPos = 0;
    uint8_t currentByte;
    std::vector<uint8_t> deviceInfo;

    std::cout << "Device Info: ";

    while(currentPos < RESP_SIZE_DEVICE_INFO)
    {
        if(m_lidar->readable() && m_lidar->read(&currentByte, sizeof(currentByte)) > 0)
        {
            std::cout << std::hex << std::bitset<8>(currentByte).to_ullong() << " ";
            deviceInfo.push_back(currentByte);
        }
        currentPos++;
    }
    std::cout << std::endl;

    //==== Formatted response ====
    std::cout << "==== Device Info ====" << std::endl;
    std::cout << std::hex;
    std::cout << "Model number: " << std::bitset<8>(deviceInfo[0]).to_ullong() << std::endl;
    std::cout << "Firmware version: " << std::bitset<8>(deviceInfo[2]).to_ullong() 
        << "." << std::bitset<8>(deviceInfo[1]).to_ullong() << std::endl;
    std::cout << "Hardware version: " << std::bitset<8>(deviceInfo[3]).to_ullong() << std::endl;
    std::cout << "Serial number: ";
    for(auto it = deviceInfo.begin() + 4; it != deviceInfo.end(); it++)
    {
        std::cout << std::bitset<8>(*it).to_ullong();
    }
    std::cout << std::endl;
    std::cout << "=====================" << std::endl;
}

void YDLidarX4::HealthStatus(void)
{
    Flush(5);

    std::cout << "SZFZ" << std::endl;
    
    Send(&CMD_HEALTH_STATUS);

    std::cout << "OLPALK" << std::endl;

    struct respHeader respHeader;
    RespHeader(&respHeader, &RESP_HEADER_TYPE_HEALTH_STATUS);

    RespHealthStatus();

    std::cout << "VUEEEKEO" << std::endl;
}

void YDLidarX4::RespHealthStatus(void)
{
    int currentPos = 0;
    uint8_t currentByte;
    std::vector<uint8_t> healthStatus;

    std::cout << "Health Status: ";
    while(currentPos < RESP_SIZE_HEALTH_STATUS)
    {
        if(m_lidar->readable() && m_lidar->read(&currentByte, sizeof(currentByte)) > 0)
        {
            std::cout << std::hex << std::bitset<8>(currentByte).to_ullong() << " ";
            healthStatus.push_back(currentByte);
        }
        currentPos++;
    }
    std::cout << std::endl;

    //==== Formatted response ====
    std::cout << "==== Health Status ====" << std::endl;
    std::cout << std::hex;
    std::cout << "Status code: " << std::bitset<8>(healthStatus[0]).to_ullong() << std::endl;
    std::cout << "Error code: " << std::bitset<8>(healthStatus[1]).to_ullong()
        << std::bitset<8>(healthStatus[2]).to_ullong() << std::endl;
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
        std::cout << std::hex << std::bitset<8>(buffer).to_ullong() << " ";
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





bool YDLidarX4::RespHeader(struct respHeader* respHeader, const uint8_t* cmd)
{

    //uint8_t* currentHeader = (uint8_t*) respHeader;//dynamic_cast<uint8_t*>(respHeader);
    uint8_t respHeaderSize = sizeof(*respHeader); //Response header frame is made of 7 bytes
    uint8_t currentHeaderByte;
    uint8_t currentHeaderPos = 0;
    std::vector<uint8_t> currentHeader;

    std::cout << "HEEEELLO" << std::endl;

    while(currentHeaderPos < respHeaderSize)
    {
        if(/*m_lidar->readable() &&*/ m_lidar->read(&currentHeaderByte, sizeof(currentHeaderByte)) > 0)
        {
            //std::cout << "Reading... (" << std::hex << std::bitset<8>(currentHeaderPos).to_ullong() << ") #" << std::hex << std::bitset<8>(currentHeaderByte).to_ullong() << std::endl;

            currentHeader.push_back(currentHeaderByte);

           /* if(currentHeaderPos == 0 && currentHeaderByte == RESP_HEADER_START_MSB)
            {
                currentHeaderPos++;
                std::cout << "ATQF" << std::endl;
                continue;
            }

            if(currentHeaderPos == 1 && currentHeaderByte == RESP_HEADER_START_LSB)
            {
                currentHeaderPos++;
                std::cout << "KSKZ" << std::endl;
                continue;
            }*/

            
            /*switch (currentHeaderPos)
            {
                case 0:
                    if (currentHeaderByte != RESP_HEADER_START_LSB) //First starting byte not found so continue reading header buffer
                    {
                        continue;
                    }
                    break;
                case 1:
                    if (currentHeaderByte != RESP_HEADER_START_MSB)
                    {
                        currentHeaderPos = 0;
                        continue;
                    }
                    break;
            }
            currentHeader[currentHeaderPos] = currentHeaderByte;*/
            currentHeaderPos++;
            
            //continue;
        }
        //currentHeaderPos++;

        //currentHeaderPos++;
    }

    //std::cout << "Vectorizing..." << std::endl;
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

    if(*cmd == CMD_START_SCAN)
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

