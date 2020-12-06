/*****************************************************/
//Organization: Stuba Green Team
//Authors: Tibor Kominko, Juraj Krasňanský
/*****************************************************/


#include "../include/JetsonCanInterface.h"

JetsonCanInterface::JetsonCanInterface()
{
    m_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(m_ifr.ifr_name, NETWORKING_INTERFACE_NAME);
    ioctl(m_socket, SIOCGIFINDEX, &m_ifr);
	
    m_socketAddress.can_family  = AF_CAN;
    m_socketAddress.can_ifindex = m_ifr.ifr_ifindex;

    bind(m_socket, (struct sockaddr *)&m_socketAddress, sizeof(m_socketAddress));

    m_canFrame.can_id  = INVERTOR_MSG_ID;
    m_canFrame.can_dlc = 8;
}

JetsonCanInterface::~JetsonCanInterface()
{

}

void JetsonCanInterface::Do(const sgtdv_msgs::Control::ConstPtr &msg)
{
    SPEED_CONTROL control;
    control.status_flag = 0;
    control.required_RPM = 20;	//TODO set from msg
    control.timestamp = 0;

    uint64_t *temp = reinterpret_cast<uint64_t*>(&(m_canFrame.data));
    (*temp) = *(reinterpret_cast<uint64_t*>(&control));

    int bytesWritten = 0;
    char * data = reinterpret_cast<char*>(&m_canFrame);

    while(bytesWritten != CAN_BYTES_TO_SEND)
    {
        bytesWritten += write(m_socket, data + bytesWritten - 1, CAN_BYTES_TO_SEND - bytesWritten);
    }  
}

void JetsonCanInterface::Do(const sgtdv_msgs::Control &msg)
{
    SPEED_CONTROL control;
    control.status_flag = 1;
    control.required_RPM = msg.speed;
    control.timestamp = 0;

    control.status_flag = __bswap_16(control.status_flag);	//MCU works with different endian
    control.required_RPM = __bswap_16(control.required_RPM);
    control.timestamp = __bswap_32(control.timestamp);    

    uint64_t *temp = reinterpret_cast<uint64_t*>(&(m_canFrame.data));
    (*temp) = *(reinterpret_cast<uint64_t*>(&control));

    int bytesWritten = 0;
    char * data = reinterpret_cast<char*>(&m_canFrame);

    while(bytesWritten != CAN_BYTES_TO_SEND)
    {
        bytesWritten += write(m_socket, data + bytesWritten, CAN_BYTES_TO_SEND - bytesWritten);	
//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
