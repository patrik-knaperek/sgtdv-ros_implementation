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
    m_canFrame.data[0] = 0x11;
    m_canFrame.data[1] = 0x22;
    //TODO send actual values for msg
    //msg->speed
    //msg->steeringAngle
    int bytesWritten = 0;
    char * data = reinterpret_cast<char*>(&m_canFrame);

    while(bytesWritten != INVERTOR_BYTES_TO_SEND)
    {
        bytesWritten += write(m_socket, data + bytesWritten - 1, INVERTOR_BYTES_TO_SEND - bytesWritten);
    }  
}

void JetsonCanInterface::Do(const sgtdv_msgs::Control &msg)
{
    m_canFrame.data[0] = 0xFF;
    m_canFrame.data[1] = 0xFF;
    m_canFrame.data[2] = 0xFF;
    m_canFrame.data[3] = 0xFF;
    m_canFrame.data[4] = 0xFF;
    m_canFrame.data[5] = 0xFF;
    m_canFrame.data[6] = 0xFF;
    m_canFrame.data[7] = 0xFF;
    //TODO send actual values for msg
    //msg->speed
    //msg->steeringAngle
    int bytesWritten = 0;
    char * data = reinterpret_cast<char*>(&m_canFrame);

    write(m_socket, &m_canFrame, sizeof(m_canFrame));

    //while(bytesWritten != INVERTOR_BYTES_TO_SEND)
    //{
    //    bytesWritten += write(m_socket, data + bytesWritten - 1, INVERTOR_BYTES_TO_SEND - bytesWritten);
    //}  
}
