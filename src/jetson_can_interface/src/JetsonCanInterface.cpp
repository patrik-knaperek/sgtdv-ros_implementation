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
	std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}

void JetsonCanInterface::DoListen(const std::vector<int> msgIDs)
{
    int listenSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    ifreq ifr;
    sockaddr_can socketAddress;
    can_frame canFrame;
    struct can_filter *idFilter = new can_filter[msgIDs.size()];

    for(size_t i = 0; i < msgIDs.size(); ++i)
    {
        idFilter[i].can_id = msgIDs[i];
        idFilter[i].can_mask = CAN_SFF_MASK;        
    }

    setsockopt(listenSocket, SOL_CAN_RAW, CAN_RAW_FILTER, idFilter, sizeof(*idFilter));

    strcpy(ifr.ifr_name, NETWORKING_INTERFACE_NAME);
    ioctl(listenSocket, SIOCGIFINDEX, &ifr);
	
    socketAddress.can_family  = AF_CAN;
    socketAddress.can_ifindex = ifr.ifr_ifindex;

    bind(listenSocket, (struct sockaddr *)&socketAddress, sizeof(socketAddress));

    canFrame.can_id  = INVERTOR_MSG_ID;
    canFrame.can_dlc = 8;

    int nbytes = 0;

    while (ros::ok())
    {
        nbytes = read(listenSocket, &canFrame, sizeof(struct can_frame));

        if (nbytes < 0)
        {
            std::cerr << "can raw socket - read";
            ros::shutdown();
            delete[] idFilter;
            return;
        }

        if (nbytes < sizeof(struct can_frame))
        {
            std::cerr << "read: incomplete CAN frame\n";
            continue;
        }

        switch(canFrame.can_id)
        {
            case 0x230:
	    if (*reinterpret_cast<uint64_t*>(canFrame.data) != 0)
	    {
	        std::cout << "Got 230\n";		
	        ros::shutdown();
	        delete[] idFilter;
	        return;
	    }
            break;
            default:
            std::cout << "Unknown id\n";
            break;
        }
    }

    delete[] idFilter;
}
