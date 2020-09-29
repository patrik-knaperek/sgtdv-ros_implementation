/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

constexpr const char *NETWORKING_INTERFACE_NAME = "can0"; //"can1"
constexpr int INVERTOR_BYTES_TO_SEND = 20;//TODO
constexpr int INVERTOR_MSG_ID = 0x230;
constexpr int DLCSHIT = 0;//TODO

#include <sgtdv_msgs/Control.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>


struct CAN_MSG
{
    
};


class JetsonCanInterface
{
public:
    JetsonCanInterface();
    ~JetsonCanInterface();

    void Do(const sgtdv_msgs::Control::ConstPtr &msg);
    void Do(const sgtdv_msgs::Control &msg);

private:
    int m_socket = -1;
    struct sockaddr_can m_socketAddress;
    struct can_frame m_canFrame;
    struct ifreq m_ifr;
};
