/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include <sgtdv_msgs/Control.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <cstdint>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

constexpr const char *NETWORKING_INTERFACE_NAME = "can0";
constexpr int CAN_BYTES_TO_SEND = sizeof(can_frame);
constexpr int INVERTOR_MSG_ID = 0x20;


struct SPEED_CONTROL
{
    uint16_t status_flag;
    uint16_t required_RPM;
    uint32_t timestamp;
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
