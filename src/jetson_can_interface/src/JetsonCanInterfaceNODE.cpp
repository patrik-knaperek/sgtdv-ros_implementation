/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/

#include "../../SGT_Macros.h"
#include <ros/ros.h>
#include "../include/JetsonCanInterface.h"
#include <sgtdv_msgs/Control.h>
#include <thread>
#include <chrono>
#include <std_msgs/Float64.h>


#ifdef SGT_USE_JOYSTICK

#include <fcntl.h>
#include <unistd.h>

constexpr int JS_EVENT_BUTTON = 0x01;    /* button pressed/released */
constexpr int JS_EVENT_AXIS = 0x02;    /* joystick moved */
constexpr int JS_EVENT_INIT = 0x80;    /* initial state of device */
constexpr int MAX_AXIS_VALUE = 32767;
constexpr int MAX_ALLOWED_TORQUE = 10;
constexpr float MAX_ALLOWED_STEERING_VELOCITY = 0.5f;
constexpr float DEADZONE_RATIO = 0.05;  //5% tolerance for zero value of analog stick
constexpr int MIN_DEADZONE_VALUE = -static_cast<int>(MAX_AXIS_VALUE * DEADZONE_RATIO);
constexpr int MAX_DEADZONE_VALUE = static_cast<int>(MAX_AXIS_VALUE * DEADZONE_RATIO);

struct js_event {
		__u32 time;     /* event timestamp in milliseconds */
		__s16 value;    /* value */
		__u8 type;      /* event type */
		__u8 number;    /* axis/button number */
	};

enum ESticks
{
    LEFT_LEFT_RIGHT = 0,
    LEFT_UP_DOWN = 1,
    RIGHT_LEFT_RIGHT = 3,
    RIGHT_UP_DOWN = 4
};

#endif  //SGT_USE_JOYSTICK

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jetsonCanInterface");
    JetsonCanInterface jetsonCanInterface;
    ros::NodeHandle handle;

    ros::Publisher steeringPublisher = handle.advertise<std_msgs::Float64>("/steering_motor/joint_velocity_controller/command", 1);

    sgtdv_msgs::Control control1;

    while(ros::ok())
    {
        //std::cin >> control1.speed;
        jetsonCanInterface.Do(control1);
        std::cout << "Seding control\n";
    }

    return 0;


#ifdef SGT_USE_JOYSTICK

    sgtdv_msgs::Control control;
    control.speed = 0;
    std_msgs::Float64 steeringVelocity;
    steeringVelocity.data = 0.;

    int fd = open ("/dev/input/js0", O_RDONLY);

    while (ros::ok())
    {       
        struct js_event e;
	    size_t bytesRead = read (fd, &e, sizeof(e));
        int value = e.value;

        if (bytesRead == sizeof(e) && e.type == JS_EVENT_AXIS)
        {
            switch (e.number)
            {
                case LEFT_LEFT_RIGHT: break;
                case LEFT_UP_DOWN:

                    if (e.value <= MAX_DEADZONE_VALUE && e.value >= 0 ||            //cannot check this before switch statement in case event is not axis but button
                        e.value >= MIN_DEADZONE_VALUE && e.value <= 0) value = 0;                    

                    control.speed = -(value / static_cast<float>(MAX_AXIS_VALUE)) * MAX_ALLOWED_TORQUE;
                    std::cout << control.speed << "\n";
                    break;

                case RIGHT_LEFT_RIGHT:

                    if (e.value <= MAX_DEADZONE_VALUE && e.value >= 0 ||               //cannot check this before switch statement in case event is not axis but button
                        e.value >= MIN_DEADZONE_VALUE && e.value <= 0) value = 0;                   

                    steeringVelocity.data = static_cast<double>((value / static_cast<float>(MAX_AXIS_VALUE)) * MAX_ALLOWED_STEERING_VELOCITY);
                    std::cout << steeringVelocity.data << "\n";
                    break;

                case RIGHT_UP_DOWN: break;
                default: break;
            }

            jetsonCanInterface.Do(control);
            steeringPublisher.publish(steeringVelocity);            
        }
    }

    close(fd);

#else
    std::thread listenThread;    

    ros::Subscriber pathTrackingSub = handle.subscribe("pathtracking_commands", 1, &JetsonCanInterface::Do, &jetsonCanInterface);

    std::vector<int> msgIDListenFilter = {0x230};

    listenThread = std::thread(&JetsonCanInterface::DoListen, msgIDListenFilter);
    listenThread.detach();

    ros::spin();
#endif  //SGT_USE_JOYSTICK

    return 0;
}
