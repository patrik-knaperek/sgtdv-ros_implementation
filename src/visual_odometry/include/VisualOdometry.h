/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class VisualOdometry {
    public:
        VisualOdometry();
        ~VisualOdometry();

        void BroadcastTf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);


    private:
        tf::TransformBroadcaster m_broadcaster;

};
