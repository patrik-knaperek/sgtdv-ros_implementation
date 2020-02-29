#include <sgtdv_msgs/Control.h>


class JetsonCanInterface
{
public:
    JetsonCanInterface();
    ~JetsonCanInterface();

    void Do(const sgtdv_msgs::Control::ConstPtr &msg);
private:
};