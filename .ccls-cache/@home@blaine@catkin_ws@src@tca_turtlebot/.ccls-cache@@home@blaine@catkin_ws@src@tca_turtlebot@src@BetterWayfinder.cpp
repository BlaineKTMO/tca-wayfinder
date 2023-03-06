#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <

class Wayfinder 
{
    protected:
    ros::NodeHandle n;
    ros::Subscriber gwpt_sub;
    ros::Subscriber vfh_sub;
    ros::Subscriber lane_follow_sub;
    

    void receiveGoal(geometry_msgs::PoseStampedConstPtr &msg) 
    {
        // Preprocess goal (?)
        // wayfind(msg)
    }

    void sendIwpt(geometry_msgs::PoseStamped iwpt)
    {
        iwpt_pub.publish(iwpt);
    }

    void wayfind(geometry_msgs::PoseStamped gwpt) 
    {
        // Read gwpt and request (?) from VFH and Lane Follow
        // Make decision based on covariance and confidence
        // sendIwpt(iwpt);
    }

    public:
    Wayfinder() : 
        gwpt_sub(n.subscribe("/gwpt", 10, &Wayfinder::receiveGoal, this)),
        // vfh_sub(n.subscribe("/vfh", ))
        iwpt_pub(n.advertise<geometry_msgs::PoseStamped>("/wpt", 10))
    {}
};

int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "wayfinder");
}