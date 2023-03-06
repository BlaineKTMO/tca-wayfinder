#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class test
    {
        ros::NodeHandle n;
        ros::Publisher local_goal_pub;
        ros::Publisher target_vel_pub;

    public:
        test() :
            local_goal_pub(n.advertise<geometry_msgs::PoseStamped>("/local_goal", 10)),
            target_vel_pub(n.advertise<geometry_msgs::Twist>("/target_vel", 10))
        {

        }

        void publish() {
            geometry_msgs::PoseStamped goal;
            geometry_msgs::Twist target_vel;

            goal.header.frame_id = "map";
            
            goal.pose.orientation.x = 0;
            goal.pose.orientation.y = 0;
            goal.pose.orientation.z = 0;
            goal.pose.orientation.w = 1;

            goal.pose.position.x = 2;
            goal.pose.position.y = 0;
            
            target_vel.linear.x = 5;
            
            local_goal_pub.publish(goal);
            target_vel_pub.publish(target_vel);
        }
    };

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "test");

    test foo;
    
    while(true)
    {
        foo.publish();
        ros::spinOnce();
    }
    return 0;
}
