#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class test
    {
        ros::NodeHandle n;
        ros::Publisher local_goal_pub;
        ros::Publisher target_vel_pub;
        ros::Rate rate;

    public:
        test() :
            local_goal_pub(n.advertise<geometry_msgs::PoseStamped>("/local_goal", 10)),
            target_vel_pub(n.advertise<geometry_msgs::Twist>("/target_velocity", 10)),
            rate(ros::Rate(1))
        {

        }

        void publish() {
            geometry_msgs::PoseStamped goal;
            geometry_msgs::Twist target_vel;

            goal.header.frame_id = "odom";

            goal.pose.orientation.x = 0;
            goal.pose.orientation.y = 0;
            goal.pose.orientation.z = 0;
            goal.pose.orientation.w = 1;

            goal.pose.position.x = 25;
            goal.pose.position.y = 0;

            target_vel.linear.x = 3;

            local_goal_pub.publish(goal);
            target_vel_pub.publish(target_vel);

            rate.sleep();
        }
    };

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "test");

    test foo;

    foo.publish();
    ros::spin();
    return 0;
}
