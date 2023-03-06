#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Mb_client;

class Pathfinder {
    protected:
    ros::NodeHandle n;
    ros::Subscriber wpt_sub;
    ros::Publisher mb_pub;
    ros::ServiceClient path_client;
    ros::Rate rate;
    Mb_client mb_client;
    
    
    public:
    Pathfinder() :
        path_client(n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan")),
        wpt_sub(n.subscribe("/wpt", 1000, &Pathfinder::Pathfind, this)),
        mb_pub(n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1)),
        rate(ros::Rate(1)),
        mb_client("move_base", true)
    {}

    void Pathfind(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        geometry_msgs::PoseStamped goal;
        geometry_msgs::PoseStamped start;       
        geometry_msgs::PoseStamped iwpt;
        nav_msgs::OdometryConstPtr odom_ptr;
        nav_msgs::Odometry odom_msg;

        nav_msgs::GetPlan path_service;

        goal = *msg.get();

        ROS_INFO("Callback initiated.");

        while(distToWpt(odom_msg.pose.pose.position, goal.pose.position) >= 10) {
            odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
            if (odom_ptr == NULL)
                ROS_INFO("No odom data found.");
            else
                odom_msg = *odom_ptr;

            ROS_INFO("Calling path service...");

            start.header.frame_id = "map";
            start.header.stamp = ros::Time::now();
            start.pose = odom_msg.pose.pose;

            // // Cancel hack
            // mb_pub.publish(start);

            path_service.request.goal = *msg.get();
            path_service.request.start = start;
            path_service.request.tolerance = 3;

            if(path_client.call(path_service))
            {
                ROS_INFO("Path service called sucessfully!");
                if (path_service.response.plan.poses.size() >= 100) {
                    ROS_INFO("Sending path at index 30.");
                    iwpt = path_service.response.plan.poses.at(100);
                }
                else {
                    ROS_INFO("Sending end of path.");
                    iwpt = *(path_service.response.plan.poses.cend() - 1);
                }

                ROS_INFO("Publishing iwpt: %f, %f",
                    iwpt.pose.position.x,
                    iwpt.pose.position.y);

                move_base_msgs::MoveBaseGoal mb_goal;
                mb_goal.target_pose.header.frame_id = "map";
                mb_goal.target_pose = goal;
                mb_goal.target_pose.pose.orientation.x = 0;
                mb_goal.target_pose.pose.orientation.y = 0;
                mb_goal.target_pose.pose.orientation.z = 0;
                mb_goal.target_pose.pose.orientation.w = 1;

                ROS_INFO("Goal sent: (%f, %f)", 
                    mb_goal.target_pose.pose.position.x, 
                    mb_goal.target_pose.pose.position.y);

                mb_client.sendGoal(mb_goal);

                while(distToWpt(odom_msg.pose.pose.position, iwpt.pose.position) > 3
                    && distToWpt(odom_msg.pose.pose.position, goal.pose.position) > 3) {
                    ROS_INFO("Not yet at iwpt ...");
                        

                    rate.sleep();
                }

                ROS_INFO("Reached iwpt.");
                mb_client.cancelAllGoals();
            } 
        }
    }

    private:
    double distToWpt(geometry_msgs::Point initial_pos, geometry_msgs::Point goal) {
        double dx = goal.x - initial_pos.x;
        double dy = goal.y - initial_pos.y;

        return pow((dx * dx) + (dy * dy), 0.5);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pathfinder");
    ros::NodeHandle n;

    ROS_INFO("Starting pathfinder service ... ");
    Pathfinder pathfinder;

    ros::spin();
}
