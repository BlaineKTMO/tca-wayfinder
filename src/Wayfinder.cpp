#include <ros/ros.h>
#include <tca_turtlebot/GwptService.h>
#include <tca_turtlebot/NavToGwptAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>

typedef actionlib::SimpleActionServer<tca_turtlebot::NavToGwptAction> Gwpt_server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Mb_client;

class Wayfinder {

    protected:
    ros::NodeHandle n;
    Gwpt_server gwpt_server;
    geometry_msgs::PoseStamped global_goal;
    ros::ServiceClient path_client;
    Mb_client mb_client;

    public:
    
    Wayfinder() : 
        gwpt_server(n, "NavToGwpt", true), 
        path_client(n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan")),
        mb_client("move_base", true)
    {
        gwpt_server.registerGoalCallback(boost::bind(&Wayfinder::goal_callback, this));
    }

    void goal_callback() {
        global_goal = gwpt_server.acceptNewGoal()->gwpt;
        ROS_INFO("Goal recieved: %s, %s",
            std::to_string(global_goal.pose.position.x).c_str(),
            std::to_string(global_goal.pose.position.y).c_str());
        wayfind();
    }

    void wayfind() {
        bool reached;
        do {
            nav_msgs::GetPlan path_service;
            nav_msgs::OdometryConstPtr odom_ptr;
            nav_msgs::Odometry odom_msg;
            geometry_msgs::PoseStamped goal;
            geometry_msgs::PoseStamped start;
            geometry_msgs::PoseStamped local_goal;
            move_base_msgs::MoveBaseGoal mb_goal;

            odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
            if (odom_ptr == NULL)
                ROS_INFO("No odom data found.");
            else
                odom_msg = *odom_ptr;

            start.header.frame_id = "map";
            start.header.stamp = ros::Time::now();
            start.pose = odom_msg.pose.pose;

            path_service.request.goal = global_goal;
            path_service.request.start = start;
            path_service.request.tolerance = 3;

            if(!path_client.call(path_service))
            {
                ROS_ERROR("Failed to call path service.");
                gwpt_server.setAborted();
                return;
            }

            ROS_INFO("path client called!");
            
            if (path_service.response.plan.poses.size() >= 20) {
                mb_goal.target_pose = path_service.response.plan.poses.at(20);
            }
            else {
                mb_goal.target_pose = path_service.response.plan.poses.at(path_service.response.plan.poses.size() - 1);
            }

            ROS_INFO("Local goal recieved: %s, %s", 
                std::to_string(mb_goal.target_pose.pose.position.x).c_str(), 
                std::to_string(mb_goal.target_pose.pose.position.y).c_str());
            
            mb_client.sendGoalAndWait(mb_goal);
            
            // Check for gwpt
            if (distToWpt(start.pose.position, mb_goal.target_pose.pose.position) < 0.5) {
                gwpt_server.setSucceeded();
                reached = true; 
                ROS_INFO("Reached Gwpt");
            }

            // // Check if reached iwpt
            // while(mb_client.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
            //     if(distToWpt(start.pose.position, global_goal.pose.position) < 0.3) {
            //         break;
            //     }
            // }

            ROS_INFO("Reached local waypoint");

        } while (!reached);
    }
    // Wayfinder() : client("Wayfinder", true) {
    //     ROS_INFO("Waiting for action server to come up . . .");
    //     client.waitForServer();
    //     ROS_INFO("Action server started.");
    // }
    
    // bool serveWaypoint(
    //     tca_turtlebot::GwptService::Request &req,
    //     tca_turtlebot::GwptService::Response &res) {

        
    //     return true;
    // }

    // void execute(const tca_turtlebot::NavToGwptActionConstPtr& goal) {

        

    //     gwpt_server.setSucceeded();
    // }

    private:
    double distToWpt(geometry_msgs::Point initial_pos, geometry_msgs::Point goal) {
        double dx = goal.x - initial_pos.x;
        double dy = goal.y - initial_pos.y;

        return pow((dx * dx) + (dy * dy), 0.5);
    }

};
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "Wayfinder");

    Wayfinder wayfinder;
    ros::spin();
    // n.advertiseService("gpwt_ser") 
}