#include <ros/ros.h>
#include <tca_turtlebot/GwptService.h>
#include <tca_turtlebot/NavToGwptAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionServer<tca_turtlebot::NavToGwptAction> Gwpt_server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Mb_client;

/**
 * @class Wayfinder
 * @brief 
 * Manages intermediate waypoints
 */
class Wayfinder {

    protected:
    ros::NodeHandle n;
    Gwpt_server gwpt_server;
    ros::ServiceClient path_client;
    ros::Publisher gwpt_pub;
    Mb_client mb_client;
    geometry_msgs::PoseStamped global_goal;
    geometry_msgs::PoseStamped start;
    ros::Rate rate;
  
    public:
    Wayfinder() : 
        gwpt_server(n, "NavToGwpt", true),
        gwpt_pub(n.advertise<geometry_msgs::PoseStamped>("/wpt", 1000)),
        path_client(n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan")),
        mb_client("move_base", true),
        rate(ros::Rate(0.5))
    {   
        gwpt_server.registerGoalCallback(boost::bind(&Wayfinder::goal_callback, this));
    }

    void goal_callback() {
        nav_msgs::GetPlan path_service;
        nav_msgs::OdometryConstPtr odom_ptr;
        nav_msgs::Odometry odom_msg;
        geometry_msgs::PoseStamped goal;
        geometry_msgs::PoseStamped local_goal;
        move_base_msgs::MoveBaseGoal mb_goal;

        double dist = 99;

        // Initiate iwpt routine with gwpt pub
        global_goal = gwpt_server.acceptNewGoal()->gwpt;
        gwpt_pub.publish(global_goal);

        // Initialize move base goal for when in proximity
        mb_goal.target_pose = global_goal;
        
        // Loop until distance to global waypoint is less than 0.4 meters.
        while(dist > 9) {
            ROS_INFO("Not yet close to gwpt...");
            odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
            if (odom_ptr == NULL)
                ROS_INFO("No odom data found.");
            else
                odom_msg = *odom_ptr;

            dist = distToWpt(odom_msg.pose.pose.position, global_goal.pose.position); 
            rate.sleep();
        }

        ROS_INFO("In range of gwpt, sending gwpt...");
        // Once distance to global waypoint is less than 0.4 meters, send the global waypoint
        mb_client.cancelAllGoals();
        rate.sleep();
        mb_client.sendGoal(mb_goal);

        // Wait until robot is on top of gwpt.
        while(dist > 3) {
            odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
            if (odom_ptr == NULL)
                ROS_INFO("No odom data found.");
            else
                odom_msg = *odom_ptr;

            dist = distToWpt(odom_msg.pose.pose.position, global_goal.pose.position); 
            ROS_INFO("Waiting to get within 1m. Current distance: %fm.", dist);
            rate.sleep();
        }

        mb_client.cancelAllGoals();
        rate.sleep();
        ROS_INFO("GWPT Reached, waiting for next.");
        gwpt_server.setSucceeded();

        // start.header.frame_id = "map";
        // start.header.stamp = ros::Time::now();
        // start.pose = odom_msg.pose.pose;

        // tf2::Quaternion goal_quat;
        // goal_quat.setX(global_goal.pose.orientation.x);
        // goal_quat.setY(global_goal.pose.orientation.y);
        // goal_quat.setZ(global_goal.pose.orientation.z);
        // goal_quat.setW(global_goal.pose.orientation.w);
        // goal_quat.normalize();

        // global_goal.pose.orientation.x = goal_quat.getX();
        // global_goal.pose.orientation.y = goal_quat.getY();
        // global_goal.pose.orientation.z = goal_quat.getZ();
        // global_goal.pose.orientation.w = goal_quat.getW();

        // ROS_INFO("Goal recieved: %s, %s",
        //     std::to_string(global_goal.pose.position.x).c_str(),
        //     std::to_string(global_goal.pose.position.y).c_str());

        // // if(!wayfind()) {
        // //     move_base_msgs::MoveBaseGoal recovery_goal;
        // //     recovery_goal.target_pose = start;
        // //     recovery_goal.target_pose.pose.position.x -= 1;
        // //     recovery_goal.target_pose.pose.position.y -= 1;

        // //     if(!recovery(recovery_goal)) {
        // //         ROS_INFO("Recovery failed");
        // //         gwpt_server.setAborted();
        // //     }
        // //     ROS_INFO("Recovery succeeded.");
        // //     wayfind();
        // //     gwpt_server.setAborted();
        // // }

        // wayfind(odom_msg);

        // if(distToWpt(odom_msg.pose.pose.position, global_goal.pose.position) <= 0.4) {
        //     ROS_INFO("Close to global goal, sending gwpt.");

        //     mb_goal.target_pose = global_goal;
        //     ROS_INFO("Goal sent: (%f, %f)", 
        //         mb_goal.target_pose.pose.position.x, 
        //         mb_goal.target_pose.pose.position.y);

        //     mb_client.sendGoal(mb_goal);
        // }
        // else {
        //     ROS_INFO("Far enough from global goal, sending iwpt.");

        //     mb_goal.target_pose = wayfind(odom_msg);
        //     mb_goal.target_pose.header.frame_id = "map";
        //     mb_goal.target_pose.pose.orientation.x = 0;
        //     mb_goal.target_pose.pose.orientation.y = 0;
        //     mb_goal.target_pose.pose.orientation.z = 0;
        //     mb_goal.target_pose.pose.orientation.w = 1;

        //     ROS_INFO("Goal sent: (%f, %f)", 
        //         mb_goal.target_pose.pose.position.x, 
        //         mb_goal.target_pose.pose.position.y);

        //     mb_client.sendGoal(mb_goal);
        // }

        // while(distToWpt(odom_msg.pose.pose.position, mb_goal.target_pose.pose.position) > 0.1) {

        // }


        // // while(distToWpt(odom_msg.pose.pose.position, global_goal.pose.position) > 0.1)
        // // {
        // //     mb_client.cancelAllGoals();

        // //     odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
        // //     if (odom_ptr == NULL)
        // //         ROS_INFO("No odom data found.");
        // //     else
        // //         odom_msg = *odom_ptr;
            
        // //     if(distToWpt(odom_msg.pose.pose.position, global_goal.pose.position) <= 0.4) {
        // //         ROS_INFO("Close to global goal, sending gwpt.");
        // //         mb_goal.target_pose = global_goal;
        // //         mb_client.sendGoalAndWait(mb_goal);
        // //         if(mb_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    
        // //         }
        // //     }
        // //     else {
        // //         ROS_INFO("Far enough from global goal, sending iwpt.");
        // //         mb_goal.target_pose = wayfind(odom_msg);
        // //     }

        // //     mb_goal.target_pose.header.frame_id = "map";
        // //     mb_goal.target_pose.pose.orientation.x = 0;
        // //     mb_goal.target_pose.pose.orientation.y = 0;
        // //     mb_goal.target_pose.pose.orientation.z = 0;
        // //     mb_goal.target_pose.pose.orientation.w = 1;

        // //     ROS_INFO("Goal sent: (%f, %f)", 
        // //         mb_goal.target_pose.pose.position.x, 
        // //         mb_goal.target_pose.pose.position.y);

        // //     mb_client.sendGoal(mb_goal);

        // //     ROS_INFO("Traveling to wpt. . .");
        // //     while(distToWpt(odom_msg.pose.pose.position, mb_goal.target_pose.pose.position) > 0.4) {
        // //         odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
        // //         if (odom_ptr == NULL)
        // //             ROS_INFO("No odom data found.");
        // //         else
        // //             odom_msg = *odom_ptr;

        // //         if(mb_client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        // //             ROS_ERROR("Move base client aborted.");
        // //             mb_client.cancelAllGoals();
        // //             gwpt_server.setAborted();
        // //             return;
        // //         }
        // //     }

        // //     ROS_INFO("IWPT reached.");
        // //     ROS_INFO("Distance to gwpt: %f",
        // //         distToWpt(odom_msg.pose.pose.position, global_goal.pose.position));
        // // }

        // ROS_INFO("GWPT reached.");
        // mb_client.cancelAllGoals();
        // gwpt_server.setSucceeded();
    }

    geometry_msgs::PoseStamped wayfind(const nav_msgs::Odometry &odom_msg) {
        // bool reached = false;
        // do {
        //     nav_msgs::GetPlan path_service;
        //     nav_msgs::OdometryConstPtr odom_ptr;
        //     nav_msgs::Odometry odom_msg;
        //     geometry_msgs::PoseStamped goal;
        //     geometry_msgs::PoseStamped local_goal;
        //     move_base_msgs::MoveBaseGoal mb_goal;

        //     odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
        //     if (odom_ptr == NULL)
        //         ROS_INFO("No odom data found.");
        //     else
        //         odom_msg = *odom_ptr;

        //     start.header.frame_id = "map";
        //     start.header.stamp = ros::Time::now();
        //     start.pose = odom_msg.pose.pose;

        //     path_service.request.goal = global_goal;
        //     path_service.request.start = start;
        //     path_service.request.tolerance = 3;

        //     if(!path_client.call(path_service))
        //     {
        //         ROS_ERROR("Failed to call path service.");
        //         gwpt_server.setAborted();
        //         return false;
        //     }

        //     ROS_INFO("path client called!");
            
        //     if (path_service.response.plan.poses.size() >= 30) {
        //         mb_goal.target_pose = path_service.response.plan.poses.at(30);
        //     }
        //     else {
        //         mb_goal.target_pose = global_goal;
        //     }

        //     ROS_INFO("Local goal recieved: %s, %s", 
        //         std::to_string(mb_goal.target_pose.pose.position.x).c_str(), 
        //         std::to_string(mb_goal.target_pose.pose.position.y).c_str());
            
        //     mb_client.sendGoalAndWait(mb_goal);
            
        //     // Check for gwpt
        //     if (distToWpt(start.pose.position, mb_goal.target_pose.pose.position) < 0.1) {
        //         gwpt_server.setSucceeded();
        //         reached = true; 
        //     }

        //     // // // Check if reached iwpt
        //     // // while(mb_client.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
        //     // //     if(distToWpt(start.pose.position, global_goal.pose.position) < 0.3) {
        //     // //         break;
        //     // //     }
        //     // // }

        //     // ROS_INFO("Reached local waypoint");

        //     ROS_INFO("Checking if reached lwpt");
        //     if (mb_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        //         ROS_INFO("Reached local waypoint");
        //     }
        //     else {
        //         mb_client.cancelAllGoals();
        //         ROS_INFO("LWPT not reached, initiating recovery");
        //         return false;
        //     }

        // } while (!reached);

        // return true;
        nav_msgs::GetPlan path_service;

        start.header.frame_id = "map";
        start.header.stamp = ros::Time::now();
        start.pose = odom_msg.pose.pose;

        path_service.request.goal = global_goal;
        path_service.request.start = start;
        path_service.request.tolerance = 3;

        if(!path_client.call(path_service))
        {
            ROS_ERROR("Failed to call path service.");
            // gwpt_server.setAborted();
            // return false;
        }

        if (path_service.response.plan.poses.size() >= 30) {
            return path_service.response.plan.poses.at(30);
        }
        else {
            return *(path_service.response.plan.poses.cend() - 1);
        }
    }

    /**
     * @brief Manages recovery behavior
     *
     * @param recovery_goal 
     * @return True if recovery succeeded
     */
    bool recovery(move_base_msgs::MoveBaseGoal recovery_goal) {
        mb_client.sendGoalAndWait(recovery_goal);

        return mb_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
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
    /**
     * @brief Calculates distance between two positions
     *
     * @param initial_pos first position
     * @param goal second position
     * @return distance
     */
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
