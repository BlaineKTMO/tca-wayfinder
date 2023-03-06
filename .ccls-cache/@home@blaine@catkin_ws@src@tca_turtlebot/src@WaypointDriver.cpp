#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <tca_turtlebot/NavToGwptAction.h>
#include <fstream>
#include <vector>

typedef actionlib::SimpleActionClient<tca_turtlebot::NavToGwptAction> Nav_client;

/**
 * Sends gps goals to robot
*/
class WaypointDriver {
    protected:
    ros::NodeHandle n;
    Nav_client nav_client;
    ros::Rate rate;

    /// @brief Current waypoint, starts at 0.
    int wpt_counter = 0;
    std::vector<geometry_msgs::Point> wpt_vec;

    public:
    /// @brief Class constructor. 
    WaypointDriver() : 
        nav_client("NavToGwpt", true),
        rate(ros::Rate(2))
    {
        ROS_INFO("Waiting for gwpt action server to come up . . .");
        nav_client.waitForServer();
        ROS_INFO("Server has come up.");
    }

    /**
     * Gets gps coordinates from text file.
     * 
     * @param file_name name of file to read from.
    */
    void readWaypoints(std::string file_name) {
        std::string path = ros::package::getPath("tca_turtlebot").append("/src/");
        std::ifstream file;
        std::string line;
        
        // Guard
        if(file_name.length() < 0)
            throw std::invalid_argument("Goals text file name invalid.");

        path.append(file_name);

        // Pass empty vector
        file.open(path, std::ios::in);
        if(!file.is_open())
            return;
        
        // Fill vector
        while(getline(file, line))
        {
            geometry_msgs::Point waypoint;

            std::istringstream iss(line);
            std::string token;

            std::getline(iss, token, ';');
            waypoint.x = std::stod(token);
            
            std::getline(iss, token, ';');
            waypoint.y = std::stod(token);
            
            std::getline(iss, token, ';');
            waypoint.z = std::stod(token);
            
            this->wpt_vec.push_back(waypoint);
        }
        
        file.close();
    }

    /**
     * Executes loop for driving through waypoints.
    */
    void start() {
        while(wpt_counter < wpt_vec.size() - 1) {
            tca_turtlebot::NavToGwptGoal goal;

            // Initialize global waypoint.
            goal.gwpt.header.frame_id = "map";
            goal.gwpt.header.stamp = ros::Time(0);
            goal.gwpt.pose.position = wpt_vec.at(wpt_counter);
            goal.gwpt.pose.position.z = 0;

            goal.gwpt.pose.orientation.x = 0;
            goal.gwpt.pose.orientation.y = 0;
            goal.gwpt.pose.orientation.z = 0;
            goal.gwpt.pose.orientation.w = 1;

            ROS_INFO("Sending gwpt %d: %f, %f",
                wpt_counter,
                goal.gwpt.pose.position.x,
                goal.gwpt.pose.position.y);

            nav_client.sendGoalAndWait(goal, ros::Duration(0), ros::Duration(0));
            
            if(nav_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                // Waypoint reached
                wpt_counter++;
                ROS_INFO("Reached GWPT");
            }
            else if (nav_client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
                // Waypoint failed
                ROS_ERROR("Recovering to previous gwpt");
                wpt_counter--;
            }

            rate.sleep();
        }
    }

    // bool requestNav(tca_turtlebot::NavToGwptGoal goal) {
    //     nav_client.sendGoalAndWait(goal, ros::Duration(0), ros::Duration(0));
    //     if(nav_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //         return true;
    //     else
    //         return false;
    // }
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "WaypointDriver");

    WaypointDriver waypointDriver;
    waypointDriver.readWaypoints("goals_karina.txt");
    waypointDriver.start();

    ros::spin();
}
