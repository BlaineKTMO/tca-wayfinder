#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tca_turtlebot/GwptService.h>
#include <vector>
#include <ros/package.h>
#include <fstream>

class WaypointServer {
    public:

    void readWaypoints(std::string file_name) {
        std::string path = ros::package::getPath("first_bts").append("/src/");
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

    bool serveWaypoint(tca_turtlebot::GwptService::Request &req, tca_turtlebot::GwptService::Response &res) {
        
        return true;
    }


    private:
    int wpt_counter;
    ros::Publisher gwpt_pub; // Global Waypoint Publisher
    std::vector<geometry_msgs::Point> wpt_vec;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Waypoint_Server");
    ros::NodeHandle n("~");
    
    WaypointServer wptServer;

    ros::ServiceServer service = n.advertiseService("get_gwpt", &WaypointServer::serveWaypoint, &wptServer);

    ros::spin();
}