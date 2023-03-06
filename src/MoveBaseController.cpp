// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tca_turtlebot/MoveBaseInterfaceAction.h>

// Messages
#include <move_base_msgs/MoveBaseAction.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Mb_client;
// typedef actionlib::SimpleActionClient<

class MoveBaseController 
{
    protected:
    ros::NodeHandle n;
    // Move base controller should not be able to 
    // differentiate between iwpt and gwpt
    ros::Subscriber wpt_sub;
    Mb_client mb_client;


    void processGoal(geometry_msgs::PoseStampedConstPtr &goal) 
    {
        // Goal preprocessing (?)
        move_base_msgs::MoveBaseGoal mbgoal;

        mbgoal.target_pose = *goal.get();
        mbgoal.target_pose.header.frame_id = "map";
        
        sendGoal(mbgoal);
    }

    void sendGoal(const move_base_msgs::MoveBaseGoal goal)
    {
        mb_client.sendGoal(goal);
        mb_client.
    }

    // void sendGoal(move_base_msgs::MoveBaseGoal goal, geometry_msgs::PoseStamped gwpt)
    // {
    //     // Drive to iwpt, but nav direct to goal if near
    // }

    public:
    MoveBaseController() :
        wpt_sub(n.subscribe("/wpt", 10, &MoveBaseController::processGoal, this)),
        mb_client("move_base", true)
    {}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mb_controller");
}

