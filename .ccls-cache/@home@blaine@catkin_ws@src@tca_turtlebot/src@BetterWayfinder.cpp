#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

const float THRESHOLD = 0.2f;

class Wayfinder 
    { 
    protected:
        ros::NodeHandle n;
        ros::Subscriber gwpt_sub;
        ros::Subscriber vfh_sub;
        ros::Subscriber lane_follow_sub;
        ros::Publisher iwpt_pub;
        ros::Publisher progress_pub;

        geometry_msgs::Point start_pos;
        float distance;

        void receiveGoal(geometry_msgs::PoseStampedConstPtr &msg) 
        {
            // Preprocess goal (?)
            // wayfind(msg)
            start_pos = getCurrentPos();
            sendIwpt(*msg);
        }

        void sendIwpt(geometry_msgs::PoseStamped iwpt)
        {
            iwpt_pub.publish(iwpt);
        }

        void wayfind(geometry_msgs::PoseStamped goal) 
        {
            geometry_msgs::PoseStamped iwpt;
            // Loop until reached
            while(distToWpt(getCurrentPos(), goal.pose.position) > THRESHOLD)
            {
                // Read gwpt and request (?) from VFH and Lane Follow
                // Make decision based on covariance and confidence

                sendIwpt(iwpt);
            }
        }

        /**
   * @brief Publish progress to goal
   *
   * @param goal  
   */
        void updateProgress(geometry_msgs::PoseStamped goal)
        {

            float dist = distToWpt(getCurrentPos(), goal.pose.position);
            std_msgs::Float32 progress;
            progress.data = dist/distance; 

            progress_pub.publish(progress);
        }

    public:
        Wayfinder() : 
            gwpt_sub(n.subscribe("/gwpt", 10, &Wayfinder::receiveGoal, this)),
            // vfh_sub(n.subscribe("/vfh", ))
            iwpt_pub(n.advertise<geometry_msgs::PoseStamped>("/wpt", 10)),
            progress_pub(n.advertise<std_msgs::Float32>("progress", 10))
        {}

    private:

        /**
   * @brief Calculates distance between two positions
   *
   * @param initial_pos first position
   * @param goal second position
   * @return distance
   */
        float distToWpt(geometry_msgs::Point initial_pos, geometry_msgs::Point goal) 
        {
            float dx = goal.x - initial_pos.x;
            float dy = goal.y - initial_pos.y;

            return pow((dx * dx) + (dy * dy), 0.5);
        }

        /**
   * @brief Returns the current odom position
   *
   * @return geometry_msgs::Point
   */
        geometry_msgs::Point getCurrentPos()
        {

            nav_msgs::OdometryConstPtr odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", n);
            geometry_msgs::Point ret;
            if (odom_ptr == NULL)
                ROS_INFO("No odom data found.");
            else
                ret = odom_ptr->pose.pose.position; 

            return ret;
        }
    };

int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "wayfinder");
}
