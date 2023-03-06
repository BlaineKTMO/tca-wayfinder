#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

/**
 * @class Watcher
 * @brief Monitors robot goals and behaviors to ensure progress
 *
 */
class Watcher
    {
    protected:
        ros::NodeHandle n;
        ros::Subscriber goal_sub;
        ros::Subscriber wayfinder_sub;
        ros::Subscriber mbc_sub;
        ros::Rate rate;

        bool activeGoal;
        geometry_msgs::PoseStamped goal;
        std_msgs::Float32 wayfindProgress;
        std_msgs::Float32 moveBaseProgress;

        /**
         * @brief goal topic callback
         *
         * @param msg 
         */
        void updateGoal(const geometry_msgs::PoseStampedConstPtr& msg)
        {
            activeGoal = true;
            goal = *msg;
        }

        /**
         * @brief wayfinder progress callback
         *
         * @param msg 
         */
        void updateWayfindProgress(const std_msgs::Float32ConstPtr &msg)
        {
            wayfindProgress = *msg;
        }

        /**
         * @brief move_base controller progress callback
         *
         * @param msg 
         */
        void updateMoveBaseProgress(const std_msgs::Float32ConstPtr &msg)
        {
            moveBaseProgress = *msg;
        }

        /**
         * @brief Main monitor loop
         */
        void watch()
        {
            if (!activeGoal)
                return;

            /**
            * Implement checks here
            */

            rate.sleep();
            watch();
        }

    public:
        Watcher() :
            n("Watcher"),
            goal_sub(n.subscribe("goal", 10, &Watcher::updateGoal, this)),
            wayfinder_sub(n.subscribe("Wayfinder/progress", 10, &Watcher::updateWayfindProgress, this)),
            mbc_sub(n.subscribe("MoveController/progress", 10, &Watcher::updateMoveBaseProgress, this)),
            rate(ros::Rate(5))
        {}
        Watcher(Watcher &&) = default;
        Watcher(const Watcher &) = default;
        Watcher &operator=(Watcher &&) = default;
        Watcher &operator=(const Watcher &) = default;
        ~Watcher();
    };

int main (int argc, char *argv[])
{

    return 0;
}
