#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class TestTimerPub
{
public:
    ros::Publisher pub_;
    ros::Timer timer_;
    void publish(const ros::TimerEvent &evt)
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = evt.current_real;
        pub_.publish(odom);
    }
    TestTimerPub(ros::NodeHandle &nh, ros::NodeHandle &pnh):
            pub_(nh.advertise<nav_msgs::Odometry>("odom", 5)),
            timer_(nh.createTimer(ros::Duration(1./pnh.param("freq", 10.)), &TestTimerPub::publish, this))
    {
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_timer_pub");
    ros::NodeHandle nh, pnh("~");
    TestTimerPub node(nh, pnh);
//    ros::spin();
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
//    ros::AsyncSpinner spinner(4);  // Use 4 threads
//    spinner.start();
//    ros::waitForShutdown();
    return 0;
}
