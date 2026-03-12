#include <ros/ros.h>
#include <std_msgs/String.h>
#include "legged_trajectory/Trajectory.hpp"

int main(int argc, char **argv) {
    // Node'u ilklendir
    ros::init(argc, argv, "TrajectoryNode");
    ros::NodeHandle nh;

    // 500 Hz frekansında bir Rate objesi oluştur (Saniyede 500 kez)
    ros::Rate loop_rate(500);
    double dt = loop_rate.expectedCycleTime().toSec();

    Trajectory traj(dt);
    int count = 0;
    while (ros::ok()) {

        ros::spinOnce();   // Callback'leri kontrol et
        loop_rate.sleep(); // Belirlenen frekansı korumak için bekle
        count++;
    }

    return 0;
}