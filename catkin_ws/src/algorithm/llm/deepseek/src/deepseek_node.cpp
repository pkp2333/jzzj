#include "deepseek/DeepSeek.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "deepseek_node");
    ros::NodeHandle nh;

    DeepSeek deepseek(nh);
    ros::Rate r(30);
    
    while (ros::ok() && deepseek.ds_task) {
        ros::spinOnce();
        r.sleep();
    }

    ROS_WARN("DeepSeek Node exit");

    return 0;
}