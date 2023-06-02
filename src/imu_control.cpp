#include <move_basic/imu_control.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "imu_control");
    ros::NodeHandle nh("~");
    ImuControl imu_control(nh);
    ros::spin();
    return 0;
}





