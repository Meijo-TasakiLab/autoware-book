#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "autoware_msgs/VehicleCmd.h"
ros::Publisher cmd_pub;
int closest_wp=0;

//現在位置から最も近いwaypointの番号を取得する
void cwpCallback(const std_msgs::Int32::ConstPtr& msg){
    closest_wp=msg->data;
}

//Autowareからの移動命令をTurtlebot3で扱えるメッセージにする
void cmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg){
    geometry_msgs::Twist gazebo_cmd;
    gazebo_cmd=msg->twist_cmd.twist;
    if(msg->mode!=1 || closest_wp==-1){
        gazebo_cmd.linear.x=0.0;
        gazebo_cmd.linear.y=0.0;
        gazebo_cmd.linear.z=0.0;
        gazebo_cmd.angular.x=0.0;
        gazebo_cmd.angular.y=0.0;
        gazebo_cmd.angular.z=0.0;
    }
    cmd_pub.publish(gazebo_cmd);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "trans_cmd");
    ros::NodeHandle nh;
    ros::Subscriber cmd_sub=nh.subscribe("/vehicle_cmd", 1000, cmdCallback);
    ros::Subscriber cwp_sub=nh.subscribe("/closest_waypoint", 1000, cwpCallback);
    cmd_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::spin();
    return 0;
}