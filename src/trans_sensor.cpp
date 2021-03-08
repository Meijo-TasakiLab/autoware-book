#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/point_cloud2_iterator.h"

ros::Publisher sensor_pub;
void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    /*送信するPointCloud2型のメッセージの設定*/
    sensor_msgs::PointCloud2 pc2_msg;
    pc2_msg.header=msg->header;
    pc2_msg.width=msg->ranges.size();
    pc2_msg.height=1;
    pc2_msg.point_step=16;
    pc2_msg.row_step=pc2_msg.point_step*msg->ranges.size();
    sensor_msgs::PointCloud2Modifier modify(pc2_msg);
    modify.setPointCloud2Fields(4, 
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32);
    modify.resize(pc2_msg.row_step);

    /*PointCloud2型のメッセージへ点群データを書き込み*/
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc2_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pc2_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pc2_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pc2_msg, "b");
    double d, theta, x, y;

    //1点ずつ三次元座標と色を設定する
    for(size_t i=0; i<msg->ranges.size(); ++i, 
    ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
        if(std::isnan(msg->ranges[i]) || 
        msg->ranges[i]<msg->range_min || 
        msg->ranges[i]>msg->range_max){
            d=0.0;
            theta=0.0;
        }else{
            d=msg->ranges[i];
            theta=msg->angle_min+msg->angle_increment*(double)i;
        }
        x=d*cos(theta);
        y=d*sin(theta);
        *iter_x=x;
        *iter_y=y;
        *iter_z=0.0;
        *iter_r=255;
        *iter_g=0;
        *iter_b=0;
    }

    sensor_pub.publish(pc2_msg);
} //sensorCallback関数終了のとじかっこ

int main(int argc, char **argv){
    ros::init(argc,argv,"trans_sensor");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("/scan",1000,sensorCallback);
    sensor_pub=nh.advertise<sensor_msgs::PointCloud2>("/points_raw",1000);
    ros::spin();
    return 0;
}