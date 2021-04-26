#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>

int main(int argc, char** argv){
    ros::init(argc,argv,"odom_publish");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",100);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double z = 0.5;
    double theta = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vtheta = 0.1;

    ros::Time current_time = ros::Time::now();
    ros::Time last_time = current_time;
    ros::Rate rate(10.0);
    while (ros::ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        double dt = (current_time-last_time).toSec();
        double dx = (vx*cos(theta) - vy*sin(theta))*dt;
        double dy = (vx*sin(theta) + vy*cos(theta))*dt;
        double dtheta = vtheta*dt;

        x += dx;
        y += dy;
        theta += dtheta;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "base_link";


        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = z;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "map";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vtheta;

        odom_pub.publish(odom);
        last_time = current_time;
        rate.sleep();
    }
    return 0;
}