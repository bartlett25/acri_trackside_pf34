#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <pose_cov_ops/pose_cov_ops.h>

class convertOdometry{
public:
    convertOdometry() : sub_topic_("input"), pub_topic_("output"){
        pub_ = nh_.advertise<nav_msgs::Odometry>(pub_topic_,100);
        sub_ = nh_.subscribe<nav_msgs::Odometry>(sub_topic_,100,&convertOdometry::callBack,this);
    }
    void callBack(const nav_msgs::Odometry::ConstPtr& sub_msg){
        nav_msgs::Odometry pub_msg;
        pub_msg.child_frame_id = "base_link";               //child_frame_;
        pub_msg.header.frame_id = "odom";
        pub_msg.header.stamp = sub_msg->header.stamp;
    
        tf::StampedTransform transform;
        try{
            listener_.waitForTransform("odom","map",ros::Time(0),ros::Duration(1.0));
            listener_.lookupTransform("odom","map",ros::Time(0),transform);
        }catch (tf::TransformException &ex){
            ROS_WARN("Convert 0dometry: [%s]}",ex.what());
        }
        geometry_msgs::PoseWithCovariance origin; 
        origin.pose.position.x = transform.getOrigin().x();
        origin.pose.position.y = transform.getOrigin().y();
        origin.pose.position.z = transform.getOrigin().z();
        origin.pose.orientation.x = transform.getRotation().x();
        origin.pose.orientation.y = transform.getRotation().y();
        origin.pose.orientation.z = transform.getRotation().z();
        origin.pose.orientation.w = transform.getRotation().w();
        origin.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        geometry_msgs::PoseWithCovariance odom_pose;
        pose_cov_ops::inverseCompose(origin,sub_msg->pose,odom_pose);

        // Twist information already in base_link frame. Dont need to change.

        pub_msg.pose = odom_pose;
        pub_msg.twist = sub_msg->twist;
        pub_.publish(pub_msg);
    }
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::string sub_topic_;
    std::string pub_topic_;
    tf::TransformListener listener_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "convertOdometry");
    convertOdometry obj;
    ros::spin();
    return 0;
}