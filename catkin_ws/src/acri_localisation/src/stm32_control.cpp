#include <ros/ros.h>
#include <acri_localisation/controlToNUC.h>
#include <acri_localisation/controlFromNUC.h>
#include <acri_localisation/railLine.h>
#include <acri_localisation/railClosestPair.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/UInt32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <vector>
#include <iostream>

typedef acri_localisation::railLine railLine;
typedef acri_localisation::railClosestPair railClosestPair;
typedef acri_localisation::controlToNUC controlToNUC;
typedef acri_localisation::controlFromNUC controlFromNUC;

class Control{
private:
    uint32_t mode_nuc_;
    uint32_t mode_stm_;
    railLine midline_;
    geometry_msgs::Pose2D pose2D_;
    std::string sub_topic_line_;
    std::string sub_topic_odom_;
    std::string sub_topic_control_;
    std::string pub_topic_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_line_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_control_;
    bool in_range_;
    double voltage24_;
    double voltage48_;
public:
    double rate;
    Control(): sub_topic_line_("in_line"), sub_topic_odom_("in_odom"),
    sub_topic_control_("in_control"),pub_topic_("out_control"){
        pub_ = nh_.advertise<controlFromNUC>(pub_topic_,10);
        sub_line_ = nh_.subscribe<railClosestPair>(sub_topic_line_,1,&Control::sub_cb_line,this);
        sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(sub_topic_odom_,1,&Control::sub_cb_odom,this);
        sub_control_ = nh_.subscribe<controlToNUC>(sub_topic_control_,1,&Control::sub_cb_control,this);
        pub_topic_ = nh_.resolveName(pub_topic_);
        sub_topic_line_ = nh_.resolveName(sub_topic_line_);
        sub_topic_odom_ = nh_.resolveName(sub_topic_odom_);
        sub_topic_control_ = nh_.resolveName(sub_topic_control_);
        nh_.param<double>("/stm32/stm32_control/rate",rate,10.0);
        ROS_INFO("stm32 control: subscribed topic line[%s]",sub_topic_line_.c_str());
        ROS_INFO("stm32 control: subscribed topic odom [%s]",sub_topic_odom_.c_str());
        ROS_INFO("stm32 control: subscribed topic control [%s]",sub_topic_control_.c_str());
        ROS_INFO("stm32 control: publishing topic [%s]",pub_topic_.c_str());
        ROS_INFO("stm32 control: rate of messages [%f]",rate);
        mode_nuc_ = 0;
        mode_stm_ = 0; 
        midline_.point1.x = 0.0;
        midline_.point1.y = 0.0;
        midline_.point1.z = 0.0;
        midline_.point2.x = 0.0;
        midline_.point2.y = 0.0;
        midline_.point2.z = 0.0;
        pose2D_.x = 0.0;
        pose2D_.y = 0.0;
        pose2D_.theta = 0.0;
        in_range_ = false;
        voltage24_ = 0.0;
        voltage48_ = 0.0;
    }
    ~Control(){}

    /*----------------------------------------------------------
     * sub_cb_line: 
     * get data related to midline of a rail pair
     *---------------------------------------------------------*/

    void sub_cb_line(const railClosestPair::ConstPtr& msg){
        in_range_ = msg->inrange.data;
        midline_ = msg->midline;       
    }

    /*----------------------------------------------------------
     * sub_cb_odom: 
     * get data related to vehicle pose
     *---------------------------------------------------------*/

    void sub_cb_odom(const nav_msgs::Odometry::ConstPtr& msg){
        Eigen::Vector3f rpy = getRPY(msg->pose.pose.orientation);
        pose2D_.x = msg->pose.pose.position.x;
        pose2D_.y = msg->pose.pose.position.y;
        pose2D_.theta = rpy(2);
    }

    /*----------------------------------------------------------
     * sub_cb_control: 
     * get data related to driving mode
     *---------------------------------------------------------*/

    void sub_cb_control(const controlToNUC::ConstPtr& msg){
        voltage24_ = msg->voltage24.data;
        voltage48_ = msg->voltage48.data;
        mode_stm_ = msg->mode.data;
        if(mode_stm_ != mode_nuc_){
            mode_nuc_ = mode_stm_;
            ROS_INFO("stm32 control: driving mode changed to [%s]",getMode().c_str());
        }
    }

    /*----------------------------------------------------------
     * sendControlData: 
     * send data to STM32
     *---------------------------------------------------------*/

    void sendControlData(){
        controlFromNUC out_msg;
        out_msg.header.seq = 0;
        out_msg.header.stamp = ros::Time::now();
        out_msg.mode.data = mode_nuc_;
        out_msg.pose2D = pose2D_;
        out_msg.midline = midline_;
        out_msg.inrange.data = in_range_;
        out_msg.voltage24.data = voltage24_;
        out_msg.voltage48.data = voltage48_;
        pub_.publish(out_msg);
        //ROS_WARN("Robot X Position in NUC [%f]\n",pose2D_.x);
       // ROS_WARN("Rail X2 [%f]\n",midline_.point2.x);
    }

    /*----------------------------------------------------------
     * getRPY: 
     * get roll, pitch, and yaw from quaternion
     *---------------------------------------------------------*/

    Eigen::Vector3f getRPY(geometry_msgs::Quaternion q){

        // roll (x-axis rotation)
        Eigen::Vector3f rpy;
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        rpy(0) = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1){
            // use 90 degrees if out of range
            rpy(1) = std::copysign(M_PI / 2, sinp); 
        }else{
            rpy(1) = std::asin(sinp);
        }

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        rpy(2) = std::atan2(siny_cosp, cosy_cosp);  
        return rpy;
    }

    /*----------------------------------------------------------
     * getRotation: 
     * get rotation matrix from roll, pitch, and yaw
     *---------------------------------------------------------*/

    Eigen::Matrix3f getRotation(Eigen::Vector3f rpy){
        double cosr = cos(rpy(0));
        double sinr = sin(rpy(0));
        double cosp = cos(rpy(1));
        double sinp = sin(rpy(1));
        double cosy = cos(rpy(2));
        double siny = sin(rpy(2));
        Eigen::Matrix3f Rx;
        Eigen::Matrix3f Ry;
        Eigen::Matrix3f Rz;
        Rx <<  1   , 0,      0,
               0   , cosr,  -sinr, 
               0   , sinr,   cosr;
        Ry <<  cosp, 0,      sinp,
               0,    1,      0,
              -sinp, 0,      cosp;
        Rz <<  cosy, -siny,  0,
               siny,  cosy,  0,
               0,     0,     1;
        Eigen::Matrix3f Rnb = Rz*Ry*Rx;
        return Rnb;
    }

    /*----------------------------------------------------------
     * get mode: 
     * get mode string corresponding to flag
     *---------------------------------------------------------*/

    std::string getMode(){
        std::string mode;
        if (mode_nuc_ == 0){
            mode = "manual";
        }else if(mode_nuc_ == 1){
            mode = "deploying";
        }else if(mode_nuc_ == 2){
            mode = "deployed";
        }else if(mode_nuc_ == 3){
            mode = "isolation deploying";
        }else if(mode_nuc_ == 4){
            mode = "isolation deployed";
        }else{
            mode = "fault";
        }
        return mode;         
    }
};

int main(int argc, char** argv){
    ros::init(argc,argv,"stm32_control");
    Control obj;
    ros::Rate rate(obj.rate);
    while (ros::ok()){    
        obj.sendControlData(); 
        ros::spinOnce();
        rate.sleep();
    }   
    return 0;
}