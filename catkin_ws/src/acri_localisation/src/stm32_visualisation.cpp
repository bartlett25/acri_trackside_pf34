#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <acri_localisation/controlFromNUC.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_listener.h>
#include <cmath>
#include <string>
#include <iostream>

typedef acri_localisation::controlFromNUC controlFromNUC;

class stm32Visualiser{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_state_;
    ros::Publisher pub_legend_;
    std::string sub_topic_;
    std::string pub_topic_state_;
    std::string pub_topic_legend_;
    std::string frame_id_;
    std::string child_frame_id_;
    tf::TransformListener listener_;
    std_msgs::ColorRGBA color_;
    bool first_color_;
    double length_;
    double width_;
    double height_;
    double thres24_;
    double thres48_;
public:
    stm32Visualiser() : sub_topic_("input"), pub_topic_state_("out_state"), pub_topic_legend_("out_legend"){
        sub_= nh_.subscribe(sub_topic_,10,&stm32Visualiser::sub_cb,this);
        pub_legend_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_topic_legend_,10);
        pub_state_ = nh_.advertise<visualization_msgs::Marker>(pub_topic_state_,10);
        sub_topic_ = nh_.resolveName(sub_topic_);
        pub_topic_legend_ = nh_.resolveName(pub_topic_legend_);
        pub_topic_state_ = nh_.resolveName(pub_topic_state_);
        nh_.param<std::string>("/stm32/stm32_vis/frame_id",frame_id_,"map");
        nh_.param<std::string>("/stm32/stm32_vis/child_frame_id",child_frame_id_,"base_link");
        nh_.param<double>("/stm32/stm32_vis/length",length_,1.0);
        nh_.param<double>("/stm32/stm32_vis/width",width_,1.0);
        nh_.param<double>("/stm32/stm32_vis/height",height_,1.0);
        ROS_INFO("stm32 vis: subscribed topic [%s]",sub_topic_.c_str());
        ROS_INFO("stm32 vis: publishing topic state [%s]",pub_topic_state_.c_str());
        ROS_INFO("stm32 vis: publishing topic legend [%s]",pub_topic_legend_.c_str());
        first_color_ = false;
        color_.a = 0.7;
        color_.r = 0.0;
        color_.g = 1.0;
        color_.b = 0.0;  
        thres48_ = 47.0;
        thres24_ = 23.0;     
    }

    void sub_cb(const controlFromNUC::ConstPtr msg){
        uint32_t mode = msg->mode.data; 
        double voltage24 = msg->voltage24.data;
        double voltage48 = msg->voltage48.data;
        setColor(mode); 

        // Vehicle box display

        tf::StampedTransform transform;
        try{
            listener_.waitForTransform(frame_id_,child_frame_id_,ros::Time(0),ros::Duration(1.0));
            listener_.lookupTransform(frame_id_,child_frame_id_,ros::Time(0),transform);
        }catch (tf::TransformException &ex){
            ROS_WARN("stm_32 vis: [%s]",ex.what());
            return;
        }
        visualization_msgs::Marker vehicle;
        vehicle.header.frame_id = frame_id_;
        vehicle.header.stamp = ros::Time::now();     
        vehicle.action = visualization_msgs::Marker::ADD;
        vehicle.type = visualization_msgs::Marker::CUBE;
        vehicle.id = 0;
        vehicle.pose.position.x = transform.getOrigin().x();
        vehicle.pose.position.y = transform.getOrigin().y();
        vehicle.pose.position.z = transform.getOrigin().z() - 0.5*height_;
        vehicle.pose.orientation.w = transform.getRotation().w();
        vehicle.pose.orientation.x = transform.getRotation().x();
        vehicle.pose.orientation.y = transform.getRotation().y();
        vehicle.pose.orientation.z = transform.getRotation().z();
        vehicle.scale.x = length_;
        vehicle.scale.y = width_;
        vehicle.scale.z = height_;
        vehicle.color = color_; 
        pub_state_.publish(vehicle);

        // Mode text display

        //ROS_WARN("24V voltage: [%f]",voltage24);
        //ROS_WARN("48V voltage: [%f]",voltage48);
        visualization_msgs::MarkerArray status_box;
        visualization_msgs::Marker title;
        title.header.frame_id = frame_id_;
        title.header.stamp = ros::Time::now();
        title.action = visualization_msgs::Marker::ADD;
        title.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        title.text = "Status:";
        title.id = 0;
        title.pose.orientation.w = 1.0;
        title.pose.position.x = -10.0;
        title.pose.position.y = -1.0;
        title.pose.position.z = 0.0;
        title.scale.z = 0.5;
        title.color.a = 1.0;
        title.color.r = 0.7;
        title.color.g = 0.7;
        title.color.b = 0.7;
        status_box.markers.push_back(title);
        for(uint32_t ii = 0; ii < 3; ii++){
            visualization_msgs::Marker message = title;
            if(ii == 0){
                message.text = getModeStr(mode);
                message.color = color_;
            }else if(ii == 1){
                message.text = "24V Battery at" + std::to_string(voltage24);
                if(voltage24 < thres24_){
                    message.color.r = 1.0;
                    message.color.g = 0.0;
                    message.color.b = 0.0;
                }
            }else{
                message.text = "48V Battery at" + std::to_string(voltage48);
                if(voltage48 < thres48_){
                    message.color.r = 1.0;
                    message.color.g = 0.0;
                    message.color.b = 0.0;
                }
            }
            message.id = ii + 1;
            message.pose.position.y -= (0.75)*(ii+1);
            status_box.markers.push_back(message);
        }      
        pub_legend_.publish(status_box);
    }

    void setColor(const uint32_t mode){
        //Manual
        if(mode == 0){
            color_.r = 0.0;
            color_.g = 1.0;
            color_.b = 0.0;
        //Deploying
        }else if(mode == 1){
            if(first_color_){
                color_.r = 0.0;
                color_.g = 1.0;
                color_.b = 0.0; 
                first_color_ = false;   
            }else{
                color_.r = 1.0;
                color_.g = 0.0;
                color_.b = 0.0; 
                first_color_ = true;      
            }
        //Deployed
        }else if(mode == 2){
            color_.r = 1.0;
            color_.g = 0.0;
            color_.b = 0.0;
        //Isolation deploying
        }else if(mode == 3){
            if(first_color_){
                color_.r = 0.0;
                color_.g = 1.0;
                color_.b = 0.0; 
                first_color_ = false;   
            }else{
                color_.r = 0.0;
                color_.g = 0.0;
                color_.b = 1.0; 
                first_color_ = true;      
            }
        //Isolation deployed
        }else if(mode == 4){
            color_.r = 0.0;
            color_.g = 0.0;
            color_.b = 1.0;
        //Fault
        }else{
            color_.r = 0.6;
            color_.g = 0.6;
            color_.b = 0.6;
        }
        return;
    }

    std::string getModeStr(const uint32_t mode){
        if(mode == 0){
            return "Manual";
        }else if(mode == 1){
            return "Deploying";
        }else if(mode == 2){
            return "Deployed";
        }else if(mode == 3){
            return "Isolation Deploying";
        }else if(mode == 4){
            return "Isolation Deployed";
        }
        return "Fault";
    }
};

int main( int argc, char** argv ){
  ros::init(argc, argv,"stm32_vis");
  stm32Visualiser obj;
  ros::spin();
  return 0;
}