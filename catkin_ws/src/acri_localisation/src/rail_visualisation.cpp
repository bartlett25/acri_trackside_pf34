#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <acri_localisation/railLine.h>
#include <acri_localisation/railLineVector.h>
#include <acri_localisation/railPair.h>
#include <acri_localisation/railPairVector.h>
#include <acri_localisation/railClosestPair.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <string>
#include <iostream>

class railVisualiser{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_lines_;
    ros::Subscriber sub_pairs_;
    ros::Subscriber sub_close_;
    ros::Publisher pub_lines_;
    ros::Publisher pub_pairs_;
    ros::Publisher pub_close_;
    std::string sub_topic_lines_;
    std::string sub_topic_pairs_;
    std::string sub_topic_close_;
    std::string pub_topic_lines_;
    std::string pub_topic_pairs_;
    std::string pub_topic_close_;
    bool display_mid_line_;
public:
    railVisualiser() : sub_topic_lines_("in_lines"), sub_topic_pairs_("in_pairs"), sub_topic_close_("in_closest"), 
    pub_topic_lines_("out_lines"), pub_topic_pairs_("out_pairs"),pub_topic_close_("out_closest"){
        sub_lines_ = nh_.subscribe(sub_topic_lines_,10,&railVisualiser::sub_cb_lines,this);
        pub_lines_ = nh_.advertise<visualization_msgs::Marker>(pub_topic_lines_,10);
        sub_pairs_ = nh_.subscribe(sub_topic_pairs_,10,&railVisualiser::sub_cb_pairs,this);
        pub_pairs_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_topic_pairs_,10);
        sub_close_ = nh_.subscribe(sub_topic_close_,10,&railVisualiser::sub_cb_close,this);
        pub_close_ = nh_.advertise<visualization_msgs::Marker>(pub_topic_close_,10);      
        sub_topic_lines_ = nh_.resolveName(sub_topic_lines_);
        pub_topic_lines_ = nh_.resolveName(pub_topic_lines_);
        sub_topic_pairs_ = nh_.resolveName(sub_topic_pairs_);
        pub_topic_pairs_ = nh_.resolveName(pub_topic_pairs_);
        sub_topic_close_ = nh_.resolveName(sub_topic_close_);
        pub_topic_close_ = nh_.resolveName(pub_topic_close_);
        nh_.param<bool>("/rail/rail_vis/display_mid_line",display_mid_line_,true);
        ROS_INFO("rail vis: subscribed topic lines [%s]",sub_topic_lines_.c_str());
        ROS_INFO("rail vis: subscribed topic pairs [%s]",sub_topic_pairs_.c_str());
        ROS_INFO("rail vis: subscribed topic close [%s]",sub_topic_close_.c_str());
        ROS_INFO("rail vis: publishing topic lines [%s]",pub_topic_lines_.c_str());
        ROS_INFO("rail vis: publishing topic pairs [%s]",pub_topic_pairs_.c_str());
        ROS_INFO("rail vis: publishing topic close [%s]",pub_topic_close_.c_str());
    }

    void sub_cb_lines(const acri_localisation::railLineVector::ConstPtr msg){
        //ROS_INFO("num lines [%d]",(int)msg->lines.size());
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = msg->header.frame_id;
        line_list.header.stamp = ros::Time::now();
        line_list.action = visualization_msgs::Marker::ADD;   
        line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.08;
        line_list.color.a = 1.0;
        line_list.color.r = 1.0;
        line_list.color.g = 0.0;
        line_list.color.b = 1.0;
        // line_list.color.r = 0.494;
        // line_list.color.g = 0.184;
        // line_list.color.b = 0.556;  
        const size_t numLines = msg->lines.size();
        for(size_t ii = 0; ii < numLines; ii++){
            line_list.points.push_back(msg->lines[ii].point1);
            line_list.points.push_back(msg->lines[ii].point2);      
        }
        pub_lines_.publish(line_list);
    }

     void sub_cb_pairs(const acri_localisation::railPairVector::ConstPtr msg){
        //ROS_INFO("num pairs [%d]",(int)msg->pairs.size());
        visualization_msgs::MarkerArray line_array;
        const int numPairs = (int)msg->pairs.size();
        for(int ii = 0; ii < numPairs; ii++){  
            visualization_msgs::Marker line_list;
            line_list.header.frame_id = msg->header.frame_id;
            line_list.header.stamp = ros::Time::now();
            line_list.action = visualization_msgs::Marker::ADD;   
            line_list.pose.orientation.w = 1.0;
            line_list.type = visualization_msgs::Marker::LINE_LIST;
            line_list.id = ii;
            line_list.scale.x = 0.08;
            line_list.color = getColor(ii); 
            line_list.points.push_back(msg->pairs[ii].line1.point1);
            line_list.points.push_back(msg->pairs[ii].line1.point2);
            line_list.points.push_back(msg->pairs[ii].line2.point1);
            line_list.points.push_back(msg->pairs[ii].line2.point2); 
            line_array.markers.push_back(line_list);    
        }
        pub_pairs_.publish(line_array);
    }

    void sub_cb_close(const acri_localisation::railClosestPair::ConstPtr msg){
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = msg->header.frame_id;
        line_list.header.stamp = ros::Time::now();
        line_list.action = visualization_msgs::Marker::ADD; 
        line_list.type = visualization_msgs::Marker::LINE_LIST;  
        line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.scale.x = 0.08;
        line_list.color.a = 1.0;
        if (msg->inrange.data){
            line_list.color.r = 0.0;
            line_list.color.b = 0.0;
            line_list.color.g = 1.0;
        }else{
            line_list.color.r = 1.0;
            line_list.color.b = 0.0;
            line_list.color.g = 0.0;
        }    
        line_list.points.push_back(msg->line1.point1);
        line_list.points.push_back(msg->line1.point2);
        line_list.points.push_back(msg->line2.point1);
        line_list.points.push_back(msg->line2.point2); 
        if (display_mid_line_){
            line_list.points.push_back(msg->midline.point1);
            line_list.points.push_back(msg->midline.point2);
        }
        pub_close_.publish(line_list);
    }

    std_msgs::ColorRGBA getColor(const size_t idx){
        std_msgs::ColorRGBA color;
        color.a = 1.0;
        int modulus = (int)idx % 4;
        if(modulus == 0){
            color.r = 1.0;//0.0;
            color.g = 0.0;//0.447;
            color.b = 0.0;//0.741;
        }else if(modulus == 1){
            color.r = 0.0;//0.85;
            color.g = 1.0;//0.325;
            color.b = 0.0;//0.098;
        }else if(modulus == 2){
            color.r = 0.0;//0.301;
            color.g = 0.0;//0.745;
            color.b = 0.0;//0.556;    
        }else{
            color.r = 0.0;//0.635;
            color.g = 1.0;//0.078;
            color.b = 1.0;//0.184;        
        }
        return color;
    }
};

int main( int argc, char** argv ){
  ros::init(argc, argv,"rail_vis");
  railVisualiser obj;
  ros::spin();
  return 0;
}