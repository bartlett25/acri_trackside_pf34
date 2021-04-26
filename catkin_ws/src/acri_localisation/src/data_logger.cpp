#include <ros/ros.h>
#include <rosbag/bag.h>
#include <topic_tools/shape_shifter.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <iostream>
#include <vector>
#include <string>

using topic_tools::ShapeShifter;
class DataLogger{
private:
    std::vector<ros::Subscriber> subVector_;
    std::vector<std::string> topicNames_;
    rosbag::Bag bagFile_;
    std::string bagName_;
public:
    DataLogger(ros::NodeHandle *nh){
        boost::posix_time::ptime rawStamp = ros::Time::now().toBoost();
        std::string timeStamp = boost::posix_time::to_iso_extended_string(rawStamp);
        std::string folder, name;
        nh->getParam("bag_name",name);
        nh->getParam("bag_folder",folder);
        bagName_ = folder + name + "_" + timeStamp + ".bag";
        bagFile_.open(bagName_,rosbag::bagmode::Write);
        if(!bagFile_.isOpen()){
            ROS_ERROR_STREAM("Data logging: could not open bag file");
        }
        nh->getParam("topics",topicNames_); 
        ROS_INFO_STREAM("Data logging: Logging messages from the following topics:");
        for(auto it : topicNames_){
            ros::Subscriber sub = nh->subscribe<ShapeShifter>(it, 100, boost::bind(&DataLogger::callback,this, _1, it));
            subVector_.push_back(sub);
            ROS_INFO("%s",it.c_str());
        }
    }
    void callback(const ShapeShifter::ConstPtr& msg, std::string topic){
        bagFile_.write(topic,ros::Time::now(),msg);
        return;
    }
    std::string getName(){
        return bagName_;
    }
    ~DataLogger(){
        bagFile_.close();
    }
};

int main(int argc, char** argv){
    ros::init(argc,argv,"data_logger");
    ros::NodeHandle nh("~");
    DataLogger obj(&nh);
    ROS_INFO_STREAM("Logging data...");
    ros::spin();
    std::cout << "Data logging: Data saved in " << obj.getName().c_str() << std::endl;
    return 0;
}