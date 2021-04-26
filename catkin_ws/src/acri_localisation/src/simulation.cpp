#include <ros/ros.h>
#include <acri_localisation/railLine.h>
#include <acri_localisation/railLineVector.h>
#include <string>
#include <vector>
#include <iostream>

typedef acri_localisation::railLine railLine;
typedef acri_localisation::railLineVector railLineVector;

int main(int argc, char** argv){
    ros::init(argc,argv,"line_simulation");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);
    ros::Publisher pub = nh.advertise<railLineVector>("simulation_lines",10);
    railLineVector line_msg;
    std::string frame_id;
    int num_lines;
    nh.param<std::string>("/sim_lines/frame_id",frame_id,"map");
    nh.param<int>("/sim_lines/num_lines",num_lines,0);
    std::cout << num_lines << std::endl;
    for(auto ii = 0; ii < num_lines; ii++){
        railLine this_line;
        std::vector<double> vect_line;
        std::string this_str = "/sim_lines/line" + std::to_string(ii);
        nh.getParam(this_str,vect_line);
        this_line.point1.x = vect_line[0];
        this_line.point1.y = vect_line[1];
        this_line.point2.x = vect_line[2];
        this_line.point2.y = vect_line[3];
        line_msg.lines.push_back(this_line);
        std::cout << "line: [" << this_line.point1.x << "," << this_line.point1.y << "," << this_line.point2.x << "," << this_line.point2.y << "]" << std::endl;
    }
    //int count = 0;
    while(ros::ok()){
        // line_msg.lines[0].point1.x += (double)0.01*count;
        // line_msg.lines[0].point2.x += (double)0.01*count;
        // count++;
        pub.publish(line_msg); 
        line_msg.header.frame_id = frame_id;
        line_msg.header.stamp = ros::Time::now();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}