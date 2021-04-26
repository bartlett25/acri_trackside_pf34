#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <acri_localisation/railLine.h>
#include <acri_localisation/railLineVector.h>
#include <string>
#include <vector>
#include <iostream>
#include <nav_msgs/Odometry.h>

static const std::string OPENCV_WINDOW = "Image window";
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> EigenMapXf;
typedef Eigen::Matrix<bool,Eigen::Dynamic, Eigen::Dynamic> EigenMatrixXb;
typedef Eigen::Matrix<bool,1,Eigen::Dynamic> EigenRowVectorXb;
typedef acri_localisation::railLine railLine;
typedef acri_localisation::railLineVector railLineVector;

class LineDetection{
private: 
    std::string sub_topic_;
    std::string pub_topic_lines_;
    std::string frame_id_;
    std::string child_frame_id_;
    ros::NodeHandle nh_;
    ros::Publisher pub_lines_;
    ros::Subscriber sub_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    tf::TransformListener listener_;
    double resolution_;
    double radius_;
    double floor_;
    double height_from_floor_;
    double min_rail_height_;
    double max_rail_height_; 
    double min_line_image_width_;
    double max_line_gap_; 
    double angle_;
    int accumulator_;
    int morph_scale_;
    int morph_size_; 
    int image_width_;   
    cv::Mat element_;
public:
    LineDetection(): sub_topic_("input"), pub_topic_lines_("out_lines"){
        pub_lines_ = nh_.advertise<railLineVector>(pub_topic_lines_,10);
        sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(sub_topic_,10,&LineDetection::sub_cb,this);  
        sub_topic_ = nh_.resolveName(sub_topic_);
        pub_topic_lines_ = nh_.resolveName(pub_topic_lines_);
        double max_height;
        double angle_degrees;
        nh_.param<std::string>("/rail/line_detection/frame_id",frame_id_,"map");
        nh_.param<std::string>("/rail/line_detection/child_frame_id",child_frame_id_,"base_link");
        nh_.param<double>("/rail/line_detection/resolution",resolution_,100);
        nh_.param<double>("/rail/line_detection/radius",radius_,5.0);
        nh_.param<double>("/rail/line_detection/height",max_height,2.0);
        nh_.param<double>("/rail/line_detection/floor",floor_,-2.0);
        nh_.param<double>("/rail/line_detection/min_rail_height",min_rail_height_,0.6);
        nh_.param<double>("/rail/line_detection/max_rail_height",max_rail_height_,1.2);
        nh_.param<double>("/rail/line_detection/min_line_length",min_line_image_width_,5.0);
        nh_.param<double>("/rail/line_detection/max_line_gap",max_line_gap_,10.0);
        nh_.param<double>("rail/line_detection/angle",angle_degrees,5.0);
        nh_.param<int>("/rail/line_detection/morph_scale",morph_scale_,3);
        nh_.param<int>("/rail/line_deteciton/accululator",accumulator_,50);
        height_from_floor_ = max_height + floor_;
        morph_size_ = 2*morph_scale_ + 1;
        angle_ = M_PI*(angle_degrees/180.0);
        image_width_ = (int)std::ceil(2.0*radius_*resolution_);
        element_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_size_,morph_size_), cv::Point(-1,-1));    
        ROS_INFO("line detection: subscribed topic [%s]",sub_topic_.c_str());
        ROS_INFO("line detection: publishing topic [%s]",pub_topic_lines_.c_str());

        // std::cout << "resolution " << resolution_ << std::endl;
        // std::cout << "radius " << radius_ << std::endl;
        // std::cout << "total height " << height_from_floor_ << std::endl;
        // std::cout << "floor" << floor_ << std::endl;
        // std::cout << "min_rail_height " << min_rail_height_<< std::endl;
        // std::cout << "max_rail_height " << max_rail_height_ << std::endl; 
        // std::cout << "morph_size " << morph_scale_ << std::endl;
        // std::cout << "min_line_length " << min_line_image_width_ << std::endl;
        // std::cout << "max_line_gap " << max_line_gap_ << std::endl;
        // std::cout << "angle " << angle_ << std::endl;
    }
    ~LineDetection(){}
    void sub_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
        if ((msg->width*msg->height) == 0){
            ROS_WARN_STREAM("line detection: subscribed cloud is not dense");
            return;
        }
        try{
            pcl::fromROSMsg(*msg,cloud_);
        }catch (std::runtime_error &ex){
            ROS_WARN("line detection: could not convert subscribed cloud to pcl: %s", ex.what());
            return;
        } 

        // place point cloud into discrete frame with origin (-radius_,-radius_,-floor_) away from base_link position.

        tf::StampedTransform transform;
        try{
            listener_.waitForTransform(frame_id_,child_frame_id_,ros::Time(0),ros::Duration(1.0));
            listener_.lookupTransform(frame_id_,child_frame_id_,ros::Time(0),transform);
        }catch (tf::TransformException &ex){
            ROS_WARN("line detection: [%s]}",ex.what());
            return;
        }
        auto const map_global = cloud_.getMatrixXfMap(3,4,0);
        Eigen::Vector3f offset(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
        offset[0] = offset[0] - radius_;
        offset[1] = offset[1] - radius_;
        offset[2] = offset[2] - floor_;
        Eigen::Matrix3Xf map_local = map_global.colwise() - offset;
        map_local.topRows(2) = map_local.topRows(2)*resolution_;
        Eigen::Matrix3Xf map_discrete = map_local.array().ceil();
        map_discrete.row(2) = map_local.row(2);

        // find all points in the current field of view

        const int num_points = map_discrete.cols();
        EigenMatrixXb in_radius_mat(6,num_points);
        in_radius_mat.row(0) = (map_discrete.row(0).array() > 0); 
        in_radius_mat.row(1) = (map_discrete.row(0).array() < image_width_); 
        in_radius_mat.row(2) = (map_discrete.row(1).array() > 0);
        in_radius_mat.row(3) = (map_discrete.row(1).array() < image_width_);
        in_radius_mat.row(4) = (map_discrete.row(2).array() > 0);
        in_radius_mat.row(5) = (map_discrete.row(2).array() < height_from_floor_); 
        EigenRowVectorXb in_fov = ((in_radius_mat.colwise().count()).array() >= 6);
        const int num_points_in_fov = in_fov.count();
        if (num_points_in_fov == 0){
            ROS_WARN("line detection: no points in fov");
            return;
        }

        // create birds-eye view of point-cloud within fov
        
        Eigen::MatrixXi index_img_to_points(image_width_,image_width_);
        index_img_to_points.setConstant(-1);
        cv::Mat img(image_width_,image_width_,CV_32FC1,cv::Scalar(0));
        for (int ii = 0; ii < num_points; ii++){
            if(in_fov(ii)){
                int x = (int)map_discrete(0,ii);
                int y = (int)map_discrete(1,ii);
                float z = map_discrete(2,ii);
                if (z > img.at<float>(cv::Point(x,y))){
                    img.at<float>(cv::Point(x,y)) = z;
                    index_img_to_points(x,y) = ii;
                }
            }
        }

        // Perform a morphological closing operation to remove discontinuties.
  
        cv::Mat morph_img;     
        cv::morphologyEx(img,morph_img,cv::MORPH_CLOSE,element_);

        // Obtain height difference image with thresholding

        float max_coeff = 0;
        EigenMapXf morph_matrix(morph_img.ptr<float>(), morph_img.rows, morph_img.cols);
        cv::Mat height_diff_img(image_width_,image_width_,CV_32FC1,cv::Scalar(0));
        for(int ii = 1; ii < (morph_matrix.rows()-1); ii++){
            for(int jj = 1; jj < (morph_matrix.cols()-1); jj++){
                if (morph_matrix(ii,jj) > 0){
                    Eigen::Matrix3f neighbours = morph_matrix.block<3,3>(ii-1,jj-1);
                    Eigen::Matrix3f neighbour_height_diff_abs = ((-neighbours).array() + morph_matrix(ii,jj)).abs();
                    for(int kk = 0; kk < 3; kk++){
                        for(int ll = 0; ll < 3; ll++){
                            if(neighbours(kk,ll) <= 0){
                                neighbour_height_diff_abs(kk,ll) = 0;
                            }
                        }
                    }
                    height_diff_img.at<float>(ii,jj) = neighbour_height_diff_abs.maxCoeff();
                    max_coeff = std::max(max_coeff,height_diff_img.at<float>(ii,jj));
                }
            }
        }
        cv::inRange(height_diff_img,min_rail_height_,max_rail_height_,height_diff_img);

        // Perform Hough Transform

        cv::Mat hough_img;
        cv::Mat scaled_img = 255.0*height_diff_img;
        scaled_img.convertTo(hough_img,CV_8UC1);
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(hough_img,lines, 1, CV_PI/90, accumulator_, min_line_image_width_, max_line_gap_);
      
        // Make line projection into occupancy map

        railLineVector lines_msg;
        for(size_t ii = 0; ii < lines.size(); ii++){
            int x1 = lines[ii][0];
            int y1 = lines[ii][1]; 
            bool match = true;         
            railLine rail_line;
            if (img.at<float>(cv::Point(x1,y1)) <= 0){
                match = false;
                int min_x = x1-morph_scale_;
                int max_x = x1+morph_scale_;
                int min_y = y1-morph_scale_;
                int max_y = y1+morph_scale_;
                for(int jj = min_x; jj < max_x; jj++){
                    int tempx = std::max(std::min(jj,image_width_-1),0);
                    for(int kk = min_y; kk < max_y; kk++){
                        int tempy = std::max(std::min(kk,image_width_-1),0);
                        if (img.at<float>(cv::Point(tempx,tempy)) > 0){
                            x1 = tempx;
                            y1 = tempy;
                            match = true;
                            break;
                        }
                    }
                }
            }   
            int x2 = lines[ii][2];
            int y2 = lines[ii][3];
            if (img.at<float>(cv::Point(x2,y2)) <= 0){
                match = false;
                int min_x = x2-morph_scale_;
                int max_x = x2+morph_scale_;
                int min_y = y2-morph_scale_;
                int max_y = y2+morph_scale_;
                for(int jj = min_x; jj < max_x; jj++){
                    int tempx = std::max(std::min(jj,image_width_-1),0);
                    for(int kk = min_y; kk < max_y; kk++){
                        int tempy = std::max(std::min(kk,image_width_-1),0);
                        if (img.at<float>(cv::Point(tempx,tempy)) > 0){
                            x2 = tempx;
                            y2 = tempy;
                            match = true;
                            break;
                        }
                    }
                }
            }
            if (match){
                int index_img_to_point1 = index_img_to_points(x1,y1);
                if(index_img_to_point1 < 0){
                    ROS_INFO_STREAM("line detection: No pixel match found for line IDX 1");
                    break;    
                }
                Eigen::Vector3f point1 = map_global.col(index_img_to_point1);
                rail_line.point1.x = point1(0);
                rail_line.point1.y = point1(1);
                rail_line.point1.z = point1(2)+0.05;
                int index_img_to_point2 = index_img_to_points(x2,y2);
                if(index_img_to_point2 < 0){
                    ROS_INFO_STREAM("line detection: No pixel match found for line IDX 2");
                    break;    
                }
                Eigen::Vector3f point2 = map_global.col(index_img_to_point2);
                rail_line.point2.x = point2(0);
                rail_line.point2.y = point2(1);
                rail_line.point2.z = point2(2)+0.05;

                //check gradient, if not correct, set z equal to lowest of the two.
         
                double diff_x = point1(0) - point2(0);
                double diff_y = point1(1) - point2(1);
                double diff_z = std::abs(point1(2)) - point2(2);
                double dist = std::sqrt(diff_x*diff_x + diff_y*diff_y);
                double max_diff_z = dist*std::tan(angle_);
                if (diff_z > max_diff_z){
                    if (point1(2) > point2(2)){
                        rail_line.point1.z = rail_line.point2.z;
                    }else{
                        rail_line.point2.z = rail_line.point1.z;
                    }
                }
                lines_msg.lines.push_back(rail_line);
            }else{
                 ROS_INFO_STREAM("line detection: No pixel match found for line");
            }
        }
        lines_msg.header.frame_id = frame_id_;
        lines_msg.header.stamp = ros::Time::now();
        pub_lines_.publish(lines_msg);

        // cv::imshow("Initial Image",img);
        // cv::waitKey(0);
        // cv::imshow("Morph Image",morph_img);
        // cv::waitKey(0);
        // cv::imshow("Height Image",height_diff_img);
        // cv::waitKey(0);
        // int numLines = (int)lines.size();
        // cv::Mat color_img;
        // cv::cvtColor(img,color_img,CV_GRAY2BGR);
        // for(int ii = 0; ii < numLines; ii++){
        //     cv::Point p1;
        //     p1.x = lines[ii][0];
        //     p1.y = lines[ii][1];
        //     cv::Point p2;
        //     p2.x = lines[ii][2];
        //     p2.y = lines[ii][3];
        //     cv::line(color_img, p1, p2, cv::Scalar(0,2550,0),1);
        // }
        // cv::imshow("Hough Image",color_img);
        // cv::waitKey(0);
        // ROS_WARN("number of lines detected [%d]", (int)lines.size());
    }
};

int main(int argc, char** argv){
    ros::init(argc,argv,"line_detection");
    LineDetection line_detect_obj;
    ros::spin();
    return 0;
}