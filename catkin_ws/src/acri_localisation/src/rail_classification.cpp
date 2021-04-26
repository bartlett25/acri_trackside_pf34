#include <ros/ros.h>
#include <acri_localisation/railLine.h>
#include <acri_localisation/railLineVector.h>
#include <acri_localisation/railPair.h>
#include <acri_localisation/railPairVector.h>
#include <acri_localisation/railClosestPair.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <string>
#include <iostream>

typedef acri_localisation::railLine railLine;
typedef acri_localisation::railPair railPair;
typedef acri_localisation::railLineVector railLineVectorMsg;
typedef acri_localisation::railPairVector railPairVectorMsg;
typedef acri_localisation::railClosestPair railClosestPairMsg;
typedef std::vector<railLine> railLineVector;
typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayXb;
typedef geometry_msgs::Point Point;

class railClass{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_lines_;
    ros::Publisher pub_pairs_;
    ros::Publisher pub_close_;
    std::string sub_topic_;
    std::string pub_topic_lines_;
    std::string pub_topic_pairs_;
    std::string pub_topic_close_;
    std::string frame_id_;
    std::string child_frame_id_;
    tf::TransformListener listener_;
    double class_max_dist_square_;
    double class_max_theta_;
    double pair_min_length_square_;
    double pair_max_theta_;
    double pair_width_var_;
    double pair_length_var_;
    double pair_max_width_;
    double pair_max_length_diff_;
    double rail_width_;
    double range_;
    double cost_max_;
    int max_rail_number_;
    Eigen::Vector2f pose_;
    railLineVector line_segments_;
    railLineVector rail_lines_;
public:

    /*----------------------------------------------------------
     * railClass: 
     * initialises all parameters and ROS topics
     *---------------------------------------------------------*/

    railClass() : sub_topic_("input"), pub_topic_lines_("out_lines"), pub_topic_pairs_("out_pairs"), pub_topic_close_("out_closest"){
        sub_ = nh_.subscribe(sub_topic_,10,&railClass::sub_cb,this);
        pub_lines_ = nh_.advertise<railLineVectorMsg>(pub_topic_lines_,10);
        pub_pairs_ = nh_.advertise<railPairVectorMsg>(pub_topic_pairs_,10);
        pub_close_ = nh_.advertise<railClosestPairMsg>(pub_topic_close_,10);
        sub_topic_ = nh_.resolveName(sub_topic_);
        pub_topic_lines_ = nh_.resolveName(pub_topic_lines_);
        pub_topic_pairs_ = nh_.resolveName(pub_topic_pairs_);
        pub_topic_close_ = nh_.resolveName(pub_topic_close_);
        double class_max_dist;
        double pair_min_length;
        nh_.param<std::string>("/rail/rail_class/frame_id",frame_id_,"map");
        nh_.param<std::string>("/rail/rail_class/child_frame_id",child_frame_id_,"base_link");
        nh_.param<int>("/rail/rail_class/max_rail_number",max_rail_number_,200);
        nh_.param<double>("/rail/rail_class/class_max_dist",class_max_dist,0.5);
        nh_.param<double>("/ral/rail_class/class_max_theta",class_max_theta_,0.0873);
        nh_.param<double>("/rail/rail_class/pair_min_length",pair_min_length,5);
        nh_.param<double>("/rail/rail_class/pair_max_theta",pair_max_theta_,0.0873);
        nh_.param<double>("/rail/rail_class/pair_width_var",pair_width_var_,0.25);
        nh_.param<double>("/rail/rail_class/pair_length_var",pair_length_var_,4.0);
        nh_.param<double>("/rail/rail_class/pair_max_width",pair_max_width_,0.5);
        nh_.param<double>("/rail/rail_class/pair_max_length_diff",pair_max_length_diff_,5);
        nh_.param<double>("/rail/rail_class/rail_width",rail_width_,1.067); 
        nh_.param<double>("/rail/rail_class/range",range_,5);
        class_max_dist_square_ = class_max_dist*class_max_dist;
        pair_min_length_square_ = pair_min_length*pair_min_length;
        double pair_max_width_square = pair_max_width_*pair_max_width_;
        double pair_max_length_diff_square = pair_max_length_diff_*pair_max_length_diff_;
        double cost_max_width = 1 - exp(-2.0*pair_max_width_square/pair_width_var_);
        double cost_max_length = 1 - exp(-2.0*pair_max_length_diff_square/pair_length_var_);
        cost_max_ = cost_max_width + cost_max_length;
        pose_.setZero();
        ROS_INFO("rail class: subscribed topic [%s]",sub_topic_.c_str());
        ROS_INFO("rail class: publishing topic lines [%s]",pub_topic_lines_.c_str());
        ROS_INFO("rail class: publishing topic pairs [%s]",pub_topic_pairs_.c_str());
        ROS_INFO("rail class: publishing topic close [%s]",pub_topic_close_.c_str());

        // std::cout << "frame_id " << frame_id_ << std::endl;
        // std::cout << "child_frame_id " << child_frame_id_ << std::endl;
        // std::cout << "max_rail_number " << max_rail_number_ << std::endl;
        // std::cout << "class_max_dist " << class_max_dist << std::endl;
        // std::cout << "class_max_theta_ " << class_max_theta_ << std::endl;
        // std::cout << "pair_min_length " << pair_min_length << std::endl;
        // std::cout << "pair_max_theta " << pair_max_theta_ << std::endl;
        // std::cout << "pair_width_var " << pair_width_var_ << std::endl;
        // std::cout << "pair_length_var " << pair_length_var_ << std::endl;
        // std::cout << "pair_max_width " << pair_max_width_ << std::endl;
        // std::cout << "pair_max_length_diff " << pair_max_length_diff_ << std::endl;
        // std::cout << "rail_width " << rail_width_ << std::endl;
        // std::cout << "range " << range_ << std::endl;
    }

     /*----------------------------------------------------------
     * sub_cb: 
     * subscriber callback for rail classification.
     * Finds all rail lines, rail pairs, and closest rail pair.
     *---------------------------------------------------------*/

    void sub_cb(const railLineVectorMsg::ConstPtr msg){ 
        clusterLineSegments(msg);       
        pairRailLines(); 
        capRailNumber();  
    }

    /*----------------------------------------------------------
     * clusterLineSegments: 
     * finds and filters detected rail lines
     *---------------------------------------------------------*/

    void clusterLineSegments(const railLineVectorMsg::ConstPtr msg){

        // Append observed line segments to previous filtered rail lines

        const int num_msg = msg->lines.size();
        if(num_msg <= 0){            
            return;
        }
        rail_lines_.clear();
        for(auto ii = 0; ii < num_msg; ii++){
            line_segments_.push_back(msg->lines[ii]);    
        }

        // Get angle of each line segment

        const int num_segments = line_segments_.size();
        if(num_segments <= 0){
            return;
        }
        Eigen::VectorXf theta(num_segments);
        for(auto ii = 0; ii < num_segments; ii++){
            Point p1 = line_segments_[ii].point1;
            Point p2 = line_segments_[ii].point2;
            double dx = p1.x - p2.x;
            double dy = p1.y - p2.y;
            theta(ii) = std::atan2(dy,dx);
            if (theta(ii) < 0){
                theta(ii) = theta(ii) + M_PI;
            }
            if (theta(ii) >= M_PI){
                theta(ii) = theta(ii) - M_PI;
            }
        }

        // Cluster line segments due to direction
        // A line belongs to a subgroup if it has an angle < class_max_theta_ w.r.t all lines in the subgroup

        Eigen::VectorXi idx_theta(num_segments);
        idx_theta.setZero();
        idx_theta(0) = 1;
        Eigen::MatrixXf grouped_theta(num_segments,num_segments);
        grouped_theta.setZero();
        grouped_theta(0,0) = theta(0);
        std::vector<railLineVector> grouped_theta_segments;
        railLineVector first_theta_group;
        first_theta_group.push_back(line_segments_[0]);
        grouped_theta_segments.push_back(first_theta_group);    
        int num_theta_groups = 1;
        for(auto ii = 1; ii < num_segments; ii++){
            bool in_group = false;
            double segment_theta = theta(ii);
            for(auto jj = 0; jj < num_theta_groups; jj++){
                const int this_group = jj+1;
                const int num_group = (idx_theta.array() == this_group).count();
                Eigen::VectorXf this_theta_group = grouped_theta.block(0,jj,num_group,1);
                Eigen::VectorXf diff_theta = this_theta_group.array() - segment_theta;
                const int diff_theta_sum = (diff_theta.array().abs() < class_max_theta_).count(); 
                if (diff_theta_sum >= num_group){
                    in_group = true;
                    idx_theta(ii) = this_group;
                    grouped_theta(num_group,jj) = segment_theta;
                    grouped_theta_segments[jj].push_back(line_segments_[ii]);
                    break;
                }
            }
            if(!in_group){
                grouped_theta(0,num_theta_groups) = segment_theta;
                num_theta_groups++;
                idx_theta(ii) = num_theta_groups;
                railLineVector new_group;
                new_group.push_back(line_segments_[ii]);
                grouped_theta_segments.push_back(new_group);
            }
        }

        // Cluster lines further due to distance
        // A line belongs to a subgroup if it has a distance < class_max_dist_ w.r.t any line in the subgroup
       
        std::vector<railLineVector> grouped_segments;
        int num_lines = 0;
        for(auto ii = 0; ii < num_theta_groups; ii++){
            int this_theta_group = ii + 1;
            const int num_theta_group = (idx_theta.array() == this_theta_group).count();
            railLineVector theta_group = grouped_theta_segments[ii];
            std::vector<railLineVector> grouped_dist_segments;
            railLineVector first_group;
            first_group.push_back(theta_group[0]);
            grouped_dist_segments.push_back(first_group);     
            int num_dist = 1;
            for(int jj = 1; jj < num_theta_group; jj++){
                railLine line_dist = theta_group[jj];
                bool in_group = false;
                for(int kk = 0; kk < num_dist; kk++){
                    in_group = findDist(line_dist,grouped_dist_segments[kk]);
                    if(in_group){
                        grouped_dist_segments[kk].push_back(line_dist);
                        break;
                    }
                }
                if(!in_group){
                    num_dist++;  
                    railLineVector new_group;
                    new_group.push_back(line_dist);
                    grouped_dist_segments.push_back(new_group);
                }
            } 
            num_lines = num_lines + num_dist;
            for(auto jj = 0; jj < num_dist; jj++){
                grouped_segments.push_back(grouped_dist_segments[jj]);
            }
        }

        // Fit a rail line to each group of segments
        // Publish rail lines
         // Clear segment line history and set to current

        for(int ii = 0; ii < num_lines; ii++){
            polyFitWeighted(grouped_segments[ii]);
        }
        railLineVectorMsg rail_lines_msg;
        rail_lines_msg.header.frame_id = frame_id_;
        rail_lines_msg.header.stamp = ros::Time::now();
        rail_lines_msg.lines = rail_lines_;
        pub_lines_.publish(rail_lines_msg);
        line_segments_ = rail_lines_;

        // ROS_WARN("Number rail lines: [%d]", (int)rail_lines_.size());
    }

    /*----------------------------------------------------------
     * pairRailLines: 
     * finds rail line pairs contained in rail_lines_msg_
     * cost of pair culmination of two Rayleigh CDFs
     * performs 'greedy' optimisation over costs to find
     * SUB-OPTIMAL pairings (not guaranteed optimal)
     * order or greedy pairing based upon rail line length 
     * (biggest gets cost satisfied first)
     * finally, publishes any rail pairs (rail_pairs) and
     * closest pair (closest_pair)
     *---------------------------------------------------------*/

    void pairRailLines(){
        const int num_lines = rail_lines_.size();
        if (num_lines <= 0){
            return;
        }
        railLineVector valid_lines;
        Eigen::VectorXf all_lengths(num_lines);
        all_lengths.setZero();
        int num_valid = 0;
        for(int ii = 0; ii < num_lines; ii++){
            Point p1 = rail_lines_[ii].point1;
            Point p2 = rail_lines_[ii].point2;
            double dx = p1.x - p2.x;
            double dy = p1.y - p2.y;
            double length_square = dx*dx + dy*dy;
            if (length_square > pair_min_length_square_){
                valid_lines.push_back(rail_lines_[ii]);
                all_lengths(num_valid) = sqrt(length_square);
                num_valid++;
            }
        }
        Eigen::VectorXf valid_length = all_lengths.head(num_valid);
        if (num_valid <= 0){
            return;
        }

        // std::cout << "num valid " << num_valid << std::endl;

        // Find sub-optimal pairings via 'greedy' technique

        Eigen::MatrixXf cost(num_valid,num_valid);
        getCostMatrix(valid_lines,valid_length,cost); 
        std::vector<int> sort_idx(num_valid);
        std::iota(sort_idx.begin(),sort_idx.end(),0);
        std::stable_sort(sort_idx.begin(),sort_idx.end(),
            [&valid_length](size_t i1, size_t i2){return valid_length[i1] > valid_length[i2];});
        railPairVectorMsg rail_pairs;
        for(int ii = 0; ii < num_valid; ii++){
            int idx = sort_idx[ii];
            Eigen::VectorXf cost_column = cost.col(idx);
            Eigen::VectorXf::Index min_idx;
            double min_val = cost_column.minCoeff(&min_idx);
            if ((min_idx != idx) && (min_val <= cost_max_)){
                railPair this_pair;
                this_pair.line1 = valid_lines[idx];
                this_pair.line2 = valid_lines[min_idx];
                this_pair.midline = getMidLine(this_pair.line1,this_pair.line2); 
                rail_pairs.pairs.push_back(this_pair);   
                cost.col(min_idx) = std::numeric_limits<float>::infinity()*Eigen::VectorXf::Ones(num_valid);
            }
            cost.row(idx) = std::numeric_limits<float>::infinity()*Eigen::RowVectorXf::Ones(num_valid);
        }         
        rail_pairs.header.frame_id = frame_id_;
        rail_pairs.header.stamp = ros::Time::now();
        pub_pairs_.publish(rail_pairs);

        // std::cout << "Number rail pairs: " << rail_pairs.pairs.size() << std::endl;
        
        // Find the closest rail pair to vehicle
      
        const size_t num_pairs = rail_pairs.pairs.size();
        if(num_pairs <= 0){
            return;
        }
        tf::StampedTransform transform;
        try{
            listener_.waitForTransform(frame_id_,child_frame_id_,ros::Time(0),ros::Duration(1.0));
            listener_.lookupTransform(frame_id_,child_frame_id_,ros::Time(0),transform);
        }catch (tf::TransformException &ex){
            ROS_WARN("rail classifcation: [%s]",ex.what());
            return;
        }
        Point pose;
        pose.x = transform.getOrigin().x();
        pose.y = transform.getOrigin().y();
        double full_dist_min = std::numeric_limits<double>::infinity();
        double dist_min =  std::numeric_limits<double>::infinity();
        size_t closest_idx = 0;
        for(size_t ii = 0; ii < num_pairs; ii++){
            Point l1p1 = rail_pairs.pairs[ii].line1.point1;
            Point l1p2 = rail_pairs.pairs[ii].line1.point2;
            Point l2p1 = rail_pairs.pairs[ii].line2.point1;
            Point l2p2 = rail_pairs.pairs[ii].line2.point2;
            double dist1 = sqrt(getDistSq(l1p1,l1p2,pose));
            double dist2 = sqrt(getDistSq(l2p1,l2p2,pose));
            double this_full_dist = 0.5*(dist1 + dist2);
            if (this_full_dist < full_dist_min){
                full_dist_min = this_full_dist;
                dist_min = std::min(dist1,dist2);
                closest_idx = ii;
            }
        }   
        railClosestPairMsg closest_pair;
        closest_pair.header.frame_id = frame_id_;
        closest_pair.header.stamp = ros::Time::now();
        closest_pair.line1 = rail_pairs.pairs[closest_idx].line1;
        closest_pair.line2 = rail_pairs.pairs[closest_idx].line2;
        closest_pair.midline = rail_pairs.pairs[closest_idx].midline;    
        if(dist_min < range_){
            closest_pair.inrange.data = true; 
        }else{
            closest_pair.inrange.data = false;
        }
        pub_close_.publish(closest_pair);
        return;
    }

    /*----------------------------------------------------------
     * capRailNumber: 
     * caps the number of observed rail lines
     *---------------------------------------------------------*/

    void capRailNumber(){
        const int num_lines = (int)rail_lines_.size();
        if (num_lines > max_rail_number_){
            railLineVector::iterator it1, it2;
            it1 = rail_lines_.begin() + max_rail_number_;
            it2 = rail_lines_.end();
            rail_lines_.erase(it1,it2);
        }     
        return;
    }

    /*----------------------------------------------------------
     * getmid_line: 
     * calculates the mid line of a rail pair
     *---------------------------------------------------------*/

    railLine getMidLine(railLine line1, railLine line2){
        railLine mid_line;
        Eigen::Vector2f l1p1(line1.point1.x,line1.point1.y);
        Eigen::Vector2f l1p2(line1.point2.x,line1.point2.y);
        Eigen::Vector2f l2p1(line2.point1.x,line2.point1.y);
        Eigen::Vector2f l2p2(line2.point2.x,line2.point2.y);
        double dist1 = (l1p1-l2p1).norm() + (l1p2-l2p2).norm();
        double dist2 = (l1p1-l2p2).norm() + (l1p2-l2p1).norm();
        if (dist1 < dist2){
            Eigen::Vector2f point1 = 0.5*(l1p1 + l2p1);
            Eigen::Vector2f point2 = 0.5*(l1p2 + l2p2);
            mid_line.point1.x = point1(0);
            mid_line.point1.y = point1(1);
            mid_line.point1.z = 0.5*(line1.point1.z + line2.point1.z);
            mid_line.point2.x = point2(0);
            mid_line.point2.y = point2(1);
            mid_line.point2.z = 0.5*(line1.point2.z + line2.point2.z);
        }else{
            Eigen::Vector2f point1 = 0.5*(l1p1 + l2p2);
            Eigen::Vector2f point2 = 0.5*(l1p2 + l2p1);
            mid_line.point1.x = point1(0);
            mid_line.point1.y = point1(1);
            mid_line.point1.z = 0.5*(line1.point1.z + line2.point2.z);
            mid_line.point2.x = point2(0);
            mid_line.point2.y = point2(1);
            mid_line.point2.z = 0.5*(line1.point2.z + line2.point1.z);         
        }     
        return mid_line;
    }

    /*----------------------------------------------------------
     * polyFitWeighted: 
     * fits a line to a group of line segments
     * contribution of each segment is weighted by its length
     * once finished, published all fitted lines (rail_lines_msg_)
     *---------------------------------------------------------*/

    void polyFitWeighted(const railLineVector segments){
        const int num_lines = segments.size();
        if(num_lines == 1){
            rail_lines_.push_back(segments[0]);
            return;   
        }
        const int vect_size = 2*num_lines;
        Eigen::VectorXf x(vect_size);
        Eigen::VectorXf y(vect_size);
        Eigen::VectorXf z(vect_size);
        Eigen::VectorXf w(vect_size);
        bool xunique = true;
        bool yunique = true;
        for(int ii = 0; ii < num_lines; ii++){
            railLine this_segment = segments[ii];
            int idx = 2*ii;
            x(idx) = this_segment.point1.x;
            y(idx) = this_segment.point1.y;
            z(idx) = this_segment.point1.z;
            x(idx+1) = this_segment.point2.x;
            y(idx+1) = this_segment.point2.y;
            z(idx+1) = this_segment.point2.z;
            double dx = x(idx) - x(idx+1);
            double dy = y(idx) - y(idx+1);
            w(idx) = dx*dx + dy*dy;
            w(idx+1) = w(idx);
            if(dx != 0){
                xunique = false;
            }
            if(dy != 0){
                yunique = false;
            }
        } 
        double wnorm_inv = 1.0/w.norm();
        Eigen::MatrixXf wmat = (wnorm_inv*w).asDiagonal();

        // Test uniqueness of data points

        railLine new_line;
        if(xunique){
            Eigen::VectorXf::Index min_idx;
            Eigen::VectorXf::Index maxIdx;
            new_line.point1.x = x(0);
            new_line.point1.y = y.minCoeff(&min_idx);
            new_line.point1.z = z(min_idx);
            new_line.point2.x = x(0);
            new_line.point2.y = y.maxCoeff(&maxIdx);
            new_line.point2.z = z(maxIdx);
            rail_lines_.push_back(new_line);
            return;
        }
        if(yunique){
            Eigen::VectorXf::Index min_idx;
            Eigen::VectorXf::Index maxIdx;
            new_line.point1.x = x.minCoeff(&min_idx);
            new_line.point1.y = y(0);
            new_line.point1.z = z(min_idx);
            new_line.point2.x = x.maxCoeff(&maxIdx);
            new_line.point2.y = y(0);
            new_line.point2.z = z(maxIdx);
            rail_lines_.push_back(new_line);
            return;
        }

        // Perform least squares method w.r.t x

        double rx;
        bool xflag = false;
        Eigen::MatrixXf xmat(vect_size,2);
        xmat.col(0).setOnes();
        xmat.col(1) = x;
        Eigen::MatrixXf xspd = xmat.transpose()*wmat*xmat;
        Eigen::Vector2f kx;
        kx.setZero();
        try{
            kx = xspd.inverse()*xmat.transpose()*wmat*y;
            Eigen::VectorXf diff = y - xmat*kx;
            rx = diff.squaredNorm();
        }catch(const std::exception& e){
            xflag = true;
            rx = std::numeric_limits<double>::infinity();
        }

        // Perform least squares method w.r.t y (account for vertical lines)

        double ry;
        bool yflag = false;
        Eigen::MatrixXf ymat(vect_size,2);
        ymat.col(0).setOnes();
        ymat.col(1) = y;
        Eigen::MatrixXf yspd = ymat.transpose()*wmat*ymat;
        Eigen::Vector2f ky;
        ky.setZero();
        try{
            ky = yspd.inverse()*ymat.transpose()*wmat*x;
            Eigen::VectorXf diff = x - ymat*ky;
            ry = diff.squaredNorm();
        }catch(const std::exception& e){
            yflag = true;
            ry = std::numeric_limits<double>::infinity();
        }

        // Fit line based upon which method has smallest residual error

        if(xflag && yflag){
            ROS_WARN_STREAM("rail classifcation: Line could not be fitted to the data");
            return;
        }
        if(rx < ry){
            Eigen::VectorXf::Index min_idx;
            Eigen::VectorXf::Index maxIdx;
            double xmin = x.minCoeff(&min_idx);
            double xmax = x.maxCoeff(&maxIdx);
            double ymin = kx(0) + xmin*kx(1);
            double ymax = kx(0) + xmax*kx(1);
            new_line.point1.x = xmin;
            new_line.point1.y = ymin;
            new_line.point1.z = z(min_idx);
            new_line.point2.x = xmax;
            new_line.point2.y = ymax;
            new_line.point2.z = z(maxIdx);
        }else{
            Eigen::VectorXf::Index min_idx;
            Eigen::VectorXf::Index maxIdx;
            double ymin = y.minCoeff(&min_idx);
            double ymax = y.maxCoeff(&maxIdx);
            double xmin = ky(0) + ymin*ky(1);
            double xmax = ky(0) + ymax*ky(1);
            new_line.point1.x = xmin;
            new_line.point1.y = ymin;
            new_line.point1.z = z(min_idx);
            new_line.point2.x = xmax;
            new_line.point2.y = ymax;  
            new_line.point2.z = z(maxIdx);  
        }
        rail_lines_.push_back(new_line);
    }

    /*----------------------------------------------------------
     * getCostMatrix: 
     * computes the cost between each possible rail pairing
     * cost of pair culmination of two Rayleigh CDFs
     *---------------------------------------------------------*/

    void getCostMatrix(const railLineVector& lines, const Eigen::VectorXf& length, Eigen::MatrixXf& cost){
        const int num_lines = lines.size();

        // get angle of all valid lines

        Eigen::VectorXf theta(num_lines);
        for(int ii = 0; ii < num_lines; ii++){
            Point p1 = lines[ii].point1;
            Point p2 = lines[ii].point2;
            double dx = p1.x - p2.x;
            double dy = p1.y - p2.y;
            theta(ii) = std::atan2(dy,dx);
            if (theta(ii) < 0){
                theta(ii) = theta(ii) + M_PI;
            }
            if (theta(ii) >= M_PI){
                theta(ii) = theta(ii) - M_PI;
            }
        }

        // std::cout << "theta:\n" << theta << std::endl;

        // obtain costs of pairing
        // impossible pairings are set with Inf cost

        cost.setZero();
        for(int ii = 0; ii < num_lines; ii++){
            Point p1ii = lines[ii].point1;
            Point p2ii = lines[ii].point2;
            double lengthii = length(ii);
            for(int jj = 0; jj < num_lines; jj++){
                if(ii == jj){
                    cost(ii,jj) = cost_max_;
                }else{
                    double theta_diff = std::abs(theta(ii) - theta(jj));
                    if (theta_diff < pair_max_theta_){
                        Point p1jj = lines[jj].point1;
                        Point p2jj = lines[jj].point2;
                        double average_width = getAverageWidth(p1ii,p2ii,p1jj,p2jj);

                        // std::cout << "Average width " << average_width << std::endl;

                        double diff_width = std::abs(average_width - rail_width_);
                        if(diff_width > pair_max_width_){
                            cost(ii,jj) = std::numeric_limits<float>::infinity();
                        }else{
                            double lengthjj = length(jj);
                            double diff_length = std::abs(lengthii - lengthjj);
                            if(diff_length > pair_max_length_diff_){
                                cost(ii,jj) = std::numeric_limits<float>::infinity();
                            }else{
                                double diff_width_square = diff_width*diff_width;
                                double diff_length_square = diff_length*diff_length;
                                double cost_width = 1 - exp(-2.0*diff_width_square/pair_width_var_);
                                double cost_length = 1 - exp(-2.0*diff_length_square/pair_length_var_);
                                cost(ii,jj) = cost_width + cost_length;
                            }
                        }
                    }else{
                        cost(ii,jj) = std::numeric_limits<float>::infinity();    
                    }
                }
            }
        }
        return;
    }

    /*----------------------------------------------------------
     * findDist: 
     * finds the minimum distance between line segment and
     * group of line segments.
     * informs whether line segments should be in group
     *---------------------------------------------------------*/

    bool findDist(const railLine line, const railLineVector group){
        Point lp1 = line.point1;
        Point lp2 = line.point2;
        const int num = group.size();
        for(int ii = 0; ii < num; ii++){
            Point gp1 = group[ii].point1;
            Point gp2 = group[ii].point2;

            // Test orientation cases to see if line segments intersect

            int o1 = getOrientation(lp1,lp2,gp1);
            int o2 = getOrientation(lp1,lp2,gp2);
            int o3 = getOrientation(gp1,gp2,lp1);
            int o4 = getOrientation(gp1,gp2,lp2);
            if ((o1 != o2) && (o3 != o4)){
                return true;
            }
            if ((o1 == 0) && (onSegment(lp1,lp2,gp1))){
                return true;
            }
            if ((o2 == 0) && (onSegment(lp1,lp2,gp2))){
                return true;
            }
            if ((o3 == 0) && (onSegment(gp1,gp2,lp1))){
                return true;
            }
            if ((o4 == 0) && (onSegment(gp1,gp2,lp2))){
                return true;
            }

            // Test distance between end-points and alternate line segment

            double dist_sqaure;
            dist_sqaure = getDistSq(lp1,lp2,gp1);
            if (dist_sqaure < class_max_dist_square_){
                return true;
            }
            dist_sqaure = getDistSq(lp1,lp2,gp2);
            if (dist_sqaure < class_max_dist_square_){
                return true;
            }
            dist_sqaure = getDistSq(gp1,gp2,lp1);
            if (dist_sqaure < class_max_dist_square_){
                return true;
            }
            dist_sqaure = getDistSq(gp1,gp2,lp2);
            if (dist_sqaure < class_max_dist_square_){
                return true;
            }
        } 
        return false;
    }

    /*----------------------------------------------------------
     * getOrientation: 
     * finds the orientation of ordered points p1,p2,p3
     * 0: collinear, 1: clockwise, 2: anti-clockwise
     *---------------------------------------------------------*/

    int getOrientation(const Point p1, const Point p2, const Point p3){
        double val = (p2.y-p1.y)*(p3.x-p2.x) - (p2.x-p1.x)*(p3.y-p2.y);
        if(val == 0){
            return 0;
        }
        if(val > 0){
            return 1;
        }
        return 2;
    }

     /*----------------------------------------------------------
     * onSegment: 
     * reports if point p lies on line segment vw
     *---------------------------------------------------------*/

    bool onSegment(const Point v, const Point w, const Point p){
        double xmax = std::max(v.x,w.x);
        double xmin = std::min(v.x,w.x);
        double ymax = std::max(v.y,w.y);
        double ymin = std::min(v.y,w.y);
        if((p.x <= xmax) && (p.x >= xmin) && (p.y <= ymax) && (p.y >= ymin)){
            return true;
        }
        return false;
    }

     /*----------------------------------------------------------
     * getDistSq: 
     * gets the minimum distance squared between point p and
     * line segment vw
     *---------------------------------------------------------*/

    double getDistSq(const Point v, const Point w, const Point p){
        Eigen::Vector2f v_eig(v.x,v.y);
        Eigen::Vector2f w_eig(w.x,w.y);
        Eigen::Vector2f p_eig(p.x,p.y);
        Eigen::Vector2f vw = w_eig - v_eig;
        Eigen::Vector2f vp = p_eig - v_eig;
        double vw_length_square = vw.squaredNorm();
        if(vw_length_square == 0){
            return vp.squaredNorm();
        }
        double scale = (double)vp.dot(vw)/vw_length_square;
        double t = std::max(0.0,std::min(1.0,scale));
        Eigen::Vector2f projection = v_eig + t*vw;
        Eigen::Vector2f projection_to_p = p_eig - projection;
        return projection_to_p.squaredNorm();
    }

    /*----------------------------------------------------------
     * getAverageWidth:
     * finds the average width between line segments vw 
     * and pq
     *---------------------------------------------------------*/  

    double getAverageWidth(Point v, Point w, Point p, Point q){
        bool lines_cross = getLineCross(v,w,p,q);
        if(lines_cross){
            double distSq1 = getWidthSq(v,w,p);
            double distSq2 = getWidthSq(v,w,q);
            double distSq3 = getWidthSq(p,q,v);
            double distSq4 = getWidthSq(p,q,w);
            double dist = 0.25*(sqrt(distSq1) + sqrt(distSq2) + sqrt(distSq3) + sqrt(distSq4));
            return dist;
        }
        double dist = rail_width_ + pair_max_width_ +  10.0;
        return dist;
    }

    /*----------------------------------------------------------
     * getLineCross: 
     * determine if lines segments vw and pq have same x or y value
     *---------------------------------------------------------*/

    bool getLineCross(Point v,Point w, Point p, Point q){
        Eigen::Vector2f v_eig(v.x,v.y);
        Eigen::Vector2f w_eig(w.x,w.y);
        Eigen::Vector2f p_eig(p.x,p.y);
        Eigen::Vector2f vw = w_eig - v_eig;
        Eigen::Vector2f vp = p_eig - v_eig;
        double vw_length_square = vw.squaredNorm();
        if(vw_length_square == 0){
            return false;
        }
        double t = (double)vp.dot(vw)/vw_length_square;
        if(t >= 0 && t <= 1){
            return true;
        }
        Eigen::Vector2f q_eig(q.x,q.y);
        Eigen::Vector2f vq = q_eig - v_eig;
        t = (double)vq.dot(vw)/vw_length_square;
        if(t >= 0 && t <= 1){
            return true;
        }
        return false;
    }

     /*----------------------------------------------------------
     * getWidthSq: 
     * gets the distance squared between point p and infinte
     * line containing points v and w
     *---------------------------------------------------------*/

    double getWidthSq(Point v,Point w, Point p){
        Eigen::Vector2f v_eig(v.x,v.y);
        Eigen::Vector2f w_eig(w.x,w.y);
        Eigen::Vector2f p_eig(p.x,p.y);
        Eigen::Vector2f vw = w_eig - v_eig;
        Eigen::Vector2f vp = p_eig - v_eig;
        double vw_length_square = vw.squaredNorm();
        if(vw_length_square == 0){
            return vp.squaredNorm();
        }
        double t = (double)vp.dot(vw)/vw_length_square;
        Eigen::Vector2f projection = v_eig + t*vw;
        Eigen::Vector2f projection_to_p = p_eig - projection;
        return projection_to_p.squaredNorm();
    }
};

int main( int argc, char** argv ){
  ros::init(argc, argv, "rail_class");
  railClass obj;
  ros::spin();
  return 0;
}