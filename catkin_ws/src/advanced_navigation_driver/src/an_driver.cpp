/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*        ROS Driver, Packet to Published Message Example       */
/*          Copyright 2017, Advanced Navigation Pty Ltd         */
/*                                                              */
/****************************************************************/
/*
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/*
2018.02.09  Kyler Laird
added RTCM handling, configuration parameters

2018.03.10  Kyler Laird
added REP compliance, transform
*/

/*
2020.10.23 Nathan Bartlett
added transform modifcations to better link with RealSense nodes
ensured all coordinate frames are of the appropriate REP103 type with the expection
of world_frame and imu_frame, which are NED and FRD respectively.
*/

#include <ros/ros.h>
#include <ros/serialization.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <cmath>								//2010-08-11: was <math.h>	  
#include <cstdio>								//2010-08-11: was <stdio.h>	 

/* LatLong-UTM.c++
Conversions:  LatLong to UTM;  and UTM to LatLong;
by Eugene Reimer, ereimer@shaw.ca, 2002-December;
with LLtoUTM & UTMtoLL routines based on those by Chuck Gantz chuck.gantz@globalstar.com;
with ellipsoid & datum constants from Peter H Dana website (http://www.colorado.edu/geography/gcraft/notes/datum/edlist.html);
Usage:  see the Usage() routine below;
Copyright © 1995,2002,2010 Eugene Reimer, Peter Dana, Chuck Gantz.  Released under the GPL;  see http://www.gnu.org/licenses/gpl.html
(Peter Dana's non-commercial clause precludes using the LGPL)
*/ 

#define RADIANS_TO_DEGREES (180.0/M_PI)
#define fr 298.257223563
#define a 6378137
#define k0 0.9996
const double PI = 4*atan(1);				// Gantz used: PI=3.14159265;
const double deg2rad  = PI/180;
const double ee = 2/fr-1/(fr*fr);
const double EE = ee/(1-ee);
double LongOriginRad;

int an_packet_transmit(an_packet_t *an_packet){
	an_packet_encode(an_packet);
	return SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

void set_filter_options(){
	an_packet_t *an_packet;
	filter_options_packet_t filter_options_packet;

	// initialise the structure by setting all the fields to zero 

	memset(&filter_options_packet, 0, sizeof(filter_options_packet_t));
	filter_options_packet.permanent = TRUE;
	filter_options_packet.vehicle_type = vehicle_type_car;
	filter_options_packet.internal_gnss_enabled = TRUE;
	filter_options_packet.atmospheric_altitude_enabled = TRUE;
	filter_options_packet.velocity_heading_enabled = TRUE;
	filter_options_packet.reversing_detection_enabled = TRUE;
	filter_options_packet.motion_analysis_enabled = TRUE;

	an_packet = encode_filter_options_packet(&filter_options_packet);
	an_packet_transmit(an_packet);
	an_packet_free(&an_packet);
}

void set_filter_options_x(){
	// an_packet_t *an_packet = an_packet_allocate(17, 186);

	an_packet_t *an_packet = an_packet_allocate(4, 55);
	memcpy(&an_packet->data[0], "test", 5 * sizeof(uint8_t));
        an_packet_transmit(an_packet);
        an_packet_free(&an_packet);
}

void handle_rtcm(const std_msgs::String::ConstPtr& msg){
	const char *rtcm_data;
	uint32_t string_length = msg->data.length();

	// ROS_INFO("RTCM: %d bytes",  string_length);
	
	an_packet_t *an_packet = an_packet_allocate(string_length, packet_id_rtcm_corrections);
	memcpy(&an_packet->data[0], &msg->data[0], string_length);
	an_packet_transmit(an_packet);
	an_packet_free(&an_packet);
}

void LLtoUTM(int Zone, double LatRad, double LongRad,  double& Northing, double& Easting){
	// converts LatLong to UTM coords;  3/22/95: by ChuckGantz chuck.gantz@globalstar.com, from USGS Bulletin 1532.

	double N, T, C, A, M;
	N = a/sqrt(1-ee*sin(LatRad)*sin(LatRad));
	T = tan(LatRad)*tan(LatRad);
	C = EE*cos(LatRad)*cos(LatRad);
	A = cos(LatRad)*(LongRad-LongOriginRad);
	M= a*((1 - ee/4    - 3*ee*ee/64 - 5*ee*ee*ee/256  ) *LatRad 
	    - (3*ee/8 + 3*ee*ee/32 + 45*ee*ee*ee/1024) *sin(2*LatRad)
	    + (15*ee*ee/256 + 45*ee*ee*ee/1024	  ) *sin(4*LatRad)
	    - (35*ee*ee*ee/3072			  ) *sin(6*LatRad));
	Easting = k0*N*(A+(1-T+C)*A*A*A/6+(5-18*T+T*T+72*C-58*EE)*A*A*A*A*A/120) + 500000.0;
	Northing = k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
			    + (61-58*T+T*T+600*C-330*EE)*A*A*A*A*A*A/720));
}

//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
//-------------------------------------------main-----------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "an_device");
	ros::NodeHandle nh("~");
	std::string com_port_s;
	nh.param<std::string>("port", com_port_s, "/dev/ttyUSB0");
	char *com_port = (char *)com_port_s.c_str();
	int baud_rate;
	nh.param<int>("baud", baud_rate, 115200);
	if (OpenComport(com_port, baud_rate)){
		ROS_INFO("Could not open serial port %s at %d baud.", com_port, baud_rate);
		exit(EXIT_FAILURE);
	}
	ROS_INFO("port:%s@%d", com_port, baud_rate);
	
	// Obtain UTM zone and reference frames.
	// Note: the UTM zone is static to avoid problems due to changing when near a zone boundary.

	std::string world_frame;
	std::string map_frame;
	std::string odom_frame;
	std::string base_link_frame;
	std::string imu_link_frame;
	nh.param<std::string>("world_frame",world_frame,"world_ned");
	nh.param<std::string>("map_frame",map_frame,"map");
	nh.param<std::string>("odom_frame",odom_frame,"odom");
	nh.param<std::string>("base_link_frame",base_link_frame,"base_link");
	nh.param<std::string>("imu_link_frame",imu_link_frame,"imu_link");
	static int utm_zone;
	if (nh.getParam("utm_zone",utm_zone)){
		LongOriginRad = (utm_zone*6 - 183) * deg2rad;
		ROS_INFO("using UTM Zone [%d] to publish static transform [%s]", utm_zone, map_frame.c_str());
	}else{
		utm_zone = 0;
	}

	// Initialise publishers and subscribers

	static double frequency;
	nh.param<double>("frequency",frequency,10.0);
	ros::Publisher nav_sat_fix_pub = nh.advertise<sensor_msgs::NavSatFix>("navSatFix",10);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("twist",10);
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu",10);
	ros::Publisher system_status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("systemStatus",10);
	ros::Publisher filter_status_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("filterStatus",10);
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",frequency);
	ros::Subscriber rtcm_sub = nh.subscribe("rtcm", 1000, handle_rtcm);

	// Initialise messages

	static int gps_service;
	nh.param<int>("gps_service",gps_service,1);
	sensor_msgs::NavSatFix nav_sat_fix_msg;
	nav_sat_fix_msg.header.stamp.sec = 0;
	nav_sat_fix_msg.header.stamp.nsec = 0;
	nav_sat_fix_msg.header.frame_id = imu_link_frame;								// MIGHT BE IMU_LINK to keep in (N.B) ??
	nav_sat_fix_msg.status.status = -1; 											// initialise with unable to fix position
	nav_sat_fix_msg.status.service = gps_service; 					
	nav_sat_fix_msg.latitude = 0.0;
	nav_sat_fix_msg.longitude = 0.0;
	nav_sat_fix_msg.altitude = 0.0;
	nav_sat_fix_msg.position_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; 	// east,north,up
	nav_sat_fix_msg.position_covariance_type = 2; 									// fixed to variance on the diagonal

	nav_msgs::Odometry odom_msg;
	odom_msg.header.frame_id = map_frame; 
	odom_msg.child_frame_id = base_link_frame;

	geometry_msgs::Twist twist_msg;
	twist_msg.linear.x = 0.0;
	twist_msg.linear.y = 0.0;
	twist_msg.linear.z = 0.0;
	twist_msg.angular.x = 0.0;
	twist_msg.angular.y = 0.0;
	twist_msg.angular.z = 0.0;

	sensor_msgs::Imu imu_msg;
	imu_msg.header.stamp.sec = 0;
	imu_msg.header.stamp.nsec = 0;
	imu_msg.header.frame_id = imu_link_frame;
	imu_msg.orientation.x = 0.0;
	imu_msg.orientation.y = 0.0;
	imu_msg.orientation.z = 0.0;
	imu_msg.orientation.w = 0.0;
	imu_msg.orientation_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	imu_msg.angular_velocity.x = 0.0;
	imu_msg.angular_velocity.y = 0.0;
	imu_msg.angular_velocity.z = 0.0;
	imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; 	// fixed
	imu_msg.linear_acceleration.x = 0.0;
	imu_msg.linear_acceleration.y = 0.0;
	imu_msg.linear_acceleration.z = 0.0;
	imu_msg.linear_acceleration_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // fixed
	
	diagnostic_msgs::DiagnosticStatus system_status_msg;
	system_status_msg.level = 0; // default OK state
	system_status_msg.name = "System Status";
	system_status_msg.message = "";
	
	diagnostic_msgs::DiagnosticStatus filter_status_msg;
	filter_status_msg.level = 0; // default OK state
	filter_status_msg.name = "Filter Status";
	filter_status_msg.message = "";
	
	//---------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------
	//--------------------------------------------Obtain the first GPS location--------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------

	// Get data from com port

	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	system_state_packet_t system_state_packet;
	velocity_standard_deviation_packet_t velocity_standard_deviation_packet;
	quaternion_orientation_standard_deviation_packet_t quaternion_orientation_standard_deviation_packet;
	int bytes_received;
	an_decoder_initialise(&an_decoder);

	// Obtain initial GPS position after fix and place in static transform

	bool no_fix = true;
	double wait_for_lock;
	nh.param<double>("wait_for_lock",wait_for_lock,10.0);	
	ros::Rate rate(frequency);
	uint32_t time_start = ros::Time::now().sec;
	double time_for_fix = 0.0;
	double time_change = 1.0/frequency;	
	geometry_msgs::TransformStamped map_transformStamped;	
	while (ros::ok() && time_for_fix < wait_for_lock && no_fix){
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0){
			an_decoder_increment(&an_decoder, bytes_received);		
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL){
				if (an_packet->id == 0){
					ROS_INFO("acknowledgement data: %d", an_packet->data[3]);
				}
				if (an_packet->id == 69){
					ROS_INFO("receiver information: %d", an_packet->data[0]);
				}
				if (an_packet->id == packet_id_system_state) {
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0){
						nav_sat_fix_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						nav_sat_fix_msg.header.stamp.nsec = system_state_packet.microseconds*1000;
						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||  // 2D
							(system_state_packet.filter_status.b.gnss_fix_type == 2)){	 // 3D
							nav_sat_fix_msg.status.status = 0; 	// Unaugmented fix
						}else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||  	// SBAS
							(system_state_packet.filter_status.b.gnss_fix_type == 5)){   		// Omnistar/Starfire
							nav_sat_fix_msg.status.status = 1; 	// Satellite-based augmentation
						}else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||  	// differential
							(system_state_packet.filter_status.b.gnss_fix_type == 6) ||  		// RTK float
							(system_state_packet.filter_status.b.gnss_fix_type == 7)){   		// RTK fixed
							nav_sat_fix_msg.status.status = 2; 	// Ground-based augmentation
						}else{
							nav_sat_fix_msg.status.status = -1;	// Unable to fix position
						}
						if (utm_zone){
							double N, E;
							LLtoUTM(utm_zone,system_state_packet.latitude,system_state_packet.longitude,N, E);		
							map_transformStamped.header.stamp = ros::Time(system_state_packet.unix_time_seconds,
									system_state_packet.microseconds*1000);
							map_transformStamped.transform.translation.x = N;
							map_transformStamped.transform.translation.y = E;
							map_transformStamped.transform.translation.z = system_state_packet.height;
						}
						if (nav_sat_fix_msg.status.status >= 0){
							no_fix = false;
						}
					}
				}                          
				an_packet_free(&an_packet);
			}
		}
		rate.sleep();
		ros::spinOnce();
		time_for_fix += time_change;			
	}
	const float std_value = 1e03;
	if (no_fix){
		ROS_ERROR(
			"Spatial: could not obtain a GPS lock after [%f] seconds. Pose deviation will be set as [%f]",
			wait_for_lock,std_value);
	}else{
		ROS_INFO(
			"Spatial: GPS lock obtained after [%f] seconds.", 
			time_for_fix);
	}

	// Create necessary transfroms.

	tf2_ros::StaticTransformBroadcaster map_broadcaster;
	tf::Quaternion world_to_map_quat;
	world_to_map_quat.setRPY(PI,0.0,PI/2.0);
	map_transformStamped.header.frame_id = world_frame;
	map_transformStamped.child_frame_id = map_frame;	
	map_transformStamped.transform.rotation.x = world_to_map_quat.x();
	map_transformStamped.transform.rotation.y = world_to_map_quat.y();
	map_transformStamped.transform.rotation.z = world_to_map_quat.z();
	map_transformStamped.transform.rotation.w = world_to_map_quat.w();	
	map_broadcaster.sendTransform(map_transformStamped);

	tf::StampedTransform transform;
	tf::TransformListener listener;
	ros::Time init_time = map_transformStamped.header.stamp;
	bool node_not_loaded = true;
	while(node_not_loaded){
		bool found = true;
		try{
			listener.waitForTransform(odom_frame,map_frame,ros::Time(0),ros::Duration(1.0));
			listener.lookupTransform(odom_frame,map_frame,ros::Time(0),transform);
			listener.waitForTransform(base_link_frame,odom_frame,ros::Time(0),ros::Duration(1.0));
			listener.lookupTransform(base_link_frame,odom_frame,ros::Time(0),transform);
			ROS_INFO_STREAM("Spatial: all transforms found");
		}catch(tf::TransformException &ex){
			found = false;
			ROS_INFO_STREAM("Spatial: waiting for all neccessary transforms...");		
		}
		node_not_loaded = !found;
	}

	tf2_ros::StaticTransformBroadcaster imu_link_broadcaster;
	geometry_msgs::TransformStamped imu_link_transformStamped;
	tf::Quaternion base_link_to_imu_link_quat;
	base_link_to_imu_link_quat.setRPY(PI,0.0,PI/2.0);
	imu_link_transformStamped.header.frame_id = base_link_frame;
	imu_link_transformStamped.child_frame_id = imu_link_frame;	
	imu_link_transformStamped.header.stamp = map_transformStamped.header.stamp;
	imu_link_transformStamped.transform.translation.x = 0.0;
	imu_link_transformStamped.transform.translation.y = 0.0;
	imu_link_transformStamped.transform.translation.z = 0.0;
	imu_link_transformStamped.transform.rotation.x = base_link_to_imu_link_quat.x();
	imu_link_transformStamped.transform.rotation.y = base_link_to_imu_link_quat.y();
	imu_link_transformStamped.transform.rotation.z = base_link_to_imu_link_quat.z();
	imu_link_transformStamped.transform.rotation.w = base_link_to_imu_link_quat.w();	
	imu_link_broadcaster.sendTransform(imu_link_transformStamped);

	//---------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------
	//-------------------------------------------Obtain the current GPS location-------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------
	//---------------------------------------------------------------------------------------------------------------------------

	while (ros::ok()){
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0){
			an_decoder_increment(&an_decoder, bytes_received);
			bool odom_vel_set = false;
			bool imu_vel_set = false;
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL){
				if (an_packet->id == 0){
					ROS_INFO("acknowledgement data: %d", an_packet->data[3]);
				}
				if (an_packet->id == 69) {
					ROS_INFO("receiver information: %d", an_packet->data[0]);
				}
				if (an_packet->id == packet_id_system_state){
					if(decode_system_state_packet(&system_state_packet, an_packet) == 0){	
						// NavSatFix

						nav_sat_fix_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						nav_sat_fix_msg.header.stamp.nsec = system_state_packet.microseconds*1000;
						if ((system_state_packet.filter_status.b.gnss_fix_type == 1) ||  		// 2D
							(system_state_packet.filter_status.b.gnss_fix_type == 2)){	 		// 3D
							nav_sat_fix_msg.status.status = 0; 									// Unaugmented fix
						}else if ((system_state_packet.filter_status.b.gnss_fix_type == 3) ||  	// SBAS
							(system_state_packet.filter_status.b.gnss_fix_type == 5)){   		// Omnistar/Starfire
							nav_sat_fix_msg.status.status = 1; 									// Satellite-based augmentation
						}else if ((system_state_packet.filter_status.b.gnss_fix_type == 4) ||  	// differential
							(system_state_packet.filter_status.b.gnss_fix_type == 6) ||  		// RTK float
							(system_state_packet.filter_status.b.gnss_fix_type == 7)){   		// RTK fixed
							nav_sat_fix_msg.status.status = 2; 									// Ground-based augmentation
						}else{
							nav_sat_fix_msg.status.status = -1;									// Unable to fix position
						}
						nav_sat_fix_msg.latitude = system_state_packet.latitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.longitude = system_state_packet.longitude * RADIANS_TO_DEGREES;
						nav_sat_fix_msg.altitude = system_state_packet.height;
						if (nav_sat_fix_msg.status.status >= 0 && !no_fix){
							nav_sat_fix_msg.position_covariance = {pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
								0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
								0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};
						}else{
							nav_sat_fix_msg.position_covariance = {pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
								0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
								0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};
						}

						// Twist in NED coordinates

						twist_msg.linear.x = system_state_packet.velocity[0];
						twist_msg.linear.y = system_state_packet.velocity[1];
						twist_msg.linear.z = system_state_packet.velocity[2];
						twist_msg.angular.x = system_state_packet.angular_velocity[0];
						twist_msg.angular.y = system_state_packet.angular_velocity[1];
						twist_msg.angular.z = system_state_packet.angular_velocity[2];
						
						// IMU in FRD coordinates

						imu_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
						imu_msg.header.stamp.nsec = system_state_packet.microseconds*1000;
						tf::Quaternion orientation;
						orientation.setRPY(
							system_state_packet.orientation[0],
							system_state_packet.orientation[1],
							system_state_packet.orientation[2] // In FRD coordinates
						);
						imu_msg.orientation.x = orientation[0];
						imu_msg.orientation.y = orientation[1];
						imu_msg.orientation.z = orientation[2];
						imu_msg.orientation.w = orientation[3];
						imu_msg.angular_velocity.x = system_state_packet.angular_velocity[0]; // In FRD coordinates
						imu_msg.angular_velocity.y = system_state_packet.angular_velocity[1];
						imu_msg.angular_velocity.z = system_state_packet.angular_velocity[2];
						imu_msg.linear_acceleration.x = system_state_packet.body_acceleration[0];
						imu_msg.linear_acceleration.y = system_state_packet.body_acceleration[1];
						imu_msg.linear_acceleration.z = system_state_packet.body_acceleration[2];

						// Get GPS pose into the map_frame (ENU) and place into the published odometry message

						if (utm_zone){
							double N, E;
							LLtoUTM(
								utm_zone,
								system_state_packet.latitude,
								system_state_packet.longitude,
								N, E
							);
							geometry_msgs::Pose gps_map;
							gps_map.position.x = E - map_transformStamped.transform.translation.y; 
							gps_map.position.y = N - map_transformStamped.transform.translation.x;
							gps_map.position.z = system_state_packet.height - map_transformStamped.transform.translation.z;
							gps_map.orientation = imu_msg.orientation;				
							odom_msg.header.stamp.sec = system_state_packet.unix_time_seconds;
							odom_msg.header.stamp.nsec = system_state_packet.microseconds*1000;
							odom_msg.pose.pose = gps_map;
							if (!no_fix){
								
								odom_msg.pose.covariance = {pow(system_state_packet.standard_deviation[1],2), 0.0, 0.0,
									0.0, pow(system_state_packet.standard_deviation[0],2), 0.0,
									0.0, 0.0, pow(system_state_packet.standard_deviation[2],2)};
							}else{		
								odom_msg.pose.covariance = {pow(std_value,2), 0.0, 0.0,
									0.0, pow(std_value,2), 0.0,
									0.0, 0.0, pow(std_value,2)};
							}

							// Set angular velocity in the base_link frame (ENU)

							odom_msg.twist.twist.angular.x = system_state_packet.angular_velocity[1];
							odom_msg.twist.twist.angular.y = system_state_packet.angular_velocity[0];
							odom_msg.twist.twist.angular.z = -system_state_packet.angular_velocity[2];

							// Set linear velocities in the base_link frame (ENU). Have to transfer from (NED) version of map_frame

							// geometry_msgs::PointStamped velocity_map;
							// geometry_msgs::PointStamped velocity_base_link;
							// velocity_map.header.frame_id = map_frame;
							// velocity_map.header.stamp = odom_msg.header.stamp;
							// velocity_map.point.x = system_state_packet.velocity[1];
							// velocity_map.point.y = system_state_packet.velocity[0];
							// velocity_map.point.z = -system_state_packet.velocity[2];
							// try{
							// 	listener.waitForTransform(base_link_frame,map_frame,ros::Time(0),ros::Duration(1.0));
							// 	listener.transformPoint(base_link_frame,velocity_map,velocity_base_link);
							// 	odom_msg.twist.twist.linear.x = velocity_base_link.point.x;
							// 	odom_msg.twist.twist.linear.y = velocity_base_link.point.y;
							// 	odom_msg.twist.twist.linear.z = velocity_base_link.point.z;
							// }catch(tf::TransformException &ex){
							// 	ROS_WARN("Spatial: [%s]",ex.what());
							// 	odom_msg.twist.twist.linear.x = 0.0;
							// 	odom_msg.twist.twist.linear.y = 0.0;
							// 	odom_msg.twist.twist.linear.z = 0.0;	
							// 	continue;
							// }
						}

						// System Status	
						system_status_msg.message = "";
						system_status_msg.level = 0; // default OK state
						if (system_state_packet.system_status.b.system_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "0. System Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "1. Accelerometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gyroscope_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "2. Gyroscope Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.magnetometer_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "3. Magnetometer Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.pressure_sensor_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "4. Pressure Sensor Failure! ";
						}
						if (system_state_packet.system_status.b.gnss_failure) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "5. GNSS Failure! ";
						}
						if (system_state_packet.system_status.b.accelerometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "6. Accelerometer Over Range! ";
						}
						if (system_state_packet.system_status.b.gyroscope_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "7. Gyroscope Over Range! ";
						}
						if (system_state_packet.system_status.b.magnetometer_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "8. Magnetometer Over Range! ";
						}
						if (system_state_packet.system_status.b.pressure_over_range) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "9. Pressure Over Range! ";
						}
						if (system_state_packet.system_status.b.minimum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "10. Minimum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.maximum_temperature_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "11. Maximum Temperature Alarm! ";
						}
						if (system_state_packet.system_status.b.low_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "12. Low Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.high_voltage_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "13. High Voltage Alarm! ";
						}
						if (system_state_packet.system_status.b.gnss_antenna_disconnected) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "14. GNSS Antenna Disconnected! ";
						}
						if (system_state_packet.system_status.b.serial_port_overflow_alarm) {
							system_status_msg.level = 2; // ERROR state
							system_status_msg.message = system_status_msg.message + "15. Data Output Overflow Alarm! ";
						}
						
						// Filter Status
						filter_status_msg.message = "";
						filter_status_msg.level = 0; // default OK state
						if (system_state_packet.filter_status.b.orientation_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter Initialised. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "0. Orientation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.ins_filter_initialised) {
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter Initialised. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "1. Navigation Filter NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.heading_initialised) {
							filter_status_msg.message = filter_status_msg.message + "2. Heading Initialised. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "2. Heading NOT Initialised. ";
						}
						if (system_state_packet.filter_status.b.utc_time_initialised) {
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time Initialised. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "3. UTC Time NOT Initialised. ";
						}											
						if (system_state_packet.filter_status.b.event1_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 Occured. ";	
						}
						else {							
							filter_status_msg.message = filter_status_msg.message + "7. Event 1 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.event2_flag) {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 Occured. ";	
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "8. Event 2 NOT Occured. ";
						}
						if (system_state_packet.filter_status.b.internal_gnss_enabled) {
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS Enabled. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "9. Internal GNSS NOT Enabled. ";
						}
						if (system_state_packet.filter_status.b.magnetic_heading_enabled) {
							filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading Active. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "10. Magnetic Heading NOT Active. ";
						}											
						if (system_state_packet.filter_status.b.velocity_heading_enabled) {							
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading Enabled. ";	
						}
						else {							
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "11. Velocity Heading NOT Enabled. ";
						}
						if (system_state_packet.filter_status.b.atmospheric_altitude_enabled) {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude Enabled. ";	
						}
						else {
							filter_status_msg.message = filter_status_msg.message + "12. Atmospheric Altitude NOT Enabled. ";
							filter_status_msg.level = 1; // WARN state
						}
						if (system_state_packet.filter_status.b.external_position_active) {
							filter_status_msg.message = filter_status_msg.message + "13. External Position Active. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "13. External Position NOT Active. ";
						}
						if (system_state_packet.filter_status.b.external_velocity_active) {
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity Active. ";	
						}
						else {
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "14. External Velocity NOT Active. ";
						}											
						if (system_state_packet.filter_status.b.external_heading_active) {
							filter_status_msg.message = filter_status_msg.message + "15. External Heading Active. ";	
						}
						else {							
							filter_status_msg.level = 1; // WARN state
							filter_status_msg.message = filter_status_msg.message + "15. External Heading NOT Active. ";
						}	
					}
				}

				// Velocity standard deviation packet

				if (an_packet->id == packet_id_velocity_standard_deviation){
					if(decode_velocity_standard_deviation_packet(&velocity_standard_deviation_packet, an_packet) == 0){	
						// ODOM in base_link (FLU)
						odom_msg.twist.covariance[0] = pow(velocity_standard_deviation_packet.standard_deviation[1],2);
						odom_msg.twist.covariance[4] = pow(velocity_standard_deviation_packet.standard_deviation[0],2);
						odom_msg.twist.covariance[8] = pow(velocity_standard_deviation_packet.standard_deviation[2],2);	
						odom_vel_set = true;					
					}	
				}
				
				// Quaternion orientation standard deviation packet

				if (an_packet->id == packet_id_quaternion_orientation_standard_deviation) {
					// copy all the binary data into the typedef struct for the packet
					// this allows easy access to all the different values
					if(decode_quaternion_orientation_standard_deviation_packet(&quaternion_orientation_standard_deviation_packet, an_packet) == 0){	
						// IMU
						imu_msg.orientation_covariance[0] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[0],2);
						imu_msg.orientation_covariance[4] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[1],2);
						imu_msg.orientation_covariance[8] = pow(quaternion_orientation_standard_deviation_packet.standard_deviation[2],2);
						imu_vel_set = true;						
					}
				}

				// Ensure that you free the an_packet when you're done with it or you will leak memory
				an_packet_free(&an_packet);	
				map_transformStamped.header.stamp = ros::Time::now();
				imu_link_transformStamped.header.stamp = map_transformStamped.header.stamp;
				map_broadcaster.sendTransform(map_transformStamped);
				imu_link_broadcaster.sendTransform(imu_link_transformStamped);
			}
			// Publish messages
			if (!odom_vel_set){
				odom_msg.pose.covariance[0] = pow(2500,2);
				odom_msg.pose.covariance[4] = pow(2500,2);
				odom_msg.pose.covariance[8] = pow(2500,2);
				ROS_WARN("Spatial: [%s] velocity covariance set large. No packet obtained", odom_frame.c_str());
			}
			if (!imu_vel_set){
				imu_msg.orientation_covariance[0] = pow(2500,2);
				imu_msg.orientation_covariance[4] = pow(2500,2);
				imu_msg.orientation_covariance[8] = pow(2500,2);
				ROS_WARN("Spatial: [%s] orientation covariance set large. No packet obtained", imu_link_frame.c_str());
			}
		}
		nav_sat_fix_pub.publish(nav_sat_fix_msg);
		twist_pub.publish(twist_msg);
		imu_pub.publish(imu_msg);
		system_status_pub.publish(system_status_msg);
		filter_status_pub.publish(filter_status_msg);
		odom_pub.publish(odom_msg);
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}