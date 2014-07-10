#ifndef __EXTRACT_EKF_ODOM_H__
#define __EXTRACT_EKF_ODOM_H__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
#include <nav_msgs/Odometry.h>  

//unsigned int origin_defined = 0;   

class ExtractEkfOdom
{
	public: 
		virtual ~ExtractEkfOdom(void) {}

		static ExtractEkfOdom&  Instance(void)
		{
			static ExtractEkfOdom INSTANCE;
			return INSTANCE;
		}

	void Initialize(void);
	void PoseMessageReceived(const geometry_msgs::PoseWithCovarianceStamped& msg);
	//void   ExtractEkfOdom::SendOdomTransform(void);

	private:
		ros::NodeHandle           	_nh;
		ros::Time 		  	_last_time;

		ros::Subscriber 		_combined_odom_sub;
		nav_msgs::Odometry        	_filtered_odom;
		ros::Publisher 		  	_filtered_odom_pub;

		geometry_msgs::TransformStamped _odom_trans;
		geometry_msgs::PoseWithCovarianceStamped  _originCovPose;
		geometry_msgs::PoseWithCovarianceStamped  oldCovPose;
		geometry_msgs::PoseWithCovarianceStamped  newCovPose;

		geometry_msgs::Quaternion       _odom_quat;

		unsigned int _origin_assigned;
		double 		     	 _x; // Current X value on coordinateframe
		double 			  	_y; // Current Y value on coordinateframe 
		double 			  	_th; // Current orientation on CF
		double			  	_r; // Linear X velocity (forward/back)
		double 			  	_vx; // Change in x velocity
		double 			 	_vy; // Change in y velocity
		double 			  	_vth; // Change in angular velocity
		double			  	_deltaX; // change in X direction between calls
		double			  	_deltaY; // change in Y direction between calls
		double			  	_deltaTh; // change in orientation between calls	
		
		ExtractEkfOdom(void);
};

#endif