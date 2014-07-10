#include <ros/ros.h>
//#include "extract_ekf_odom.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 


#define TWOPI 6.2831853

geometry_msgs::PoseWithCovarianceStamped  oldCovPose;
geometry_msgs::PoseWithCovarianceStamped  newCovPose;
unsigned int origin_assigned = 0;



/*
ExtractEkfOdom::ExtractEkfOdom(void)
 : _origin_assigned(0), _x(0.0), _y(0.0), _th(0.0), _delta_x(0.0), _delta_y(0.0), _delta_th(0.0)
{
}

void ExtractEkfOdom::Initialize(void)
{
	_combined_odom_sub 				= _nh.subscribe("combined_odom", 10, &ExtractEkfOdom::PoseMessageReceived, this);
	_filtered_odom_pub 				= _nh.advertise<nav_msgs::Odometry>("filtered_odom", 50);
	_last_time 				= ros::Time::now();
	boost::bind(&ExtractEkfOdom::SendOdomTransform, this);
	
	//_odom_broadcaster.sendTransform(_odom_trans);	
}
*/
void PoseMessageReceived(const geometry_msgs::PoseWithCovarianceStamped& msg)
{ 	
	ROS_INFO("Received message");
	if(origin_assigned)
	{
		oldCovPose = newCovPose;
		newCovPose = msg;
	}
	else
	{
		oldCovPose = msg;
		newCovPose = msg;
		origin_assigned = true;
	}
// 	ros::Time _current_time 		= ros::Time::now();

// 	double _dt 			= (_current_time - _last_time).toSec();

// 	//filling the odometry
//     _filtered_odom.header.stamp 		= _current_time;
//     _filtered_odom.header.frame_id 		= "odom";
//     _filtered_odom.child_frame_id 		= "base_footprint";
	
// 	if(_origin_assigned)
// 	{
// 		_newCovPose = msg;

// 		_delta_x = _newCovPose.pose.pose.position.x - _oldCovPose.pose.pose.position.x;

// 		_delta_y = _newCovPose.pose.pose.position.y - _oldCovPose.pose.pose.position.y;

// 		_delta_th = tf::get_yaw(_newCovPose.pose.pose.orientation) - tf::get_yaw(_oldCovPose.pose.pose.orientation);

// 		_odom_quat = tf::createQuaternionMsgFrom_yaw(_delta_th); // create new quat from yaw diff

// 		_r = sqrt(_delta_x*_delta_x + _delta_y*_delta_y) / _dt;

// 		if(_delta_th > TWOPI/2.0) // theta can only have a value between -pi and pi to account for rotational direction
// 		{
// 			_delta_th -= TWOPI;
// 		}
// 		else if(_delta_th < TWOPI/2.0)
// 		{
// 			_delta_th += TWOPI;
// 		}

// 		_vx = _delta_x / _dt;

// 		_vy = _delta_y / _dt;

// 		_vth = _delta_th / _dt;

// 		_x += _delta_x;

// 		_y += _delta_y;

// 	}
// 	else
// 	{
// 		_originCovPose = msg; // so far doing nothing with this
// 		_oldCovPose = _originCovPose;

// 		_origin_assigned = true;
// 	}

	
// //Update the transform
// 	/*
//     _odom_trans.header.stamp 		= _currentTime;
//     _odom_trans.transform.translation.x 	= _x;
//     _odom_trans.transform.translation.y 	= _y;
//     _odom_trans.transform.translation.z 	= 0.0;
//     _odom_trans.transform.rotation 		= tf::createQuaternionMsgFrom_yaw(_theta);
//     */

// //position
//     _filtered_odom.pose.pose.position.x 	= _x;
//     _filtered_odom.pose.pose.position.y 	= _y;
//     _filtered_odom.pose.pose.position.z 	= 0.0;
//     _filtered_odom.pose.pose.orientation = _odom_quat; 

// //velocity
//     _filtered_odom.twist.twist.linear.x 	= _r; //_vx
//     _filtered_odom.twist.twist.linear.y 	= 0.0; //_vy;
//     _filtered_odom.twist.twist.linear.z 	= 0.0;
//     _filtered_odom.twist.twist.angular.x 	= 0.0;
//     _filtered_odom.twist.twist.angular.y 	= 0.0;
//     _filtered_odom.twist.twist.angular.z 	= _vth; 

	
// 	_filtered_odom_pub.publish(_filtered_odom);
// 	//_odom_broadcaster.sendTransform(_odom_trans);
// 	_last_time 			= _current_time;
// 	_oldCovPose = _newCovPose;	
}

/*
void    ExtractEkfOdom::SendOdomTransform(void)
{
    _odom_trans.header.frame_id             = "odom";
    _odom_trans.child_frame_id              = "base_footprint";
    _odom_trans.header.stamp                = ros::Time::now(); 
    _odom_trans.transform.translation.x     = _x;
    _odom_trans.transform.translation.y     = _y;
    _odom_trans.transform.translation.z     = 0.0;
    _odom_trans.transform.rotation          = tf::createQuaternionMsgFrom_yaw(_delta_th);

    _odom_broadcaster.sendTransform(_odom_trans);
}
*/


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "extract_ekf_odom");

	ros::NodeHandle nh;
    ros::Subscriber combined_odom_sub = nh.subscribe("robot_pose_ekf/odom_combined", 10, &PoseMessageReceived);
	ros::Publisher filtered_odom_pub = nh.advertise<nav_msgs::Odometry>("filtered_odom", 50);

	// Initial position is at 0,0
    double x    = 0.0;
    double y    = 0.0;
    double th   = 0.0;

    float r             = 0.0;
    float theta         = 0.0;
    double vx           = 0.0;
    double vy           = 0.0;
    double vth          = 0.0;

    double delta_x      = 0.0;
    double delta_y      = 0.0;
    double delta_th     = 0.0;
    //unsigned int _origin_assigned = 0;

    ros::Time last_time = ros::Time::now();       

    tf::TransformBroadcaster    odom_broadcaster;

    ros::Rate loop_rate(20);

    geometry_msgs::TransformStamped odom_trans;
    //odom_trans.header.frame_id      = "odom";
    //odom_trans.child_frame_id       = "base_footprint";

    geometry_msgs::Quaternion       odom_quat;

    nav_msgs::Odometry filtered_odom;

	//ExtractEkfOdom::Instance().Initialize();

	while(ros::ok())
	{
		ros::spinOnce();
		ros::Time current_time    = ros::Time::now();
		double dt   = (current_time - last_time).toSec();


		//filling the odometry
	    filtered_odom.header.stamp 		    = current_time;
	    filtered_odom.header.frame_id 		= "odom";
	    filtered_odom.child_frame_id 		= "base_footprint";

		
		if(origin_assigned)
		{
			//newCovPose = msg;

			x = newCovPose.pose.pose.position.x;

			delta_x = x - oldCovPose.pose.pose.position.x;

			y = newCovPose.pose.pose.position.y;

			delta_y = y - oldCovPose.pose.pose.position.y;

			//geometry_msgs::Point new_point = newCovPose.pose.pose.position;
			//geometry_msgs::Point old_point = oldCovPose.pose.pose.position;

			geometry_msgs::Quaternion new_orientation = newCovPose.pose.pose.orientation;
			geometry_msgs::Quaternion old_orientation = oldCovPose.pose.pose.orientation;

			//ROS_INFO("new orientation: x=%f,y=%f,z=%f,w=%f",new_orientation.x,new_orientation.y,new_orientation.z,new_orientation.w);

			//ROS_INFO("old orientation: x=%f,y=%f,z=%f,w=%f",old_orientation.x,old_orientation.y,old_orientation.z,old_orientation.w);

			ROS_INFO("old Yaw: %f rads",tf::getYaw(old_orientation));
			ROS_INFO("new Yaw: %f rads",tf::getYaw(new_orientation));

			//ROS_INFO("new point: x=%f,y=%f,z=%f",new_point.x,new_point.y,new_point.z);
			//ROS_INFO("old point: x=%f,y=%f,z=%f",old_point.x,old_point.y,old_point.z);

			delta_th = tf::getYaw(newCovPose.pose.pose.orientation) - tf::getYaw(oldCovPose.pose.pose.orientation); //FIX

			ROS_INFO("delta_th = %f rads",delta_th);
			ROS_INFO("time diff: %f",dt);

			odom_quat = tf::createQuaternionMsgFromYaw(delta_th); // create new quat from yaw diff

			r = sqrt(delta_x*delta_x + delta_y*delta_y) / dt;

			// if the change in direction is opposite the current coordinate signs0, linear velocity is negative 
			if((x/delta_x <0) && (y/delta_y < 0))
			{
				r = -1*r;
			}

			if(delta_th > TWOPI/2.0) // theta can only have a value between -pi and pi to account for rotational direction
			{
				delta_th -= TWOPI;
			}
			else if(delta_th < -TWOPI/2.0)
			{
				delta_th += TWOPI;
			}

			vx = delta_x / dt;

			vy = delta_y / dt;

			ROS_INFO("AFTER if: delta_th = %f rads",delta_th);
			

			vth = delta_th / dt;
			ROS_INFO("AFTER if: vth = %f rads/s",vth);

			//x += delta_x;

			//y += delta_y;

			odom_trans.header.frame_id          = "odom";
	        odom_trans.child_frame_id           = "base_footprint";
	        odom_trans.header.stamp             = current_time;
	        odom_trans.transform.translation.x  = x;
	        odom_trans.transform.translation.y  = y;
	        odom_trans.transform.translation.z  = 0.0;
	        odom_trans.transform.rotation       = odom_quat;

			//position
		    filtered_odom.pose.pose.position.x 	= x;
		    filtered_odom.pose.pose.position.y 	= y;
		    filtered_odom.pose.pose.position.z 	= 0.0;
		    filtered_odom.pose.pose.orientation = odom_quat; 

			//velocity
		    filtered_odom.twist.twist.linear.x 	= r; //_vx
		    filtered_odom.twist.twist.linear.y 	= 0.0; //_vy;
		    filtered_odom.twist.twist.linear.z 	= 0.0;
		    filtered_odom.twist.twist.angular.x 	= 0.0;
		    filtered_odom.twist.twist.angular.y 	= 0.0;
		    filtered_odom.twist.twist.angular.z 	= vth; 

			last_time 			= current_time;

			odom_broadcaster.sendTransform(odom_trans);
			filtered_odom_pub.publish(filtered_odom);
		}
		/*else
		{
			//originCovPose = msg; // so far doing nothing with this
			//oldCovPose = msg;

			origin_assigned = true;
		}
		*/
		//0ROS_INFO("odom_quat: x= %f, y= %f, z= %f, w= %f", odom_quat.x,odom_quat.y,odom_quat.z,odom_quat.w);
		//Update the transform

		//_odom_broadcaster.sendTransform(_odom_trans);

		loop_rate.sleep();
	}
	return 0;
}