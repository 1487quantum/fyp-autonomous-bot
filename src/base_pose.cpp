#include "base_pose/base_pose.hpp"

#include <ros/ros.h>

namespace base_pose
{
base_pose::base_pose( const ros::NodeHandle &_nh, const ros::NodeHandle &_nh_priv ) :
	nh( _nh ),
	nh_priv( _nh_priv ),
	pose_stamped_callback( boost::bind(&base_pose::pose_stamped_cb, this) )
{
	nh_priv.param( "wheel_base", wheel_base, 0.2635 );
	nh_priv.param( "wheel_diam", wheel_diam, 0.0750 );
	nh_priv.param( "wheel_diam2", wheel_diam2, wheel_diam );
	nh_priv.param<std::string>( "frame_id", frame_id, "base_link" );
	nh_priv.param<std::string>( "left_wheel_joint", left_joint_name, "left_wheel_joint" );
        nh_priv.param<std::string>( "right_wheel_joint", right_joint_name, "right_wheel_joint" );
}

base_pose::~base_pose( )
{
}

bool base_pose::start( )
{
	if( !( pose_stamped_pub = nh.advertise<geometry_msgs::PoseStamped>( "robot_pose", 1, pose_stamped_callback, pose_stamped_callback, ros::VoidConstPtr( ), false ) ) )
		return false;

	pose_stamped_cb( );

	return true;
}

void base_pose::stop( )
{
	if( joint_state_sub )
		joint_state_sub.shutdown( );

	if( pose_stamped_pub )
		pose_stamped_pub.shutdown( );
}

bool base_pose::stat( )
{
	return pose_stamped_pub;
}

void base_pose::pose_stamped_cb( )
{
	if( pose_stamped_pub.getNumSubscribers( ) > 0 )
	{
		if( !joint_state_sub && !( joint_state_sub = nh.subscribe( "joint_state", 1, &base_pose::joint_state_cb, this ) ) )
			ROS_ERROR( "Failed to start joint state subscription" );
	}
	else if( joint_state_sub )
		joint_state_sub.shutdown( );
}

void base_pose::joint_state_cb( const sensor_msgs::JointStatePtr &msg )
{
	static ros::Time last_time = msg->header.stamp;
	static double x = 0;
	static double y = 0;
	static double th = 0;

	static double old_left = 0;
	static double old_right = 0;

	double left_diff = 0;
	double right_diff = 0;
	double dt = 0;

	unsigned int i;
 
	bool joint_recv = false;
 
	double rad_right = 0;
	double rad_left = 0;

	for( i = 0; i < msg->name.size(); i++)
	{
		if(msg->name[i] == right_joint_name)
		{
			rad_right = msg->position[i];
			joint_recv = true;
		}
		else if(msg->name[i] == left_joint_name)
		{
			rad_left = msg->position[i];
			joint_recv = true;
		}
	}

	if( !joint_recv )
		return;

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header = msg->header;
	pose_stamped.header.frame_id = frame_id;

	right_diff = old_right - rad_right;
	left_diff = old_left - rad_left;

	dt = (pose_stamped.header.stamp - last_time).toSec();

	//generate pose
	x += ( ( ( wheel_diam * dt * left_diff ) / 4.0 ) + ( ( wheel_diam2 * dt * right_diff ) / 4.0 ) ) * cos(th);
	y += ( ( ( wheel_diam * dt * left_diff ) / 4.0 ) + ( ( wheel_diam2 * dt * right_diff ) / 4.0 ) ) * sin(th);
	th += ( ( wheel_diam * dt * left_diff ) / ( 4.0 * wheel_base ) ) - ( ( wheel_diam * dt * right_diff ) / ( 4.0 * wheel_base ) );

	//publish the message
	pose_stamped.pose.position.x = x;
	pose_stamped.pose.position.y = y;

	pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(th);
	
	pose_stamped_pub.publish(pose_stamped);

	old_left = rad_left;
	old_right = rad_right;

	last_time = pose_stamped.header.stamp;
}
}
