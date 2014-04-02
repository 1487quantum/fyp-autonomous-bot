#ifndef _base_pose_hpp
#define _base_pose_hpp

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

namespace base_pose
{
class base_pose
{
public:
	base_pose( const ros::NodeHandle &_nh = ros::NodeHandle( ), const ros::NodeHandle &_nh_priv = ros::NodeHandle( "~" ) );
	~base_pose( );

	bool start( );
	void stop( );
	bool stat( );

private:
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv;
	ros::Publisher pose_stamped_pub;
	ros::Subscriber joint_state_sub;
	const ros::SubscriberStatusCallback pose_stamped_callback;

	std::string frame_id;
	std::string left_joint_name;
	std::string right_joint_name;
	double wheel_base;
	double wheel_diam;
	double wheel_diam2;

	void pose_stamped_cb( );
	void joint_state_cb( const sensor_msgs::JointStatePtr &msg );
};
}

#endif /* _base_pose_hpp */
