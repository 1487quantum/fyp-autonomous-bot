/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Russell Toris - rctoris@wpi.edu
 * \date April 3, 2014
 */

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
geometry_msgs::PoseWithCovarianceStamped pose_stamped1;
int start=0;

/*!
 * Creates and runs the robot_pose_publisher node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // configuring parameters
  std::string map_frame, base_frame;
  double publish_frequency;
  bool is_stamped;
  ros::Publisher p_pub;
  ros::Publisher p_pub2;

  nh_priv.param<std::string>("map_frame",map_frame,"/map");
  nh_priv.param<std::string>("base_frame",base_frame,"/base_link");
  nh_priv.param<double>("publish_frequency",publish_frequency,10);
  nh_priv.param<bool>("is_stamped", is_stamped, true);

  if(is_stamped)
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
  else
    p_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);

 if(start==0)
    p_pub2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);


  // create the listener
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));

  ros::Rate rate(publish_frequency);
      pose_stamped1.header.frame_id = map_frame;
      pose_stamped1.header.stamp = ros::Time::now();

      pose_stamped1.pose.pose.orientation.x = 0.0;
      pose_stamped1.pose.pose.orientation.y = 0.0;
      pose_stamped1.pose.pose.orientation.z = 0.7016;
      pose_stamped1.pose.pose.orientation.w = 0.7125;

      pose_stamped1.pose.pose.position.x = -0.62;
      pose_stamped1.pose.pose.position.y = -0.52;
      pose_stamped1.pose.pose.position.z = 0.0;

      if(start==0)
        p_pub2.publish(pose_stamped1);


  while (nh.ok())
  {
    tf::StampedTransform transform;
    try
    {
      listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
	     start=1;
      // construct a pose message
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();

      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();

      if(is_stamped)
      p_pub.publish(pose_stamped);
    else
      p_pub.publish(pose_stamped.pose);
    }
    catch (tf::TransformException &ex)
    {
      // just continue on
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}
