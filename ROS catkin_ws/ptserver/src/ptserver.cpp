#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <vector>
typedef std::vector<double> vec_d;  //Allows dynamic sizing

//For I/O
#include <fstream>
#include <iostream>

int count;
vec_d coordX,coordY, angW;
std::string const& fpath = "/home/fyp-trolley/catkin_ws/waypts.bag";  //Bag location


//Count no of waypoints
void countContents(std::string const& filename) {
  ROS_INFO_STREAM("Counting presents..");

  rosbag::Bag b;
  b.open(filename, rosbag::bagmode::Read);

  int message_count = 0;

  rosbag::View view(b);
  foreach(rosbag::MessageInstance m, view) {
    geometry_msgs::PoseStamped::ConstPtr s = m.instantiate<geometry_msgs::PoseStamped>(); //Count only geometry_msgs::PoseStamped type
    if (s != NULL) {
      //ASSERT_EQ(s->data, foo_.data);
      message_count++;
    }
  }

  ROS_INFO_STREAM("Presents to open: "<< message_count);
  b.close();
}

//Extract contents
void dumpContents(rosbag::Bag& b) {
  ROS_INFO_STREAM("Opening presents..");
  count = 0;
  rosbag::View view(b);
  foreach(rosbag::MessageInstance m, view){
    geometry_msgs::PoseStamped::ConstPtr msg_g = m.instantiate<geometry_msgs::PoseStamped>();
    coordX.push_back(msg_g->pose.position.x);
    coordY.push_back(msg_g->pose.position.y);
    angW.push_back(msg_g->pose.orientation.w);

    std::cout << "Waypoint " << count << ":"<< std::endl; //Waypoint id
    std::cout << coordX.at(count) << ", "; //X
    std::cout << coordY.at(count) << ", "; //Y
    std::cout << angW.at(count) << std::endl; //Angle
    count++;
  }
}

//Open bag location
void findBag(std::string const& filename) {
  rosbag::Bag b;
  b.open(filename, rosbag::bagmode::Read);
  dumpContents(b);
  b.close();
}

//Point to point navigation
void p2p(){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  double distance_x = .5;
  goal.target_pose.pose.position.x = 0.5;
  goal.target_pose.pose.orientation.w = 0.45;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("Hooray, the base moved %f meter forward", distance_x);
  else
  ROS_INFO("The base failed to move forward %f meters for some reason", distance_x);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "ptserver_node");

  //rosbag::Bag bag;
  //bag.open("/home/fyp-trolley/catkin_ws/waypts.bag", rosbag::bagmode::Read);

  countContents(fpath);
  findBag(fpath);
  //p2p();
  ros::spin();
  return 0;
}
