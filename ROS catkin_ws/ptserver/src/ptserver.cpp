#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include "std_msgs/String.h"
#include <sstream>

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
vec_d coordX,coordY, angW,angZ;
double currentX,currentY;
std::string const& fpath = "/home/fyp-trolley/catkin_ws/waypts.bag";  //Bag location

geometry_msgs::PoseStamped msg;
std::stringstream ss;

class ptServer{
public:
  ptServer();
  void countContents(std::string const& filename);
  void dumpContents(rosbag::Bag& b);
  void findBag(std::string const& filename);
  void p2p(double distance_x,double distance_y,double angle_z,double angle_w);
  void publishPoint(geometry_msgs::PoseStamped msg);


private:
  ros::Publisher pointPub; // publish robot_pose
  ros::NodeHandle nh; // Nodehandler
  ros::Timer timeout; // Ros timer
};

ptServer::ptServer(){
  pointPub= nh.advertise<geometry_msgs::PoseStamped>("/target_pts", 10);
  //ros::Subscriber sub = nh.subscribe("odom", 1000, chatterCallback);
}

void ptServer::publishPoint(geometry_msgs::PoseStamped msg){
  pointPub.publish(msg);
}



//Count no of waypoints
void ptServer::countContents(std::string const& filename) {
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
void ptServer::dumpContents(rosbag::Bag& b) {
  ROS_INFO_STREAM("Opening presents..");
  count = 0;
  rosbag::View view(b);

  foreach(rosbag::MessageInstance m, view){
    geometry_msgs::PoseStamped::ConstPtr msg_g = m.instantiate<geometry_msgs::PoseStamped>();

    //publishPoint(msg_g);
    coordX.push_back(msg_g->pose.position.x);
    coordY.push_back(msg_g->pose.position.y);
    angW.push_back(msg_g->pose.orientation.w);
    angZ.push_back(msg_g->pose.orientation.z);

    //Display in console
    std::cout << "Waypoint " << count << ":"<< std::endl; //Waypoint id
    std::cout << coordX.at(count) << ", "; //X
    std::cout << coordY.at(count) << ", "; //Y
    std::cout << angZ.at(count) << ", "; // Angle w
    std::cout << angW.at(count) << std::endl; //Angle z

    //For publishing
    count++;
  }
  //ROS_INFO("%s", msg.data.c_str());


}

//Open bag location
void ptServer::findBag(std::string const& filename) {
  rosbag::Bag b;
  b.open(filename, rosbag::bagmode::Read);
  dumpContents(b);
  b.close();
}

//Point to point navigation
void ptServer::p2p(double distance_x,double distance_y,double angle_z,double angle_w){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  geometry_msgs::PoseStamped goalx;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map"; // reference to map
  goal.target_pose.header.stamp = ros::Time::now();

  // set x,y coordinates
  goal.target_pose.pose.position.x = distance_x;
  goal.target_pose.pose.position.y = distance_y;

  //

  goalx.header.frame_id = "map"; // reference to map
  goalx.header.stamp = ros::Time::now();

  // set x,y coordinates
  goalx.pose.position.x = distance_x;
  goalx.pose.position.y = distance_y;

  // Set quaternion(angle)
  double radians = 0.0 * (M_PI/180);
  tf::Quaternion quaternion;
  quaternion = tf::createQuaternionFromYaw(radians);

  geometry_msgs::Quaternion qMsg;
  tf::quaternionTFToMsg(quaternion, qMsg);

  //goal.target_pose.pose.orientation = qMsg;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = angle_z;
    goal.target_pose.pose.orientation.w = angle_w;


    goalx.pose.orientation.x = 0.0;
    goalx.pose.orientation.y = 0.0;
    goalx.pose.orientation.z = angle_z;
    goalx.pose.orientation.w = angle_w;

  ROS_INFO("Sending next command");
  ac.sendGoal(goal);
  publishPoint(goalx);

  ac.waitForResult();
  ROS_INFO("Moving to goal");
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  ROS_INFO("The base successfully moved to %f,%f facing %f,%f ", distance_x,distance_y,angle_z,angle_w);
  else
  ROS_INFO("The base failed to move to %f,%f facing %f,%f ", distance_x,distance_y,angle_z,angle_w);

}




int main(int argc, char** argv){
  ros::init(argc, argv, "ptserver_node");
  ptServer ps;
  ps.countContents(fpath);
  ps.findBag(fpath);

  //msg.data = ss.str();
  //ps.publishPoint(msg);

  for (double i=0.0; i<coordX.size(); i+=1)
  {
    ps.p2p(coordX.at(i),coordY.at(i),angZ.at(i),angW.at(i)); // send goals
    //ROS_INFO("Waiting for 2sec...");
    //ros::Duration(2).sleep(); // wait 2 sec
  }


  ros::spin();
  return 0;
}
