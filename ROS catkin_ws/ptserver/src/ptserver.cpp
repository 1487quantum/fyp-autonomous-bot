#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sstream>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <vector>

typedef std::vector<double> vec_d;  //Allows dynamic sizing
typedef boost::shared_ptr<geometry_msgs::PoseStamped const> PoseConstPtr;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Joystick button lookup table

int btnB=2,btnX=0,btnRT=7,btnLB=4,btnLT=6;

//-----------------------------------------------------------

//For I/O
#include <fstream>
#include <iostream>

// navigation
int count,bus_stop,route,move;
vec_d coordX,coordY, angW,angZ;
double currentX,currentY;
double distToGoal,distToGoalCount;
double goalCanceled;
double distObstacle=10;
std::string const& fpath = "/home/fyp-trolley/catkin_ws/waypts.bag";  //Bag location

sensor_msgs::LaserScan scanMsg;
geometry_msgs::PoseStamped msg;
geometry_msgs::PoseStamped pos; //current pos
double xd,xy;
int wayptCounter,eBrake;

class ptServer{
public:
  ptServer();
  ros::Publisher distToGoalPub;
  void countContents(std::string const& filename);
  void dumpContents(rosbag::Bag& b);
  void findBag(std::string const& filename);
  void p2p(double distance_x,double distance_y,double angle_z,double angle_w);
  void publishPoint(geometry_msgs::PoseStamped msg);
  void poseCallback(const PoseConstPtr& msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
  void publishVel(geometry_msgs::Twist msg);
  void setParam();
  void pubTgoal();


private:
  ros::Publisher pointPub; // publish pose_goal
  ros::Publisher velPub; // publish cmd_vel
  ros::Subscriber poseSub; // Subscribe to robot_pose
  ros::Subscriber joySub; // subscribe to joystick
  ros::Subscriber scanSub; // subscribe to laser
  ros::NodeHandle nh; // Nodehandler
  ros::Timer timeout; // Ros timer
};

ptServer::ptServer(){
  velPub= nh.advertise<geometry_msgs::Twist>("stop_vel", 1); //eBrake using lidar
  pointPub= nh.advertise<geometry_msgs::PoseStamped>("/target_pts", 10);
  distToGoalPub= nh.advertise<std_msgs::Float64>("/tgoal", 10);

  poseSub= nh.subscribe("/robot_pose", 10, &ptServer::poseCallback, this);
  joySub = nh.subscribe("/joy", 10, &ptServer::joyCallback, this);
  scanSub = nh.subscribe("/scan", 10, &ptServer::scanCallback, this);



}

void ptServer::publishPoint(geometry_msgs::PoseStamped msg){
  pointPub.publish(msg);
}
void ptServer::setParam()
{
  while(move==0)
  {
    ros::spinOnce();
  }
  move=0;
  nh.param<int>("route", route, 1);
}
void ptServer::publishVel(geometry_msgs::Twist msg){
  velPub.publish(msg);
}

void ptServer::poseCallback(const PoseConstPtr& msg) {
  pos = *msg;
}
void ptServer::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  // process and publish
  geometry_msgs::Twist twistMsg;

  // check button, change variable button to switch to another button
  bool switchActive = (msg->buttons[btnX] == 1);
  if (switchActive) {
    //eBrake=0;
    move=1;
  }
  else
  {
    //eBrake=0;
  }

}

void ptServer::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  scanMsg = *scan;
  geometry_msgs::Twist stop;
  stop.linear.x=0;
  stop.linear.y=0;
  stop.linear.z=0;
  stop.angular.z=0;
  int size = scan->ranges.size();
  for(int i=0; i<size ;i++){
    if ((scanMsg.ranges[i]<0.4) && (scanMsg.ranges[i]>0.004))
    {
      distObstacle=scanMsg.ranges[i];
      //std::cout<<"Obstacle detected: " <<distObstacle<<"m away ";
      //std::cout<<std::endl;
      eBrake=1;
      publishVel(stop);
      break;
    }
    else
    {
      eBrake=0;
    }
    //ros::Duration(0.1).sleep();
  }
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
void ptServer::pubTgoal()
{
  if (distToGoalCount==10000)
  {
  std_msgs::Float64 tgoal;
  tgoal.data = distToGoal;
  distToGoalPub.publish(tgoal);
  distToGoalCount=0;
  }
  distToGoalCount++;
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

  //we'll send a goal to the robot to move to the waypoints
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

  // Set target goal
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = angle_z;
  goal.target_pose.pose.orientation.w = angle_w;

  // Set goal marker
  goalx.pose.orientation.x = 0.0;
  goalx.pose.orientation.y = 0.0;
  goalx.pose.orientation.z = angle_z;
  goalx.pose.orientation.w = angle_w;

  ROS_INFO("Sending next goal");
  ac.sendGoal(goal);
  publishPoint(goalx);
  wayptCounter++;

  double xd=goalx.pose.position.x-pos.pose.position.x;
  double yd=goalx.pose.position.y-pos.pose.position.y;
  double odz=abs(goalx.pose.orientation.z-pos.pose.orientation.z);
  distToGoal=sqrt(xd*xd+yd*yd);

  while (distToGoal>0.5 || (odz>0.20))
  {
    if ((goalCanceled==1))
    {
      ros::Duration(3).sleep(); //Time delay of 3sec before resuming
      if (eBrake!=1)
      {
      ac.sendGoal(goal);
      goalCanceled=0;
      ROS_INFO("[%d]Goal to x:%f y:%f resumed",wayptCounter,distance_x,distance_y);
      }
    }
    else if ((eBrake==1) && (goalCanceled==0))
    {
      ac.cancelAllGoals();
      goalCanceled=1;
      ROS_INFO("[%d]Goal to x:%f y:%f canceled due to an obstacle",wayptCounter,distance_x,distance_y);
      ROS_INFO("Please clear the obstruction and wait 3sec...");
    }
    xd=(distance_x)-pos.pose.position.x;
    yd=(distance_y)-pos.pose.position.y;
    odz=abs(goalx.pose.orientation.z-pos.pose.orientation.z);
    distToGoal=sqrt(xd*xd+yd*yd);

    pubTgoal();
    ros::spinOnce(); // refresh callback from robot_pose
  }
  if ((wayptCounter%10)!=0){
    ROS_INFO("[%d]The base is approaching x:%f y:%f facing z:%f w:%f ",wayptCounter, distance_x,distance_y,angle_z,angle_w);
  }
  else
  {
    while ((distToGoal!=0.0) && (odz!=0.0)) // Wait for complete alignment with target
    {
      pubTgoal();
      if ((goalCanceled==1))
      {
        ros::Duration(3).sleep(); //Time delay of 3sec before resuming
        if (eBrake!=1)
        {
        ac.sendGoal(goal);
        goalCanceled=0;
        ROS_INFO("[%d]Goal to x:%f y:%f resumed",wayptCounter,distance_x,distance_y);
        }
      }
      else if ((eBrake==1) && (goalCanceled==0))
      {
        ac.cancelAllGoals();
        goalCanceled=1;
        ROS_INFO("[%d]Goal to x:%f y:%f canceled due to an obstacle",wayptCounter,distance_x,distance_y);
        ROS_INFO("Please clear the obstruction and wait 3sec...");
      }
  }
}
  ROS_INFO("[%d]The base has reached x:%f y:%f facing z:%f w:%f ", wayptCounter, distance_x,distance_y,angle_z,angle_w);

}


/*
ac.waitForResult();
ROS_INFO("Moving to goal");
if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
ROS_INFO("The base successfully moved to %f,%f facing %f,%f ", distance_x,distance_y,angle_z,angle_w);
else
ROS_INFO("The base failed to move to %f,%f facing %f,%f ", distance_x,distance_y,angle_z,angle_w);
*/




int main(int argc, char** argv){

  ros::init(argc, argv, "ptserver_node");
  ptServer ps;
  move=0;
  ps.countContents(fpath);
  ps.findBag(fpath);
  ros::Duration(5).sleep();
  while(1){
  ROS_INFO("Use 'rosparam set route x' to select route");
  ROS_INFO("1=to door,2=to charging pt (Default:1)");
  ROS_INFO("Press X to continue");
  ps.setParam();
  bus_stop=coordX.size(); // set bus_stop
  ROS_INFO("Route %d selected",route);
  if (route==1){
    ROS_INFO("Route towards door selected");
  for (double i=0.0; i<13.0; i+=1)
  {
    ps.p2p(coordX.at(i),coordY.at(i),angZ.at(i),angW.at(i)); // send goals
  }
    }
  else
  {
    ROS_INFO("Route towards charging pt selected");
    for (double i=13.0; i<21.0; i+=1)
    {
      wayptCounter=12;
      ps.p2p(coordX.at(i),coordY.at(i),angZ.at(i),angW.at(i)); // send goals
    }
  }
  std_msgs::Float64 tgoal;
  tgoal.data=0;
  ps.distToGoalPub.publish(tgoal);
  ROS_INFO("Docked at Bus Terminal");
  ROS_INFO("This bus service has terminated");
  ros::Duration(3).sleep();
}
  ros::spin();
  return 0;
}
