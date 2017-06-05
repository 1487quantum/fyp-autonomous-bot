#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>

typedef boost::shared_ptr<geometry_msgs::PoseStamped const> PoseConstPtr;

//Variables
rosbag::Bag bag;
bool ft;  //First time
double wayptCounter;

class ptRecorder
{

  public:
  ptRecorder();

  private: // declare all the required functions and variables
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void poseCallback(const PoseConstPtr& msg);
  void updateParameters();
  void timerCallback(const ros::TimerEvent& e);
  void publishZeroMessage();
  void publishPoint(); // function to publish point

  double linearScale, angularScale;
  int Button,Button2,Button3;
  ros::Subscriber joySub; // subscribe to joystick
  ros::Subscriber poseSub; // subscribe to robot_pose
  ros::Publisher pointPub; // publish robot_pose
  ros::NodeHandle nh; // Nodehandler
  ros::Timer timeout; // Ros timer

  //Pose
  geometry_msgs::PoseStamped pos;
};

ptRecorder::ptRecorder() { // constructor for class JoyTeleop
  joySub = nh.subscribe("/joy", 10, &ptRecorder::joyCallback, this);
  poseSub = nh.subscribe("/robot_pose", 10, &ptRecorder::poseCallback, this);

  pointPub= nh.advertise<geometry_msgs::PoseStamped>("/pt", 10);

  updateParameters();
}

void ptRecorder::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  // process and publish
  geometry_msgs::Twist twistMsg;

  // check button, change variable button to switch to another button
  bool switchActive = (msg->buttons[Button] == 1);
  if (switchActive) {
    //Run following code if button is pressed for the 1st time
    if(ft){
      ft=0;
      bag.open("/home/fyp-trolley/catkin_ws/waypts.bag", rosbag::bagmode::Write);   //ToDo: Open bag only if button is pressed
    }
    publishPoint();
  }
}

void ptRecorder::poseCallback(const PoseConstPtr& msg) {
  pos = *msg;
}

void ptRecorder::updateParameters() {
  // update the parameters for processing the joystick messages
  if (!nh.getParam("button", Button))
    Button = 5;   //Button RB
  /* Future use
  if (!nh.getParam("button2", Button2))
  Button2 = 2;   //Button X
  if (!nh.getParam("button3", Button3))
  Button3 = 4;   //Button LB
  */
}

void ptRecorder::publishPoint() {
  pointPub.publish(pos);
  bag.write("point",  ros::Time::now(), pos);   //Save points in rosbag
  ROS_INFO("Waypt: %f",wayptCounter);
  ROS_INFO_STREAM("Waypoint: " << pos);   //Display in console
  wayptCounter++;

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointRecorder_node");
  ft = 1;
  ptRecorder joy_teleop_node;
  ros::spin();
  //Close bag
  bag.close();
  return 0;
}
