#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

typedef boost::shared_ptr<geometry_msgs::PoseStamped const> PoseConstPtr;

class JoyTeleop
{
public:
  JoyTeleop();

  private: // declare all the required functions and variables
  void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void poseCallback(const PoseConstPtr& msg);
  void updateParameters();
  void timerCallback(const ros::TimerEvent& e);
  void publishZeroMessage();
  void publishPoint(); // function to publish point

  double linearScale, angularScale;
  int Button;
  ros::Subscriber joySub; // subscribe to joystick
  ros::Subscriber poseSub; // subscribe to robot_pose
  ros::Publisher pointPub; // publish robot_pose
  ros::NodeHandle nh; // Nodehandler
  ros::Timer timeout; // Ros timer

  //Pose
  geometry_msgs::PoseStamped pos;

};

JoyTeleop::JoyTeleop() { // constructor for class JoyTeleop
  joySub = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
  poseSub = nh.subscribe("/robot_pose", 10, &JoyTeleop::poseCallback, this);

  //pointPub= nh.advertise<geometry_msgs::PoseStamped>("/pt", 10);

  updateParameters();
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
  // process and publish
  geometry_msgs::Twist twistMsg;

  // check button, change variable button to switch to another button
  bool switchActive = (msg->buttons[Button] == 1);

  if (switchActive) {
    publishPoint();
  }
}

void JoyTeleop::poseCallback(const PoseConstPtr& msg) {
  pos = *msg;
}

void JoyTeleop::updateParameters() {
  // update the parameters for processing the joystick messages
  if (!nh.getParam("button", Button))
  Button = 5;   //Button RB

}

void JoyTeleop::publishPoint() {
  pointPub.publish(pos);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointRecorder_node");
  JoyTeleop joy_teleop_node;
  ros::spin();

  return 0;
}
