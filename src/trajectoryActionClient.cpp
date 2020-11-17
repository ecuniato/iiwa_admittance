#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "nav_msgs/Path.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kuka_control/waypointsAction.h>

#include <tf/tf.h>

typedef actionlib::SimpleActionClient<kuka_control::waypointsAction> Client;

using namespace std;

class robotClient {
  public:
    robotClient();
    bool sendGoal(const kuka_control::waypointsGoal& goal);
    bool getPose(geometry_msgs::PoseStamped& pose);
    bool getWrench(geometry_msgs::WrenchStamped& wrench);
  private:
    void pose_cb(const geometry_msgs::PoseStampedConstPtr& message);
    void wrench_cb(const geometry_msgs::WrenchStampedConstPtr& message);
    void feedbackCb(const kuka_control::waypointsFeedbackConstPtr& feedback);

    ros::NodeHandle _nh;
    ros::Subscriber _cartpose_sub, _extWrench_sub;
    actionlib::SimpleActionClient<kuka_control::waypointsAction> _ac;
    kuka_control::waypointsFeedback _feedback;
    kuka_control::waypointsResult _result;
    kuka_control::waypointsGoal _goal;
    geometry_msgs::PoseStamped _pose;
    geometry_msgs::WrenchStamped _wrench;
    bool _firstpose, _firstwrench;
    double _completePerc;
};

// create the action client
// true causes the client to spin its own thread
robotClient::robotClient() : _ac("kukaActionServer", true) {
  _firstpose=_firstwrench=false;
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  _ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started.");
  _cartpose_sub = _nh.subscribe("/iiwa/eef_des_pose", 0, &robotClient::pose_cb, this);
  _extWrench_sub = _nh.subscribe("/iiwa/eef_ext_wrench", 0, &robotClient::wrench_cb, this);
  _completePerc=0;
}

void robotClient::pose_cb(const geometry_msgs::PoseStampedConstPtr& message) {
  //cout<<"got message"<<endl;
  _pose = *message;
  _firstpose = true;
}

void robotClient::wrench_cb(const geometry_msgs::WrenchStampedConstPtr& message) {
  _wrench = *message;
  _firstwrench = true;
}

bool robotClient::getPose(geometry_msgs::PoseStamped& pose) {
  if(!_firstpose)
    return false;
  pose = _pose;
  return true;
}

bool robotClient::getWrench(geometry_msgs::WrenchStamped& wrench) {
  if(!_firstwrench)
    return false;
  wrench = _wrench;
  return true;
}

// Called every time feedback is received for the goal
void robotClient::feedbackCb(const kuka_control::waypointsFeedbackConstPtr& feedback) {
  _completePerc = feedback->completePerc;
  //cout<<"Trajectory completed at "<<feedback->completePerc<<"% ."<<endl;
}

bool robotClient::sendGoal(const kuka_control::waypointsGoal& goal) {
  ROS_INFO("Sending goal.");
  _ac.sendGoal(goal,Client::SimpleDoneCallback(),Client::SimpleActiveCallback(), boost::bind(&robotClient::feedbackCb, this, _1));

  //wait for the action to return
  //bool finished_before_timeout = _ac.waitForResult(ros::Duration(goal.times.back()+10.0));
  actionlib::SimpleClientGoalState goalState(actionlib::SimpleClientGoalState::LOST);
  ros::Rate rate(2);
  do {
    goalState = _ac.getState();
    cout<<"Trajectory completed at "<<_completePerc<<"% ."<<endl;
    rate.sleep();
  } while( ((goalState == actionlib::SimpleClientGoalState::PENDING) || (goalState == actionlib::SimpleClientGoalState::ACTIVE)) && ros::ok() );

  _completePerc = 0;
  if (goalState == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    actionlib::SimpleClientGoalState state = _ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  return (goalState == actionlib::SimpleClientGoalState::SUCCEEDED);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "waypointsActionClient");

  ros::AsyncSpinner spinner(1); // Use 1 thread
	spinner.start();

  robotClient lwrClient;

  kuka_control::waypointsGoal goal;
  // send a goal to the action
  nav_msgs::Path waypoints;
  geometry_msgs::PoseStamped p, initPose;
  geometry_msgs::TwistStamped initVel,finalVel;
  geometry_msgs::AccelStamped initAcc,finalAcc;
  while(!lwrClient.getPose(initPose) && ros::ok()) {
    sleep(1.0);
    cout<<"Waiting for first pos."<<endl;
  }

  goal.poseOrForce=true;
  waypoints.poses.push_back(initPose);
  p=initPose;
  p.pose.position.x = 0.56;
	p.pose.position.y = 0.0;
	p.pose.position.z = 0.40;
  tf::Quaternion qinit(0.9763,0.0592,0.207,-0.0196);
  qinit.normalize();
	p.pose.orientation.z = qinit.z();
	p.pose.orientation.w = qinit.w();
	p.pose.orientation.x = qinit.x();
	p.pose.orientation.y = qinit.y();
  waypoints.poses.push_back(p);

  std::vector<double> times;
  times.push_back(0);
  times.push_back(3);

  goal.waypoints=waypoints;
  goal.times=times;
  goal.initVel=initVel;
  goal.finalVel=finalVel;
  goal.initAcc=initAcc;
  goal.finalAcc=finalAcc;
  lwrClient.sendGoal(goal);

  sleep(5);
/*
  goal.poseOrForce=false;
  geometry_msgs::WrenchStamped he,mask;
	lwrClient.getWrench(he);
  goal.initWrench=he;
  he.wrench.force.y+=5;
  goal.finalWrench=he;
  mask.wrench.force.y=1;
  goal.mask=mask;
  lwrClient.sendGoal(goal);


  sleep(5);
*/

  if(!ros::ok()) return 0;

  goal.poseOrForce=true;
  lwrClient.getPose(waypoints.poses[0]);
  waypoints.poses[1].pose.position.z += 0.30;
  goal.waypoints=waypoints;
  lwrClient.sendGoal(goal);

 // sleep(5);

  /*
  goal.poseOrForce=true;
  lwrClient.getPose(waypoints.poses[0]);
  waypoints.poses[1]=initPose;
  goal.waypoints=waypoints;
  lwrClient.sendGoal(goal);
*/
  //exit
  return 0;
}
