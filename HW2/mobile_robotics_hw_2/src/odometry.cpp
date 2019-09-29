// Your name etc. goes here!
// Name: Yizheng Zhang
// Student ID: 2018233135

#include <ros/ros.h>
#include "mobile_robotics_hw_2/Pose2DStamped.h"
// Maybe you'll need to include something else, too?
#include <tf/transform_datatypes.h>
#include "mobile_robotics_hw_2/SpeedStamped.h"
#include <cmath>
#include <Eigen/Dense>


// You may declare global variables here...
ros::Publisher pub;
ros::Publisher speedPub;

// This is how a global shared pointer to the message can be created:
mobile_robotics_hw_2::Pose2DStamped::Ptr currentPose;

void odomCallback(const mobile_robotics_hw_2::Pose2DStamped::ConstPtr& odom)
{
  // you may comment the next line...
  //ROS_INFO_STREAM("Received odometry2D :"<<*odom);

  mobile_robotics_hw_2::Pose2DStamped::Ptr posePtr(new mobile_robotics_hw_2::Pose2DStamped());
  
  // ToDo: calculate the global pose estimate using the odometry values here and set the values in posePtr accordingly.
  // Be sure to set the header in posePtr correctly (stamp and frame_id).
  
  // set header
  posePtr->header.stamp = odom->header.stamp;
  posePtr->header.seq = odom->header.seq;
  posePtr->header.frame_id = "global";

  // accumulate the data
  Eigen::Matrix3d currentT;
  Eigen::Vector3d poseT;
  Eigen::Vector3d odomT;
  currentT << cos(currentPose->pose2D.theta), -sin(currentPose->pose2D.theta), currentPose->pose2D.x,
						  sin(currentPose->pose2D.theta), cos(currentPose->pose2D.theta), currentPose->pose2D.y,
						  0, 0, 1;
  odomT <<  odom->pose2D.x,
            odom->pose2D.y,
            1;
  poseT = currentT * odomT;

  posePtr->pose2D.x = poseT[0];
  posePtr->pose2D.y = poseT[1];
  posePtr->pose2D.theta += currentPose->pose2D.theta + odom->pose2D.theta;

  // speed topic data
  mobile_robotics_hw_2::SpeedStamped::Ptr speedPtr(new mobile_robotics_hw_2::SpeedStamped());
  // set header
  speedPtr->header.stamp = odom->header.stamp;
  speedPtr->header.seq = odom->header.seq;
  speedPtr->header.frame_id = "global";

  // calcuate speed
  speedPtr->speed.data = pow(pow(posePtr->pose2D.x - currentPose->pose2D.x, 2) + pow(posePtr->pose2D.y - currentPose->pose2D.y, 2), 1.0/2) / (posePtr->header.stamp-currentPose->header.stamp).toSec();
 
  // update currentPose
  currentPose = posePtr; 
 
  pub.publish(posePtr);
  speedPub.publish(speedPtr);
  
}

int main(int argc, char **argv)
{
  
  ROS_INFO(" Mobile Robotics Homework 2 ");


  // maybe create an object (on the heap) and store the shared pointer in currentPose - also initializes the currentPose with (0,0,0)
  currentPose = mobile_robotics_hw_2::Pose2DStamped::Ptr(new mobile_robotics_hw_2::Pose2DStamped());

  ros::init(argc, argv, "Odometry_Localization_2D");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("odometry2D", 1000, odomCallback);
  ROS_INFO(" Subscribed to odometry2D ... ");

  pub = n.advertise<mobile_robotics_hw_2::Pose2DStamped>("pose2D", 1000, true);
  ROS_INFO(" Created publisher on pose2D... ");
  
  speedPub = n.advertise<mobile_robotics_hw_2::SpeedStamped>("speed", 1000, true);
  ROS_INFO(" Created publisher on speed... ");

  ros::spin();

  ROS_INFO(" Exiting cleanly! ");

  return 0;
}
