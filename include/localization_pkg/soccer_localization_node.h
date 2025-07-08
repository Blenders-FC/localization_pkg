#ifndef SOCCER_LOCALIZATION_NODE_H
#define SOCCER_LOCALIZATION_NODE_H

#include <ros/ros.h>
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>  
#include <humanoid_nav_msgs/StepTarget.h>

#include "localization_pkg/GetRelativeFootsteps.h"

class SoccerLocalizationNode
{
public:
  SoccerLocalizationNode();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient footstep_client_;
  ros::ServiceServer trigger_service_;
  ros::Publisher absolute_pose_pub_;
<<<<<<< HEAD
  ros::Subscriber relative_pose_sub_;
=======
>>>>>>> origin/steps_planner

  bool triggerCallback(localization_pkg::GetRelativeFootsteps::Request &req,
                       localization_pkg::GetRelativeFootsteps::Response &res);

  std::vector<humanoid_nav_msgs::StepTarget> callFootstepPlanner(double start_x, double start_y, double start_theta,
                                                                 double goal_x, double goal_y, double goal_theta);
<<<<<<< HEAD
  void SoccerLocalizationNode::relativePoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
=======
  void SoccerLocalizationNode::relativePoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
>>>>>>> origin/steps_planner

  int robot_id;
  
  double FEET_DISTANCE = 0.1; // 10CM
  double FEET_OFFSET = 0.015;   // 1.5CM
  double FEET_SEPARATION = FEET_DISTANCE+ FEET_OFFSET; // 10 + 1.5 CM

<<<<<<< HEAD
  double abs_x_ = 0; //initial_x;
  double abs_y_ = 0; //initial_y;
  double abs_theta_ = 0; //initial_theta;
=======
  double abs_x_ = 0;
  double abs_y_ = 0;
  double abs_theta_ = 0;
>>>>>>> origin/steps_planner

};

#endif // SOCCER_LOCALIZATION_NODE_H
