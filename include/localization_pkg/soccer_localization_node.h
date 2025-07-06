#ifndef SOCCER_LOCALIZATION_NODE_H
#define SOCCER_LOCALIZATION_NODE_H

#include <ros/ros.h>
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include "localization_pkg/GetRelativeFootsteps.h"

class SoccerLocalizationNode
{
public:
  SoccerLocalizationNode();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient footstep_client_;
  ros::ServiceServer trigger_service_;

  bool triggerCallback(localization_pkg::GetRelativeFootsteps::Request &req,
                       localization_pkg::GetRelativeFootsteps::Response &res);

  std::vector<humanoid_nav_msgs::StepTarget> callFootstepPlanner(double goal_x, double goal_y, double goal_theta)

  int robot_id;
  
  double FEET_SEPARATION = 0.1; // 10CM
  double FEET_OFFSET = 0.015;   // 1.5CM
  double FEET_SEPARATION = FEET_SEPARATION + FEET_OFFSET; 
};

#endif // SOCCER_LOCALIZATION_NODE_H
