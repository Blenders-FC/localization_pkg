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

  std::vector<humanoid_nav_msgs::StepTarget> callFootstepPlanner();

  int robot_id;
};

#endif // SOCCER_LOCALIZATION_NODE_H
