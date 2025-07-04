#ifndef SOCCER_LOCALIZATION_NODE_H
#define SOCCER_LOCALIZATION_NODE_H

#include <ros/ros.h>
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <std_srvs/Trigger.h>

class LocalizationNode
{
public:
  LocalizationNode();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient footstep_client_;
  ros::ServiceServer trigger_service_;

  bool triggerCallback(std_srvs::Trigger::Request &req,
                       std_srvs::Trigger::Response &res);

  void callFootstepPlanner();

  int robot_id;
};

#endif // SOCCER_LOCALIZATION_NODE_H
