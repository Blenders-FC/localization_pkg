#include "localization_pkg/soccer_localization_node.h"

LocalizationNode::LocalizationNode()
{
  footstep_client_ = nh_.serviceClient<humanoid_nav_msgs::PlanFootsteps>("/plan_footsteps");

  nh_.param<int>("robot_id", robot_id, 0);

  std::string service_name = "/robotis_" + std::to_string(robot_id) + "/soccer_localization_node/call_footstep_planner";

  trigger_service_ = nh_.advertiseService(service_name,
                                          &LocalizationNode::triggerCallback,
                                          this);

  ROS_INFO("SoccerLocalizationNode ready. Call %s to trigger footstep planning.", service_name.c_str());
}


bool LocalizationNode::triggerCallback(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res)
{
  ROS_INFO("Trigger service called: calling footstep planner...");
  callFootstepPlanner();
  res.success = true;
  res.message = "Footstep planner called";
  return true;
}

void LocalizationNode::callFootstepPlanner()
{
  humanoid_nav_msgs::PlanFootsteps srv;

  // start pose
  srv.request.start.x = 0.0;
  srv.request.start.y = 0.0;
  srv.request.start.theta = 0.0;

  // goal pose
  srv.request.goal.x = 1.0;
  srv.request.goal.y = 0.0;
  srv.request.goal.theta = 0.0;

  if (footstep_client_.call(srv))
  {
    ROS_INFO("Footstep plan succeeded: %s", srv.response.result ? "True" : "False");
    ROS_INFO("Number of footsteps: %zu", srv.response.footsteps.size());
    for (size_t i = 0; i < srv.response.footsteps.size(); ++i)
    {
      const auto& step = srv.response.footsteps[i];
      ROS_INFO("Step %zu: x=%.3f y=%.3f theta=%.3f leg=%d",
               i,
               step.pose.x,
               step.pose.y,
               step.pose.theta,
               step.leg);
    }
  }
  else
  {
    ROS_WARN("Failed to call /plan_footsteps service");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "soccer_localization_node");
  LocalizationNode node;

  ros::spin();
  return 0;
}
