#include "localization_pkg/soccer_localization_node.h"

SoccerLocalizationNode::SoccerLocalizationNode()
{
  footstep_client_ = nh_.serviceClient<humanoid_nav_msgs::PlanFootsteps>("/plan_footsteps");

  nh_.param<int>("robot_id", robot_id, 0);

  std::string service_name = "/robotis_" + std::to_string(robot_id) + "/soccer_localization_node/call_footstep_planner";

  trigger_service_ = nh_.advertiseService("soccer_localization_node/call_footstep_planner",
                                          &SoccerLocalizationNode::triggerCallback,
                                          this);

  ROS_INFO("SoccerLocalizationNode ready. Call %s to trigger footstep planning.", service_name.c_str());
}

bool SoccerLocalizationNode::triggerCallback(localization_pkg::GetRelativeFootsteps::Request &req,
  localization_pkg::GetRelativeFootsteps::Response &res)
{
  ROS_INFO("Trigger service called: calling footstep planner...");
  auto footsteps_plan = callFootstepPlanner();

  if (footsteps_plan.empty())
  {
    res.success = false;
    return true;
  }

  std::vector<humanoid_nav_msgs::StepTarget> relative_plan;

  for (size_t i = 0; i < footsteps_plan.size(); ++i)
  {
    humanoid_nav_msgs::StepTarget relative_step;
    if (i == 0)
    {
      relative_step.pose.x = footsteps_plan[i].pose.x;
      relative_step.pose.y = footsteps_plan[i].pose.y;
      relative_step.pose.theta = footsteps_plan[i].pose.theta;
    }
    else
    {
      relative_step.pose.x = footsteps_plan[i].pose.x - footsteps_plan[i-1].pose.x;
      relative_step.pose.y = footsteps_plan[i].pose.y - footsteps_plan[i-1].pose.y;
      relative_step.pose.theta = footsteps_plan[i].pose.theta - footsteps_plan[i-1].pose.theta;
    }
    relative_step.leg = footsteps_plan[i].leg;
    relative_plan.push_back(relative_step);
  }

  res.success = true;
  res.relative_plan = relative_plan;

  return true;
}

std::vector<humanoid_nav_msgs::StepTarget> SoccerLocalizationNode::callFootstepPlanner()
{
  humanoid_nav_msgs::PlanFootsteps srv;
  std::vector<humanoid_nav_msgs::StepTarget> empty_result;

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

    return srv.response.footsteps;
  }
  else
  {
    ROS_WARN("Failed to call /plan_footsteps service");
    return empty_result;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "soccer_localization_node");
  SoccerLocalizationNode node;

  ros::spin();
  return 0;
}
