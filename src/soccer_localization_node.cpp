#include "localization_pkg/soccer_localization_node.h"

SoccerLocalizationNode::SoccerLocalizationNode()
{
  footstep_client_ = nh_.serviceClient<humanoid_nav_msgs::PlanFootsteps>("/plan_footsteps");

  nh_.param<int>("robot_id", robot_id, 0);

  std::string service_name = "robotis_" + std::to_string(robot_id) + "/soccer_localization_node/call_footstep_planner";

  trigger_service_ = nh_.advertiseService(service_name,
                                          &SoccerLocalizationNode::triggerCallback,
                                          this);

  footstep_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("footstep_absolute_poses", 10);

  ROS_INFO("SoccerLocalizationNode ready. Call %s to trigger footstep planning.", service_name.c_str());
}

bool SoccerLocalizationNode::triggerCallback(localization_pkg::GetRelativeFootsteps::Request &req,
                                             localization_pkg::GetRelativeFootsteps::Response &res)
{
  ROS_INFO("Trigger service called: start(%.3f, %.3f, %.3f), goal(%.3f, %.3f, %.3f)",
           req.start_x, req.start_y, req.start_theta,
           req.goal_x, req.goal_y, req.goal_theta);

  auto footsteps_plan = callFootstepPlanner(req.start_x, req.start_y, req.start_theta,
                                            req.goal_x, req.goal_y, req.goal_theta);

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

    relative_step.pose.x = relative_step.pose.x / 2;
    if (fabs(relative_step.pose.y) > FEET_SEPARATION)
      relative_step.pose.y = (relative_step.pose.y - FEET_SEPARATION) / 2;
    else
      relative_step.pose.y = 0.0;

    relative_step.leg = footsteps_plan[i].leg;
    relative_plan.push_back(relative_step);
  }

  res.success = true;
  res.relative_plan = relative_plan;

  return true;
}

std::vector<humanoid_nav_msgs::StepTarget> SoccerLocalizationNode::callFootstepPlanner(
                                          double start_x, double start_y, double start_theta,
                                          double goal_x, double goal_y, double goal_theta)
{
  humanoid_nav_msgs::PlanFootsteps srv;

  srv.request.start.x = start_x;
  srv.request.start.y = start_y;
  srv.request.start.theta = start_theta;

  srv.request.goal.x = goal_x;
  srv.request.goal.y = goal_y;
  srv.request.goal.theta = goal_theta;

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
    publishFootstepPoses(srv.response.footsteps, start_x, start_y, start_theta);

    return srv.response.footsteps;
  }
  else
  {
    return {};
  }
}

void SoccerLocalizationNode::publishFootstepPoses(const std::vector<humanoid_nav_msgs::StepTarget>& footsteps,
                                                  double start_x, double start_y, double start_theta)
{
  geometry_msgs::PoseArray pose_array_msg;
  pose_array_msg.header.stamp = ros::Time::now();

  double x_ini = start_x;
  double y_ini = start_y;
  double theta = start_theta;

  for (const auto& step : footsteps)
  {
    // transform relative footsteps to absolute coordinates
    double rel_x = step.pose.x;
    double rel_y = step.pose.y;
    double rel_theta = step.pose.theta;

    double abs_x = x + rel_x * cos(theta) - rel_y * sin(theta);
    double abs_y = y + rel_x * sin(theta) + rel_y * cos(theta);
    double abs_theta = theta + rel_theta;

    // normalize angle between -pi and pi
    abs_theta = atan2(sin(abs_theta), cos(abs_theta));

    geometry_msgs::Pose pose;
    pose.position.x = abs_x;
    pose.position.y = abs_y;
    pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, abs_theta);
    pose.orientation = tf2::toMsg(q);

    pose_array_msg.poses.push_back(pose);

    // update current pose for next step
    x = abs_x;
    y = abs_y;
    theta = abs_theta;
  }

  footstep_pose_pub_.publish(pose_array_msg);
  ROS_INFO("Published %zu footstep absolute poses.", pose_array_msg.poses.size());
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "soccer_localization_node");
  SoccerLocalizationNode node;

  ros::spin();
  return 0;
}
