#include "localization_pkg/soccer_localization_node.h"

SoccerLocalizationNode::SoccerLocalizationNode()
{
  footstep_client_ = nh_.serviceClient<humanoid_nav_msgs::PlanFootsteps>("/plan_footsteps");

  nh_.param<int>("robot_id", robot_id, 0);

  std::string service_name = "robotis_" + std::to_string(robot_id) + "/soccer_localization_node/call_footstep_planner";

  trigger_service_ = nh_.advertiseService(service_name,
                                          &SoccerLocalizationNode::triggerCallback,
                                          this);

  relative_pose_sub_ = nh_.subscribe("/robotis_" + std::to_string(robot_id) + "/relative_pose_steps", 10, &SoccerLocalizationNode::relativePoseCallback, this);
  absolute_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/robotis_" + std::to_string(robot_id) + "/footstep_absolute_poses", 10);

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

    return srv.response.footsteps;
  }
  else
  {
    return {};
  }
}

void SoccerLocalizationNode::relativePoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    
    double rel_x = msg->position.x;
    double rel_y = msg->position.y;

    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    double roll, pitch, rel_theta;
    tf::Matrix3x3(q).getRPY(roll, pitch, rel_theta);

    // compute absolute pose
    double abs_x = abs_x_ + rel_x * cos(abs_theta_) - rel_y * sin(abs_theta_);
    double abs_y = abs_y_ + rel_x * sin(abs_theta_) + rel_y * cos(abs_theta_);
    double abs_theta = abs_theta_ + rel_theta;

    // normalize angle to [-pi, pi]
    abs_theta = atan2(sin(abs_theta), cos(abs_theta));

    // update internal state
    abs_x_ = abs_x;
    abs_y_ = abs_y;
    abs_theta_ = abs_theta;

    // publish absolute coordinates
    geometry_msgs::Pose abs_pose_msg;
    abs_pose_msg.position.x = abs_x;
    abs_pose_msg.position.y = abs_y;
    abs_pose_msg.position.z = 0.0;

    abs_pose_msg.orientation = tf::createQuaternionMsgFromYaw(abs_theta);

    absolute_pose_pub_.publish(abs_pose_msg);

    ROS_INFO("Published absolute pose: x=%.3f, y=%.3f, theta=%.3f", abs_x, abs_y, abs_theta);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "soccer_localization_node");
  SoccerLocalizationNode node;

  ros::spin();
  return 0;
}
