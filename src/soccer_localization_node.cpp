#include "localization_pkg/soccer_localization_node.h"

SoccerLocalizationNode::SoccerLocalizationNode()
{
  footstep_client_ = nh_.serviceClient<humanoid_nav_msgs::PlanFootsteps>("/plan_footsteps");

  nh_.param<int>("robot_id", robot_id, 0);
  nh_.param<int>("quadrant", quadrant, 0);

  std::string service_name = "robotis_" + std::to_string(robot_id) + "/soccer_localization_node/call_footstep_planner";

  trigger_service_ = nh_.advertiseService(service_name,
                                          &SoccerLocalizationNode::triggerCallback,
                                          this);

  relative_pose_sub_ = nh_.subscribe("/robotis_" + std::to_string(robot_id) + "/relative_pose_steps", 10, &SoccerLocalizationNode::relativePoseCallback, this);
  goal_params_sub_ = nh_.subscribe("/robotis_" + std::to_string(robot_id) + "/robot_pose/goal_params", 10, &SoccerLocalizationNode::goalParamsCallback, this);
  
  absolute_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/robotis_" + std::to_string(robot_id) + "/footstep_absolute_poses", 10);
  init_pose_pub_ = nh_.advertise<blenders_msgs::RobotPose>("/robotis_" + std::to_string(robot_id) + "/robot_pose/init_pose", 0);

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
    rel_x = rel_x * 2;
    double rel_y = msg->position.y;
    rel_y = (rel_y * 2) + FEET_SEPARATION;

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

void SoccerLocalizationNode::goalParamsCallback(const blenders_msgs::GoalParams::ConstPtr& msg)
{
    double dist = msg->distance.data;
    double ang = msg->angle.data;

    init_robot_pose_msg_.pose = calcInitRobotPosition(dist, ang);
    init_robot_pose_msg_.valid = true;
    init_pose_pub_.publish(init_robot_pose_msg_);
}

geometry_msgs::Pose SoccerLocalizationNode::calcInitRobotPosition(double distance, double angle_rad)
{
    geometry_msgs::Pose robot_pose;
    std::pair<int, int> post_coord;

    // Choose post (left or right)
    int post_index = (quadrant % 2 == 1) ? 0 : 1;

    // Post coordinates
    if (quadrant == 1 || quadrant == 4)
        post_coord = (post_index == 0) ? std::make_pair(POST_X_SUP, POST_Y_SUP)
                                       : std::make_pair(POST_X_SUP, POST_Y_SUB);
    else
        post_coord = (post_index == 0) ? std::make_pair(POST_X_SUB, POST_Y_SUB)
                                       : std::make_pair(POST_X_SUB, POST_Y_SUP);

    int base_heading_deg = (quadrant == 1 || quadrant == 4) ? 180 : 0;
    double global_angle_rad = (base_heading_deg + angle_rad * 180.0 / M_PI) * M_PI / 180.0;

    // Compute position
    robot_pose.position.x = post_coord.first + distance * std::cos(global_angle_rad);

    if (quadrant == 1 || quadrant == 3)
        robot_pose.position.y = post_coord.second + distance * std::sin(global_angle_rad);
    else
        robot_pose.position.y = post_coord.second - distance * std::sin(global_angle_rad);

    robot_pose.position.z = 0.0;

    // Orientation: yaw in quaternion
    robot_pose.orientation = tf::createQuaternionMsgFromYaw(0);

    return robot_pose;
}

// geometry_msgs::Point SoccerLocalizationNode::calcInitRobotPosition(double distance, double angle_rad)
// {
//     geometry_msgs::Point pos;

//     // Constants
//     const int POST_X_SUP = 900;
//     const int POST_Y_SUP = 170;
//     const int POST_Y_SUB = 430;

//     std::pair<int, int> post_coord;

//     // Which post to use (left or right)
//     int post_index = (quadrant % 2 == 1) ? 0 : 1;

//     // Determine post position
//     if (quadrant == 1 || quadrant == 4)
//         post_coord = (post_index == 0) ? std::make_pair(POST_X_SUP, POST_Y_SUP) :
//                                          std::make_pair(POST_X_SUP, POST_Y_SUB);
//     else
//         post_coord = (post_index == 0) ? std::make_pair(0, POST_Y_SUB) :
//                                          std::make_pair(0, POST_Y_SUP);

//     // Base heading (deg)
//     int base_heading_deg = 0;
//     if (quadrant == 1 || quadrant == 4)
//         base_heading_deg = 180;

//     // Final global angle in radians
//     double global_angle = (base_heading_deg + angle_rad * 180.0 / M_PI) * M_PI / 180.0;

//     // X is always added
//     pos.x = post_coord.first + distance * cos(global_angle);

//     // Y depends on quadrant
//     if (quadrant == 1 || quadrant == 3)
//         pos.y = post_coord.second + distance * sin(global_angle);
//     else
//         pos.y = post_coord.second - distance * sin(global_angle);

//     pos.z = 0.0;
//     return pos;
// }


std::pair<double, double> SoccerLocalizationNode::calculateRobotPositionFromPosts(const Post& post1, const Post& post2)
{
    double robot_x = NAN;
    double robot_y = NAN;

    double post_1_x = POST_X_SUP;
    double post_1_y = POST_Y_SUP;
    double post_2_x = POST_X_SUP;
    double post_2_y = POST_Y_SUB;

    double init_robot_x = std::abs(post1.distance * std::cos(post1.angle)) + post_1_x;
    double comp_robot_x = std::abs(post2.distance * std::cos(post2.angle)) + post_2_x;

    double init_robot_y, comp_robot_y;

    if (post1.distance > post2.distance) {
        init_robot_y = std::abs(post1.distance * std::sin(post1.angle) + post_1_y);
        comp_robot_y = std::abs(post2.distance * std::sin(post2.angle) + post_2_y);
    } else {
        init_robot_y = std::abs(post1.distance * std::sin(post1.angle) + post_2_y);
        comp_robot_y = std::abs(post2.distance * std::sin(post2.angle) + post_1_y);
    }

    if (std::abs(init_robot_x - comp_robot_x) < 30)
        robot_x = (init_robot_x + comp_robot_x) / 2.0;
    if (std::abs(init_robot_y - comp_robot_y) < 30)
        robot_y = (init_robot_y + comp_robot_y) / 2.0;

    return std::make_pair(robot_x, robot_y);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "soccer_localization_node");
  SoccerLocalizationNode node;

  ros::spin();
  return 0;
}
