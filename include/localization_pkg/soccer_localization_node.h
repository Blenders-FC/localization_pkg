#ifndef SOCCER_LOCALIZATION_NODE_H
#define SOCCER_LOCALIZATION_NODE_H

#include <utility>
#include <cmath>
#include <ros/ros.h>
#include <humanoid_nav_msgs/PlanFootsteps.h>
#include <geometry_msgs/PoseArray.h>
#include <blenders_msgs/RobotPose.h>
#include <blenders_msgs/GoalParams.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>  
#include <humanoid_nav_msgs/StepTarget.h>
#include "localization_pkg/GetRelativeFootsteps.h"


struct Post {
    double distance;
    double angle;
    Post(double d, double a) : distance(d), angle(a) {}
};

class SoccerLocalizationNode
{
public:
  SoccerLocalizationNode();

private:
  ros::NodeHandle nh_;
  ros::ServiceClient footstep_client_;
  ros::ServiceServer trigger_service_;
  ros::Subscriber relative_pose_sub_;
  ros::Subscriber goal_params_sub_;
  ros::Publisher absolute_pose_pub_;
  ros::Publisher init_pose_pub_;

  blenders_msgs::RobotPose init_robot_pose_msg_;

  bool triggerCallback(localization_pkg::GetRelativeFootsteps::Request &req,
                       localization_pkg::GetRelativeFootsteps::Response &res);

  std::vector<humanoid_nav_msgs::StepTarget> callFootstepPlanner(double start_x, double start_y, double start_theta,
                                                                 double goal_x, double goal_y, double goal_theta);

  void relativePoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void goalParamsCallback(const blenders_msgs::GoalParams::ConstPtr& msg);

  std::pair<double, double> calculateRobotPositionFromPosts(const Post& post1, const Post& post2);
  geometry_msgs::Pose calcInitPositionFromAngles(double robot_angle);
  geometry_msgs::Pose calcInitRobotPosition(double distance, double angle_rad);

  int robot_id;
  int quadrant;
  
  double FEET_DISTANCE = 0.1; // 10CM
  double FEET_OFFSET = 0.015;   // 1.5CM
  double FEET_SEPARATION = FEET_DISTANCE+ FEET_OFFSET; // 10 + 1.5 CM

  double abs_x_ = 0; //initial_x;
  double abs_y_ = 0; //initial_y;
  double abs_theta_ = 0; //initial_theta;
  double complementary_angle;

  // Constants
  const int POST_X_SUP = 900;
  const int POST_X_SUB = 0;
  const int POST_Y_SUP = 170;
  const int POST_Y_SUB = 430;
  const int FIELD_HEIGHT = 600;
};

#endif // SOCCER_LOCALIZATION_NODE_H
