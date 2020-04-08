/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <person_local_planner/person_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

#include "tf/transform_datatypes.h"


//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(person_local_planner::PersonPlannerROS, nav_core::BaseLocalPlanner)

namespace person_local_planner {

  void PersonPlannerROS::reconfigureCB(PersonPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }

      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_trans_vel = config.max_trans_vel;
      limits.min_trans_vel = config.min_trans_vel;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_rot_vel = config.max_rot_vel;
      limits.min_rot_vel = config.min_rot_vel;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_limit_trans = config.acc_limit_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.rot_stopped_vel = config.rot_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  PersonPlannerROS::PersonPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false) {

  }

  void PersonPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

      //构造行人和机器人odom订阅器
      ros::NodeHandle private_nh_person("");
      person_odom_sub_ = private_nh_person.subscribe("person_odom", 1000, &PersonPlannerROS::personOdomCallback, this);
      robot_odom_sub_ = private_nh_person.subscribe("odom", 1000, &PersonPlannerROS::robotOdomCallback, this);

      //构造行人区域marker发布器
      front_zone_pub_ = private_nh_person.advertise<visualization_msgs::Marker>("front_zone_marker", 10);
      back_zone_pub_ = private_nh_person.advertise<visualization_msgs::Marker>("back_zone_marker", 10);
      relative_angle_ = 0;

      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<PersonPlanner>(new PersonPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      latchedStopRotateController_.initialize(name);

      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<PersonPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<PersonPlannerConfig>::CallbackType cb = boost::bind(&PersonPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  //行人和机器人odom回调函数实现
  void PersonPlannerROS::personOdomCallback(const nav_msgs::Odometry& msg)
  {
    person_odom_ = msg;
  }
  void PersonPlannerROS::robotOdomCallback(const nav_msgs::Odometry& msg)
  {
    robot_odom_ = msg;
  }

  //计算行人区域函数实现
  Eigen::Matrix<double, 5, 1> PersonPlannerROS::calculatePersonZone(nav_msgs::Odometry p_odom, nav_msgs::Odometry r_odom, double& angle)
  {
    Eigen::Matrix<double, 5, 1> pz;

    //将行人和机器人的四元数转换为平面角
    tf::Quaternion p_quat,r_quat;
    tf::quaternionMsgToTF(p_odom.pose.pose.orientation, p_quat);
    tf::quaternionMsgToTF(r_odom.pose.pose.orientation, r_quat);
    double roll, pitch, p_yaw, r_yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(p_quat).getRPY(roll, pitch, p_yaw);
    tf::Matrix3x3(r_quat).getRPY(roll, pitch, r_yaw);

    pz(0) = 0.8;
    //计算行人和机器人线速度在两个坐标轴上的分量
    double p_vel[2];
    double r_vel[2];
    p_vel[0] = p_odom.twist.twist.linear.x * cos(p_yaw);
    p_vel[1] = p_odom.twist.twist.linear.x * sin(p_yaw);
    r_vel[0] = r_odom.twist.twist.linear.x * cos(r_yaw);
    r_vel[1] = r_odom.twist.twist.linear.x * sin(r_yaw);

    //ROS_INFO("r_x=%f,r_y=%f,r_yaw=%f,p_x=%f,p_y=%f,p_yaw:%f",r_vel[0],r_vel[1],r_yaw,p_vel[0],p_vel[1],p_yaw);
    //计算相对速度和相对朝向
    double relative_vel = sqrt((p_vel[0]-r_vel[0])*(p_vel[0]-r_vel[0])+(p_vel[1]-r_vel[1])*(p_vel[1]-r_vel[1]));
    double relative_vel_yaw = atan2((2*p_vel[1]-r_vel[1]),(2*p_vel[0]-r_vel[0]));
    angle = relative_vel_yaw;
    //double relative_vel_yaw = atan2((r_odom.pose.pose.position.y-p_odom.pose.pose.position.y),(r_odom.pose.pose.position.x-p_odom.pose.pose.position.x));

    //将椭圆的主方向控制在(-pi/2,pi/2]中
    if(relative_vel_yaw > M_PI/2)
      relative_vel_yaw = relative_vel_yaw - M_PI;
    else if(relative_vel_yaw <= -M_PI/2)
      relative_vel_yaw = M_PI + relative_vel_yaw;

    //计算离心率
    double lamda = 1.05;
    double r_max_vel = 0.5;
    double p_max_vel = 0.5;
    //pz(1) = relative_vel / (lamda*(r_max_vel+p_max_vel));
    pz(1) = relative_vel / (lamda*relative_vel);
    
    pz(2) = p_odom.pose.pose.position.x;
    pz(3) = p_odom.pose.pose.position.y;
    pz(4) = relative_vel_yaw;

    double distance = sqrt((p_odom.pose.pose.position.x - r_odom.pose.pose.position.x)*(p_odom.pose.pose.position.x - r_odom.pose.pose.position.x) + (p_odom.pose.pose.position.y - r_odom.pose.pose.position.y)*(p_odom.pose.pose.position.y - r_odom.pose.pose.position.y));
    ROS_INFO("relative_distance:%f",distance);
    return pz;
  }

  //绘制行人区域函数实现
  Eigen::Matrix<double, 5, 1> PersonPlannerROS::drawPersonZone(nav_msgs::Odometry p_odom, nav_msgs::Odometry r_odom)
  {
    Eigen::Matrix<double, 5, 1> pz = calculatePersonZone(p_odom,r_odom, relative_angle_);
    front_marker_.header.frame_id = "/odom";
    front_marker_.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    front_marker_.ns = "basic_shapes";
    front_marker_.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    front_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;       // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    front_marker_.mesh_resource = "package://smart_robot_navigation/meshes/front.dae";
    front_marker_.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    front_marker_.pose.position = p_odom.pose.pose.position;
    front_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(relative_angle_);
    front_marker_.scale.x = 2 * pz(0) /(sqrt(1-pz(1)*pz(1)));
    front_marker_.scale.y = 2 * pz(0);
    front_marker_.scale.z = 0.25;
    // Set the color -- be sure to set alpha to something non-zero!
    front_marker_.color.r = 1.0f;
    front_marker_.color.g = 0.0f;
    front_marker_.color.b = 0.0f;
    front_marker_.color.a = 1.0f;
    //marker.lifetime = ros::Duration();
    front_zone_pub_.publish(front_marker_);

    back_marker_.header.frame_id = "/odom";
    back_marker_.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    back_marker_.ns = "basic_shapes";
    back_marker_.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    back_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;       // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    back_marker_.mesh_resource = "package://smart_robot_navigation/meshes/back.dae";
    back_marker_.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    back_marker_.pose.position = p_odom.pose.pose.position;
    back_marker_.pose.orientation = tf::createQuaternionMsgFromYaw(relative_angle_);
    back_marker_.scale.x = 2 * pz(0);
    back_marker_.scale.y = 2 * pz(0);
    back_marker_.scale.z = 0.25;
    // Set the color -- be sure to set alpha to something non-zero!
    back_marker_.color.r = 1.0f;
    back_marker_.color.g = 0.0f;
    back_marker_.color.b = 0.0f;
    back_marker_.color.a = 1.0f;
    //marker.lifetime = ros::Duration();
    back_zone_pub_.publish(back_marker_);
    return pz;
  }

  void PersonPlannerROS::drawEmptyZone()
  {
    back_marker_.header.frame_id = "/odom";
    back_marker_.header.stamp = ros::Time::now();
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    back_marker_.ns = "basic_shapes";
    back_marker_.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    back_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    back_marker_.mesh_resource = "package://smart_robot_navigation/meshes/back.dae";
    back_marker_.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    back_marker_.pose.orientation.w = 1.0;

    back_marker_.scale.x = 1;
    back_marker_.scale.y = 1;
    back_marker_.scale.z = 0.1;
    // Set the color -- be sure to set alpha to something non-zero!
    back_marker_.color.r = 1.0f;
    back_marker_.color.g = 0.0f;
    back_marker_.color.b = 0.0f;
    back_marker_.color.a = 0.0f; 
    //marker.lifetime = ros::Duration();
    back_zone_pub_.publish(back_marker_);
    front_zone_pub_.publish(back_marker_);
  }


  bool PersonPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool PersonPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }
  }

  void PersonPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  void PersonPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  PersonPlannerROS::~PersonPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }



  bool PersonPlannerROS::personComputeVelocityCommands(tf::Stamped<tf::Pose> &global_pose, geometry_msgs::Twist& cmd_vel) {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    person_zone_ = drawPersonZone(person_odom_, robot_odom_);
    //bool flag = isRobotInsideZone(person_odom_, robot_odom_, costmap_ros_->getRobotFootprint(), person_zone_);

    /*
    bool flag = isRobotInsideZone(person_odom_, robot_odom_, person_zone_);
    if(flag){
      std::string side = onWhichSide(person_odom_, robot_odom_);
      if(side == "left")
        side_ = 1;
      else if(side == "right")
        side_ = 2;
      else  //person_back
        side_ = 3;
    }
    else{
      side_ = 0;
    }
    recover_flag = isRecover(last_state_, side_);
    dp_->robotSide_ = side_;
    dp_->recover_ = recover_flag;
    dp_->relative_angle_ = relative_angle_;
    */

    dp_->pz_ = person_zone_;
    dp_->person_odom_ = person_odom_;
    dp_->robot_odom_ = robot_odom_;

    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

    // call with updated footprint
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      ROS_DEBUG_NAMED("person_local_planner",
          "The person local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("person_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      tf::Stamped<tf::Pose> p =
              tf::Stamped<tf::Pose>(tf::Pose(
                      tf::createQuaternionFromYaw(p_th),
                      tf::Point(p_x, p_y, 0.0)),
                      ros::Time::now(),
                      costmap_ros_->getGlobalFrameID());
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }

    //publish information to the visualizer

    publishLocalPlan(local_plan);
    return true;
  }




  bool PersonPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either person sampling control or stop and rotate control, depending on whether we have been close enough to goal
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if ( ! planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty()) {
      ROS_WARN_NAMED("person_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("person_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in person_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {

      //机器人到达目标点后
      drawEmptyZone();

      //publish an empty plan because we've reached our goal position
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      return latchedStopRotateController_.computeVelocityCommandsStopRotate(
          cmd_vel,
          limits.getAccLimits(),
          dp_->getSimPeriod(),
          &planner_util_,
          odom_helper_,
          current_pose_,
          boost::bind(&PersonPlanner::checkTrajectory, dp_, _1, _2, _3));
    } else {
      bool isOk = personComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) {
        publishGlobalPlan(transformed_plan);
      } else {
        ROS_WARN_NAMED("person_local_planner", "Person planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }


};
