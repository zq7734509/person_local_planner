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
#ifndef PERSON_LOCAL_PLANNER_PERSON_PLANNER_ROS_H_
#define PERSON_LOCAL_PLANNER_PERSON_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <person_local_planner/PersonPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <person_local_planner/person_planner.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>

namespace person_local_planner {

  //定义行人区域参数的结构:1.半短轴长度 2.离心率 3.在odom坐标系下的位置x坐标 4.在odom坐标系下的位置y坐标 5.在odom坐标系下的朝向角


  /**
   * @class PersonPlannerROS
   * @brief ROS Wrapper for the PersonPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   */
  class PersonPlannerROS : public nav_core::BaseLocalPlanner {
    public:
      /**
       * @brief  Constructor for PersonPlannerROS wrapper
       */
      PersonPlannerROS();

      /**
       * @brief  Constructs the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~PersonPlannerROS();

      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


      /**
       * @brief  Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base, using dynamic window approach
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool personComputeVelocityCommands(tf::Stamped<tf::Pose>& global_pose, geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Set the plan that the controller is following
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();



      bool isInitialized() {
        return initialized_;
      }



    private:
      /**
       * @brief Callback to update the local planner's parameters based on dynamic reconfigure
       */
      void reconfigureCB(PersonPlannerConfig &config, uint32_t level);

      void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

      void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);


      //行人和机器人odom回调函数
      void personOdomCallback(const nav_msgs::Odometry& msg);
      void robotOdomCallback(const nav_msgs::Odometry& msg);

      //计算行人区域参数
      Eigen::Matrix<double, 5, 1> calculatePersonZone(nav_msgs::Odometry person, nav_msgs::Odometry robot, double& angle);

      //绘制行人区域
      Eigen::Matrix<double, 5, 1> drawPersonZone(nav_msgs::Odometry person, nav_msgs::Odometry robot);
      void drawEmptyZone();

      //判断机器人是否在椭圆范围内
      //bool isRobotInsideZone(nav_msgs::Odometry person, nav_msgs::Odometry robot, std::vector<geometry_msgs::Point> footprint, person_zone_param pz);
      //bool isRobotInsideZone(nav_msgs::Odometry person, nav_msgs::Odometry robot, person_zone_param pz);

      //判断机器人在行人的哪一侧
      //std::string onWhichSide(nav_msgs::Odometry person, nav_msgs::Odometry robot);

      //int isRecover(int& last_state, int current_state);
      //int recover_flag;


      tf::TransformListener* tf_; ///< @brief Used for transforming point clouds

      // for visualisation, publishers of global and local plan
      ros::Publisher g_plan_pub_, l_plan_pub_;

      // 订阅行人和机器人的odom
      ros::Subscriber person_odom_sub_,robot_odom_sub_;

      // 行人区域marker发布器
      ros::Publisher front_zone_pub_, back_zone_pub_;

      base_local_planner::LocalPlannerUtil planner_util_;

      boost::shared_ptr<PersonPlanner> dp_; ///< @brief The trajectory controller

      costmap_2d::Costmap2DROS* costmap_ros_;

      dynamic_reconfigure::Server<PersonPlannerConfig> *dsrv_;
      person_local_planner::PersonPlannerConfig default_config_;
      bool setup_;
      tf::Stamped<tf::Pose> current_pose_;

      base_local_planner::LatchedStopRotateController latchedStopRotateController_;


      bool initialized_;


      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;

      //行人和机器人的odom对象
      nav_msgs::Odometry person_odom_,robot_odom_;

      //行人区域marker对象
      visualization_msgs::Marker front_marker_, back_marker_;

      //椭圆参数对象
      Eigen::Matrix<double, 5, 1> person_zone_;

      //机器人在行人哪侧的标志位,0代表未进入区域，1代表在行人左侧，2代表在行人右侧
      //int side_;

      //int last_state_;
      double relative_angle_;
  };
};
#endif
