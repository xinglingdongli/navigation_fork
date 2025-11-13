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

#include <dwa_local_planner/dwa_planner_ros.h>
#include <Eigen/Core>
#include <cmath>
#include <angles/angles.h>

#include <ros/console.h>

#include <pluginlib/class_list_macros.hpp>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <costmap_2d/cost_values.h>

#include <nav_core/parameter_magic.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_local_planner::DWAPlannerROS, nav_core::BaseLocalPlanner)

namespace dwa_local_planner {

  void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) {
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
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;
      planner_util_.reconfigureCB(limits, config.restore_defaults);

      // update dwa specific configuration
      dp_->reconfigure(config);
  }

  DWAPlannerROS::DWAPlannerROS() : initialized_(false),
      odom_helper_("odom"), setup_(false), recovery_attempts_(0), in_final_approach_(false) {

  }

  void DWAPlannerROS::initialize(
      std::string name,
      tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* costmap_ros) {
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));

      if( private_nh.getParam( "odom_topic", odom_topic_ ))
      {
        odom_helper_.setOdomTopic( odom_topic_ );
      }
      
      initialized_ = true;

      // Warn about deprecated parameters -- remove this block in N-turtle
      nav_core::warnRenamedParameter(private_nh, "max_vel_trans", "max_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_trans", "min_trans_vel");
      nav_core::warnRenamedParameter(private_nh, "max_vel_theta", "max_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "min_vel_theta", "min_rot_vel");
      nav_core::warnRenamedParameter(private_nh, "acc_lim_trans", "acc_limit_trans");
      nav_core::warnRenamedParameter(private_nh, "theta_stopped_vel", "rot_stopped_vel");

      dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = [this](auto& config, auto level){ reconfigureCB(config, level); };
      dsrv_->setCallback(cb);
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }
  
  bool DWAPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");
    return dp_->setPlan(orig_global_plan);
  }

  bool DWAPlannerROS::isGoalReached() {
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

  void DWAPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, l_plan_pub_);
  }


  void DWAPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
    base_local_planner::publishPlan(path, g_plan_pub_);
  }

  DWAPlannerROS::~DWAPlannerROS(){
    //make sure to clean things up
    delete dsrv_;
  }



  bool DWAPlannerROS::dwaComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) {
    // dynamic window sampling approach to get useful velocity commands
    if(! isInitialized()){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    /* For timing uncomment
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    */

    //compute what trajectory to drive along
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();
    
    // call with updated footprint
    base_local_planner::Trajectory path = dp_->findBestPath(global_pose, robot_vel, drive_cmds);
    // ROS_DEBUG_NAMED("dwa_local_planner", "Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);

    /* For timing uncomment
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_INFO("Cycle time: %.9f", t_diff);
    */

    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

    //if we cannot move... tell someone
    std::vector<geometry_msgs::PoseStamped> local_plan;
    if(path.cost_ < 0) {
      ROS_DEBUG_NAMED("dwa_local_planner",
          "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      local_plan.clear();
      publishLocalPlan(local_plan);
      return false;
    }

    ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", 
                    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    // Fill out the local plan
    for(unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);

      geometry_msgs::PoseStamped p;
      p.header.frame_id = costmap_ros_->getGlobalFrameID();
      p.header.stamp = ros::Time::now();
      p.pose.position.x = p_x;
      p.pose.position.y = p_y;
      p.pose.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, p_th);
      tf2::convert(q, p.pose.orientation);
      local_plan.push_back(p);
    }

    //publish information to the visualizer

    publishLocalPlan(local_plan);
    return true;
  }




  bool DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
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
      ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

    // Get goal pose and calculate distance
    geometry_msgs::PoseStamped goal_pose;
    if (!planner_util_.getGoal(goal_pose)) {
      ROS_ERROR("Could not get goal pose");
      return false;
    }
    
    double goal_x = goal_pose.pose.position.x;
    double goal_y = goal_pose.pose.position.y;
    double distance_to_goal = base_local_planner::getGoalPositionDistance(current_pose_, goal_x, goal_y);
    
    // Check if we should use final straight line approach
    if (distance_to_goal <= FINAL_APPROACH_DISTANCE) {
      if (!in_final_approach_) {
        ROS_INFO("DWA: Entering final approach mode (distance: %.3f m)", distance_to_goal);
        in_final_approach_ = true;
      }
      
      // Use straight line approach for final 20cm
      bool success = finalStraightLineApproach(cmd_vel, goal_pose);
      if (success) {
        std::vector<geometry_msgs::PoseStamped> local_plan;
        publishLocalPlan(local_plan);
        publishGlobalPlan(transformed_plan);
      }
      return success;
    } else {
      // Reset final approach flag when far from goal
      if (in_final_approach_) {
        ROS_INFO("DWA: Exiting final approach mode");
        in_final_approach_ = false;
      }
    }

    if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)) {
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
          [this](auto pos, auto vel, auto vel_samples){ return dp_->checkTrajectory(pos, vel, vel_samples); });
    } else {
      bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      if (isOk) {
        publishGlobalPlan(transformed_plan);
      } else {
        ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
        
        // Recovery behavior: Try to move backward slightly if stuck in inflation layer
        if (attemptRecoveryManeuver(cmd_vel)) {
          ROS_INFO_NAMED("dwa_local_planner", "Executing recovery maneuver");
          std::vector<geometry_msgs::PoseStamped> empty_plan;
          publishGlobalPlan(empty_plan);
          return true;
        }
        
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }
      return isOk;
    }
  }

  bool DWAPlannerROS::finalStraightLineApproach(geometry_msgs::Twist& cmd_vel, const geometry_msgs::PoseStamped& goal_pose) {
    static ros::Time straight_line_start_time;
    static bool straight_line_phase = false;
    static bool rotation_phase = false;
    
    // Calculate distance and angle to goal
    double dx = goal_pose.pose.position.x - current_pose_.pose.position.x;
    double dy = goal_pose.pose.position.y - current_pose_.pose.position.y;
    double distance_to_goal = sqrt(dx * dx + dy * dy);
    
    // Get current robot orientation
    double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
    double goal_yaw = tf2::getYaw(goal_pose.pose.orientation);
    double angle_to_goal = atan2(dy, dx);
    
    ros::Time current_time = ros::Time::now();
    
    // State machine for final approach
    if (!straight_line_phase && !rotation_phase) {
      // Initialize straight line phase
      straight_line_start_time = current_time;
      straight_line_phase = true;
      rotation_phase = false;
      ROS_INFO("DWA: Starting straight line approach phase (distance: %.3f m)", distance_to_goal);
    }
    
    if (straight_line_phase) {
      double elapsed_time = (current_time - straight_line_start_time).toSec();
      
      // Move straight towards goal for exactly 1 second
      if (elapsed_time < 1.0) {
        // Calculate normalized direction vector
        double norm = sqrt(dx * dx + dy * dy);
        if (norm > 0.01) {
          cmd_vel.linear.x = FINAL_APPROACH_LINEAR_VEL * (dx / norm);
          cmd_vel.linear.y = FINAL_APPROACH_LINEAR_VEL * (dy / norm);
          cmd_vel.angular.z = 0.0;
          
          ROS_DEBUG("DWA: Straight line movement - elapsed: %.2fs", elapsed_time);
          return true;
        }
      }
      
      // Transition to rotation phase after 1 second
      straight_line_phase = false;
      rotation_phase = true;
      ROS_INFO("DWA: Transitioning to rotation alignment phase");
    }
    
    if (rotation_phase) {
      // Check if position is close enough to goal
      // if (distance_to_goal > 0.05) {
      //   // Continue moving towards goal if still far
      //   double norm = sqrt(dx * dx + dy * dy);
      //   if (norm > 0.01) {
      //     cmd_vel.linear.x = 0.1 * (dx / norm);  // Slower approach
      //     cmd_vel.linear.y = 0.1 * (dy / norm);
      //     cmd_vel.angular.z = 0.0;
      //     return true;
      //   }
      // }
      
      // Align orientation to goal orientation
      double angle_diff = angles::shortest_angular_distance(current_yaw, goal_yaw);
      
      if (fabs(angle_diff) > 0.1) {  // 0.1 rad ≈ 5.7 degrees tolerance
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = (angle_diff > 0) ? 0.3 : -0.3;
        
        ROS_DEBUG("DWA: Aligning orientation - angle diff: %.3f rad", angle_diff);
        return true;
      }
      
      // Goal reached - both position and orientation
      ROS_INFO("DWA: Final approach completed successfully!");
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      
      // Reset state for next goal
      straight_line_phase = false;
      rotation_phase = false;
      
      return true;
    }
    
    // Fallback - stop
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool DWAPlannerROS::attemptRecoveryManeuver(geometry_msgs::Twist& cmd_vel) {
    ros::Time current_time = ros::Time::now();
    
    // Reset recovery attempts if enough time has passed
    if ((current_time - last_recovery_time_).toSec() > RECOVERY_TIMEOUT) {
      recovery_attempts_ = 0;
    }
    
    // Check if we've exceeded maximum recovery attempts
    if (recovery_attempts_ >= MAX_RECOVERY_ATTEMPTS) {
      ROS_ERROR_NAMED("dwa_local_planner", "Maximum recovery attempts reached. Robot may be stuck.");
      return false;
    }
    
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    unsigned int mx, my;
    
    if (!costmap->worldToMap(current_pose_.pose.position.x, current_pose_.pose.position.y, mx, my)) {
      return false;
    }
    
    unsigned char cost = costmap->getCost(mx, my);
    
    // Only attempt recovery if in inflation layer
    if (cost > costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost < costmap_2d::LETHAL_OBSTACLE) {
      recovery_attempts_++;
      last_recovery_time_ = current_time;
      
      ROS_WARN_NAMED("dwa_local_planner", 
                     "Recovery attempt %d/%d: Robot in inflation layer (cost: %d)", 
                     recovery_attempts_, MAX_RECOVERY_ATTEMPTS, cost);
      
      // Different recovery strategies based on attempt number
      switch (recovery_attempts_) {
        case 1:
          // First attempt: small backward movement
          cmd_vel.linear.x = -0.05;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
          ROS_INFO_NAMED("dwa_local_planner", "Recovery: Moving backward");
          break;
          
        case 2:
          // Second attempt: rotate in place
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.3;
          ROS_INFO_NAMED("dwa_local_planner", "Recovery: Rotating in place");
          break;
          
        case 3:
          // Third attempt: backward with rotation
          cmd_vel.linear.x = -0.03;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = -0.2;
          ROS_INFO_NAMED("dwa_local_planner", "Recovery: Backward with rotation");
          break;
          
        default:
          return false;
      }
      
      return true;
    }
    
    // If not in inflation layer, reset recovery attempts
    if (recovery_attempts_ > 0) {
      ROS_INFO_NAMED("dwa_local_planner", "Robot escaped inflation layer, resetting recovery attempts");
      recovery_attempts_ = 0;
    }
    
    return false;
  }

};
