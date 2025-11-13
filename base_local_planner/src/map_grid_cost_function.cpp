/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/map_grid_cost_function.h>
#include <costmap_2d/cost_values.h>
#include <cmath>

namespace base_local_planner {

MapGridCostFunction::MapGridCostFunction(costmap_2d::Costmap2D* costmap,
    double xshift,
    double yshift,
    bool is_local_goal_function,
    CostAggregationType aggregationType) :
    costmap_(costmap),
    map_(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()),
    aggregationType_(aggregationType),
    xshift_(xshift),
    yshift_(yshift),
    is_local_goal_function_(is_local_goal_function),
    stop_on_failure_(true) {}

void MapGridCostFunction::setTargetPoses(std::vector<geometry_msgs::PoseStamped> target_poses) {
  target_poses_ = target_poses;
}

bool MapGridCostFunction::prepare() {
  map_.resetPathDist();
  
  // ROS_INFO("MapGridCostFunction::prepare() - target_poses_.size() = %zu", target_poses_.size());
  
  // 在设置目标之前，验证和修复全局路径
  std::vector<geometry_msgs::PoseStamped> validated_poses;
  if (!validateGlobalPath(target_poses_, validated_poses)) {
    ROS_WARN("Global path validation failed, using original path");
    validated_poses = target_poses_;
  }
  // } else {
  //   // ROS_INFO("Global path validated and fixed: %zu -> %zu poses", 
  //            target_poses_.size(), validated_poses.size());
  // }
  
  if (is_local_goal_function_) {
    // ROS_INFO("Using setLocalGoal");
    map_.setLocalGoal(*costmap_, validated_poses);
  } else {
    // ROS_INFO("Using setTargetCells"); 
    map_.setTargetCells(*costmap_, validated_poses);
  }
  
  // 检查map_中有多少cell的target_dist不等于unreachableCellCosts
  int reachable_cells = 0;
  int total_cells = costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY();
  for (int i = 0; i < total_cells; i++) {
    if (map_(i % costmap_->getSizeInCellsX(), i / costmap_->getSizeInCellsX()).target_dist < map_.unreachableCellCosts()) {
      reachable_cells++;
    }
  }
      // ROS_WARN("After target setting: %d/%d cells are reachable (target_dist < unreachableCellCosts)", 
      //          reachable_cells, total_cells);
  
  // 如果可达性太低，尝试使用更宽松的路径设置
  if (reachable_cells < total_cells * 0.01) { // 少于1%的格子可达
    // ROS_WARN("Very low reachability (%d/%d), attempting relaxed path setting", 
    //          reachable_cells, total_cells);
    
    // 尝试使用原始路径但跳过不可达的点
    std::vector<geometry_msgs::PoseStamped> relaxed_poses;
    filterReachablePoses(target_poses_, relaxed_poses);
    
    if (!relaxed_poses.empty()) {
      map_.resetPathDist();
      if (is_local_goal_function_) {
        map_.setLocalGoal(*costmap_, relaxed_poses);
      } else {
        map_.setTargetCells(*costmap_, relaxed_poses);
      }
      
      // 重新检查可达性
      reachable_cells = 0;
      for (int i = 0; i < total_cells; i++) {
        if (map_(i % costmap_->getSizeInCellsX(), i / costmap_->getSizeInCellsX()).target_dist < map_.unreachableCellCosts()) {
          reachable_cells++;
        }
      }
      ROS_WARN("After relaxed setting: %d/%d cells are reachable", reachable_cells, total_cells);
    }
  }
  
  return true;
}

double MapGridCostFunction::getCellCosts(unsigned int px, unsigned int py) {
  double grid_dist = map_(px, py).target_dist;
  return grid_dist;
}

double MapGridCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0.0;
  if (aggregationType_ == Product) {
    cost = 1.0;
  }
  double px, py, pth;
  unsigned int cell_x, cell_y;
  double grid_dist;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);

    // translate point forward if specified
    if (xshift_ != 0.0) {
      px = px + xshift_ * cos(pth);
      py = py + xshift_ * sin(pth);
    }
    // translate point sideways if specified
    if (yshift_ != 0.0) {
      px = px + yshift_ * cos(pth + M_PI_2);
      py = py + yshift_ * sin(pth + M_PI_2);
    }

    //we won't allow trajectories that go off the map... shouldn't happen that often anyways
    if ( ! costmap_->worldToMap(px, py, cell_x, cell_y)) {
      //we're off the map
      ROS_WARN("Off Map %f, %f", px, py);
      return -4.0;
    }
    grid_dist = getCellCosts(cell_x, cell_y);
    //if a point on this trajectory has no clear path to the goal... it may be invalid
    if (stop_on_failure_) {
      if (grid_dist == map_.obstacleCosts()) {
        ROS_ERROR("Trajectory point (%u,%u) failed: grid_dist=%.1f equals obstacleCosts=%.1f", 
                  cell_x, cell_y, grid_dist, map_.obstacleCosts());
        return -3.0;
      } else if (grid_dist == map_.unreachableCellCosts()) {
        ROS_ERROR("Trajectory point (%u,%u) failed: grid_dist=%.1f equals unreachableCellCosts=%.1f", 
                  cell_x, cell_y, grid_dist, map_.unreachableCellCosts());
        ROS_ERROR("Map size: %d, %d, obstacleCosts: %.1f, unreachableCellCosts: %.1f", 
                  map_.size_x_, map_.size_y_, map_.obstacleCosts(), map_.unreachableCellCosts());
        return -2.0;
      }
    }

    switch( aggregationType_ ) {
    case Last:
      cost = grid_dist;
      break;
    case Sum:
      cost += grid_dist;
      break;
    case Product:
      if (cost > 0) {
        cost *= grid_dist;
      }
      break;
    }
  }
  return cost;
}

bool MapGridCostFunction::validateGlobalPath(const std::vector<geometry_msgs::PoseStamped>& original_path,
                                            std::vector<geometry_msgs::PoseStamped>& validated_path) {
  validated_path.clear();
  
  if (original_path.empty()) {
    return false;
  }
  
  int skipped_points = 0;
  int fixed_points = 0;
  
  for (size_t i = 0; i < original_path.size(); i++) {
    unsigned int mx, my;
    double wx = original_path[i].pose.position.x;
    double wy = original_path[i].pose.position.y;
    
    // 检查点是否在局部地图范围内
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      // 如果超出局部地图范围，跳过该点
      skipped_points++;
      continue;
    }
    
    // 检查该点是否可通行
    unsigned char cost = costmap_->getCost(mx, my);
    if (cost == costmap_2d::LETHAL_OBSTACLE || 
        cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        cost == costmap_2d::NO_INFORMATION) {
      
      // 尝试在附近找到可通行的点
      geometry_msgs::PoseStamped fixed_pose;
      if (findNearbyFreePose(original_path[i], fixed_pose)) {
        validated_path.push_back(fixed_pose);
        fixed_points++;
        ROS_DEBUG("Fixed pose at (%.2f, %.2f) -> (%.2f, %.2f)", 
                 wx, wy, fixed_pose.pose.position.x, fixed_pose.pose.position.y);
      } else {
        // 如果找不到可通行的点，跳过该点
        skipped_points++;
        ROS_DEBUG("Skipped unreachable pose at (%.2f, %.2f)", wx, wy);
      }
    } else {
      validated_path.push_back(original_path[i]);
    }
  }
  
  if (skipped_points > 0) {
    ROS_WARN("Path validation: skipped %d unreachable points, fixed %d points", 
             skipped_points, fixed_points);
  }
  
  return !validated_path.empty();
}

void MapGridCostFunction::filterReachablePoses(const std::vector<geometry_msgs::PoseStamped>& original_path,
                                              std::vector<geometry_msgs::PoseStamped>& filtered_path) {
  filtered_path.clear();
  
  for (const auto& pose : original_path) {
    unsigned int mx, my;
    double wx = pose.pose.position.x;
    double wy = pose.pose.position.y;
    
    // 检查点是否在局部地图范围内且可通行
    if (costmap_->worldToMap(wx, wy, mx, my)) {
      unsigned char cost = costmap_->getCost(mx, my);
      if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        filtered_path.push_back(pose);
      }
    }
  }
  
  // ROS_INFO("Filtered path: %zu -> %zu reachable poses", 
  //          original_path.size(), filtered_path.size());
}

bool MapGridCostFunction::findNearbyFreePose(const geometry_msgs::PoseStamped& original_pose,
                                            geometry_msgs::PoseStamped& free_pose,
                                            double search_radius) {
  double resolution = costmap_->getResolution();
  int search_cells = static_cast<int>(search_radius / resolution);
  
  unsigned int orig_mx, orig_my;
  if (!costmap_->worldToMap(original_pose.pose.position.x, 
                           original_pose.pose.position.y, 
                           orig_mx, orig_my)) {
    return false;
  }
  
  // 螺旋搜索模式 - 从最近的位置开始搜索
  for (int radius = 1; radius <= search_cells; radius++) {
    for (int dx = -radius; dx <= radius; dx++) {
      for (int dy = -radius; dy <= radius; dy++) {
        // 只搜索当前半径的边界，避免重复搜索
        if (abs(dx) != radius && abs(dy) != radius) continue;
        
        int check_mx = static_cast<int>(orig_mx) + dx;
        int check_my = static_cast<int>(orig_my) + dy;
        
        // 检查边界
        if (check_mx < 0 || check_my < 0 || 
            check_mx >= static_cast<int>(costmap_->getSizeInCellsX()) || 
            check_my >= static_cast<int>(costmap_->getSizeInCellsY())) {
          continue;
        }
        
        unsigned char cost = costmap_->getCost(check_mx, check_my);
        if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          // 找到可通行的点
          double wx, wy;
          costmap_->mapToWorld(check_mx, check_my, wx, wy);
          
          free_pose = original_pose;
          free_pose.pose.position.x = wx;
          free_pose.pose.position.y = wy;
          return true;
        }
      }
    }
  }
  return false;
}

} /* namespace base_local_planner */
