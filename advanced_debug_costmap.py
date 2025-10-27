#!/usr/bin/env python

"""
Advanced debug script to analyze the wavefront propagation and target distance calculation
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

class AdvancedCostmapDebugger:
    def __init__(self):
        rospy.init_node('advanced_costmap_debugger', anonymous=True)
        
        # TF listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Subscribers
        self.local_costmap_sub = rospy.Subscriber('/move_base/local_costmap/costmap', 
                                                 OccupancyGrid, 
                                                 self.local_costmap_callback)
        self.global_plan_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', 
                                               Path, 
                                               self.global_plan_callback)
        # We'll get robot pose from tf transform instead of amcl_pose
        # since the user is using /base_footprint
        
        self.local_costmap = None
        self.global_plan = None
        
        rospy.loginfo("Advanced costmap debugger initialized. Waiting for data...")
        
    def get_robot_pose_from_tf(self):
        """Get robot pose from tf transform between map and base_footprint"""
        try:
            # Get transform from map to base_footprint
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
            
            # Create PoseStamped from transform
            pose_stamped = PoseStamped()
            pose_stamped.header = transform.header
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation
            
            return pose_stamped
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Could not get robot pose from tf: {e}")
            return None
        
    def local_costmap_callback(self, msg):
        self.local_costmap = msg
        rospy.loginfo(f"Received local costmap: {msg.info.width}x{msg.info.height}")
        self.analyze_detailed()
        
    def global_plan_callback(self, msg):
        self.global_plan = msg
        rospy.loginfo(f"Received global plan with {len(msg.poses)} poses")
        
    def get_robot_pose(self):
        """Get current robot pose from tf"""
        return self.get_robot_pose_from_tf()
        
    def analyze_detailed(self):
        if self.local_costmap is None or self.global_plan is None:
            return
            
        costmap = self.local_costmap
        plan = self.global_plan
        
        rospy.logwarn("=== DETAILED ANALYSIS ===")
        
        # 1. Check costmap characteristics
        data = np.array(costmap.data).reshape(costmap.info.height, costmap.info.width)
        
        # Find actual cost values (not just occupancy grid values)
        unique_values = np.unique(data)
        rospy.logwarn(f"Unique cost values in costmap: {unique_values}")
        
        # 2. Analyze global plan points in detail
        self.analyze_plan_in_costmap(costmap, plan)
        
        # 3. Check robot position
        robot_pose = self.get_robot_pose()
        if robot_pose:
            self.analyze_robot_position(costmap, robot_pose)
            
        # 4. Simulate setTargetCells behavior
        self.simulate_target_cells_setting(costmap, plan)
        
    def analyze_plan_in_costmap(self, costmap, plan):
        rospy.logwarn("=== GLOBAL PLAN IN COSTMAP ANALYSIS ===")
        
        plan_points_in_costmap = []
        first_valid_point = None
        last_valid_point = None
        
        for i, pose_stamped in enumerate(plan.poses):
            pose = pose_stamped.pose
            
            # Convert world coordinates to map coordinates
            mx = int((pose.position.x - costmap.info.origin.position.x) / costmap.info.resolution)
            my = int((pose.position.y - costmap.info.origin.position.y) / costmap.info.resolution)
            
            # Check if point is within costmap bounds
            if 0 <= mx < costmap.info.width and 0 <= my < costmap.info.height:
                idx = my * costmap.info.width + mx
                if idx < len(costmap.data):
                    cell_value = costmap.data[idx]
                    plan_points_in_costmap.append({
                        'index': i,
                        'world_x': pose.position.x,
                        'world_y': pose.position.y,
                        'map_x': mx,
                        'map_y': my,
                        'cost': cell_value
                    })
                    
                    if first_valid_point is None:
                        first_valid_point = plan_points_in_costmap[-1]
                    last_valid_point = plan_points_in_costmap[-1]
        
        rospy.logwarn(f"Plan points in local costmap: {len(plan_points_in_costmap)}")
        
        if first_valid_point:
            rospy.logwarn(f"First valid point: index {first_valid_point['index']}, "
                         f"world({first_valid_point['world_x']:.2f}, {first_valid_point['world_y']:.2f}), "
                         f"map({first_valid_point['map_x']}, {first_valid_point['map_y']}), "
                         f"cost={first_valid_point['cost']}")
                         
        if last_valid_point:
            rospy.logwarn(f"Last valid point: index {last_valid_point['index']}, "
                         f"world({last_valid_point['world_x']:.2f}, {last_valid_point['world_y']:.2f}), "
                         f"map({last_valid_point['map_x']}, {last_valid_point['map_y']}), "
                         f"cost={last_valid_point['cost']}")
        
        # Check for high-cost cells that might block wavefront propagation
        high_cost_points = [p for p in plan_points_in_costmap if p['cost'] >= 99]  # Near lethal
        if high_cost_points:
            rospy.logerr(f"WARNING: {len(high_cost_points)} plan points are in high-cost areas (cost >= 99)")
            for p in high_cost_points[:5]:  # Show first 5
                rospy.logerr(f"  High-cost point: index {p['index']}, map({p['map_x']}, {p['map_y']}), cost={p['cost']}")
                
        return plan_points_in_costmap
        
    def analyze_robot_position(self, costmap, robot_pose):
        rospy.logwarn("=== ROBOT POSITION ANALYSIS ===")
        
        pose = robot_pose.pose
        
        # Convert robot position to map coordinates
        mx = int((pose.position.x - costmap.info.origin.position.x) / costmap.info.resolution)
        my = int((pose.position.y - costmap.info.origin.position.y) / costmap.info.resolution)
        
        rospy.logwarn(f"Robot world position: ({pose.position.x:.2f}, {pose.position.y:.2f})")
        rospy.logwarn(f"Robot map position: ({mx}, {my})")
        
        # Check if robot is within costmap
        if 0 <= mx < costmap.info.width and 0 <= my < costmap.info.height:
            idx = my * costmap.info.width + mx
            if idx < len(costmap.data):
                robot_cost = costmap.data[idx]
                rospy.logwarn(f"Robot cell cost: {robot_cost}")
                
                if robot_cost >= 99:
                    rospy.logerr("ERROR: Robot is in high-cost area! This could prevent path planning.")
            else:
                rospy.logerr("ERROR: Robot position index out of bounds!")
        else:
            rospy.logerr("ERROR: Robot is outside local costmap bounds!")
            
    def simulate_target_cells_setting(self, costmap, plan):
        rospy.logwarn("=== SIMULATING setTargetCells BEHAVIOR ===")
        
        # This simulates the setTargetCells function from map_grid.cpp
        valid_target_cells = 0
        blocked_by_obstacles = 0
        blocked_by_unknown = 0
        
        for pose_stamped in plan.poses:
            pose = pose_stamped.pose
            
            # Convert world coordinates to map coordinates  
            mx = int((pose.position.x - costmap.info.origin.position.x) / costmap.info.resolution)
            my = int((pose.position.y - costmap.info.origin.position.y) / costmap.info.resolution)
            
            # Check if point is within costmap bounds
            if 0 <= mx < costmap.info.width and 0 <= my < costmap.info.height:
                idx = my * costmap.info.width + mx
                if idx < len(costmap.data):
                    cell_cost = costmap.data[idx]
                    
                    # Simulate the conditions in updatePathCell function
                    if cell_cost == 255:  # NO_INFORMATION (unlikely since we found 0%)
                        blocked_by_unknown += 1
                    elif cell_cost == 254:  # LETHAL_OBSTACLE  
                        blocked_by_obstacles += 1
                    elif cell_cost == 253:  # INSCRIBED_INFLATED_OBSTACLE
                        blocked_by_obstacles += 1
                    elif cell_cost >= 99:   # Very high cost (might be treated as obstacle)
                        blocked_by_obstacles += 1
                        rospy.logwarn(f"High cost target cell at ({mx}, {my}): cost={cell_cost}")
                    else:
                        valid_target_cells += 1
                        if valid_target_cells <= 5:  # Log first few valid cells
                            rospy.logwarn(f"Valid target cell at ({mx}, {my}): cost={cell_cost}")
        
        rospy.logwarn(f"Valid target cells for wavefront: {valid_target_cells}")
        rospy.logwarn(f"Blocked by obstacles: {blocked_by_obstacles}")  
        rospy.logwarn(f"Blocked by unknown: {blocked_by_unknown}")
        
        if valid_target_cells == 0:
            rospy.logerr("CRITICAL: No valid target cells for wavefront propagation!")
            rospy.logerr("This explains why all trajectory costs are -2.0 (unreachable)")
            rospy.logerr("Solutions:")
            rospy.logerr("1. Increase local_costmap size to include more of the global plan")
            rospy.logerr("2. Reduce inflation_radius in costmap configuration")
            rospy.logerr("3. Check if global planner is creating valid paths")
        elif valid_target_cells < 5:
            rospy.logerr(f"WARNING: Very few valid target cells ({valid_target_cells})")
            rospy.logerr("Wavefront propagation might be severely limited")

if __name__ == '__main__':
    try:
        debugger = AdvancedCostmapDebugger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
