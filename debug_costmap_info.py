#!/usr/bin/env python

"""
Debug script to analyze costmap NO_INFORMATION issues causing -2.0 trajectory costs
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import matplotlib.pyplot as plt

class CostmapDebugger:
    def __init__(self):
        rospy.init_node('costmap_debugger', anonymous=True)
        
        # Subscribers
        self.global_costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', 
                                                  OccupancyGrid, 
                                                  self.global_costmap_callback)
        self.local_costmap_sub = rospy.Subscriber('/move_base/local_costmap/costmap', 
                                                 OccupancyGrid, 
                                                 self.local_costmap_callback)
        self.global_plan_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', 
                                               Path, 
                                               self.global_plan_callback)
        
        self.global_costmap = None
        self.local_costmap = None
        self.global_plan = None
        
        rospy.loginfo("Costmap debugger initialized. Waiting for data...")
        
    def global_costmap_callback(self, msg):
        self.global_costmap = msg
        rospy.loginfo(f"Received global costmap: {msg.info.width}x{msg.info.height}")
        
    def local_costmap_callback(self, msg):
        self.local_costmap = msg
        rospy.loginfo(f"Received local costmap: {msg.info.width}x{msg.info.height}")
        self.analyze_costmap()
        
    def global_plan_callback(self, msg):
        self.global_plan = msg
        rospy.loginfo(f"Received global plan with {len(msg.poses)} poses")
        
    def analyze_costmap(self):
        if self.local_costmap is None:
            return
            
        data = np.array(self.local_costmap.data)
        
        # Analyze cost values
        unknown_count = np.sum(data == -1)  # Unknown/NO_INFORMATION in occupancy grid
        free_count = np.sum(data == 0)      # Free space
        occupied_count = np.sum(data == 100) # Occupied
        
        total_cells = len(data)
        
        rospy.logwarn("=== COSTMAP ANALYSIS ===")
        rospy.logwarn(f"Total cells: {total_cells}")
        rospy.logwarn(f"Unknown/NO_INFORMATION cells: {unknown_count} ({100*unknown_count/total_cells:.1f}%)")
        rospy.logwarn(f"Free space cells: {free_count} ({100*free_count/total_cells:.1f}%)")
        rospy.logwarn(f"Occupied cells: {occupied_count} ({100*occupied_count/total_cells:.1f}%)")
        
        if unknown_count > total_cells * 0.5:
            rospy.logerr("WARNING: More than 50% of local costmap is unknown!")
            rospy.logerr("This is likely causing the -2.0 trajectory costs.")
            rospy.logerr("Possible solutions:")
            rospy.logerr("1. Check sensor data (laser, camera, etc.)")
            rospy.logerr("2. Verify sensor_msgs/LaserScan topics are publishing")
            rospy.logerr("3. Check costmap_2d configuration")
            rospy.logerr("4. Verify robot localization (AMCL/odom)")
        
        # Check if global plan intersects with local costmap
        if self.global_plan is not None and len(self.global_plan.poses) > 0:
            self.check_plan_validity()
            
    def check_plan_validity(self):
        if self.local_costmap is None or self.global_plan is None:
            return
            
        costmap = self.local_costmap
        plan = self.global_plan
        
        # Convert global plan points to local costmap coordinates
        valid_points = 0
        unknown_points = 0
        
        for pose_stamped in plan.poses:
            pose = pose_stamped.pose
            
            # Convert world coordinates to map coordinates
            mx = int((pose.position.x - costmap.info.origin.position.x) / costmap.info.resolution)
            my = int((pose.position.y - costmap.info.origin.position.y) / costmap.info.resolution)
            
            # Check if point is within costmap bounds
            if 0 <= mx < costmap.info.width and 0 <= my < costmap.info.height:
                idx = my * costmap.info.width + mx
                if idx < len(costmap.data):
                    cell_value = costmap.data[idx]
                    if cell_value == -1:  # Unknown
                        unknown_points += 1
                    elif cell_value >= 0:  # Known (free or occupied)
                        valid_points += 1
        
        total_plan_points = len(plan.poses)
        rospy.logwarn("=== GLOBAL PLAN ANALYSIS ===")
        rospy.logwarn(f"Total plan points: {total_plan_points}")
        rospy.logwarn(f"Valid points in local costmap: {valid_points}")
        rospy.logwarn(f"Unknown points in local costmap: {unknown_points}")
        
        if unknown_points > valid_points:
            rospy.logerr("ERROR: Most of global plan is in unknown territory!")
            rospy.logerr("This explains why trajectory costs are -2.0")
            rospy.logerr("Solutions:")
            rospy.logerr("1. Increase sensor range or add more sensors")
            rospy.logerr("2. Increase local_costmap size")
            rospy.logerr("3. Check if robot is properly localized")

if __name__ == '__main__':
    try:
        debugger = CostmapDebugger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
