/*
 * 全局路径穿过不可通行区域的解决方案
 * 
 * 问题分析：
 * 1. 全局路径规划器基于全局代价地图规划路径
 * 2. 局部路径规划器基于局部代价地图执行路径跟踪
 * 3. 两个代价地图可能不同步，导致全局路径在局部看来不可通行
 */

// 解决方案1：路径验证和修复
bool validateAndFixGlobalPath(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                             const costmap_2d::Costmap2D& local_costmap,
                             std::vector<geometry_msgs::PoseStamped>& fixed_plan) {
    fixed_plan.clear();
    
    for (size_t i = 0; i < global_plan.size(); i++) {
        unsigned int mx, my;
        double wx = global_plan[i].pose.position.x;
        double wy = global_plan[i].pose.position.y;
        
        // 检查点是否在局部地图范围内
        if (!local_costmap.worldToMap(wx, wy, mx, my)) {
            // 如果超出局部地图范围，跳过该点
            continue;
        }
        
        // 检查该点是否可通行
        unsigned char cost = local_costmap.getCost(mx, my);
        if (cost == costmap_2d::LETHAL_OBSTACLE || 
            cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
            cost == costmap_2d::NO_INFORMATION) {
            
            // 尝试在附近找到可通行的点
            geometry_msgs::PoseStamped fixed_pose;
            if (findNearbyFreePose(global_plan[i], local_costmap, fixed_pose)) {
                fixed_plan.push_back(fixed_pose);
            }
            // 如果找不到可通行的点，跳过该点
        } else {
            fixed_plan.push_back(global_plan[i]);
        }
    }
    
    return !fixed_plan.empty();
}

// 解决方案2：在附近寻找可通行的位置
bool findNearbyFreePose(const geometry_msgs::PoseStamped& original_pose,
                       const costmap_2d::Costmap2D& costmap,
                       geometry_msgs::PoseStamped& free_pose,
                       double search_radius = 1.0) {
    
    double resolution = costmap.getResolution();
    int search_cells = static_cast<int>(search_radius / resolution);
    
    unsigned int orig_mx, orig_my;
    if (!costmap.worldToMap(original_pose.pose.position.x, 
                           original_pose.pose.position.y, 
                           orig_mx, orig_my)) {
        return false;
    }
    
    // 螺旋搜索模式
    for (int radius = 1; radius <= search_cells; radius++) {
        for (int dx = -radius; dx <= radius; dx++) {
            for (int dy = -radius; dy <= radius; dy++) {
                if (abs(dx) != radius && abs(dy) != radius) continue; // 只搜索边界
                
                unsigned int check_mx = orig_mx + dx;
                unsigned int check_my = orig_my + dy;
                
                if (check_mx >= costmap.getSizeInCellsX() || 
                    check_my >= costmap.getSizeInCellsY()) continue;
                
                unsigned char cost = costmap.getCost(check_mx, check_my);
                if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                    // 找到可通行的点
                    double wx, wy;
                    costmap.mapToWorld(check_mx, check_my, wx, wy);
                    
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

// 解决方案3：路径平滑和插值
std::vector<geometry_msgs::PoseStamped> smoothPath(
    const std::vector<geometry_msgs::PoseStamped>& original_path,
    const costmap_2d::Costmap2D& costmap,
    double max_distance = 0.1) {
    
    std::vector<geometry_msgs::PoseStamped> smoothed_path;
    
    if (original_path.empty()) return smoothed_path;
    
    smoothed_path.push_back(original_path[0]);
    
    for (size_t i = 1; i < original_path.size(); i++) {
        geometry_msgs::PoseStamped current = original_path[i];
        geometry_msgs::PoseStamped previous = smoothed_path.back();
        
        // 计算两点间距离
        double dx = current.pose.position.x - previous.pose.position.x;
        double dy = current.pose.position.y - previous.pose.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        // 如果距离太大，插入中间点
        if (distance > max_distance) {
            int num_points = static_cast<int>(ceil(distance / max_distance));
            
            for (int j = 1; j < num_points; j++) {
                double ratio = static_cast<double>(j) / num_points;
                geometry_msgs::PoseStamped interpolated = previous;
                
                interpolated.pose.position.x = previous.pose.position.x + ratio * dx;
                interpolated.pose.position.y = previous.pose.position.y + ratio * dy;
                
                // 检查插值点是否可通行
                unsigned int mx, my;
                if (costmap.worldToMap(interpolated.pose.position.x, 
                                     interpolated.pose.position.y, mx, my)) {
                    unsigned char cost = costmap.getCost(mx, my);
                    if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                        smoothed_path.push_back(interpolated);
                    }
                }
            }
        }
        
        smoothed_path.push_back(current);
    }
    
    return smoothed_path;
}
