#include "map_server/lift_manager.hpp"

rclcpp::Logger LIFTM_LOG = rclcpp::get_logger("LiftManager");

LiftManager* LiftManager::m_instance = nullptr;
pthread_mutex_t LiftManager::mutex = PTHREAD_MUTEX_INITIALIZER;

LiftManager::LiftManager()
{
    mapDatabase = MapDatabase::getInstance();
}

LiftManager::~LiftManager()
{
}

LiftManager* LiftManager::getInstance()
{
    pthread_mutex_lock(&mutex);
    if(nullptr == m_instance) {
        m_instance = new LiftManager();
    }
    pthread_mutex_unlock(&mutex);
    return m_instance;
}

bool LiftManager::getLiftPoiById(const std::string lift_id, LiftPoi &lift_poi) {
    bool result = false;
    std::unordered_map<std::string, LiftPoi> lift_points;
    if (!mapDatabase->getLiftPoints(lift_points))
    {
        RCLCPP_ERROR(LIFTM_LOG, "getLiftPoiById lift_points is empty");
        return false;
    }

    auto lift_itor = lift_points.find(lift_id);
    if (lift_itor != lift_points.end()) {
        lift_poi = lift_itor->second;
        result = true;
    }
    RCLCPP_INFO(LIFTM_LOG, "getLiftPoiById result: %d, lift_id: %s", result, lift_id.c_str());
    return result;
}

bool LiftManager::queryLiftPoiType(const sps_common_msgs::msg::PixelPose &point, std::string& lift_id,
            LiftPoiType& lift_poi_type) {
    RCLCPP_INFO(LIFTM_LOG, "queryLiftPoiType x:%f, y:%f", point.x, point.y);
    bool valid = false;
    lift_id = "";
    lift_poi_type = UNKOWN_POINT;

    std::unordered_map<std::string, LiftPoi> lift_points;
    if (!mapDatabase->getLiftPoints(lift_points))
    {
        RCLCPP_ERROR(LIFTM_LOG, "queryLiftPoiType lift_points is empty");
        return false;
    }

    for (const auto& lift_tmp : lift_points) {
        const auto& inner_poi = lift_tmp.second.pixel_inner_poi;
        if (std::abs(inner_poi.x - point.x) < 1e-3 && std::abs(inner_poi.y - point.y) < 1e-3) {
            lift_id = lift_tmp.first;
            lift_poi_type = INNER_POINT;
            valid = true;
            return valid;
        }
    }

    for (const auto& lift_tmp : lift_points) {
        const auto& entry_poi = lift_tmp.second.pixel_entry_poi;
        for (const auto& entry_tmp : entry_poi) {
            if (std::abs(entry_tmp.x - point.x) < 1e-3 && std::abs(entry_tmp.y - point.y) < 1e-3
                    && std::abs(entry_tmp.theta - point.theta) < 1e-3) {
                lift_id = lift_tmp.first;
                lift_poi_type = ENTRY_POINT;
                valid = true;
                return valid;
            }
        }
    }

    for (const auto& lift_tmp : lift_points) {
        const auto& entry_poi = lift_tmp.second.pixel_entry_poi;
        const auto& out_poi = lift_tmp.second.pixel_out_poi;
        if (std::abs(out_poi.x - point.x) < 1e-4 && std::abs(out_poi.y - point.y) < 1e-4) {
            lift_id = lift_tmp.first;
            lift_poi_type = OUT_POINT;
            valid = true;
            return valid;
        }
        // 兼容老的电梯兴趣点
        else {
            for (const auto& entry_poi_tmp : entry_poi) {
                if (std::abs(entry_poi_tmp.x - point.x) < 1e-4 && std::abs(entry_poi_tmp.y - point.y) < 1e-4) {
                    lift_id = lift_tmp.first;
                    lift_poi_type = OUT_POINT;
                    valid = true;
                    return valid;
                }
            }
        }
    }

    return valid;
}

bool LiftManager::checkLiftInsideState(const geometry_msgs::msg::Point32 &point, const std::string& lift_id,
            const double liftin_distance_thresh) {
    RCLCPP_INFO(LIFTM_LOG, "CheckLiftInsideState x:%f, y:%f, lift_id:%s, liftin_distance_thresh:%f",
          point.x, point.y, lift_id.c_str(), liftin_distance_thresh);
    bool inside = false;
    rclcpp::Clock clock(RCL_ROS_TIME);

    std::unordered_map<std::string, LiftPoi> lift_points;
    if (!mapDatabase->getLiftPoints(lift_points))
    {
        RCLCPP_ERROR(LIFTM_LOG, "checkLiftInsideState lift_points is empty");
        return false;
    }

    // 以门左边的点为原点求解向量
    const auto lift_itor = lift_points.find(lift_id);
    if (lift_itor == lift_points.end()) {
        RCLCPP_ERROR_THROTTLE(LIFTM_LOG, clock, 5000, "CheckLiftInsideState cannot find lift: %s", lift_id.c_str());
        return inside;
    }
    double vec_door_x = lift_itor->second.door_poi[1].position.x - lift_itor->second.door_poi[0].position.x;
    double vec_door_y = lift_itor->second.door_poi[1].position.y - lift_itor->second.door_poi[0].position.y;
    double vec_robot_x = point.x - lift_itor->second.door_poi[0].position.x;
    double vec_robot_y = point.y - lift_itor->second.door_poi[0].position.y;
    // ROS_INFO("Lift door: (%0.3f, %0.3f), (%0.3f, %0.3f)", lift_itor->second.door_poi[0].position.x,
    //             lift_itor->second.door_poi[0].position.y, lift_itor->second.door_poi[1].position.x,
    //             lift_itor->second.door_poi[1].position.y);
    //向量叉乘
    double cross_value = vec_door_x * vec_robot_y - vec_door_y * vec_robot_x;

    if (cross_value > 0.0) {   // >0说明机器人在门里面
        double vec_door_norm = std::sqrt(vec_robot_x * vec_robot_x + vec_robot_y * vec_robot_y);
        double robot_dist = cross_value / vec_door_norm;
        RCLCPP_ERROR_THROTTLE(LIFTM_LOG, clock, 1000, "robot's distance to lift door: %0.3f, threshold: %0.3f", robot_dist, liftin_distance_thresh);
        if (robot_dist > liftin_distance_thresh) {
            inside = true;
        }
    }
    return inside;
}

// 获取机器人在电梯坐标系下的位姿
bool LiftManager::getRobotLiftPose2(const std::string& lift_id, const geometry_msgs::msg::PoseStamped& cur_robot_pose,
                     std::vector<double>& pose_lift_robot) {
    std::unordered_map<std::string, LiftPoi> lift_points;
    if (!mapDatabase->getLiftPoints(lift_points))
    {
        RCLCPP_ERROR(LIFTM_LOG, "getRobotLiftPose2 lift_points is empty");
        return false;
    }

    // 建立电梯坐标系：内点方向为x轴，垂直x轴的方向建立y轴（右手坐标系）
    const auto lift_itor = lift_points.find(lift_id);
    if (lift_itor == lift_points.end()) {
        RCLCPP_ERROR(LIFTM_LOG, "getRobotLiftPose2: cannot find lift: %s", lift_id.c_str());
        return false;
    }
    const auto& lift_inner = lift_itor->second.inner_poi;

    tf2::Quaternion q_map_lift;
    tf2::convert(lift_inner.orientation, q_map_lift);
    tf2::Quaternion q_lift_map = q_map_lift.inverse();
    tf2::Matrix3x3 m_lift_map(q_lift_map);

    double R_lift_map[2][2] = {{m_lift_map.getRow(0).getX(), m_lift_map.getRow(0).getY()},
                             {m_lift_map.getRow(1).getX(), m_lift_map.getRow(1).getY()}};
    double t_lift_map[2] = {-R_lift_map[0][0] * lift_inner.position.x - R_lift_map[0][1] * lift_inner.position.y,
                          -R_lift_map[1][0] * lift_inner.position.x - R_lift_map[1][1] * lift_inner.position.y};

    // 获取机器人坐标系到map坐标系的关系
    tf2::Quaternion quat;
    tf2::convert(cur_robot_pose.pose.orientation, quat);
    tf2::Matrix3x3 R_map_robot(quat);

    // 求解机器人坐标系到电梯坐标系的位姿关系
    double R_lift_robot[2][2];
    R_lift_robot[0][0] = R_lift_map[0][0] * R_map_robot[0][0] + R_lift_map[0][1] * R_map_robot[1][0];
    R_lift_robot[0][1] = R_lift_map[0][0] * R_map_robot[0][1] + R_lift_map[0][1] * R_map_robot[1][1];
    R_lift_robot[1][0] = R_lift_map[1][0] * R_map_robot[0][0] + R_lift_map[1][1] * R_map_robot[1][0];
    R_lift_robot[1][1] = R_lift_map[1][0] * R_map_robot[0][1] + R_lift_map[1][1] * R_map_robot[1][1];

    double t_lift_robot[2];
    t_lift_robot[0] = R_lift_map[0][0] * cur_robot_pose.pose.position.x
                    + R_lift_map[0][1] * cur_robot_pose.pose.position.y + t_lift_map[0];
    t_lift_robot[1] = R_lift_map[1][0] * cur_robot_pose.pose.position.x
                    + R_lift_map[1][1] * cur_robot_pose.pose.position.y + t_lift_map[1];
    pose_lift_robot.clear();
    pose_lift_robot.push_back(R_lift_robot[0][0]);
    pose_lift_robot.push_back(R_lift_robot[0][1]);
    pose_lift_robot.push_back(R_lift_robot[1][0]);
    pose_lift_robot.push_back(R_lift_robot[1][1]);
    pose_lift_robot.push_back(t_lift_robot[0]);
    pose_lift_robot.push_back(t_lift_robot[1]);

    //Debug
    tf2::Matrix3x3 R_lift_robot_tmp;
    R_lift_robot_tmp.setValue(R_lift_robot[0][0], R_lift_robot[0][1], 0.0,
                        R_lift_robot[1][0], R_lift_robot[1][1], 0.0,
                        0.0, 0.0, 1.0);
    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    R_lift_robot_tmp.getRPY(roll, pitch, yaw);

    RCLCPP_INFO(LIFTM_LOG, "getRobotLiftPose2 Robot in lift: %lf %lf %lf", t_lift_robot[0], t_lift_robot[1], yaw);

    return true;
}

// 获取机器人在地图坐标系下的位姿
bool LiftManager::getRobotMapPose2(const std::string& lift_id, const std::vector<double>& pose_lift_robot,
                        geometry_msgs::msg::PoseStamped& cur_robot_pose) {
    if (pose_lift_robot.size() < 6) {
        RCLCPP_ERROR(LIFTM_LOG, "GetRobotMapPose2 Failed get robot map pose: %ld", pose_lift_robot.size());
        return false;
    }

    std::unordered_map<std::string, LiftPoi> lift_points;
    if (!mapDatabase->getLiftPoints(lift_points))
    {
        RCLCPP_ERROR(LIFTM_LOG, "getRobotMapPose2 lift_points is empty");
        return false;
    }

    // 建立电梯坐标系：内点方向为x轴，垂直x轴的方向建立y轴（右手坐标系）
    const auto lift_itor = lift_points.find(lift_id);
    if (lift_itor == lift_points.end()) {
        RCLCPP_ERROR(LIFTM_LOG, "GetRobotMapPose2 cannot find lift: %s", lift_id.c_str());
        return false;
    }
    const auto& lift_inner = lift_itor->second.inner_poi;

    tf2::Quaternion q_lift_map;
    tf2::convert(lift_inner.orientation, q_lift_map);
    tf2::Matrix3x3 m_map_lift(q_lift_map);

    double R_map_lift[2][2] = {{m_map_lift.getRow(0).getX(), m_map_lift.getRow(0).getY()},
                             {m_map_lift.getRow(1).getX(), m_map_lift.getRow(1).getY()}};
    double t_map_lift[2] = {lift_inner.position.x, lift_inner.position.y};

    // 根据机器人到电梯坐标系的位姿关系(pose_lift_robot)， 求解机器人到Map的位姿关系
    double R_map_robot[2][2];
    R_map_robot[0][0] = R_map_lift[0][0] * pose_lift_robot[0] + R_map_lift[0][1] * pose_lift_robot[2];
    R_map_robot[0][1] = R_map_lift[0][0] * pose_lift_robot[1] + R_map_lift[0][1] * pose_lift_robot[3];
    R_map_robot[1][0] = R_map_lift[1][0] * pose_lift_robot[0] + R_map_lift[1][1] * pose_lift_robot[2];
    R_map_robot[1][1] = R_map_lift[1][0] * pose_lift_robot[1] + R_map_lift[1][1] * pose_lift_robot[3];

    double t_map_robot[2];
    t_map_robot[0] = R_map_lift[0][0] * pose_lift_robot[4] + R_map_lift[0][1] * pose_lift_robot[5] + t_map_lift[0];
    t_map_robot[1] = R_map_lift[1][0] * pose_lift_robot[4] + R_map_lift[1][1] * pose_lift_robot[5] + t_map_lift[1];

    tf2::Matrix3x3 R_map_robot_tmp;
    R_map_robot_tmp.setValue(R_map_robot[0][0], R_map_robot[0][1], 0.0,
                        R_map_robot[1][0], R_map_robot[1][1], 0.0,
                        0.0, 0.0, 1.0);

    // 转换成 PoseStamped 格式
    tf2::Quaternion quat;
    R_map_robot_tmp.getRotation(quat);
    cur_robot_pose.pose.position.x = t_map_robot[0];
    cur_robot_pose.pose.position.y = t_map_robot[1];
    cur_robot_pose.pose.position.z = 0.0;
    cur_robot_pose.pose.orientation.w = quat.w();
    cur_robot_pose.pose.orientation.x = quat.x();
    cur_robot_pose.pose.orientation.y = quat.y();
    cur_robot_pose.pose.orientation.z = quat.z();

    //Debug
    double yaw = 0.0, pitch = 0.0, roll = 0.0;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(LIFTM_LOG, "GetRobotMapPose2 Robot in new map: %lf %lf %lf", t_map_robot[0], t_map_robot[1], yaw);
    return true;
}

bool LiftManager::calcuLiftCheckPixelPoint(const std::string& lift_id, sps_common_msgs::msg::PixelPose& check_pixel_pt) {
    std::unordered_map<std::string, LiftPoi> lift_points;
    if (!mapDatabase->getLiftPoints(lift_points))
    {
        RCLCPP_ERROR(LIFTM_LOG, "calcuLiftCheckPixelPoint lift_points is empty");
        return false;
    }

    auto lift_poi_ptr = lift_points.find(lift_id);
    if (lift_poi_ptr == lift_points.end()) {
        RCLCPP_ERROR(LIFTM_LOG, "calcuLiftCheckPixelPoint cannot find lift by lift_id:%s", lift_id.c_str());
        return false;
    }
    check_pixel_pt = lift_poi_ptr->second.pixel_out_poi;
    check_pixel_pt.theta = ((int)check_pixel_pt.theta + 180) % 360;

    return true;
}

// 机器人出电梯后，先旋转至正对门口，再导航出题，这里得到出电梯方向角度
bool LiftManager::getLiftoutAngle(const std::string& lift_id, double& liftout_angle) {
    std::unordered_map<std::string, LiftPoi> lift_points;
    if (!mapDatabase->getLiftPoints(lift_points))
    {
        RCLCPP_ERROR(LIFTM_LOG, "getRobotMapPose2 lift_points is empty");
        return false;
    }

    auto lift_poi_ptr = lift_points.find(lift_id);
    if (lift_poi_ptr == lift_points.end()) {
        RCLCPP_ERROR(LIFTM_LOG, "getLiftoutAngle cannot find lift by lift_id:%s", lift_id.c_str());
        return false;
    }

    const auto& lift_out_poi = lift_poi_ptr->second.out_poi;
    const auto& lift_inner_poi = lift_poi_ptr->second.inner_poi;
    liftout_angle = std::atan2((lift_out_poi.position.y - lift_inner_poi.position.y),
                            (lift_out_poi.position.x - lift_inner_poi.position.x));
    // check_pt = lift_out_poi;
    // check_pt.orientation = tf::createQuaternionMsgFromYaw(theta);
    return true;
}