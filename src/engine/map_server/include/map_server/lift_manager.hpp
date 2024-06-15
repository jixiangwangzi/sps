#ifndef LIFT_MANAGER_HPP_
#define LIFT_MANAGER_HPP_

#include <mutex>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "map_database.hpp"
#include "map_structure.h"

#include "sps_common_msgs/msg/pixel_pose.hpp"

class LiftManager
{
private:
    static LiftManager* m_instance;
    static pthread_mutex_t mutex;
    MapDatabase* mapDatabase;

    LiftManager();
    ~LiftManager();

public:
    static LiftManager* getInstance();

    bool getLiftPoiById(const std::string lift_id, LiftPoi &lift_poi);
    bool queryLiftPoiType(const sps_common_msgs::msg::PixelPose &point, std::string& lift_id, LiftPoiType& lift_poi_type);
    bool checkLiftInsideState(const geometry_msgs::msg::Point32 &point, const std::string& lift_id, const double liftin_distance_thresh);
    bool getRobotLiftPose2(const std::string& lift_id, const geometry_msgs::msg::PoseStamped& cur_robot_pose, std::vector<double>& pose_lift_robot);
    bool getRobotMapPose2(const std::string& lift_id, const std::vector<double>& pose_lift_robot, geometry_msgs::msg::PoseStamped& cur_robot_pose);
    bool calcuLiftCheckPixelPoint(const std::string& lift_id, sps_common_msgs::msg::PixelPose& check_pixel_pt);
    bool getLiftoutAngle(const std::string& lift_id, double& liftout_angle);
};

#endif