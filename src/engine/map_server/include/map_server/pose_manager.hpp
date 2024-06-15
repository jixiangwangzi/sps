#ifndef POSE_MANAGER_HPP_
#define POSE_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_database.hpp"
#include <mutex>

class PoseManager
{
private:
    static PoseManager* m_instance;
    MapDatabase* mapDatabase;
    static pthread_mutex_t mutex;
    PoseManager();
    ~PoseManager();

public:
    static PoseManager* getInstance();

    bool changePixelPoseToWordPose(const sps_common_msgs::msg::PixelPose &p_pose, geometry_msgs::msg::Pose &w_pose);
    bool changePixelPointToWordPoint(const float p_x, const float p_y, geometry_msgs::msg::Point &w_point);
    bool changeWordPoseToPixelPose(const geometry_msgs::msg::Pose &w_pose, sps_common_msgs::msg::PixelPose &pose);
    bool queryPoseByName(const std::string &name, sps_common_msgs::msg::PixelPose &pose);
    PoiType queryPoiTypeByPose(const sps_common_msgs::msg::PixelPose &pixel_pose);
    bool checkPoseValid(const sps_common_msgs::msg::PixelPose &pixel_pose);
    bool queryChargePileByName(const std::string &name, sps_common_msgs::msg::PixelPose &pose);
    bool setDefaultChargePile(const std::string &map_id, const std::string &map_name, const std::string &charger_name, const sps_common_msgs::msg::PixelPose &pose);
    bool getDefaultChargePile(sps_common_msgs::msg::PixelPose &pose);
    bool getNearestChargePile(const sps_common_msgs::msg::PixelPose &cur_pose, sps_common_msgs::msg::PixelPose &charger_pose);
    bool getChargePile(geometry_msgs::msg::Pose &pose);
};

#endif