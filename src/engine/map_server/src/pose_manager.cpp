#include "map_server/pose_manager.hpp"

rclcpp::Logger POSE_LOG = rclcpp::get_logger("PoseManager");

PoseManager* PoseManager::m_instance = nullptr;
pthread_mutex_t PoseManager::mutex = PTHREAD_MUTEX_INITIALIZER;

PoseManager::PoseManager()
{
    mapDatabase = MapDatabase::getInstance();
}

PoseManager::~PoseManager()
{
}

PoseManager* PoseManager::getInstance()
{
    pthread_mutex_lock(&mutex);
    if(nullptr == m_instance) {
        m_instance = new PoseManager();
    }
    pthread_mutex_unlock(&mutex);
    return m_instance;
}


bool PoseManager::changePixelPoseToWordPose(const sps_common_msgs::msg::PixelPose &p_pose, geometry_msgs::msg::Pose &w_pose) {
    nav_msgs::msg::OccupancyGrid occMap;
    if (!mapDatabase->getOccMap(occMap))
    {
        RCLCPP_ERROR(POSE_LOG, "changePixelPoseToWordPose occ_map is empty");
        return false;
    }
    w_pose.position.x = p_pose.x * occMap.info.resolution + occMap.info.origin.position.x;
    w_pose.position.y =
            (occMap.info.height - p_pose.y - 1) * occMap.info.resolution +
            occMap.info.origin.position.y;

    double yaw = (450 - p_pose.theta) * M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    w_pose.orientation = tf2::toMsg(q);

    return true;
}

bool PoseManager::changePixelPointToWordPoint(const float p_x, const float p_y, geometry_msgs::msg::Point &w_point) {
    nav_msgs::msg::OccupancyGrid occMap;
    if (!mapDatabase->getOccMap(occMap))
    {
        RCLCPP_ERROR(POSE_LOG, "changePixelPointToWordPoint occ_map is empty");
        return false;
    }
    w_point.x = p_x * occMap.info.resolution + occMap.info.origin.position.x;
    w_point.y = (occMap.info.height - p_y - 1) * occMap.info.resolution +
            occMap.info.origin.position.y;

    return true;
}

bool PoseManager::changeWordPoseToPixelPose(const geometry_msgs::msg::Pose &w_pose, sps_common_msgs::msg::PixelPose &pose) {
    nav_msgs::msg::OccupancyGrid occMap;
    if (!mapDatabase->getOccMap(occMap))
    {
        RCLCPP_ERROR(POSE_LOG, "changeWordPoseToPixelPose occ_map is empty");
        return false;
    }

    double x = (w_pose.position.x - occMap.info.origin.position.x)/occMap.info.resolution;
    double y = occMap.info.height - (w_pose.position.y - occMap.info.origin.position.y)/occMap.info.resolution - 1;
    double yaw = tf2::getYaw<geometry_msgs::msg::Quaternion>(w_pose.orientation);

    int theta = (static_cast<int>(yaw * 180 / M_PI) + 360) % 360;
    pose.x = static_cast<float>(x);
    pose.y = static_cast<float>(y);
    pose.theta = static_cast<float>((450 - theta) % 360);

    return true;
}

bool PoseManager::queryPoseByName(const std::string &name, sps_common_msgs::msg::PixelPose &pose) {
    RCLCPP_INFO(POSE_LOG, "queryPoseByName name: %s", name.c_str());
    std::unordered_map<std::string, CommonPoi> all_points;
    if (!mapDatabase->getAllPoints(all_points))
    {
        RCLCPP_ERROR(POSE_LOG, "queryPoseByName points is empty");
        return false;
    }

    auto find_res = all_points.find(name);
    if (find_res != all_points.end()) {
        pose = find_res->second.pixel_pose;
        return true;
    } else {
        return false;
    }
}

PoiType PoseManager::queryPoiTypeByPose(const sps_common_msgs::msg::PixelPose &pixel_pose) {
    RCLCPP_INFO(POSE_LOG, "queryPoiTypeByPose x:%f, y:%f, theta:%f", pixel_pose.x, pixel_pose.y, pixel_pose.theta);
    PoiType poi_type = UNKOWN_TYPE;
    std::unordered_map<std::string, CommonPoi> all_points;
    if (!mapDatabase->getAllPoints(all_points))
    {
        RCLCPP_ERROR(POSE_LOG, "queryPoiTypeByPose points is empty");
    } else {
        for (const auto& point : all_points) {
            if (point.second.pixel_pose.x == pixel_pose.x && point.second.pixel_pose.y == pixel_pose.y
                    && point.second.pixel_pose.theta == pixel_pose.theta) {
                poi_type = point.second.poi_type;
            }
        }
    }
    RCLCPP_INFO(POSE_LOG, "queryPoiTypeByPose poi_type:%d", poi_type);
    return poi_type;
}

bool PoseManager::checkPoseValid(const sps_common_msgs::msg::PixelPose &pixel_pose) {
    RCLCPP_INFO(POSE_LOG, "checkPoseValid x:%f, y:%f", pixel_pose.x, pixel_pose.y);
    nav_msgs::msg::OccupancyGrid occMap;
    if (!mapDatabase->getOccMap(occMap))
    {
        RCLCPP_ERROR(POSE_LOG, "checkPoseValid occ_map is empty");
        return false;
    }

    if (pixel_pose.x <=0 || pixel_pose.x >=occMap.info.width
            || pixel_pose.y <=0 || pixel_pose.y >=occMap.info.height){
        RCLCPP_ERROR(POSE_LOG, "checkPoseValid pixel_pose point is beyond the map");
        return false;
    }

    uint32_t occ_index = (int) pixel_pose.x + (occMap.info.height - (int) pixel_pose.y - 1) * occMap.info.width;
    if (occMap.data[occ_index] != 0){
        RCLCPP_ERROR(POSE_LOG, "checkPoseValid pixel_pose point is in unknown area");
        return false;
    }

    return true;
}

bool PoseManager::queryChargePileByName(const std::string &name, sps_common_msgs::msg::PixelPose &pose) {
    RCLCPP_INFO(POSE_LOG, "queryChargePileByName name: %s", name.c_str());
    std::unordered_map<std::string, ChargerPoi> charger_points;
    if (!mapDatabase->getChargerPoints(charger_points))
    {
        RCLCPP_ERROR(POSE_LOG, "queryChargePileByName points is empty");
        return false;
    }

    auto find_res = charger_points.find(name);
    if (find_res != charger_points.end()) {
        pose = find_res->second.pixel_pose;
        return true;
    } else {
        return false;
    }
}

bool PoseManager::setDefaultChargePile(const std::string &map_id, const std::string &map_name,
            const std::string &charger_name, const sps_common_msgs::msg::PixelPose &pose) {
    std::string map_uuid = mapDatabase->getMapInfo("id");
    if (map_uuid != map_id) {
        RCLCPP_ERROR(POSE_LOG, "setDefaultChargePile Using map_uuid: %s != request map_id: %s", map_uuid.c_str(), map_id.c_str());
        return false;
    } else {
        return mapDatabase->setDefaultChargePile(charger_name, pose);
    }
}

bool PoseManager::getDefaultChargePile(sps_common_msgs::msg::PixelPose &pose) {
    bool result = false;
    std::unordered_map<std::string, ChargerPoi> charger_points;
    if (!mapDatabase->getChargerPoints(charger_points))
    {
        RCLCPP_ERROR(POSE_LOG, "getDefaultChargePile points is empty");
        return false;
    }

    RCLCPP_INFO(POSE_LOG, "getDefaultChargePile charger_points.size:%ld", charger_points.size());
    if(charger_points.size() == 1) {
        auto iter = charger_points.begin();
        pose = iter->second.pixel_pose;
        result = true;
    } else {
        for (const auto& charger_poi: charger_points) {
            if (charger_poi.second.is_default) {
                pose = charger_poi.second.pixel_pose;
                result = true;
                break;
            }
        }
    }
    RCLCPP_INFO(POSE_LOG, "getDefaultChargePile result:%d", result);
    return result;
}

bool PoseManager::getNearestChargePile(const sps_common_msgs::msg::PixelPose &cur_pose,
            sps_common_msgs::msg::PixelPose &charger_pose) {
    std::unordered_map<std::string, ChargerPoi> charger_points;
    if (!mapDatabase->getChargerPoints(charger_points))
    {
        RCLCPP_ERROR(POSE_LOG, "getNearestChargePile points is empty");
        return false;
    }

    if (charger_points.size() == 1) {
        auto iter = charger_points.begin();
        charger_pose = iter->second.pixel_pose;
    } else {
        double cur_nearest = std::numeric_limits<double>::max();
        for (const auto &pile : charger_points) {
            double diff_x = pile.second.pixel_pose.x - cur_pose.x;
            double diff_y = pile.second.pixel_pose.y - cur_pose.y;
            double diff = std::sqrt(diff_x * diff_x + diff_y * diff_y);
            if (diff < cur_nearest) {
                cur_nearest = diff;
                charger_pose = pile.second.pixel_pose;
            }
        }
    }
    RCLCPP_INFO(POSE_LOG, "getNearestChargePile charger_pose x:%f, y:%f", charger_pose.x, charger_pose.y);
    return true;
}

//TODO: 多充电桩
bool PoseManager::getChargePile(geometry_msgs::msg::Pose &pose) {
    std::unordered_map<std::string, ChargerPoi> charger_points;
    if (!mapDatabase->getChargerPoints(charger_points))
    {
        RCLCPP_ERROR(POSE_LOG, "getChargePile points is empty");
        return false;
    }

    if (charger_points.size() > 1) {
        RCLCPP_ERROR(POSE_LOG, "getChargePile There are more than one charging point");
        return false;
    }
    changePixelPoseToWordPose(charger_points.cbegin()->second.pixel_pose, pose);
    return true;
}
