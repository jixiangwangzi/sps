#include "map_server/map_database.hpp"

#include <filesystem>
#include "yaml-cpp/yaml.h"
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR == 4
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#endif

rclcpp::Logger MAPDB_LOG = rclcpp::get_logger("MapDatabase");

MapDatabase* MapDatabase::m_instance = nullptr;
pthread_mutex_t MapDatabase::mutex = PTHREAD_MUTEX_INITIALIZER;

MapDatabase::MapDatabase() {
    map_floor_ = 0;
    bit_map_ = std::make_shared<cm_msgs::msg::BitMap>();
    occ_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    tag_poses_ = std::make_shared<std::vector<TagPoi>>();
    //planner_ = std::make_shared<GingerPlanner>();
    process_lift_poi_ = LiftPoiProcess::getInstance();
    //rdm_pub_ = std::make_shared<ros::Publisher>(nh_->advertise<ginger_msgs::RdmAlarm>("rdm_alarm", 1, true));

    map_uuid_ = "fake uuid";
    map_name_ = "fake name";
    map_loaded_ = false;
}

MapDatabase::~MapDatabase() {
    if (bit_map_) {
        bit_map_.reset();
        bit_map_ = nullptr;
    }
    if (occ_map_) {
        occ_map_.reset();
        occ_map_ = nullptr;
    }
    if (static_map_) {
        static_map_.reset();
        static_map_ = nullptr;
    }
    if (tag_poses_) {
        tag_poses_.reset();
        tag_poses_ = nullptr;
    }
}

MapDatabase* MapDatabase::getInstance() {
    pthread_mutex_lock(&mutex);
    if(nullptr == m_instance) {
        m_instance = new MapDatabase();
    }
    pthread_mutex_unlock(&mutex);
    return m_instance;
}

void MapDatabase::init(const std::string& home_path) {
    home_path_ = home_path;
    RCLCPP_INFO(MAPDB_LOG, "MapDatabase::init home_path: %s", home_path.c_str());
}

bool MapDatabase::getMapLoadStatus() {
    return map_loaded_;
}

int MapDatabase::loadTestMap(const std::string& map_path) {
    clearAllMapData();
    const std::string info_file(map_path + "/info.json");
    const std::string yaml_file{map_path + "/map.yaml"};
    const std::string image_file{map_path + "/map.png"};

    // load grid map
    int grid_res = loadGridMap(info_file, yaml_file, image_file);
    map_loaded_ = true;
    return grid_res;
}

int MapDatabase::loadMap(const std::string& map_path, std::string uuid) {
    clearAllMapData();
    const std::string info_file(map_path + "/info.json");
    const std::string yaml_file{map_path + "/grid_map_layer/map.yaml"};
    const std::string image_file{map_path + "/grid_map_layer/map.png"};
    const std::string tags_file{map_path + "/grid_map_layer/tag.json"};
    const std::string poi_file{map_path + "/semantics_poi_layer/poi.json"};
    const std::string wall_file{map_path + "/semantics_forbiddenline_layer/forbiddenline.json"};
    const std::string area_file{map_path + "/semantics_area_layer/area.json"};

    // load grid map
    int grid_res = loadGridMap(info_file, yaml_file, image_file);
    if (grid_res < 0) {
        return grid_res;
    }

    updateLoadedMapId(map_path, uuid);

    // load poi
    loadSemanticsPoi(poi_file);
  
    // load virtual wall
    loadVirtualWall(wall_file);

    // load semantics area
    loadSemanticsArea(area_file);

    //用于加载地图之后，设置初始位置
    //readRobotPoseFromFile();
    map_loaded_ = true;
    load_time_ = rclcpp::Time();
    return 1;
}

int MapDatabase::loadGridMap(const std::string& info_file, const std::string& yaml_file, const std::string& image_file) {
    // for check file
    if (!std::filesystem::exists(info_file)) {
        RCLCPP_WARN(MAPDB_LOG, "LoadGridMap info_file is not exist: %s", info_file.c_str());
    }
    if (!std::filesystem::exists(yaml_file)) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadGridMap yaml_file is not exist: %s", yaml_file.c_str());
    }
    if (!std::filesystem::exists(image_file)) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadGridMap image_file is not exist: %s", image_file.c_str());
    }

    YAML::Node map_yaml;
    try {
        map_yaml = YAML::LoadFile(yaml_file);
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadGridMap cant open map yaml file: %s", e.msg.c_str());
        //rdmPub(); TODO:
        return -4;  // NO_MAP_YAML
    }

    try {
        auto image = map_yaml["image"].as<std::string>();
        origin_.clear();
        for (int i = 0; i < 3; i++) {
            origin_.push_back(map_yaml["origin"][i].as<double>());
        }
        resolution_ = map_yaml["resolution"].as<double>();
        threshold_free_ = map_yaml["free_thresh"].as<double>();
        threshold_occupied_ = map_yaml["occupied_thresh"].as<double>();
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadGridMap error occured when parsing map yaml file: %s", e.msg.c_str());
        //rdmPub(); TODO:
        return -6;  // MAP_YAML_INVALID
    }

    // cv::Mat grid_img_;
    try {
        grid_img_ = cv::imread(image_file, CV_LOAD_IMAGE_GRAYSCALE);
        if (grid_img_.empty()) {
            RCLCPP_ERROR(MAPDB_LOG, "LoadGridMap map png file is empty");
            //rdmPub(); TODO:
            return -5;  // MAP_PNG_UNVALID
        }
    } catch (cv::Exception &e) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadGridMap cant open map png file");
        //rdmPub(); TODO:
        return -3;  // NO_MAP_PNG
    }

    // get occ maps
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, origin_[2]);
    q.normalize();
    if (!occ_map_->data.empty()) {
        occ_map_->data.clear();
    }

    if (!bit_map_->data.empty()) {
        bit_map_->data.clear();
    }
    bit_map_->height = grid_img_.rows;
    bit_map_->width = grid_img_.cols;
    bit_map_->data.resize(grid_img_.rows * grid_img_.cols);
    bit_map_->stamp = rclcpp::Time();
    cv::imencode(".png", grid_img_, bit_map_->data);
    occ_map_->info.resolution = resolution_;
    occ_map_->info.origin.position.x = origin_[0];
    occ_map_->info.origin.position.y = origin_[1];
    occ_map_->info.origin.orientation = tf2::toMsg(q);
    occ_map_->info.width = grid_img_.cols;
    occ_map_->info.height = grid_img_.rows;
    occ_map_->data.resize(grid_img_.rows * grid_img_.cols);
    occ_map_->header.frame_id = map_frame_;
    occ_map_->header.stamp = rclcpp::Time();
    for (uint32_t i = 0; i < grid_img_.rows; i++) {
        for (uint32_t j = 0; j < grid_img_.cols; j++) {
            uint32_t occ_index = j + (grid_img_.rows - i - 1) * grid_img_.cols;
            uint8_t pixel = grid_img_.at<uint8_t>(i, j);
            if (pixel < 70) {
                occ_map_->data[occ_index] = 100;
            } else if (pixel > 220) {
                occ_map_->data[occ_index] = 0;
            } else {
                occ_map_->data[occ_index] = -1;
            }
        }
    }

    // load map info
    try {
        auto mapinfo = YAML::LoadFile(info_file);
        map_floor_ = mapinfo["map_floor"].as<int>();
        map_name_ = mapinfo["map_name"].as<std::string>();
        map_uuid_ = mapinfo["map_id"].as<std::string>();
        RCLCPP_INFO(MAPDB_LOG, "LoadGridMap map_uuid_: %s, map_name_: %s, map_floor_: %d",
                map_uuid_.c_str(), map_name_.c_str(), map_floor_);
    } catch (YAML::Exception &e) {
        RCLCPP_WARN(MAPDB_LOG, "LoadGridMap error occur when parsing map info file: %s", e.msg.c_str());
    }

    RCLCPP_INFO(MAPDB_LOG, "LoadGridMap, raw size (%d x %d), after encode: %ld",
               grid_img_.cols, grid_img_.rows, bit_map_->data.size());

    return 1;
}

void MapDatabase::updateLoadedMapId(const std::string &path, const std::string &uuid) {
    RCLCPP_INFO(MAPDB_LOG, "updateLoadedMapId path %s, uuid %s", path.c_str(), uuid.c_str());
    std::string FLAG_STR = "/ginger_maps/release/";
    if(!uuid.empty()) {
        map_uuid_ = uuid;
    } else {
        int index = path.find(FLAG_STR);
        if(index < path.length()) {
            std::string temp_str = path.substr(index + FLAG_STR.length(), path.length());
            int slash = temp_str.find("/");
            std::string mapId = temp_str.substr(0, slash);
            map_uuid_ = mapId;
            RCLCPP_INFO(MAPDB_LOG, "updateLoadedMapId mapId %s", mapId.c_str());
        }
    }
}

bool MapDatabase::loadSemanticsPoi(const std::string& poi_file) {
    YAML::Node poi_node;
    try {
        poi_node = YAML::LoadFile(poi_file);
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadSemanticsPoi cannot open map poi file: %s", e.msg.c_str());
        return false;
    }
    //ginger_msgs::Mode tag_pose_mode_srv;
    //geometry_msgs::msg::Quaternion q;
    tf2::Quaternion q;
    //std::vector<geometry_msgs::msg::PoseStamped> poses_stamped;
    geometry_msgs::msg::PoseStamped pose_stamped;
    std::map<std::string, sps_common_msgs::msg::PixelPose> lift_inner_poi;
    std::map<std::string, sps_common_msgs::msg::PixelPose> lift_out_poi;
    std::map<std::string, std::vector<sps_common_msgs::msg::PixelPose>> lift_entry_poi;
    std::map<std::string, std::vector<sps_common_msgs::msg::PixelPose>> lift_corner_poi;
    try {
        for (auto iter = poi_node.begin(); iter != poi_node.end(); ++iter) {
            if (!iter->second.IsMap()) {
                continue;
            }
            if (!iter->second["x"].IsDefined() || !iter->second["y"].IsDefined() ||
                        !iter->second["yaw"].IsDefined()) {
                auto p_name = iter->first.as<std::string>();
                RCLCPP_ERROR_STREAM(MAPDB_LOG, "LoadSemanticsPoi poi \'" << p_name << "\' format error");
                continue;
            }
            std::string group_id("-1");
            if (iter->second["group_id"].IsDefined()) {
                group_id = iter->second["group_id"].as<std::string>();
            }
            sps_common_msgs::msg::PixelPose pose;
            int type = iter->second["type"].as<int>();
            pose.x = iter->second["x"].as<float>();
            pose.y = iter->second["y"].as<float>();
            pose.theta = iter->second["yaw"].as<float>();
            RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi {\"%s\": {\"group_id\": \"%s\", \"x\": %f, \"y\": %f, \"yaw\": %f, \"type\": %d}}",
                   iter->first.as<std::string>().c_str(), group_id.c_str(), pose.x, pose.y, pose.theta, type);

            CommonPoi common_poi;
            common_poi.name = iter->first.as<std::string>();
            common_poi.pixel_pose = pose;
            common_poi.poi_type = PoiType(type);
            all_poi_.insert(std::make_pair(common_poi.name, common_poi));
            switch (PoiType(type)) {
                case NORMAL_POI: { //普通兴趣点
                    NormalPoi normal_poi;
                    normal_poi.name = iter->first.as<std::string>();
                    normal_poi.pixel_pose = pose;
                    normal_poi_.insert(std::make_pair(normal_poi.name, normal_poi));
                    break;
                }
                case CHARGER_POI: {//充电点
                    ChargerPoi charger_poi;
                    charger_poi.name = iter->first.as<std::string>();
                    charger_poi.pixel_pose = pose;
                    charger_poi_.insert(std::make_pair(charger_poi.name, charger_poi));
                    break;
                }
                case LIFT_ENTRY_POI: {
                    auto entry_itor = lift_entry_poi.find(group_id);
                    if (entry_itor == lift_entry_poi.end()) {
                        std::vector<sps_common_msgs::msg::PixelPose> entry_poi_tmp;
                        entry_poi_tmp.push_back(pose);
                        lift_entry_poi.insert(std::make_pair(group_id, entry_poi_tmp));
                    } else {
                        // ROS_ERROR("GingerMap: lift waiting poi has repeat group id");
                        entry_itor->second.push_back(pose);
                    }
                    break;
                }
                case LIFT_INNER_POI: {
                    if (lift_inner_poi.find(group_id) == lift_inner_poi.end()) {
                        lift_inner_poi.insert(std::make_pair(group_id, pose));
                    } else {
                        RCLCPP_ERROR(MAPDB_LOG, "LoadSemanticsPoi lift inner poi has repeat group id");
                    }
                    break;
                }
                case LIFT_CORNER_POI: {
                    auto corner_iter = lift_corner_poi.find(group_id);
                    if (corner_iter == lift_corner_poi.end()) {
                        std::vector<sps_common_msgs::msg::PixelPose> corner_pose;
                        corner_pose.push_back(pose);
                        lift_corner_poi.insert(std::make_pair(group_id, corner_pose));
                    } else {
                        corner_iter->second.push_back(pose);
                    }
                    break;
                }
                case DOOR_POI: {
                    //TODO:
                    break;
                }
                case LIFT_OUT_POI: {
                    if (lift_out_poi.find(group_id) == lift_out_poi.end()) {
                        lift_out_poi.insert(std::make_pair(group_id, pose));
                    } else {
                        RCLCPP_ERROR(MAPDB_LOG, "LoadSemanticsPoi lift out poi has repeat group id: %s", group_id.c_str());
                    }
                    break;
                }
                case LIFT_WAITING_POI: {
                    WaitingPoi wait_poi_tmp;
                    wait_poi_tmp.name = iter->first.as<std::string>();
                    wait_poi_tmp.pixel_waiting_poi = pose;
                    lift_waiting_Poi_.insert(std::make_pair(wait_poi_tmp.name, wait_poi_tmp));
                    break;
                }
                case CHARGER_WAITING_POI: {
                    WaitingPoi wait_poi_tmp;
                    wait_poi_tmp.name = iter->first.as<std::string>();
                    wait_poi_tmp.pixel_waiting_poi = pose;
                    charger_waiting_Poi_.insert(std::make_pair(wait_poi_tmp.name, wait_poi_tmp));
                    break;
                }
                case TEM_STOP_POI: {
                    TempStopPoi temp_stop_poi;
                    temp_stop_poi.name = iter->first.as<std::string>();
                    temp_stop_poi.pixel_pose = pose;
                    temp_stop_poi_.insert(std::make_pair(temp_stop_poi.name, temp_stop_poi));
                    break;
                }
                case FOOD_DELIVERY_POI: {
                    FoodDeliveryPoi food_delivery_poi;
                    food_delivery_poi.name = iter->first.as<std::string>();
                    food_delivery_poi.pixel_pose = pose;
                    food_delivery_poi_.insert(std::make_pair(food_delivery_poi.name, food_delivery_poi));
                    break;
                }
                case DISINFECTION_WAREHOUSE_POI: {
                    DisinfectionWarehousePoi disinfection_warehouse_poi;
                    disinfection_warehouse_poi.name = iter->first.as<std::string>();
                    disinfection_warehouse_poi.pixel_pose = pose;
                    disinfection_warehouse_poi_.insert(std::make_pair(disinfection_warehouse_poi.name, disinfection_warehouse_poi));
                    break;
                }
                case TAG_POI: {
                    RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi Pixel pose ==> id: %s, x: %f, y: %f, z: %f ==> roll: %f, pitch: %f, yaw: %f",
                        iter->first.as<std::string>().c_str(),
                        iter->second["x"].as<double>(),
                        iter->second["y"].as<double>(),
                        iter->second["x"].as<double>(),
                        iter->second["roll"].as<double>(),
                        iter->second["pitch"].as<double>(),
                        iter->second["yaw"].as<double>()
                    );

                    pose_stamped.header.frame_id = iter->first.as<std::string>();
                    pose_stamped.pose.position.x = iter->second["x"].as<double>() * occ_map_->info.resolution + occ_map_->info.origin.position.x;
                    pose_stamped.pose.position.y = (occ_map_->info.height - iter->second["y"].as<double>() - 1) * occ_map_->info.resolution + occ_map_->info.origin.position.y;
                    pose_stamped.pose.position.z = iter->second["x"].as<double>() * occ_map_->info.resolution;

                    double world_roll = getWorldRadian(iter->second["roll"].as<double>());
                    double world_pitch = getWorldRadian(iter->second["pitch"].as<double>());
                    double world_yaw = getWorldRadian(iter->second["yaw"].as<double>());

                    q.setRPY(world_roll, world_pitch, world_yaw);
                    q.normalize();
                    pose_stamped.pose.orientation = tf2::toMsg(q);
                    RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi World pose ==> id: %s, x: %f, y: %f, z: %f ==> x: %f, y: %f, z: %f, w: %f",
                        pose_stamped.header.frame_id.c_str(),
                        pose_stamped.pose.position.x,
                        pose_stamped.pose.position.y,
                        pose_stamped.pose.position.z,
                        pose_stamped.pose.orientation.x,
                        pose_stamped.pose.orientation.y,
                        pose_stamped.pose.orientation.z,
                        pose_stamped.pose.orientation.w
                    );
                    poses_stamped_.push_back(pose_stamped);
                    break;
                }
                default: {
                    RCLCPP_WARN(MAPDB_LOG, "LoadSemanticsPoi invalid poi type: %d", type);
                    break;
                }
            }
        }
      /**
      if (!poses_stamped.empty()) {
      // call service to zero begin navi mode
      tag_pose_mode_srv.request.mode_type = 1;  // ginger_msgs::ModeRequest::SLAM_LOC;
      tag_pose_mode_srv.request.tag_id_pose = poses_stamped;
      bool res = tag_pose_service_client_.call(tag_pose_mode_srv);
      RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi response ==> feedback: %s, result: %d",
               tag_pose_mode_srv.response.feedback.c_str(),
               tag_pose_mode_srv.response.result
      );
      if (res) {
        RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoiCall tag pose server to set mode SLAM_LOC success!");
      }
      else {
        RCLCPP_WARN(MAPDB_LOG, "LoadSemanticsPoi Call tag pose server failed!");
      };
      }
      else {
      RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi Call No tag poi found in \"%s\"!", poi_file.c_str());
      }
      **/
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadSemanticsPoi error occur when parsing map poi file: %s", e.msg.c_str());
        return false;
    }

    // process lift poi
    processLiftPoi(lift_entry_poi, lift_out_poi, lift_inner_poi, lift_corner_poi, lift_poi_);

    //debug
    for (auto lift_tmp : lift_poi_) {
        RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi lift id: %s", lift_tmp.first.c_str());
        RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi lift inner: world: (%f, %f), pixel: (%f, %f, %f)", lift_tmp.second.inner_poi.position.x,
            lift_tmp.second.inner_poi.position.y, lift_tmp.second.pixel_inner_poi.x,
            lift_tmp.second.pixel_inner_poi.y, lift_tmp.second.pixel_inner_poi.theta);
        RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi lift door(world): %f, %f, %f, %f", lift_tmp.second.door_poi[0].position.x,
            lift_tmp.second.door_poi[0].position.y, lift_tmp.second.door_poi[1].position.x,
            lift_tmp.second.door_poi[1].position.y);
        RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi lift corner: (%f, %f), (%f, %f), (%f, %f), (%f, %f)",
            lift_tmp.second.corner_poi[0].position.x, lift_tmp.second.corner_poi[0].position.y,
            lift_tmp.second.corner_poi[1].position.x, lift_tmp.second.corner_poi[1].position.y,
            lift_tmp.second.corner_poi[2].position.x, lift_tmp.second.corner_poi[2].position.y,
            lift_tmp.second.corner_poi[3].position.x, lift_tmp.second.corner_poi[3].position.y);
    }

    RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsPoi poi info: normal_poi: %d, charger poi: %d, charger waiting: %d, lift: %d, lift waiting: %d, door: %d, temp stop: %lu, food delivery: %lu, disinfection warehouse: %lu",
            int(normal_poi_.size()), int(charger_poi_.size()), int(charger_waiting_Poi_.size()),
            int(lift_poi_.size()), int(lift_waiting_Poi_.size()), int(door_poi_.size()),
            temp_stop_poi_.size(), food_delivery_poi_.size(), disinfection_warehouse_poi_.size());

    return true;
}

int MapDatabase::loadVirtualWall(const std::string& wall_file) {
    if (!virtual_wall_.start.empty()) {
        virtual_wall_.start.clear();
        virtual_wall_.end.clear();
    }
    YAML::Node wall_node;
    try {
        wall_node = YAML::LoadFile(wall_file);
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadVirtualWall cant open map virtual wall file: %s", e.msg.c_str());
    }
    try {
        for (auto iter = wall_node.begin(); iter != wall_node.end(); ++iter) {
            if (!iter->second.IsMap()) {
                continue;
            }
            if (!iter->second["startx"].IsDefined() ||
                    !iter->second["starty"].IsDefined() ||
                    !iter->second["endx"].IsDefined() ||
                    !iter->second["endy"].IsDefined()) {
                auto p_name = iter->first.as<std::string>();
                RCLCPP_ERROR_STREAM(MAPDB_LOG, "LoadVirtualWall virtual wall \'" << p_name << "\' format error");
                continue;
            }
            cv::Point start, end;
            geometry_msgs::msg::Point32 wstart, wend;
            start.x = iter->second["startx"].as<int>();
            start.y = iter->second["starty"].as<int>();
            end.x = iter->second["endx"].as<int>();
            end.y = iter->second["endy"].as<int>();
            RCLCPP_INFO(MAPDB_LOG, "LoadVirtualWall {\"%s\": {\"startx\": %d, \"endx\": %d, \"starty\": %d, \"endy\": %d}}",
                   iter->first.as<std::string>().c_str(), start.x, end.x, start.y, end.y);
            cv::line(grid_img_, start, end, cv::Scalar(150), 1, CV_AA);

            wstart.x = iter->second["startx"].as<float>();
            wstart.y = iter->second["starty"].as<float>();
            wend.x = iter->second["endx"].as<float>();
            wend.y = iter->second["endy"].as<float>();
            virtual_wall_.start.push_back(wstart);
            virtual_wall_.end.push_back(wend);
        }
        virtual_wall_.header.stamp = rclcpp::Time();
        RCLCPP_INFO(MAPDB_LOG, "LoadVirtualWall virtual wall loaded, size: %ld", virtual_wall_.start.size());
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadVirtualWall error occured when parsing map virtual wall file: %s", e.msg.c_str());
    }

    //debeug
    // cv::imwrite("/home/cloud/vwall.png", grid_img_);

    if (virtual_wall_.start.size() != 0) {
        static_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        static_map_->info = occ_map_->info;
        static_map_->header = occ_map_->header;
        static_map_->data.resize(occ_map_->data.size());
        for (uint32_t i = 0; i < grid_img_.rows; i++) {
            for (uint32_t j = 0; j < grid_img_.cols; j++) {
                uint32_t occ_index = j + (grid_img_.rows - i - 1) * grid_img_.cols;
                uint8_t pixel = grid_img_.at<uint8_t>(i, j);
                if (pixel > 220) {
                    static_map_->data[occ_index] = 0;
                } else {
                    static_map_->data[occ_index] = 100;
                }
            }
        }
    } else {
        static_map_ = occ_map_;
    }
    return 1;
}

bool MapDatabase::loadSemanticsArea(const std::string& area_file) {
    semantics_areas_.clear();
    YAML::Node area_node;
    try {
        area_node = YAML::LoadFile(area_file);
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadSemanticsArea cant open semantic area file: %s", e.msg.c_str());
        return false;
    }

    try {
        for (auto iter = area_node.begin(); iter != area_node.end(); ++iter) {
            if (!iter->second.IsMap()) {
                continue;
            }
            if (!iter->second["slope"].IsDefined() ||
                    !iter->second["type"].IsDefined() ||
                    !iter->second["speed"].IsDefined() ||
                    !iter->second["points"].IsDefined()) {
                auto p_name = iter->first.as<std::string>();
                RCLCPP_ERROR_STREAM(MAPDB_LOG, "LoadSemanticsArea semantic area file \'" << p_name << "\' format error");
                continue;
            }

            SemanticsArea curr_area;
            curr_area.name = iter->first.as<std::string>();
            curr_area.slope = iter->second["slope"].as<double>();
            curr_area.type = (AreaType) iter->second["type"].as<int>();
            curr_area.speed = iter->second["speed"].as<double>();

            if (iter->second["yaw"].IsDefined()) {
                curr_area.yaw = iter->second["yaw"].as<double>();
            } else {
                curr_area.yaw = 0.0;
            }

            YAML::Node point_node = iter->second["points"];
            RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsArea {\"%s\": {\"slope\": %f, \"type\": %d, \"speed\": %f, \"yaw\": %f, \"points\": %s}}",
                   curr_area.name.c_str(), curr_area.slope, curr_area.type, curr_area.speed, curr_area.yaw, Dump(point_node).c_str());
            for (size_t i = 0; i < point_node.size(); ++i) {
                geometry_msgs::msg::Point32 tmp_pt;
                tmp_pt.x = point_node[i]["x"].as<double>();
                tmp_pt.y = point_node[i]["y"].as<double>();
                tmp_pt.z = 0.0;
                curr_area.points.push_back(tmp_pt);
            }

            //兼容云端的5个兴趣点组成一个四边形区域(首尾两个点相同)
            if (curr_area.points.size() == 5) {
                int repeat_index = -1;
                for (int i=0; i<5; ++i) {
                    for (int j=i+1; j<5; ++j) {
                        if (curr_area.points[i].x == curr_area.points[j].x
                                && curr_area.points[i].y == curr_area.points[j].y) {
                            repeat_index = j;
                            break;
                        }
                    }
                }
                curr_area.points.erase(curr_area.points.begin() + repeat_index);
            }

            if (curr_area.points.size() == 4) {
                RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsArea semantics_areas_ index: %zu ==> name: %s, type: %d, speed: %f, slope: %f points: [{%f, %f}, {%f, %f}, {%f, %f}, {%f, %f}]",
                        semantics_areas_.size(), curr_area.name.c_str(), curr_area.type, curr_area.speed, curr_area.slope,
                        curr_area.points[0].x, curr_area.points[0].y,
                        curr_area.points[1].x, curr_area.points[1].y,
                        curr_area.points[2].x, curr_area.points[2].y,
                        curr_area.points[3].x, curr_area.points[3].y
                        );
                semantics_areas_.push_back(curr_area);
            } else {
                RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsArea area points count is invalid: %d", (int)curr_area.points.size());
            }
        }
        RCLCPP_INFO(MAPDB_LOG, "LoadSemanticsArea semantic area file loaded, size: %d", (int)semantics_areas_.size());
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPDB_LOG, "LoadSemanticsArea error occured when parsing semantic area wall file: %s", e.msg.c_str());
        return false;
    }
    return true;
}

int MapDatabase::processLiftPoi(
        const std::map<std::string, std::vector<sps_common_msgs::msg::PixelPose>>& entry_poi,
        const std::map<std::string, sps_common_msgs::msg::PixelPose>& out_poi,
        const std::map<std::string, sps_common_msgs::msg::PixelPose>& inner_poi,
        const std::map<std::string, std::vector<sps_common_msgs::msg::PixelPose>>& corner_poi,
        std::unordered_map<std::string, LiftPoi>& lift_poi) {
    lift_poi.clear();
    if (entry_poi.empty() || inner_poi.empty() || corner_poi.empty()) {
        RCLCPP_WARN(MAPDB_LOG, "ProcessLiftPoi invalid input poi, %d, %d, %d",
            int(entry_poi.size()), int(inner_poi.size()), int(corner_poi.size()));
        return 0;
    }

    // 兼容老的版本： 1个电梯，内外点没有group_id
    if (entry_poi.size() == 1 && inner_poi.size() == 1 && corner_poi.size() == 1) {
        LiftPoi cur_lift_poi;
        cur_lift_poi.name = corner_poi.begin()->first;
        cur_lift_poi.group_id = corner_poi.begin()->first;
        cur_lift_poi.pixel_inner_poi = inner_poi.begin()->second;
        cur_lift_poi.pixel_entry_poi = entry_poi.begin()->second;
        cur_lift_poi.pixel_corner_poi = corner_poi.begin()->second;
        if (out_poi.size() == 1) {
            cur_lift_poi.pixel_out_poi = out_poi.begin()->second;
        }
        lift_poi.insert(std::make_pair(cur_lift_poi.group_id, cur_lift_poi));
    } else {
        for (auto corner_tmp : corner_poi) {
            if (corner_tmp.second.size() != 4) {
                RCLCPP_WARN(MAPDB_LOG, "ProcessLiftPoi lift corner number is not valid: %d", int(corner_tmp.second.size()));
                continue;
            }
            auto entry_itor = entry_poi.find(corner_tmp.first);
            auto inner_itor = inner_poi.find(corner_tmp.first);
            auto out_itor = out_poi.find(corner_tmp.first);
            if (entry_itor == entry_poi.end() || inner_itor == inner_poi.end() || out_itor == out_poi.end()) {
                RCLCPP_WARN(MAPDB_LOG, "ProcessLiftPoi No corresponding inner or wating poi: %s", corner_tmp.first.c_str());
                continue;
            }
            LiftPoi cur_lift_poi;
            cur_lift_poi.name = corner_tmp.first;
            cur_lift_poi.group_id = corner_tmp.first;
            cur_lift_poi.pixel_out_poi = out_itor->second;
            cur_lift_poi.pixel_inner_poi = inner_itor->second;
            cur_lift_poi.pixel_entry_poi = entry_itor->second;
            cur_lift_poi.pixel_corner_poi = corner_tmp.second;
            lift_poi.insert(std::make_pair(cur_lift_poi.group_id, cur_lift_poi));
        }
    }

    if (lift_poi.empty()) {
        RCLCPP_ERROR(MAPDB_LOG, "setDefaultChargePile No valid lift poi");
        return 0;
    }
    optimizeLiftPoi(lift_poi);
    return lift_poi.size();
}

void MapDatabase::optimizeLiftPoi(std::unordered_map<std::string, LiftPoi>& lift_poi) {
    process_lift_poi_->ProcessLiftPoi(grid_img_, origin_, resolution_, lift_poi);
    //图像坐标系转map坐标系
    for (auto& lift_poi_tmp : lift_poi) {
        // inner poi
        getWorldPose(lift_poi_tmp.second.pixel_inner_poi, lift_poi_tmp.second.inner_poi);
        // out poi
        getWorldPose(lift_poi_tmp.second.pixel_out_poi, lift_poi_tmp.second.out_poi);
        // entry poi
        for (const auto& entry_tmp : lift_poi_tmp.second.pixel_entry_poi) {
            geometry_msgs::msg::Pose w_pose;
            getWorldPose(entry_tmp, w_pose);
            lift_poi_tmp.second.entry_poi.push_back(w_pose);
        }
        // door poi
        for (const auto& door_tmp : lift_poi_tmp.second.pixel_door_poi) {
            geometry_msgs::msg::Pose w_pose;
            getWorldPose(door_tmp, w_pose);
            lift_poi_tmp.second.door_poi.push_back(w_pose);
        }
        // corner poi
        for (const auto& corner_tmp : lift_poi_tmp.second.pixel_corner_poi) {
            geometry_msgs::msg::Pose w_pose;
            getWorldPose(corner_tmp, w_pose);
            lift_poi_tmp.second.corner_poi.push_back(w_pose);
        }
    }
}

void MapDatabase::clearAllMapData() {
    map_floor_ = 0;
    bit_map_ = std::make_shared<cm_msgs::msg::BitMap>();
    occ_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    static_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    map_uuid_ = "fake uuid";
    map_name_ = "fake name";
    map_loaded_ = false;

    all_poi_.clear();
    normal_poi_.clear();
    charger_poi_.clear();
    charger_waiting_Poi_.clear();
    lift_poi_.clear();
    lift_waiting_Poi_.clear();
    door_poi_.clear();
    temp_stop_poi_.clear();
    food_delivery_poi_.clear();
    disinfection_warehouse_poi_.clear();
    semantics_areas_.clear();
    poses_stamped_.clear();
    origin_.clear();
}

double MapDatabase::getWorldRadian(const double &pixel_radian) const {
    double pixel_angle = pixel_radian / M_PI * 180;
    double real_pixel_angle = fmod(450 - pixel_angle, 360);
    if (real_pixel_angle > 180) real_pixel_angle -= 360;
    double world_radian = real_pixel_angle / 180 * M_PI;
    return world_radian;
}

bool MapDatabase::getWorldPose(const sps_common_msgs::msg::PixelPose &p_pose, geometry_msgs::msg::Pose &w_pose) {
    if (occ_map_->data.empty()) {
        RCLCPP_ERROR(MAPDB_LOG, "getWorldPose occ_map is empty");
        return false;
    }
    w_pose.position.x = p_pose.x * occ_map_->info.resolution + occ_map_->info.origin.position.x;
    w_pose.position.y =
            (occ_map_->info.height - p_pose.y - 1) * occ_map_->info.resolution +
            occ_map_->info.origin.position.y;

    double yaw = (450 - p_pose.theta) * M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    w_pose.orientation = tf2::toMsg(q);

    return true;
}

std::string MapDatabase::getMapInfo(const std::string &key) {
    if (key == "id") {
        return map_uuid_;
    } else if (key == "name") {
        return map_name_;
    } else if (key == "version") {
        return map_version_;
    } else {
        return "";
    }
}

bool MapDatabase::getMapInfo(sps_common_msgs::msg::MapInfo &map_info) {
    map_info.header.frame_id = "map";
    map_info.header.stamp = rclcpp::Time();
    map_info.load_map_state = map_loaded_ ? sps_common_msgs::msg::MapInfo::LM_SUCCESS : sps_common_msgs::msg::MapInfo::LM_NONE;
    if(map_loaded_){
        map_info.map_uuid = map_uuid_;
        map_info.map_name = map_name_;
        map_info.map_version = map_version_;
        map_info.map_load_stamp_sec = load_time_.sec;
        map_info.origin_x = origin_[0];
        map_info.origin_y = origin_[1];
        map_info.resolution = resolution_;
        map_info.grid_width = occ_map_->info.width;
        map_info.grid_height = occ_map_->info.height;
        return true;
    } else {
        return false;
    }
}

bool MapDatabase::getBitMap(cm_msgs::msg::BitMap& bitMap) {
    if (!bit_map_->data.empty()){
        bitMap = *bit_map_;
        return true;
    } else {
        return false;
    }
}

bool MapDatabase::getOccMap(nav_msgs::msg::OccupancyGrid& occMap) {
    if (!occ_map_->data.empty()){
        occMap = *occ_map_;
        return true;
    } else {
        return false;
    }
}

bool MapDatabase::getStaticMap(nav_msgs::msg::OccupancyGrid& staticMap) {
    if (!static_map_->data.empty()){
        staticMap = *static_map_;
        return true;
    } else {
        return false;
    }
}

bool MapDatabase::getVirtualWall(sps_common_msgs::msg::VirtualWall& virtualWall) {
    if(!virtual_wall_.start.empty()){
        virtualWall = virtual_wall_;
        return true;
    } else {
        return false;
    }
}

bool MapDatabase::getAllPoints(std::unordered_map<std::string, CommonPoi> &all_points) {
    if (!all_poi_.empty()){
        all_points = all_poi_;
        return true;
    } else {
        return false;
    }
}

bool MapDatabase::getChargerPoints(std::unordered_map<std::string, ChargerPoi>& charger_points) {
    if (!charger_poi_.empty()){
        charger_points = charger_poi_;
        return true;
    } else {
        return false;
    }
}

bool MapDatabase::setDefaultChargePile(const std::string &charger_name, const sps_common_msgs::msg::PixelPose &pose) {
    bool result = false;
    auto find_res = charger_poi_.find(charger_name);
    if (find_res == charger_poi_.end()) {
        result = false;
        RCLCPP_ERROR(MAPDB_LOG, "setDefaultChargePile There is no charger named %s", charger_name.c_str());
    } else {
        if (find_res->second.pixel_pose.x != pose.x || find_res->second.pixel_pose.y != pose.y || find_res->second.pixel_pose.theta != pose.theta) {
            result = false;
            RCLCPP_ERROR(MAPDB_LOG, "setDefaultChargePile Using charger pose(%f, %f, %f) !=  request poe(%f, %f, %f)",
                        find_res->second.pixel_pose.x, find_res->second.pixel_pose.y, find_res->second.pixel_pose.theta,
                        pose.x, pose.y, pose.theta);
        } else {
            for (auto &item_pair: charger_poi_) {
                if (item_pair.second.is_default && item_pair.first != charger_name) {
                    item_pair.second.is_default = false;
                    RCLCPP_ERROR(MAPDB_LOG, "setDefaultChargePile Find a default charger named %s, set it to no default charger", item_pair.second.name.c_str());
                }
            }
            if (find_res->second.is_default) {
                result = true;
                RCLCPP_WARN(MAPDB_LOG, "setDefaultChargePile This charger named %s already is Non-default charger", charger_name.c_str());
            } else {
                find_res->second.is_default = true;
                result = true;
                RCLCPP_INFO(MAPDB_LOG, "setDefaultChargePile Find a charger named %s, now set it to default charger", charger_name.c_str());
            }
        }
    }

    return result;
}

bool MapDatabase::getSemanticAreaInfo(std::vector<sps_common_msgs::msg::SemanticsArea>& semantics_areas) {
    if (semantics_areas_.empty()) return false;

    for (int area_index = 0; area_index < semantics_areas_.size(); ++area_index) {
        const auto &area_tmp = semantics_areas_[area_index];
        sps_common_msgs::msg::SemanticsArea area_msg;
        area_msg.header.frame_id = map_frame_;
        area_msg.header.stamp = rclcpp::Time();
        area_msg.name = area_tmp.name;
        area_msg.slope = area_tmp.slope;
        area_msg.type = area_tmp.type;
        area_msg.speed = area_tmp.speed;
        area_msg.yaw = area_tmp.yaw;
        area_msg.points = area_tmp.points;
        semantics_areas.push_back(area_msg);
    }
    return true;
}

bool MapDatabase::getSemanticAreaInfo(const double pose_x, const double pose_y, double &area_speed,
            AreaType &area_type, sps_common_msgs::msg::SpecialAreas &special_areas) {
    area_speed = -1.0;
    area_type = UNDEFINED_AREA;
    if (semantics_areas_.empty()) return false;
    //map pose -> pixel pose
    geometry_msgs::msg::Point32 pixel_xy;
    pixel_xy.x = static_cast<int>((pose_x - occ_map_->info.origin.position.x) / occ_map_->info.resolution);
    pixel_xy.y = static_cast<int>(occ_map_->info.height - (pose_y - occ_map_->info.origin.position.y) / occ_map_->info.resolution);
    RCLCPP_INFO(MAPDB_LOG, "GetSemanticAreaInfo pose_x:%f, pose_y:%f, pixel_xy.x:%f, pixel_xy.y:%f",
          pose_x, pose_y, pixel_xy.x, pixel_xy.y);

    std::vector<int> hit_area_indexes;
    // for (auto& area_tmp : semantics_areas_) {
    for (int area_index = 0; area_index < semantics_areas_.size(); ++area_index) {
        const auto &area_tmp = semantics_areas_[area_index];
        if (isInRectangle(area_tmp.points[0], area_tmp.points[1], area_tmp.points[2], area_tmp.points[3], pixel_xy)) {
            hit_area_indexes.push_back(area_index);
        }
    }
    bool res = false;
  
    if (!hit_area_indexes.empty()) {
        double min_navi_vel = 1000;
        std::string string_hit_area_types;
        rclcpp::Clock clock(RCL_ROS_TIME);
        for (int hit_area_index : hit_area_indexes) {
            SemanticsArea hit_area = semantics_areas_[hit_area_index];
            string_hit_area_types += std::to_string(hit_area.type) + " ";
            special_areas.area_types.push_back(hit_area.type);
            switch (hit_area.type) {
                case UNDEFINED_AREA:
                    break;
                case GENERAL_SPEED_LIMIT_AREA:
                    if (min_navi_vel > hit_area.speed && hit_area.speed > 0.0) {
                        min_navi_vel = hit_area.speed;
                    }
                    break;
                case STEEP_SLOPE_AREA:
                    if (min_navi_vel > hit_area.speed && hit_area.speed > 0.0) {
                        min_navi_vel = hit_area.speed;
                    }
                    break;
                case CLOSE_SONAR_AREA:
                    area_type = CLOSE_SONAR_AREA;
                    if (hit_area.speed < min_navi_vel && hit_area.speed > 0.0) {
                        min_navi_vel = hit_area.speed;
                    }
                    break;
                case LONG_CORRIDOR_AREA:
                    area_type = LONG_CORRIDOR_AREA;
                    break;
                case NARROW_CHANNEL_AREA:
                    area_type = NARROW_CHANNEL_AREA;
                    break;
                case SPECIAL_INDUCTION_AREA:
                    area_type = SPECIAL_INDUCTION_AREA;
                    break;
                case NOISE_AREA:
                    area_type = NOISE_AREA;
                    break;
            }
        }
        RCLCPP_INFO_THROTTLE(MAPDB_LOG, clock, 5000, "GetSemanticAreaInfo Robot Now In Areas: [%s]", string_hit_area_types.c_str());
        if (min_navi_vel != 1000) {
            area_speed = min_navi_vel;
        }
        res = true;
    }

    // if (!area_navi_vel.empty()) {
    //   area_speed = *(std::min_element(area_navi_vel.begin(),area_navi_vel.end()));
    //   res = true;
    // }

    return res;
}

bool MapDatabase::getLiftPoints(std::unordered_map<std::string, LiftPoi>& lift_points) {
    if (!lift_poi_.empty()){
        lift_points = lift_poi_;
        return true;
    } else {
        return false;
    }
}

bool MapDatabase::getPosesStamped(std::vector<geometry_msgs::msg::PoseStamped>& poses_stamped) {
    if (!poses_stamped_.empty()){
        poses_stamped = poses_stamped_;
        return true;
    } else {
        return false;
    }
}

bool MapDatabase::isInRectangle(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2,
                              const geometry_msgs::msg::Point32& p3, const geometry_msgs::msg::Point32& p4,
                              const geometry_msgs::msg::Point32& p) {

    return getCross(p4,p3,p) * getCross(p2,p1,p) >= 0 && getCross(p3,p2,p) * getCross(p1,p4,p) >= 0;
}

float MapDatabase::getCross(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2,
                          const geometry_msgs::msg::Point32& p) {

    return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
}

void MapDatabase::updateTagPoses(const std::vector<TagPoi> tag_poses) {
    RCLCPP_INFO(MAPDB_LOG, "updateTagPoses tag_poses.size():%ld", tag_poses.size());
    tag_poses_->clear();
    tag_poses_ = std::make_shared<std::vector<TagPoi>>(tag_poses);
}

void MapDatabase::updateMapDataInMapping(const nav_msgs::msg::OccupancyGrid& occ_map, cm_msgs::msg::BitMap& bit_map) {
    map_uuid_ = "mapping";
    map_name_ = "mapping";

    // publish bitmap
    bit_map_->stamp = rclcpp::Time();
    bit_map_->width = occ_map.info.width;
    bit_map_->height = occ_map.info.height;
    bit_map_->data.clear();
    bit_map_->data.resize(bit_map_->width * bit_map_->height);
    cv::Mat tmp(bit_map_->height, bit_map_->width, CV_8UC1);
    for (uint32_t i = 0; i < bit_map_->height; i++) {
        for (uint32_t j = 0; j < bit_map_->width; j++) {
            uint32_t index = j + (bit_map_->height - i - 1) * bit_map_->width;
            if (occ_map.data[index] < 0) {
                tmp.at<uint8_t>(i, j) = 205;  // unknown place
            } else if (occ_map.data[index] <= threshold_free_) {
                tmp.at<uint8_t>(i, j) = 254;  // free place [0, thres_free]
            } else if (occ_map.data[index] > threshold_occupied_) {  // occ place (occ, 255]
                tmp.at<uint8_t>(i, j) = 0;
            } else {
                tmp.at<uint8_t>(i, j) = 205;
            }
        }
    }
    cv::imencode(".png", tmp, bit_map_->data);
    RCLCPP_INFO(MAPDB_LOG, "update_occ_map_callback Update Map, raw size (%d x %d), after encode: %ld",
            tmp.cols, tmp.rows, bit_map_->data.size());

    // record origin
    origin_.clear();
    origin_.push_back(occ_map.info.origin.position.x);
    origin_.push_back(occ_map.info.origin.position.y);
    origin_.push_back(tf2::getYaw(occ_map.info.origin.orientation));

    occ_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(occ_map);
    occ_map_->header.stamp = rclcpp::Time();
    occ_map_->info.map_load_time = rclcpp::Time();

    bit_map = *bit_map_;
}
