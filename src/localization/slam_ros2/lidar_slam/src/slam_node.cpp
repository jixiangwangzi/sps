#include "lidar_slam/slam_node.h"
#include<string>
#include <glog/logging.h>
#include "Magick++.h"
#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Quaternion.h"
#include <libgen.h>

// #include "lidar_slam/GingerCartographerMapping.h"
#include "lidar_slam/SpsCartographerLoc.h"
namespace sps {
constexpr double kTfBufferCacheTimeInSeconds = 10.;
SlamNode::SlamNode(RosNodeHandlePtr& nh) {
    nh_ = nh;
    GetRosParams();
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock(), tf2::durationFromSec(kTfBufferCacheTimeInSeconds), nh_);
    tf2_cast_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh_);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    v_servers_.push_back(nh_->create_service<sps_common_msgs::srv::SetSlamModel>("set_slam_model", 
        std::bind(&SlamNode::SetSlamModelService, this, std::placeholders::_1, std::placeholders::_2)));
    v_servers_.push_back(nh_->create_service<sps_common_msgs::srv::AidedPose>("set_aided_pose", 
        std::bind(&SlamNode::SetAidedPose, this, std::placeholders::_1, std::placeholders::_2)));
    v_servers_.push_back(nh_->create_service<sps_common_msgs::srv::SpsLoadMap>("load_map", 
        std::bind(&SlamNode::LoadMap, this, std::placeholders::_1, std::placeholders::_2)));

    // v_sub_.push_back(nh_->create_subscription<nav_msgs::msg::OccupancyGrid>("occ_map", 
    //     rclcpp::SensorDataQoS(), std::bind(&SlamNode::SetOccMapCallback, this, std::placeholders::_1)));
    
    v_sub_.push_back(nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 
        rclcpp::SensorDataQoS(), std::bind(&SlamNode::SetInitPoseCallback, this, std::placeholders::_1)));
    // v_sub_.push_back(nh_->create_subscription<>("initialpose_self", 5, &SlamNode::SetInitPoseSelfCallback, this));
    // v_sub_.push_back(nh_->create_subscription<>("reloc_state", 2, &SlamNode::SetRelocalizationState, this));
    v_sub_.push_back(nh_->create_subscription<std_msgs::msg::Int32>("navi_type", 
        rclcpp::SensorDataQoS(),  std::bind(&SlamNode::naviTypeCB, this, std::placeholders::_1)));
    v_sub_.push_back(nh_->create_subscription<std_msgs::msg::Float32>("loc_confidence", 
        rclcpp::SensorDataQoS(),  std::bind(&SlamNode::GetLocConfidence, this, std::placeholders::_1)));
    std::string loc_message;
    if (!StartLocalizationModel(loc_message))  {
        slam_model_ = MODE_IDLE;
        LOG(ERROR) <<"SlamNode: Failed to start localization model";
    }
    else {
        LOG(ERROR) <<"SlamNode: Localization is running...";
    }
}

SlamNode::~SlamNode(){

}

void SlamNode::GetRosParams() {
// /*
    int cur_mapping_method = 2;
    nh_->get_parameter("mapping_model", cur_mapping_method);
    if(cur_mapping_method == 0)
    {
        mapping_model_ = MAP_CARTOGRAPHER;
        LOG(INFO) << "SlamNode: using cartographer for mapping";
    }
    else{
        LOG(ERROR) << "SlamNode: undefined mapping method";
    }
    LOG(INFO) << "SlamNode: Mapping Method: " << cur_mapping_method;
    
    int cur_loc_method = 0;
    nh_->get_parameter("localization_model", cur_loc_method);
    if(cur_loc_method == 0)
    {
        loc_model_ = LOC_CARTOGRAPHER;
        // ROS_INFO("SlamNode: using cartographer for localization");
    }
    else{
        LOG(ERROR) << "undefined loc_model!";
    }

    nh_->get_parameter("map_frame", map_frame_);
    nh_->get_parameter("odom_frame", odom_frame_);
    nh_->get_parameter("base_frame", base_frame_);

//   */
}



void SlamNode::SetSlamModelService(const std::shared_ptr<sps_common_msgs::srv::SetSlamModel::Request> req, 
                                std::shared_ptr<sps_common_msgs::srv::SetSlamModel::Response> res){
    bool res_flag = false;
    LOG(INFO) << "SlamNode: current Slam model: " << int(slam_model_);

    switch (req->slam_mode)
    {
        case sps_common_msgs::srv::SetSlamModel::Request::SLAM_IDLE:
            // 如果IDLE，则不论现在出于各种模式，强制设置成IDLE
            // 设置该模式的目的是，出现状态混乱后可以进行初始设置
            slam_model_ = MODE_IDLE;
            slam_obj_.reset();
            res_flag = true;
            res->description = "IDLE model has been set successfully";
            LOG(INFO) << "SlamNode: " << res->description.c_str();
            break;
        case sps_common_msgs::srv::SetSlamModel::Request::SLAM_LOC:
            // localization model
            if (slam_model_ == MODE_LOCALIZATION) {
                res->result = sps_common_msgs::srv::SetSlamModel::Response::REPEAT_CMD;
                res->description = "Localization model has been started, don't repeat";
                LOG(WARNING) << "SlamNode: " << res->description.c_str();
                return;
            }
            res_flag = StartLocalizationModel(res->description);
            break;
        
        case sps_common_msgs::srv::SetSlamModel::Request::SLAM_MAPPING:
            // mapping model
            if(slam_model_ == MODE_MAPPING 
                || slam_model_ == MODE_MAPPING_PART_INCREMENTAL
                || slam_model_ == MODE_MAPPING_ALL_INCREMENTAL_AND_ALIGN) {
                res->result = sps_common_msgs::srv::SetSlamModel::Response::REPEAT_CMD;
                res->description = "Mapping has been started, don't repeat";
                LOG(WARNING) << "SlamNode: " << res->description.c_str();
                return;
            }
            res_flag = StartMappingModel(res->description, MODE_MAPPING);
            break;
        
        case sps_common_msgs::srv::SetSlamModel::Request::SLAM_MAPPING_PART_INCREMENTAL:
            // mapping model
            if(slam_model_ == MODE_MAPPING 
                || slam_model_ == MODE_MAPPING_PART_INCREMENTAL
                || slam_model_ == MODE_MAPPING_ALL_INCREMENTAL_AND_ALIGN) {
                res->result = sps_common_msgs::srv::SetSlamModel::Response::REPEAT_CMD;
                res->description = "Mapping has been started, don't repeat";
                LOG(WARNING) << "SlamNode: " << res->description.c_str();
                return;
            }
            res_flag = StartMappingModel(res->description, MODE_MAPPING_PART_INCREMENTAL);
            if(slam_obj_ != nullptr){
                LOG(INFO) << "slam_obj_->setOccMap(occ_map_);";
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                slam_obj_->setOccMap(occ_map_);
                LOG(INFO) << "slam_obj_->setInitialPose(cur_pose_);";
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                slam_obj_->setInitialPose(cur_pose_);
                LOG(INFO) << "slam_obj_->setInitialPose(cur_pose_) end;";
            }
            else{
                LOG(ERROR) << "slam_obj_ is nullptr";
                res_flag = false;
            }
            break;

        case sps_common_msgs::srv::SetSlamModel::Request::SLAM_MAPPING_ALL_INCREMENTAL_AND_ALIGN:
            // mapping model
            if(slam_model_ == MODE_MAPPING 
                || slam_model_ == MODE_MAPPING_PART_INCREMENTAL
                || slam_model_ == MODE_MAPPING_ALL_INCREMENTAL_AND_ALIGN) {
                res->result = sps_common_msgs::srv::SetSlamModel::Response::REPEAT_CMD;
                res->description = "Mapping has been started, don't repeat";
                LOG(WARNING) << "SlamNode: " << res->description.c_str();
                return;
            }
            res_flag = StartMappingModel(res->description, MODE_MAPPING_ALL_INCREMENTAL_AND_ALIGN);
            if(slam_obj_ != nullptr){
                LOG(INFO) << "slam_obj_->setOccMap(occ_map_);";
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                slam_obj_->setOccMap(occ_map_);
                LOG(INFO) << "slam_obj_->setInitialPose(cur_pose_);";
                rclcpp::sleep_for(std::chrono::milliseconds(200));
                slam_obj_->setInitialPose(cur_pose_);
                LOG(INFO) << "slam_obj_->setInitialPose(cur_pose_) end;";
            }
            else{
                LOG(ERROR) << "slam_obj_ is nullptr";
                res_flag = false;
            }
            break;
        case sps_common_msgs::srv::SetSlamModel::Request::SLAM_MAPPING_ODOM_CALIB:
            // mapping model
            if( slam_model_ == MODE_MAPPING_ODOM_CALIB ) {
                res->result = sps_common_msgs::srv::SetSlamModel::Response::REPEAT_CMD;
                res->description = "Mapping has been started, don't repeat";
                LOG(WARNING) << "SlamNode: " << res->description.c_str();
                return;
            }
            res_flag = StartMappingModel(res->description, MODE_MAPPING_ODOM_CALIB);
            break;        
        default:
            LOG(ERROR) << "SlamNode: Not set mappig method";
            break;
    }

    if (res_flag)
    {
        res->result = sps_common_msgs::srv::SetSlamModel::Response::SUCCESS;
    }
    else
    {
        res->result = sps_common_msgs::srv::SetSlamModel::Response::FAILED;
    }

    LOG(INFO) << "SlamNode: current Slam model: " << int(slam_model_);
    return;
}


//map
                
/// Get the given subnode value.
/// The only reason this function exists is to wrap the exceptions in slightly nicer error messages,
/// including the name of the failed key
/// @throw YAML::Exception
template<typename T>
T yaml_get_value(const YAML::Node & node, const std::string & key)
{
    try {
        return node[key].as<T>();
    } catch (YAML::Exception & e) {
        std::stringstream ss;
        ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
        throw YAML::Exception(e.mark, ss.str());
    }
}

LoadParameters SlamNode::LoadMapYaml(const std::string& yaml_filename){
  LOG(INFO) << "yaml file name:" << yaml_filename;
  YAML::Node doc = YAML::LoadFile(yaml_filename);
  LoadParameters load_parameters;
  auto image_file_name = yaml_get_value<std::string>(doc, "image");
  LOG(INFO) << "image file name:" << image_file_name;
  if (image_file_name.empty()) {
    throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
  }
  if (image_file_name[0] != '/') {
    // dirname takes a mutable char *, so we copy into a vector
    std::vector<char> fname_copy(yaml_filename.begin(), yaml_filename.end());
    fname_copy.push_back('\0');
    image_file_name = std::string(dirname(fname_copy.data())) + '/' + image_file_name;
  }
  load_parameters.image_file_name = image_file_name;
  load_parameters.resolution = yaml_get_value<double>(doc, "resolution");
  load_parameters.origin = yaml_get_value<std::vector<double>>(doc, "origin");
  if (load_parameters.origin.size() != 3) {
    throw YAML::Exception(
            doc["origin"].Mark(), "value of the 'origin' tag should have 3 elements, not " +
            std::to_string(load_parameters.origin.size()));
  }
  load_parameters.free_thresh = yaml_get_value<double>(doc, "free_thresh");
  load_parameters.occupied_thresh = yaml_get_value<double>(doc, "occupied_thresh");

  auto map_mode_node = doc["mode"];
//   if (!map_mode_node.IsDefined()) {
  load_parameters.mode = MapMode::Trinary;
//   } 
//   else {
//     load_parameters.mode = map_mode_from_string(map_mode_node.as<std::string>());
//   }

  load_parameters.negate = false;

  LOG(INFO) << "[map_io]: resolution: " << load_parameters.resolution;
  LOG(INFO) << "[map_io]: origin[0]: " << load_parameters.origin[0];
  LOG(INFO) << "[map_io]: origin[1]: " << load_parameters.origin[1];
  LOG(INFO) << "[map_io]: origin[2]: " << load_parameters.origin[2];
  LOG(INFO) << "[map_io]: free_thresh: " << load_parameters.free_thresh;
  LOG(INFO) << "[map_io]: occupied_thresh: " << load_parameters.occupied_thresh;
  LOG(INFO) << "[map_io]: negate: " << load_parameters.negate;  //NOLINT

  return load_parameters;
}

void SlamNode::LoadMapFromFile(const LoadParameters& load_parameters){
  Magick::InitializeMagick(nullptr);
  nav_msgs::msg::OccupancyGrid msg;
  LOG(INFO) << "Loading image_file: " << load_parameters.image_file_name;
  Magick::Image img(load_parameters.image_file_name);

  // Copy the image data into the map structure
  msg.info.width = img.size().width();
  msg.info.height = img.size().height();

  tf2::Quaternion q;
  q.setRPY(0, 0, load_parameters.origin[2]);  // void returning function
  msg.info.resolution = load_parameters.resolution;
  msg.info.origin.position.x = load_parameters.origin[0];
  msg.info.origin.position.y = load_parameters.origin[1];
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation = tf2::toMsg(q);;

  // Allocate space to hold the data
  msg.data.resize(msg.info.width * msg.info.height);

  // Copy pixel data into the map structure
  for (size_t y = 0; y < msg.info.height; y++) {
    for (size_t x = 0; x < msg.info.width; x++) {
      auto pixel = img.pixelColor(x, y);

      std::vector<Magick::Quantum> channels = {pixel.redQuantum(), pixel.greenQuantum(),
        pixel.blueQuantum()};
      if (load_parameters.mode == MapMode::Trinary && img.matte()) {
        // To preserve existing behavior, average in alpha with color channels in Trinary mode.
        // CAREFUL. alpha is inverted from what you might expect. High = transparent, low = opaque
        channels.push_back(MaxRGB - pixel.alphaQuantum());
      }
      double sum = 0;
      for (auto c : channels) {
        sum += c;
      }
      /// on a scale from 0.0 to 1.0 how bright is the pixel?
      double shade = Magick::ColorGray::scaleQuantumToDouble(sum / channels.size());

      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied. Otherwise, it's vice versa.
      /// on a scale from 0.0 to 1.0, how occupied is the map cell (before thresholding)?
      double occ = (load_parameters.negate ? shade : 1.0 - shade);

      int8_t map_cell;
      switch (load_parameters.mode) {
        case MapMode::Trinary:
          if (load_parameters.occupied_thresh < occ) {
            map_cell = OCC_GRID_OCCUPIED;
          } else if (occ < load_parameters.free_thresh) {
            map_cell = OCC_GRID_FREE;
          } else {
            map_cell = OCC_GRID_UNKNOWN;
          }
          break;
        case MapMode::Scale:
          if (pixel.alphaQuantum() != OpaqueOpacity) {
            map_cell = OCC_GRID_UNKNOWN;
          } else if (load_parameters.occupied_thresh < occ) {
            map_cell = OCC_GRID_OCCUPIED;
          } else if (occ < load_parameters.free_thresh) {
            map_cell = OCC_GRID_FREE;
          } else {
            map_cell = std::rint(
              (occ - load_parameters.free_thresh) /
              (load_parameters.occupied_thresh - load_parameters.free_thresh) * 100.0);
          }
          break;
        case MapMode::Raw: {
            double occ_percent = std::round(shade * 255);
            if (OCC_GRID_FREE <= occ_percent &&
              occ_percent <= OCC_GRID_OCCUPIED)
            {
              map_cell = static_cast<int8_t>(occ_percent);
            } else {
              map_cell = OCC_GRID_UNKNOWN;
            }
            break;
          }
        default:
          throw std::runtime_error("Invalid map mode");
      }
      msg.data[msg.info.width * (msg.info.height - y - 1) + x] = map_cell;
    }
  }
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  msg.info.map_load_time = clock.now();
  msg.header.frame_id = "map";
  msg.header.stamp = clock.now();

  LOG(INFO) <<
    "map_io]: Read map " << load_parameters.image_file_name << ": " 
    << msg.info.width << " X " << msg.info.height << " map @ " << msg.info.resolution << " m/cell"
    << " data size: " << msg.data.size();
  occ_map_ = msg;
  LOG(INFO) << "occ_map_ data size: " << occ_map_.data.size();
}

void SlamNode::LoadMap(const std::shared_ptr<sps_common_msgs::srv::SpsLoadMap::Request> req, 
                        std::shared_ptr<sps_common_msgs::srv::SpsLoadMap::Response> res){
    std::string path = req->map_path + "/grid_map_layer/map.yaml"; //grid_map_layer
    LOG(INFO) << "load map path: " << path.c_str();
    LoadParameters loadParameters = LoadMapYaml(path);
    LoadMapFromFile(loadParameters);
    if((slam_model_ == MODE_LOCALIZATION) && (slam_obj_ != nullptr)){
        slam_obj_->setOccMap(occ_map_);
        res->result = res->SUCCESS;
        res->description = "load map success";
    }
    else{
        res->result = res->FAILED;
        res->description = "slam_model is not MODE_LOCALIZATION or slam_obj_ is nullptr";
    }
    LOG(INFO) << res->description.c_str();
}


void SlamNode::SetOccMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg) {
    if ((slam_model_ == MODE_LOCALIZATION) && (slam_obj_ != nullptr) ) {
        slam_obj_->setOccMap(*map_msg);
    }
    occ_map_.info.resolution = map_msg->info.resolution;
    occ_map_.info.origin.position.x = map_msg->info.origin.position.x;
    occ_map_.info.origin.position.y = map_msg->info.origin.position.y;
    occ_map_.info.origin.orientation = map_msg->info.origin.orientation;
    occ_map_.info.width = map_msg->info.width;
    occ_map_.info.height = map_msg->info.height;
    occ_map_.data = map_msg->data;
} 

//init loc
void SlamNode::SetAidedPose(const std::shared_ptr<sps_common_msgs::srv::AidedPose::Request> req,
                        std::shared_ptr<sps_common_msgs::srv::AidedPose::Response> res){
    
    if (slam_obj_ == nullptr || slam_model_ != MODE_LOCALIZATION) {
        LOG(ERROR) << "SLAM mode is not MODE_LOCALIZATION or slam_obj_ is nullptr, SetAidedPose Failed!";
        res->result = res->MODE_ERROR;
        res->description = "SetAidedPose while SLAM mode is wrong!";
        return;
    }
    sps_common_msgs::msg::InitialSlamPose initialPose;
    if (!req->poses.empty()) {
        for (const auto pose : req->poses) {
            geometry_msgs::msg::PoseWithCovarianceStamped msg;
            msg.pose.pose = pose;
            initialPose.initial_pose.push_back(msg);
        }   
    }
    initialPose.scene_type = req->scene_type;
    initialPose.optimize_type = req->match_type;
    using SetAidedPoseResult = sps_common_msgs::srv::AidedPose::Response;
    switch (slam_obj_->SetInitPoseSelf(initialPose))
    {
    case SetAidedPoseResult::SUCCESS :
        res->result = res->SUCCESS;
        res->description = "SetAidedPose success!";
        break;
    case SetAidedPoseResult::FAILED :
        res->result = res->FAILED;
        res->description = "SetAidedPose failed!";
        break;
    case SetAidedPoseResult::SETTING_POSE :
        res->result = res->SETTING_POSE;
        res->description = "SetAidedPose while last init pose is setting!";
    case SetAidedPoseResult::POSE_UNVALID:
        res->result = res->POSE_UNVALID;
        res->description = "SetAidedPose while init pose is unvalid!";
    default:
        res->result = res->FAILED;
        res->description = "SetAidedPose with unknown result type!";
        break;
    }
    LOG(INFO) << res->description.c_str();
}

void SlamNode::SetInitPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg) 
{
    // if (slam_model_ == MODE_LOCALIZATION) {
        if ((slam_obj_ != nullptr) && (slam_obj_->setInitialPose(*initial_pose_msg) == sps_common_msgs::srv::AidedPose::Response::SUCCESS)) {
            LOG(INFO) <<"SlamNode: Set initial pose successfully: "
                      << initial_pose_msg->pose.pose.position.x << " " <<initial_pose_msg->pose.pose.position.y;
        }
        else {
            LOG(ERROR) << "SlamNode: Failed to set initial pose";
        }
    // else {
    //     ROS_ERROR("SlamNode: Failed to set initial pose, because not in localization: %d", int(slam_model_));
    // }
}
/*
void SlamNode::SetInitPoseSelfCallback(const sps_common_msgs::InitialSlamPose& initial_pose_msg){
    if ((slam_obj_ != nullptr) && (!slam_obj_->SetInitPoseSelf(initial_pose_msg))) {
        ROS_ERROR("SlamNode: SetInitPoseSelf Failed to set initial pose");
    }
}
*/

// void SlamNode::SetRelocalizationState(const std_msgs::msg::Int32& reloc_state_msg){
//     if(slam_obj_ != nullptr){
//         slam_obj_->SetRelocalizationState(reloc_state_msg);
//     }
// }

void SlamNode::naviTypeCB(const std_msgs::msg::Int32& msg)
{
    if(slam_obj_ != nullptr){
        slam_obj_->naviTypeCB(msg);
    }
}


bool SlamNode::StartMappingModel(std::string& message, const SlamModel& slam_model) {
    if(slam_obj_ != nullptr){
        cur_pose_ = slam_obj_->GetRobotPose();
    }
    else{
        LOG(WARNING) << "slam_obj_ is nullptr";
    }
    slam_obj_.reset();
    slam_model_ = MODE_IDLE;
    bool res = false;

    switch (mapping_model_)
    {

        case MAP_CARTOGRAPHER:
            // slam_obj_.reset(new GingerCartographerMapping(nh_, tf2_buffer_, tf2_cast_, (int) slam_model));  
            message = std::string("Start mapping by using Cartographer");
            break;
        
        default:
            message = std::string("Mapping method is error");
            res = false;
            break;
    }

    if (slam_obj_ != nullptr) {
        slam_model_ = slam_model;
        slam_obj_->startUpdateMap();
        LOG(INFO) << "SlamNode: ", message.c_str();
        res = true;
    }
    else {
        // slam_model_ = MODE_IDLE;
        LOG(INFO) << "SlamNode: ", message.c_str();
    }
    return res;
}


bool SlamNode::StartLocalizationModel(std::string& message) {
    // default localization model
    slam_obj_.reset();
    bool res = false;

    switch (loc_model_)
    {
        case LOC_CARTOGRAPHER:
            LOG(INFO) <<"loc_model_ = LOC_CARTOGRAPHER !!!";
            if(nh_ == nullptr){
                LOG(ERROR) << "nh_ is nullptr";
            }
            slam_obj_ = std::make_shared<sps::SpsCartographerLoc>(nh_, tf2_buffer_, tf2_cast_);
            message = std::string("Start localization model by using cartographer method");
            res = true;
            break;
        default:
            message = std::string("Failed to start localization by using error loc method");
            res = false;
            break;
    }
    if (res && slam_obj_) {
        slam_model_ = MODE_LOCALIZATION;
        slam_obj_->stopUpdateMap();
        LOG(INFO) << "SlamNode: " << message.c_str();
    }
    else {
        // slam_model_ = MODE_IDLE;
        LOG(ERROR) << "SlamNode: " << message.c_str();
    }
    return res;
}

void SlamNode::GetLocConfidence(const std_msgs::msg::Float32& msg){
    loc_confidence_ = msg.data;
    // LOG(INFO) << "loc_confidence_: " << loc_confidence_;
}



} //namespace

