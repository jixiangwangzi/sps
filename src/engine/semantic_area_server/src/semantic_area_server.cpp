#include "semantic_area_server/semantic_area_server.hpp"

#include <chrono>

rclcpp::Logger AREA_LOG = rclcpp::get_logger("SemanticArea");

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SemanticAreaServer>();
    rclcpp::executors::SingleThreadedExecutor exector;
    exector.add_node(node);
    exector.spin();
    rclcpp::shutdown();
    return 0;
}

SemanticAreaServer::SemanticAreaServer() : Node("SemanticAreaServer") {
    RCLCPP_INFO(AREA_LOG, "hello this is SemanticAreaServer");
    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    transforms_init();
    client_init();
    publish_init();
    subscription_init();
}

SemanticAreaServer::~SemanticAreaServer() {
}

void SemanticAreaServer::transforms_init() {

}

void SemanticAreaServer::client_init() {
    get_semantics_areas_client_ = this->create_client<sps_common_msgs::srv::GetSemanticsAreas>("/get_semantic_area_info");
}

void SemanticAreaServer::publish_init() {
    special_areas_pub_ = this->create_publisher<sps_common_msgs::msg::SpecialAreas>("/SpecialAreas",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    area_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/area_nav_vel",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    sonar_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/sonar_scan",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void SemanticAreaServer::subscription_init() {
    occ_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/occ_map", 1,
                                            std::bind(&SemanticAreaServer::occ_map_callback, this, std::placeholders::_1));
    navi_state_sub_ = this->create_subscription<sps_common_msgs::msg::SpsNaviState>("/system_navi_state", 1,
                                            std::bind(&SemanticAreaServer::navi_state_callback, this, std::placeholders::_1));
    local_status_sub_ = this->create_subscription<sps_common_msgs::msg::LocalizationStatus>("/localization_status", 1,
                                            std::bind(&SemanticAreaServer::local_status_callback, this, std::placeholders::_1));
    sps_ultrasonic_sub_ = this->create_subscription<cm_msgs::msg::SpsUltrasonic>("/sps_ultrasonic", 1,
                                            std::bind(&SemanticAreaServer::ultrasonic_callback, this, std::placeholders::_1));
    navi_type_sub_ = this->create_subscription<std_msgs::msg::Int32>("/navi_type", 1,
                                            std::bind(&SemanticAreaServer::navi_type_callback, this, std::placeholders::_1));
}

void SemanticAreaServer::navi_state_callback(const sps_common_msgs::msg::SpsNaviState::SharedPtr msg) {
    RCLCPP_INFO(AREA_LOG, "navi_state_callback navi_state:%d, navi_type_:%d", msg->navi_state, navi_type_);
    if(navi_type_ != 1 && navi_type_ != 3) {
        if(msg->navi_state == sps_common_msgs::msg::SpsNaviState::ACTIVE ||
                          msg->navi_state == sps_common_msgs::msg::SpsNaviState::RUNNING) {
            if(!area_monitor_ && occ_map_.info.width > 0 && occ_map_.info.height > 0 && !semantics_areas_.empty()) {
                RCLCPP_INFO(AREA_LOG, "navi_state_callback create_wall_timer");
                area_monitor_ = true;
                area_timer_ = this->create_wall_timer(100ms, std::bind(&SemanticAreaServer::areaTimerCallback, this));
            }
        } else if(area_monitor_) {
            RCLCPP_INFO(AREA_LOG, "navi_state_callback cancel area_timer_");
            area_timer_->cancel();
            area_monitor_ = false;
        }
    }
}

void SemanticAreaServer::navi_type_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    int pub_sonar = 5;
    navi_type_ = msg->data;
    RCLCPP_INFO(AREA_LOG, "navi_type_callback navi_type_:%d", navi_type_);

    if(navi_type_ == 1 || navi_type_ == 3) {
        pub_sonar = 3;
    }
    setPubSonar(pub_sonar);
}

void SemanticAreaServer::local_status_callback(const sps_common_msgs::msg::LocalizationStatus::SharedPtr msg) {
    latest_loc_pose_ = msg->pose;
    latest_loc_status_ = msg->status;
}

void SemanticAreaServer::occ_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    occ_map_ = *msg;
    RCLCPP_INFO(AREA_LOG, "occ_map_callback map info width %d, height %d", occ_map_.info.width, occ_map_.info.height);

    getSemanticsAreas();
}

void SemanticAreaServer::ultrasonic_callback(const cm_msgs::msg::SpsUltrasonic::SharedPtr msg) {
    int l = 0, m = 1, r = 2;
    int sum_weight = 0;

    int br = 3, bl = 4;
    bool br_blocked = false;
    bool bl_blocked = false;

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = rclcpp::Time();
    scan.header.frame_id = sonar_frame_;
    scan.angle_min = -1 * M_PI;  //-0.5236;
    scan.angle_max = M_PI;       // 0.5236;
    scan.range_min = 0.1;
    scan.range_max = 1.4;
    scan.angle_increment = 0.0175;
    scan.scan_time = 0.1;
    scan.time_increment = 0.00125;
    scan.ranges.resize(360);
    for (auto &r : scan.ranges)
    {
        r = std::numeric_limits<float>::infinity();
    }

    if (pub_sonar_ == 0 || default_use_sonar_count_ == 0)
    {
        // sonar_scan_pub_->publish(scan);
        return;
    }
    else if (pub_sonar_ == 2 || default_use_sonar_count_ == 2)
    {
        // 只用后面的超声
        if (msg->distance[br] < 0.20)
        {
            double d = msg->distance[br] + 0.28;
            for (int i = 25; i < 35; i++)
            {
                scan.ranges[i] = d;
            }
        }
        if (msg->distance[bl] < 0.20)
        {
            double d = msg->distance[bl] + 0.28;
            for (int i = 325; i < 335; i++)
            {
                scan.ranges[i] = d;
            }
        }

        sonar_scan_pub_->publish(scan);
        return;
    }
    else if (pub_sonar_ == 3 || default_use_sonar_count_ == 3)
    {
        // 只用中间的超声和后面兩個超声
        if (msg->distance[m] < 0.28)
        {
            double d = msg->distance[m] + 0.28;
            for (int i = 170; i < 190; i++)
            {
                scan.ranges[i] = d;
            }
        }
        if (msg->distance[br] < 0.20)
        {
            double d = msg->distance[br] + 0.28;
            for (int i = 25; i < 35; i++)
            {
                scan.ranges[i] = d;
            }
        }
        if (msg->distance[bl] < 0.20)
        {
            double d = msg->distance[bl] + 0.28;
            for (int i = 325; i < 335; i++)
            {
                scan.ranges[i] = d;
            }
        }

        sonar_scan_pub_->publish(scan);
        return;
    }
    else if (pub_sonar_ == 5 || default_use_sonar_count_ == 5)
    {
        if (msg->distance[r] < side_detect_range_)
        {
            sum_weight += R_WEIGHT;
        }
        if (msg->distance[m] < detect_range_ + 0.03)
        {
            sum_weight += M_WEIGHT;
        }
        if (msg->distance[l] < side_detect_range_)
        {
            sum_weight += L_WEIGHT;
        }

        // enable back sonar
        if (msg->distance[br] < 0.20)
        {
            br_blocked = true;
        }
        if (msg->distance[bl] < 0.20)
        {
            bl_blocked = true;
        }

        auto blocked_sonar = (BlockedSonar)sum_weight;
        if (blocked_sonar == R_BLOCKED)
        {
            double d = msg->distance[r] + 0.28;
            for (int i = 150; i < 165; i++)
            {
                scan.ranges[i] = d;
            }
        }
        else if (blocked_sonar == MR_BLOCKED)
        {
            double distance = std::min(msg->distance[r], msg->distance[m]);
            double d = distance + 0.28;
            for (int i = 150; i < 195; i++)
            {
                scan.ranges[i] = d;
            }
        }
        else if (blocked_sonar == M_BLOCKED)
        {
            double d = msg->distance[m] + 0.28;
            for (int i = 160; i < 200; i++)
            {
                scan.ranges[i] = d;
            }
        }
        else if (blocked_sonar == LM_BLOCKED)
        {
            double distance = std::min(msg->distance[l], msg->distance[m]);
            double d = distance + 0.28;
            for (int i = 165; i < 210; i++)
            {
                scan.ranges[i] = d;
            }
        }
        else if (blocked_sonar == L_BLOCKED)
        {
            double d = msg->distance[l] + 0.28;
            for (int i = 195; i < 210; i++)
            {
                scan.ranges[i] = d;
            }
        }
        else if (blocked_sonar == LMR_BLOCKED || blocked_sonar == LR_BLOCKED)
        {
            double d = msg->distance[m] + 0.28;
            for (int i = 150; i < 210; i++)
            {
                scan.ranges[i] = d;
            }
        }

        // enable back sonar
        if (br_blocked)
        {
            double d = msg->distance[br] + 0.28;
            for (int i = 25; i < 35; i++)
            {
                scan.ranges[i] = d;
            }
        }
        if (bl_blocked)
        {
            double d = msg->distance[bl] + 0.28;
            for (int i = 325; i < 335; i++)
            {
                scan.ranges[i] = d;
            }
        }
        sonar_scan_pub_->publish(scan);
    }
}

void SemanticAreaServer::setPubSonar(int pub_sonar) {
    if(default_use_sonar_count_ != 0) {
        if(pub_sonar != pub_sonar_) {
            pub_sonar_ = pub_sonar;
            RCLCPP_INFO(AREA_LOG, "setSonarPub pub_sonar_:%d", pub_sonar_);
        }
    }
}

void SemanticAreaServer::areaTimerCallback() {
    geometry_msgs::msg::Point32 pixel_xy;
    pixel_xy.x = static_cast<int>(
        (latest_loc_pose_.position.x - occ_map_.info.origin.position.x) / occ_map_.info.resolution);
    pixel_xy.y = static_cast<int>(
       occ_map_.info.height -
       (latest_loc_pose_.position.y - occ_map_.info.origin.position.y) / occ_map_.info.resolution - 1);

    //RCLCPP_INFO(AREA_LOG, "areaTimerCallback pixel_xy.x %f, pixel_xy.y %f", pixel_xy.x, pixel_xy.y);

    std::vector<int> hit_area_indexes;
    // for (auto& area_tmp : semantics_areas_) {
    for (int area_index = 0; area_index < semantics_areas_.size(); ++area_index) {
        const auto &area_tmp = semantics_areas_[area_index];
        if (isInRectangle(area_tmp.points[0], area_tmp.points[1], area_tmp.points[2], area_tmp.points[3], pixel_xy)) {
            hit_area_indexes.push_back(area_index);
        }
    }

    int pub_sonar = 5;
    double area_speed = -1.0;
    sps_common_msgs::msg::SpecialAreas special_areas;
    if (!hit_area_indexes.empty()) {
        double min_navi_vel = 1000;
        std::string string_hit_area_types;
        rclcpp::Clock clock(RCL_ROS_TIME);
        for (int hit_area_index : hit_area_indexes) {
            sps_common_msgs::msg::SemanticsArea hit_area = semantics_areas_[hit_area_index];
            string_hit_area_types += std::to_string(hit_area.type) + " ";
            special_areas.area_types.push_back(hit_area.type);
            switch (hit_area.type) {
                case sps_common_msgs::msg::SemanticsArea::UNDEFINED_AREA:
                    break;
                case sps_common_msgs::msg::SemanticsArea::GENERAL_SPEED_LIMIT_AREA:
                    if (min_navi_vel > hit_area.speed && hit_area.speed > 0.0) {
                        min_navi_vel = hit_area.speed;
                    }
                    break;
                case sps_common_msgs::msg::SemanticsArea::STEEP_SLOPE_AREA:
                    if (min_navi_vel > hit_area.speed && hit_area.speed > 0.0) {
                        min_navi_vel = hit_area.speed;
                    }
                    break;
                case sps_common_msgs::msg::SemanticsArea::CLOSE_SONAR_AREA:
                    //area_type = CLOSE_SONAR_AREA;
                    if (hit_area.speed < min_navi_vel && hit_area.speed > 0.0) {
                        min_navi_vel = hit_area.speed;
                    }
                    pub_sonar = 2; //只打开后面两个超声
                    break;
            }
        }
        RCLCPP_INFO_THROTTLE(AREA_LOG, clock, 5000, "areaTimerCallback Robot Now In Areas: [%s]", string_hit_area_types.c_str());
        if (min_navi_vel != 1000) {
            area_speed = min_navi_vel;
        }
    }

    setPubSonar(pub_sonar);

    geometry_msgs::msg::Twist max_vel;
    max_vel.linear.x = area_speed;
    max_vel.angular.z = 0.3;
    area_vel_pub_->publish(max_vel);

    special_areas_pub_->publish(special_areas);
}

void SemanticAreaServer::getSemanticsAreas() {
    semantics_areas_.clear();

    auto get_semantics_areas_req = std::make_shared<sps_common_msgs::srv::GetSemanticsAreas::Request>();
    get_semantics_areas_req->header.frame_id = "map";
    get_semantics_areas_req->header.stamp = rclcpp::Time();

    if(!get_semantics_areas_client_->wait_for_service(1000ms)) {
        RCLCPP_WARN(AREA_LOG, "getSemanticsAreas service can not connected!");
        return;
    }

    get_semantics_areas_client_->async_send_request(get_semantics_areas_req,
                                std::bind(&SemanticAreaServer::semantics_areas_callback, this, std::placeholders::_1));

    /**
    using ServiceResponseFuture = rclcpp::Client<sps_common_msgs::srv::GetSemanticsAreas>::SharedFuture;
	auto response_received_callback = [this](ServiceResponseFuture future) {
            auto resp_data = future.get();
	        if(resp_data->result == sps_common_msgs::srv::GetSemanticsAreas::Response::SUCCESS) {
                semantics_areas_ = resp_data->areas;
                RCLCPP_INFO(AREA_LOG, "semantics_areas_callback semantics_areas_ size: %ld", semantics_areas_.size());
	        } else {
                RCLCPP_INFO(AREA_LOG, "semantics_areas_callback response ==> result: %d", resp_data->result);
	        }
	    };
	auto future_result = get_semantics_areas_client_->async_send_request(get_semantics_areas_req, response_received_callback);
    **/
}

void SemanticAreaServer::semantics_areas_callback(rclcpp::Client<sps_common_msgs::srv::GetSemanticsAreas>::SharedFuture response) {
    auto resp_data = response.get();
    if(resp_data->result == sps_common_msgs::srv::GetSemanticsAreas::Response::SUCCESS) {
        semantics_areas_ = resp_data->areas;
        RCLCPP_INFO(AREA_LOG, "semantics_areas_callback semantics_areas_ size: %ld", semantics_areas_.size());
    } else {
        RCLCPP_INFO(AREA_LOG, "semantics_areas_callback response ==> result: %d", resp_data->result);
    }
}

bool SemanticAreaServer::isInRectangle(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2,
                              const geometry_msgs::msg::Point32& p3, const geometry_msgs::msg::Point32& p4,
                              const geometry_msgs::msg::Point32& p) {

    return getCross(p4,p3,p) * getCross(p2,p1,p) >= 0 && getCross(p3,p2,p) * getCross(p1,p4,p) >= 0;
}

float SemanticAreaServer::getCross(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2,
                          const geometry_msgs::msg::Point32& p) {

    return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
}
