#include "nav2_costmap_2d/temporary_obstacle_layer.hpp"

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::TemporaryObstacleLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;

namespace nav2_costmap_2d
{

TemporaryObstacleLayer::~TemporaryObstacleLayer()
{
    for (auto & notifier : observation_notifiers_) {
        notifier.reset();
    }
}

void TemporaryObstacleLayer::onInitialize()
{
    bool track_unknown_space;
    double transform_tolerance;

    // The topics that we'll subscribe to from the parameter server
    std::string topics_string;

    // TODO(mjeronimo): these four are candidates for dynamic update
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
    declareParameter("max_obstacle_height", rclcpp::ParameterValue(2.0));
    // declareParameter("combination_method", rclcpp::ParameterValue(1));
    declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "footprint_clearing_enabled", footprint_clearing_enabled_);
    node->get_parameter(name_ + "." + "max_obstacle_height", max_obstacle_height_);
    // node->get_parameter(name_ + "." + "combination_method", combination_method_);
    node->get_parameter("track_unknown_space", track_unknown_space);
    node->get_parameter("transform_tolerance", transform_tolerance);
    node->get_parameter(name_ + "." + "observation_sources", topics_string);

    RCLCPP_INFO(
        logger_,
        "Subscribed to Topics: %s", topics_string.c_str());
    
    rolling_window_ = layered_costmap_->isRolling();

    if (track_unknown_space) {
        default_value_ = NO_INFORMATION;
    } else {
        default_value_ = FREE_SPACE;
    }

    TemporaryObstacleLayer::matchSize();
    current_ = true;
    was_reset_ = false;

    global_frame_ = layered_costmap_->getGlobalFrameID();

    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    std::string source;
    while (ss >> source) {
        // get the parameters for the specific topic
        double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
        std::string topic, sensor_frame, data_type;
        bool inf_is_valid;

        declareParameter(source + "." + "topic", rclcpp::ParameterValue(source));
        declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
        declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "data_type", rclcpp::ParameterValue(std::string("LaserScan")));
        declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(2.0));
        declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
        declareParameter(source + "." + "obstacle_max_range", rclcpp::ParameterValue(2.5));
        declareParameter(source + "." + "obstacle_min_range", rclcpp::ParameterValue(0.0));
        declareParameter(source + "." + "raytrace_max_range", rclcpp::ParameterValue(3.0));
        declareParameter(source + "." + "raytrace_min_range", rclcpp::ParameterValue(0.0));

        node->get_parameter(name_ + "." + source + "." + "topic", topic);
        node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);
        node->get_parameter(
        name_ + "." + source + "." + "observation_persistence",
        observation_keep_time);
        node->get_parameter(
        name_ + "." + source + "." + "expected_update_rate",
        expected_update_rate);
        node->get_parameter(name_ + "." + source + "." + "data_type", data_type);
        node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);
        node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);
        node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);

        if (!(data_type == "PointCloud2" || data_type == "LaserScan")) {
        RCLCPP_FATAL(
            logger_,
            "Only topics that use point cloud2s or laser scans are currently supported");
        throw std::runtime_error(
                "Only topics that use point cloud2s or laser scans are currently supported");
        }


        // get the obstacle range for the sensor
        double obstacle_max_range, obstacle_min_range;
        node->get_parameter(name_ + "." + source + "." + "obstacle_max_range", obstacle_max_range);
        node->get_parameter(name_ + "." + source + "." + "obstacle_min_range", obstacle_min_range);

        // get the raytrace ranges for the sensor
        double raytrace_max_range, raytrace_min_range;
        node->get_parameter(name_ + "." + source + "." + "raytrace_min_range", raytrace_min_range);
        node->get_parameter(name_ + "." + source + "." + "raytrace_max_range", raytrace_max_range);


        RCLCPP_DEBUG(
        logger_,
        "Creating an observation buffer for source %s, topic %s, frame %s",
        source.c_str(), topic.c_str(),
        sensor_frame.c_str());


        // create an observation buffer
        observation_buffers_.push_back(
            std::shared_ptr<ObservationBuffer
            >(
                new ObservationBuffer(
                    node, topic, observation_keep_time, expected_update_rate,
                    min_obstacle_height, max_obstacle_height,
                    obstacle_max_range, obstacle_min_range,
                    raytrace_max_range, raytrace_min_range, *tf_, global_frame_,
                    sensor_frame, tf2::durationFromSec(transform_tolerance))));

        RCLCPP_INFO(
        logger_,
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(),
        global_frame_.c_str(), expected_update_rate, observation_keep_time);

        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
        custom_qos_profile.depth = 50;

        if (data_type == "LaserScan") {
            std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>
                sub(new message_filters::Subscriber<sensor_msgs::msg::LaserScan>(
                    rclcpp_node_, topic, custom_qos_profile));
            sub->unsubscribe();

            std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
                filter(new tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>(
                    *sub, *tf_, global_frame_, 50,rclcpp_node_,
                    tf2::durationFromSec(transform_tolerance)));

            if (inf_is_valid) {
                filter->registerCallback(
                      std::bind(
                        &TemporaryObstacleLayer::laserScanValidInfCallback, this,
                        std::placeholders::_1, observation_buffers_.back()));
            } else {
                filter->registerCallback(
                    std::bind(
                        &TemporaryObstacleLayer::laserScanCallback, this, 
                        std::placeholders::_1, observation_buffers_.back()));
            }

            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);

            observation_notifiers_.back()->setTolerance(rclcpp::Duration::from_seconds(0.05));
        } else {
            std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
                sub(new message_filters::Subscriber<sensor_msgs::msg::PointCloud2>(
                    rclcpp_node_, topic, custom_qos_profile));

            if (inf_is_valid) {
                RCLCPP_WARN(
                logger_,
                "obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
            }

            std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>
                filter(new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(
                    *sub, *tf_, global_frame_, 50, rclcpp_node_,
                    tf2::durationFromSec(transform_tolerance)));
            filter->registerCallback(
                std::bind(
                    &TemporaryObstacleLayer::pointCloud2Callback, this,
                    std::placeholders::_1, observation_buffers_.back()));

            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);
        }

        if (sensor_frame != "") {
            std::vector<std::string> target_frames;
            target_frames.push_back(global_frame_);
            target_frames.push_back(sensor_frame);
            observation_notifiers_.back()->setTargetFrames(target_frames);
        }
    }

    mark_sensors_obstacle_service_ = node->create_service<std_srvs::srv::SetBool>(
    "/mark_sensors_obstacle",
    std::bind(
      &TemporaryObstacleLayer::enableCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void TemporaryObstacleLayer::enableCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    RCLCPP_ERROR(logger_,
        "Mark sensors obstacle server in!!");
    start();
    enb_mark_ = request->data;
    response->success = true; 
    mark_done_ = false;
    if (enb_mark_) {
        response->message = "mark";
        mark_start_time_ = steady_clock_.now();
        
    } else {
        response->message = "unmark";
    }
    RCLCPP_ERROR(logger_,
        "Mark sensors obstacle server out!! enb_mark_ : %d, mark_done_ : %d.", 
        enb_mark_, mark_done_); 
    
}

void TemporaryObstacleLayer::laserScanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer>& buffer)
{
    // project the laser into a point cloud
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = message->header;

    // project the scan into a point cloud
    try {
        projector_.transformLaserScanToPointCloud(
            message->header.frame_id, *message, cloud, *tf_);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
        logger_,
        "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
        global_frame_.c_str(),
        ex.what());
        projector_.projectLaser(*message, cloud);
    } catch (std::runtime_error & ex) {
        RCLCPP_WARN(
        logger_,
        "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
        " Ignore this message. what(): %s",
        ex.what());
        return;
    }


    // buffer the point cloud
    if (enb_mark_) {
        auto during_time = steady_clock_.now() - mark_start_time_;
        auto during_time_seconds = during_time.seconds();
        RCLCPP_WARN(
            logger_,
            "Marking : %.2f s, mark_done_ : %d.", during_time_seconds, mark_done_); 
        // When triggered asynchronously, prevent the addition from being delayed.
        if (during_time_seconds > 1.0 && mark_done_) {
            enb_mark_ = false;
        }
        buffer->lock();
        buffer->bufferCloud(cloud);
        buffer->unlock();
    }

}

void TemporaryObstacleLayer::laserScanValidInfCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr  raw_message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer>& buffer)
{
    // Filter positive infinities ("Inf"s) to max_range.
    float epsilon = 0.0001;  // a tenth of a millimeter
    sensor_msgs::msg::LaserScan message = *raw_message;
    for (size_t i = 0; i < message.ranges.size(); i++) {
        float range = message.ranges[i];
        if (!std::isfinite(range) && range > 0) {
            message.ranges[i] = message.range_max - epsilon;
        }
    }

    // project the laser into a point cloud
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = message.header;

    // project the scan into a point cloud
    try {
        projector_.transformLaserScanToPointCloud(
            message.header.frame_id, message, cloud, *tf_);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
        logger_,
        "High fidelity enabled, but TF returned a transform exception to frame %s: %s",
        global_frame_.c_str(), ex.what());
        projector_.projectLaser(message, cloud);
    } catch (std::runtime_error & ex) {
        RCLCPP_WARN(
        logger_,
        "transformLaserScanToPointCloud error, it seems the message from laser is malformed."
        " Ignore this message. what(): %s",
        ex.what());
        return;
    }

    // buffer the point cloud
    if (enb_mark_) {
        auto during_time = steady_clock_.now() - mark_start_time_;
        auto during_time_seconds = during_time.seconds();
        RCLCPP_WARN(
            logger_,
            "Marking : %.2f s, mark_done_ : %d.", during_time_seconds, mark_done_); 
        // When triggered asynchronously, prevent the addition from being delayed.
        if (during_time_seconds > 1.0 && mark_done_) {
            enb_mark_ = false;
        }
        buffer->lock();
        buffer->bufferCloud(cloud);
        buffer->unlock();
    }
}

void TemporaryObstacleLayer::pointCloud2Callback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer>& buffer)
{

    // buffer the point cloud
    if (enb_mark_) {
        auto during_time = steady_clock_.now() - mark_start_time_;
        auto during_time_seconds = during_time.seconds();
        RCLCPP_WARN(
            logger_,
            "Marking : %.2f s, mark_done_ : %d.", during_time_seconds, mark_done_); 
        // When triggered asynchronously, prevent the addition from being delayed.
        if (during_time_seconds > 1.0 && mark_done_) {
            enb_mark_ = false;
        }
        buffer->lock();
        buffer->bufferCloud(*message);
        buffer->unlock();
    }
}

void TemporaryObstacleLayer::activate()
{
    for (auto & notifier : observation_notifiers_) {
        notifier->clear();
    }

    // if we're stopped we need to re-subscribe to topics
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
        if (observation_subscribers_[i] != NULL)
            observation_subscribers_[i]->subscribe();
    }

    resetBuffersLastUpdated();
}

void TemporaryObstacleLayer::deactivate()
{
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i) {
        if(observation_subscribers_[i] != NULL)
            observation_subscribers_[i]->unsubscribe();
    }
}

void TemporaryObstacleLayer::reset()
{
    // resetMaps();
    // resetBuffersLastUpdated();
    enb_mark_ = false;
    // current_ = false;
    was_reset_ = true;
    pause();
}

void TemporaryObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                  double *min_x, double *min_y, double *max_x, double *max_y)
{
    if (rolling_window_)
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

    if (!enabled_)
        return;

    *min_x = robot_x - getSizeInMetersX() / 2;
    *max_x = robot_x + getSizeInMetersX() / 2;
    *min_y = robot_y - getSizeInMetersY() / 2;
    *max_y = robot_y + getSizeInMetersY() / 2;

    bool current = true;
    current_ = current;

    resetMaps();
    std::vector<Observation> observations;

    for (unsigned int i = 0; i < observation_buffers_.size(); ++i) {
        observation_buffers_[i]->lock();
        observation_buffers_[i]->getObservations(observations);
        observation_buffers_[i]->unlock();
    }

    for (std::vector<Observation>::const_iterator it = observations.begin();
        it != observations.end(); ++it) {
        const Observation& obs = *it;

        const sensor_msgs::msg::PointCloud2 & cloud = *(obs.cloud_);

        // double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

        double sq_obstacle_max_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;
        double sq_obstacle_min_range = obs.obstacle_min_range_ * obs.obstacle_min_range_;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        double px = *iter_x, py = *iter_y, pz = *iter_z;

        // if the obstacle is too high or too far away from the robot we won't add it
        if (pz > max_obstacle_height_) {
            RCLCPP_DEBUG(logger_, "The point is too high");
            continue;
        }

        // compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist =
            (px -
            obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y) +
            (pz - obs.origin_.z) * (pz - obs.origin_.z);

        // if the point is far enough away... we won't consider it
        if (sq_dist >= sq_obstacle_max_range) {
            RCLCPP_DEBUG(logger_, "The point is too far away");
            continue;
        }

        // if the point is too close, do not conisder it
        if (sq_dist < sq_obstacle_min_range) {
            RCLCPP_DEBUG(logger_, "The point is too close");
            continue;
        }

        // now we need to compute the map coordinates for the observation
        unsigned int mx, my;
        if (!worldToMap(px, py, mx, my)) {
            RCLCPP_DEBUG(logger_, "Computing map coords failed");
            continue;
        }

        unsigned int index = getIndex(mx, my);
        costmap_[index] = LETHAL_OBSTACLE;
        }

    }
    updateFootprint(robot_x, robot_y, robot_yaw);
    mark_done_ = true;
}

void TemporaryObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw)
{
    if (!footprint_clearing_enabled_)
        return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
}

void TemporaryObstacleLayer::updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_)
        return;

    // if not current due to reset, set current now after clearing
    // if (!current_ && was_reset_) {
    if (!current_ && was_reset_) {    
        was_reset_ = false;
        current_ = true;
    }

    if (footprint_clearing_enabled_) {
        setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
    }

    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}


void TemporaryObstacleLayer::resetBuffersLastUpdated()
{
    for (unsigned int i = 0; i < observation_buffers_.size(); ++i) {
        if (observation_buffers_[i]) {
        observation_buffers_[i]->resetLastUpdated();
        }
    }
}

void TemporaryObstacleLayer::pause()
{
    RCLCPP_WARN(logger_,"Pause layer!");
    enabled_ = false;
    deactivate();
    resetMaps();
    current_ = true;
}

void TemporaryObstacleLayer::start()
{
    RCLCPP_WARN(logger_,"Start layer!");
    enabled_ = true;
    current_ = true;
    activate();
}

} // namespace nav2_costmap_2d
