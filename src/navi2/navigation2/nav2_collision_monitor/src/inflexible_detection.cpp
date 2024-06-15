#include "nav2_collision_monitor/inflexible_detection.hpp"

namespace nav2_collision_monitor
{

InflexibleDetection::InflexibleDetection(const nav2_util::LifecycleNode::WeakPtr & node,
        const std::string & name,const std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : node_(node), name_(name),tf_buffer_(tf_buffer)
{
    RCLCPP_INFO(logger_, "[%s]: Creating InflexibleDetection", name_.c_str());
} 

InflexibleDetection::~InflexibleDetection()
{
    ultras_sub_.reset();
    laser1_sub_.reset();
    lite_ultrasonic_sub_.reset();
    // if(visualize_lidar_cloud_){
    //     lidar_base_tf_pub_.reset();
    // }
}

bool InflexibleDetection::configure()
{
    RCLCPP_INFO(logger_, "[%s]: Configuring InflexibleDetection", name_.c_str());
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }   
    clock_ = node->get_clock();
    //std::string ultras_sub_topic_; 
    if (!getParameters()) {
        return false;
    }
    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();  // set to default

    if(visualize_lidar_cloud_){
        lidar_base_tf_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            "lidar_base_tf",10);
    }

    lite_ultrasonic_sub_ = node->create_subscription<cm_msgs::msg::SpsUltrasonic>(
        "/sps_ultrasonic", sensor_qos,
        std::bind(&InflexibleDetection::LiteUltrasonicCallback, this, std::placeholders::_1));

    ultras_sub_ = node->create_subscription<sensor_msgs::msg::ChannelFloat32>(
        ultras_sub_topic_, sensor_qos,
        std::bind(&InflexibleDetection::RangesMsgCallback, this, std::placeholders::_1));

    distance_remaining_sub_ = node->create_subscription<std_msgs::msg::Float32>(
        "/distance_remaining", rclcpp::SystemDefaultsQoS(),
        std::bind(&InflexibleDetection::DistRemainMsgCallback, this, std::placeholders::_1));

    laser1_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        laser1_topic_, sensor_qos,
        std::bind(&InflexibleDetection::LaserCallback, this, std::placeholders::_1));
    return true;
}

bool InflexibleDetection::getParameters()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }      
    try {
        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".distance_remaining_threshold", rclcpp::ParameterValue(0.3));
        distance_remaining_threshold_ = node->get_parameter(name_ + ".distance_remaining_threshold").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".ultras_detection_open", rclcpp::ParameterValue(true));
        ultras_detection_open_ = node->get_parameter(name_ + ".ultras_detection_open").as_bool();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".ultras_sub_topic", rclcpp::ParameterValue("all_ultra_data"));
        ultras_sub_topic_ = node->get_parameter(name_ + ".ultras_sub_topic").as_string();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".laser1_topic", rclcpp::ParameterValue("scan_rslidar_3d"));
        laser1_topic_ = node->get_parameter(name_ + ".laser1_topic").as_string();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".lidar_frame", rclcpp::ParameterValue("rslidar"));
        lidar_frame_ = node->get_parameter(name_ + ".lidar_frame").as_string();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".base_frame", rclcpp::ParameterValue("base_link"));
        base_frame_ = node->get_parameter(name_ + ".base_frame").as_string();
        
        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".emergency_stop_distance", rclcpp::ParameterValue(0.1));
        emergency_stop_distance_ = node->get_parameter(name_ + ".emergency_stop_distance").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".omega", rclcpp::ParameterValue(0.1));
        omega_ = node->get_parameter(name_ + ".omega").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".forward_emergency_stop_dist", rclcpp::ParameterValue(0.05));
        forward_emergency_stop_dist_ = node->get_parameter(name_ + ".forward_emergency_stop_dist").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".side_forward_emergency_stop_dist", rclcpp::ParameterValue(0.22));
        side_forward_emergency_stop_dist_ = node->get_parameter(name_ + ".side_forward_emergency_stop_dist").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".rotate_forward_emergency_stop_dist", rclcpp::ParameterValue(0.22));
        rotate_forward_emergency_stop_dist_ = node->get_parameter(name_ + ".rotate_forward_emergency_stop_dist").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".scan_forward_dist", rclcpp::ParameterValue(0.15));
        scan_forward_dist_ = node->get_parameter(name_ + ".scan_forward_dist").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".scan_side_dist", rclcpp::ParameterValue(0.25));
        scan_side_dist_ = node->get_parameter(name_ + ".scan_side_dist").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".turn_emergency_stop_dist", rclcpp::ParameterValue(0.1));
        turn_emergency_stop_dist_ = node->get_parameter(name_ + ".turn_emergency_stop_dist").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".obstacle_detect_distance", rclcpp::ParameterValue(5.0));
        obstacle_detect_distance_ = node->get_parameter(name_ + ".obstacle_detect_distance").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".min_range_threshold", rclcpp::ParameterValue(0.03));
        min_range_threshold_ = node->get_parameter(name_ + ".min_range_threshold").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".range_sensor_emergency_stop_dist", rclcpp::ParameterValue(0.1));
        range_sensor_emergency_stop_dist_ = node->get_parameter(name_ + ".range_sensor_emergency_stop_dist").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".diff_mode_min_turning_radius", rclcpp::ParameterValue(0.6));
        diff_mode_min_turning_radius_ = node->get_parameter(name_ + ".diff_mode_min_turning_radius").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".straight_threshold", rclcpp::ParameterValue(2.0));
        straight_threshold_ = node->get_parameter(name_ + ".straight_threshold").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".check_window_x", rclcpp::ParameterValue(1.0));
        check_window_x_ = node->get_parameter(name_ + ".check_window_x").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".check_window_y", rclcpp::ParameterValue(1.0));
        check_window_y_ = node->get_parameter(name_ + ".check_window_y").as_double();

        nav2_util::declare_parameter_if_not_declared(
        node, name_ + ".visualize_lidar_cloud", rclcpp::ParameterValue(true));
        visualize_lidar_cloud_ = node->get_parameter(name_ + ".visualize_lidar_cloud").as_bool();

        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----distance_remaining_threshold:     %f",distance_remaining_threshold_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----ultras_detection_open:            %d",ultras_detection_open_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----ultras_sub_topic:                 %s",ultras_sub_topic_.c_str());
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----laser1_topic:                     %s",laser1_topic_.c_str());
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----lidar_frame:                      %s",lidar_frame_.c_str());
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----base_frame:                       %s",base_frame_.c_str());
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----emergency_stop_distance:          %f",emergency_stop_distance_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----omega:                            %f",omega_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----forward_emergency_stop_dist:      %f",forward_emergency_stop_dist_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----side_forward_emergency_stop_dist: %f",side_forward_emergency_stop_dist_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----rotate_forward_emergency_stop_dist: %f",rotate_forward_emergency_stop_dist_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----scan_forward_dist:                %f",scan_forward_dist_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----scan_side_dist:                   %f",scan_side_dist_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parametee----turn_emergency_stop_dist:         %f",turn_emergency_stop_dist_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----obstacle_detect_distance:         %f",obstacle_detect_distance_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----min_range_threshold:              %f",min_range_threshold_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----range_sensor_emergency_stop_dist: %f",range_sensor_emergency_stop_dist_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----diff_mode_min_turning_radius:     %f",diff_mode_min_turning_radius_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----straight_threshold_:              %f",straight_threshold_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----check_window_x:                   %f",check_window_x_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----check_window_y:                   %f",check_window_y_);
        RCLCPP_INFO(logger_,"collision_monitor inflexible_detection parameter----visualize_lidar_cloud:            %d",visualize_lidar_cloud_);

    } catch (const std::exception & ex) {
        RCLCPP_ERROR(
        logger_,
        "[%s]: Error while getting parameters: %s",
        name_.c_str(), ex.what());
        return false;
    }    
    return true;
}


void InflexibleDetection::DistRemainMsgCallback(std_msgs::msg::Float32::ConstSharedPtr msg)
{
    distance_remaining_ = msg->data;
}
// float32[] distance
// bool[]    isValid
void InflexibleDetection::LiteUltrasonicCallback(cm_msgs::msg::SpsUltrasonic::ConstSharedPtr msg)
{
    lite_ultrasonic_datas_ = *msg;
}

void InflexibleDetection::LaserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    //scan_datas  
    //frame_id: "laser"  angle_min: -2.094 angle_max: 2.094   angle_increment: 0.00436787307262 
    //time_increment: 4.42608470621e-05  scan_time: 0.063691355288 
    //range_min: 0.15  range_max: 16.0
    scan_datas_ = *msg;
    #if 0
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }
    lidar_cloud_.header.stamp = node->get_clock()->now(); 
    lidar_cloud_.header.frame_id = base_frame_;

    try{
        projector_.transformLaserScanToPointCloud(
            base_frame_,*msg,lidar_cloud_,*tf_buffer_);
    } catch(tf2::TransformException &ex){
        RCLCPP_WARN(logger_,"Transform failed during transform lidar scan to cloud, %s.",ex.what());
        projector_.projectLaser(*msg,lidar_cloud_);
    }
    if(visualize_lidar_cloud_){
        lidar_base_tf_pub_->publish(lidar_cloud_);
    }
    #endif
    
}

void InflexibleDetection::RangesMsgCallback(sensor_msgs::msg::ChannelFloat32::ConstSharedPtr ranges_message)
{
    
    ultra_datas_ =*ranges_message;
}

void InflexibleDetection::activate()
{
  if (visualize_lidar_cloud_) {
    lidar_base_tf_pub_->on_activate();
  }
}

void InflexibleDetection::deactivate()
{
  if (visualize_lidar_cloud_) {
    lidar_base_tf_pub_->on_deactivate();
  }
}

bool InflexibleDetection::CheckSurroundSafety()
{
    if (check_window_x_ == 0.0 || check_window_y_ == 0.0) {
        return true;
    }  
    return true;

    
    // int fatal_point = 0;
    
    // for(int i = 0 ; i < lidar_cloud_.data.size() ; i++)
    // {
    //     if(std::fabs(lidar_cloud_.data[i].))
    //     lidar_cloud_.data[i]
    // }
}

bool InflexibleDetection::CheckRangeSafety(const geometry_msgs::msg::Twist& cmd,const std::string &car_type)
{
    // for test
    // if(cmd.angular.z == 1.0){
    //     return false;
    // }
    // if(cmd.angular.z == 2.0){
    //     return true;
    // }
    if(car_type != "patrol" && car_type != "ginger_lite" && car_type != "ginger" && car_type != "ginger2.0"){
        return true;
    }

    if(car_type == "patrol"){
        if (!ultras_detection_open_) {
            return true;
        }

        if (ultra_datas_.values.size() != 10) {
            RCLCPP_WARN(logger_,"The range sensors date lost!!!!  ultra_data.size = %zu",ultra_datas_.values.size());
            return true;
        }

        ControlState control_state;

        double vx = cmd.linear.x;
        double omega = cmd.angular.z;

        #if 0 
        if (omega != 0.) {
            double turning_radius = std::fabs(vx / omega);
            if (turning_radius <= diff_mode_min_turning_radius_ )
            {
                control_state = ROTATION;
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> ROTATION", vx, omega);
            }
            else if (fabs(omega) < 0.1)
            {
                control_state = STRAIGHT;
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> STRAIGHT", vx, omega);
            }
            else if (omega >= 0.1)
            {
                control_state = LEFTTURN;
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> LEFTTURN", vx, omega);
            }
            else{
                control_state = RIGHTTURN;
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> RIGHTTURN", vx, omega);
            }
        } else {
            control_state = STRAIGHT;
            RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> STRAIGHT", vx, omega);
        }

        switch (control_state) {
        case STRAIGHT : {
            if ((ultra_datas_.values.at(0) > min_range_threshold_ && ultra_datas_.values.at(0) < forward_emergency_stop_dist_) 
                || (ultra_datas_.values.at(2) > min_range_threshold_ && ultra_datas_.values.at(2) < forward_emergency_stop_dist_) 
                || (ultra_datas_.values.at(3) > min_range_threshold_ && ultra_datas_.values.at(3) < forward_emergency_stop_dist_) 
                || (ultra_datas_.values.at(7) > min_range_threshold_ && ultra_datas_.values.at(7) < forward_emergency_stop_dist_)  
                || (ultra_datas_.values.at(8) > min_range_threshold_ && ultra_datas_.values.at(8) < forward_emergency_stop_dist_) ) 
            {
                RCLCPP_WARN(logger_,"Broadsafe cloud be collison!!!! cmd(%f, %f)---> STRAIGHT", vx, omega);
                return false;
            }
            break;
        }
        case ROTATION: {
            for(size_t i = 0; i < ultra_datas_.values.size(); i++)
            {
                if(ultra_datas_.values.at(i) > min_range_threshold_ && ultra_datas_.values.at(i) < turn_emergency_stop_dist_)
                {
                    RCLCPP_WARN(logger_,"Rotation cloud be collison!!!! the %zu data collision! cmd(%f, %f)---> ROTATION", i,vx, omega);
                    return false;
                }
            }
            break;
        }
        case RIGHTTURN : {
            if ((ultra_datas_.values.at(1) > min_range_threshold_ && ultra_datas_.values.at(1) < turn_emergency_stop_dist_) 
                || (ultra_datas_.values.at(2) > min_range_threshold_ && ultra_datas_.values.at(2) < turn_emergency_stop_dist_)
                || (ultra_datas_.values.at(3) > min_range_threshold_ && ultra_datas_.values.at(3) < turn_emergency_stop_dist_)
                || (ultra_datas_.values.at(6) > min_range_threshold_ && ultra_datas_.values.at(6) < turn_emergency_stop_dist_)) 
            {
                RCLCPP_WARN(logger_,"RIGHTTURN cloud be collison!!!! cmd(%f, %f)---> STRAIGHT", vx, omega);
                return false;
            }
            break;
        }
        case LEFTTURN : {
            if ((ultra_datas_.values.at(4) > min_range_threshold_ && ultra_datas_.values.at(4) < turn_emergency_stop_dist_) 
                || (ultra_datas_.values.at(9) > min_range_threshold_ && ultra_datas_.values.at(9) < turn_emergency_stop_dist_)
                || (ultra_datas_.values.at(7) > min_range_threshold_ && ultra_datas_.values.at(7) < turn_emergency_stop_dist_)
                || (ultra_datas_.values.at(8) > min_range_threshold_ && ultra_datas_.values.at(8) < turn_emergency_stop_dist_)) 
            {
                RCLCPP_WARN(logger_,"LEFTTURN cloud be collison!!!! cmd(%f, %f)---> STRAIGHT", vx, omega);
                return false;
            }
            break;
        }
        default :
            break;
        }
        #else
        if (omega != 0.) {
            double turning_radius = std::fabs(vx / omega);
            if (turning_radius < diff_mode_min_turning_radius_ && omega > 0.) {
                control_state = LEFTTURN;
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> LEFTTURN", vx, omega);
            } else if (turning_radius < diff_mode_min_turning_radius_) {
                control_state = RIGHTTURN;
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> RIGHTTURN", vx, omega);
            } else {
                control_state = STRAIGHT;
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> STRAIGHT", vx, omega);
            }
        } else {
            control_state = STRAIGHT;
            RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> STRAIGHT", vx, omega);
        }

        switch (control_state) {
        case STRAIGHT : {
            if ((ultra_datas_.values.at(0) > min_range_threshold_ && ultra_datas_.values.at(0) < forward_emergency_stop_dist_) ||
                (ultra_datas_.values.at(2) > min_range_threshold_ && ultra_datas_.values.at(2) < forward_emergency_stop_dist_) ||
                (ultra_datas_.values.at(3) > min_range_threshold_ && ultra_datas_.values.at(3) < forward_emergency_stop_dist_) ||
                (ultra_datas_.values.at(7) > min_range_threshold_ && ultra_datas_.values.at(7) < forward_emergency_stop_dist_)  ||
                (ultra_datas_.values.at(8) > min_range_threshold_ && ultra_datas_.values.at(8) < forward_emergency_stop_dist_) ) {
                RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"Broadsafe cloud be collison!!!! cmd(%f, %f)---> STRAIGHT, data(0,2,3,7,8) = (%f,%f,%f,%f,%f)", 
                            vx, omega, ultra_datas_.values.at(0),ultra_datas_.values.at(2),
                            ultra_datas_.values.at(3),ultra_datas_.values.at(7),ultra_datas_.values.at(8));
                return false;
            }
            break;
        }
        case LEFTTURN : {
            if ((ultra_datas_.values.at(4) > min_range_threshold_ && ultra_datas_.values.at(4) < turn_emergency_stop_dist_) ||
                (ultra_datas_.values.at(9) > min_range_threshold_ && ultra_datas_.values.at(9) < turn_emergency_stop_dist_) ||
                (ultra_datas_.values.at(3) > min_range_threshold_ && ultra_datas_.values.at(3) < turn_emergency_stop_dist_) ||
                (ultra_datas_.values.at(8) > min_range_threshold_ && ultra_datas_.values.at(8) < turn_emergency_stop_dist_)) {
                RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"Left rurn cloud be collison!!!! cmd(%f, %f)---> LEFTTURN, data(3,4,8,9) = (%f,%f,%f,%f)",vx, omega,
                            ultra_datas_.values.at(3),ultra_datas_.values.at(4),
                            ultra_datas_.values.at(8),ultra_datas_.values.at(9));
                return false;
            }
            break;
        }
        case RIGHTTURN : {
            if ((ultra_datas_.values.at(1) > min_range_threshold_ && ultra_datas_.values.at(1) < turn_emergency_stop_dist_) ||
                (ultra_datas_.values.at(6) > min_range_threshold_ && ultra_datas_.values.at(6) < turn_emergency_stop_dist_) ||
                (ultra_datas_.values.at(2) > min_range_threshold_ && ultra_datas_.values.at(2) < turn_emergency_stop_dist_) ||
                (ultra_datas_.values.at(7) > min_range_threshold_ && ultra_datas_.values.at(7) < turn_emergency_stop_dist_)) {
                RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"Right turn cloud be collison!!!! cmd(%f, %f)---> RIGHTTURN, data(1,2,6,7) = (%f,%f,%f,%f)", vx, omega,
                            ultra_datas_.values.at(1),ultra_datas_.values.at(2),
                            ultra_datas_.values.at(6),ultra_datas_.values.at(7));
                return false;
            }
            break;
        }
        default :
            break;
        }
        #endif 
    }
    // death_rigion:13.7cm  <13.7cm  : 13.7   >1.17m:1.17  wholy_cover is 1.17m
    // FOV:hor 30
    // hz: 10
    //sensor_numbers:0,1,2,3,4    left_front,middle,right_front,right_behind,left_behind(direct:clockwise）
    //right turn:w<0
    if(car_type == "ginger_lite"){
        if(distance_remaining_ < distance_remaining_threshold_){
            RCLCPP_WARN_ONCE(logger_,"close ultra emergency stop function, distance_remaining_threshold: %f",distance_remaining_threshold_);
            return true;
        }

        if(lite_ultrasonic_datas_.distance.size() != 5){
            RCLCPP_WARN(logger_,"The lite ultra sensors date lost!!!!lite_ultrasonic_datas_.size = %zu",lite_ultrasonic_datas_.distance.size());
            return true;
        }

        ControlState control_state;
        
        double vx = cmd.linear.x;
        double omega = cmd.angular.z;
        // RCLCPP_WARN(logger_,"cmd(%f, %f)---> (%f,%f,%f)", 
        //         vx,omega,lite_ultrasonic_datas_.distance[0], lite_ultrasonic_datas_.distance[1],lite_ultrasonic_datas_.distance[2]
        // );

        if(fabs(omega) > omega_){
            double turning_radius = std::fabs(vx / omega); 
            if(turning_radius <= diff_mode_min_turning_radius_ ){
                control_state = ROTATION;
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> ROTATION", vx, omega);
                //return true;
            }else if(turning_radius >= straight_threshold_){  //add a param_threshlod
                control_state = STRAIGHT;
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> STRAIGHT", vx, omega);
            }else if(omega < 0){
                control_state = STRAIGHT; //STRAIGHT  RIGHTTURN
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> RIGHTTURN", vx, omega);
            }else{ //omega>0
                control_state = STRAIGHT;  //STRAIGHT  LEFTTURN
                RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> LEFTTURN", vx, omega);
            }
        }else{
            control_state = STRAIGHT;
            RCLCPP_DEBUG(logger_,"cmd(%f, %f)---> STRAIGHT", vx, omega);
        }

        switch (control_state)
        {
        case STRAIGHT:
            #if 0  
            if((lite_ultrasonic_datas_.distance[1] < forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[1] & 0x1))
            && (lite_ultrasonic_datas_.distance[0] < side_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[0] & 0x1)) 
            && (lite_ultrasonic_datas_.distance[2] < side_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[2] & 0x1))
            ){
                RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"Broadsafe cloud be collison!!!! cmd(%f, %f)---> STRAIGHT, data(0,1,2) = (%f,%f,%f),valid(0,1,2) = (%d,%d,%d)",
                            vx,omega,lite_ultrasonic_datas_.distance[0], lite_ultrasonic_datas_.distance[1],lite_ultrasonic_datas_.distance[2],
                            lite_ultrasonic_datas_.is_valid[0]& 0x1,lite_ultrasonic_datas_.is_valid[1]& 0x1,lite_ultrasonic_datas_.is_valid[2]& 0x1);
                return false;
            }
            #endif
            #if 1
            if((lite_ultrasonic_datas_.distance[1] < forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[1] & 0x1))
            && (lite_ultrasonic_datas_.distance[0] < side_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[0] & 0x1)) 
            && (lite_ultrasonic_datas_.distance[2] < side_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[2] & 0x1))
            ){
                  // && !scan[i],1： scan can see the obs;  donot stop;   0: scan cannot see the obs; should stop
                bool scan[3] = {0,0,0};
                if(scan_datas_.ranges.size() < 960){
                    scan[0] = 0;
                    scan[1] = 0;
                    scan[2] = 0;
                    return false;
                }
                // sonar detect leftside has obs;
                if( (lite_ultrasonic_datas_.distance[0] < (side_forward_emergency_stop_dist_) && (lite_ultrasonic_datas_.is_valid[0] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 580; i < 830; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[0] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[0] + scan_side_dist_)){
                            num++;
                            if(num >=10){
                                scan[0] = 1;
                                break;
                            }
                        }
                    }
                }
                if( (lite_ultrasonic_datas_.distance[1] < forward_emergency_stop_dist_  && (lite_ultrasonic_datas_.is_valid[1] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 350; i < 600; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[1] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[1] + scan_forward_dist_)){
                            num++;
                            if(num >=10){
                                scan[1] = 1;
                                break;
                            }
                        }
                    }
                }
                if( (lite_ultrasonic_datas_.distance[2] < (side_forward_emergency_stop_dist_) && (lite_ultrasonic_datas_.is_valid[2] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 120; i < 370; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[2] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[2] + scan_side_dist_)){
                            num++;
                            if(num >=10){
                                scan[2] = 1;
                                break;
                            }
                        }
                    }
                }            
                
                if((lite_ultrasonic_datas_.distance[1] < forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[1] & 0x1) && !scan[1])
                && (lite_ultrasonic_datas_.distance[0] < (side_forward_emergency_stop_dist_ ) && (lite_ultrasonic_datas_.is_valid[0] & 0x1) && !scan[0]) 
                && (lite_ultrasonic_datas_.distance[2] < (side_forward_emergency_stop_dist_ ) && (lite_ultrasonic_datas_.is_valid[2] & 0x1) && !scan[2])
                ){
                    //RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"scan cannot see obs! stop it! scan{0,1,2} = (%d,%d,%d)",scan[0],scan[1],scan[2]);
                    RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"Broadsafe cloud be collison!!!! cmd(%f, %f)---> STRAIGHT, data(0,1,2) = (%f,%f,%f),valid(0,1,2) = (%d,%d,%d)",
                                vx,omega,lite_ultrasonic_datas_.distance[0], lite_ultrasonic_datas_.distance[1],lite_ultrasonic_datas_.distance[2],
                                lite_ultrasonic_datas_.is_valid[0]& 0x1,lite_ultrasonic_datas_.is_valid[1]& 0x1,lite_ultrasonic_datas_.is_valid[2]& 0x1);
                    RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"broadsafe : scan can not see obs! donnot stop it! scan{0,1,2} = (%d,%d,%d)",scan[0],scan[1],scan[2]);                    
                    return false;
                }else{

                    return true;
                }                  
            }
            #endif
            break;

        case LEFTTURN:
            if((lite_ultrasonic_datas_.distance[1] < forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[1] & 0x1))
            && (lite_ultrasonic_datas_.distance[0] < side_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[0] & 0x1)) 
            //|| (lite_ultrasonic_datas_.distance[2] < side_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[2] & 0x1))
            ){
                RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"LEFTTURN cloud be collison!!!! cmd(%f, %f)---> LEFTTURN, data(0,1,2) = (%f,%f,%f),valid(0,1,2) = (%d,%d,%d)",
                            vx,omega,lite_ultrasonic_datas_.distance[0], lite_ultrasonic_datas_.distance[1],lite_ultrasonic_datas_.distance[2],
                            lite_ultrasonic_datas_.is_valid[0]& 0x1,lite_ultrasonic_datas_.is_valid[1]& 0x1,lite_ultrasonic_datas_.is_valid[2]& 0x1);
                return false;
                #if 0// && !scan[i],1： scan can see the obs;  donot stop;   0: scan cannot see the obs; should stop
                bool scan[3] = {0,0,0};
                if(scan_datas_.ranges.size() < 960){
                    scan[0] = 0;
                    scan[1] = 0;
                    scan[2] = 0;
                    return false;
                }
                // sonar detect leftside has obs;
                if( (lite_ultrasonic_datas_.distance[0] < (side_forward_emergency_stop_dist_) && (lite_ultrasonic_datas_.is_valid[0] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 580; i < 830; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[0] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[0] + scan_side_dist_)){
                            num++;
                            if(num >=10){
                                scan[0] = 1;
                                break;
                            }
                        }
                    }
                }
                if( (lite_ultrasonic_datas_.distance[1] < forward_emergency_stop_dist_  && (lite_ultrasonic_datas_.is_valid[1] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 350; i < 600; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[1] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[1] + scan_forward_dist_)){
                            num++;
                            if(num >=10){
                                scan[1] = 1;
                                break;
                            }
                        }
                    }
                }          
                
                if((lite_ultrasonic_datas_.distance[1] < forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[1] & 0x1) && !scan[1])
                && (lite_ultrasonic_datas_.distance[0] < (side_forward_emergency_stop_dist_ ) && (lite_ultrasonic_datas_.is_valid[0] & 0x1) && !scan[0]) 
                ){
                    RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"LEFTTURN cloud be collison!!!! cmd(%f, %f)---> LEFTTURN, data(0,1,2) = (%f,%f,%f),valid(0,1,2) = (%d,%d,%d)",
                                vx,omega,lite_ultrasonic_datas_.distance[0], lite_ultrasonic_datas_.distance[1],lite_ultrasonic_datas_.distance[2],
                                lite_ultrasonic_datas_.is_valid[0]& 0x1,lite_ultrasonic_datas_.is_valid[1]& 0x1,lite_ultrasonic_datas_.is_valid[2]& 0x1);
                    RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"scan cannot see obs! stop it! scan{0,1,2} = (%d,%d,%d)",scan[0],scan[1],scan[2]);
                    return false;
                }else{
                    //RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"scan can see obs! donnot stop it! scan{0,1,2} = (%d,%d,%d)",scan[0],scan[1],scan[2]);
                    return true;
                }  
                #endif 
            }
            break;

        case RIGHTTURN:
            if((lite_ultrasonic_datas_.distance[2] < forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[2] & 0x1))
            && (lite_ultrasonic_datas_.distance[0] < side_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[0] & 0x1)) 
            //|| (lite_ultrasonic_datas_.distance[1] < side_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[1] & 0x1))
            ){
                RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"RIGHTTURN cloud be collison!!!! cmd(%f, %f)---> RIGHTTURN, data(0,1,2) = (%f,%f,%f),valid(0,1,2) = (%d,%d,%d)",
                            vx,omega,lite_ultrasonic_datas_.distance[0], lite_ultrasonic_datas_.distance[1],lite_ultrasonic_datas_.distance[2],
                            lite_ultrasonic_datas_.is_valid[0]& 0x1,lite_ultrasonic_datas_.is_valid[1]& 0x1,lite_ultrasonic_datas_.is_valid[2]& 0x1);
                return false;
                #if 0 // && !scan[i],1： scan can see the obs;  donot stop;   0: scan cannot see the obs; should stop
                bool scan[3] = {0,0,0};
                if(scan_datas_.ranges.size() < 960){
                    scan[0] = 0;
                    scan[1] = 0;
                    scan[2] = 0;
                    return false;
                }
                // sonar detect leftside has obs;
                if( (lite_ultrasonic_datas_.distance[1] < forward_emergency_stop_dist_  && (lite_ultrasonic_datas_.is_valid[1] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 350; i < 600; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[1] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[1] + scan_forward_dist_)){
                            num++;
                            if(num >=10){
                                scan[1] = 1;
                                break;
                            }
                        }
                    }
                }
                if( (lite_ultrasonic_datas_.distance[2] < (side_forward_emergency_stop_dist_) && (lite_ultrasonic_datas_.is_valid[2] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 130; i < 350; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[2] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[2] + scan_side_dist_)){
                            num++;
                            if(num >=10){
                                scan[2] = 1;
                                break;
                            }
                        }
                    }
                }            
                
                if((lite_ultrasonic_datas_.distance[1] < forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[1] & 0x1) && !scan[1])
                //|| (lite_ultrasonic_datas_.distance[0] < (rotate_forward_emergency_stop_dist_ + 0.05) && (lite_ultrasonic_datas_.is_valid[0] & 0x1) && !scan[0]) 
                && (lite_ultrasonic_datas_.distance[2] < (side_forward_emergency_stop_dist_) && (lite_ultrasonic_datas_.is_valid[2] & 0x1) && !scan[2])
                ){
                    RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"RIGHTTURN cloud be collison!!!! cmd(%f, %f)---> RIGHTTURN, data(0,1,2) = (%f,%f,%f),valid(0,1,2) = (%d,%d,%d)",
                                vx,omega,lite_ultrasonic_datas_.distance[0], lite_ultrasonic_datas_.distance[1],lite_ultrasonic_datas_.distance[2],
                                lite_ultrasonic_datas_.is_valid[0]& 0x1,lite_ultrasonic_datas_.is_valid[1]& 0x1,lite_ultrasonic_datas_.is_valid[2]& 0x1);
                    RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"scan cannot see obs! stop it! scan{0,1,2} = (%d,%d,%d)",scan[0],scan[1],scan[2]);
                    return false;
                }else{
                    //RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"scan can see obs! donnot stop it! scan{0,1,2} = (%d,%d,%d)",scan[0],scan[1],scan[2]);
                    return true;
                }  
                #endif         
            } 
            break;

        case ROTATION:
            if((lite_ultrasonic_datas_.distance[1] < rotate_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[1] & 0x1))
            || ((lite_ultrasonic_datas_.distance[0] < rotate_forward_emergency_stop_dist_ ) && (lite_ultrasonic_datas_.is_valid[0] & 0x1)) 
            || ((lite_ultrasonic_datas_.distance[2] < rotate_forward_emergency_stop_dist_ ) && (lite_ultrasonic_datas_.is_valid[2] & 0x1))
            ){
                RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"ROTATION cloud be collison!!!! cmd(%f, %f)---> ROTATION, data(0,1,2) = (%f,%f,%f),valid(0,1,2) = (%d,%d,%d)",
                            vx,omega,lite_ultrasonic_datas_.distance[0], lite_ultrasonic_datas_.distance[1],lite_ultrasonic_datas_.distance[2],
                            lite_ultrasonic_datas_.is_valid[0]& 0x1,lite_ultrasonic_datas_.is_valid[1]& 0x1,lite_ultrasonic_datas_.is_valid[2]& 0x1);
                //return false;
                #if 1  // && !scan[i],1： scan can see the obs;  donot stop;   0: scan cannot see the obs; should stop
                bool scan[3] = {0,0,0};
                if(scan_datas_.ranges.size() < 960){
                    scan[0] = 0;
                    scan[1] = 0;
                    scan[2] = 0;
                    return false;
                }
                // sonar detect leftside has obs;
                if( (lite_ultrasonic_datas_.distance[0] < (rotate_forward_emergency_stop_dist_ ) && (lite_ultrasonic_datas_.is_valid[0] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 580; i < 830; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[0] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[0] + scan_side_dist_)){
                            num++;
                            if(num >=10){
                                scan[0] = 1;
                                break;
                            }
                        }
                    }
                }
                if( (lite_ultrasonic_datas_.distance[1] < rotate_forward_emergency_stop_dist_  && (lite_ultrasonic_datas_.is_valid[1] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 350; i < 600; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[1] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[1] + scan_forward_dist_)){
                            num++;
                            if(num >=10){
                                scan[1] = 1;
                                break;
                            }
                        }
                    }
                }
                if( (lite_ultrasonic_datas_.distance[2] < (rotate_forward_emergency_stop_dist_ ) && (lite_ultrasonic_datas_.is_valid[2] & 0x1)) )
                {
                    int num = 0;
                    for(auto i = 120; i < 370; i++){
                        if(scan_datas_.ranges[i] > (lite_ultrasonic_datas_.distance[2] + 0.00) && (scan_datas_.ranges[i] < lite_ultrasonic_datas_.distance[2] + scan_side_dist_)){
                            num++;
                            if(num >=10){
                                scan[2] = 1;
                                break;
                            }
                        }
                    }
                }            
                
                if((lite_ultrasonic_datas_.distance[1] < rotate_forward_emergency_stop_dist_ && (lite_ultrasonic_datas_.is_valid[1] & 0x1) && !scan[1])
                || (lite_ultrasonic_datas_.distance[0] < (rotate_forward_emergency_stop_dist_ ) && (lite_ultrasonic_datas_.is_valid[0] & 0x1) && !scan[0]) 
                || (lite_ultrasonic_datas_.distance[2] < (rotate_forward_emergency_stop_dist_ ) && (lite_ultrasonic_datas_.is_valid[2] & 0x1) && !scan[2])
                ){
                    RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"scan cannot see obs! stop it! scan{0,1,2} = (%d,%d,%d)",scan[0],scan[1],scan[2]);
                    return false;
                }else{
                    RCLCPP_WARN_THROTTLE(logger_,*(clock_),500,"scan can see obs! donnot stop it! scan{0,1,2} = (%d,%d,%d)",scan[0],scan[1],scan[2]);
                    return true;
                }  
                #endif         
            }
            break;

        default:
            break;
        }

    }

    if(car_type == "ginger2.0"){
        return true;
    }

    if(car_type == "ginger"){
        return true;
    }




    return true;
}
} // namespace nav2_collision_monitor
