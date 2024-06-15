#include "laser_filter.h"

using namespace std;
using namespace std::chrono_literals;

namespace sensor_vel_filters{
LaserScanFilter::LaserScanFilter(
      const nav2_util::LifecycleNode::WeakPtr & node,
      const std::string & filter_name)
    : node_(node),filter_name_(filter_name)
{
    RCLCPP_INFO(logger_, "[%s]: Creating LaserScanFilter", filter_name_.c_str());
}

LaserScanFilter::~LaserScanFilter()
{
    RCLCPP_INFO(logger_, "[%s]: Destroying LaserScanFilter", filter_name_.c_str());
    sub_laserscan_.reset();
    pub_laserscan_filter_.reset();
    sub_odom_.reset();
}

void LaserScanFilter::configure()
{
    auto node = node_.lock();
    if(!node){
        throw std::runtime_error{"failed to lock node"};
    }

    double temp_replacement_value = std::numeric_limits<double>::quiet_NaN();
    replacement_value_ = static_cast<float>(temp_replacement_value);
    vel_ = {0.0,0.0,0.0};
    clock_ = node->get_clock();

    getParameters();
    
    pub_laserscan_filter_ = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_out_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS(); 
    //sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(sub_vel_topic_, pointcloud_qos, std::bind(&LaserScanFilter::velCallback, this, std::placeholders::_1));
    sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>("/odom", sensor_qos, std::bind(&LaserScanFilter::odomvelCallback, this, std::placeholders::_1));
    
    sub_laserscan_ = node->create_subscription<sensor_msgs::msg::LaserScan>(topic_in_, sensor_qos, std::bind(&LaserScanFilter::scanCallback, this, std::placeholders::_1));

}

void LaserScanFilter::activate()
{
    RCLCPP_INFO(logger_, "publish activate");
    pub_laserscan_filter_->on_activate();
}

void LaserScanFilter::deactivate()
{
    pub_laserscan_filter_->on_deactivate();
}

void LaserScanFilter::getParameters()
{
    auto node = node_.lock();
    if(!node){
        throw std::runtime_error{"failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(node, filter_name_ + ".topic_in", rclcpp::ParameterValue("/scan"));
    topic_in_ = node->get_parameter(filter_name_ + ".topic_in").as_string();

    nav2_util::declare_parameter_if_not_declared(node, filter_name_ + ".topic_out", rclcpp::ParameterValue("/scan_filter"));
    topic_out_ = node->get_parameter(filter_name_ + ".topic_out").as_string();

    nav2_util::declare_parameter_if_not_declared(node, filter_name_ + ".omega", rclcpp::ParameterValue(0.0));
    omega_ = node->get_parameter(filter_name_ + ".omega").as_double();

    nav2_util::declare_parameter_if_not_declared(node, filter_name_ + ".truncate_range", rclcpp::ParameterValue(1.0));
    truncate_range_ = node->get_parameter(filter_name_ + ".truncate_range").as_double();

    nav2_util::declare_parameter_if_not_declared(node, filter_name_ + ".diff_mode_min_turning_radius", rclcpp::ParameterValue(0.5));
    diff_mode_min_turning_radius_ = node->get_parameter(filter_name_ + ".diff_mode_min_turning_radius").as_double();
    
    RCLCPP_INFO(logger_,"LaserScanFilter %s parameter----topic_in:                         %s",filter_name_.c_str(),topic_in_.c_str());
    RCLCPP_INFO(logger_,"LaserScanFilter %s parameter----topic_out:                        %s",filter_name_.c_str(),topic_out_.c_str());
    RCLCPP_INFO(logger_,"LaserScanFilter %s parameter----omega:                            %f",filter_name_.c_str(),omega_);
    RCLCPP_INFO(logger_,"LaserScanFilter %s parameter----truncate_range:                   %f",filter_name_.c_str(),truncate_range_);
    RCLCPP_INFO(logger_,"LaserScanFilter %s parameter----diff_mode_min_turning_radius:     %f",filter_name_.c_str(),diff_mode_min_turning_radius_);

}

void LaserScanFilter::publish()
{
    // if(pub_laserscan_filter_->is_activated()){
    //     pub_laserscan_filter_->publish(scan_msg_);
    // }
    //RCLCPP_INFO(logger_, "publish scan_msgs_");
}

// void LaserScanFilter::velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
// {
//     RCLCPP_INFO(this->get_logger(), "vel:(%f,%f)",msg->linear.x,msg->angular.z);
//     vel_ = {msg->linear.x,msg->linear.y,msg->angular.z};
// }

void LaserScanFilter::odomvelCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    //RCLCPP_INFO(logger_, "vel:(%f,%f)",msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    vel_ = {msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.angular.z};
}

#if 0
//according to vel to simu the robot state in future ,then filter the sensordata. 
void LaserScanFilter::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),"range.size = %zu",msg->ranges.size());
    float dist = 0.0;
    calDist(vel_,dist);

    float ratio_dist = dist/1.4;
    float new_upper_threshold = ratio_dist * upper_threshold_max_;
    if(new_upper_threshold <= upper_threshold_min_){
        new_upper_threshold = upper_threshold_min_;
    }
    if(new_upper_threshold >= upper_threshold_max_){
        new_upper_threshold = upper_threshold_max_; 
    }

    float new_lower_threshold = ratio_dist * lower_threshold_;
    if(new_lower_threshold < lower_threshold_){
        new_lower_threshold = lower_threshold_;
    }
    RCLCPP_INFO(this->get_logger(),"threshold = (%f,%f)",new_lower_threshold,new_upper_threshold);
    
    for(size_t i=0; i<msg->ranges.size(); i++)
    {
        if(msg->ranges[i] >= lower_threshold_ && msg->ranges[i] <= new_upper_threshold)
        {
            msg->ranges[i] = msg->ranges[i];
        }
        else if(msg->ranges[i] > new_upper_threshold)
        {
            //msg->ranges[i] = replacement_value_;
            msg->ranges[i] = 1/0.0;
            //msg->ranges[i] = 0.0;
        }
        else
        {
            msg->ranges[i] = 1/0.0;
            //msg->ranges[i] = replacement_value_;
        }
    }
    pub_laserscan_filter_->publish(*msg);

}
#else
void LaserScanFilter::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    double vx = vel_.x;
    double omega = vel_.tw;
    if(omega >= omega_)
    {
        double turning_radius = std::fabs(vx / omega);
        if(turning_radius <= diff_mode_min_turning_radius_) //rotation or like rotation
        {
            //truncate 1m sensor data
            for(size_t i=0; i<msg->ranges.size(); i++)
            {
                if(msg->ranges[i] > truncate_range_ )
                {
                    msg->ranges[i] = 1/0.0;
                }
            }
        }
    }
    scan_msg_ = *msg;
    if(pub_laserscan_filter_->is_activated()){
        //RCLCPP_INFO_THROTTLE(logger_, *(clock_), 2000,"%s publish scan_msgs_",filter_name_.c_str());
        //RCLCPP_INFO(logger_,"%s pub!!!",filter_name_.c_str());
        pub_laserscan_filter_->publish(scan_msg_);
    }
}
#endif

void LaserScanFilter::projectState(const double &dt, Pose &pose, Velocity &velocity)
{
  const double theta = velocity.tw * dt;
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  // p' = p + vel*dt
  // where:
  //   p - initial pose
  //   p' - projected pose
  pose.x = pose.x + velocity.x * dt;
  pose.y = pose.y + velocity.y * dt;
  // Rotate the pose on theta
  pose.theta = pose.theta + theta;

  // vel' = R*vel
  // where:
  //   vel - initial velocity
  //   R - rotation matrix
  //   vel' - rotated velocity
  const double velocity_upd_x = velocity.x * cos_theta - velocity.y * sin_theta;
  const double velocity_upd_y = velocity.x * sin_theta + velocity.y * cos_theta;
  velocity.x = velocity_upd_x;
  velocity.y = velocity_upd_y;
}

void LaserScanFilter::calDist(const Velocity vel, float &dist)
{
    Pose pose = {0.0,0.0,0.0};
    Velocity v = vel;
    double dt = 0.1;
    for(double time = 0.0; time <= simutime_; time += dt)
    {
        projectState(dt,pose,v);
    }
    dist = sqrt(pose.x * pose.x + pose.y * pose.y);
    //RCLCPP_INFO(this->get_logger(),"dist = %f",dist);
}
}

#if 0 
    // this->declare_parameter<std::double_t>("range_max",1.3);
    // this->declare_parameter<std::double_t>("range_min",0.1);
    // this->get_parameter("range_max",range_max_);
    // this->get_parameter("range_min",range_min_);

    // this->declare_parameter<std::double_t>("truncate_range",1.0);
    // this->get_parameter("truncate_range",truncate_range_);

    // this->declare_parameter<std::double_t>("simutime",2.0);
    // this->get_parameter("simutime",simutime_);

    // this->declare_parameter<std::double_t>("upper_threshold_max",1.0);
    // this->declare_parameter<std::double_t>("lower_threshold",0.1);
    // this->declare_parameter<std::double_t>("upper_threshold_min",1.0);
    // //this->declare_parameter<std::double_t>("lower_threshold",0.1);
    // this->get_parameter("upper_threshold_max",upper_threshold_max_);
    // this->get_parameter("upper_threshold_min",upper_threshold_min_);
    // this->get_parameter("lower_threshold",lower_threshold_);
#endif