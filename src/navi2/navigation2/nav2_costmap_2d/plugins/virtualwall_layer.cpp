/*********************************************************************
 * Author: Jate Xiong
 *********************************************************************/
#include "nav2_costmap_2d/virtualwall_layer.hpp"

#include <algorithm>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::VirtualWallLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;
//#define ABS(x) ((x)>=0?(x):(-(x)))
#define GET_SIGN(x) ((x)>=0?(1):(-1))

namespace nav2_costmap_2d
{

VirtualWallLayer::VirtualWallLayer()
    : virtualwalls_buffer_(nullptr)
{
    virtualwalls_.virtualwalls.resize(0);    
}

VirtualWallLayer::~VirtualWallLayer()
{
}

void VirtualWallLayer::onInitialize()
{
    global_frame_ = layered_costmap_->getGlobalFrameID();

    getParameters();

    rclcpp::QoS map_qos(10);  // initialize to default
    if (virtualwall_subscribe_transient_local_) {
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1);
    }
    current_ = true;

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }
    // auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    // we'll subscribe to the latched topic that the map server uses
    RCLCPP_WARN(logger_, "Requesting the virtualwalls...");
    virtualwall_sub_ = node->create_subscription<cm_msgs::msg::Virtualwalls>(
        virtualwall_topic_, map_qos,
        std::bind(&VirtualWallLayer::incomingVirtualwalls, this, std::placeholders::_1));

    RCLCPP_WARN(
        logger_, "Subscribing to the virtualwall topic (%s) with %s durability",
        virtualwall_topic_.c_str(),
        virtualwall_subscribe_transient_local_ ? "transient local" : "volatile");            
   
}

void VirtualWallLayer::activate()
{
    // onInitialize();
}

void VirtualWallLayer::deactivate()
{
    // virtualwall_sub_.shutdown();
    dyn_params_handler_.reset();
}

void VirtualWallLayer::reset()
{
    has_updated_data_ = true;
    // current_ = false;
    // deactivate();
    // activate();
}

void VirtualWallLayer::getParameters()
{
    double temp_tf_tol = 0.0; 
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("virtualwall_topic", rclcpp::ParameterValue("virtualwalls")); 
    declareParameter("virtualwall_subscribe_transient_local", rclcpp::ParameterValue(true));
    declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
    
    auto node = node_.lock();
    clock_ = node->get_clock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }

    node->get_parameter(name_ + "." + "enabled", enabled_);    
    // node->get_parameter(name_ +  "." + "virtualwall_topic", virtualwall_topic_);
    node->get_parameter(name_ + "." + "virtualwall_subscribe_transient_local",
        virtualwall_subscribe_transient_local_);
   
    node->get_parameter("transform_tolerance", temp_tf_tol);
    std::string private_virtualwall_topic, global_virtualwall_topic;
    node->get_parameter(name_ + "." + "virtualwall_topic", private_virtualwall_topic);
    node->get_parameter("virtualwall_topic", global_virtualwall_topic);
    RCLCPP_INFO(
        logger_,
         "private_virtualwall_topic : %s, global_virtualwall_topic : %s.",
          private_virtualwall_topic.c_str(), global_virtualwall_topic.c_str()); 
    if (!private_virtualwall_topic.empty()) {
        virtualwall_topic_ = private_virtualwall_topic;
    } else {
        virtualwall_topic_ = global_virtualwall_topic;
    }
    RCLCPP_WARN(
        logger_,
         "virtualwall_topic_ : %s.", virtualwall_topic_.c_str());
    virtualwall_received_ = false;
    update_in_progress_.store(false);
    transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

    // Add callback for dynamic parameters
    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(
        &VirtualWallLayer::dynamicParametersCallback,
        this, std::placeholders::_1));
}

void VirtualWallLayer::processVirtualwalls(const cm_msgs::msg::Virtualwalls & new_virtualwalls)
{
    // RCLCPP_INFO(logger_, "Received virtualwalls."); 

    unsigned int size_x = new_virtualwalls.info.width;
    unsigned int size_y = new_virtualwalls.info.height;
    double origin_x = new_virtualwalls.info.origin.position.x;
    double origin_y = new_virtualwalls.info.origin.position.y;
    double resolution = new_virtualwalls.info.resolution;
    RCLCPP_WARN_THROTTLE(
        logger_, *(clock_), 10000, 
        "VirtualWallLayer::Received a %d X %d new_virtualwalls at %f m/pix, origin(x: %f, y: %f)", size_x, size_y,
        new_virtualwalls.info.resolution, origin_x, origin_y);
        map_frame_ = new_virtualwalls.header.frame_id;

    if (!layered_costmap_->isRolling() && (size_x_ != size_x
            || size_y_ != size_y
            || resolution_ != resolution
            || origin_x_ != new_virtualwalls.info.origin.position.x
            || origin_y_ != new_virtualwalls.info.origin.position.y)) {
        RCLCPP_INFO(logger_, "Resizing Virtuelwall layer to %d X %d at %f m/pix",
                 size_x, size_y, resolution);
        resizeMap(size_x, size_y,
                  resolution,
                  new_virtualwalls.info.origin.position.x,
                  new_virtualwalls.info.origin.position.y);
    } else if (size_x_ != size_x || size_y_ != size_y ||  // NOLINT
      resolution_ != new_virtualwalls.info.resolution ||
      origin_x_ != new_virtualwalls.info.origin.position.x ||
      origin_y_ != new_virtualwalls.info.origin.position.y)
    {
      // only update the size of the costmap stored locally in this layer
      RCLCPP_INFO_THROTTLE(
        logger_, *(clock_), 10000, 
        "Resizing Virtuelwall layer to %d X %d at %f m/pix", size_x, size_y,
        new_virtualwalls.info.resolution);
      resizeMap(
        size_x, size_y, new_virtualwalls.info.resolution,
        new_virtualwalls.info.origin.position.x, new_virtualwalls.info.origin.position.y);
    }

    width_ = size_x_;
    height_ = size_y_;
    virtualwalls_ = new_virtualwalls;

    if (layered_costmap_->isRolling()) {
        resetMaps();
    }
    insertAllVirtualWalls(virtualwalls_);

    // virtualwall_received_ = true;
    has_updated_data_ = true;
    current_ = true;
    RCLCPP_INFO_THROTTLE(
      logger_, *(clock_), 10000,  "Received virtualwalls");
    return;
}

void VirtualWallLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
        double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!virtualwall_received_) {
        RCLCPP_WARN_THROTTLE(
          logger_, *(clock_), 100000, "No virtualwalls received!!!");
        return;
    }

    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    update_in_progress_.store(true); 

    // If there is a new available map, load it.
    if (virtualwalls_buffer_) {
        processVirtualwalls(*virtualwalls_buffer_);
        virtualwalls_buffer_ = nullptr;
    }
    
    if (!layered_costmap_->isRolling() ) {
        if (!(has_updated_data_ || has_extra_bounds_)) {
            return;
        }
    }

    useExtraBounds(min_x, min_y, max_x, max_y);
    double wx, wy;

    mapToWorld(0, 0, wx, wy);
    *min_x = std::min(wx, *min_x);
    *min_y = std::min(wy, *min_y);
    worldToMap(*min_x, *min_y, min_x_, min_y_);

    mapToWorld(width_, height_, wx, wy);
    *max_x = std::max(wx, *max_x);
    *max_y = std::max(wy, *max_y);

    worldToMap(*max_x,*max_y,max_x_,max_y_);

    has_updated_data_ = false;
}

void VirtualWallLayer::updateCosts(
        nav2_costmap_2d::Costmap2D& master_grid,
        int min_i, int min_j, int max_i, int max_j)
{
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex()); 
    if (!enabled_) {
        update_in_progress_.store(false);
        return;
    }
    if (!virtualwall_received_) {
        update_in_progress_.store(false);
        return;
    }
    if (!virtualwall_sub_) {
        return;
    }

    if (!layered_costmap_->isRolling()) {
        // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
        insertAllVirtualWalls(virtualwalls_);
    } else {
        unsigned int mx, my;
        double wx, wy;
        // Might even be in a different frame
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_->lookupTransform(
                map_frame_, global_frame_, tf2::TimePointZero,
                transform_tolerance_);
        } catch (tf2::TransformException & ex) {
            RCLCPP_ERROR(logger_, "VirtualWallLayer: %s", ex.what());
            return;
        }
        // Copy map data given proper transformations
        tf2::Transform tf2_transform;
        tf2::fromMsg(transform.transform, tf2_transform);

        for (int i = min_i; i < max_i; ++i) {
            for (int j = min_j; j < max_j; ++j) {
                // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
                layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
                // Transform from global_frame_ to map_frame_
                tf2::Vector3 p(wx, wy, 0);
                p = tf2_transform * p;
                // Set master_grid with cell from map
                if (worldToMap(p.x(), p.y(), mx, my)) {
                    if (costmap_[getIndex(mx,my)] == LETHAL_OBSTACLE) {
                        master_grid.setCost(i, j, costmap_[getIndex(mx,my)]);
                    }
                }
            }
        }
    }
    // update_in_progress_.store(false);
    current_ = true;
}

void VirtualWallLayer::matchSize()
{
    // If we are using rolling costmap, the static map size is
    //   unrelated to the size of the layered costmap
    if (layered_costmap_->isRolling()) {
        return; 
    }
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(),
              master->getSizeInCellsY(),
              master->getResolution(),
              master->getOriginX(),
              master->getOriginY());
}

void VirtualWallLayer::incomingVirtualwalls(
        const cm_msgs::msg::Virtualwalls::SharedPtr new_virtualwalls)
{
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    if (!virtualwall_received_) {
        virtualwall_received_ = true;
        processVirtualwalls(*new_virtualwalls);
    }
    if (update_in_progress_.load()) {
        virtualwalls_buffer_ = new_virtualwalls;
    } else {
        processVirtualwalls(*new_virtualwalls);
        virtualwalls_buffer_ = nullptr;
    }
 
}

void VirtualWallLayer::insertOneVirtualWall(double start_x, double start_y, double end_x, double end_y)
{
    std::vector<Point2DInt> line;
    unsigned int mx1, my1, mx2, my2 ;
    // Might even be in a different frame
    if (!layered_costmap_->isRolling()) {
        Costmap2D* master = layered_costmap_->getCostmap();
        if (!master->worldToMap(start_x, start_y, mx1, my1) ||
             !master->worldToMap(end_x, end_y, mx2, my2)) {
            // RCLCPP_WARN_THROTTLE(logger_, *(clock_), 3000, 
            //   "WorldPoint(%.2f, %.2f)/(%.2f, %.2f) transferred failed", start_x, start_y, end_x, end_y);
            RCLCPP_WARN_ONCE(logger_,
              "WorldPoint(%.2f, %.2f)/(%.2f, %.2f) transferred failed", start_x, start_y, end_x, end_y);              
            return;
        }
        getLineCells(mx1, my1, mx2, my2, line);
        for (size_t i = 0; i < line.size(); i++) {
            master->setCost(line[i].x, line[i].y, LETHAL_OBSTACLE);
        }
    } else {
        double v_origin_x = virtualwalls_.info.origin.position.x;
        double v_origin_y = virtualwalls_.info.origin.position.y;
        double resolution = virtualwalls_.info.resolution;

        mx1 = static_cast<int>((start_x- v_origin_x) / resolution);
        my1 = static_cast<int>((start_y - v_origin_y) / resolution);
        mx2 = static_cast<int>((end_x- v_origin_x) / resolution);
        my2 = static_cast<int>((end_y - v_origin_y) / resolution);
        getLineCells(mx1, my1, mx2, my2, line);
        for (size_t i = 0; i < line.size(); i++) {
            // if (line[i].x < 0 || line[i].x > size_x_
            //         || line[i].y <0 || line[i].y > size_y_) {
            if (line[i].x > size_x_ || line[i].y > size_y_) {                        
                // RCLCPP_WARN_THROTTLE(logger_, *(clock_), 3000, 
                //   "Point(%d, %d) is OUT of map(%d, %d).", line[i].x, line[i].y, size_x_, size_y_);
                RCLCPP_WARN_ONCE(logger_,
                  "Point(%d, %d) is OUT of map(%d, %d).", line[i].x, line[i].y, size_x_, size_y_);                  
                continue;
            }
            costmap_[getIndex(line[i].x,line[i].y)] = LETHAL_OBSTACLE;
        }
    }
}

void VirtualWallLayer::insertAllVirtualWalls(const cm_msgs::msg::Virtualwalls& virtualwalls)
{

    if (virtualwalls.virtualwalls.size() == 0) {
        return;
    }
    int wall_cnt = static_cast<int>(virtualwalls.virtualwalls.size());
    for (int i = 0; i < wall_cnt; ++i) {
        int point_cnt = static_cast<int>(virtualwalls.virtualwalls[i].point_array.size());
        for (int j = 0; j < point_cnt-1; ++j) {
            double x1 = virtualwalls.virtualwalls[i].point_array[j].x;
            double y1 = virtualwalls.virtualwalls[i].point_array[j].y;
            double x2 = virtualwalls.virtualwalls[i].point_array[j+1].x;
            double y2 = virtualwalls.virtualwalls[i].point_array[j+1].y;
            insertOneVirtualWall(x1, y1, x2, y2);
        }
    }
}

void VirtualWallLayer::virtualWalls2map(double start_x, double start_y, double end_x, double end_y)
{
    double v_origin_x = virtualwalls_.info.origin.position.x;
    double v_origin_y = virtualwalls_.info.origin.position.y;
    double resolution = virtualwalls_.info.resolution;

    unsigned int start_mx , start_my , end_mx , end_my;

    std::vector<Point2DInt> line;

    start_mx = static_cast<int>((start_x- v_origin_x) / resolution);
    start_my = static_cast<int>((start_y - v_origin_y) / resolution);
    end_mx = static_cast<int>((end_x- v_origin_x) / resolution);
    end_my = static_cast<int>((end_y - v_origin_y) / resolution);

    getLineCells(start_mx, start_my, end_mx, end_my, line);

    for (size_t i = 0; i < line.size(); i++) {
        costmap_[getIndex(line[i].x,line[i].y)] = LETHAL_OBSTACLE;
    }
}

void VirtualWallLayer::getLineCells(int x0, int y0, int x1, int y1,
    std::vector<Point2DInt> &cells)
{
    //Bresenham Ray-Tracing
    int dx = abs(x1 - x0);            // The difference between the x's
    int dy = abs(y1 - y0);            // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, e, k, length;

    Point2DInt point;
    if (x1 >= x0) {         // The x-values are increasing
        xinc1 = 1;
        xinc2 = 1;
    } else {            // The x-values are decreasing
        xinc1 = -1;
        xinc2 = -1;
    }

    if (y1 >= y0) {       // The y-values are increasing
        yinc1 = 1;
        yinc2 = 1;
    } else {               // The y-values are decreasing
        yinc1 = -1;
        yinc2 = -1;
    }

    if (dx >= dy) {       // There is at least one x-value for every y-value
        xinc1 = 0;      // Don't change the x when numerator >= denominator
        yinc2 = 0;      // Don't change the y for every iteration
        e = dx;
        den = 2 * dx;
        k =  2 * dy;
        length = dx;     // There are more x-values than y-values
    } else {             // There is at least one y-value for every x-value
        xinc2 = 0;       // Don't change the x for every iteration
        yinc1 = 0;      // Don't change the y when numerator >= denominator
        e = dy;
        den = 2 * dy;
        k = 2 * dx;
        length = dy;    // There are more y-values than x-values
    }

    for (int i = 0; i <= length; i++) {
        point.x = x;      //Draw the current pixel
        point.y = y;
        cells.push_back(point);

        e += k;            // Increase the numerator by the top of the fraction
        if (e >= den) {    // Check if numerator >= denominator
            e -= den;      // Calculate the new numerator value
            x += xinc1;    // Change the x as appropriate
            y += yinc1;    // Change the y as appropriate
        }
        x += xinc2;        // Change the x as appropriate
        y += yinc2;       // Change the y as appropriate
    }
}

/**
  * @brief Callback executed when a parameter change is detected
  * @param event ParameterEvent message
  */
rcl_interfaces::msg::SetParametersResult
VirtualWallLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    rcl_interfaces::msg::SetParametersResult result;

    for (auto parameter : parameters) {
        const auto & param_type = parameter.get_type();
        const auto & param_name = parameter.get_name();

        if (param_type == ParameterType::PARAMETER_BOOL) {
            if (param_name == name_ + "." + "enabled") {
                enabled_ = parameter.as_bool();
                
                x_ = y_ = 0;
                width_ = size_x_;
                height_ = size_y_;
                has_updated_data_ = true;
                current_ = false;
            }
        }
    }

    result.successful = true;
    return result;
}


}  // namespace wr_costmap_2d
