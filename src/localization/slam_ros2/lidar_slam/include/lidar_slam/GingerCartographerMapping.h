// #ifndef GINGERCARTOGRAPHERMAPPING_H
// #define GINGERCARTOGRAPHERMAPPING_H

// #include "cartographer_ros/interface_ros.h"
// #include "lidar_slam/GingerSlam.h"

// namespace ginger
// {

// class GingerCartographerMapping : public GingerSlam
// {
// public:
//     GingerCartographerMapping(RosNodeHandlePtr &nh, RosTf2BufferPtr &buffer, RosTf2BroadPtr &cast, const int& slam_model);
//     ~GingerCartographerMapping();
//     bool setOccMap(const nav_msgs::OccupancyGrid &occ_map);
//     bool setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& intial_pose) override;
//     bool SetInitPoseSelf(const ginger_msgs::InitialSlamPose& initial_pose_msg) override;
    
// private:
//     bool updateMap();
//     void publishTransform();
//     void updateMapRun(const ::ros::WallTimerEvent& timer_event);
//     ros::NodeHandle carto_handle_;
//     cartographer_ros::CartographerRosInterface* carto_ros_ = NULL;

//     ::ros::CallbackQueue carto_queue_;
//     ::ros::AsyncSpinner spinner_;
//     ::ros::WallTimer occupancy_grid_publisher_timer_;
//     ::ros::WallDuration map_update_interval_;
//     bool got_map_;
// };

// }


// #endif //GINGERCARTOGRAPHER_H
