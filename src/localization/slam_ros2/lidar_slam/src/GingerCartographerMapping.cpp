// #include "lidar_slam/GingerCartographerMapping.h"

// namespace ginger
// {
//     GingerCartographerMapping::GingerCartographerMapping(RosNodeHandlePtr &nh, RosTf2BufferPtr &buffer, RosTf2BroadPtr &cast, const int& slam_model) :
//          GingerSlam(nh, buffer, cast, 2), carto_handle_("carto_mapping"), spinner_(2, &carto_queue_), map_update_interval_(2),  got_map_(false) 
//     {
//         ROS_INFO("GingerCartographerMapping start");
//         carto_handle_.setCallbackQueue(&carto_queue_);
//         carto_ros_ = new cartographer_ros::CartographerRosInterface(carto_handle_, tf2_buffer_.get(), slam_model);
//         occupancy_grid_publisher_timer_ = carto_handle_.createWallTimer(map_update_interval_,
//                                        &GingerCartographerMapping::updateMapRun, this);
//         spinner_.start();
//     }
//     GingerCartographerMapping::~GingerCartographerMapping(){
//         occupancy_grid_publisher_timer_.stop();
//         if (carto_ros_ != NULL)
//         {
//             spinner_.stop();
//             delete carto_ros_;
//             carto_ros_ = NULL;
//         }
//     }

//     bool GingerCartographerMapping::setOccMap(const nav_msgs::OccupancyGrid &occ_map)
//     {
//         return carto_ros_->SetOccMap(occ_map);
//     }

//     bool GingerCartographerMapping::setInitialPose(const geometry_msgs::PoseWithCovarianceStamped& intial_pose)
//     {
//         spinner_.stop();
//         ros::Duration(0.2).sleep();
//         bool res = carto_ros_->SetInitialPose(intial_pose);
//         spinner_.start();
//         return res;
//     }


//     bool GingerCartographerMapping::SetInitPoseSelf(const ginger_msgs::InitialSlamPose& initial_pose_msg){
//         spinner_.stop();
//         ros::Duration(0.2).sleep();
//         auto intial_pose = initial_pose_msg.initial_pose;
//         bool res = carto_ros_->SetInitialPose(intial_pose);
//         if(initial_pose_msg.need_optimize == initial_pose_msg.OPTIMIZE){
//             res = carto_ros_->SetFlagMatchMap(true);
//             ROS_INFO("SetInitPoseSelf:: set match map true");
//         }
//         else{
//             res = carto_ros_->SetFlagMatchMap(false);
//             ROS_INFO("SetInitPoseSelf:: set match map false");
//         }
//         spinner_.start();
//         return res;
//     }

//     void GingerCartographerMapping::updateMapRun(const ::ros::WallTimerEvent& unused_timer_event){
//         if(!update_map_) {
//             return;
//         }
//         if (!carto_ros_) return;
//         auto carto_occ_map = carto_ros_->GetOccMap();

//         if (carto_occ_map)
//         {
//             for(int i = 0; i < carto_occ_map->data.size(); i++)
//             {
//                 if(carto_occ_map->data[i] > 60){
//                     carto_occ_map->data[i] = 100;
//                 }
//                 else if(carto_occ_map->data[i] >= 0 and carto_occ_map->data[i] < 40){
//                     carto_occ_map->data[i] = 0;
//                 }
//                 else{
//                     carto_occ_map->data[i] = -1;
//                 }
//             }	
//             // Set the header information on the map
//             carto_occ_map->header.stamp = ros::Time::now();
//             // carto_occ_map->header.frame_id = map_frame_;
//             PublishOccMap(*carto_occ_map);
//         }
//     }
// }
