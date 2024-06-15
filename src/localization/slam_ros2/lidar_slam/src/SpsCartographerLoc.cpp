#include "lidar_slam/SpsCartographerLoc.h"
#include <glog/logging.h>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


namespace sps
{
    SpsCartographerLoc::SpsCartographerLoc(RosNodeHandlePtr &nh, RosTf2BufferPtr &buffer, RosTf2BroadPtr &cast) :
         SpsSlam(nh, buffer, cast)//, carto_handle_("carto_loc")//, spinner_(1, &carto_queue_)
    {
        LOG(INFO) << "SpsCartographerLoc start!";
        // ROS_INFO("GingerCartographerLoc start");
        // carto_handle_.setCallbackQueue(&carto_queue_);
        carto_ros_ = new cartographer_ros::CartographerLocRosInterface(nh, tf2_buffer_);
        loc_status_pub_ = nh_->create_publisher<sps_common_msgs::msg::LocalizationStatus>("/localization_status", 10);
        imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>("/imu/data",rclcpp::SensorDataQoS(),
                            std::bind(&SpsCartographerLoc::ImuCallBack,this,std::placeholders::_1));

        map_pub_ = nh_->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_loc",10);

        // loc_status_pub_ = nh_->create_publisher<sps_common_msgs::msg::LocalizationStatus>("/localization_status", 10);
        // callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        loc_confidence_timers_ = nh_->create_wall_timer(5000ms, 
                         std::bind(&SpsCartographerLoc::PublishLocConfidence, this));
        LOG(INFO) << "SpsCartographerLoc end!";
    }
    
    SpsCartographerLoc::~SpsCartographerLoc()
    {

        if (loc_confidence_timers_) {
            loc_confidence_timers_->cancel();
            loc_confidence_timers_.reset();
        }

        if (carto_ros_ != NULL)
        {
            rclcpp::sleep_for(200ms);
            delete carto_ros_;
            carto_ros_ = NULL;
        }
    }

    bool SpsCartographerLoc::setOccMap(const nav_msgs::msg::OccupancyGrid &occ_map)
    {
        LOG(INFO) << "occ_map: resolution: " << occ_map.info.resolution;
        map_ = occ_map;
        return carto_ros_->SetOccMap(occ_map);
    }

    int SpsCartographerLoc::setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& intial_pose)
    {
        rclcpp::sleep_for(200ms);
        // bool res = carto_ros_->SetInitialPose(intial_pose);
        LOG(INFO)<<"GingerCartographerLoc:: setInitialPose start";
        carto_ros_->SetFlagMatchMap(true);
        LOG(INFO)<<"SetInitPose:: set match map true";
        std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> pose;
        pose.push_back(intial_pose);
        int res = carto_ros_->SetInitialPose(pose);
        return res;
    }


    int SpsCartographerLoc::SetInitPoseSelf(const sps_common_msgs::msg::InitialSlamPose& initial_pose_msg){
        rclcpp::sleep_for(200ms);
        auto intial_pose = initial_pose_msg.initial_pose;
        LOG(INFO)<<"GingerCartographerLoc:: SetInitPoseSelf start";

        using InitPose = sps_common_msgs::msg::InitialSlamPose;
        int res = sps_common_msgs::srv::AidedPose::Response::SUCCESS;
        LOG(INFO) << "Initial pose with scene type: " << static_cast<int>(initial_pose_msg.scene_type)
                  << ", optimize type: " << static_cast<int>(initial_pose_msg.optimize_type);
        if (initial_pose_msg.scene_type == InitPose::UNKNOWN) {
            LOG(INFO) << "Received init pose in origin of map, DO NOT set!";
            return res;
        } else {
            switch (initial_pose_msg.scene_type)
            {
            case InitPose::CHARGING_PILE:
                if (intial_pose.size() > 1) { // multi charging pile
                    LOG(INFO)<<"SetInitPoseSelf:: multi charging pile pose.";
                    carto_ros_->SetFlagMatchMap(true);
                    LOG(INFO)<<"SetInitPoseSelf:: set match map true";
                    res = carto_ros_->SetInitialPose(intial_pose);
                } else {
                    carto_ros_->SetFlagMatchMap(false);
                    LOG(INFO)<<"SetInitPoseSelf:: set match map false";
                    res = carto_ros_->SetInitialPose(intial_pose);
                }
                break;
            case InitPose::LIFT_INTERIOR:
                carto_ros_->SetFlagMatchMap(false);
                LOG(INFO)<<"SetInitPoseSelf:: set match map false";
                res = carto_ros_->SetInitialPose(intial_pose);
                break;
            case InitPose::START_UP_WITH_SAVED_POSE:
            case InitPose::SWITCH_MAP_WITH_SAVED_POSE:
            case InitPose::CLOUD_INSTRUCTION:
                carto_ros_->SetFlagMatchMap(true);
                LOG(INFO)<<"SetInitPoseSelf:: set match map true";
                res = carto_ros_->SetInitialPose(intial_pose);
                break;
            default:
                LOG(ERROR) << "Received init pose with unknown scene type";
                break;
            }

        }

        return res;
    }
    
    bool SpsCartographerLoc::naviTypeCB(const std_msgs::msg::Int32& msg){
        int naviType = msg.data;
        return carto_ros_->naviTypeCB(naviType);
    }

    void SpsCartographerLoc::ImuCallBack(const sensor_msgs::msg::Imu::ConstPtr msg){
        last_stamp_imu_ = nh_->now().seconds();
    }

    void SpsCartographerLoc::PublishLocConfidence() 
    {

        sps_common_msgs::msg::LocalizationStatus loc_status;
        rclcpp::Clock clock(RCL_SYSTEM_TIME);
        loc_status.header.frame_id = "map";
        loc_status.header.stamp = clock.now();
        loc_status.status = 0;



        map_pub_->publish(map_);
        bool isBadScan = carto_ros_->IsBadScanData();
        bool useVisualFlag = carto_ros_->UseVisualFlag();
        bool jumpPoseFlag = carto_ros_->GetJumpPoseFlag();
        bool isFinishChangeMap = carto_ros_->IsFinishChangeMap();
        bool useConfidence = carto_ros_->UseConfidence();
        bool laser_lost = carto_ros_->GetLaserLostFlag();
        bool odom_lost = carto_ros_->GetOdomLostFlag();
        bool imu_lost = false;
        bool init_fail = false; 
        // if(nh_->now().seconds() - last_stamp_imu_ > 1.0)
        // {    
        //     loc_status.status = 5; //传感器数据异常
        //     loc_status.specific_causes = 9; 
        //     imu_lost = true;
        // }
        bool initfinish = carto_ros_->GetInitFinishFlag();

        if(!initfinish)
        {
            loc_status.status= 1; //初始化中
            if(fisrt_flag)
            {
                last_stamp_init_ = nh_->now().seconds();
                fisrt_flag = false;
            }

        }
        else
        {       
            geometry_msgs::msg::PoseWithCovarianceStamped temp_data = GetRobotPose();
            geometry_msgs::msg::Pose robot_pose;
            robot_pose = temp_data.pose.pose;
            loc_status.pose = robot_pose;

            loc_status.status = 2; //初始化完成运行中
            last_stamp_init_ = nh_->now().seconds();
        }
        
        
        if((nh_->now().seconds()-last_stamp_init_) > 30.0 && !initfinish)
            init_fail = true;
        
        double loc_confidence = carto_ros_->GetLocConfidence();
        double visual_confidence = carto_ros_->GetVisualConfidence();

        double laser_th = carto_ros_->GetLaserTh();
        double visual_th = carto_ros_->GetVisualTh();
        double clear_th = carto_ros_->GetCancelTh();

        float real_confidence = loc_confidence;
        real_confidence = std::min<float>(real_confidence, 1.0f);

        loc_status.score = real_confidence;

        bool flagInWall = carto_ros_->GetFlagInWall();
        bool laser_low_confidence = false;
        bool visual_low_confidence = false;
        if(isFinishChangeMap)
        {
            if(useConfidence)
            {
                if(!isBadScan)
                {

                    if(loc_confidence < laser_th)
                    {
                        laser_count_++;
                        if(laser_count_ > 15)
                        {
                            laser_low_confidence = true;
                        }
                    }
                    else
                    {
                        laser_count_=0;
                        laser_low_confidence = false;

                    }
                }

                if(visual_confidence < visual_th && useVisualFlag)
                {
                    visual_count_++;
                    if(visual_count_ >15)
                    {
                        visual_low_confidence = true;
                    }
                }
                else
                {
                    visual_count_ =0;
                    visual_low_confidence = false;
                }

                if(laser_low_confidence)
                {
                    loc_status.status =3;
                    loc_status.specific_causes =4;
                }

                if(visual_low_confidence)
                {
                    loc_status.status =3;
                    loc_status.specific_causes =5;
                }

                if(laser_low_confidence && visual_low_confidence)
                {
                    loc_status.status = 4;
                    loc_status.specific_causes =3;
                }

                if(laser_lost && initfinish)
                {
                    loc_status.status =5;
                    loc_status.specific_causes =6;
                }

                if(odom_lost && initfinish)
                {
                    loc_status.status =5;
                    loc_status.specific_causes =8;
                }

                if(imu_lost && initfinish)
                {
                    loc_status.status = 5; //传感器数据异常
                    loc_status.specific_causes = 9; 
                }

                if(flagInWall)
                {
                    loc_status.status =4;
                    loc_status.specific_causes =1;
                }

                if(jumpPoseFlag)
                {
                    loc_status.status =4;
                    loc_status.specific_causes =2;
                }

            }
        }
        if(initfinish)
        {
            publishLocConfidence(real_confidence);
            PublishRobotPose();
            loc_status_pub_->publish(loc_status);
        }



    }


} //namespace
