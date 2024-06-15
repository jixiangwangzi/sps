#include "system_manager/naviengine_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace naviengine
{

NaviEngineNode::NaviEngineNode(RobotBt* robotBt) : Node("naviengine_node")
{
    ROS_INFO("hello this is naviengine_node.");
    robotBt_ = robotBt;
    ParameterInit();
    ServiceInit();
    ClientInit();
    PublishInit();
    SubscriberInit();
    // publish_twist_timer_ = this->create_wall_timer(1s, std::bind(&NaviEngineNode::TwistPublish, this));
    publish_twist_thread_ = std::make_shared<std::thread>(&NaviEngineNode::TwistPublishThread, this);
}

NaviEngineNode::~NaviEngineNode()
{
    if (publish_twist_thread_)
    {
        publish_twist_thread_->join();
    }
}

void NaviEngineNode::ParameterInit()
{
    ROS_INFO("ParameterInit");
}

void NaviEngineNode::ServiceInit()
{
    ROS_INFO("servicen_init");
    navi_to_pose_server_ = std::make_shared<SystemNaviToPoseService>(*this);
}

void NaviEngineNode::ClientInit()
{
    ROS_INFO("clientInit_init");
    pixel_to_word_server_client_ = std::make_shared<PixelToWordServiceClient>();
    query_poi_type_server_client_ = std::make_shared<QueryPoiTypeServiceClient>();
    query_lift_poi_type_server_client_ = std::make_shared<QueryLiftPoiTypeServiceClient>();
}

void NaviEngineNode::PublishInit()
{
    ROS_INFO("publish_init");
    navi_state_pub_ = std::make_shared<SystemNaviStatePublisher>(*this);
    cloud_twist_mux_pub_ = std::make_shared<CloudTwistMuxPublisher>(*this);
    cloud_twist_chassis_pub_ = std::make_shared<CloudTwistChassisPublisher>(*this);
}

void NaviEngineNode::SubscriberInit()
{
    ROS_INFO("subscription_init");
    map_info_sub_ = std::make_shared<SystemMapInfoSubscriber>(*this);

    robot_state_alarm_sub_ = this->create_subscription<sps_common_msgs::msg::RobotStateAlarm>(
        "/robot_state_alarm", 1, std::bind(&NaviEngineNode::robot_state_alarm_callback, this, std::placeholders::_1));
}

void NaviEngineNode::robot_state_alarm_callback(const sps_common_msgs::msg::RobotStateAlarm::SharedPtr msg)
{
    ROS_INFO("NaviEngineNode::robot_state_alarm_callback visual_drop_state:%d, emergency_state:%d, ha_state:%d, collision_state:%d, hw_error_state:%d",
                msg->visual_drop_state, msg->emergency_state, msg->ha_state, msg->collision_state, msg->hw_error_state);

    visual_drop_state_ = msg->visual_drop_state;
    emergency_state_ = msg->emergency_state;
    HA_state_ = msg->ha_state;
    collision_state_ = msg->collision_state;
    hw_error_state_ = msg->hw_error_state;

    robotBt_->RobotAlarmHandle(IsAlarmState());
}

void NaviEngineNode::TwistPublish()
{
    ROS_INFO("TwistPublish");
    if (continuous_twist_ctrl_)
    {
        ROS_INFO("cloud_twist_publish");
        if (collision_check_)
        {
            cloud_twist_mux_pub_->Publish(twist_stamped_);
        }
        else
        {
            cloud_twist_chassis_pub_->Publish(twist_stamped_);
        }
    }
}


ERESULT NaviEngineNode::ChangePixelToWord(const PixelPose &pixel_pose, Pose &pose)
{
    ROS_INFO("ChangePixelToWord");
    auto pixel_to_word = std::make_shared<sps_common_msgs::srv::ChangePixelPoseToWordPose::Request>();
    pixel_to_word->pixel_pose.x = pixel_pose.x;
    pixel_to_word->pixel_pose.y = pixel_pose.y;
    pixel_to_word->pixel_pose.theta = pixel_pose.theta;
    auto word_pose = std::make_shared<sps_common_msgs::srv::ChangePixelPoseToWordPose::Response>();
    if (pixel_to_word_server_client_->call(pixel_to_word, word_pose))
    {
        pose.position.x = word_pose->word_pose.position.x;
        pose.position.y = word_pose->word_pose.position.y;
        pose.position.z = word_pose->word_pose.position.z;
        pose.orientation.x = word_pose->word_pose.orientation.x;
        pose.orientation.y = word_pose->word_pose.orientation.y;
        pose.orientation.z = word_pose->word_pose.orientation.z;
        pose.orientation.w = word_pose->word_pose.orientation.w;
        return E_OK;
    }
    else
    {
        return E_FALSE;
    }
}


ERESULT NaviEngineNode::QueryPointType(const PixelPose &pixel_pose, PoiType &poi_type)
{
    ROS_INFO("QueryPointType");
    auto query_poi_type_req = std::make_shared<sps_common_msgs::srv::QueryPoiTypeByPose::Request>();
    query_poi_type_req->pixel_pose.x = pixel_pose.x;
    query_poi_type_req->pixel_pose.y = pixel_pose.y;
    query_poi_type_req->pixel_pose.theta = pixel_pose.theta;
    auto query_poi_type_resp = std::make_shared<sps_common_msgs::srv::QueryPoiTypeByPose::Response>();
    if (query_poi_type_server_client_->call(query_poi_type_req, query_poi_type_resp))
    {
        poi_type = (PoiType)query_poi_type_resp->poi_type;
        return E_OK;
    }
    else
    {
        poi_type = UNKOWN_TYPE;
        return E_FALSE;
    }
}


ERESULT NaviEngineNode::QueryLiftPointType(const PixelPose &pixel_pose, PoiType &poi_type, std::string &lift_id)
{
    ROS_INFO("QueryLiftPointType");
    auto query_lift_poi_type_req = std::make_shared<sps_common_msgs::srv::QueryLiftPoiType::Request>();
    query_lift_poi_type_req->pixel_pose.x = pixel_pose.x;
    query_lift_poi_type_req->pixel_pose.y = pixel_pose.y;
    query_lift_poi_type_req->pixel_pose.theta = pixel_pose.theta;
    auto query_lift_poi_type_resp = std::make_shared<sps_common_msgs::srv::QueryLiftPoiType::Response>();
    if (query_lift_poi_type_server_client_->call(query_lift_poi_type_req, query_lift_poi_type_resp))
    {
        poi_type = (PoiType)query_lift_poi_type_resp->lift_poi_type;
        lift_id = query_lift_poi_type_resp->lift_id;
        return E_OK;
    }
    else
    {
        poi_type = UNKOWN_TYPE;
        lift_id = "";
        return E_FALSE;
    }
}


ERESULT NaviEngineNode::StartMove(const Twist &twist, const bool collision_check)
{
    ROS_INFO("NaviEngineNode::StartMove entry");
    collision_check_ = collision_check;
    continuous_twist_ctrl_ = true;
    twist_stamped_.header.stamp = rclcpp::Time();
    twist_stamped_.twist.linear.x = twist.vx;
    twist_stamped_.twist.linear.y = twist.vy;
    twist_stamped_.twist.angular.x = twist.vz;
    return E_OK;
}

ERESULT NaviEngineNode::StopMove()
{
    ROS_INFO("NaviEngineNode::StopMove entry");
    if (continuous_twist_ctrl_)
    {
        continuous_twist_ctrl_ = false;
        return E_OK;
    }
    else
    {
        return E_REPEAT;
    }
}

void NaviEngineNode::SetNaviTaskId(const std::string &task_id)
{
    ROS_INFO("NaviEngineNode::SetNaviTaskId entry");
    completed_task_id_ = task_id;
}

bool NaviEngineNode::IsCompletedTask(const std::string &task_id)
{
    ROS_INFO("NaviEngineNode::IsCompletedTask entry");


    if (!task_id.empty() && !completed_task_id_.empty() && task_id == completed_task_id_)
    {
        ROS_INFO("IsCompletedTask exit with repeated task");

        return true;
    }
    return false;
}

bool NaviEngineNode::IsAlarmState()
{
    bool result = false;
    if(visual_drop_state_ || emergency_state_ || HA_state_ || collision_state_ || hw_error_state_)
    {
        ROS_INFO("NaviEngineNode::IsAlarmState visual_drop_state_:%d, emergency_state_:%d, HA_state_:%d, collision_state_:%d, hw_error_state_:%d",
                visual_drop_state_, emergency_state_, HA_state_, collision_state_, hw_error_state_);
        result = true;
    }
    return result;
}

void NaviEngineNode::TwistPublishThread()
{
    rclcpp::Rate rate(cloud_twist_pub_rate_);


    while (rclcpp::ok())
    {
        // ROS_INFO("TwistPublish");
        if (continuous_twist_ctrl_)
        {
            ROS_INFO("cloud_twist_publish");
            if (collision_check_)
            {
                cloud_twist_mux_pub_->Publish(twist_stamped_);
            }
            else
            {
                cloud_twist_chassis_pub_->Publish(twist_stamped_);
            }
        }
        rate.sleep();
    }
}

}