#include "server_interface/api_service/set_slam_model.hpp"

SetSlamModelService::SetSlamModelService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<cm_msgs::srv::SpsSetSlamModel>("/sps_set_slam_model",
                                             std::bind(&SetSlamModelService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void SetSlamModelService::ServiceCallback(cm_msgs::srv::SpsSetSlamModel::Request::SharedPtr request,
                            cm_msgs::srv::SpsSetSlamModel::Response::SharedPtr response)
{
    ROS_INFO("set_slam_model_service header.stamp: %d, slam_mode: %d", request->header.stamp.sec, request->slam_mode);

    ERESULT res = engine_.SetSlamModel(request->slam_mode);
    response->result = res;

    ROS_INFO("set_slam_model_service result: %d", res);
}



