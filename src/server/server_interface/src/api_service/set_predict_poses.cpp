#include "server_interface/api_service/set_predict_poses.hpp"

SetPredictPosesService::SetPredictPosesService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::SetPredictPoses>("/set_predict_poses",
                                             std::bind(&SetPredictPosesService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void SetPredictPosesService::ServiceCallback(sps_common_msgs::srv::SetPredictPoses::Request::SharedPtr request,
                            sps_common_msgs::srv::SetPredictPoses::Response::SharedPtr response) {
    ROS_INFO("set_predict_poses_service");
    //engine_.StartBuildMap("789456", E_NORMAL_MAPPING_MODE);
    response->result = 100;
}

