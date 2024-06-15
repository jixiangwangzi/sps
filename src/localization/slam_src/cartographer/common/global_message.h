#ifndef CARTOGRAPHER_GLOBAL_MESSAGE_H_
#define CARTOGRAPHER_GLOBAL_MESSAGE_H_
#include "glog/logging.h"
#include <iostream>
#include <mutex>
#include "time.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/sensor/point_cloud.h"
namespace cartographer{

using ::cartographer::transform::Rigid2d;
struct PoseWithScore{
    transform::Rigid2d pose;
    double score;
};

enum InitState {
    FAILED = -1,
    NONE,
    NOT_INITIALZED,
    SET_INITIAL_POSE,
    BUILD_CONSTRAINT,
    SUCCESS
};
class GlobalMessage{
public:
    GlobalMessage():flagTrigGlobalLocalization_(false), num_accumulated_(0), flagMatchMap_(true), flagInWall_(false), 
                    flagUseVisualSlamPose_(false), flagUseTagPose_(false), flagIsLocalization_(false), 
                    flagHasSufficientFeatures_(true), flagInsertSubmapID1_(false),init_constraint_finished_(false) {
        LOG(INFO) << "GlobalMessage()";
    }
    ~GlobalMessage(){        
        LOG(INFO) << "~GlobalMessage()" << std::endl;
    }
    bool setFlagTrigGlobalLocalization(bool value){ 
        flagTrigGlobalLocalization_ = value;
        return true;
    }
    bool getFlagTrigGlobalLocalization(){return flagTrigGlobalLocalization_;}

    bool setNumAccumulated(int num){
        num_accumulated_ = num;
        return true;
    }

    int getNumAccumulated(){ return num_accumulated_;}

    bool setFlagMatchMap(bool value){
        flagMatchMap_ = value;
        return true;
    }
    bool getFlagMatchMap(){return flagMatchMap_;}

    bool setFlagInWall(bool value){
        time_t now = time(NULL);
        if(value == true){
            auto duration = now - lastTimeSetFlagInWall_;
            if(duration < 3 || lastTimeSetFlagInWall_ == 0){
                return false;
            }
            // LOG(INFO) << "lastTimeSetFlagInWall_ : " << lastTimeSetFlagInWall_;
            // LOG(INFO) << "timeNow_ : " << now;
        }
       flagInWall_ = value;
       lastTimeSetFlagInWall_ = now;
       return true; 
    }
    bool getFlagInWall(){return flagInWall_;}

    bool setPoseFromVisualSlam(const PoseWithScore& pose){
        std::lock_guard<std::mutex> lock(mutex_);
        poseFromVisualSlam_ = pose;
        flagUseVisualSlamPose_ = true;
        return true;
    }
    PoseWithScore getPoseFromVisualSlam(){
        std::lock_guard<std::mutex> lock(mutex_);
        flagUseVisualSlamPose_ = false;
        return poseFromVisualSlam_;
    }
    bool setFlagUseVisualSlamPose(const bool& value){
        std::lock_guard<std::mutex> lock(mutex_);
        flagUseVisualSlamPose_ = value;
        return true;
    }
    bool getFlagUseVisualSlamPose(){
        std::lock_guard<std::mutex> lock(mutex_);
        return flagUseVisualSlamPose_;
    }

    bool setFlagUseSubmapMatch(const bool& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        flagUseSubmapMatch_ = value;
        return true;
    }

    bool getFlagUseSubmapMatch() {
        std::lock_guard<std::mutex> lock(mutex_);
        return flagUseSubmapMatch_;
    }

    bool setSubmapFreeThreshold(const double& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        submap_free_threshold_ = value;
        return true;
    }

    double getSubmapFreeThreshold() {
        std::lock_guard<std::mutex> lock(mutex_);
        return submap_free_threshold_;
    }

    bool setTimedSubmapPointcloud(cartographer::common::Time t, std::shared_ptr<sensor::PointCloud> submap_pointcloud_ptr) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::pair<cartographer::common::Time, std::shared_ptr<sensor::PointCloud>> tmp(t, submap_pointcloud_ptr);
        timedSubmapPointCloud_.swap(tmp);
        return true;
    }

    std::pair<cartographer::common::Time, std::shared_ptr<sensor::PointCloud>> getTimedSubmapPointcloud() {
        std::lock_guard<std::mutex> lock(mutex_);
        return timedSubmapPointCloud_;
    }

    bool setPoseFromTag(const PoseWithScore& pose){
        std::lock_guard<std::mutex> lock(mutex_);
        poseFromTag_ = pose;
        flagUseTagPose_ = true;
        return true;
    }
    PoseWithScore getPoseFromTag(){
        std::lock_guard<std::mutex> lock(mutex_);
        flagUseTagPose_ = false;
        return poseFromTag_;
    }
    bool getFlagUseTagPose(){
        std::lock_guard<std::mutex> lock(mutex_);
        return flagUseTagPose_;
    }

    bool setFlagIsLocalization(const bool& value){
        flagIsLocalization_ = value;
        return true;
    }
    bool getFlagIsLocalization(){
        return flagIsLocalization_;
    }

    bool setFlagHasSufficientFeatures(const bool& value){
        std::lock_guard<std::mutex> lock(mutex_);
        flagHasSufficientFeatures_ = value;
        return true;
    }
    bool getFlagHasSufficientFeatures(){
        std::lock_guard<std::mutex> lock(mutex_);
        return flagHasSufficientFeatures_;
    }

    void setFlagInsertSubMapID1(const bool& value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        flagInsertSubmapID1_ = value;
    }

    bool getFlagInsertSubMapID1() { 
        std::lock_guard<std::mutex> lock(mutex_);
        return flagInsertSubmapID1_;
    }
    
    void setMaxRange(const float& max_range){
        std::lock_guard<std::mutex> lock(mutex_);
        max_range_ = max_range;
    }

    float getMaxRange(){
        std::lock_guard<std::mutex> lock(mutex_);
        return max_range_;
    }

    void setFlagInitConstraint(const bool value) {
        std::lock_guard<std::mutex> lock(mutex_);
        init_constraint_finished_ = value;
    }

    bool getFlagInitConstraint() {
        std::lock_guard<std::mutex> lock(mutex_);
        return init_constraint_finished_;
    }

    void setInitState(const int& state) {
        std::lock_guard<std::mutex> lock(mutex_);
        init_state_ = state;

    }

    int getInitState() {
        std::lock_guard<std::mutex> lock(mutex_);
        return  init_state_;
    }

    void setMultiInitPoses(const std::vector<Rigid2d>& init_poses) {
        std::lock_guard<std::mutex> lock(mutex_);
        initial_poses_.clear();
        initial_poses_ = init_poses;
    }

    std::vector<Rigid2d> getMultiInitPoses() { // carto::transform::
        std::lock_guard<std::mutex> lock(mutex_);
        return initial_poses_;
    }

private:
    std::mutex mutex_;
    //触发重定位标志位
    bool flagTrigGlobalLocalization_;
    //累计的激光帧数
    int num_accumulated_;
    //是否与地图间建立约束
    bool flagMatchMap_;
    //是否处于墙内
    bool flagInWall_;
    time_t lastTimeSetFlagInWall_; 
    //视觉slam的位置
    PoseWithScore poseFromVisualSlam_;
    bool flagUseVisualSlamPose_;
    //二维码tag的位置
    PoseWithScore poseFromTag_;
    bool flagUseTagPose_;
    //是否定位模式
    bool flagIsLocalization_;
    //是否包含充足特征
    bool flagHasSufficientFeatures_;
    // 
    bool flagInsertSubmapID1_;
    float max_range_;
    //子图与地图匹配
    bool flagUseSubmapMatch_;
    double submap_free_threshold_;
    std::pair<cartographer::common::Time, std::shared_ptr<sensor::PointCloud>> timedSubmapPointCloud_;
    // localization init 
    int init_state_ = InitState::NONE;
    bool init_constraint_finished_; // 初始化时后端约束构建完成(激光与地图匹配，视觉定位或Tag约束)
    std::vector<Rigid2d> initial_poses_;


};
}
#endif
