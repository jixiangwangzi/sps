#ifndef CARTOGRAPHER_GLOBAL_MESSAGE_H_
#define CARTOGRAPHER_GLOBAL_MESSAGE_H_
#include "glog/logging.h"
#include <iostream>
#include <mutex>
#include "time.h"
#include "cartographer/transform/rigid_transform.h"
namespace cartographer{
struct PoseWithScore{
    transform::Rigid2d pose;
    double score;
};
class GlobalMessage{
public:
    GlobalMessage():flagTrigGlobalLocalization_(false), num_accumulated_(0), flagMatchMap_(true), flagInWall_(false), 
                    flagUseVisualSlamPose_(false), flagUseTagPose_(false), flagIsLocalization_(false), flagHasSufficientFeatures_(true), flagInsertSubmapID1_(false) {}
    ~GlobalMessage(){        
        std::cout << "~GlobalMessage()" << std::endl;
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

    bool getFlagInsertSubMapID1() 
    { 
	std::lock_guard<std::mutex> lock(mutex_);
        return flagInsertSubmapID1_;
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

};
}
#endif
