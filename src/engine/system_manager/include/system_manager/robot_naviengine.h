

#ifndef _ROBOT_NAVIENGINE_H_
#define _ROBOT_NAVIENGINE_H_

#include <set>
#include <string>

#include "bt/bt_define.h"
#include "bt/robot_bt.h"
#include "define.h"
#include "naviengine_node.hpp"
#include "robot_ctrl.h"
#include "robot_engine_observer.h"
#include "sps_fsm.hpp"


namespace naviengine
{


class RobotNaviEngine : public IEngineCtrl
{
public:
    static RobotNaviEngine &GetInstance()
    {
        static RobotNaviEngine instance;
        return instance;
    }

    // Must call before any func call once you get instance
    ERESULT Init(IRobotEngineObserver &robotEngineObserver, rclcpp::executors::MultiThreadedExecutor &exector);

public:
    // Map Interface
    ERESULT StartBuildMap(const std::string &task_id, EMAPPINGTYPE mapping_type, EMAPPINGMODE mapping_mode);

    ERESULT StopBuildMap(const std::string &task_id);

    ERESULT PauseBuildMap(const std::string &task_id);

    ERESULT ResumeBuildMap(const std::string &task_id);

    ERESULT SaveMap(const std::string &task_id, const std::string &map_name);

    ERESULT UpdateMap(const std::string &task_id, const std::string &map_name);

    ERESULT LoadMap(const std::string &task_id, const std::string &map_name);

    // Navi Interface
    // ERESULT StartNavi(const std::string &task_id, ENAVITYPE type, const std::string &goal_name, const PixelPose
    // &goal_pose,
    //           const float &goal_tolerance = 0.5);

    ERESULT StartNaviToPose(const Header &header,
                            const std::string &task_id,
                            const std::string &lift_id,
                            ENAVITYPE type,
                            const Pose &pose,
                            const Twist &twist,
                            const std::string &bt_xml_filename,
                            const bool align_angle);

    ERESULT StartNaviThroughPoses(const Header &header,
                                  const std::string &task_id,
                                  ENAVITYPE type,
                                  const std::vector<Pose> &points,
                                  const std::vector<int> &points_type,
                                  const Twist &twist,
                                  const std::string &bt_xml_filename,
                                  const bool align_angle);

    ERESULT StopNavi(const std::string &task_id);

    ERESULT PauseNavi(const std::string &task_id);

    ERESULT ResumeNavi(const std::string &task_id);

    // State Interface
    SPSSTATE GetState();

    // Normal
    ERESULT CheckPose(const std::string name, PixelPose &pixel_pose);


    ERESULT SetPoseinitialize(const Pose &world_pose, SENSORMATCHTYPE match_type, LOCALINITTYPE init_type, PoiType poi_type = UNKOWN_TYPE);

    ERESULT SetLocalization(const Pose &world_pose, SENSORMATCHTYPE match_type, LOCALINITTYPE init_type);

    ERESULT SetMaxNaviVel(const double linear_max, const double angular_max);

    double GetNaviPosesMileage(const Pose &start_pose, const Pose &target_pose);

    ERESULT Move()
    {
        return E_NOTSUPPORT;
    }

    ERESULT StartHA(const std::string &task_id);

    ERESULT StopHA(const std::string &task_id);

    ERESULT SetDefaultCharger(const std::string &map_id,
                              const std::string &map_name,
                              const std::string &charger_name,
                              const PixelPose &pose);

    ERESULT SetNaviHalfPoints(const std::vector<PixelPose> &points);

    ERESULT ReportNaviPath(const std::string &task_id);

    ERESULT ChargingToDock(const std::string &task_id, const bool cmd);

    ERESULT NaviCmd(const std::string &task_id, const NAVICMD cmd);

    ERESULT SetParamData(const std::string &name, const std::string &data);

    ERESULT SwitchSensorData(SENSORTYPE sensor_type, const bool state);

    ERESULT SwitchSensorEnable(SENSORTYPE sensor_type, const bool state);

    ERESULT SetSlamModel(const int slam_model);

    ERESULT StartMove(const Twist &twist, const bool collision_check);

    ERESULT StopMove();


    // Event
    ERESULT NotifyDropEvent(DropType type);

    ERESULT NotifyCollisionEvent(CollisionType type);

    ERESULT NotifyEStopEvent(EmergencyType type);

    ERESULT NotifyChargingEvent(const BatteryState &state);

    ERESULT NotifyCriticalHwErrEvent(HWErrorType type);

    // EngineCtrl
    ERESULT OnUpdateFsmState(NAVISTATE state, const std::string &task_id) override;

    bool GetTaskInfo(std::string &task_id, ENAVITYPE &navi_type);

    ERESULT ChangePixelToWord(const PixelPose &pixel_pose, Pose &pose);

    ERESULT QueryPointType(const PixelPose &pixel_pose, PoiType &poi_type);

    ERESULT QueryLiftPointType(const PixelPose &pixel_pose, PoiType &poi_type, std::string &lift_id);

private:
    RobotNaviEngine();

    virtual ~RobotNaviEngine();

    RobotNaviEngine(const RobotNaviEngine &){};

    RobotNaviEngine &operator=(const RobotNaviEngine &)
    {
        return *this;
    };


    ERESULT StartNaviToPoseIfTransformState();
    ERESULT StartNaviIfTransformState();
    ERESULT InDocking(const std::string &task_id);
    ERESULT UnDocking(const std::string &task_id);
    ERESULT NaviCmdInner(const std::string &task_id, const NAVICMD cmd);
    std::string SetNaviXml(const std::string &bt_xml_filename);
    RobotBt *robotBt_;

    std::shared_ptr<NaviEngineNode> naviEngineNode_;

    std::shared_ptr<Fsm> fsm = nullptr;
};


}  // namespace naviengine

#endif
