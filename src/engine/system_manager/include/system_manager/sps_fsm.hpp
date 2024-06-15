
#ifndef SPS_FSM_HPP_
#define SPS_FSM_HPP_


#include <string>
#include <limits>
#include "map"
#include "set"
#include "define.h"
#include "ament_index_cpp/get_package_share_directory.hpp"


namespace naviengine
{


const std::string SPS_STATE_INIT      = "INIT";
const std::string SPS_STATE_IDLE      = "IDLE";
const std::string SPS_STATE_WORKING   = "WORKING";
const std::string SPS_STATE_DOCKING   = "DOCKING";
const std::string SPS_STATE_ERROR     = "ERROR";


const std::string SPS_STATE_IDLE_BT_XML           = "/bt_resource/idle_state_bt.xml";
const std::string SPS_STATE_WORKING_BT_XML        = "/bt_resource/working_state_bt.xml";
const std::string SPS_STATE_WORKING_NAVI_BT_XML   = "/bt_resource/working_state_navi_bt.xml";
const std::string SPS_STATE_DOCKING_BT_XML        = "/bt_resource/docking_state_bt.xml";
const std::string SPS_STATE_ERROR_BT_XML          = "/bt_resource/error_state_bt.xml";
const std::string SPS_STATE_DEFAULT_BT_XML        = "/bt_resource/robot_naviengine_bt.xml";
typedef enum
{
    SPS_INIT = 0,
    SPS_IDLE,
    SPS_WORKING,
    SPS_DOCKING,
    SPS_ERROR,
    SPS_MAX
}SPSSTATE;





typedef enum
{
    EVENT_INIT_SUCCESS = 0,
    EVENT_MONITOR_ERROR,
    EVENT_IN_DOCKING,
    EVENT_DOCKING_ERROR,
    EVENT_UNDOCKING_SUCCESS,
    EVENT_START_TASK,
    EVENT_TASK_FINISHED,
    EVENT_TASK_ERROR,
    EVENT_CLERA_ERROR,
    EVENT_MAX,
}EVENTID;



const int EVENT_RANGE = EVENT_MAX;
const int STATE_RANGE = SPS_MAX;

struct StateTransGraph
{
    SPSSTATE current_state; 
    EVENTID input_event;
    SPSSTATE next_state;
};


class State
{
public:
    State();
    State* transition[EVENT_RANGE];
    StateTransGraph* graph;
};


class Fsm
{
public:
    Fsm();
    virtual ~Fsm();
    void Init();
    bool StateTransform(EVENTID event);
    bool IdleState();
    bool WorkingState();
    bool DockingState();
    bool ErrorState();
    std::string IdleStateXml();
    std::string WorkingStateXml();
    std::string WorkingStateNaviXml();
    std::string DockingStateXml();
    std::string ErrorStateXml();
    std::string DefaultXml();
    SPSSTATE GetState();

private:
    bool DoomState();
    std::string GetPkgDir();
    State* p_current;
    State* p_state;
    SPSSTATE global_state;
};

}

#endif
