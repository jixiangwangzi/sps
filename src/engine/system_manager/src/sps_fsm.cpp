#include "rclcpp/rclcpp.hpp"
#include "system_manager/sps_fsm.hpp"

namespace naviengine
{

std::map<SPSSTATE, std::string> E_SPS_STATE_ENUM_STRING_MAPPING{
    {SPS_INIT,      SPS_STATE_INIT},
    {SPS_IDLE,      SPS_STATE_IDLE},
    {SPS_WORKING,   SPS_STATE_WORKING},
    {SPS_DOCKING,   SPS_STATE_DOCKING},
    {SPS_ERROR,     SPS_STATE_ERROR},
};



StateTransGraph STATE_GRAPH[] = 
{
    {SPS_INIT, EVENT_INIT_SUCCESS, SPS_IDLE}, {SPS_INIT, EVENT_MONITOR_ERROR, SPS_ERROR}, 
    {SPS_IDLE, EVENT_START_TASK, SPS_WORKING}, {SPS_IDLE, EVENT_IN_DOCKING, SPS_DOCKING},{SPS_IDLE, EVENT_MONITOR_ERROR, SPS_ERROR},
    {SPS_WORKING, EVENT_TASK_FINISHED, SPS_IDLE}, {SPS_WORKING, EVENT_TASK_ERROR, SPS_ERROR}, {SPS_WORKING, EVENT_START_TASK, SPS_WORKING},
    {SPS_DOCKING, EVENT_UNDOCKING_SUCCESS, SPS_IDLE}, {SPS_DOCKING, EVENT_MONITOR_ERROR, SPS_ERROR},{SPS_DOCKING, EVENT_DOCKING_ERROR, SPS_ERROR},
    {SPS_ERROR, EVENT_CLERA_ERROR, SPS_IDLE},
    {SPS_MAX, EVENT_MAX, SPS_MAX},
};


State::State()
{
    for (int i = 0; i < EVENT_RANGE; ++i)
        transition[i] = NULL;
}

Fsm::Fsm()
{


    p_state = new State[STATE_RANGE];
    for (StateTransGraph* p_temp = STATE_GRAPH; p_temp->current_state != SPS_MAX; ++p_temp)
    {
        p_state[p_temp->current_state].transition[p_temp->input_event] = &p_state[p_temp->next_state];
        p_state[p_temp->current_state].graph = p_temp;
    }


    p_current = NULL;
}

Fsm::~Fsm()
{
    delete []p_state;
}

void Fsm::Init()
{
    p_current = &p_state[SPS_INIT];
    global_state = p_current->graph->current_state;

}

bool Fsm::StateTransform(EVENTID event)
{
    if (p_current != NULL)
    {
        ROS_INFO("StateTransform  current_state = %s.",E_SPS_STATE_ENUM_STRING_MAPPING[p_current->graph->current_state].c_str());
        if(p_current->transition[event] != NULL)
        {
            p_current = p_current->transition[event];
            global_state = p_current->graph->current_state;
            ROS_INFO("StateTransform  next_state = %s.",E_SPS_STATE_ENUM_STRING_MAPPING[global_state].c_str());
            return true;
            
        } else {
            ROS_INFO("Invalid event id:%d.", event);
        }
        ROS_INFO("StateTransform  next_state = %s.",E_SPS_STATE_ENUM_STRING_MAPPING[global_state].c_str());
   
    } else {
        ROS_ERROR("Invalid State");
    }
     return false;
}


bool Fsm::IdleState()
{
    return !DoomState() && (global_state == SPS_IDLE);
}

bool Fsm::WorkingState()
{
    return !DoomState() && (global_state == SPS_WORKING);
}

bool Fsm::DockingState()
{
    return !DoomState() && (global_state == SPS_DOCKING);
}


bool Fsm::ErrorState()
{
    return !DoomState() && (global_state == SPS_ERROR);
}


bool Fsm::DoomState()
{
    return p_current == NULL;
}


std::string Fsm::IdleStateXml()
{
    return GetPkgDir() + SPS_STATE_IDLE_BT_XML;
}

std::string Fsm::WorkingStateXml()
{
    return GetPkgDir() + SPS_STATE_WORKING_BT_XML;
}

std::string Fsm::WorkingStateNaviXml()
{
    return GetPkgDir() + SPS_STATE_WORKING_NAVI_BT_XML;
}

std::string Fsm::DockingStateXml()
{
    return GetPkgDir() + SPS_STATE_DOCKING_BT_XML;
}


std::string Fsm::ErrorStateXml()
{
    return GetPkgDir() + SPS_STATE_ERROR_BT_XML;
}

std::string Fsm::DefaultXml()
{
    return GetPkgDir() + SPS_STATE_DEFAULT_BT_XML;
}


std::string Fsm::GetPkgDir()
{
    return ament_index_cpp::get_package_share_directory("system_manager");
}



SPSSTATE Fsm::GetState()
{
    //RCLCPP_INFO(rclcpp::get_logger("FSM"), "GetState  global_state = %d.",global_state);
    return global_state;
}

}



