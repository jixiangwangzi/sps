

#ifndef _ROBOT_ENGINE_OBSERVER_H_
#define _ROBOT_ENGINE_OBSERVER_H_

#include <string>

#include "define.h"

namespace naviengine
{

class IRobotEngineObserver
{
public:
    virtual ERESULT OnUpdateNaviState(const NaviStateInfo navi_state) = 0;
    virtual ERESULT OnUpdateNaviPath(const NaviPathInfo navi_path) = 0;
    virtual ERESULT OnUpdateRetentionStatus(const bool &is_tapped, const std::string& task_id) = 0;
    virtual ERESULT OnNaviState(const std::string& task_id, ENAVITYPE type, NAVISTATE state) = 0;
};

}  // namespace naviengine

#endif
