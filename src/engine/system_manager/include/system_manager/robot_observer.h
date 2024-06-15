

#ifndef _ROBOT_OBSERVER_H_
#define _ROBOT_OBSERVER_H_

#include "define.h"

namespace naviengine
{

class IRobotObserver
{
public:
    virtual ERESULT OnInitDone(bool success) = 0;
    virtual ERESULT OnUpdateNaviState(const NaviStateInfo& state) = 0;
    virtual ERESULT OnUpdateNaviPath(const NaviPathInfo& path) = 0;
    virtual ERESULT OnUpdateRetentionStatus(const bool &is_tapped, const std::string& task_id) = 0;
    virtual ERESULT OnNaviState(const std::string& task_id, ENAVITYPE type, NAVISTATE state) = 0;
};

}  // namespace naviengine

#endif
