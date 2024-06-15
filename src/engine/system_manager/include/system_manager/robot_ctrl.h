

#ifndef _ROBOT_CTRL_H_
#define _ROBOT_CTRL_H_

#include <string>

#include "define.h"

namespace naviengine
{

class IEngineCtrl
{
public:
    virtual ERESULT OnUpdateFsmState(NAVISTATE state, const std::string &task_id) = 0;
};

}  // namespace naviengine

#endif
