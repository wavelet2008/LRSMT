/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 20:20:11 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-27 21:23:52
 */

#include "controller/controller_monitor.h"

namespace agiletaur{
namespace control{
void ControllerMonitor::Init(){
    pid_controller_ptr_ = new PIDcontroller();
    pid_controller_ptr_->Init();
}
} //name space control
} //name space agiletaur