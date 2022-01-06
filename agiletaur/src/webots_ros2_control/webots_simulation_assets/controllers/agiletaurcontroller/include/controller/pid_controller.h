/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 15:44:32 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-27 21:37:10
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include "controller/basecontroller.h"


namespace agiletaur{
namespace control{
class PIDcontroller : public BaseController
{
public:
    PIDcontroller() = default;

    virtual ~PIDcontroller() = default;

    void Init() override;

    void ComputeControlCommand() override;

    void Reset() override;

    void Stop() override;

private:
    bool is_init = false;

};
} // namespace control
} // namespace agiletaur

#endif