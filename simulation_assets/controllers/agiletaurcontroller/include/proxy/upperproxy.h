/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 16:00:26 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-27 21:23:15
 */

#ifndef UPPER_PROXY_H_
#define UPPER_PROXY_H_

// #include "gait_math.h"
// #include "adrc.h"
// #include "convexMPC_interface.h"
// #include "common_types.h"
// #include "SolverMPC.h"
// #include "cppTypes.h"



namespace agiletaur{
namespace control{

class upperproxy{
  public:
    static upperproxy& GetUpperProxy() {
        static upperproxy singleton;
        return singleton;
    }
    void Init();
  private:
    upperproxy() = default;
    virtual ~upperproxy() = default;
    
};

} //namespace control
} //namespace agiletaur



#endif