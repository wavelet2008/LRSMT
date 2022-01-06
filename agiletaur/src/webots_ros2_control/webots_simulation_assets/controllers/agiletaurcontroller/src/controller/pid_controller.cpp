/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 15:47:05 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-27 21:36:34
 */


#include "controller/pid_controller.h"
#include <cmath>
const double PI = 3.141592653589793238463;

using namespace std;

namespace agiletaur{
namespace control{

    
void PIDcontroller::Init(){
}

void PIDcontroller::ComputeControlCommand(){

}
void PIDcontroller::Reset() { is_init = false; }

void PIDcontroller::Stop(){
}


} // namespace control
} // namespace agiletaur
