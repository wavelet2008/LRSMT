/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-28 21:24:55 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-28 21:45:19
 */
#include "config/config_parser.h"

namespace agiletaur {
namespace control {

  
int ControlConfig::LoadParams() {
  Config cfg;
  // Read the file. If there is an error, report it and exit.
  try
  {
    cfg.readFile("agiletaur.cfg");
  }
  catch(const FileIOException &fioex)
  {
    std::cerr << "I/O error while reading file." << std::endl;
    return(EXIT_FAILURE);
  }
  catch(const ParseException &pex)
  {
    std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
              << " - " << pex.getError() << std::endl;
    return(EXIT_FAILURE);
  }
  // Get the configure file name.
  string name = cfg.lookup("NAME");
  cout << "Read Configure File From: " << name << endl;

  /* Output a list of all books in the inventory. */
  cfg.lookupValue("ROBOT.BODY_WEIGHT", robot_config_.robot_base.BODY_WEIGHT);
  cfg.lookupValue("ROBOT.UPPER_LEG_LENGTH", 
                                  robot_config_.robot_base.UPPER_LEG_LENGTH);
  cfg.lookupValue("ROBOT.LOWER_LEG_LENGTH", 
                                  robot_config_.robot_base.LOWER_LEG_LENGTH);
  cfg.lookupValue("ROBOT.KP_LF", robot_config_.robot_controller.KP_LF);
  cfg.lookupValue("ROBOT.KD_LF", robot_config_.robot_controller.KD_LF);
  cfg.lookupValue("ROBOT.KI_LF", robot_config_.robot_controller.KD_LF);
  cfg.lookupValue("ROBOT.KP_LB", robot_config_.robot_controller.KP_LB);
  cfg.lookupValue("ROBOT.KD_LB", robot_config_.robot_controller.KD_LB);
  cfg.lookupValue("ROBOT.KI_LB", robot_config_.robot_controller.KI_LB);
  cfg.lookupValue("ROBOT.KP_RF", robot_config_.robot_controller.KP_RF);
  cfg.lookupValue("ROBOT.KD_RF", robot_config_.robot_controller.KD_RF);
  cfg.lookupValue("ROBOT.KI_RF", robot_config_.robot_controller.KI_RF);
  cfg.lookupValue("ROBOT.KP_RB", robot_config_.robot_controller.KP_RB);
  cfg.lookupValue("ROBOT.KD_RB", robot_config_.robot_controller.KD_RB);
  cfg.lookupValue("ROBOT.KI_RB", robot_config_.robot_controller.KI_RB);
  cout << "Load all parameters to robot_config_ struct" << endl;
  return(EXIT_SUCCESS);
}
}  // namespace control
}  // namespace agiletaur


