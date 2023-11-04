#pragma once
#include "selector.h"
namespace auton {
 /** 
  * @returns -1 when robot starts on left side/defensive zone and +1 on right side/offensive zone   
  */
int leftOrRight();

namespace actions {
void removeMatchLoad();
void scoreAllianceTriball();
void touchElevationBar();
void shootTriballIntoOffensiveZone();
} // namespace actions

namespace autons {
  extern Auton defensive;
  extern Auton offensive;
  extern Auton skills;
}
} // namespace auton