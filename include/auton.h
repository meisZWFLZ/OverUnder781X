#include "lemlib/api.hpp"

namespace auton {
 /** 
  * @returns -1 when robot starts on left side/defensive zone and +1 on right side/offensive zone   
  */
int leftOrRight(int ifLeft = -1, int ifRight = 1);

namespace actions {
void removeMatchLoad();
/**
 * @brief pushes triball in match load zone to offensive zone 
 * @zone DEFENSIVE ZONE
 * @order MUST BE FIRST ACTION
 */
void pushMatchLoadZoneTriball();
void scoreAllianceTriball();
void touchElevationBar();
void shootTriballIntoOffensiveZone();
void intakeTriball(lemlib::Pose pose);
} // namespace actions
} // namespace auton