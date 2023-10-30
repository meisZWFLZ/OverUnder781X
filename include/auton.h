namespace auton {
 /** 
  * @returns -1 when robot starts on left side/defensive zone and +1 on right side/offensive zone   
  */
int leftOrRight(int ifLeft = -1, int ifRight = 1);

namespace actions {
void removeMatchLoad();
void scoreAllianceTriball();
void touchElevationBar();
void shootTriballIntoOffensiveZone();
} // namespace actions
} // namespace auton