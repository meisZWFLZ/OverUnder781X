#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void runDefensive() {
  using namespace fieldDimensions;
  using namespace auton::actions;
  Robot::chassis->setPose(leftStartingPose);
  // @todo add defensive auton

  pushMatchLoadZoneTriball();
  // scoreAllianceTriball();
  intakeTriball(
      {-TILE_LENGTH - 2_in, 0_in - Robot::Dimensions::drivetrainLength / 2 - 6.25_in,
       UP});
  shootTriballIntoOffensiveZone();
  touchElevationBar();
}

auton::Auton auton::autons::defensive = {(char*)("defensive / left"),
                                         runDefensive};