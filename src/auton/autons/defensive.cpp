#include "auton.h"
#include "robot.h"
#include "fieldDimensions.h"

using namespace fieldDimensions;

void runDefensive() {
  using namespace fieldDimensions;
  using namespace auton::actions;
  Robot::chassis->setPose(leftStartingPose, false);
  // @todo add defensive auton

  pushMatchLoadZoneTriball();
  scoreAllianceTriball();
  intakeTriball(
      {-TILE_LENGTH - 2, 0 - Robot::Dimensions::drivetrainLength / 2 - 0.85,
       UP});
  shootTriballIntoOffensiveZone();
  touchElevationBar();
}

auton::Auton auton::autons::defensive = {(char*)("defensive / left"),
                                         runDefensive};