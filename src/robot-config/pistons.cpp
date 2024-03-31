#include "robot.h"
const ADIPortConfig frontLeft('H');
const ADIPortConfig frontRight('A');
const ADIPortConfig backLeft('G');
const ADIPortConfig backRight('G');

const FourWingSubsystem::PortConfig& Robot::Pistons::wingConfig {
    .front = {frontLeft, frontRight},
    .back = {backLeft, backRight}};

pros::ADIDigitalOut Robot::Pistons::retractLift {'B'};
pros::ADIDigitalOut Robot::Pistons::extendLift {'F'};

pros::ADIDigitalOut Robot::Pistons::tankSplitter {'E'};