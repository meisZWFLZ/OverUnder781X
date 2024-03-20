#include "robot.h"
const ADIPortConfig frontLeft('B');
const ADIPortConfig frontRight('C');
const ADIPortConfig backLeft('D');
const ADIPortConfig backRight('E');

const FourWingSubsystem::PortConfig& Robot::Pistons::wingConfig {
    .front = {frontLeft, frontRight},
    .back = {backLeft, backRight}};

pros::ADIDigitalOut Robot::Pistons::retractLift {'G'};
pros::ADIDigitalOut Robot::Pistons::extendLift {'A'};