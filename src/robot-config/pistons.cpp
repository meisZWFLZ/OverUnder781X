#include "robot.h"

std::unique_ptr<FourWingSubsystem::PortConfig>
    Robot::Pistons::wingConfig(new FourWingSubsystem::PortConfig {
        .front = std::make_pair(std::make_unique<ADIPortConfig>('B'),
                                std::make_unique<ADIPortConfig>('C')),
        .back = std::make_pair(std::make_unique<ADIPortConfig>('D'),
                               std::make_unique<ADIPortConfig>('E'))});

pros::ADIDigitalOut Robot::Pistons::retractLift {'G'};
pros::ADIDigitalOut Robot::Pistons::extendLift {'A'};