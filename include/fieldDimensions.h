#pragma once
#include "lemlib/api.hpp"
#include "robot.h"

namespace fieldDimensions {
enum HEADING { LEFT = 270, RIGHT = 90, UP = 0, DOWN = 180 };

constexpr int MAX_X = 72;
constexpr int MIN_X = -72;
constexpr int MAX_Y = 72;
constexpr int MIN_Y = -72;

constexpr int TILE_LENGTH = 24;
constexpr int TILE_RADIUS = TILE_LENGTH / 2;
// @TODO
const inline lemlib::Pose leftStartingPose {
    (MIN_X + TILE_LENGTH + Robot::Dimensions::drivetrainWidth / 2),
    MIN_Y + Robot::Dimensions::drivetrainLength / 2};
const inline lemlib::Pose rightStartingPose = {-leftStartingPose.x,
                                               leftStartingPose.y};
const inline lemlib::Pose skillsStartingPose = {-48.1757, -55.4117, -202.2711};
} // namespace fieldDimensions