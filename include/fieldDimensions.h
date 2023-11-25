#pragma once
#include "lemlib/api.hpp"
#include "robot.h"

namespace fieldDimensions {
const constexpr Angle LEFT = 270_deg;
const constexpr Angle RIGHT = 90_deg;
const constexpr Angle UP = 0_deg;
const constexpr Angle DOWN = 180_deg;

const constexpr Length MAX_X = 72_in;
const constexpr Length MIN_X = -72_in;
const constexpr Length MAX_Y = 72_in;
const constexpr Length MIN_Y = -72_in;

const constexpr Length TILE_LENGTH = 24_in;
const constexpr Length TILE_RADIUS = TILE_LENGTH / 2;
// @TODO
const inline lemlib::Pose leftStartingPose {
    (MIN_X + TILE_LENGTH + Robot::Dimensions::drivetrainWidth / 2),
    MIN_Y + Robot::Dimensions::drivetrainLength / 2};
const inline lemlib::Pose rightStartingPose = {-leftStartingPose.x,
                                               leftStartingPose.y};
} // namespace fieldDimensions