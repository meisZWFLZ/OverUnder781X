#include "lemlib/api.hpp"
namespace fieldDimensions {
  enum HEADING {
    LEFT = 270,
    RIGHT = 90,
    UP = 0,
    DOWN = 180
  };

  constexpr int MAX_X = 72;
  constexpr int MIN_X = -72;
  constexpr int MAX_Y = 72;
  constexpr int MIN_Y = -72;

  constexpr int TILE_LENGTH = 24;
  constexpr int TILE_RADIUS = TILE_LENGTH / 2;
  // @TODO
  lemlib::Pose nextToMatchLoadZone{};
}