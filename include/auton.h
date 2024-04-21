#pragma once
#include "lemlib/api.hpp"
#include "selector.h"
#include <climits>
#include <optional>
#include <unordered_map>
#include <vector>

namespace auton {

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
/**
 * @brief Matchloads triballs
 * @param triballs number of triballs to shoot
 * @param until timestamp to run until
 */
void matchload(int triballs= 46, int until = pros::millis() + 35000);
} // namespace actions

namespace utils {
const float DEFAULT_SLEW = 15;

/**
 * @returns -1 when robot starts on left side/defensive zone and +1 on right
 * side/offensive zone
 */
int leftOrRight(int ifLeft = -1, int ifRight = 1);

/**
 * @brief Tank drive with slew rate control
 * Slew rate control is a method of limiting the rate of change of the voltage
 * sent to the motors. This is useful for ensuring that the tracking wheels
 * always contact the ground.
 *
 * @param left left side power
 * @param right right side power
 * @param ms time to run for
 * @param slew slew rate
 */
void tank(float left, float right, int ms, float slew = DEFAULT_SLEW);

/**
 * @brief Stops sending voltage to the drivetrain motors
 */
void stop();

/**
 * @brief Wait until the robot is within a circle with a radius of error and
 * centered at pose for time milliseconds.
 *
 * @param pose pose to find distance from robot to
 * @param error
 * @param time
 */
void waitUntilDistToPose(lemlib::Pose pose, float error, int time = 0,
                         bool checkMotionRunning = false);

/**
 * @brief Gets the distance between the robot's heading and a target heading
 *
 * @param ang target heading in degrees
 * @return a positive float with range of [0,180]
 */
float robotAngDist(float target);

bool isTriballInIntake();
bool isMotionRunning();

void waitUntil(std::function<bool(void)> condition, int timeConditionIsTrue = 0,
               int timeout = INT_MAX, bool resetTrueStartTime = false);

} // namespace utils

namespace autons {
extern Auton defensive;
extern Auton fiveRush;
extern Auton sixRushElims;
extern Auton sixBall;
extern Auton skills;
extern Auton disrupt;
extern Auton doNothing;
} // namespace autons
} // namespace auton

template <typename T> class Event {
  public:
    virtual std::optional<T> isTriggered() = 0;
};

class IntEvent : Event<int> {
  public:
    std::optional<int> isTriggered() override;
};

template <typename T> class EventListener {
  public:
    virtual void onEvent(T event) = 0;
};

class IntEventListener : EventListener<int> {
  public:
    void onEvent(int event) override;
};

/* template <typename EventKey, typename EventMessage> */ class EventHandler {
  private:
    // how can I bind the event key to the event type?
    std::unordered_map<int, IntEvent> events;
    std::unordered_map<int, std::vector<IntEventListener>> listeners;
  public:
    void update() { 
      for (auto [key, event] : events) {
        auto triggered = event.isTriggered();
        if (triggered.has_value()) {
          for (auto listener : listeners[key]) {
            listener.onEvent(triggered.value());
          }
        }
      }
    }
};