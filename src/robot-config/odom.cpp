#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/timer.hpp"
#include "pros/adi.h"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "robot.h"
#include <cmath>
#include <cstdint>
#include <vector>

lemlib::OdomSensors* Robot::odomSensors = nullptr;
lemlib::Chassis* Robot::chassis = nullptr;

class HeadingSource {
  public:
    virtual double getHeading() const = 0;
    virtual bool isMessedUp() const = 0;
    virtual bool calibrate() const = 0;
    virtual bool isDoneCalibrating() const = 0;
};

class IMUHeadingSource : public HeadingSource {
  public:
    IMUHeadingSource(pros::IMU* imu) : imu(imu) {}

    double getHeading() const override { return imu->get_rotation(); }

    bool isMessedUp() const override { return imu->get_rotation() == PROS_ERR; }

    bool calibrate() const override { return imu->reset(false); }

    bool isDoneCalibrating() const override { return !imu->is_calibrating(); }
  private:
    pros::IMU* imu;
};

class TrackingWheelHeadingSource : public HeadingSource {
  public:
    TrackingWheelHeadingSource(std::vector<lemlib::TrackingWheel*> wheels)
      : wheels(wheels), prevDistanceTraveled(wheels.size(), 0) {
      this->offsets = {};
      for (int i = 0; i < wheels.size(); i++) {
        this->offsets.push_back(wheels[i]->getOffset());
      }
    }

    double getHeading() const override {
      std::vector<float> deltaDistanceTraveled;
      for (int i = 0; i < wheels.size(); i++) {
        wheels[i]->getDistanceTraveled() - prevDistanceTraveled[i];
      }

      return 1;
    }

    bool isMessedUp() const override { return false; }
  private:
    std::vector<float> offsets;
    std::vector<lemlib::TrackingWheel*> wheels;
    std::vector<float> prevDistanceTraveled;
};

// only make one of these!!!
class MockIMU : public pros::IMU {
  public:
    MockIMU(std::vector<HeadingSource*> sources)
      : pros::IMU(0), sources(sources) {}

    double get_rotation() const override {
      static std::vector<double> prevHeadings(this->sources.size(), 0);
      static double prevReturnedHeading = 0;
      // return the average of the closest sources that are not messed up
      std::vector<double> deltaHeadings;
      std::vector<double> newHeadings;

      for (int i = 0; i < sources.size(); i++) {
        if (!sources[i]->isMessedUp()) {
          newHeadings.push_back(sources[i]->getHeading());
          deltaHeadings.push_back(sources[i]->getHeading() - prevHeadings[i]);
        } else {
          newHeadings.push_back(prevHeadings[i]);
        }
      }
      double out = 0;

      if (deltaHeadings.size() == 0) {
        if (pros::millis() % 100 < 10) printf("0\n");
        out = prevReturnedHeading;
      } else if (deltaHeadings.size() == 1) {
        if (pros::millis() % 100 < 10) printf("1\n");
        out = prevReturnedHeading + deltaHeadings[0];
      } else if (deltaHeadings.size() == 2) {
        if (pros::millis() % 100 < 10) printf("2\n");
        out = prevReturnedHeading + (deltaHeadings[0] + deltaHeadings[1]) / 2;
      } else {
        std::sort(deltaHeadings.begin(), deltaHeadings.end());
        std::vector<double> headingDiffs;
        for (int i = 0; i < deltaHeadings.size() - 1; i++) {
          headingDiffs.push_back(deltaHeadings[i + 1] - deltaHeadings[i]);
        }
        double smallestDiff = headingDiffs[0];
        int smallestDiffIndex = 0;
        for (int i = 1; i < headingDiffs.size(); i++) {
          if (headingDiffs[i] < smallestDiff) {
            smallestDiff = headingDiffs[i];
            smallestDiffIndex = i;
          }
        }
        if (pros::millis() % 100 < 10)
          printf("3,delta: %f\n", deltaHeadings[smallestDiffIndex]);

        out = prevReturnedHeading + (deltaHeadings[smallestDiffIndex] +
                                     deltaHeadings[smallestDiffIndex + 1]) /
                                  
                                  
                                  
                                  
                                        2;
      }
      newHeadings.swap(prevHeadings);
      prevReturnedHeading = out;
      return out;
    }

    int32_t reset(bool idk) const override { return 1; }

    bool calibrate() {
      enum CALIBRATION_STATE { NOT_CALIBRATING, CALIBRATING, DONE_CALIBRATING };

      std::vector<CALIBRATION_STATE> calibrationStates(this->sources.size(),
                                                       NOT_CALIBRATING);
      int doneCount = 0;
      lemlib::Timer calibrateTimer = 3000;
      while (doneCount < sources.size() && !calibrateTimer.isDone()) {
        for (int i = 0; i < sources.size(); i++) switch (calibrationStates[i]) {
            case NOT_CALIBRATING:
              if (sources[i]->calibrate()) {
                calibrationStates[i] = CALIBRATING;
              }
              break;
            case CALIBRATING:
              if (sources[i]->isDoneCalibrating()) {
                calibrationStates[i] = DONE_CALIBRATING;
                doneCount++;
              }
              break;
            case DONE_CALIBRATING: break;
          }
        pros::delay(10);
      }
      if (doneCount == 0) return false;
      else if (doneCount == sources.size()) return true;
      else {
        // if only some sources are done calibrating, return true and only use
        // those
        std::vector<HeadingSource*> calibratedSources;
        for (int i = 0; i < sources.size(); i++)
          if (calibrationStates[i] == DONE_CALIBRATING)
            calibratedSources.push_back(sources[i]);
        sources.swap(calibratedSources);
        return true;
      }
    }
  private:
    std::vector<HeadingSource*> sources;
};

void Robot::initializeOdometryConfig() {
  lemlib::Drivetrain drivetrain {
      &Robot::Motors::leftDrive,        &Robot::Motors::rightDrive,
      Robot::Dimensions::trackWidth,    Robot::Dimensions::driveWheelDiameter,
      Robot::Dimensions::driveWheelRpm, Robot::Tunables::chasePower};

  lemlib::TrackingWheel* leftVert =
      Robot::Sensors::vert.get_angle() != PROS_ERR
          ? new lemlib::TrackingWheel(
                &Robot::Sensors::vert, Robot::Dimensions::vertEncDiameter,
                Robot::Dimensions::vertEncDistance,
                Robot::Dimensions::vertEncGearRatio /* 300 */ /* 1 */)
          : nullptr;

  lemlib::TrackingWheel* rightVert = nullptr;
  lemlib::TrackingWheel* hori =
      (&Robot::Sensors::hori)->get_angle() != PROS_ERR
          ? new lemlib::TrackingWheel(&Robot::Sensors::hori,
                                      Robot::Dimensions::horiEncDiameter,
                                      Robot::Dimensions::horiEncDistance,
                                      Robot::Dimensions::horiEncGearRatio)
          : nullptr;
  auto goofyIMU = new MockIMU({new IMUHeadingSource(&Robot::Sensors::imuA),
                               new IMUHeadingSource(&Robot::Sensors::imuB),
                               new IMUHeadingSource(&Robot::Sensors::imuC)});
  goofyIMU->calibrate();
  Robot::odomSensors = new lemlib::OdomSensors {
      leftVert, rightVert /* nullptr */, hori, nullptr, goofyIMU};

  Robot::chassis =
      new lemlib::Chassis {drivetrain, Robot::Tunables::lateralController,
                           Robot::Tunables::angularController, *odomSensors};
}