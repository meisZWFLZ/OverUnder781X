#include "lemlib/util.hpp"
#include "pros/error.h"
#include "robot.h"
#include <cmath>
#include <sstream>

lemlib::OdomSensors* Robot::odomSensors = nullptr;
lemlib::Chassis* Robot::chassis = nullptr;

class HeadingSource {
  public:
    // returns the heading of the source
    /** returns NaN if messed up */
    virtual double getHeading() const = 0;
    virtual bool calibrate() = 0;
    virtual bool isDoneCalibrating() const = 0;
};

class IMUHeadingSource : public HeadingSource {
  public:
    IMUHeadingSource(pros::IMU* imu, const float coefficient = 1)
      : imu(imu), coefficient(coefficient) {}

    double getHeading() const override {
      if (imu->get_rotation() == PROS_ERR_F) return NAN;
      return imu->get_rotation() * coefficient;
    }

    bool calibrate() override { return imu->reset(false); }

    bool isDoneCalibrating() const override { return !imu->is_calibrating(); }

    /**
     * @brief adjusts imu output to account for linear drift
     */
    float coefficient;
  private:
    pros::IMU* imu;
};

class TrackingWheelHeadingSource : public HeadingSource {
  public:
    TrackingWheelHeadingSource(std::vector<lemlib::TrackingWheel*> _wheels)
      : wheels(_wheels) {
      // sort the wheels such that unpowered wheels are before powered wheels
      std::sort(this->wheels.begin(), this->wheels.end(),
                [](lemlib::TrackingWheel* a, lemlib::TrackingWheel* b) {
                  return a->getType() < b->getType();
                });
      this->offsets = {};
      for (int i = 0; i < this->wheels.size(); i++) {
        this->offsets.push_back(this->wheels[i]->getOffset());
      }
    }

    double getHeading() const override {
      struct TrackingWheelData {
          float deltaDist, offset;
      };

      static std::vector<float> prevDistanceTraveled(wheels.size(), 0);
      static float prevReturnedHeading(0);

      std::vector<TrackingWheelData> deltaDistsAndOffsets;
      std::vector<float> newDistanceTraveled;
      for (int i = 0; i < wheels.size(); i++) {
        const float dist = wheels[i]->getDistanceTraveled();
        if (dist != PROS_ERR_F) {
          deltaDistsAndOffsets.push_back(
              {wheels[i]->getDistanceTraveled() - prevDistanceTraveled[i],
               offsets[i]});
          newDistanceTraveled.push_back(wheels[i]->getDistanceTraveled());
        } else {
          newDistanceTraveled.push_back(prevDistanceTraveled[i]);
        }
      }
      float out = prevReturnedHeading;
      if (deltaDistsAndOffsets.size() < 2) out = NAN;
      else {
        const auto first = deltaDistsAndOffsets[0];
        const auto second = deltaDistsAndOffsets[1];

        out -= lemlib::radToDeg((first.deltaDist - second.deltaDist) /
               (first.offset - second.offset));
      }
      newDistanceTraveled.swap(prevDistanceTraveled);
      if (out != NAN) prevReturnedHeading = out;
      return out;
    }

    bool calibrate() override {
      for (auto wheel : wheels) { wheel->reset(); }
      return true;
    }

    bool isDoneCalibrating() const override { return true; }
  private:
    std::vector<float> offsets;
    std::vector<lemlib::TrackingWheel*> wheels;
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
      std::stringstream printSS;
      static const char* telemetryPrefix = "!:";
      printSS << telemetryPrefix;
      for (int i = 0; i < sources.size(); i++) {
        char buffer[10];
        const float heading = sources[i]->getHeading();
        if (heading != NAN) {
          newHeadings.push_back(heading);
          deltaHeadings.push_back(heading - prevHeadings[i]);
          sprintf(buffer, "%0.4f", heading - prevHeadings[i]);
        } else {
          newHeadings.push_back(prevHeadings[i]);
          sprintf(buffer, "NAN");
        }
        printSS << buffer << ",";
      }
      double out = 0;

      if (deltaHeadings.size() == 0) {
        out = prevReturnedHeading;
      } else if (deltaHeadings.size() == 1) {
        out = prevReturnedHeading + deltaHeadings[0];
      } else if (deltaHeadings.size() == 2) {
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

        out = prevReturnedHeading + (deltaHeadings[smallestDiffIndex] +
                                     deltaHeadings[smallestDiffIndex + 1]) /
                                        2;
      }
      printSS << out << "\n";
      printf("%s", printSS.str().c_str());

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

void Robot::initializeOdometry() {
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

  lemlib::TrackingWheel* leftDriveTracker = new lemlib::TrackingWheel(
      &Robot::Motors::leftDrive, Robot::Dimensions::driveWheelDiameter,
      -Robot::Dimensions::trackWidth / 2, Robot::Dimensions::driveWheelRpm);
  lemlib::TrackingWheel* rightDriveTracker = new lemlib::TrackingWheel(
      &Robot::Motors::rightDrive, Robot::Dimensions::driveWheelDiameter,
      Robot::Dimensions::trackWidth / 2, Robot::Dimensions::driveWheelRpm);

  auto trackingWheelHeading = new TrackingWheelHeadingSource(
      {leftDriveTracker, rightDriveTracker, leftVert});
  auto goofyIMU = new MockIMU({new IMUHeadingSource(&Robot::Sensors::imuA),
                               new IMUHeadingSource(&Robot::Sensors::imuB),
                               new IMUHeadingSource(&Robot::Sensors::imuC),
                               trackingWheelHeading});
  goofyIMU->calibrate();
  Robot::odomSensors = new lemlib::OdomSensors {
      leftVert, rightVert /* nullptr */, hori, nullptr, goofyIMU};

  Robot::chassis =
      new lemlib::Chassis {drivetrain, Robot::Tunables::lateralController,
                           Robot::Tunables::angularController, *odomSensors};
}