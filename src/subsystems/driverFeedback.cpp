#include "driverFeedback.h"
#include "pros/rtos.hpp"
#include "robot.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>

constexpr int MIN_TIME_BETWEEN_UPDATES = 500;

Device::Device(pros::Motor* motor)
  : isConnected([motor] {
      errno = 0;
      if (motor->get_position() == PROS_ERR_F && errno == ENODEV) return false;
      return true;
    }),
    getTemp([motor] { return motor->get_temperature(); }) {};
Device::Device(pros::Rotation* rot)
  : Device([rot] {
      errno = 0;
      if (rot->get_position() == PROS_ERR && errno == ENODEV) return false;
      return true;
    }) {};
Device::Device(pros::IMU* imu)
  : Device([imu] {
      errno = 0;
      if (imu->get_heading() == PROS_ERR_F && errno == ENODEV) return false;
      return true;
    }) {};
Device::Device(pros::ADILineSensor* lineSensor)
  : Device([lineSensor] {
      errno = 0;
      if (lineSensor->get_value() == PROS_ERR && errno == ENODEV) return false;
      return true;
    }) {};

float Device::hasNoTemp() { return NAN; }

DriverFeedback::DriverFeedback() { initializeDevices(); }

void DriverFeedback::addMotorGroup(const char* name, pros::Motor_Group* group) {
  for (int i = 0; i < group->size(); i++) {
    addDevice(name, new Device(&group->at(i)));
  }
}

void DriverFeedback::addDevice(const char* name, Device* device) {
  this->devices.push_back({name, device});
}

void DriverFeedback::initializeDevices() {
  this->devices = {};
  addMotorGroup("RDri", &Robot::Motors::rightDrive);
  addMotorGroup("LDri", &Robot::Motors::leftDrive);
  addMotorGroup("Lift", &Robot::Motors::elevator);
  addMotorGroup("Cata", &Robot::Motors::catapult);
  addMotorGroup("Int", &Robot::Motors::intake);
  addDevice("VPod", new Device(&Robot::Sensors::vert));
  addDevice("HPod", new Device(&Robot::Sensors::hori));
  addDevice("CRot", new Device(&Robot::Sensors::cata));
  addDevice("CLin", new Device(&Robot::Sensors::cataTriball));
  addDevice("IMU", new Device(&Robot::Sensors::imu));
}

void DriverFeedback::update() {
  std::unordered_map<const char*, int> disconnectedDevices = {};
  for (auto device : this->devices) {
    if (!device.second->isConnected()) { disconnectedDevices[device.first]++; }
  }
  std::string msg = "";
  static const char* DELIM = ",";
  bool disconnected = !disconnectedDevices.empty();
  if (disconnected) {
    msg += "!";
    for (auto dev = disconnectedDevices.begin();
         dev != disconnectedDevices.end() && msg.length() < 14;
         dev++, msg += DELIM) {
      msg += dev->first;
      if (dev->second > 1) msg += "x" + std::to_string(dev->second);
    }
  } else {
    std::unordered_map<const char*, float> devicesWithTemp = {};
    for (auto device : this->devices) {
      const float temp = device.second->getTemp();
      if (!std::isnan(temp)) {
        if (devicesWithTemp.count(device.first))
          devicesWithTemp[device.first] =
              std::max(devicesWithTemp[device.first], temp);
        else devicesWithTemp[device.first] = temp;
      }
    }
    std::sort(devicesWithTemp.begin(), devicesWithTemp.end(),
              [](auto a, auto b) { return a.second > b.second; });
    for (auto device = devicesWithTemp.begin();
         device != devicesWithTemp.end() && msg.length() < 14;
         device++, msg += DELIM) {
      msg += device->first;
      msg += ":";
      msg += std::to_string(int(floor(device->second)));
    }
  }
  if (msg.length() > 14) msg = msg.substr(0, msg.find_last_of(DELIM));

  if (msg != prevMsg) {
    if (pros::millis() - lastScreenUpdate > MIN_TIME_BETWEEN_UPDATES) {
      Robot::control.print(0, 0, msg.c_str());
      prevMsg = msg.c_str();
      lastScreenUpdate = pros::millis();
    }
    if (disconnected) { Robot::control.rumble("."); }
  }
}