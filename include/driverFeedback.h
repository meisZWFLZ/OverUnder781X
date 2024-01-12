#pragma once

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <vector>

class Device {
  public:
    std::function<bool(void)> isConnected;
    std::function<float(void)> getTemp;
    Device(pros::Motor* motor);
    Device(pros::Rotation* rot);
    Device(pros::IMU* imu);
    Device(pros::ADILineSensor* lineSensor);
    Device(std::function<bool(void)> isConnected,
           std::function<float(void)> getTemp = hasNoTemp);
  protected:
    static float hasNoTemp();
};

class DriverFeedback {
  public:
    void update();
    DriverFeedback();
  private:
    std::vector<std::pair<const char*, Device*>> devices;
    void initializeDevices();
  
    void addMotorGroup(const char* name, pros::Motor_Group* group);
    void addDevice(const char* name, Device* device);
};