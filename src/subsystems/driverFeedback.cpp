#include "driverFeedback.h"
#include "pros/error.h"
#include "robot.h"
#include <cmath>
#include <functional>

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

Device::Device(std::function<bool(void)> isConnected,
               std::function<float(void)> getTemp)
  : isConnected(isConnected), getTemp(getTemp) {};

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
  // addDevice("IMU", new Device(&Robot::Sensors::imu));
}

unsigned int prevDisconnectedDeviceFlags = 9;

struct ControllerMessageBuilderConfig {
    const int rows = 3;
    const int columns = 14;
    const char* delim = " ";
    const char filler = ' ';
};

class ControllerMessageBuilder {
  public:
    ControllerMessageBuilder(
        const ControllerMessageBuilderConfig& config = {.rows = 3,
                                                        .columns = 14,
                                                        .delim = " ",
                                                        .filler = ' '})
      : config(config), msg(config.rows, ""), delim(config.delim) {
      printf("config rows: %i\n", this->config.rows);
    };

    void setDelim(const char* delim) { this->delim = delim; };

    void setPrefix(const char* pre) { this->setPrefix(std::string(pre)); }

    void setPrefix(std::string pre) { this->linePrefix = pre; }

    void clearPrefix() { this->linePrefix = ""; }

    bool add(const char* str, bool useDelim = true) {
      return this->add(std::string(str), useDelim);
    };

    bool add(std::string str, bool useDelim = true) {
      if (str.length() > config.columns) return false;
      const int currLineDelimLen =
          useDelim && msg[row].length() >
                          (currLineHasCurrPrefix() ? linePrefix.length() : 0)
              ? delim.length()
              : 0;
      if (msg[row].length() + currLineDelimLen + str.length() > config.columns)
        this->newLine();
      if (full()) return false;
      if (useDelim && msg[row].length() > 0) msg[row] += delim;
      msg[row] += str;
      return true;
    };

    bool newLine() {
      row++;
      printf("new lining \n");
      if (full()) return false;
      printf(" row: %i \n", row);
      msg[row] = linePrefix;
      printf(" we made it\n");
      return true;
    }

    bool rightAdd(const char* str) { return this->rightAdd(std::string(str)); }

    bool rightAdd(std::string str, bool requireSomeFiller = true) {
      const int fillerNum = config.columns - (msg[row].length() + str.length());
      if (fillerNum < (requireSomeFiller ? 1 : 0)) return false;

      return this->add(std::string(fillerNum, config.filler) + str);
    }

    std::vector<std::string> build() const { return msg; };

    std::string buildWithNewlines() const {
      std::string result;
      for (auto line : this->build()) { result += line + "\n"; }
      return result;
    };

    bool full() const {
      printf("is full?\n");
      printf(" row: %i\n", row);
      printf(" numOfRows: %i\n", config.rows);
      printf(" columns: %i\n", config.columns);
      printf(" full?: %i\n", row >= config.rows);
      return row >= config.rows;
    };
  private:
    bool currLineHasCurrPrefix() const {
      return msg[row].substr(0, linePrefix.length()) == linePrefix;
    }

    std::vector<std::string> msg;
    std::string delim = " ";
    unsigned int row = 0;
    const ControllerMessageBuilderConfig& config;
    std::string linePrefix;
};

void DriverFeedback::update() {
  std::unordered_map<const char*, int> disconnectedDevices = {};
  unsigned int disconnectedDeviceFlags = 0;
  bool newDisconnect = false;
  printf("aaaa\n");

  for (int i = 0; i < this->devices.size(); i++) {
    if (!devices[i].second->isConnected()) {
      disconnectedDeviceFlags |= 1 << i;
      disconnectedDevices[devices[i].first]++;
      if (prevDisconnectedDeviceFlags ^ disconnectedDeviceFlags & 1 << i)
        newDisconnect = true;
    }
  }
  printf("bbbb\n");

  ControllerMessageBuilder builder;

  bool disconnected = !disconnectedDevices.empty();

  printf("dddd\n");
  if (disconnected) {
    printf("eeee\n");
    builder.setPrefix("!");
    for (auto dev = disconnectedDevices.begin();
         dev != disconnectedDevices.end() && !builder.full(); dev++) {
      std::string disConnMsg = dev->first;
      if (dev->second > 1) disConnMsg += "x" + std::to_string(dev->second);
      builder.add(disConnMsg);
    }
    builder.clearPrefix();
    builder.newLine();
  }

  printf("ffff\n");
  std::unordered_map<const char*, float> deviceTempMap = {};
  for (auto device : this->devices) {
    const float temp = device.second->getTemp();
    if (!std::isnan(temp)) {
      if (deviceTempMap.count(device.first))
        deviceTempMap[device.first] =
            std::max(deviceTempMap[device.first], temp);
      else deviceTempMap[device.first] = temp;
    }
  }
  printf("gggg\n");
  std::vector<std::pair<const char*, float>> deviceTemps = {};
  for (auto device : deviceTempMap) { deviceTemps.push_back(device); }
  printf("hhhh\n");

  std::vector<std::string> eStopMsgs = {};

  printf("iiii\n");
  // if (Robot::Subsystems::lift->getState() ==
  //     LiftArmStateMachine::EMERGENCY_STOPPED)
  //   eStopMsgs.push_back("E:LIFT");
  printf("jjjj\n");
  if (Robot::Subsystems::catapult->getState() ==
      CatapultStateMachine::EMERGENCY_STOPPED)
    eStopMsgs.push_back("E:CATA");

  printf("kkkk\n");
  std::sort(deviceTemps.begin(), deviceTemps.end(),
            [](auto a, auto b) { return a.second > b.second; });

  printf("llll size: %i\n", deviceTempMap.size());
  for (int i = 0; i < deviceTemps.size() && !builder.full(); i++) {
    printf("<build msg>\n%s\n</build msg>\n",
           builder.buildWithNewlines().c_str());
    printf("device name: %s\ndevice temp: %f\n", deviceTemps[i].first,
           deviceTemps[i].second);
    std::string tempMsg = deviceTemps[i].first;
    printf("laaa\n");
    tempMsg += ":";
    printf("lbbb\n");
    tempMsg += std::to_string(int(floor(deviceTemps[i].second)));
    printf("lccc\n");
    builder.add(tempMsg);
    printf("lddd\n");

    if (i < eStopMsgs.size()) builder.rightAdd(eStopMsgs[i]);
    printf("leee\n");

    builder.newLine();
    printf("lFff\n");
  }
  printf("mmmm\n");

  const auto msgs = builder.build();
  for (int i = 0; i < msgs.size(); i++) {
    Robot::Subsystems::controller->print(i, msgs[i].c_str());
  }
  printf("nnnn\n");

  if (newDisconnect) {
    Robot::Subsystems::controller->vibrate(".");
    prevDisconnectedDeviceFlags = disconnectedDeviceFlags;
  }
}