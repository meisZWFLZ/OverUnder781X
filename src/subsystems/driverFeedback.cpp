#include "driverFeedback.h"
#include "robot.h"

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

unsigned int prevDisconnectedDeviceFlags = 9;

struct ControllerMessageBuilderConfig {
    unsigned int rows = 3;
    unsigned int columns = 14;
    const char* delim = " ";
    char filler = ' ';
};

class ControllerMessageBuilder {
  public:
    ControllerMessageBuilder(const ControllerMessageBuilderConfig& config = {})
      : config(config), msg(config.rows, ""), delim(config.delim) {};

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
      if (full()) return false;
      msg[row] = linePrefix;
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

    bool full() const { return row > config.rows; };
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

  for (int i = 0; i < this->devices.size(); i++) {
    if (!devices[i].second->isConnected()) {
      disconnectedDeviceFlags |= 1 << i;
      disconnectedDevices[devices[i].first]++;
      if (prevDisconnectedDeviceFlags ^ disconnectedDeviceFlags & 1 << i)
        newDisconnect = true;
    }
  }

  ControllerMessageBuilder builder;

  bool disconnected = !disconnectedDevices.empty();

  if (disconnected) {
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
  std::vector<std::pair<const char*, float>> deviceTemps = {};
  for (auto device : deviceTempMap) { deviceTemps.push_back(device); }

  std::vector<std::string> eStopMsgs = {};

  if (Robot::Subsystems::lift->getState() ==
      LiftArmStateMachine::EMERGENCY_STOPPED)
    eStopMsgs.push_back("E:LIFT");
  if (Robot::Subsystems::catapult->getState() ==
      CatapultStateMachine::EMERGENCY_STOPPED)
    eStopMsgs.push_back("E:CATA");

  std::sort(deviceTemps.begin(), deviceTemps.end(),
            [](auto a, auto b) { return a.second > b.second; });

  for (int i = 0; i < deviceTempMap.size() && !builder.full(); i++) {
    std::string tempMsg = deviceTemps[i].first;
    tempMsg += ":";
    tempMsg += std::to_string(int(floor(deviceTemps[i].second)));
    builder.add(tempMsg);

    if (i < eStopMsgs.size()) builder.rightAdd(eStopMsgs[i]);

    builder.newLine();
  }

  const auto msgs = builder.build();
  for (int i = 0; i < msgs.size(); i++) {
    Robot::Subsystems::controller->print(i, msgs[i].c_str());
  }

  if (newDisconnect) {
    Robot::Subsystems::controller->vibrate(".");
    prevDisconnectedDeviceFlags = disconnectedDeviceFlags;
  }
}