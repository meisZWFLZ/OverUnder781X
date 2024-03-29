#pragma once
#include "pros/rotation.hpp"
#include <vector>
#include <functional>

namespace auton {

struct Auton {
    char* label;
    std::function<void(void)> run;
};

class AutonSelector {
  private:
    std::vector<Auton*> autons;
    /** Whether the AutonSelector is enabled */
    bool state = false;

    const unsigned int gearTeeth;

    void clearDisplay();
    void updateDisplay();

    // remove once LVGL is done
    void initScreen();

    /**
     * @brief Returns the rotation's angle in degrees in the range of [0, 360).
     * Uses rotation.getPosition() to get the angle
     *
     * @return The rotation's angle in degrees in the range of [0, 360)
     */
    float getRotationAngle();

    pros::Rotation rotation;
  protected:
    float getDegreesPerAuton() const;
  public:
    AutonSelector(pros::Rotation rotation, unsigned int gearTeeth);

    unsigned int getIndex();
    void setIndex(unsigned int newIndex);
    void changeIndex(int indexChange);

    /**
     * @return true when auton is added
     * @return false when auton failed to be added due to too many autons
     */
    bool addAuton(Auton* auton);
    void runAuton();

    void enable();
    void disable();
    bool isEnabled();

    char* getCurrentAuton();

    void init();

    /**
     * @brief changes auton based on rotation sensor
     */
    void update();
};

} // namespace auton