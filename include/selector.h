#pragma once
#include <vector>
#include <functional>

namespace auton {

struct Auton {
    char* label;
    std::function<void(void)> run;
};

class AutonSelector {
  private:
    static std::vector<Auton*> autons;
    static int index;
    /** Whether the AutonSelector is enabled */
    static bool state;

    static void incrementListener();
    static void decrementListener();

    static void clearDisplay();
    static void updateDisplay();

    // remove once LVGL is done
    static void attachListeners();
    static void initScreen();

    AutonSelector() = delete;
  public:
    static void addAuton(Auton* auton);
    static void runAuton();

    static void enable();
    static void disable();
    static bool isEnabled();

    static char *getCurrentAuton();

    static void init();
};

} // namespace auton