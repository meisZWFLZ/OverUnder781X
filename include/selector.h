#include <string>
#include <vector>

namespace auton {

struct Auton {
    char* label;
    void run();
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
    static void addAutons(Auton* auton[]);
    static void runAuton();

    static void enable();
    static void disable();
    static bool isEnabled();

    static void init();
};

} // namespace auton