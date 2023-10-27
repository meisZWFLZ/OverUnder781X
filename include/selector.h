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

    static void increment();
    static void decrement();

    static void updateDisplay();

    // remove once LVGL is done
    static void attachListeners();
    static void initScreen();

    AutonSelector() = delete;
  public:
    static void addAuton(Auton* auton);
    static void runAuton();

    static void init();
};

} // namespace auton