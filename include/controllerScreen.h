#include "pros/misc.hpp"
#include <vector>

struct ControllerScreenConfig {
    unsigned int lines;
    unsigned int columns;
    unsigned int minTimeBetweenPrints;
    unsigned int minTimeBetweenVibrates;
};

/**
 * @brief Manages prints to the controller screen
 */
class ControllerScreen {
  public:
    void print(unsigned int line, const char* cStr);
    void setText(unsigned int line, unsigned int col, const char* cStr);
    const char* getLine(unsigned int line) const;
    void vibrate(const char* pattern);

    void update();

    static const ControllerScreenConfig DEFAULT_CONFIG;

    ControllerScreen(pros::Controller* con,
                     const ControllerScreenConfig& config = DEFAULT_CONFIG);
  private:
    void printToController(int line);
    void vibrateController();

    const ControllerScreenConfig config;

    std::vector<const char*> currLines;
    std::vector<const char*> queuedLines;

    pros::Controller* con;

    const char* vibrateRequested;

    int lastPrintTime;
    int lastVibrateTime;
    int lastLinePrinted;
};