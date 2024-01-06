# File Structure
For my program, I organize code into many different files in order to enhance my ability to navigate the code. Heres a tree diagram of the code's organization:
```
.
├── include
│   ├── lemlib
│   │   └── ...
│   ├── pros
│   │   └── ...
│   ├── auton.h
│   ├── catapult.h
│   ├── fieldDimensions.h
│   ├── lift.h
│   ├── main.h
│   ├── robot.h
│   └── selector.h
└── src
    ├── auton
    │   ├── autons
    │   │   ├── defensive.cpp
    │   │   ├── sixBall.cpp
    │   │   └── skills.cpp
    │   ├── selector.cpp
    │   └── util.cpp
    ├── robot-config
    │   ├── dimensions.cpp
    │   ├── motors.cpp
    │   ├── odom.cpp
    │   ├── pistons.cpp
    │   ├── sensors.cpp
    │   └── tunables.cpp
    ├── subsystems
    │   ├── catapult.cpp
    │   ├── initialize.cpp
    │   └── lift.cpp
    └── main.cpp
```
### Whats the difference between .h and .cpp?
- .h's are header files that declare variables, functions, and classes so that they can be used in the files.
- .cpp's implement the declarations made in the headers. These implementations are called definitions

For example:
```h
// inside helloWorld.h

void helloWorld(); // declares what helloWorld() is
```
```cpp
// inside helloWorld.cpp

// tells the compiler to include the symbols from helloWorld.h
#include "helloWorld.h" 

// defines the implementation of helloWorld()
void helloWorld() {
    printf("Hello World \n");
}
```
```cpp
// inside main.cpp

// tells the compiler to include the symbols from helloWorld.h
#include "helloWorld.h" 

// the function that will be run at the start of the program
int main() {
    // runs the helloWorld() function
    helloWorld();
}
```

### Include
This folder contains a several headers, which are files that declare symbols (variables, functions, classes, etc.) so that they can be used in other files.

- LemLib - offers tools for position tracking and motion algorithms.
- PROS - declare how the user program can interact with the v5 devices
- auton.h - the autonomous functions and the utility functions used by autons.
- catapult.h - the catapult subsystem class and the methods for controlling the catapult
- fieldDimensions.h - constants describing the dimensions of the field which is useful for autons. 
- lift.h - the lift subsystem class and the methods for controlling the lift
- main.h - declares competition event functions (opcontrol, autonomous, initialize)
- robot.h - declares all of the robot's devices, subsystems, dimensions, and tunables
- selector.h - the autonomous selector class

### SRC
Short for source. Contains several source files which define the symbols declared in the headers.
- auton - contains everything related to autonomous
  - autons - contains all the autons
    - defensive.cpp - auton that runs on the side of the field closest to our alliance station
    - sixBall.cpp - auton that runs on the side of the field furthest to our alliance station. Scores 6 triballs into the goal
    - skills.cpp - programming skills routine
  - selector.cpp - the auton selector class's methods
  - util.cpp - utility functions for autons that reduce the verbosity of the autonomous functions.
- robot-config
  - dimensions.cpp - the dimensions of the robot, including the wheel diameters, gear ratios, and tracking wheel positions
  - motors.cpp - configures the robot's motors' ports, gear cartridges, and directions 
  - odom.cpp - configures position tracking's tracking wheels and defines the lemlib chassis object
  - pistons.cpp - configures the robot's pistons' ports 
  - sensors.cpp - configures the robot's sensors' ports and directions
  - tunables.cpp - controller settings and chasePower for the drive and PID constants for the lift
- subsystems
  - catapult.cpp - catapult subsystem's state machine
  - initialize.cpp - starts subsystem update task and initializes the two subsystems.
  - lift.cpp - lift subsystem's pid controller and state machine
- main.cpp - driver control, brain screen printing, program initialize, and runs the autonomous function 