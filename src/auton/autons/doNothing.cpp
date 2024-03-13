#include "auton.h"

void runNothing() {}

auton::Auton auton::autons::doNothing = {(char*)("do nothing"), runNothing};