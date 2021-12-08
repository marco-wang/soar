#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor frontR = motor(PORT10, ratio6_1, false);
motor frontL = motor(PORT1, ratio6_1, true);
motor backR = motor(PORT9, ratio6_1, true);
motor backL = motor(PORT2, ratio6_1, false);
inertial gyros = inertial(PORT6);
digital_out lockF = digital_out(Brain.ThreeWirePort.A);
motor convL = motor(PORT7, ratio6_1, true);
digital_out lockB = digital_out(Brain.ThreeWirePort.B);
motor liftL = motor(PORT3, ratio36_1, false);
motor liftR = motor(PORT4, ratio36_1, true);
digital_out lockC = digital_out(Brain.ThreeWirePort.C);
motor convR = motor(PORT8, ratio6_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}