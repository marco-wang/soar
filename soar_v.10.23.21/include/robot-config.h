using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor frontR;
extern motor frontL;
extern motor backL;
extern motor backR;
extern motor liftB;
extern motor liftF;
extern inertial gyros;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );