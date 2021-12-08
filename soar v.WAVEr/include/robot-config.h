using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor frontR;
extern motor frontL;
extern motor backR;
extern motor backL;
extern inertial gyros;
extern digital_out lockF;
extern motor convL;
extern digital_out lockB;
extern motor liftL;
extern motor liftR;
extern digital_out lockC;
extern motor convR;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );