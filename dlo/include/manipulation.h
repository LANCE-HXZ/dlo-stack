#ifndef MANIPULATION
#define MANIPULATION

#include "kuka_moveit.h"
#include "gripper_control.h"
#include "dlo_global.h"
using namespace cv;

void manipulation(SOperation oprt);

void optionI(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc);

#endif