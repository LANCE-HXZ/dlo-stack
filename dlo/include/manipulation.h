#ifndef MANIPULATION
#define MANIPULATION

#define CLS "220"
#define MDL "160"
#define OPN "0"

#include "kuka_moveit.h"
#include "gripper_control.h"
#include "dlo_global.h"
using namespace cv;

extern Point ptEdge;

void manipulation(SOperation oprt);

void optionI(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc);
void optionD(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc);
void optionQ(SOperation oprt, CKukaMoveit& mkm, CGripperControl& mgc);

#endif