#ifndef MANIPULATION
#define MANIPULATION

#define CLS "220"       //  夹爪关
#define MDL "160"       //  夹爪半开, 用于限位线缆
#define OPN "0"         //  夹爪开
#define MDLEGE 400    //  左右臂分界线的像素坐标x
#define OPRTZ 0.7

// #include "kuka_moveit.h"
#include "kukafri_hw/servo.h"
#include "gripper_control.h"
#include "dlo_global.h"
using namespace cv;

extern Point ptEdge;

void manipulation(SOperation oprt);

void optionI(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc);
void optionD(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc);
void optionQ(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc);
void optionIX(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc);
void optionT(SOperation oprt, CIiwaServo& msv, CGripperControl& mgc);

#endif