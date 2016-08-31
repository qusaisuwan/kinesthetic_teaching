#include "kinestheticTeacher.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

#define SENSORTOPIC "/wrist"
#define MOVECOMMANDTOPIC "/real/robotino/joint_control/move"
#define MODETOPIC "/real/robotino/settings/switch_mode"

using namespace std;
using namespace arma;
using namespace kukadu;
using namespace UIBK_Teaching;
std::shared_ptr<KinestheticTeacher> kTeacher;

int main(int argc, char** args){

    ros::init(argc, args, "kinesthetic_teaching_node");
    ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(7); spinner.start();


    kTeacher= std::shared_ptr<KinestheticTeacher> (new KinestheticTeacher(node,(char*)SENSORTOPIC));
    kTeacher->init();
    cout << "apply some torques and forces for calibration" << endl;
    getchar();

    kTeacher->startTeaching();
    cout << "press any key to start recording" << endl;
    getchar();
    kTeacher->startRecording();

    cout << "press any key to stop recording" << endl;
    getchar();
    kTeacher->stopRecording();

    cout << "press to play" << endl;
    getchar();
    kTeacher->stopTeaching();
    cout << "playing" << endl;
    kTeacher->play();

    cout << "press to end" << endl;
    getchar();
    std::cout << "stopped!" << std::endl;
    kTeacher->stopArm();




//    ros::Rate myRate(20);

//    while(ros::ok){

//    //pubMove.publish(kTeacher->generateNextPositionCommand());
//    myRate.sleep();

//    }





}
