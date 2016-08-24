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

    kTeacher->runArm();
//    getchar();
//    kTeacher->ptp({0.0,0.0,0.0,0.0,0.0,0.,0.0,0.0});
//    cout << "press any key to move " << endl;
//    getchar();
//    kTeacher->ptp({0.0,0.0,0.0,0.0,0.0,0.,0.0,1.57});
//    cout << "press any key to move " << endl;
//    getchar();
//    kTeacher->ptp({0.0,0.0,0.3,0.3,0.0,0.0,1.57,0.0});
//    cout << "press any key to move " << endl;
//    getchar();
//    kTeacher->ptp({0.0,0.0,0.5,0.0,0.0,0.8,0.0,0.8});
//    cout << "press any key to move " << endl;
//    getchar();
//    kTeacher->ptp({0.0,0.0,0.0,0.4,0.0,-0.3,-1.0,-0.7});
//    while(1){
    //        kTeacher->printFilteredSensorVal();
    //        usleep(100000);
    //    }
//    cout << kTeacher->isDiferentialCommandSafe(stdToArmadilloVec({0.0,0.0,0.0,0.0,1.0,0.0,0.0,1.0}),stdToArmadilloVec({0.0,0.0,0.0,0.0,1.0,0.0,0.0,1.0})) << endl;
//    cout << kTeacher->isColliding(stdToArmadilloVec({0.0,0.0,1.8,0.0,0.0,0.0,1.0,0.0})) << endl;
//    cout << kTeacher->isColliding(stdToArmadilloVec({0.0,0.0,1.8,2.8,0.0,0.0,1.0,0.0})) << endl;
//    cout << kTeacher->isColliding(stdToArmadilloVec({0.0,0.0,1.8,-2.8,0.0,1.0,0.0,0.0})) << endl;
//    cout << kTeacher->isColliding(stdToArmadilloVec({0.0,0.0,1.8,2.0,3.0,4.0,0.0,0.0})) << endl;
//    getchar();
//    cout << "press" << endl;
    //    sleep(20);
    //    kTeacher->generateNextPositionCommand();
    getchar();
    kTeacher->stopArm();
    std::cout << "stopped!" << std::endl;



//    ros::Rate myRate(20);

//    while(ros::ok){

//    //pubMove.publish(kTeacher->generateNextPositionCommand());
//    myRate.sleep();

//    }





}
