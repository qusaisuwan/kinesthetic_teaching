#include "kinestheticTeacher.h"
#include "Eigen/Jacobi"

using namespace std;
using namespace arma;
using namespace kukadu;

namespace UIBK_Teaching{

KinestheticTeacher::KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic){//(ros::NodeHandle &node)
    myNode =  std::shared_ptr<ros::NodeHandle> (&node);
    teacherRunning=false;
    filterRunning=false;
    // Kukadu

    robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", *myNode));
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    mvKin = std::make_shared<MoveItKinematics>(robotinoQueue, node, "robotino", controlledJoints, "arm_link5");
    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(mvKin);

    sensorUpdateSub = myNode->subscribe(sensorTopic, 1, &KinestheticTeacher::sensorUpdate,this);
}


void KinestheticTeacher::init(){


    qThread = robotinoQueue->startQueue();
    if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    }

    moveThread =std::make_shared<std::thread>(&KinestheticTeacher::teachingThreadHandler, this);
    filterThread =std::make_shared<std::thread>(&KinestheticTeacher::filterHandler, this);
}


void KinestheticTeacher::generateNextCommand(){

    auto currentJointState=robotinoQueue->getCurrentJoints().joints;
    auto diff = getNextDifferentialCommand(mvKin->getJacobian(),IK);
    if (isDifferentialCommandSafe(diff,currentJointState))
        robotinoQueue->move(currentJointState +diff); // cout << "next command: " <<  currentJointState + diff << endl;
    else
        cout << "not safe command" << endl;

}

arma::vec KinestheticTeacher::getNextDifferentialCommand(Eigen::MatrixXd jacobian, ControllerType myType){


    std::vector<double> sensorReading= myFilter.getProcessedReading();
    auto numberOfCartesianFTs=jacobian.rows();
    if (numberOfCartesianFTs != sensorReading.size()){
        cout << "Problem in sensor readings vector size" << endl;
        return stdToArmadilloVec({0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    }
    Eigen::VectorXd forceVector = stdToEigenVec(scaleForcesTorques(sensorReading));
    std::vector<double> jacobianMethodDifferential;

    switch(myType){
    case JACOBIAN:
        jacobianMethodDifferential =  eigenToStdVec(jacobian.transpose() * forceVector);

    case IK:
        jacobianMethodDifferential =  eigenToStdVec(jacobian.transpose() * forceVector);
    }
    //Eigen::JacobiSVD<Eigen::MatrixXd> svd(input, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto scaledDiffMovement= scaleJointCommands(jacobianMethodDifferential);

    return stdToArmadilloVec(capVec(scaledDiffMovement,MAXIMUM_JOINT_STEP));

}

std::vector<double> KinestheticTeacher::scaleForcesTorques(std::vector<double> myVec){
        auto torqueIndexStart=myVec.size()/2;
        //weigh force and torque readings
        for (int i=0;i< torqueIndexStart; ++i)
            myVec.at(i) *= FORCES_MOVING_MULTIPLIER;
        for (int i=torqueIndexStart;i< myVec.size(); ++i)
            myVec.at(i) *= TORQUES_MOVING_MULTIPLIER;
        myVec.at(2)*=Z_FORCE_MOVING_MULTIPLIER;
        return myVec;
}

std::vector<double> KinestheticTeacher::scaleJointCommands(std::vector<double> myVec){
    //weigh base and arm movements
    myVec.at(0)*= BASE_XY_MOVING_MULTIPLIER;
    myVec.at(1)*=BASE_XY_MOVING_MULTIPLIER;
    myVec.at(2)*=BASE_Z_MOVING_MULTIPLIER;
    for (auto i=3; i< myVec.size();i++)
        myVec.at(i)*=ARM_ALLJOINTS_MOVING_MULTIPLIER;
    return myVec;
}
Eigen::VectorXd KinestheticTeacher::stdToEigenVec(std::vector<double> myVec){
    int n = myVec.size();
    Eigen::VectorXd temp(n);
    for (int i=0;i< n; ++i)
        temp(i)= myVec.at(i);
    return temp;
}

std::vector<double> KinestheticTeacher::eigenToStdVec(Eigen::VectorXd myVec){
    int n = myVec.rows();
    std::vector<double> temp;
    for (int i=0;i< n; ++i)
        temp.push_back(myVec(i));
    return temp;

}
bool KinestheticTeacher::isDifferentialCommandSafe(arma::vec diffCommand,arma::vec currentJointStates){
    bool okay=true;
    int checkTimesAhead=3;
    for (auto i = 1;i<=checkTimesAhead;i++)
        if (isColliding(armadilloToStdVec(i*diffCommand + currentJointStates)))
            okay=false;
    return okay;
}

bool KinestheticTeacher::isColliding(std::vector<double> jointStates){
    auto pose = mvKin->computeFk(jointStates);
    return mvKin->isColliding(stdToArmadilloVec(jointStates),pose);
}

//void KinestheticTeacher::ptp(std::vector<double> target){

//    auto jointPlan = mvKin->planJointTrajectory({robotinoQueue->getCurrentJoints().joints,stdToArmadilloVec(target)});
//    robotinoQueue->setNextTrajectory(jointPlan);
//    robotinoQueue->synchronizeToQueue(1);
//}


std::vector<double> KinestheticTeacher::capVec(std::vector<double> input, double maxCap){

    for (int i =0;i< input.size();i++)
        input.at(i)= std::isnan(input.at(i)) || std::isinf(input.at(i)) ? 0.0 : sign(input.at(i)) * min(maxCap,sign(input.at(i)) * input.at(i));
    return input;

}

int KinestheticTeacher::sign(double x){
    return (x>=0.0) ? 1: -1;
}



void KinestheticTeacher::runArm(){
    teacherRunning=true;
}

void KinestheticTeacher::teachingThreadHandler(){
    ros::Rate myRate(50);
    while(1){
        if (teacherRunning){
            myFilter.printFilteredSensorVal();
            generateNextCommand();

        }

        myRate.sleep();
    }

}


void KinestheticTeacher::filterHandler(){
    ros::Rate myRate(FILTER_FREQ);
    double timeStamp=0;
    while(1){
        if (filterRunning){
            if (sensorVal.at(6) != timeStamp){ // new reading
                sensorMutex.lock();
                auto temp=sensorVal;
                sensorMutex.unlock();
                temp.pop_back();
                myFilter.filterUpdate(temp,mvKin->computeFk(armadilloToStdVec(robotinoQueue->getCurrentJoints().joints)));
            }


        }
        myRate.sleep();
    }

}

void KinestheticTeacher::sensorUpdate(std_msgs::Float64MultiArray msg){

    sensorMutex.lock();

    if (msg.data.size() ==7) {// 3 forces, 3 torques and timestamp
        sensorVal=msg.data;
        if (firstReading){
            filterRunning=true;
            firstReading=false;
        }
    }
    else
        cout << "wrong vector size from FT sensor" << endl;
    sensorMutex.unlock();
}


void KinestheticTeacher::stopArm(){
    teacherRunning=false;
    robotinoQueue->stopQueue();
    moveThread->join();
    filterThread->join();
    qThread->join();
}
}
