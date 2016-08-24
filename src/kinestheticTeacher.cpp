#include "kinestheticTeacher.h"
#include "Eigen/Jacobi"

using namespace std;
using namespace arma;
using namespace kukadu;

namespace UIBK_Teaching{

KinestheticTeacher::KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic){//(ros::NodeHandle &node)
    myNode =  std::shared_ptr<ros::NodeHandle> (&node);
    teacherRunning=false;
    filtersRunning=false;
    firstReading=true;
    projectedFilteredReadings=DCFilter1Memory=DCFilter2Memory=smoothingFilterMemory={0.0,0.0,0.0,0.0,0.0,0.0};
    limitsFilterMemory = {std::pair<double,double>(-1.0 * FORCE_MIN_LIMIT,1.0 * FORCE_MIN_LIMIT),
                          std::pair<double,double>(-1.0 * FORCE_MIN_LIMIT,1.0 * FORCE_MIN_LIMIT),
                          std::pair<double,double>(-1.0 * FORCE_MIN_LIMIT,1.0 * FORCE_MIN_LIMIT),
                          std::pair<double,double>(-1.0 * TORQUE_MIN_LIMIT,1.0 * TORQUE_MIN_LIMIT),
                          std::pair<double,double>(-1.0 * TORQUE_MIN_LIMIT,1.0 * TORQUE_MIN_LIMIT),
                          std::pair<double,double>(-1.0 * TORQUE_MIN_LIMIT,1.0 * TORQUE_MIN_LIMIT)};
    //maxFilterMemory={0.0
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
    filterSmoothingThread =std::make_shared<std::thread>(&KinestheticTeacher::filterSmoothingThreadHandler, this);
    filterDC1Thread =std::make_shared<std::thread>(&KinestheticTeacher::filterDC1ThreadHandler, this);
    filterDC2Thread =std::make_shared<std::thread>(&KinestheticTeacher::filterDC2ThreadHandler, this);
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


    std::vector<double> sensorReading=getProcessedReading();//{0.0,0.0,-0.2,0.0,0.0,0.0};
    auto numberOfJoints=jacobian.cols();
    auto numberOfCartesianFTs=jacobian.rows();
    if (numberOfCartesianFTs != sensorReading.size()){
        cout << "Problem in sensor readings vector size" << endl;
        return stdToArmadilloVec({0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    }

    Eigen::VectorXd forceVector(numberOfCartesianFTs);
    auto torqueIndexStart=numberOfCartesianFTs/2;
    //weigh force and torque readings
    for (int i=0;i< torqueIndexStart; ++i)
        forceVector(i)= sensorReading.at(i)*FORCES_MOVING_MULTIPLIER;
    for (int i=torqueIndexStart;i< numberOfCartesianFTs; ++i)
        forceVector(i)= sensorReading.at(i)*TORQUES_MOVING_MULTIPLIER;
    forceVector(2)*=Z_FORCE_MOVING_MULTIPLIER;
    Eigen::MatrixXd jacobianMethodDifferential =  jacobian.transpose() * forceVector;



    //weigh base and arm movements
    std::vector<double> scaledDiffMovement;
    scaledDiffMovement.push_back(jacobianMethodDifferential(0)*BASE_XY_MOVING_MULTIPLIER);
    scaledDiffMovement.push_back(jacobianMethodDifferential(1)*BASE_XY_MOVING_MULTIPLIER);
    scaledDiffMovement.push_back(jacobianMethodDifferential(2)*BASE_Z_MOVING_MULTIPLIER);
    for (auto i=3; i<numberOfJoints;i++)
        scaledDiffMovement.push_back(jacobianMethodDifferential(i)*ARM_ALLJOINTS_MOVING_MULTIPLIER);
    //Eigen::JacobiSVD<Eigen::MatrixXd> svd(input, Eigen::ComputeThinU | Eigen::ComputeThinV);


    return stdToArmadilloVec(capVec(scaledDiffMovement,MAXIMUM_JOINT_STEP));

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

void KinestheticTeacher::ptp(std::vector<double> target){

    auto jointPlan = mvKin->planJointTrajectory({robotinoQueue->getCurrentJoints().joints,stdToArmadilloVec(target)});
    robotinoQueue->setNextTrajectory(jointPlan);
    robotinoQueue->synchronizeToQueue(1);
}


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

            generateNextCommand();
            printFilteredSensorVal();
        }
        myRate.sleep();
    }

}



void KinestheticTeacher::sensorUpdate(std_msgs::Float64MultiArray msg){

    sensorMutex.lock();

    if (msg.data.size() ==7) //3 forces, 3 torques and timestamp
    {
        sensorVal=msg.data;
        sensorVal.erase(sensorVal.end()-1); //erase timestamp
        if (firstReading){
            DCFilter1Memory=sensorVal;
            firstReading=false;
            filtersRunning=true;
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
    filterSmoothingThread->join();
    filterDC1Thread->join();
    filterDC2Thread->join();
    qThread->join();
}
}
