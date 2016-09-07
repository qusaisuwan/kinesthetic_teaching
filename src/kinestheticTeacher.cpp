#include "kinestheticTeacher.h"
#include "Eigen/Jacobi"

using namespace std;
using namespace arma;
using namespace kukadu;

namespace UIBK_Teaching{

KinestheticTeacherUIBK::KinestheticTeacherUIBK(ros::NodeHandle &node,char *sensorTopic){//(ros::NodeHandle &node)
    myNode =  std::shared_ptr<ros::NodeHandle> (&node);
    teacherRunning=false;
    filterRunning=false;
    // Kukadu

    recordingPath="/home/qusai/catkin_ws/src/kinesthetic_teaching/src/rrt";
    robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", *myNode));
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    mvKin = std::make_shared<MoveItKinematics>(robotinoQueue, node, "robotino", controlledJoints, "arm_link5");
    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(mvKin);
    store = std::shared_ptr<kukadu::SensorStorage>(new kukadu::SensorStorage({robotinoQueue}, std::vector<KUKADU_SHARED_PTR<GenericHand> >(), 400));


    sensorUpdateSub = myNode->subscribe(sensorTopic, 1, &KinestheticTeacherUIBK::sensorUpdate,this);
}


void KinestheticTeacherUIBK::init(){


    qThread = robotinoQueue->startQueue();
    if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    }

    moveThread =std::make_shared<std::thread>(&KinestheticTeacherUIBK::teachingThreadHandler, this);
    filterThread =std::make_shared<std::thread>(&KinestheticTeacherUIBK::filterHandler, this);
}


void KinestheticTeacherUIBK::generateNextCommand(){



    auto currentJointState=robotinoQueue->getCurrentJoints().joints;
    auto diff = getNextDifferentialCommand(mvKin->getJacobian(),currentJointState,JACOBIAN);
    if (isDifferentialCommandSafe(diff,currentJointState))
        robotinoQueue->move(currentJointState +diff); // cout << "next command: " <<  currentJointState + diff << endl;
    else
        cout << "not safe command" << endl;

}

void KinestheticTeacherUIBK::startRecording(){

    store->setExportMode(SensorStorage::STORE_TIME | SensorStorage::STORE_RBT_CART_POS | SensorStorage::STORE_RBT_JNT_POS);
    deleteDirectory(recordingPath);
    recordingThread= store->startDataStorage(recordingPath);

}

void KinestheticTeacherUIBK::stopRecording(){

    store->stopDataStorage();
}

void KinestheticTeacherUIBK::play(){

    std::shared_ptr<SensorData> res= kukadu::SensorStorage::readStorage(robotinoQueue,recordingPath + "/kuka_lwr_real_robotino_0");
    std::vector<arma::vec> jointPlan;

    for (int i =0;i<res->getTimes().size();i++){
        auto temp=armadilloToStdVec(res->getJointPosRow(i));
        jointPlan.push_back(stdToArmadilloVec(temp));

    }
     ptp(armadilloToStdVec(jointPlan.front()));
//   cout << jointPlan.front()<< endl;

    auto myPlan=mvKin->planJointTrajectory(jointPlan);
//    for (auto & curr: myPlan)
//        cout << curr << endl;

     robotinoQueue->setNextTrajectory(jointPlan);
     robotinoQueue->synchronizeToQueue(1);
}
arma::vec KinestheticTeacherUIBK::getNextDifferentialCommand(Eigen::MatrixXd jacobian,arma::vec currentJointState, ControllerType myType){


    std::vector<double> sensorReading= myFilter.getProcessedReading();
    auto numberOfCartesianFTs=jacobian.rows();
    if (numberOfCartesianFTs != sensorReading.size()){
        cout << "Problem in sensor readings vector size" << endl;
        return stdToArmadilloVec({0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0});
    }
    std::vector<double> forceVector = scaleForcesTorques(sensorReading);
    std::vector<double> additiveDifferential;

    switch(myType){
    case JACOBIAN:
    {

        additiveDifferential =  eigenToStdVec(jacobian.transpose() * stdToEigenVec(forceVector));
        additiveDifferential= scaleJointCommands(additiveDifferential);
        break;
    }
    case INVERSE://Pseudo inverse..horrible!
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        additiveDifferential= scaleJointCommands(eigenToStdVec(0.15*svd.solve(stdToEigenVec(forceVector))));
        break;
    }
    case IK:
    {
        const double scaleIK=0.2;
        forceVector = armadilloToStdVec( scaleIK*stdToArmadilloVec(forceVector));
        geometry_msgs::Pose currentPose = mvKin->computeFk(armadilloToStdVec(currentJointState));
        geometry_msgs::Pose newPose;
        newPose.position.x= currentPose.position.x + forceVector.at(0);
        newPose.position.y=currentPose.position.y + forceVector.at(1);
        newPose.position.z=currentPose.position.z + forceVector.at(2);
        tf::Quaternion quat(currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w);
        tf::Matrix3x3 m(quat);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);
        roll+= forceVector.at(3);
        pitch+= forceVector.at(4);
        yaw+= forceVector.at(5);
        newPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
        auto planIK = mvKin->computeIk(armadilloToStdVec(currentJointState),newPose);
        auto target =currentJointState;
        if (planIK.size()>0)
            target=planIK.back();
        else
            cout << "IK solution not found" << endl;

        additiveDifferential = armadilloToStdVec( target - currentJointState);
        if (isBigJump(additiveDifferential)){
            additiveDifferential={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            cout << " Big Jump from IK solution" << endl;
        }
        break;
    }
    case HYBRID:
    {
        const double scaleIK=0.1;
        additiveDifferential={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        forceVector = armadilloToStdVec( scaleIK*stdToArmadilloVec(forceVector));
        geometry_msgs::Pose currentPose = mvKin->computeFk(armadilloToStdVec(currentJointState));
        geometry_msgs::Pose newPose;
        newPose.position.x= currentPose.position.x + forceVector.at(0);
        newPose.position.y=currentPose.position.y + forceVector.at(1);
        newPose.position.z=currentPose.position.z + forceVector.at(2);
        tf::Quaternion quat(currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w);
        tf::Matrix3x3 m(quat);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);
        roll+= forceVector.at(3);
        pitch+= forceVector.at(4);
        yaw+= forceVector.at(5);
        newPose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
        auto planIK = mvKin->computeIk(armadilloToStdVec(currentJointState),newPose);
        auto target =currentJointState;
        bool jacobianUse = false;
        if (planIK.size()>0){
            target=planIK.back();
            additiveDifferential = armadilloToStdVec( target - currentJointState);
        }
        else
            jacobianUse =true;

        if (isBigJump(additiveDifferential))
            jacobianUse =true;


        if (jacobianUse){
            additiveDifferential =  eigenToStdVec(jacobian.transpose() * stdToEigenVec(forceVector));
            additiveDifferential= scaleJointCommands(additiveDifferential);
            cout << " jacobian" << endl;
        }
        break;
    }
    }



    return stdToArmadilloVec(capVec(additiveDifferential,MAXIMUM_JOINT_STEP));

}

bool KinestheticTeacherUIBK::isBigJump(std::vector<double> myVec){
    bool bigJump =false;
    for (auto element: myVec)
        if (element > MAX_JOINT_MOVEMENT_ALLOWED_FOR_IK)
            bigJump =true;
    return bigJump;
}

std::vector<double> KinestheticTeacherUIBK::scaleForcesTorques(std::vector<double> myVec){
    auto torqueIndexStart=myVec.size()/2;
    //weigh force and torque readings
    for (int i=0;i< torqueIndexStart; ++i)
        myVec.at(i) *= FORCES_MOVING_MULTIPLIER;
    for (int i=torqueIndexStart;i< myVec.size(); ++i)
        myVec.at(i) *= TORQUES_MOVING_MULTIPLIER;
    return myVec;
}

std::vector<double> KinestheticTeacherUIBK::scaleJointCommands(std::vector<double> myVec){
    //weigh base and arm movements
    myVec.at(0)*= BASE_XY_MOVING_MULTIPLIER;
    myVec.at(1)*=BASE_XY_MOVING_MULTIPLIER;
    myVec.at(2)*=BASE_Z_MOVING_MULTIPLIER;
    for (auto i=3; i< myVec.size();i++)
        myVec.at(i)*=ARM_ALLJOINTS_MOVING_MULTIPLIER;
    return myVec;
}
Eigen::VectorXd KinestheticTeacherUIBK::stdToEigenVec(std::vector<double> myVec){
    int n = myVec.size();
    Eigen::VectorXd temp(n);
    for (int i=0;i< n; ++i)
        temp(i)= myVec.at(i);
    return temp;
}

std::vector<double> KinestheticTeacherUIBK::eigenToStdVec(Eigen::VectorXd myVec){
    int n = myVec.rows();
    std::vector<double> temp;
    for (int i=0;i< n; ++i)
        temp.push_back(myVec(i));
    return temp;

}
bool KinestheticTeacherUIBK::isDifferentialCommandSafe(arma::vec diffCommand,arma::vec currentJointStates){
    bool okay=true;
    int checkTimesAhead=3;
    for (auto i = 1;i<=checkTimesAhead;i++)
        if (isColliding(armadilloToStdVec(i*diffCommand + currentJointStates)))
            okay=false;
    return okay;
}

bool KinestheticTeacherUIBK::isColliding(std::vector<double> jointStates){
    auto pose = mvKin->computeFk(jointStates);
    return mvKin->isColliding(stdToArmadilloVec(jointStates),pose);
}

void KinestheticTeacherUIBK::ptp(std::vector<double> target){

    auto jointPlan = mvKin->planJointTrajectory({robotinoQueue->getCurrentJoints().joints,stdToArmadilloVec(target)});
    robotinoQueue->setNextTrajectory(jointPlan);
    robotinoQueue->synchronizeToQueue(1);
}


std::vector<double> KinestheticTeacherUIBK::capVec(std::vector<double> input, double maxCap){

    for (int i =0;i< input.size();i++)
        input.at(i)= std::isnan(input.at(i)) || std::isinf(input.at(i)) ? 0.0 : sign(input.at(i)) * min(maxCap,sign(input.at(i)) * input.at(i));
    return input;

}

int KinestheticTeacherUIBK::sign(double x){
    return (x>=0.0) ? 1: -1;
}



void KinestheticTeacherUIBK::startTeaching(){
    teacherRunning=true;
}

void KinestheticTeacherUIBK::stopTeaching(){
    teacherRunning=false;
}

void KinestheticTeacherUIBK::teachingThreadHandler(){
    ros::Rate myRate(50);
    while(1){
        if (teacherRunning){
            //myFilter.printFilteredSensorVal();
            generateNextCommand();

        }

        myRate.sleep();
    }

}


void KinestheticTeacherUIBK::filterHandler(){
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

void KinestheticTeacherUIBK::sensorUpdate(std_msgs::Float64MultiArray msg){

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


void KinestheticTeacherUIBK::stopArm(){
    teacherRunning=false;
    robotinoQueue->stopQueue();
    moveThread->join();
    filterThread->join();
    qThread->join();
}
}
