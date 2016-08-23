#include "kinestheticTeacher.h"
#include "Eigen/Jacobi"

using namespace std;
using namespace arma;
using namespace kukadu;

KinestheticTeacher::KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic){//(ros::NodeHandle &node)
    myNode =  std::shared_ptr<ros::NodeHandle> (&node);
    teacherRunning=false;
    filtersRunning=false;
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


        auto sensorReading=getProcessedReading();
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

void KinestheticTeacher::printFilteredSensorVal(){

//    filterSmoothingMutex.lock();
//    auto readings = limitsFilterMemory;//getProcessedReading();
//    filterSmoothingMutex.unlock();

    auto readings = getProcessedReading();

    cout << "Force/Torque sensor values are : ";
    for (int i=0;i< readings.size();i++)
        cout << setw(13) << fixed << setprecision(5)  << readings.at(i);
    cout << endl;

}

std::vector<double> KinestheticTeacher::capVec(std::vector<double> input, double maxCap){

      for (int i =0;i< input.size();i++)
          input.at(i)= std::isnan(input.at(i)) || std::isinf(input.at(i)) ? 0.0 : sign(input.at(i)) * min(maxCap,sign(input.at(i)) * input.at(i));
      return input;

}

  int KinestheticTeacher::sign(double x){
     return (x>=0.0) ? 1: -1;
  }



  std::vector<double> KinestheticTeacher::scaleAndLimitReadings(std::vector<double> msg){
      int torquesStartingIndex=limitsFilterMemory.size()/2;
      double forceMin=FORCE_MIN_LIMIT;
      double torqueMin=TORQUE_MIN_LIMIT;

      for (int i=0;i<torquesStartingIndex;i++)
          msg.at(i)= min(1.0,max(-1.0,(sign(msg.at(i)) == 1 ? msg.at(i)/max(forceMin,fabs(limitsFilterMemory.at(i).second)) : msg.at(i)/max(forceMin,fabs(limitsFilterMemory.at(i).first)))));

      for (int i=torquesStartingIndex ;i<limitsFilterMemory.size();i++)
          msg.at(i)= min(1.0,max(-1.0,(sign(msg.at(i)) == 1 ? msg.at(i)/max(torqueMin,fabs(limitsFilterMemory.at(i).second)) : msg.at(i)/max(torqueMin,fabs(limitsFilterMemory.at(i).first)))));

      return msg;
  }

  std::vector<double> KinestheticTeacher::removeBias(std::vector<double> values,std::vector<double> bias){
      std::vector<double> res;
      for(int i=0;i< values.size();i++)
          res.push_back(values.at(i)-bias.at(i));
      return res;

  }
  void KinestheticTeacher::lowpassFilter(std::vector<double> &filter,std::vector<double> newVal, double forgettingFactor){
      for(int i=0;i< filter.size();i++)
          filter.at(i)= filter.at(i) * forgettingFactor + (1-forgettingFactor) * newVal.at(i);
  }

  void KinestheticTeacher::limitsFilter(std::vector<std::pair<double,double>> &filter,std::vector<std::vector<double>> newDataSet, double forgettingFactorShrink, double forgettingFactorExpand, double percentageWithinLimits){
      int vectorsNumber=newDataSet.size();
      for(int i=0;i< vectorsNumber;i++){
          std::vector<double> vectorReadings=newDataSet.at(i);
          std::sort(vectorReadings.begin(),vectorReadings.end());
          int limitIndex = (int)(percentageWithinLimits *vectorReadings.size());
          double lowerLimit=vectorReadings.at(vectorReadings.size()-limitIndex);
          double upperLimit=vectorReadings.at(limitIndex);
           filter.at(i).first = lowerLimit < filter.at(i).first ? (filter.at(i).first * forgettingFactorExpand + lowerLimit * (1-forgettingFactorExpand)) : (filter.at(i).first * forgettingFactorShrink + lowerLimit * (1-forgettingFactorShrink));
           filter.at(i).second = upperLimit > filter.at(i).second ? (filter.at(i).second * forgettingFactorExpand + upperLimit * (1-forgettingFactorExpand)) : (filter.at(i).second * forgettingFactorShrink + upperLimit * (1-forgettingFactorShrink));
//           cout << " lowerLimit " << lowerLimit << "   filter.at(i).first " << filter.at(i).first  << " upperLimit" << upperLimit << "  filter.at(i).second " <<  filter.at(i).second << endl;
      }
  }

  void KinestheticTeacher::runArm(){
    teacherRunning=true;
}

  void KinestheticTeacher::teachingThreadHandler(){
      ros::Rate myRate(50);
      while(1){
          if (teacherRunning){

              //generateNextCommand();
              printFilteredSensorVal();
          }
          myRate.sleep();
      }

}
  void KinestheticTeacher::filterSmoothingThreadHandler(){
      ros::Rate myRate(FILTER_FREQ);
      int cnt=0;
      std::vector<std::vector<double>> newDataSet;
      newDataSet.resize(6);
      while(true){
          if (filtersRunning){
              sensorMutex.lock();
              auto current = sensorVal;
              sensorMutex.unlock();
              filterSmoothingMutex.lock();
              filterDC1Mutex.lock();
              auto valuesAC=removeBias(current,DCFilter1Memory);

              if (cnt < 100){
                  for(auto i=0;i<valuesAC.size();i++)
                      newDataSet.at(i).push_back(valuesAC.at(i));

                  cnt++;
              }else{
                  limitsFilter(limitsFilterMemory,newDataSet,FORGET_SHRINK,FORGET_EXPAND,ACCEPTANCE_INTERVAL);
                  newDataSet.clear();

                  newDataSet.resize(6);
                  cnt=0;
              }
              lowpassFilter(smoothingFilterMemory,(valuesAC),SMOOTHING);//
              filterDC1Mutex.unlock();
              kukaduMutex.lock();
              transformedValueMutex.lock();
              auto jointState = armadilloToStdVec(robotinoQueue->getCurrentJoints().joints);
              projectedFilteredReadings=projectReadings(smoothingFilterMemory,mvKin->computeFk(jointState));
              transformedValueMutex.unlock();
              kukaduMutex.unlock();
              filterSmoothingMutex.unlock();

          }
          myRate.sleep();
      }

}

void KinestheticTeacher::filterDC1ThreadHandler(){
    ros::Rate myRate(FILTER_FREQ);

    while(true){
        if (filtersRunning){
            sensorMutex.lock();
            auto current = sensorVal;
            sensorMutex.unlock();

            filterDC1Mutex.lock();
            lowpassFilter(DCFilter1Memory,current,DC_FILTER1);
            filterDC1Mutex.unlock();

        }
        myRate.sleep();
    }

}
void KinestheticTeacher::filterDC2ThreadHandler(){
    ros::Rate myRate(FILTER_FREQ);

    while(true){
        if (filtersRunning){
            filterSmoothingMutex.lock();
            filterDC2Mutex.lock();
            lowpassFilter(DCFilter2Memory,smoothingFilterMemory,DC_FILTER2);
            filterDC2Mutex.unlock();
            filterSmoothingMutex.unlock();
        }
        myRate.sleep();
    }

}

std::vector<double> KinestheticTeacher::projectReadings(std::vector<double> readings, geometry_msgs::Pose currentPose){ //memory problem?
    tf::Quaternion quat(currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    //cout << "roll " << roll << " pitch " << pitch << " yaw " << yaw << endl;
    vector<double> res1 = projectVectors(readings.at(0),readings.at(1) ,readings.at(2),roll,pitch,yaw);
    vector<double> res2 = projectVectors(readings.at(3),readings.at(4) ,readings.at(5),roll,pitch,yaw);
    res1.insert(res1.end(),res2.begin(),res2.end());

    return res1;
}

vector<double> KinestheticTeacher::projectVectors(double vecX,double vecY,double vecZ,double alpha,double beta,double gamma){
    vector<double> newVec;
    double i= vecX * (cos(beta)*cos(gamma)) + vecY *(cos(gamma)* sin(alpha)* sin(beta) - cos(alpha)*sin(gamma)) + vecZ *(cos(alpha)*cos(gamma)*sin(beta) +sin(alpha)*sin(gamma));
    double j= vecX * (cos(beta)*sin(gamma)) + vecY *(cos(alpha)* cos(gamma) + sin(alpha)*sin(gamma)*sin(beta)) + vecZ *(-1*cos(gamma)*sin(alpha) +cos(alpha)*sin(beta)*sin(gamma));
    double k= vecX * (-1*sin(beta)) + vecY *(cos(beta)* sin(alpha)) + vecZ *(cos(alpha)*cos(beta));
    newVec.push_back(i);
    newVec.push_back(j);
    newVec.push_back(k);
    return newVec;

}
std::vector<double>  KinestheticTeacher::getProcessedReading(){
    transformedValueMutex.lock();
    filterDC2Mutex.lock();
    std::vector<double>  res = scaleAndLimitReadings(removeBias(projectedFilteredReadings,DCFilter2Memory));
    filterDC2Mutex.unlock();
    transformedValueMutex.unlock();
    return res;
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
