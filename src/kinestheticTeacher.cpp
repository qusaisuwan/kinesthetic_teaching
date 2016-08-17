#include "kinestheticTeacher.h"
#include "Eigen/Jacobi"

using namespace std;
using namespace arma;
using namespace kukadu;

KinestheticTeacher::KinestheticTeacher(ros::NodeHandle &node,char *sensorTopic){//(ros::NodeHandle &node)
    myNode =  std::shared_ptr<ros::NodeHandle> (&node);
    teacherRunning=false;
    filtersRunning=false;
    firstReading=true;
    projectedFilteredReadings=DCFilter1Memory=DCFilter2Memory=smoothingFilterMemory={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
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


void KinestheticTeacher::generateNextPositionCommand(){
    sensorMutex.lock();
    auto msg = sensorVal;
    sensorMutex.unlock();

//    auto pose = mvKin->computeFk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//    cout << "pose in 0 joint position: " << pose.position.x << " " << pose.position.y << " " << pose.position.z << endl;
    kukaduMutex.lock();
//    auto jacobian = mvKin->getJacobian();

//    Eigen::MatrixXd jacobianTranspose= jacobian.transpose();
//   // auto jacobianInverse= jacobian.inverse();
//   // cout << "jacobian is " <<  endl << jacobian << endl;
////    cout << jacobianInverse << endl;
////    cout << jacobianTranspose << endl;
//    Eigen::VectorXd forceVector(msg.size());

//    for (int i=0;i< msg.size(); ++i)
//        forceVector(i)= msg[i];
//        //cout << "force vector is is " << forceVector << endl;
////    cout << forceVector<< endl;
////    cout << jacobianTranspose << endl;

//    double scaling=0.04;
//    Eigen::MatrixXd multiplierMatrix=capMatrix(jacobianTranspose, 2.0,robotinoQueue->getJointNames().size()+1,6);
//    Eigen::MatrixXd jointSpaceDifferential = scaling * multiplierMatrix * forceVector;
//    cout << "differential is " << endl << jointSpaceDifferential << endl;
//    //cout << "res " << res << endl;
//    // jointSpaceDifferential = jacobianTranspose * forceVector;

//    auto jointNextState=  armadilloToStdVec(robotinoQueue->getCurrentJoints().joints);

//    for (int i=0;i< jointNextState.size(); ++i){

//        jointNextState.at(i) = jointNextState.at(i) + jointSpaceDifferential(i);
//    }

    vec target ;//= (stdToArmadilloVec(jointNextState));
    target = stdToArmadilloVec({0.0,0.0,0.0,0.0,0.0,0.0,0.3,0.3});
    //auto endJacobian = mvKin->getJacobian(armadilloToStdVec(end));
   // cout << "target is " << target << endl;
     auto jointPlan = mvKin->planJointTrajectory({target});

//    cout << "planning in joint space" << endl;
//    auto jointPlan = mvKin->planJointTrajectory({start, end});
//    cout << "done with planning" << endl;

      // vector<vec> dcJointPlan= fixJointPlanTemp(jointPlan);

//        cout << "from: " << dcJointPlan.front().t() << "to: " << dcJointPlan.back().t() << endl;
//       cout << "joint planning worked (path of length " << dcJointPlan.size() << " generated) (press key to execute it)" << endl;
       robotinoQueue->setNextTrajectory(jointPlan);
       robotinoQueue->synchronizeToQueue(1);
   // cout  << target << endl;

     //  robotinoQueue->move(target);
    kukaduMutex.unlock();
    usleep(100000);

    /* for moving with the queue!!!! */
//    vector<double> nextJointPositions = {0.0,0.0,0.0,0.0,0.0,0.0};\
 
 //JointPositions << endl\

    //robotinoQueue->submitNextJointMove(stdToArmadilloVec(nextJointPositions));


}


void KinestheticTeacher::printFilteredSensorVal(){


    auto readings = getProcessedReading();

    cout << "Force/Torque sensor values are : ";
    for (int i=0;i< readings.size()-1;i++)
        cout << setw(10) << fixed << setprecision(5)  << readings.at(i);
    cout << endl;

}

Eigen::MatrixXd KinestheticTeacher::capMatrix(Eigen::MatrixXd input, double maxCap,int x, int y){//cap 6*jointsNumber matrix

      for (int i =0;i< x*y;i++)
          input(i)= std::isnan(input(i)) || std::isinf(input(i)) ? 0.0 : sign(input(i)) * min(maxCap,sign(input(i)) * input(i));
      return input;
      //Eigen::JacobiSVD<Eigen::MatrixXd> svd(input, Eigen::ComputeThinU | Eigen::ComputeThinV);

}

  int KinestheticTeacher::sign(double x){

     return (x>=0.0) ? 1: -1;

  }



  std::vector<double> KinestheticTeacher::scaleReadings(std::vector<double> msg){

      msg.at(0) *= SCALEXFORCE;
      msg.at(1) *= SCALEYFORCE;
      msg.at(2) *= SCALEZFORCE;
      msg.at(3) *= SCALEXTORQUE;
      msg.at(4) *= SCALEYTORQUE;
      msg.at(5) *= SCALEZTORQUE;
      return msg;
  }

  std::vector<double> KinestheticTeacher::removeBias(std::vector<double> values,std::vector<double> bias){
      std::vector<double> res;
      int limit =values.size()-1;//Don't avg the time stamp
      for(int i=0;i< limit;i++)
          res.push_back(values.at(i)-bias.at(i));
      res.push_back(values.at(limit));
      return res;

  }
  void KinestheticTeacher::lowpassFilter(std::vector<double> &filter,std::vector<double> newVal, double forgettingFactor){
      if (filter.size() == 0)//Filter initialization
          filter=newVal;
      int limit =filter.size()-1;//Don't avg the time stamp
      for(int i=0;i< limit;i++)
          filter.at(i)= filter.at(i) * forgettingFactor + (1-forgettingFactor) * newVal.at(i);
        filter.at(limit)=newVal.at(limit);
  }

  void KinestheticTeacher::runArm(){
    teacherRunning=true;
}

  void KinestheticTeacher::teachingThreadHandler(){
      ros::Rate myRate(50);
      while(1){
          if (teacherRunning){

              printFilteredSensorVal();
          }
          myRate.sleep();
      }

}
  void KinestheticTeacher::filterSmoothingThreadHandler(){
      ros::Rate myRate(FILTER_FREQ);

      while(true){
          if (filtersRunning){

              filterSmoothingMutex.lock();
              filterDC1Mutex.lock();
              lowpassFilter(smoothingFilterMemory,removeBias(sensorVal,DCFilter1Memory),SMOOTHING);
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
            filterDC1Mutex.lock();
            lowpassFilter(DCFilter1Memory,sensorVal,DC_FILTER);
            filterDC1Mutex.unlock();
            sensorMutex.unlock();
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
            lowpassFilter(DCFilter2Memory,smoothingFilterMemory,DC_FILTER);
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
    std::vector<double>  res = removeBias(projectedFilteredReadings,DCFilter2Memory);
    filterDC2Mutex.unlock();
    transformedValueMutex.unlock();
    return res;
}
void KinestheticTeacher::sensorUpdate(std_msgs::Float64MultiArray msg){

    sensorMutex.lock();

    if (msg.data.size() ==7) //3 forces, 3 torques and timestamp
    {
        sensorVal=msg.data;//scaleReadings(msg.data);
        if (firstReading){
            DCFilter1Memory=DCFilter2Memory=smoothingFilterMemory=sensorVal;
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
